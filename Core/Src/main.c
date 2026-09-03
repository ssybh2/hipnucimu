/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/*
 * Three ID slots are enough for six IMUs because CAN1 and CAN2 are independent
 * buses and may reuse the same identifiers.
 *
 * SLOT 1: 0x01 / 0x02 / 0x03
 * SLOT 2: 0x04 / 0x05 / 0x06
 * SLOT 3: 0x07 / 0x08 / 0x09
 *
 * The CMake build creates one firmware image for each slot. If no build-time
 * slot is specified, SLOT 1 is used for backwards compatibility.
 */
#ifndef HIPNUC_IMU_SLOT
#define HIPNUC_IMU_SLOT 1
#endif

#if HIPNUC_IMU_SLOT == 1
#define PACKET1_CAN_ID 0x01U
#define PACKET2_CAN_ID 0x02U
#define PACKET3_CAN_ID 0x03U
#elif HIPNUC_IMU_SLOT == 2
#define PACKET1_CAN_ID 0x04U
#define PACKET2_CAN_ID 0x05U
#define PACKET3_CAN_ID 0x06U
#elif HIPNUC_IMU_SLOT == 3
#define PACKET1_CAN_ID 0x07U
#define PACKET2_CAN_ID 0x08U
#define PACKET3_CAN_ID 0x09U
#else
#error "HIPNUC_IMU_SLOT must be 1, 2 or 3"
#endif

/* Effective forwarding period: 3 ms = 333.33 Hz. */
#define PACKET_PERIOD_MS 3U

/* HiPNUC binary frame outer format: SOF(2) + LEN(2) + CRC(2) + payload. */
#define HIPNUC_SOF0 0x5AU
#define HIPNUC_SOF1 0xA5U
#define HIPNUC_HI92_TAG 0x92U
#define HIPNUC_FRAME_OVERHEAD 6U
#define IMU_RX_BUFFER_SIZE 82U

/* One logical IMU sample is always forwarded as exactly three CAN frames. */
#define IMU_CAN_FRAMES_PER_SAMPLE 3U

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t buf[IMU_RX_BUFFER_SIZE];
uint32_t last_send_tick = 0;
struct packet_0x01_t packet0X01;
struct packet_0x02_t packet0X02;
struct packet_0x03_t packet0X03;
FDCAN_TxHeaderTypeDef shared_tx_header;
uint8_t shared_tx_data[8];
FDCAN_ProtocolStatusTypeDef status;

/* CAN diagnostics: inspect these in CubeIDE during stress tests. */
volatile uint32_t can_tx_ok_count = 0;
volatile uint32_t can_tx_fail_count = 0;
volatile uint32_t can_tx_group_ok_count = 0;
volatile uint32_t can_tx_group_deferred_count = 0;
volatile uint32_t can_tx_group_partial_fail_count = 0;
volatile uint32_t can_busoff_recovery_count = 0;

/* UART / protocol diagnostics. */
volatile uint32_t uart_rx_event_count = 0;
volatile uint32_t uart_frame_count = 0;
volatile uint32_t uart_short_frame_count = 0;
volatile uint32_t uart_header_error_count = 0;
volatile uint32_t uart_length_error_count = 0;
volatile uint32_t uart_crc_error_count = 0;
volatile uint32_t uart_tag_error_count = 0;
volatile uint32_t uart_half_transfer_event_count = 0;
volatile uint32_t uart_dma_restart_error_count = 0;
volatile uint32_t uart_rate_limited_count = 0;

static int16_t read_i16_le(const uint8_t *p) {
    const uint16_t value = (uint16_t)p[0] | ((uint16_t)p[1] << 8U);
    return (int16_t)value;
}

/* CRC-16/XMODEM: polynomial 0x1021, init 0x0000, no reflection/xorout. */
static uint16_t crc16_xmodem_update(uint16_t crc, const uint8_t *src, uint16_t length) {
    for (uint16_t j = 0; j < length; ++j) {
        crc ^= (uint16_t)src[j] << 8U;
        for (uint8_t i = 0; i < 8U; ++i) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }
    return crc;
}

static uint8_t validate_hi92_frame(const uint16_t size) {
    if (size < (HIPNUC_FRAME_OVERHEAD + 1U) || size > IMU_RX_BUFFER_SIZE) {
        uart_short_frame_count++;
        return 0U;
    }

    if (buf[0] != HIPNUC_SOF0 || buf[1] != HIPNUC_SOF1) {
        uart_header_error_count++;
        return 0U;
    }

    const uint16_t payload_len = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8U);
    if ((uint32_t)payload_len + HIPNUC_FRAME_OVERHEAD != size) {
        uart_length_error_count++;
        return 0U;
    }

    if (payload_len == 0U || payload_len > (IMU_RX_BUFFER_SIZE - HIPNUC_FRAME_OVERHEAD)) {
        uart_length_error_count++;
        return 0U;
    }

    const uint16_t received_crc = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8U);
    uint16_t calculated_crc = 0U;

    /* CRC covers SOF + LEN and payload, but excludes the two CRC bytes. */
    calculated_crc = crc16_xmodem_update(calculated_crc, buf, 4U);
    calculated_crc = crc16_xmodem_update(calculated_crc, buf + HIPNUC_FRAME_OVERHEAD, payload_len);

    if (calculated_crc != received_crc) {
        uart_crc_error_count++;
        return 0U;
    }

    /* This firmware uses the fixed offsets of the legacy HI92 payload. */
    if (buf[HIPNUC_FRAME_OVERHEAD] != HIPNUC_HI92_TAG) {
        uart_tag_error_count++;
        return 0U;
    }

    return 1U;
}

static void start_imu_rx_dma(void) {
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, buf, IMU_RX_BUFFER_SIZE) != HAL_OK) {
        uart_dma_restart_error_count++;
        return;
    }

    /* ReceiveToIdle DMA normally also generates a Half-Transfer callback at
     * 41 bytes. That is not a complete 82-byte HI92 frame and must never be
     * parsed or used to restart the DMA reception. */
    if (huart2.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
}

void can_filter_init(void) {
    FDCAN_FilterTypeDef can_filter_st;
    can_filter_st.IdType = FDCAN_STANDARD_ID;
    can_filter_st.FilterIndex = 0;
    can_filter_st.FilterType = FDCAN_FILTER_MASK;
    can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    can_filter_st.FilterID1 = 0x00000000;
    can_filter_st.FilterID2 = 0x00000000;

    HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_st);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);
}

static void recover_can_if_bus_off(void) {
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &status) != HAL_OK) {
        return;
    }

    if (!status.BusOff) {
        return;
    }

    can_busoff_recovery_count++;
    HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    HAL_FDCAN_DeInit(&hfdcan1);
    MX_FDCAN1_Init();
    can_filter_init();
}

static uint8_t send(uint32_t packet_id, uint32_t data_length) {
    shared_tx_header.IdType = FDCAN_STANDARD_ID;
    shared_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    shared_tx_header.DataLength = data_length;
    shared_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    shared_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    shared_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    shared_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    shared_tx_header.MessageMarker = 0;
    shared_tx_header.Identifier = packet_id;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &shared_tx_header, shared_tx_data) == HAL_OK) {
        can_tx_ok_count++;
        return 1U;
    }

    can_tx_fail_count++;
    return 0U;
}

static uint8_t enqueue_complete_can_sample(void) {
    /* The STM32G431 FDCAN implementation has exactly three Tx FIFO/Queue
     * elements, and one IMU sample requires exactly three CAN frames.
     * Never start a sample unless all three slots are available. */
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < IMU_CAN_FRAMES_PER_SAMPLE) {
        can_tx_group_deferred_count++;
        return 0U;
    }

    memcpy(shared_tx_data, &packet0X01, 8U);
    if (send(PACKET1_CAN_ID, FDCAN_DLC_BYTES_8) == 0U) {
        can_tx_group_partial_fail_count++;
        recover_can_if_bus_off();
        return 0U;
    }

    memcpy(shared_tx_data, &packet0X02, 8U);
    if (send(PACKET2_CAN_ID, FDCAN_DLC_BYTES_8) == 0U) {
        can_tx_group_partial_fail_count++;
        recover_can_if_bus_off();
        return 0U;
    }

    memcpy(shared_tx_data, &packet0X03, 5U);
    if (send(PACKET3_CAN_ID, FDCAN_DLC_BYTES_5) == 0U) {
        can_tx_group_partial_fail_count++;
        recover_can_if_bus_off();
        return 0U;
    }

    can_tx_group_ok_count++;
    return 1U;
}

void update_imu(void) {
    const uint32_t now = HAL_GetTick();

    /* The IMU itself may output at 1 kHz. Only forward a new sample every
     * 3 ms, giving an effective CAN feedback rate of at most 333.33 Hz without
     * blocking inside the UART interrupt callback. */
    if ((uint32_t)(now - last_send_tick) < PACKET_PERIOD_MS) {
        uart_rate_limited_count++;
        return;
    }

    packet0X01.q0 = read_i16_le(buf + 6U + 40U);
    packet0X01.q1 = read_i16_le(buf + 6U + 42U);
    packet0X01.q2 = read_i16_le(buf + 6U + 44U);
    packet0X01.q3 = read_i16_le(buf + 6U + 46U);

    packet0X02.accx = read_i16_le(buf + 6U + 16U);
    packet0X02.accy = read_i16_le(buf + 6U + 18U);
    packet0X02.accz = read_i16_le(buf + 6U + 20U);
    packet0X02.gyrox = read_i16_le(buf + 6U + 10U);

    packet0X03.gyroy = read_i16_le(buf + 6U + 12U);
    packet0X03.gyroz = read_i16_le(buf + 6U + 14U);
    packet0X03.temperature = (int8_t)buf[6U + 3U];

    /* Only advance the 333.33 Hz limiter when the complete three-frame sample
     * was accepted by the FDCAN Tx FIFO. */
    if (enqueue_complete_can_sample() != 0U) {
        last_send_tick = now;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance != USART2) {
        return;
    }

    uart_rx_event_count++;

    /* HT normally does not stop DMA reception. If it ever occurs despite the
     * explicit disable below, do not parse and do not restart DMA here. */
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
        uart_half_transfer_event_count++;
        return;
    }

    if (validate_hi92_frame(Size) != 0U) {
        uart_frame_count++;
        update_imu();
    }

    start_imu_rx_dma();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        memset(buf, 0, sizeof(buf));
        start_imu_rx_dma();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_FDCAN1_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    can_filter_init();
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    HAL_Delay(300);
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);
    start_imu_rx_dma();
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file name and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert error line number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
