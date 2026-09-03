# HIPNUC ch0x0 series IMU forward board

This board receives CH0X0 IMU data through UART and forwards it to CAN using three Classic CAN packets per logical IMU sample.

The CH0X0 IMU UART side remains configured at **921600 baud**, using the **Hi92** protocol at up to **1 kHz**. On branch `feature/6imu-333hz-stable`, the STM32G431 forwarding board rate-limits CAN output to one complete three-frame sample every **3 ms**, i.e. approximately **333.33 Hz per IMU**.

This branch is intended for the 6-IMU EtherCAT setup as a lower-load A/B alternative to `feature/6imu-500hz-stable`.

## CAN packet layout

Each logical IMU sample is forwarded as exactly three Classic CAN frames:

```c++
struct packet1_t {
    int16_t q0;
    int16_t q1;
    int16_t q2;
    int16_t q3;
} __attribute__((packed));

struct packet2_t {
    int16_t accx;
    int16_t accy;
    int16_t accz;
    int16_t gyrox;
} __attribute__((packed));

struct packet3_t {
    int16_t gyroy;
    int16_t gyroz;
    int8_t temperature;
} __attribute__((packed));
```

## Three firmware slots

The build generates three reusable firmware images. CAN1 and CAN2 are independent buses, so the same three slot IDs can be reused on both buses:

- **Slot 1:** `0x01 / 0x02 / 0x03`
- **Slot 2:** `0x04 / 0x05 / 0x06`
- **Slot 3:** `0x07 / 0x08 / 0x09`

For a six-IMU setup, flash one Slot 1, one Slot 2, and one Slot 3 G431 board on CAN1, then repeat the same three slot firmwares on CAN2.

## 333 Hz forwarding behavior

The rate limiter is defined in `Core/Src/main.c`:

```c
#define PACKET_PERIOD_MS 3U
```

The G431 only advances this limiter after all three CAN frames for a logical sample have been accepted by the FDCAN TX FIFO. The HiPNUC sensor may still deliver UART frames faster than 333 Hz; excess frames are intentionally not forwarded to CAN.

## GitHub Actions firmware artifacts

GitHub Actions builds all three slots and publishes the artifact:

`hipnucimu-333hz-slots`

The artifact contains:

- `hipnucimu.elf` — Slot 1
- `hipnucimu_slot1.hex` / `hipnucimu_slot1.bin` — Slot 1
- `hipnucimu_slot2.elf` / `.hex` / `.bin` — Slot 2
- `hipnucimu_slot3.elf` / `.hex` / `.bin` — Slot 3

Repository Actions page: https://github.com/ssybh2/hipnucimu/actions

If the board is not working, use the CHCenter Tool version included in this repository to check the HIPNUC CH0X0 configuration through a USB-to-UART converter before changing firmware parameters.
