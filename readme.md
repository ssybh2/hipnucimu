# HIPNUC ch0x0 series imu forward board

This board will receive ch0x0 imu data through UART and then forward it into the CAN network using three packets.

The **baud rate** of the CH0X0 IMU is **921600**, using **Hi92** protocol at **1 kHz**.

If the board is not working, use the **CHCenter Tool** (the version in the repo, do **not** update) to check the
settings of HIPNUC CH0X0 IMU through your USB2UART converter first!

The forward result includes three packets, two packets of length 8, 1 packet of length 5, defined as follows:

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

To change the packet ID of these three packets, nav to `Core/Src/main.c`, modify the lines 49-51 as follows:

```c++
#define PACKET1_CAN_ID 0x01
#define PACKET2_CAN_ID 0x02
#define PACKET3_CAN_ID 0x03
```

Original document available resources can be found [here](https://www.hipnuc.com/resource_ch0x0.html).

You can find our pre-built artifact `.elf` firmware file under the action
page [here](https://github.com/AIMEtherCAT/hipnucimu/actions), the packet ID is `0x01`, `0x02`, and `0x03`.