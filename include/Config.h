#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <stdio.h>
#include "VUEF.h"

/********************************************\
|*  Pin Definitions
\********************************************/

/*
https://drive.google.com/file/d/1gbKM7DA7PI7s1-ne_VomcjOrb0bE2TPZ/view
---+------+----+-----+-----+-----------+---------------------------
No.| GPIO | IO | RTC | ADC | Default   | Function
---+------+----+-----+-----+-----------+---------------------------
25 |   0* | IO | R11 | 2_1 | Boot      |  
35 |   1  | IO |     |     | UART0_TXD | USB Programming/Debug
24 |   2* | IO | R12 | 2_2 |           | 
34 |   3  | IO |     |     | UART0_RXD | USB Programming/Debug
26 |   4* | IO | R10 | 2_0 |           |
29 |   5* | IO |     |     | SPI0_SS   | SD_CS (LED)  
14 |  12* | IO | R15 | 2_5 |           | Camera Pan Servo
16 |  13  | IO | R14 | 2_4 |           | Camera Tilt Servo
13 |  14  | IO | R16 | 2_6 |           | LIDAR UART RXD
23 |  15* | IO | R13 | 2_3 |           | 
27 |  16+ | IO |     |     | UART2_RXD | Hoverboard UART RXD
28 |  17+ | IO |     |     | UART2_TXD | Hoverboard UART TXD
30 |  18  | IO |     |     | SPI0_SCK  | 
31 |  19  | IO |     |     | SPI0_MISO | 
33 |  21  | IO |     |     | I2C0_SDA  |
36 |  22  | IO |     |     | I2C0_SCL  |
37 |  23  | IO |     |     | SPI0_MOSI | 
10 |  25  | IO | R06 | 2_8 |DAC1/I2S-DT| Ultrasonic Ping
11 |  26  | IO | R07 | 2_9 |DAC2/I2S-WS| Ultrasonic Echo 1
12 |  27  | IO | R17 | 2_7 | I2S-BCK   | WHEEL_L_PWM
8  |  32  | IO | R09 | 1_4 |           | WHEEL_L_C1
9  |  33  | IO | R08 | 1_5 |           | WHEEL_L_C2
6  |  34  | I  | R04 | 1_6 |           | WHEEL_L_ENC_A
7  |  35  | I  | R05 | 1_7 |           | WHEEL_L_ENC_B
4  |  36  | I  | R00 | 1_0 | SENSOR_VP | 
5  |  39  | I  | R03 | 1_3 | SENSOR_VN | 
3  |  EN  | I  |     |     | RESET     | Reset LCD       
---+------+----+-----+-----+-----------+---------------------------
(IO6 to IO11 are used by the internal FLASH and are not useable)
22 x I/O  + 4 x input only = 26 usable pins 
GPIO_34 - GPIO_39 have no internal pullup / pulldown.
+ GPIO 16 and 17 are not available on WROVER (PSRAM)
* Strapping pins: IO0, IO2, IO4, IO5 (HIGH), IO12 (LOW), IO15 (HIGH)
*/


#define LED_PIN 5

#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 21

// Camsesnse X1 LIDAR
#define LIDAR_UART_RXD_PIN 14
#define LIDAR_UART_TXD_PIN -1
#define LIDAR_UART_BAUD 115200


// GPS Module
#define USE_GPS 1
#define GPS_UART_RXD_PIN 14
#define GPS_UART_TXD_PIN -1
#define GPS_UART_BAUD 9600

// IMU 
#define USE_IMU 1
#define USE_IMU_SPI 1
#define IMU_SPI_PORT SPI
#define IMU_SPI_FREQ 5000000
#define IMU_SPI_SCK_PIN 18
#define IMU_SPI_MISO_PIN 19
#define IMU_SPI_MOSI_PIN 23
#define IMU_SPI_CS_PIN 15

#define USE_MQTT 1
#define USE_ROS1 1
#define USE_ROS2 1

#endif // CONFIG_H