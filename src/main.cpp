/*======================================================================*\
 * ESP32 ROS Robot Controller
\*======================================================================*/

#include "Config.h"
#include "MqttClient.h"
#include "VUSF/Sensor.h"
#include "VUSF/GPS.h"
#include "VUSF/IMU.h"

#if USE_ROS2
#include "Ros2.h"
#endif


// Install "Colcon Tasks" Platform IO extension

#if USE_GPS
bool gps_init();
void gps_run(uint32_t now=0);
#endif // USE_GPS


void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  vuefInit();

#if USE_GPS
   Serial2.begin(GPS_UART_BAUD, SERIAL_8N1, GPS_UART_RXD_PIN, GPS_UART_TXD_PIN);
   new VUSF::GPS(FST("GPS"), &Serial2);
#endif // USE_GPS

#if USE_IMU
  IMU_SPI_PORT.begin();
  new VUSF::Icm20948Spi(FST("IMU"), IMU_SPI_PORT, IMU_SPI_CS_PIN, IMU_SPI_FREQ);
#endif


#if USE_MQTT
  mqtt_init();
#endif

#if USE_ROS2
  ros2_init();
  ros2AddInitCB(VUSF::Sensor::ros2CreateAllPublishers);
#endif
}


void loop() {
  uint32_t now = millis();
  vuefRun(now);
  VUSF::Sensor::runAll();

#if USE_MQTT
  mqtt_run();
#endif

#if USE_ROS2
  ros2_run(now);
#endif

} 

