#include <Arduino.h>
#include "VUSF/IMU.h"


#if USE_IMU
using namespace VUSF;

RegGroup configGroupImu(FST("IMU"), &Sensor::configGroupSensor);
ConfigUInt32 configImuPeriodMs(FST("Period"), 1000, FST("IMU update time in ms."), 0, &configGroupImu);

IMU::IMU(const char* name, const char* csvHeader) :  data({0.0}), _hasMag(false), _hasTemp(false), _initializeTs(0), _nextSampleTs(0), Sensor(name, csvHeader) {}

size_t IMU::getCSV(char* buffer, size_t size) {
  size_t n = snprintf(buffer, size, FST("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f"),
    data.acc[AXIS_X], data.acc[AXIS_Y], data.acc[AXIS_Z], 
    data.gyr[AXIS_X], data.gyr[AXIS_Y], data.gyr[AXIS_Z]);
  if (_hasMag) {
    n += snprintf(buffer+n, size-n, FST(",%.2f,%.2f,%.2f"),
    data.mag[AXIS_X], data.mag[AXIS_Y], data.mag[AXIS_Z]);
  }
  if (_hasTemp) { n += snprintf(buffer+n, size-n, FST(",%.2f"), data.temp); }
  buffer[n] = '\0';
  return n;
}

size_t IMU::getJSON(char* buffer, size_t size) {
  size_t n = snprintf(buffer, size, FST("{\"acc\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},\"gyro\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}"),
    data.acc[AXIS_X], data.acc[AXIS_Y], data.acc[AXIS_Z], 
    data.gyr[AXIS_X], data.gyr[AXIS_Y], data.gyr[AXIS_Z]);
  if (_hasMag) {
    n += snprintf(buffer+n, size-n, FST(",\"mag\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}"),
    data.mag[AXIS_X], data.mag[AXIS_Y], data.mag[AXIS_Z]);
  }
  if (_hasTemp) { n += snprintf(buffer+n, size-n, FST(",\"temp\":%.2f"), data.temp); }
  if (n < size) { buffer[n++] = '}'; }
  buffer[n] = '\0';
  return n;
}

static const char _csvHeader[] PROGMEM = "ax,ay,az,gx,gy,gz,mx,my,mz,t";
Icm20948Spi::Icm20948Spi(const char* name, SPIClass& port, uint8_t csPin, uint32_t spiFreq) : _port(port), _csPin(csPin), IMU(name, _csvHeader) {
    DEBUG_println(FST("Create IMU ICM-20948 in SPI mode"));
    _hasMag = true;
    _hasTemp = true;
}

void Icm20948Spi::run(uint32_t now) {
    if (!now) { now =  millis(); }
    if (_state == SS_UNKNOWN) { _setState(SS_INITIALIZING); }
    if (_state == SS_INITIALIZING && now > _initializeTs) { 
        icm.begin(_csPin, _port, _spiFreq);
        if (icm.status == ICM_20948_Stat_Ok) {
            _setState(SS_OK);
            _nextSampleTs = now;
            DEBUG_println(FST("IMU ICM-20948 initialized"));
        } else {
            DEBUG_printf(FST("Could not initialize ICM-20948 IMU:%s\n"), icm.statusString());
            _initializeTs = now + configImuPeriodMs.get();
        }
    }
    if (_state == SS_OK && now >= _nextSampleTs &&icm.dataReady()) {
        _nextSampleTs += 1000;
        icm.getAGMT();         // The values are only updated when you call 'getAGMT'
        data.acc[AXIS_X] = icm.accX();
        data.acc[AXIS_Y] = icm.accY();
        data.acc[AXIS_Z] = icm.accZ();
        data.gyr[AXIS_X] = icm.gyrX();
        data.gyr[AXIS_Y] = icm.gyrY();
        data.gyr[AXIS_Z] = icm.gyrZ();
        data.mag[AXIS_X] = icm.magX();
        data.mag[AXIS_Y] = icm.magY();
        data.mag[AXIS_Z] = icm.magZ();
        data.temp = icm.temp();
        _sendUpdate();
    }
}

#if USE_ROS2
void Icm20948Spi::_ros2CreatePublishers() {
  _ros2ImuMsg.header.frame_id = micro_ros_string_utilities_set(_ros2ImuMsg.header.frame_id, FST("imu_link"));
  ros2_create_publisher(&_ros2ImuPublisher, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), FST("/imu/data"));
  _ros2MagneticFieldMsg.header.frame_id = micro_ros_string_utilities_set(_ros2MagneticFieldMsg.header.frame_id, FST("imu_link"));
  ros2_create_publisher(&_ros2MagneticFieldPublisher, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), FST("/imu/mag"));
  _ros2TemperatureMsg.header.frame_id = micro_ros_string_utilities_set(_ros2TemperatureMsg.header.frame_id, FST("imu_link"));
  ros2_create_publisher(&_ros2TemperaturePublisher, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), FST("/imu/temp"));
}

void Icm20948Spi::_ros2PublishUpdate() {
  struct timespec tv = {0};
  clock_gettime(0, &tv);
  _ros2ImuMsg.header.stamp.nanosec = tv.tv_nsec;
  _ros2ImuMsg.header.stamp.sec = tv.tv_sec;

  _ros2ImuMsg.linear_acceleration.x =  icm.accX();
  _ros2ImuMsg.linear_acceleration.y =  icm.accY();
  _ros2ImuMsg.linear_acceleration.z =  icm.accZ();

  _ros2ImuMsg.angular_velocity.x =  icm.gyrX();
  _ros2ImuMsg.angular_velocity.y =  icm.gyrY();
  _ros2ImuMsg.angular_velocity.z =  icm.gyrZ();

  rcl_ret_t rc = rcl_publish(&_ros2ImuPublisher, &_ros2ImuMsg, NULL);
  if(rc != RCL_RET_OK) { DEBUG_printf(FST("Could not publish ROS2 IMU data: %d\n"), rc); }

  _ros2MagneticFieldMsg.header.stamp.nanosec = tv.tv_nsec;
  _ros2MagneticFieldMsg.header.stamp.sec = tv.tv_sec;
  _ros2MagneticFieldMsg.magnetic_field.x = icm.magX();
  _ros2MagneticFieldMsg.magnetic_field.y = icm.magY();
  _ros2MagneticFieldMsg.magnetic_field.z = icm.magZ();
  rc = rcl_publish(&_ros2MagneticFieldPublisher, &_ros2MagneticFieldMsg, NULL);
  if(rc != RCL_RET_OK) { DEBUG_printf(FST("Could not publish ROS2 IMU mag: %d\n"), rc); }

  _ros2TemperatureMsg.header.stamp.nanosec = tv.tv_nsec;
  _ros2TemperatureMsg.header.stamp.sec = tv.tv_sec;
  _ros2TemperatureMsg.temperature = icm.temp();
  rc = rcl_publish(&_ros2TemperaturePublisher, &_ros2TemperatureMsg, NULL);
  if(rc != RCL_RET_OK) { DEBUG_printf(FST("Could not publish ROS2 IMU temp: %d\n"), rc); }
}

#endif // USE_ROS2


#endif // USE_IMU