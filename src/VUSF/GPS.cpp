#include <Arduino.h>
#include "VUSF/GPS.h"

#if USE_GPS
using namespace VUSF;

static const char csvHeader[] PROGMEM = "lat,lon,alt,speed,course,date,time,sats,hdop";


GPS::GPS(const char* name, Stream* serial) : _serial(serial), _lastUpdateTs(0), Sensor(name, csvHeader) {
    DEBUG_println(FST("Create GPS"));
}

void GPS::run(uint32_t now) {
    if (!_serial) { return; }
    if(!now) { now = millis(); } 
    while (_serial->available()) {
      char c = _serial->read();
      //DEBUG_print(c);
      if (_state != SS_DISABLED && data.encode(c)) {
        _lastUpdateTs = now;
        if (data.location.isUpdated()) {
          if (_state != SS_OK) { _setState(SS_OK); }
          _sendUpdate();
        }        
      }
      if (data.failedChecksum() > _crcErrorCount) { _crcErrorCount = data.failedChecksum(); _setState(SS_CRC_ERROR); }
    }
    if (_state != SS_TIMEOUT && now >_lastUpdateTs + 5000 ) { _setState(SS_TIMEOUT); }
}

size_t GPS::_valueInvalidStr(char* buffer, size_t size) {
  return snprintf(buffer, size, FST("null"));
}

size_t GPS::_floatStr(char* buffer, size_t size, const char* fmt, double val, bool isValid) {
  if (!isValid) { return _valueInvalidStr(buffer, size); }
  return snprintf(buffer, size, fmt, val);
}

size_t GPS::_intStr(char* buffer, size_t size, int32_t val, bool isValid) {
  if (!isValid) { return _valueInvalidStr(buffer, size); }
  return snprintf(buffer, size, "%d", val);
}

size_t GPS::_timeStr(char* buffer, size_t size) {
  if (!data.time.isValid()) { return _valueInvalidStr(buffer, size); }
  return snprintf(buffer, size, FST("%02d:%02d:%02d.%02d"), data.time.hour(), data.time.minute(), data.time.second(), data.time.centisecond());
}

size_t GPS::_dateStr(char* buffer, size_t size) {
  if (!data.date.isValid()) { return _valueInvalidStr(buffer, size); }
  return snprintf(buffer, size, FST("%04d-%02d-%02d"), data.date.year(), data.date.month(), data.date.day());
}

size_t getCsvHeader(char* buffer, size_t size) {
  size_t n = 0;
  static const char* h = FST("lat,lon,alt,speed,course,date,time,sat,hdop");
  while ( n < size-1 && h[n]) { buffer[n++] = h[n]; }
  buffer[n] = '\0';
  return n;
}


size_t GPS::getCSV(char* buffer, size_t size) {
  size_t n = 0;
  n += _floatStr(buffer+n, size-n, FST("%0.8g"), data.location.lat(), data.location.isValid());
  if (n < size) { buffer[n++] = ','; }
  n += _floatStr(buffer+n, size-n, FST("%0.8f"), data.location.lng(), data.location.isValid());
  if (n < size) { buffer[n++] = ','; }
  n += _floatStr(buffer+n, size-n, FST("%0.3f"), data.altitude.meters(), data.altitude.isValid());
  if (n < size) { buffer[n++] = ','; }
  n += _floatStr(buffer+n, size-n, FST("%0.2f"), data.speed.kmph(), data.speed.isValid());
  if (n < size) { buffer[n++] = ','; }
  n += _floatStr(buffer+n, size-n, FST("%0.4f"), data.course.deg(), data.course.isValid());
  if (n < size) { buffer[n++] = ','; }
  n += _dateStr(buffer+n, size-n);
  if (n < size) { buffer[n++] = ','; }
  n += _timeStr(buffer+n, size-n);
  if (n < size) { buffer[n++] = ','; }
  n += _intStr(buffer+n, size-n, data.satellites.value(), data.satellites.isValid());
  if (n < size) { buffer[n++] = ','; }
  n += _intStr(buffer+n, size-n, data.hdop.value(), data.hdop.isValid());
  buffer[n] = '\0';
  return n;
}

size_t GPS::getJSON(char* buffer, size_t size) {
  size_t n = 0;
  n += strCopy(buffer+n, size-n, FST("{\"lat\":"));
  n += _floatStr(buffer+n, size-n, FST("%0.8f"), data.location.lat(), data.location.isValid());
  n += strCopy(buffer+n, size-n, FST(",\"lon\":"));
  n += _floatStr(buffer+n, size-n, FST("%0.8f"), data.location.lng(), data.location.isValid());
  n += strCopy(buffer+n, size-n, FST(",\"alt\":"));
  n += _floatStr(buffer+n, size-n, FST("%0.3f"), data.altitude.meters(), data.altitude.isValid());
  n += strCopy(buffer+n, size-n, FST(",\"speed\":"));
  n += _floatStr(buffer+n, size-n, FST("%0.2f"), data.speed.kmph(), data.speed.isValid());
  n += strCopy(buffer+n, size-n, FST(",\"course\":"));
  n += _floatStr(buffer+n, size-n, FST("%0.4f"), data.course.deg(), data.course.isValid());
  n += strCopy(buffer+n, size-n, FST(",\"date\":\""));
  n += _dateStr(buffer+n, size-n);
  n += strCopy(buffer+n, size-n, FST("\",\"time\":\""));
  n += _timeStr(buffer+n, size-n);
  n += strCopy(buffer+n, size-n, FST("\",\"sats\":"));
  n += _intStr(buffer+n, size-n, data.satellites.value(), data.satellites.isValid());
  n += strCopy(buffer+n, size-n, FST(",\"hdop\":"));
  n += _intStr(buffer+n, size-n, data.hdop.value(), data.hdop.isValid());
  if (n < size) { buffer[n++] = '}'; }

  buffer[n] = '\0';
  return n;
}

#if USE_ROS2
void GPS::_ros2CreatePublishers() {
    _ros2GpsFixMsg.header.frame_id = micro_ros_string_utilities_set(_ros2GpsFixMsg.header.frame_id, FST("gps_link"));
    ros2_create_publisher(&_ros2GpsFixPublisher, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), FST("/gps/fix"));
}

void GPS::_ros2PublishUpdate() {
  struct timespec tv = {0};
  clock_gettime(0, &tv);
  _ros2GpsFixMsg.header.stamp.nanosec = tv.tv_nsec;
  _ros2GpsFixMsg.header.stamp.sec = tv.tv_sec;
  _ros2GpsFixMsg.latitude = data.location.isValid() ? data.location.lat() : NAN;
  _ros2GpsFixMsg.longitude = data.location.isValid() ? data.location.lng() : NAN;
  _ros2GpsFixMsg.altitude =  data.altitude.isValid() ? data.altitude.meters() : NAN;
  rcl_ret_t rc = rcl_publish(&_ros2GpsFixPublisher, &_ros2GpsFixMsg, NULL); 
	if(rc != RCL_RET_OK) { DEBUG_printf(FST("Could not publish ROS2 GPS fix: %d\n"), rc); }
}
#endif // USE_ROS2

#endif // USE_GPS
