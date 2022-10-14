#ifndef _VUSF_GPS_H_
#define _VUSF_GPS_H_

#include <TinyGPSPlus.h>

#include "Sensor.h"

#if USE_GPS

#if USE_ROS2
#include "Ros2.h"
#include <sensor_msgs/msg/nav_sat_fix.h>
#endif

namespace VUSF {

class GPS : Sensor {
public:
    GPS(const char* name, Stream* serial);
    void run(uint32_t now=0);
    size_t getCSV(char* buffer, size_t size);
    size_t getJSON(char* buffer, size_t size);

    TinyGPSPlus data;

protected:
    size_t _valueInvalidStr(char* buffer, size_t size);
    size_t _floatStr(char* buffer, size_t size, const char* fmt, double val, bool isValid);
    size_t _intStr(char* buffer, size_t size, int32_t val, bool isValid);
    size_t _timeStr(char* buffer, size_t size);
    size_t _dateStr(char* buffer, size_t size);

    Stream* _serial = &Serial2;
    uint32_t _lastUpdateTs;
    uint32_t _crcErrorCount;

#if USE_ROS2
    void _ros2CreatePublishers();
    void _ros2PublishUpdate();
    rcl_publisher_t _ros2GpsFixPublisher;
    sensor_msgs__msg__NavSatFix _ros2GpsFixMsg;
#endif

};

}

#endif // USE_GPS
#endif // _VUSF_GPS_H_