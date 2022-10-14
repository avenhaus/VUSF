#ifndef _VUSF_IMU_H_
#define _VUSF_IMU_H_

#include "Sensor.h"
#include "ICM_20948.h" 

#if USE_IMU

#if USE_ROS2
#include "Ros2.h"
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/temperature.h>
#endif

namespace VUSF {

class IMU : protected Sensor {
public:
    typedef enum ImuAxis { AXIS_X, AXIS_Y, AXIS_Z } ImuAxis;
    typedef struct ImuData {
        float acc[3];
        float gyr[3];
        float mag[3];
        float temp;
    } ImuData;

    IMU(const char* name, const char* csvHeader=nullptr);
    virtual ~IMU() {}

    // virtual void run(uint32_t now=0);
    virtual size_t getCSV(char* buffer, size_t size);
    virtual size_t getJSON(char* buffer, size_t size);
    bool hasMagnetometer() { return _hasMag; }
    bool hasMTemperature() { return _hasTemp; }

    ImuData data;

protected:
    bool _hasMag;
    bool _hasTemp;
    uint32_t _initializeTs;
    uint32_t _nextSampleTs;


#if USE_ROS2
    rcl_publisher_t _ros2ImuPublisher;
    sensor_msgs__msg__Imu _ros2ImuMsg;
#endif

};


class Icm20948Spi : IMU {
public:
    Icm20948Spi(const char* name, SPIClass& port, uint8_t csPin, uint32_t spiFreq = ICM_20948_SPI_DEFAULT_FREQ);
    void run(uint32_t now=0);

    ICM_20948_SPI icm;

protected:
    SPIClass& _port;
    uint8_t _csPin;
    uint32_t _spiFreq;


#if USE_ROS2
    void _ros2CreatePublishers();
    void _ros2PublishUpdate();
    rcl_publisher_t _ros2MagneticFieldPublisher;
    sensor_msgs__msg__MagneticField _ros2MagneticFieldMsg;
    rcl_publisher_t _ros2TemperaturePublisher;
    sensor_msgs__msg__Temperature _ros2TemperatureMsg;
#endif

};


}




#endif // USE_IMU
#endif // _VUSF_IMU_H_