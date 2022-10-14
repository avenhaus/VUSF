#include <Arduino.h>
#include "Config.h"
#include "VUSF/Sensor.h"
#include "MqttClient.h"

using namespace VUSF;

std::vector<Sensor*> Sensor::_sensors;
uint32_t Sensor::_count;
std::vector<Sensor::SendUpdateCB> Sensor::_sendUpdateCb;
RegGroup Sensor::configGroupSensor(FST("Sensor"));
char Sensor::_tmpBuffer [SENSOR_TMP_BUFFER_SIZE];


const StateEnum::Option Sensor::sensorStateOptions[] PROGMEM = {
    "OK", SS_OK, 
    "Disabled", SS_DISABLED, 
    "Unknown", SS_UNKNOWN, 
    "Initializing", SS_INITIALIZING,
    "Not Found", SS_NOT_FOUND, 
    "Timeout", SS_TIMEOUT, 
    "Error", SS_ERROR,
    "CRC Error", SS_CRC_ERROR
};
const size_t Sensor::sensorStateOptionsSize = sizeof(Sensor::sensorStateOptions) / sizeof(StateEnum::Option);


size_t Sensor::strCopy(char* buffer, size_t size, const char* s) {
    if (!s) { return 0; }
    size_t n = 0;
    while ( n < size-1 && s[n]) { buffer[n++] = s[n]; }
    buffer[n] = '\0';
    return n;
}

void Sensor::_sendUpdate() {
    for(const auto& cb: _sendUpdateCb) { cb.callback(this, cb.data); }

#if USE_ROS2
 if (!configRos2Disabled.get() && ros2ConnectionState.get() == R2CS_OK) { _ros2PublishUpdate(); }
#endif // USE_ROS2
}

