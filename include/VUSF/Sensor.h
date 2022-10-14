#ifndef _VUSF_SENSOR_H_
#define _VUSF_SENSOR_H_

#include <vector>

#include "Config.h"

#if USE_ROS2
#include "Ros2.h"
#endif

#ifndef SENSOR_TMP_BUFFER_SIZE
#define SENSOR_TMP_BUFFER_SIZE 256  // TODO: Don't use this buffer
#endif

namespace VUSF {

class Sensor {
public:
    typedef enum SensorStateEnum { SS_OK, SS_DISABLED, SS_UNKNOWN, SS_INITIALIZING, SS_NOT_FOUND, SS_TIMEOUT, SS_ERROR, SS_CRC_ERROR } SensorStateEnum;

    typedef void (*SendUpdateCbFunct)(Sensor* sensor, void* data);
    typedef struct SendUpdateCB {
        SendUpdateCbFunct callback;
        void* data;
    } SendUpdateCB;

    Sensor(const char* name, const char* csvHeader=nullptr) : _name(name), _csvHeader(csvHeader), _isUpdated(false), _isChanged(false), _state(SS_UNKNOWN) {
        _id = _count++;
        _configGroup = new RegGroup(_name, &configGroupSensor);
        _sensorState = new StateEnum(FST("State"), sensorStateOptions, sensorStateOptionsSize, SS_UNKNOWN, FST("state"), 0, _configGroup);
        _csvState = new StateStr(FST("CSV"), FST(""), FST("CSV"), 0, _configGroup, 0, _getCsvCB, this);
        _jsonState = new StateStr(FST("JSON"), FST("{}"), FST("JSON"), 0, _configGroup, 0, _getJsonCB, this);
        _sensors.push_back(this);
    }

    virtual ~Sensor() {
        if (_jsonState) { delete _jsonState; _jsonState = nullptr; }
        if (_csvState) { delete _csvState; _csvState = nullptr; }
        if (_sensorState) { delete _sensorState; _sensorState = nullptr; }
        if (_configGroup) { delete _configGroup; _configGroup = nullptr; }
        _sensors.erase(remove(_sensors.begin(), _sensors.end(), this), _sensors.end()); 
     }

    virtual void run(uint32_t now=0) {}
    virtual const char* getCsvHeader() { return _csvHeader; }
    virtual size_t getCsvHeader(char* buffer, size_t size) { return strCopy(buffer, size, _csvHeader); }
    virtual size_t getCSV(char* buffer, size_t size) { return 0; }
    virtual size_t getJSON(char* buffer, size_t size) { return 0; }
    inline uint32_t getId() { return _id; }
    inline SensorStateEnum getState() { return _state; }
    virtual const char* getName() { return _name; }
    virtual bool isUpdated() { return _isUpdated; }
    virtual bool isChanged() { return _isChanged; }
    virtual void disable(bool state) {
        if (state) { _setState(SS_DISABLED); }
        else if (_state == SS_DISABLED) { _setState(SS_UNKNOWN); }
    }
    
    static void addSendUpdateCB(SendUpdateCbFunct cb, void* data=nullptr) {
        _sendUpdateCb.push_back({cb, data});
    }
    static void runAll(uint32_t now=0) {
        for(const auto& s: _sensors) { s->run(now); }
    }
    static size_t strCopy(char* buffer, size_t size, const char* s);

    static RegGroup configGroupSensor;

    static const StateEnum::Option sensorStateOptions[];
    static const size_t sensorStateOptionsSize;

protected:
    void _setState(SensorStateEnum state) {
        if (state == _state) { return; }
        _state = state;
        if(_sensorState) { _sensorState->set(state); }
    }
    virtual void _sendUpdate();

#if USE_ROS2
public:
    static void ros2CreateAllPublishers(void* data) {
        for(const auto& s: _sensors) { s->_ros2CreatePublishers(); }  
    }
protected:
    virtual void _ros2CreatePublishers() {}
    virtual void _ros2PublishUpdate() {}
#endif // USE_ROS2

    static const char* _getCsvCB(void* sensor) {
        ((Sensor*)sensor)->getCSV(_tmpBuffer, sizeof(_tmpBuffer));
        return _tmpBuffer;
    }
    static const char* _getJsonCB(void* sensor) {
        ((Sensor*)sensor)->getJSON(_tmpBuffer, sizeof(_tmpBuffer));
        return _tmpBuffer;
    }

    const char* _name;
    uint32_t _id;
    const char* _csvHeader;
    bool _isUpdated;
    bool _isChanged;
    SensorStateEnum _state;

    RegGroup* _configGroup;
    StateEnum* _sensorState;
    StateStr* _jsonState;
    StateStr* _csvState;

    static std::vector<Sensor*> _sensors;
    static uint32_t _count;
    static std::vector<SendUpdateCB> _sendUpdateCb;
    static char _tmpBuffer [SENSOR_TMP_BUFFER_SIZE];
};


}


#endif // _VUSF_SENSOR_H_
