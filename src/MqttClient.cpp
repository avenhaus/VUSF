#include "Config.h"

#if USE_MQTT
#include <WiFi.h>
#include "MqttClient.h"
#include "VUSF/Sensor.h"

RegGroup configGroupMqtt(FST("MQTT"));

ConfigBool configMqttDisabled(FST("Disabled"), false, FST("Disable MQTT"), 0, &configGroupMqtt);
ConfigStr configMqttServer(FST("Server"), 32, MQTT_SERVER, FST("MQTT Server"), 0, &configGroupMqtt);
ConfigUInt16 configMqttPort(FST("Port"), MQTT_PORT, FST("MQTT Port"), 0, &configGroupMqtt);
ConfigStr configMqttUser(FST("User"), 32, MQTT_USER, FST("MQTT User"), 0, &configGroupMqtt);
ConfigStr configMqttPassword(FST("Password"), 32, MQTT_PASSWORD, FST("MQTT Password"), 0, &configGroupMqtt);
ConfigStr configMqttMainTopic(FST("Topic"), 64, MQTT_MAIN_TOPIC, FST("MQTT main topic"), 0, &configGroupMqtt);
ConfigBool configMqttUseJson(FST("JSON"), true, FST("Use JSON instead of CSV for MQTT"), 0, &configGroupMqtt);
ConfigUInt32 configMqttConnectRetrytMs(FST("Connect Retry"), 1000, FST("MQTT connect retry in ms"), 0, &configGroupMqtt);

WiFiClient wifi_client;

PubSubClient mqtt_client(wifi_client);
const char* mqtt_client_id;
static uint32_t mqtt_connect_next_retry_ts = 0;

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  DEBUG_print(F("Message arrived ["));
  DEBUG_print(topic);
  DEBUG_print(F("] "));
  for (int i=0;i<length;i++) {
    char receivedChar = (char)payload[i];
    DEBUG_print(receivedChar);
  }
  DEBUG_println();
}


void mqtt_publish(const char* topic, const char* buffer) {
  if (configMqttDisabled.get() || !mqtt_client.connected()) { return; }
  char t[128];
  sprintf(t, "%s%s", configMqttMainTopic.get(), topic);
  DEBUG_println(t);
  int retry = 3;
  while (retry--) {
    if (mqtt_client.publish(t, buffer, true)) {
      yield();
      delay(10);
      return;
    }
    DEBUG_println(F("MQTT failed to publish!"));
    delay(10);
  }
}


void sensor_update_cb(VUSF::Sensor* sensor, void* data) {
    char buffer[512];
    if (configMqttUseJson.get()) { sensor->getJSON(buffer, sizeof(buffer));}
    else { sensor->getCSV(buffer, sizeof(buffer)); }
    //DEBUG_println(buffer);
    mqtt_publish(sensor->getName(), buffer);
}


void mqtt_init() {
  if (configMqttDisabled.get()) {
    DEBUG_println(FST("MQTT is disabled"));
    return;  
  }
  DEBUG_printf(FST("Connecting to MQTT server %s:%d. Topic: %s\n"), configMqttServer.get(), configMqttPort.get(), configMqttMainTopic.get());
  mqtt_client.setServer(configMqttServer.get(), configMqttPort.get());
  mqtt_client.setCallback(mqtt_callback);
  VUSF::Sensor::addSendUpdateCB(sensor_update_cb);
}

void mqtt_run(uint32_t now) {
  if (configMqttDisabled.get() || mqtt_client.connected()) { return; }
  if (!now) { now=millis(); }
  if (mqtt_connect_next_retry_ts > now) { return; }
  
  DEBUG_print(FST("Attempting MQTT connection..."));
  // Attempt to connect
  if (mqtt_client.connect(mqtt_client_id, configMqttUser.get(), configMqttPassword.get())) {
    DEBUG_println(F("connected"));
    //clearError(MQTT_ERROR);
    // ... and subscribe to topic
    // mqtt_client.subscribe("ledStatus");
  } else {
    DEBUG_print(F("failed, rc="));
    DEBUG_print(mqtt_client.state());
    //setError(MQTT_ERROR);
    DEBUG_println(F(" retry "));
    mqtt_connect_next_retry_ts = now + configMqttConnectRetrytMs.get();
  }
}


#endif // USE_MQTT


