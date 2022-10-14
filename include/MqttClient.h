#ifndef _MQTT_CLIENT_H_
#define _MQTT_CLIENT_H_


#include "Config.h"
#include <PubSubClient.h>

extern PubSubClient mqtt_client;
extern const char* mqtt_client_id;
extern char mqtt_topic[128];

void mqtt_init();
void mqtt_run(uint32_t now=0);
void mqtt_publish(const char* topic, const char* buffer);

extern RegGroup configGroupMqtt;
extern ConfigBool configMqttUseJson;

#ifndef MQTT_SERVER
#define MQTT_SERVER "192.168.1.242"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif

#ifndef MQTT_USER
#define MQTT_USER "MyMqttUser"
#endif

#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD "MyMqttPassword"
#endif

#ifndef MQTT_MAIN_TOPIC
#define MQTT_MAIN_TOPIC __HOSTNAME__ "/"
#endif


#endif // _MQTT_CLIENT_H_
