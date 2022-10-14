#include <Config.h>


#if USE_ROS2

#include <WiFi.h>
#include <WiFiUdp.h>
#include <vector>

#include "Ros2.h"

#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <stdio.h>
#include <unistd.h>


extern "C" bool arduino_wifi_transport_open(struct uxrCustomTransport * transport);
extern "C" bool arduino_wifi_transport_close(struct uxrCustomTransport * transport);
extern "C" size_t arduino_wifi_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
extern "C" size_t arduino_wifi_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
struct micro_ros_agent_locator {
	IPAddress address;
	int port;
};

RegGroup configGroupRos2(FST("ROS2"));

ConfigBool configRos2Disabled(FST("Disabled"), false, FST("Disable ROS2"), 0, &configGroupRos2);
ConfigStr configRos2Server(FST("Server"), 32, FST(ROS2_SERVER), FST("ROS2 Server"), 0, &configGroupRos2);
ConfigUInt16 configRos2Port(FST("Port"), ROS2_PORT, FST("ROS2 Port"), 0, &configGroupRos2);
ConfigStr configRos2Node(FST("Node"), 32, FST(ROS2_NODE), FST("ROS2 Node"), 0, &configGroupRos2);
ConfigStr configRos2Namespace(FST("configRos2Namespace"), 32, FST(ROS2_NAMESPACE), FST("ROS2 Namespace"), 0, &configGroupRos2);
ConfigUInt32 configRos2ConnectRetrytMs(FST("Connect Retry"), 1000, FST("ROS connect retry in ms"), 0, &configGroupRos2);

const StateEnum::Option ros2ConnectionStateOptions[] PROGMEM = {
    "OK", R2CS_OK, 
    "Disabled", R2CS_DISABLED, 
    "Unknown", R2CS_UNKNOWN, 
    "Initializing", R2CS_INITIALIZING,
    "Host Not Found", R2CS_NOT_FOUND, 
    "Timeout", R2CS_TIMEOUT, 
    "Error", R2CS_ERROR
};
const size_t ros2ConnectionStateOptionsSize = sizeof(ros2ConnectionStateOptions) / sizeof(StateEnum::Option);
StateEnum ros2ConnectionState(FST("State"), ros2ConnectionStateOptions, ros2ConnectionStateOptionsSize, R2CS_UNKNOWN, FST("state"), 0, &configGroupRos2);

static rclc_support_t _ros2Support;
static rcl_allocator_t _ros2Allocator;
static rcl_node_t _ros2Node;
static uint32_t _ros2ConnectNextRetryTs = 0;
static std::vector<Ros2InitCB> _ros2InitCb;

static inline void set_microros_udp_transports(const char * agent_ip, uint agent_port) {
	static struct micro_ros_agent_locator locator;
	locator.address.fromString(agent_ip);
	locator.port = agent_port;

	rmw_uros_set_custom_transport(
		false,
		(void *) &locator,
		arduino_wifi_transport_open,
		arduino_wifi_transport_close,
		arduino_wifi_transport_write,
		arduino_wifi_transport_read
	);
}

void ros2_init() {
	if (configRos2Disabled.get()) {
		DEBUG_println(FST("ROS2 is disabled"));
		ros2ConnectionState.set(R2CS_DISABLED);
	}
	set_microros_udp_transports(configRos2Server.get(), configRos2Port.get());
	_ros2Allocator = rcl_get_default_allocator();
	_ros2ConnectNextRetryTs = millis();
}

void ros2_run(uint32_t now) {
	  if (!now) { now=millis(); }
	if (ros2ConnectionState.get() != R2CS_OK && ros2ConnectionState.get() != R2CS_DISABLED && now >= _ros2ConnectNextRetryTs ) {
		ros2ConnectionState.set(R2CS_INITIALIZING);
		_ros2ConnectNextRetryTs = now + configRos2ConnectRetrytMs.get();
		rcl_ret_t rc = rclc_support_init(&_ros2Support, 0, NULL, &_ros2Allocator); 
		if(rc != RCL_RET_OK) { 
			DEBUG_printf(FST("Could not initialize ROS2 support: %d\n"), rc);
			ros2ConnectionState.set(R2CS_NOT_FOUND);
		}
		else {
			rc = rclc_node_init_default(&_ros2Node, configRos2Node.get(), configRos2Namespace.get(), &_ros2Support);
			if(rc != RCL_RET_OK) { 
				DEBUG_printf(FST("Could not initialize ROS2 node: %d\n"), rc); 
				ros2ConnectionState.set(R2CS_ERROR);
			}
			else {
				for(const auto& cb: _ros2InitCb) { cb.callback(cb.data); }
				ros2ConnectionState.set(R2CS_OK);
				DEBUG_printf(FST("ROS2 initialized.\n")); 
			}
		}
	}
}

void ros2AddInitCB(Ros2InitCbFunct cb, void* data) {
	_ros2InitCb.push_back({cb, data});
}

rcl_ret_t ros2_create_publisher(rcl_publisher_t* publisher, const rosidl_message_type_support_t* type_support, const char* topic) {
	if (ros2ConnectionState.get() != R2CS_OK && ros2ConnectionState.get() != R2CS_INITIALIZING) {
		DEBUG_printf(FST("Not ready to create ROS2 publisher: %s. ROS2 Connection State: %s\n"), topic, ros2ConnectionState.getText());
		return RCL_RET_NOT_INIT;
	}
    rcl_ret_t rc = rclc_publisher_init_best_effort(publisher, &_ros2Node, type_support, topic);
	if(rc != RCL_RET_OK) { DEBUG_printf(FST("Could not initialize ROS2 publisher %s: %d\n"), topic, rc); }
	else { DEBUG_printf(FST("Created ROS2 publisher: %s\n"), topic); }
    return rc;
}

#endif