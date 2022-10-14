#ifndef _ROS2_H_
#define _ROS2_H_

#include <Config.h>

#if USE_ROS2

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

typedef enum Ros2ConnectionStateEnum { R2CS_OK, R2CS_DISABLED, R2CS_UNKNOWN, R2CS_INITIALIZING, R2CS_NOT_FOUND, R2CS_TIMEOUT, R2CS_ERROR } Ros2ConnectionStateEnum;
typedef void (*Ros2InitCbFunct)(void* data);
typedef struct Ros2InitCB {
    Ros2InitCbFunct callback;
    void* data;
} Ros2InitCB;

void ros2_init();
void ros2_run(uint32_t now=0);
void ros2AddInitCB(Ros2InitCbFunct cb, void* data=nullptr);
rcl_ret_t ros2_create_publisher(rcl_publisher_t* publisher, const rosidl_message_type_support_t* type_support, const char* topic);

extern RegGroup configGroupRos2;
extern ConfigBool configRos2Disabled;
extern StateEnum ros2ConnectionState;

#ifndef ROS2_SERVER
#define ROS2_SERVER "192.168.0.42"
#endif

#ifndef ROS2_PORT
#define ROS2_PORT 8888
#endif

#ifndef ROS2_NODE
#define ROS2_NODE "esp32-uros"
#endif

#ifndef ROS2_NAMESPACE
#define ROS2_NAMESPACE ""
#endif


#endif // USE_ROS2

#endif // _ROS2_H_
