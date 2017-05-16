#include <iostream>
#include <ros/ros.h>
#include "sdh_ros/SDHROSBridge.h"

int main(int argc, char** argv) {
    std::string node_name("sdh_ros_bridge");
    ros::init(argc, argv, node_name);
    ROS_INFO("Starting SDH ROS bridge");
    sdh_ros::SDHROSBridge bridge(node_name);
    ros::Rate loop_rate(10.0);  // Hz

    // init bridge
    sdh_ros::Parameters params;
    bridge.nh_.param<std::string>("joint_state_topic", params.joint_state_output_topic,
                                  "/sdh/joint_states");
    bridge.nh_.param<std::string>("tactile_sensor_topic", params.tactile_sensors_output_topic,
                                  "/sdh/tactile_sensors");
    bridge.nh_.param<std::string>("joint_command_topic", params.joint_position_input_topic,
                                  "/sdh/joint_commands");
    bridge.nh_.param<std::string>("sdh_device_name", params.sdh_device_name,
                                  "/dev/ttyUSB%d");
    bridge.nh_.param<std::string>("dsa_device_name", params.dsa_device_name,
                                  "/dev/ttyUSB%d");
    bridge.nh_.param<int>("sdh_port", params.sdh_port, 0);
    bridge.nh_.param<int>("dsa_port", params.dsa_port, 1);
    if (bridge.nh_.hasParam("joint_names")) {
        bridge.nh_.getParam("joint_names", params.joint_names);
    } else {
        params.joint_names = {"sdh_knuckle_joint", "sdh_thumb_2_joint", "sdh_thumb_3_joint",
                "sdh_finger_12_joint", "sdh_finger_13_joint", "sdh_finger_22_joint", 
                "sdh_finger_23_joint"};
    }
    
    bool init_success = bridge.init(params);
    if (!init_success) {
        return -1;
    }
    while (bridge.nh_.ok())
    {
        // publish JointState
        // publish TactileData
        bridge.update();

        // sleep and waiting for messages, callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
