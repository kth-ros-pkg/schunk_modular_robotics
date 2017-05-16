#ifndef _SDH_ROS_BRIDGE_H_
#define _SDH_ROS_BRIDGE_H_

#include <memory>
#include <mutex>
// ROS include
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
// external includes
#include <schunk_sdh/sdh.h>
#include <schunk_sdh/dsa.h>

namespace sdh_ros {
    struct Parameters {
            std::string joint_state_output_topic;
            std::string tactile_sensors_output_topic;
            std::string joint_position_input_topic;
            std::string sdh_device_name;
            std::string dsa_device_name;
            int sdh_port;
            int dsa_port;
            std::vector<std::string> joint_names;
    };

    class SDHROSBridge {
        public:
            SDHROSBridge(const std::string& node_name);
            ~SDHROSBridge();
            bool init(const Parameters& params);
            void update();
            const unsigned int DOF_ = 7;
            void targetPositionCallback(const sensor_msgs::JointState& msg);
            bool stop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
            ros::NodeHandle nh_;
        private:
            std::vector<int> axes_;
            std::vector<double> target_angles_;
            std::recursive_mutex target_angles_mutex_;
            std::vector<double> current_angles_;
            std::vector<double> current_velocities_;
            bool target_received_;
            std::shared_ptr<SDH::cSDH> sdh_;
            bool sdh_initialized_;
            std::shared_ptr<SDH::cDSA> dsa_;
            bool dsa_initialized_;
            std::vector<std::string> joint_names_;

            ros::Publisher joint_states_publisher_;
            ros::Publisher tactile_sensor_publisher_;
            ros::Subscriber joint_positions_subscriber_;
            ros::ServiceServer stop_service_;

            bool handHasReachedTarget(double threshold=0.001);
            void publishJointStates();
    };
}
#endif

