#include "sdh_ros/SDHROSBridge.h"
#include <cmath>
#include <schunk_sdh/TactileSensor.h>
#include <schunk_sdh/TactileMatrix.h>
#include <schunk_sdh/sdhexception.h>

using namespace sdh_ros;

SDHROSBridge::SDHROSBridge(const std::string& node_name) {
        nh_ = ros::NodeHandle(node_name);
        current_angles_.resize(DOF_, 0.0);
        current_velocities_.resize(DOF_, 0.0);
        target_angles_.resize(DOF_, 0.0);
        sdh_ = std::make_shared<SDH::cSDH>(true, false, 0);
        dsa_ = nullptr;
        dsa_initialized_ = false;
        sdh_initialized_ = false;
        target_received_ = false;
        axes_.resize(DOF_);
        for (size_t i = 0; i < axes_.size(); ++i) 
        {
            axes_.at(i) = i;
        }
}

SDHROSBridge::~SDHROSBridge() {
        if (dsa_initialized_) {
                dsa_->Close();
        }
        if (sdh_initialized_) {
                sdh_->Close();
        }
}

bool SDHROSBridge::init(const Parameters& params) {
    joint_states_publisher_ = nh_.advertise<sensor_msgs::JointState>(params.joint_state_output_topic, 1);
    tactile_sensor_publisher_ = nh_.advertise<schunk_sdh::TactileSensor>(params.tactile_sensors_output_topic, 1);
    joint_positions_subscriber_ = nh_.subscribe(params.joint_position_input_topic, 1,
                                            &SDHROSBridge::targetPositionCallback, this);
    joint_names_ = params.joint_names;
    try
    {
        if (!sdh_initialized_) {
            sdh_->OpenRS232(params.sdh_port, 115200, 1, params.sdh_device_name.c_str());
            ROS_INFO("Initialized RS232 for SDH");
            sdh_initialized_ = true;
            sdh_->SetController(SDH::cSDH::eCT_POSE);
            sdh_->SetVelocityProfile(sdh_->eVP_RAMP);
        }
    }
    catch (SDH::cSDHLibraryException* e) 
    {
        ROS_ERROR("Failed to initialize sdh hand: %s", e->what());
        delete e;
        return false;
    }

    // Now the tactile sensors:
    try 
    {
        if (!dsa_initialized_) {
            dsa_ = std::make_shared<SDH::cDSA>(0, params.dsa_port,
                                               params.dsa_device_name.c_str());
            dsa_initialized_ = true;        
            dsa_->SetFramerate(1, true);
        }
    }
    catch (SDH::cSDHLibraryException* e)
    {
        ROS_ERROR("Failed to initialize tactile sensors: %s", e->what());
        delete e;
        return false;
    }
    if (dsa_initialized_ && sdh_initialized_)
    {
        ROS_INFO("SDH and DSA successfully initialized");
        return true;
    } else if (dsa_initialized_) {
        ROS_WARN("Tactile sensors (DSA) initialized. Could not initialize SDH.");
        return true;
    } else if (sdh_initialized_) {
        ROS_WARN("SDH initialized. Could not initialize tactile sensors (DSA).");
        return true;
    }
    ROS_ERROR("Could not initialie SDH and DSA.");
    return false;

}

void SDHROSBridge::update() {
    if (sdh_initialized_)
    {
        // read joint state
        try
        {
            current_angles_ = sdh_->GetAxisActualAngle(axes_);
            current_velocities_ = sdh_->GetAxisActualVelocity(axes_);
            // publish joint state
            publishJointStates();

        }
        catch (SDH::cSDHLibraryException* e) {
            ROS_ERROR("Could not retrieve joint angles from hand: %s", e->what());
            delete e;
        }
        // send command to sdh
        if (target_received_)
        {
            if (handHasReachedTarget()) {
                ROS_DEBUG("Target position reached. Stopping hand.");
                sdh_->Stop();
            } else {
                try
                {
                    sdh_->SetAxisTargetAngle(axes_, target_angles_);
                    sdh_->MoveHand(false);
                }
                catch (SDH::cSDHLibraryException* e) {
                    ROS_ERROR("Could not move command target positions: %s", e->what());
                    delete e;
                }
            }
        }
    // read tactile state
    // publish tactile state

    }
}

bool SDHROSBridge::stop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    ROS_WARN("Stopping SDH hand");
    try
    {
        if (sdh_initialized_) {
            sdh_->Stop();
        }
    }
    catch (SDH::cSDHLibraryException* e)
    {
        ROS_ERROR("Failed stopping the hand: %s", e->what());
        delete e;
        res.success = false;
        return false;
    }
    ROS_INFO("Stopping SDH hand successful");
    res.success = true;
    return true;
}

void SDHROSBridge::targetPositionCallback(const sensor_msgs::JointState& msg) {
    std::lock_guard<std::recursive_mutex> guard(target_angles_mutex_);
    if (msg.position.size() < DOF_ || msg.name.size() < DOF_) {
            ROS_ERROR("Received malformed target values. Insufficient number of joint positions");
            target_received_ = false;
            return;
    }
    target_angles_.resize(DOF_);
    std::map<std::string, double> target_config;
    for (size_t i = 0; i < msg.name.size(); ++i) {
            target_config[msg.name.at(i)] = msg.position.at(i);
            ROS_DEBUG("temp goal position for joint %s: %f", msg.name[i].c_str(), msg.position.at(i));
    }
    target_angles_[0] = target_config["sdh_knuckle_joint"];  // sdh_knuckle_joint
    target_angles_[1] = target_config["sdh_finger_22_joint"];  // sdh_finger22_joint
    target_angles_[2] = target_config["sdh_finger_23_joint"];  // sdh_finger23_joint
    target_angles_[3] = target_config["sdh_thumb_2_joint"];  // sdh_thumb2_joint
    target_angles_[4] = target_config["sdh_thumb_3_joint"];  // sdh_thumb3_joint
    target_angles_[5] = target_config["sdh_finger_12_joint"];  // sdh_finger12_joint
    target_angles_[6] = target_config["sdh_finger_13_joint"];  // sdh_finger13_joint
    target_received_ = true;
}

/////////// PRIVATE METHODS ////////////////
bool SDHROSBridge::handHasReachedTarget(double threshold) {
        double diff = 0.0;
        for (size_t i = 0; i < target_angles_.size(); ++i) {
                diff = fmax(fabs(current_angles_[i] - target_angles_[i]), diff);
        }
        return diff < threshold;
}

void SDHROSBridge::publishJointStates() {
      ROS_DEBUG("Publishing joint states.");
      ros::Time time = ros::Time::now();

      // create joint_state message
      sensor_msgs::JointState msg;
      msg.header.stamp = time;
      msg.name.resize(DOF_);
      msg.position.resize(DOF_);
      msg.velocity.resize(DOF_);
      msg.effort.resize(DOF_);
      // set joint names and map them to angles
      msg.name = joint_names_;
      // ['sdh_knuckle_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint']
      // pos
      msg.position[0] = current_angles_[0];// * M_PI / 180.0;  // sdh_knuckle_joint
      msg.position[1] = current_angles_[3]; // * M_PI / 180.0;  // sdh_thumb_2_joint
      msg.position[2] = current_angles_[4]; // * M_PI / 180.0;  // sdh_thumb_3_joint
      msg.position[3] = current_angles_[5]; // * M_PI / 180.0;  // sdh_finger_12_joint
      msg.position[4] = current_angles_[6]; // * M_PI / 180.0;  // sdh_finger_13_joint
      msg.position[5] = current_angles_[1]; // * M_PI / 180.0;  // sdh_finger_22_joint
      msg.position[6] = current_angles_[2]; // * M_PI / 180.0;  // sdh_finger_23_joint
      // vel
      msg.velocity[0] = current_velocities_[0]; // * M_PI / 180.0;  // sdh_knuckle_joint
      msg.velocity[1] = current_velocities_[3]; // * M_PI / 180.0;  // sdh_thumb_2_joint
      msg.velocity[2] = current_velocities_[4]; // * M_PI / 180.0;  // sdh_thumb_3_joint
      msg.velocity[3] = current_velocities_[5]; // * M_PI / 180.0;  // sdh_finger_12_joint
      msg.velocity[4] = current_velocities_[6]; // * M_PI / 180.0;  // sdh_finger_13_joint
      msg.velocity[5] = current_velocities_[1]; // * M_PI / 180.0;  // sdh_finger_22_joint
      msg.velocity[6] = current_velocities_[2]; // * M_PI / 180.0;  // sdh_finger_23_joint
      // publish message
      joint_states_publisher_.publish(msg);

      // because the robot_state_publisher doen't know about the mimic joint, we have to publish the coupled joint separately
      sensor_msgs::JointState mimicjointmsg;
      mimicjointmsg.header.stamp = time;
      mimicjointmsg.name.resize(1);
      mimicjointmsg.position.resize(1);
      mimicjointmsg.velocity.resize(1);
      mimicjointmsg.name[0] = "sdh_finger_21_joint";
      mimicjointmsg.position[0] = msg.position[0];  // sdh_knuckle_joint = sdh_finger_21_joint
      mimicjointmsg.velocity[0] = msg.velocity[0];  // sdh_knuckle_joint = sdh_finger_21_joint
      joint_states_publisher_.publish(mimicjointmsg);
}

