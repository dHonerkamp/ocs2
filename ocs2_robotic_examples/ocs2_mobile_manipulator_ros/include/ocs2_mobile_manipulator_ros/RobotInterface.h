//
// Created by honerkam on 16/05/22.
//

#ifndef OCS2_MOBILE_MANIPULATOR_ROBOTINTERFACE_H
#define OCS2_MOBILE_MANIPULATOR_ROBOTINTERFACE_H
#include <ros/init.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>


class RobotInterface {
public:
    RobotInterface(std::string base_cmd_topic, ros::NodeHandle &nodeHandle);
    ocs2::SystemObservation currentObservation_;
    ros::Publisher base_cmd_pub_;

    void sendBaseCommand(const ocs2::vector_t &systemInput, const tf::Transform &base_tf);
    virtual void sendArmCommands(const ocs2::vector_t &joint_values, const double &timestamp) = 0;
};



class PR2Interface : public RobotInterface{
private:
    std::vector<ros::Publisher>pubs_;
    std::vector<std::string> arm_joint_names_ = {"torso_lift_joint",
                                                 "r_shoulder_pan_joint",
                                                 "r_shoulder_lift_joint",
                                                 "r_upper_arm_roll_joint",
                                                 "r_elbow_flex_joint",
                                                 "r_forearm_roll_joint",
                                                 "r_wrist_flex_joint",
                                                 "r_wrist_roll_joint"};
public:
    PR2Interface(ros::NodeHandle &nodeHandle);
    void sendArmCommands(const ocs2::vector_t &joint_values, const double &timestamp) override;
};


#endif //OCS2_MOBILE_MANIPULATOR_ROBOTINTERFACE_H
