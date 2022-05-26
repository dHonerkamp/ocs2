//
// Created by honerkam on 16/05/22.
//

#include "ocs2_mobile_manipulator_ros/RobotInterface.h"
#include "std_msgs/Float64.h"


RobotInterface::RobotInterface(std::string base_cmd_topic, ros::NodeHandle &nodeHandle, std::vector<std::string>arm_joint_names) :
        arm_joint_names_{arm_joint_names}{
      base_cmd_pub_ = nodeHandle.advertise<geometry_msgs::Twist>(base_cmd_topic, 1);
}

void RobotInterface::sendBaseCommand(const ocs2::vector_t &systemInput, const tf::Transform &base_tf){
    tf::Transform cmd_world;
    cmd_world.setOrigin(tf::Vector3(systemInput[0], systemInput[1], 0.0));

    tf::Transform base_no_trans;
    base_no_trans.setRotation(base_tf.getRotation());
    tf::Vector3 cmd_relative = (base_no_trans * cmd_world).getOrigin();

    //  TODO: in what form and frame are systemInput? Velocities? Map frame?
    // base command, in m/s and rad/s
    geometry_msgs::Twist base_cmd_rel;
    base_cmd_rel.linear.x = cmd_relative.x();
    base_cmd_rel.linear.y = cmd_relative.y();
    base_cmd_rel.linear.z = 0.0;
    base_cmd_rel.angular.z = systemInput[2];
    base_cmd_pub_.publish(base_cmd_rel);
}

void RobotInterface::sendArmCommands(const ocs2::vector_t &joint_values, const double &timestamp){
    throw std::runtime_error("Please override.");
}



/////////////////////////////
// PR2Interface
/////////////////////////////
PR2Interface::PR2Interface(ros::NodeHandle &nodeHandle)
        : RobotInterface("/base_controller/command", nodeHandle, {"torso_lift_joint",
                                                                  "r_shoulder_pan_joint",
                                                                  "r_shoulder_lift_joint",
                                                                  "r_upper_arm_roll_joint",
                                                                  "r_elbow_flex_joint",
                                                                  "r_forearm_roll_joint",
                                                                  "r_wrist_flex_joint",
                                                                  "r_wrist_roll_joint"}) {
  for (auto jn: arm_joint_names_) {
    std::string controller_name = jn.substr(0, jn.size() - 6) + "_velocity_controller/command";
    std::cout << controller_name << std::endl;
    pubs_.push_back(nodeHandle.advertise<std_msgs::Float64>(controller_name, 1));

    ;
  }
}

void PR2Interface::sendArmCommands(const ocs2::vector_t &joint_values, const double &timestamp) {
    // NOTE: 0-2 are base commands!
    if (joint_values.size() != 3 + arm_joint_names_.size()){
      std::cout<< "joint_values size: " << joint_values.size() << std::endl;
      throw std::runtime_error("joint_values has wrong size");
    }

    int next_controller = 0;
    for (int j = 3; j < joint_values.size(); j++) {
      // std::cout << j << ": " << joint_values[j] << std::endl;
      std_msgs::Float64 msg;
      msg.data = joint_values[j];
      pubs_[next_controller].publish(msg);
      next_controller ++;
    }
}


/////////////////////////////
// HSRInterface
/////////////////////////////
#include "tmc_msgs/JointVelocity.h"

HSRInterface::HSRInterface(ros::NodeHandle &nodeHandle)
        : RobotInterface("/hsrb/command_velocity", nodeHandle, {"arm_lift_joint",
                                                                "arm_flex_joint",
                                                                "arm_roll_joint",
                                                                "wrist_flex_joint",
                                                                "wrist_roll_joint"}) {
  pub_ = nodeHandle.advertise<tmc_msgs::JointVelocity>("/hsrb/pseudo_velocity_controller/ref_joint_velocity", 1);
}

void HSRInterface::sendArmCommands(const ocs2::vector_t &joint_values, const double &timestamp) {
  // NOTE: 0-2 are base commands!
  if (joint_values.size() != 3 + arm_joint_names_.size()) {
    std::cout << "joint_values size: " << joint_values.size() << std::endl;
    throw std::runtime_error("joint_values has wrong size");
  }

  tmc_msgs::JointVelocity msg;
  msg.header.stamp = ros::Time::now();
  int next_jn = 0;
  for (int j = 3; j < joint_values.size(); j++) {
    msg.name.push_back(arm_joint_names_[next_jn]);
    msg.velocity.push_back(joint_values[j]);
    next_jn++;
  }
  pub_.publish(msg);
}


/////////////////////////////
// TiagoInterface
/////////////////////////////
TiagoInterface::TiagoInterface(ros::NodeHandle &nodeHandle)
        : RobotInterface("does_not_exist", nodeHandle, {"torso_lift_joint",
                                                        "arm_1_joint",
                                                        "arm_2_joint",
                                                        "arm_3_joint",
                                                        "arm_4_joint",
                                                        "arm_5_joint",
                                                        "arm_6_joint",
                                                        "arm_7_joint"}) {
}

void TiagoInterface::sendArmCommands(const ocs2::vector_t &joint_values, const double &timestamp) {
  throw std::runtime_error("Tiago does not have velocity controllers");
}
