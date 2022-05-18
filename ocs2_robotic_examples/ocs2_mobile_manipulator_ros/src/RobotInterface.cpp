//
// Created by honerkam on 16/05/22.
//

#include "ocs2_mobile_manipulator_ros/RobotInterface.h"
#include "std_msgs/Float64.h"


RobotInterface::RobotInterface(std::string base_cmd_topic, ros::NodeHandle &nodeHandle){
      base_cmd_pub_ = nodeHandle.advertise<geometry_msgs::Twist>(base_cmd_topic, 1);
}

//void RobotInterface::getObservationCallback(const sensor_msgs::JointState::ConstPtr &joint_states) {
//    currentObservation_.time = joint_states->header.stamp.toSec();
//
//    std::vector <std::string> joint_names{"torso_lift_joint",
//                                          "r_shoulder_pan_joint",
//                                          "r_shoulder_lift_joint",
//                                          "r_upper_arm_roll_joint",
//                                          "r_elbow_flex_joint",
//                                          "r_forearm_roll_joint",
//                                          "r_wrist_flex_joint",
//                                          "r_wrist_roll_joint"};
//
//    //  currentObservation.state[0] = base_x;
//    //  currentObservation.state[1] = base_y;
//    //  currentObservation.state[2] = base_theta;
//
//  int idx = 3;
//  for (int i = 0; i < joint_names.size(); i++) {
//    for (int j = 0; j < joint_states->name.size(); j++) {
//      if (joint_states->name[j] == joint_names[i]) {
//        currentObservation_.state[idx] = joint_states->position[j];
//        idx += i;
//        std::cout << idx << " " << joint_states->name[j] << ": " << currentObservation_.state[idx] << std::endl;
//        break;
//      }
//    }
//  }
//  std::cout << "New observation" << std::endl;
//}

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
  : RobotInterface("/base_controller/command", nodeHandle){
    for (auto jn: arm_joint_names_){
      std::string controller_name = jn.substr(0, jn.size() - 6) + "_velocity_controller/command";
      std::cout << controller_name << std::endl;
      pubs_.push_back(nodeHandle.advertise<std_msgs::Float64>(controller_name, 1));
    }
}

void PR2Interface::sendArmCommands(const ocs2::vector_t &joint_values, const double &timestamp) {
    // NOTE: 0-2 are base commands!
    if (joint_values.size() != 3 + arm_joint_names_.size()){
      std::cout<< "joint_values size: " << joint_values.size() << std::endl;
      throw std::runtime_error("Initial target has wrong size");
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
