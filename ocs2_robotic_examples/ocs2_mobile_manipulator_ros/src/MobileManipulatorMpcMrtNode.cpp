//
// Created by honerkam on 16/05/22.
//
/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/*
 * This file contains on an example on how to integrate the OCS2 MPC into your own control loop.
 * In contrast to the MPC-node + Dummy-node setup, in this case we will not use ROS to communicate between the MPC and MRT.
 * The MPC will run in a separate thread, and the ocs2::MPC_MRT_Interface facilitates all communication with this thread.
 *
 * This removes latency that arises when trying to do high frequency control over ROS.
 * ROS will only be used to send commands, and to publish visualization topics.
 */

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
//#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include "ocs2_mobile_manipulator_ros/MobileManipulatorDummyVisualization.h"
#include "ocs2_mobile_manipulator_ros/RobotInterface.h"

#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <ros/subscribe_options.h>

/**
 * This function implements the evaluation of the MPC policy
 * @param currentObservation : current system observation {time, state, input} to compute the input for. (input can be left empty)
 * @param mpcMrtInterface : interface used for communication with the MPC optimization (running in a different thread)
 * @return system input u(t)
 */
ocs2::vector_t mpcTrackingController(const ocs2::SystemObservation& currentObservation, ocs2::MPC_MRT_Interface& mpcMrtInterface);


void updateBaseTf(tf::TransformListener &listener, ocs2::SystemObservation &currentObservation){
  tf::StampedTransform newBaseTransform;
  listener.lookupTransform("map", "base_footprint", ros::Time(0), newBaseTransform);
  currentObservation.state[0] = newBaseTransform.getOrigin().x();
  currentObservation.state[1] = newBaseTransform.getOrigin().y();

  double roll, pitch, yaw;
  tf::Matrix3x3(newBaseTransform.getRotation()).getRPY(roll, pitch, yaw);
  currentObservation.state[2] = yaw;
}


class ObservationCallback {
public:
    ocs2::SystemObservation currentObservation_;
    bool has_received_msg_ = false;
    ObservationCallback(ocs2::scalar_t state_dim, ocs2::scalar_t inputDim){
      currentObservation_.state.setZero(state_dim);
      currentObservation_.input.setZero(inputDim);
    };

  void getObservationCallback(const sensor_msgs::JointState::ConstPtr &joint_states) {
    currentObservation_.time = joint_states->header.stamp.toSec();
    // std::cout << "Observation time: " << currentObservation_.time << std::endl;

    std::vector <std::string> joint_names{"torso_lift_joint",
                                          "r_shoulder_pan_joint",
                                          "r_shoulder_lift_joint",
                                          "r_upper_arm_roll_joint",
                                          "r_elbow_flex_joint",
                                          "r_forearm_roll_joint",
                                          "r_wrist_flex_joint",
                                          "r_wrist_roll_joint"};

    //  currentObservation.state[0] = base_x;
    //  currentObservation.state[1] = base_y;
    //  currentObservation.state[2] = base_theta;
    int idx = 3;
    for (auto jn : joint_names) {
      for (int j = 0; j < joint_states->name.size(); j++) {
        if (joint_states->name[j] == jn) {
          currentObservation_.state[idx] = joint_states->position[j];
          // std::cout << idx << " " << joint_states->name[j] << ": " << currentObservation_.state[idx] << std::endl;
          idx += 1;
          break;
        }
      }
    }
    has_received_msg_ = true;
    // std::cout << "New observation" << std::endl;
  }
};


int main(int argc, char** argv) {
  /*
   * Load the robot specific problem specification
   */
  const std::string robotName = "mobile_manipulator";

  // task file
  std::vector<std::string> programArgs{};
  ros::removeROSArgs(argc, argv, programArgs);

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc_mrt");
  ros::NodeHandle nodeHandle;

  // Robot interface
  // Get node parameters
  std::string taskFile, libFolder, urdfFile, world_type;
  nodeHandle.getParam("/ocs2_world_type", world_type);
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/libFolder", libFolder);
  nodeHandle.getParam("/urdfFile", urdfFile);
  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::cerr << "Loading library folder: " << libFolder << std::endl;
  std::cerr << "Loading urdf file: " << urdfFile << std::endl;
  // Robot interface
  ocs2::mobile_manipulator::MobileManipulatorInterface interface(taskFile, libFolder, urdfFile, nodeHandle);

  /*
   * Set up the MPC and the MPC_MRT_interface.
   * For this example we add a command interface and a visualization which both communicate over ros
   */

  // MPC
  ocs2::MPC_DDP mpc(interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(), interface.getOptimalControlProblem(),
                    interface.getInitializer());

  // ROS ReferenceManager. This gives us the command interface. Requires the observations to be published
  std::shared_ptr<ocs2::RosReferenceManager> rosReferenceManagerPtr(new ocs2::RosReferenceManager(robotName, interface.getReferenceManagerPtr()));
  rosReferenceManagerPtr->subscribe(nodeHandle);
  auto observationPublisher = nodeHandle.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // Visualization
  // this also initializes a robot_state_publisher
  ocs2::mobile_manipulator::MobileManipulatorDummyVisualization mobileManipulatorDummyVisualization(nodeHandle, interface, world_type == "gazebo");


  // Create the MPC MRT Interface
  ocs2::MPC_MRT_Interface mpcMrtInterface(mpc);
  mpcMrtInterface.initRollout(&interface.getRollout());

  /*
   * Initialize the simulation and controller
   */
  // initial state
  ocs2::SystemObservation initObservation;

  std::vector<double> initialState, initialTarget;
  nodeHandle.getParam("/initialTarget", initialTarget);

  // initial command
  ocs2::vector_t initTarget(7);
  if (initialTarget.size()){
    if (initialTarget.size() != 7){
      throw std::runtime_error("Initial target has wrong size");
    }
    for (int i = 0; i < initialTarget.size(); i++) {
      initTarget(i) = initialTarget[i];
    }
  } else {
    initTarget.head(3) << 0, 1, 1;
    initTarget.tail(4) << Eigen::Quaternion<ocs2::scalar_t>(1, 0, 0, 0).coeffs();
  }
  std::cerr << "initialTarget: " << initTarget << std::endl;
  const ocs2::vector_t zeroInput = ocs2::vector_t::Zero(interface.getManipulatorModelInfo().inputDim);

  tf::TransformListener listener;
  ros::Subscriber joint_states_sub;

  ROS_INFO("initialising robot_interface");
  PR2Interface robot_interface = PR2Interface(nodeHandle);
  ObservationCallback callback(interface.getInitialState().size(), interface.getManipulatorModelInfo().inputDim);
  if (world_type != "sim") {
    ROS_INFO("initialising gazebo stuff");
    joint_states_sub = nodeHandle.subscribe("/joint_states", 1, &ObservationCallback::getObservationCallback, &callback);
    // joint_states_sub = nodeHandle.subscribe<sensor_msgs::JointState>("/joint_states", 1, boost::bind(getObservationCallback, _1, &robot_interface.currentObservation_));
    // ros topic subscriptions
//    ros::SubscribeOptions getObservationSo = ros::SubscribeOptions::create<sensor_msgs::JointState>("/joint_states",
//                                                                                                    1,
//                                                                                                    &RobotInterface::getObservationCallback,
//                                                                                                    &robot_interface,
//                                                                                                    nodeHandle.getCallbackQueue());
//    // Because TCP causes bursty communication with high jitter,
//    // declare a preference on UDP connections for receiving
//    // joint states, which we want to get at a high rate.
//    // Note that we'll still accept TCP connections for this topic
//    // (e.g., from rospy nodes, which don't support UDP);
//    // we just prefer UDP.
//    getObservationSo.transport_hints = ros::TransportHints().unreliable();
//    ros::Subscriber subJointStates = nodeHandle.subscribe(getObservationSo);

    std::string error_msg;
    // sleep so tf transform doesn't fail
    sleep(1.0);
    if (!listener.waitForTransform("map", "base_footprint", ros::Time::now() + ros::Duration(0.5), ros::Duration(10.0), ros::Duration(0.01), &error_msg)) {
      throw std::runtime_error(error_msg.c_str());
    }
  }

  ROS_INFO("setting initial observation");
  if (world_type != "sim") {
    while (!callback.has_received_msg_){
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      std::cout << "waiting for first joint_states msg" << std::endl;
    }
    updateBaseTf(listener, callback.currentObservation_);
    initObservation = callback.currentObservation_;
    initObservation.time = ros::Time::now().toSec();
    std::cerr << "initial time: " << initObservation.time << std::endl;
  } else {
    nodeHandle.getParam("/initialState", initialState);
    if (initialState.size()){
      if (initialState.size() != interface.getInitialState().size()){
        throw std::runtime_error("Initial state has wrong size");
      }
      ocs2::vector_t initState(initialState.size());
      for (int i = 0; i < initialState.size(); i++) {
        initState(i) = initialState[i];
      }
      initObservation.state = initState;
    } else {
      initObservation.state = interface.getInitialState();
    };
    std::cerr << "initialState: " << initObservation.state << std::endl;
    initObservation.input.setZero(interface.getManipulatorModelInfo().inputDim);
    initObservation.time = 0.0;
  }
  const ocs2::TargetTrajectories initTargetTrajectories({initObservation.time}, {initTarget}, {zeroInput});
  ocs2::SystemObservation currentObservation = initObservation;


  // Set the first observation and command and wait for optimization to finish
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  mpcMrtInterface.setCurrentObservation(initObservation);
  mpcMrtInterface.getReferenceManager().setTargetTrajectories(initTargetTrajectories);
  while (!mpcMrtInterface.initialPolicyReceived() && ros::ok() && ros::master::check()) {
    mpcMrtInterface.advanceMpc();
    ros::WallRate(interface.mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  /*
   * Launch the computation of the MPC in a separate thread.
   * This thread will be triggered at a given frequency and execute an optimization based on the latest available observation.
   */
  std::atomic_bool mpcRunning{true};
  auto mpcThread = std::thread([&]() {
      while (mpcRunning) {
        try {
          ocs2::executeAndSleep([&]() { mpcMrtInterface.advanceMpc(); }, interface.mpcSettings().mpcDesiredFrequency_);
        } catch (const std::exception& e) {
          mpcRunning = false;
          ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        }
      }
  });
  ocs2::setThreadPriority(interface.ddpSettings().threadPriority_, mpcThread);

  /*
   * Main control loop.
   */
  while (mpcRunning && ros::ok()) {
    ocs2::executeAndSleep(  // timed execution of the control loop
            [&]() {
                /*
                 * State estimation would go here to fill "currentObservation".
                 * In this example we receive the measurement directly after forward integration at the end of the loop.
                 */
                if (world_type == "gazebo") {
                  currentObservation.state = callback.currentObservation_.state;
                  currentObservation.time = callback.currentObservation_.time;
                  // NOTE: keep this order, to not overwrite the base tf with callback.currentObservation_.state again
                  updateBaseTf(listener, currentObservation);
                  // std::cout << "base_pose: " << currentObservation.state[0] << ", " << currentObservation.state[1] << ", " << currentObservation.state[2] << std::endl;
                  // std::cout << "joints: " << currentObservation.state[3] << ", " << currentObservation.state[4] << ", " << currentObservation.state[5] <<  ", " << currentObservation.state[6] << ", " << currentObservation.state[7]<< ", " << currentObservation.state[8] << ", " << currentObservation.state[9] << ", " << currentObservation.state[10] << std::endl;
                }
                ROS_INFO_THROTTLE(1.0, "### Current time %f", currentObservation.time);

                // Evaluate the control input
                const auto systemInput = mpcTrackingController(currentObservation, mpcMrtInterface);

                /*
                 * Sending the commands to the actuators would go here.
                 * In this example, we instead do a forward simulation + visualization.
                 * Simulation is done with the rollout functionality of the mpcMrtInterface, but this can be replaced by any other simulation.
                 */
                const auto dt = 1.0 / interface.mpcSettings().mrtDesiredFrequency_;

                if (world_type == "gazebo") {
                  tf::Transform base_tf;
                  base_tf.setOrigin(tf::Vector3(currentObservation.state[0], currentObservation.state[1], 0.0));
                  base_tf.setRotation(tf::Quaternion(currentObservation.state[2], 0.0, 0.0));
                  robot_interface.sendBaseCommand(systemInput, base_tf);
                  robot_interface.sendArmCommands(systemInput, 0.01);
                } else {
                  // Forward simulation
                  ocs2::SystemObservation nextObservation;
                  nextObservation.time = currentObservation.time + dt;
                  mpcMrtInterface.rolloutPolicy(currentObservation.time, currentObservation.state, dt, nextObservation.state, nextObservation.input, nextObservation.mode);

                  // "state estimation"
                  // std::cout << "systemInput: " << systemInput << std::endl;
                  // std::cout << "nextObservation: " << nextObservation << std::endl;

                  currentObservation = nextObservation;
                  // Visualization
                  mobileManipulatorDummyVisualization.update(currentObservation, mpcMrtInterface.getPolicy(), mpcMrtInterface.getCommand());
                }

                // Publish the observation. Only needed for the command interface
                observationPublisher.publish(ocs2::ros_msg_conversions::createObservationMsg(currentObservation));

                ros::spinOnce();
            },
            interface.mpcSettings().mrtDesiredFrequency_);
  }

  // Shut down the MPC thread.
  mpcRunning = false;
  if (mpcThread.joinable()) {
    mpcThread.join();
  }

  // Successful exit
  return 0;
}

ocs2::vector_t mpcTrackingController(const ocs2::SystemObservation& currentObservation, ocs2::MPC_MRT_Interface& mpcMrtInterface) {
  // Update the current state of the system
  mpcMrtInterface.setCurrentObservation(currentObservation);

  // Load the latest MPC policy
  bool policyUpdated = mpcMrtInterface.updatePolicy();
  if (policyUpdated) {
    // ROS_INFO_STREAM("<<< New MPC policy received at " << currentObservation.time);
  }

  // Evaluate the current policy
  ocs2::vector_t optimizedState;  // Evaluation of the optimized state trajectory.
  ocs2::vector_t optimizedInput;  // Evaluation of the optimized input trajectory.
  size_t plannedMode;             // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface.evaluatePolicy(currentObservation.time, currentObservation.state, optimizedState, optimizedInput, plannedMode);

  return optimizedInput;
}
