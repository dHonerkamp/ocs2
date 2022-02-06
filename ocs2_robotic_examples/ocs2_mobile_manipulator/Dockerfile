# system dependencies
sudo apt install libglpk-dev liburdfdom-dev liboctomap-dev libassimp-dev ros-melodic-grid-map-msgs

# workspace
mkdir -p ros/catkin_ws_articulated/src
cd ros/catkin_ws_articulated
catkin init
catkin config --extend /opt/ros/melodic
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo

git clone https://github.com/dHonerkamp/ocs2.git src/ocs2
git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git src/pinocchio
git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git src/hpp-fcl
# need the noetic version of this one
#git clone --branch noetic-devel git@github.com:ros/robot_state_publisher.git src/robot_state_publisher

# need cmake>=3.14
# NOTE: the following catkin build should then be run within this conda env!
conda create --name articulated python=3.8 cmake=3.14
conda activate articulated
pip install pyyaml roslibpy rospkg catkin_pkg empy numpy

catkin config -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/home/honerkam/miniconda3/envs/articulated/bin/python -DPYTHON_INCLUDE_DIR=/home/honerkam/miniconda3/envs/articulated/include/python3.8/ -DPYTHON_LIBRARY=/home/honerkam/miniconda3/envs/articulated/lib/libpython3.8.so
# ocs2_legged_robot may fail to build due to some anymal dependency
catkin config --blacklist ocs2_raisim ocs2_raisim_ros ocs2_doc ocs2_legged_robot

# add tiago meshes etc.
ln -s /home/honerkam/ros/catkin_ws_tiago2/src/pmb2_robot/pmb2_description/ src/
ln -s /home/honerkam/ros/catkin_ws_tiago2/src/tiago_robot/tiago_description/ src/
ln -s /home/honerkam/ros/catkin_ws_tiago2/src/tiago_description_calibration/ src/
ln -s /home/honerkam/ros/catkin_ws_tiago2/src/hey5_description/ src/
ln -s /home/honerkam/ros/catkin_ws_tiago2/src/pal_gripper/pal_gripper_description/ src/
ln -s /home/honerkam/ros/catkin_ws_tiago2/src/tiago_moveit_config/ src/

catkin build
source devel/setup.bash

# run tests
#catkin run_tests ocs2

# mobile manipulator example
catkin build ocs2_mobile_manipulator ocs2_mobile_manipulator_ros
source devel/setup.bash
roslaunch ocs2_mobile_manipulator_ros mobile_manipulator.launch


