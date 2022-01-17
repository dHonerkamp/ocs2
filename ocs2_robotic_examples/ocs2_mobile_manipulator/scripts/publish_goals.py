import rospy
from ocs2_msgs.msg import mpc_target_trajectories, mpc_state, mpc_input, mpc_observation
import numpy as np


def get_time():
    # return get_time()
    mpc_obs = rospy.wait_for_message("/mobile_manipulator_mpc_observation", mpc_observation, timeout=1)
    return mpc_obs.time

def main():
    rospy.init_node('test', anonymous=False)
    pub = rospy.Publisher('/mobile_manipulator_mpc_target', mpc_target_trajectories)

    msg = mpc_target_trajectories()
    t = get_time()
    print(t)
    for i, x in enumerate(np.arange(0, 10, 0.5)):
        goal = mpc_state()
        goal.value = [np.cos(x), np.sin(x), (1.2 + np.sin(x)) / 3.] + [0., 0., 0., 1.]
        msg.stateTrajectory.append(goal)
        inpt = mpc_input()
        inpt.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.inputTrajectory.append(inpt)
        msg.timeTrajectory.append(t + 0.3 + i)

    while get_time() < msg.timeTrajectory[-1]:
        rospy.loginfo(msg.stateTrajectory)
        rospy.loginfo(msg.timeTrajectory)
        pub.publish(msg)
        rospy.sleep(0.1)

    # msg = mpc_target_trajectories()
    # t = get_time()
    # for i, x in enumerate([1, 2, -1, -2]):
    #     goal = mpc_state()
    #     goal.value = [x, 0, 0.5] + [0., 0., 0., 1.]
    #     msg.stateTrajectory.append(goal)
    #     inpt = mpc_input()
    #     inpt.value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #     msg.inputTrajectory.append(inpt)
    #     msg.timeTrajectory.append(t + 10*i)
    #
    # while not rospy.is_shutdown():
    #     rospy.loginfo(msg)
    #     pub.publish(msg)
    #     rospy.sleep(0.5)


if __name__ == '__main__':
    main()
