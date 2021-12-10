#! /usr/bin/env python
# Import ROS.
import rospy
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *
from potential_field import potential_field
from gazebo_tf import trans_matrix
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import get_gazebo_poses


def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)

    # Create an object for the API.
    drone = gnc_api()
    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.wait4start()

    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 3m.
    drone.takeoff(3)
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)

    #Global transformation from local coordinate frame: 
    tran, rot = trans_matrix('global_frame', 'drone')
    rot = np.array(np.matrix('0 -1 0; 1 0 0; 0 0 1'))
    print('Translation from local drone to Global frame: ', tran)

    #Map and start goal points: 
    drone_pose, obstacles = get_gazebo_poses.listener()
    print('osbtacle information: \n \n', obstacles)
    drone_pose = [drone_pose[0], drone_pose[1], 2, 0]
    
    tran = drone_pose
    print('drone_pose:', drone_pose)
    obstacles = np.array(obstacles.values())
    
    # obstacles = np.array(np.mat('6.94 -8.3388 1.179 1.15;    1.33833 -8.11676 1.1822 1.18;    -2.8122 -7.5615 0.4999 1.822;    -7.5147 -6.6254 0.4999 1.258;    -7.4843 -3.1835 0.4999 1.0709;    -7.436  0.46083 0.4999 1.343;    -7.4736 4.1776  0.4999 1.3075;    -6.815 7.5658 0.5 1.054;    -3.9919 7.606 1.332 1.332;    -2.0723 7.6714 0.5 0.855;    2.433684 7.835614 1.05296 1.178;    4.5503 7.7688 0.5 0.735;    7.9723 4.6757 0.5 0.5;    7.04868 1.494 0.5 1.489;    7.9899 -1.586 1.096 1.0934;    7.984 -2.948 0.5 0.5;    8.11811 -5.954 1.13 1.1305'))
    start = np.array(drone_pose)
    goal = np.array([9.0,6.0,2.0, 0.0])
    waypoints = potential_field(obstacles, start, goal)
    plt.axes(projection='3d')
    plt.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:, 3])
    plt.show()
    # time.sleep(10)
    

    i = 0
    while i < len(waypoints):
        temp = waypoints[i, 0:3]
        temp = np.array([temp[0]-tran[0], temp[1] -tran[1], temp[2]])
        temp = np.matmul(rot, temp)
        print(temp)
        drone.set_destination(
            x=temp[0], y=temp[1], z=temp[2], psi=waypoints[i][3])
            
        rate.sleep()
        if drone.check_waypoint_reached(0.3):
            i += 1
    # Land after all waypoints is reached.
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()

 