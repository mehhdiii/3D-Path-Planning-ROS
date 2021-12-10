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
    print('Translation from local drone to Global frame: ', tran)

    #Map and start goal points: 
    drone_pose, obstacles = get_gazebo_poses.listener()
    obstacles = np.array(obstacles.values())
    # obstacles = np.array(np.mat('6.94 -8.3388 1.179 1.15;    1.33833 -8.11676 1.1822 1.18;    -2.8122 -7.5615 0.4999 1.822;    -7.5147 -6.6254 0.4999 1.258;    -7.4843 -3.1835 0.4999 1.0709;    -7.436  0.46083 0.4999 1.343;    -7.4736 4.1776  0.4999 1.3075;    -6.815 7.5658 0.5 1.054;    -3.9919 7.606 1.332 1.332;    -2.0723 7.6714 0.5 0.855;    2.433684 7.835614 1.05296 1.178;    4.5503 7.7688 0.5 0.735;    7.9723 4.6757 0.5 0.5;    7.04868 1.494 0.5 1.489;    7.9899 -1.586 1.096 1.0934;    7.984 -2.948 0.5 0.5;    8.11811 -5.954 1.13 1.1305'))
    start = np.array(drone_pose)
    goal = np.array([9, 6, 3, 0])
    goals = potential_field(obstacles, start, goal)
    
    

    i = 0
    while i < len(goals):
        drone.set_destination(
            x=goals[i][0]+tran[0], y=goals[i][1]+tran[1], z=goals[i][2]+tran[2], psi=goals[i][3])
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

 