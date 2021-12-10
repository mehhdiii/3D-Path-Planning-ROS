#!/usr/bin/env python  
import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

def trans_matrix(node1, node2):
    # rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform(node1, node2, rospy.Time(0))
            # print(trans)
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
    return trans, rot