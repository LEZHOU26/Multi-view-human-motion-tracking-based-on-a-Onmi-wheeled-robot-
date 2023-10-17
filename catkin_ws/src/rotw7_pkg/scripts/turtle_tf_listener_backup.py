#!/usr/bin/env python
import sys
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import pdb

if __name__ == '__main__':
    rospy.init_node('tf_listener_turtle')

    listener = tf.TransformListener()

    if len(sys.argv) < 3:
        print("usage: turtle_tf_listener.py follower_model_name model_to_be_followed_name")
    else:
        follower_model_name = sys.argv[1]
        #pdb.set_trace()
        model_to_be_followed_name = sys.argv[2]
        #pdb.set_trace()

        turtle_vel = rospy.Publisher('/turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

        rate = rospy.Rate(10.0)
        ctrl_c = False

        follower_model_frame = follower_model_name
        model_to_be_followed_frame = model_to_be_followed_name

        def shutdownhook():
            # works better than the rospy.is_shut_down()
            global ctrl_c
            print ("shutdown time! Stop the robot")
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            cmd.linear.y = 1.0
            turtle_vel.publish(cmd)
            ctrl_c = True

        rospy.on_shutdown(shutdownhook)

        while not ctrl_c:
            try:
                (trans,rot) = listener.lookupTransform(follower_model_frame, model_to_be_followed_frame, rospy.Time(0))
                #pdb.set_trace()
                #print(trans[1])
                #print(trans[0])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            if trans[0]>2:
                 #angular = 4 * math.atan2(trans[1], trans[0])]
                 angular = 0
                 linear = 0.5 * (math.sqrt(trans[0] ** 2 + trans[1] ** 2)-2)
                 cmd = geometry_msgs.msg.Twist()
                 cmd.linear.x = linear
                 cmd.angular.z = angular
            else: 
                 cmd.linear.x = 0
                 cmd.angular.z = 0
            turtle_vel.publish(cmd)
            rate.sleep()