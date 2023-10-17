#!/usr/bin/env python
"""
Author: Roberto Zegers R.
Date: August 2020
"""

import rospy
from std_msgs.msg import Float32
from math import atan2
import math
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import geometry_msgs.msg
from geometry_msgs.msg import Point, Twist

# Global variables
last_odom_pos = 2.0
# keep last measurement from laser ray
last_measurement = 0.0

### PASTE Initial state HERE ###
mu = 0.0
sig = 1000.0

# initialize motion sigma (the standard deviation of the motions normal distribution)
motion_sig = 4.0
measurement_sig = 0.05
# distance since last filter cycle
motion = 0.0

### PASTE measurement_update_step FUNCTION HERE ###
def measurement_update(mean1, var1, mean2, var2):
    global new_mean, new_var
    new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1 / (1 / var1 + 1 / var2)
    return new_mean, new_var

### PASTE state_prediction FUNCTION HERE ###
def state_prediction(mean1, var1, mean2, var2):
    global new_mean, new_var
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return new_mean, new_var

# Callback function to handle new messages received
def odom_callback(data):
    global mu, sig

    global motion_sig
    global last_odom_pos

    global measurement_sig
    global last_measurement

    global filter_pub

    global x
    global y
    global theta

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    rot_q =data.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    # distance since last filter cycle
    motion = x - last_odom_pos

    ### PASTE KALMAN FILTER CYCLE HERE ###
    mu, sig = state_prediction(mu, sig, motion, motion_sig)
    rospy.loginfo("predict: [%s, %s]", mu, sig)
    mu, sig = measurement_update(mu, sig, last_measurement, measurement_sig)
    rospy.loginfo("update: [%s, %s]", mu, sig)

    # keep for next filter cycle
    last_odom_pos = x

    # Publish filtered position estimate
    filter_pub.publish(mu)

def x_callback(msg):
    global last_measurement
    global x
    global y
    global theta
    # keep position as measured by laser ray
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    last_measurement = x


if __name__ == '__main__':
    try:
        # Initialize a ROS node
        rospy.init_node('kalman_filter')

        # Subscribe to the ROS topic called "/noisy_odom_x"
        rospy.Subscriber("/turtle1/odom", Odometry, odom_callback)

        # Subscribe to the ROS topic called "/position_from_laser_ray"
        rospy.Subscriber("/turtle1/odom", Odometry, x_callback)

        filter_pub = rospy.Publisher('/filtered_pos', Odometry, queue_size=1)

        # Create a new ROS rate limiting timer
        rate = rospy.Rate(5)

        # Print node start information
        print("-----------------------------------------------------\n")
        print("Started kalman filter node")
        print("-----------------------------------------------------\n")
          
        # Execute indefinitely until ROS tells the node to shut down.
        while not rospy.is_shutdown():

            # Sleep appropriately so as to limit the execution rate
            rate.sleep()

    except rospy.ROSInterruptException:
        pass