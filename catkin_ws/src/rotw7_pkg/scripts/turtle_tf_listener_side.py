#!/usr/bin/env python
import sys
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import pdb
import numpy as np
import time
import threading
import math
import random
from math import atan2
global last_time
global x_data
global x_data1
global contents
start_time = time.time()
last_time = start_time
kp =  0.050 #0.026
kd =  0.0308 #0.0108
ki =  0.0
speedLinear = 0.0
lastError = 0.0
x1 = 0.0
y1 = 0.0
theta1 = 0.0
x2 = 0.0
y2 = 0.0
theta2 = 0.0
speed_x = 0
speed_z = 0
n_x =[]
n_y =[]
i = 0
n_y = np.loadtxt("/home/user/postion_sideview_kalman_pid.txt", delimiter=",")
#print(y)

n_x = np.loadtxt("/home/user/theta_sideview_kalman_pid.txt", delimiter=",")
def callback(data):
    global speed_x
    global speed_z
    global i
    speed_x = data.linear.x
    speed_z = data.angular.z
    i = i + 1

def newOdom(msg):
    global x1
    global y1
    global theta1

    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta1) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def callback1(position):
    global x2
    global y2
    global theta2

    x2 = position.pose.pose.position.x
    y2 = position.pose.pose.position.y

    rot_q2 = position.pose.pose.orientation
    (roll, pitch, theta2) = euler_from_quaternion(
        [rot_q2.x, rot_q2.y, rot_q2.z, rot_q2.w])

def pid(distance,setpoint,kp,ki,kd,dt,upper,lower):
    rospy.loginfo('PID start')
    global x_distance
    global error
    global motorSpeed
    global lastError
    x_distance = distance
    error = setpoint - x_distance
    # d_input = x_distance - (previous_dis if (previous_dis is not None) else x_distance)
    motorSpeed = (-kp*error) + (ki*error*dt) - (kd*(error - lastError)/dt)
    rospy.loginfo("motorSpeed %s",motorSpeed)
    # rospy.loginfo(motorSpeed)
    LinearSpeed = clamp(motorSpeed, lower, upper)
    rospy.loginfo("speedLinear %s", speedLinear)
    previous_dis = x_distance
    lastError = error
    # rospy.loginfo(speedLinear)
    return LinearSpeed

def clamp(value, lower, upper):
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value

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
        sub = rospy.Subscriber("turtle2/odom", Odometry, newOdom)
        sub_1 = rospy.Subscriber("turtle1/odom", Odometry, callback1)
        turtle_vel_arrow = rospy.Subscriber('/turtle1/cmd_vel', Twist, callback)
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
            cmd.linear.y = 0.0
            turtle_vel.publish(cmd)
            ctrl_c = True

        rospy.on_shutdown(shutdownhook)

        while not ctrl_c:
            try:
                (trans,rot) = listener.lookupTransform(follower_model_frame, model_to_be_followed_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            cmd = geometry_msgs.msg.Twist()
            inc_x= x2-x1 + n_x[i]
            inc_y = y2-y1 + n_y[i]
            #angle_to_goal = atan2(inc_y, inc_x)
            
            if  abs(inc_x) <0.1 and  0.95 < abs(inc_y) <1.05:
                cmd.linear.x = 0.0
                cmd.linear.y = -speed_x 
                cmd.angular.z = -0.0
            else:
                cmd.linear.x = 0.0
                cmd.linear.y = -speed_x*3 
                cmd.angular.z = 0.15/3+0.01
            q = [ rot[0],rot[1],rot[2],rot[3]]
            (r, p, theta_3) = tf.transformations.euler_from_quaternion(q)
            with open('/home/user/theta.txt', 'a+') as f:
                f.write(str(theta_3))
                f.write('\n')

            turtle_vel.publish(cmd)
            rate.sleep()

                

