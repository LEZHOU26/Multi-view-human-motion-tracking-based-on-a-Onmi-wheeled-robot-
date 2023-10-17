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
def newOdom(msg):
    global x1
    global y1
    global theta1

    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta1) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

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
                #pdb.set_trace()
                #print(trans[1])
                #print(trans[0])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            '''
            if trans[0]>1:
                 angular = 4 * math.atan2(trans[1], trans[0])
                 linear = 0.5 * (math.sqrt(trans[0] ** 2 + trans[1] ** 2)-1)
                 cmd = geometry_msgs.msg.Twist()
                 #cmd.linear.x = linear
                 cmd.angular.z = angular
                 #cmd.linear.y = 1
                 cmd.linear.x = linear
            else: 
                 cmd.linear.x = 0
                 cmd.angular.z = 0
            turtle_vel.publish(cmd)
            rate.sleep()
            '''
            cmd = geometry_msgs.msg.Twist()
            if  trans[0] > 1:
                x_data1 = 1
            #elif trans[0] >= 1 and trans[0] <= 2.4:
             #    x_data1 = 0
            elif trans[0] < 1.0:
                 x_data1 = 0

            if trans[1] < -0.2:
               y_data = -1
            elif trans[1] > 0.2:
                y_data = 1
            elif trans[1] > -0.2 and trans[1] < 0.2:
                 y_data = 0

            if x_data1 == 0 and y_data == 1:
                current_time = time.time()
                dt = current_time - last_time
                setpoint_stop = 2
                kp = 1.8;ki = 0.006;kd = 0.01;    upper = 0.26; lower = 0.0
                cmd.linear.x = pid(trans[0],setpoint_stop,kp,ki,kd,dt,upper,lower)
                cmd.angular.z = 0.2
                last_time = current_time
                rospy.loginfo("Slow and Stop!!")
                rospy.loginfo("Current vel %s", cmd.linear.x)
                rospy.loginfo("dt %s", dt)
            else:
                if  x_data1 == 0 and y_data == -1:
                    current_time = time.time()
                    dt = current_time - last_time
                    setpoint_stop = 2
                    kp = 1.8;ki = 0.006;kd = 0.01;    upper = 0.26; lower = 0.0
                    cmd.linear.x = pid(trans[0],setpoint_stop,kp,ki,kd,dt,upper,lower)
                    cmd.angular.z = -0.2
                    last_time = current_time
                    rospy.loginfo("Slow and Stop!!")
                    rospy.loginfo("Current vel %s", cmd.linear.x)
                    rospy.loginfo("dt %s", dt)
                    
                elif x_data1 == 1 and y_data == 0:
                    current_time = time.time()
                    dt = current_time - last_time
                    setpoint_stop = 1
                    kp = 1;ki = 0.001;kd = 0.01;upper = 0.26; lower = 0.0
                    cmd.linear.x = pid(trans[0],setpoint_stop,kp,ki,kd,dt,upper,lower)
                    last_time = current_time
                    cmd.angular.z = 0.0
                    rospy.loginfo("Moving forward!!")
                    rospy.loginfo("Current vel %s", cmd.linear.x)
                elif x_data1 == 1 and y_data == 1:
                    cmd.angular.z = 0.2
                    current_time = time.time()
                    dt = current_time - last_time
                    setpoint_stop = 1
                    kp = 1;ki = 0.001;kd = 0.01;upper = 0.26; lower = 0.0
                    cmd.linear.x = pid(trans[0],setpoint_stop,kp,ki,kd,dt,upper,lower)
                    last_time = current_time
                    rospy.loginfo("Moving forward!!")
                    rospy.loginfo("Current vel %s", cmd.linear.x)
                elif x_data1 == 1 and y_data == -1:
                    cmd.angular.z = -0.2
                    current_time = time.time()
                    dt = current_time - last_time
                    setpoint_stop = 1
                    kp = 1;ki = 0.001;kd = 0.01;upper = 0.26; lower = 0.0
                    cmd.linear.x = pid(trans[0],setpoint_stop,kp,ki,kd,dt,upper,lower)
                    last_time = current_time
                    rospy.loginfo("Moving forward!!")
                    rospy.loginfo("Current vel %s", cmd.linear.x)
            turtle_vel.publish(cmd)
            rate.sleep()

