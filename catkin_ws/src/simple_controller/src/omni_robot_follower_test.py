#!/usr/bin/env python
from math import atan2
import math
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import geometry_msgs.msg
from geometry_msgs.msg import Point, Twist
import pdb

x = 0.0
y = 0.0
theta = 0.0


def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])


rospy.init_node("speed_controller")

sub = rospy.Subscriber("/turtle1/odom", Odometry, newOdom)
pub = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=1)

speed = Twist()
#speed = geometry_msgs.msg.Twist()
r = rospy.Rate(20)

goal = Point()
goal.x = 0
goal.y = 0
goal_x_coordinates = [3, 6, 10.0]
goal_y_coordinates = [4.0, 7.0, 9]
i = 0
while not rospy.is_shutdown():
    # goal_x_coordinates = [0.1, 3.0, 0.0, -1.5, -1.5,  4.5, 0.0]
    goal.x = goal_x_coordinates[i]
    goal.y = goal_y_coordinates[i]
    inc_x = goal.x - x
    inc_y = goal.y - y
    dis = math.sqrt(inc_x**2 + inc_y**2)
    

    angle_to_goal = atan2(inc_y, inc_x)
    speed.linear.y = 0.3
    #pdb.set_trace()
    print(angle_to_goal)
    print(theta)
    print(inc_x)
    print(inc_y)
    if abs(angle_to_goal - theta) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.7
        #speed.linear.y = 0.5
    else:
        speed.linear.x = 0.1
        speed.angular.z = 0.0
    
    if dis < 0.05:
        i = i+1

    if i == 7:
        speed.linear.x = 0.0
        speed.angular.z = 0.0

    if i == 8:
        i = 0.0


    pub.publish(speed)
    r.sleep()
    rospy.exceptions.ROSInterruptException("ROS shutdown request")
