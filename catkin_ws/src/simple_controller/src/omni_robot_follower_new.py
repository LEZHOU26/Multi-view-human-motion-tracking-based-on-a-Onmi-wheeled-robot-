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
x1 = 0.0
y1 = 0.0
theta1 = 0.0
def new_Odom(data):
    global x1
    global y1
    global theta1

    x1 = data.pose.pose.position.x
    y1 = data.pose.pose.position.y

    rot_q1 = data.pose.pose.orientation
    (roll, pitch, theta1) = euler_from_quaternion(
        [rot_q1.x, rot_q1.y, rot_q1.z, rot_q1.w])


rospy.init_node("speed_controller")

sub = rospy.Subscriber("/target/odom", Odometry, newOdom)
sub_1 = rospy.Subscriber("/follower/odom", Odometry, new_Odom)
pub = rospy.Publisher("/target/cmd_vel",Twist,queue_size=1)

speed = Twist()
#speed = geometry_msgs.msg.Twist()
r = rospy.Rate(10)

goal = Point()
goal.x = 0
goal.y = 0
goal_x_coordinates = [3, 6, 10.0]
goal_y_coordinates = [4.0, 7.0, 9]
i = 0
while not rospy.is_shutdown():
    # goal_x_coordinates = [0.1, 3.0, 0.0, -1.5, -1.5,  4.5, 0.0]
    '''
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
    '''
    if 0 < x < 5.0 and y < 3.0:
        speed.linear.x = 0.2
        speed.angular.z = 0.0
        #speed.linear.y = 0.5
    elif x >= 5.0 and y < 2.0:
        speed.linear.x = 0.1
        speed.angular.z = 0.15

    elif x < 5.0 and y >= 3.0:
        speed.linear.x = 0.2
        speed.angular.z = 0.0

    elif x <= 0.0:
        speed.linear.x = 0
        speed.angular.z = 0
    with open('/home/user/postion.txt', 'a+') as f:
            f.write(str(x) + "," + str(y) + "," + str(theta) + "," + str(x1) + "," + str(y1) + "," + str(theta1))
            f.write('\n')
    pub.publish(speed)
    r.sleep()
    rospy.exceptions.ROSInterruptException("ROS shutdown request")
