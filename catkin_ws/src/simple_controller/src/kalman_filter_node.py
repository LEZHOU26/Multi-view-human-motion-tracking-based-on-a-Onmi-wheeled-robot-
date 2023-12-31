#!/usr/bin/env python
import rospy 
import numpy as np
import scipy.stats 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from ieu_agv.msg import uwb_data
import math
import tf 
import time



global mu 
mu = [0,0, 0.0]
global sigma 
sigma = np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]])
sensor_pos=[]

rospy.init_node('kalman_filter', anonymous=True)
pub = rospy.Publisher('filtrelenmis_data', Pose, queue_size=10)
r = rospy.Rate(1)


def prediction_step(Odometry):
    global mu
    global sigma
    # sigma: 3x3 covariance matrix of belief distribution 
    
    x = mu[0]
    y = mu[1]
    theta = mu[2]

    delta_vel = Odometry.twist.twist.linear.x *1000          
    delta_w = Odometry.twist.twist.angular.z                  
    
    noise = 0.1**2
    v_noise = delta_vel**2
    w_noise = delta_w**2

    sigma_u = np.array([[noise + v_noise, 0.0],[0.0, noise + w_noise]])
    
    B = np.array([[np.cos(theta), 0.0],[np.sin(theta), 0.0],[0.0, 1.0]])

    x_new = x + delta_vel*np.cos(theta)/30
    y_new = y + delta_vel*np.sin(theta)/30
    theta_new = theta + delta_w/30
    
    G = np.array([[1.0, 0.0, -delta_vel * np.sin(theta)],
                    [0.0, 1.0, delta_vel * np.cos(theta)],
                    [0.0, 0.0, 1.0]])
    

    
    #new mu and sigma
    mu = [x_new, y_new, theta_new]
    sigma = np.dot(np.dot(G, sigma), np.transpose(G)) + np.dot(np.dot(B, sigma_u), np.transpose(B))

    #publish data 
    publish_data(mu[0],mu[1])
    
    return mu,sigma


def correction_step(uwb_data,  sensor_pos):
    global mu
    global sigma
    # sigma: 3x3 covariance matrix of belief distribution 

    x = mu[0]
    y = mu[1]
    z = 0 
    theta = mu[2]
    ids = uwb_data.destination_id
    ranges =uwb_data.distance
    
    H = []
    Z = []
    expected_ranges = []
    for i in range(len(ids)):
        lm_id = ids[i]
        meas_range = ranges[i]
        lx = sensor_pos[i][0]
        ly = sensor_pos[i][1]
        lz = sensor_pos[i][2]
        #calculate 
        range_exp = np.sqrt( (lx - x)**2 + (ly - y)**2+(lz - z)**2 )
        #compute a row of H
        H_i = [(x - lx)/range_exp, (y - ly)/range_exp, 0]
        H.append(H_i)
        Z.append(ranges[i])
        expected_ranges.append(range_exp)
    # noise covariance for the measurements
    R = 0.5 * np.eye(len(ids))
    # Kalman gain
    K_help = np.linalg.inv(np.dot(np.dot(H, sigma), np.transpose(H)) + R)
    K = np.dot(np.dot(sigma, np.transpose(H)), K_help) #gain
    # Kalman covariance
    mu = mu + np.dot(K, (np.array(Z) - np.array(expected_ranges)))
    sigma = np.dot(np.eye(len(sigma)) - np.dot(K, H), sigma)

    return mu,sigma


def subscribe_odom_data(Odometry):
    [mu,sigma]=prediction_step(Odometry)
    

def subscribe_uwb_data(uwb_data):
    [mu,sigma]=correction_step(uwb_data,  sensor_pos)


def publish_data(pose_x,pose_y):
    robot_pos = Pose()
    robot_pos.position.x = float(pose_x)
    robot_pos.position.y = float(pose_y)
    robot_pos.position.z = 0.0

    robot_pos.orientation.x = 0.0
    robot_pos.orientation.x = 0.0
    robot_pos.orientation.x = 0.0
    robot_pos.orientation.w = 0.0
    pub.publish(robot_pos)


def get_anchors_pos():
    max_anchor = 100
    sensor_pos = []   
    uwb_id = 'uwb_anchor_'
    listener = tf.TransformListener()
    
    for i in range(max_anchor):
        try:
            time.sleep(0.3)
            (trans,rot) = listener.lookupTransform('/map', uwb_id+str(i), rospy.Time(0))
            sensor_pos.append(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            break

    sensor_pos = np.dot(sensor_pos,1000)


    return sensor_pos

if __name__ == "__main__":
    sensor_pos = get_anchors_pos()
    
    rospy.Subscriber("odometry", Odometry, subscribe_odom_data)

    rospy.spin()