#!/usr/bin/env python
import rospy
import tf
from turtle_tf_3d.get_model_gazebo_pose import GazeboModel
import time
i = 0
n_x = []
n_y = []
with open('/home/user/catkin_ws/src/rotw7_pkg/scripts/noise_x.txt', 'r') as f:
        data = f.readlines() #this returns data as a string of numbers
        for num_char in data:
            n_x.append(float(num_char))

with open('/home/user/catkin_ws/src/rotw7_pkg/scripts/noise_y.txt', 'r') as f:
        data = f.readlines() #this returns data as a string of numbers
        for num_char in data:
            n_y.append(float(num_char))

def handle_turtle_pose(pose_msg, robot_name,noise_x,noise_y):
    global i
    br = tf.TransformBroadcaster()
    
    br.sendTransform((pose_msg.position.x+ noise_x[i],pose_msg.position.y+ noise_y[i],pose_msg.position.z),
                     (pose_msg.orientation.x,pose_msg.orientation.y,pose_msg.orientation.z,pose_msg.orientation.w),
                     rospy.Time.now(),
                     robot_name,
                     "/turtle")
    i = i+1
def publisher_of_tf():

    rospy.init_node('publisher_of_tf_node', anonymous=True)
    robot_name_list = ["turtle2","turtle1"]
    gazebo_model_object = GazeboModel(robot_name_list)


    for robot_name in robot_name_list:
        pose_now = gazebo_model_object.get_model_pose(robot_name)

    # Leave time enough to be sure the Gazebo Model data is initalised
    time.sleep(1)
    rospy.loginfo("Ready..Starting to Publish TF data now...")

    rate = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        for robot_name in robot_name_list:
            pose_now = gazebo_model_object.get_model_pose(robot_name)
            if not pose_now:
                print ("The Pose is not yet"+str(robot_name)+" available...Please try again later")
            else:
                handle_turtle_pose(pose_now, robot_name,n_x, n_y)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher_of_tf()
    except rospy.ROSInterruptException:
        pass