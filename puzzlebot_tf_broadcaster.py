#!/usr/bin/env python  

import rospy  
from geometry_msgs.msg import PoseStamped 
from visualization_msgs.msg import Marker 
from tf.transformations import quaternion_from_euler 
from std_msgs.msg import Float32 
from geometry_msgs.msg import PoseStamped 


# Because of transformations 
from geometry_msgs.msg import TransformStamped 
import tf2_ros 
import numpy as np 

 

class PuzzlebotTfClass():  

    def __init__(self):  
        rospy.Subscriber("pose_sim", PoseStamped, self.pose_sim_cb) 
        rospy.Subscriber('wr', Float32, self.wr_cb)
        rospy.Subscriber("wl", Float32, self.wl_cb)
        self.marker_pub = rospy.Publisher("puzzlebot_marker", Marker, queue_size = 1) 
        self.tf_br = tf2_ros.TransformBroadcaster() 
        self.robot_pose = PoseStamped() 
        self.robot_pose.pose.orientation.w = 1.0 #this is necessary to avoid errors with the quaternion. 

        self.theta_left = 0.0
        self.theta_right = 0.0
        self.time_act = 0.0
        self.time_ant = 0.0
        self.wl = 0.0
        self.wr = 0.0

        rate = rospy.Rate(50) # The rate of the while loop 

        while not rospy.is_shutdown(): 
            self.send_base_link_tf(self.robot_pose) 
            self.send_chassis_link_tf() 
            self.send_left_wheel_tf()
            self.send_right_wheel_tf()


            ######## Publish a marker to rviz ######### 
            marker = self.fill_marker(self.robot_pose) 
            self.marker_pub.publish(marker) 
            rate.sleep() 

    def pose_sim_cb(self, msg): 
        self.robot_pose = msg 

    def wr_cb(self, msg):
        self.wr = msg.data

    def wl_cb(self,msg):
        self.wl = msg.data
        print(self.wl)


    def send_left_wheel_tf(self, pose_stamped=PoseStamped()):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "/base_link"
        t.child_frame_id = "/left_wheel"

        #Copy data from the received pose to the tf  
        t.transform.translation.x = 0.05
        t.transform.translation.y = 0.09 #0.095
        t.transform.translation.z = 0.0 

        self.time_ant = self.time_act
        self.time_act = rospy.Time.now()
        #dt = self.time_act - self.time_ant

        theta_left = self.theta_left + self.wl * 0.1
        #print(theta_left)

        quat = quaternion_from_euler(0.0,theta_left,0.0)#(np.pi/2.0, 0.0, np.pi/2.0) 
        t.transform.rotation.x = quat[0] 
        t.transform.rotation.y = quat[1] 
        t.transform.rotation.z = quat[2] 
        t.transform.rotation.w = quat[3] 

        # Send the transformation 
        self.tf_br.sendTransform(t) 


    def send_right_wheel_tf(self, pose_stamped=PoseStamped()):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "/base_link"
        t.child_frame_id = "/right_wheel"

        #Copy data from the received pose to the tf  
        t.transform.translation.x = 0.05
        t.transform.translation.y = -0.09 #-0.095
        t.transform.translation.z = 0.0 

        theta_right = self.theta_right + self.wr * 0.01


        quat = quaternion_from_euler (0.0, theta_right, 0.0)#(np.pi/2.0, 0.0, np.pi/2.0) 
        t.transform.rotation.x = quat[0] 
        t.transform.rotation.y = quat[1] 
        t.transform.rotation.z = quat[2] 
        t.transform.rotation.w = quat[3] 

        # Send the transformation 
        self.tf_br.sendTransform(t) 



    def send_base_link_tf(self, pose_stamped=PoseStamped()): 
        # This receives the robot's pose and broadcast a transformation. 

        
        t = TransformStamped() 
        t.header.stamp = rospy.Time.now() 
        t.header.frame_id = "/odom" 
        t.child_frame_id = "/base_link" 

        #Copy data from the received pose to the tf  
        t.transform.translation.x = pose_stamped.pose.position.x 
        t.transform.translation.y = pose_stamped.pose.position.y 
        t.transform.translation.z = pose_stamped.pose.position.z 
        t.transform.rotation.x = pose_stamped.pose.orientation.x 
        t.transform.rotation.y = pose_stamped.pose.orientation.y 
        t.transform.rotation.z = pose_stamped.pose.orientation.z 
        t.transform.rotation.w = pose_stamped.pose.orientation.w 

        # Send the transformation 
        self.tf_br.sendTransform(t) 

 

    def send_chassis_link_tf(self): 
        t = TransformStamped() 
        t.header.stamp = rospy.Time.now() 
        t.header.frame_id = "/base_link" 
        t.child_frame_id = "/chassis" 

        #Copy data from the received pose to the tf  
        t.transform.translation.x = 0.0 
        t.transform.translation.y = 0.0 
        t.transform.translation.z = 0.0 

        quat = quaternion_from_euler(0.0, 0.0, 0.0) 
        t.transform.rotation.x = quat[0] 
        t.transform.rotation.y = quat[1] 
        t.transform.rotation.z = quat[2] 
        t.transform.rotation.w = quat[3] 

        # Send the transformation 
        self.tf_br.sendTransform(t) 

     

    def fill_marker(self, pose_stamped=PoseStamped()): 
        marker = Marker() 
        marker.header.frame_id = "/chassis" 
        marker.header.stamp = rospy.Time.now() 

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3; Mesh: 10 
        marker.type = 10 
        marker.id = 0 

 
        # Use the stl file MCR2_1000_13_Chassis.stl,  

        #marker.mesh_resource = "package://puzzlebot_sim/meshes/MCR2_1000_0_Puzzlebot.stl" 
        marker.mesh_resource = "package://puzzlebot_sim/meshes/MCR2_1000_13_Chassis.stl" 

        # Set the scale of the marker 
        marker.scale.x = 1 
        marker.scale.y = 1 
        marker.scale.z = 1 

 

        # Set the color 
        marker.color.r = 1.0 
        marker.color.g = 0.0 
        marker.color.b = 0.0 
        marker.color.a = 1.0 

 

        # Set the pose of the marker 

        marker.pose.position.x = 0.0 
        marker.pose.position.y = 0.0 
        marker.pose.position.z = 0.0 


        # Set the marker orientation 
        marker.pose.orientation.x = 0.0 
        marker.pose.orientation.y = 0.0 
        marker.pose.orientation.z = 0.0 
        marker.pose.orientation.w = 1.0 
        return marker 

 
############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    # first thing, init a node! 
    rospy.init_node('puzzlebot_tf_broadcaster')  
    PuzzlebotTfClass()