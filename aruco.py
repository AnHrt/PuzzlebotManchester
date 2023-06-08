#!/usr/bin/env python

# Import necessary libraries
import rospy
import tf2_ros
import numpy as np
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray, Float32

class Aruco:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.error = 0.0
        self.marker_detected = False
        self.marker_ids = [701, 702, 703, 704, 705, 706, 706]  # Dictionary to store marker transforms
        self.marker_x   = [-0.693, 2.248, 1.651, 4.695, 2.889, -1.0, -1.0]
        self.marker_y   = [6.850, 10.846, 5.864, 7.21,  1.289, 0.0, 0.0]
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()  # Broadcaster to publish the transform
        self.X_aruco_pub = rospy.Publisher("/X_aruco", Float32, queue_size=1)
        self.Y_aruco_pub = rospy.Publisher("/Y_aruco", Float32, queue_size=1)
        self.Rk = rospy.Publisher("/Rk", Float32, queue_size=1)
        self.z_publisher = rospy.Publisher("/aruco_z", Float64MultiArray, queue_size=10)  # Publisher for z variable
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_transforms_cb)
        self.rate = rospy.Rate(1)  # 1Hz

        while not rospy.is_shutdown():

            if self.marker_detected:
                # If any markers are detected, call function to publish transforms and z variables
                marker_transforms = self.marker_transforms
                #self.publish_transforms(marker_transforms)
                self.publish_z(marker_transforms)
                #print("Markers detected")
                self.marker_transforms = [None, None, None,None, None, None]
                self.marker_detected = False

            else:
                # If not, indicate that no markers are being detected
                print("No markers detected")
            self.rate.sleep()

    # Function to process fiducial transforms
    def fiducial_transforms_cb(self, msg):
        self.marker_transforms = [None, None, None,None, None, None]  # Clear the existing marker transforms

        for transform in msg.transforms:

            marker_id = transform.fiducial_id
            #print("Marker ID: ", marker_id)
            marker_transform = transform.transform
            self.error = transform.object_error

            self.marker_transforms.insert((marker_id%10)-1, marker_transform)
            self.marker_detected = True

    # Function to publish the marker transforms and print marker information
    def publish_transforms(self, marker_transforms):
        for marker in marker_transforms:

            if marker != None:
                index = marker_transforms.index(marker) + 1

                marker_id = "70" + str(index)
                transform_stamped = TransformStamped()
                transform_stamped.header.stamp = rospy.Time.now()
                transform_stamped.header.frame_id = "camera"
             
                transform_stamped.child_frame_id = "marker_" + str(marker_id)
                transform_stamped.transform = marker
                self.tf_broadcaster.sendTransform(transform_stamped)

                position = marker.translation
                orientation = marker.rotation
                euler_angles = euler_from_quaternion(
                    (orientation.x, orientation.y, orientation.z, orientation.w))
                
                # Print marker information
                #print("Marker ID:", marker_id)
                #print("Position (x):", position.x)
                #print("Position (y):", position.y)
                #print("Position (z):", position.z)
                #print("Orientation (roll):", euler_angles[0])
                #print("Orientation (pitch): ", euler_angles[1])
                #print("Orientation (yaw): ", euler_angles[2])
                #print("")

    # Function to publish the z variables
    def publish_z(self, marker_transforms):

        e_menor = 100000000

        for marker in marker_transforms:
            if marker != None:
                position = marker.translation
                print(marker_transforms)
                index = marker_transforms.index(marker) 

                if((index >= 0) and (index <= 5)):

                    delta_x = 0.08

                    X_aruco = position.z + delta_x
                    Y_aruco = -(position.x)

                    #print("Position.z: ",position.z)
                    #print("Position.x: ", position.x)
                    #print("X_aruco: ", X_aruco)
                    #print("Y_aruco: ", Y_aruco)

                    e_square = (X_aruco ** 2) + (Y_aruco ** 2)
                    e = np.sqrt(e_square)
                    print("e: ", e)

                    if e < e_menor:
                        theta = np.arctan2(Y_aruco, X_aruco)
                        z = np.array([e, theta])                

                        mx = self.marker_x[index]
                        my = self.marker_y[index]
                        e_menor = e

                    z_msg = Float64MultiArray()
                    z_msg.data = z
                    self.z_publisher.publish(z_msg)
                    self.X_aruco_pub.publish(mx)
                    self.Y_aruco_pub.publish(my)
                    print("Index: ", index)
                    print("Aruco ID: ", self.marker_ids[index])
                    print("z: ", z)
                    print("X: ", mx)
                    print("Y: ", my)
                    self.Rk.publish(self.error)
            
    # Cleanup function to be called on shutdown
    def cleanup(self):
        print("Shutting down...")

if __name__ == "__main__":
    # Node initialization
    rospy.init_node("aruco_transform_publisher", anonymous=True)
    print("Aruco node initialized")
    try:
        Aruco()
    except rospy.ROSInterruptException:
        pass

