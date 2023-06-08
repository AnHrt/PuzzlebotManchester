#!/usr/bin/env python  

import rospy  
import numpy as np 
from std_msgs.msg import Float32, Float64MultiArray
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseStamped 
from visualization_msgs.msg import Marker


from tf.transformations import quaternion_from_euler 

np.set_printoptions(suppress=True) 
np.set_printoptions(formatter={'float': '{: 0.4f}'.format}) 
 
class EKFClass():  
    def __init__(self):  
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1) 
        self.sk_pub = rospy.Publisher("sk", Float64MultiArray, queue_size=1) 
        self.pose_sim_pub = rospy.Publisher('pose_sim', PoseStamped ,queue_size=1) #Publisher to pose_sim topic
        self.marker_pub = rospy.Publisher("/robot_marker", Marker, queue_size = 1)



        rospy.Subscriber("wl", Float32, self.wl_cb ) 
        rospy.Subscriber("wr", Float32, self.wr_cb )
        rospy.Subscriber("/X_aruco", Float32, self.X_aruco_cb)
        rospy.Subscriber("/Y_aruco", Float32, self.Y_aruco_cb)
        rospy.Subscriber("/aruco_z", Float64MultiArray, self.aruco_z_cb)
        rospy.Subscriber("/Rk", Float32, self.Rk_cb) 

        odom = Odometry() 

        #Robot constants  
        self.r = 0.05 #[m] radius of the wheels 
        self.L = 0.18 #[m] distance between wheels     

        #Robot state 
        self.wl = 0.0 #Robot left wheel angular speed 
        self.wr = 0.0 #Robot right wheel angular speed 
        x = 0.0 #Robot x-axis postion 
        y = 0.0 #Robot position 
        self.theta = 0.0 # Robot orientation 
        dt = 0.0 #time interval  
        v = 0.0 #[m/s] Robot linear speed 
        w = 0.0 #[rad/s] Robot angular speed         

        #aruco marker
        self.mx = 0 #aruco position 
        self.my = 0 
        self.Rk = 0 
        self.z = np.zeros([2,1])

        sk = np.zeros([3,1]) #Posicion real
        #sk_est = np.zeros([3,1]) #Posicion del sensor
        miu_k = np.zeros([3,1]) #Posicion corregida (update kf)




        sk_ant = np.zeros([3,1]) #Posicion real anterior
        sk_ant[0,0] = 0.0
        sk_ant[1,0] = 0.0
        sk_ant[2,0] = 0.0







        Sigma_pose_Ant = np.zeros([3,3])
        Sigma_pose_est = np.zeros([3,3])
        Sigma_pose = np.zeros([3,3])





        while rospy.get_time() == 0: 
            print("no simulated time has been received yet") 
        print("Got time") 

        last_time = rospy.get_time() 
        self.received_wl = 0 
        self.received_wr = 0 
        self.received_aruco = 0
        rate = rospy.Rate(10) # The rate of the while loop 

        while not rospy.is_shutdown(): 
            if self.received_wl and self.received_wr: 
                #Define the sampling time  
                current_time = rospy.get_time() 
                dt = current_time-last_time 
                #dt = 0.03

                #Get a copy of wr an wl to get unexpected changes 
                wr = self.wr 
                wl = self.wl 

                #Compute the robot linear and angular speeds.  
                v = self.r*(wr+wl)/2.0 #Robot's linear speed 
                w = self.r*(wr-wl)/self.L #Robot's Angular Speed 


                #miu_k_est, Sigma_pose_est = self.predict(sk_ant, Sigma_pose_Ant, v, w, wr, wl, dt)


                if self.received_aruco == 1:
                    #print("update")
                    Rk = np.zeros([2,2])
                    Rk[0,0] = self.Rk
                    Rk[1,0] = 0
                    Rk[0,1] = 0
                    Rk[1,1] = self.Rk
                    z = self.z
                    #print("miu_k before: ", miu_k_est)
                    miu_k, Sigma_pose = self.update(miu_k_est, Sigma_pose_est, z, Rk)
                    #print("miu_k after: ", miu_k)
                    Sigma_pose_Ant = Sigma_pose

                    sk = miu_k
                    print("update: ", sk)

                else:

                    miu_k_est, Sigma_pose_est = self.predict(sk_ant, Sigma_pose_Ant, v, w, wr, wl, dt)
                    sk = miu_k_est
                    Sigma_pose_Ant = Sigma_pose_est
                    print("predict: ", sk)

                sk_ant = sk

                sk_msg = Float64MultiArray()
                sk_msg.data = sk
                    
                self.sk_pub.publish(sk_msg)  

                #print("sk: ", sk)

                #print("Position real [x,y,theta]", sk)
                #print("Siga Pose", Sigma_pose_Ant)
                #print("Sigma_Pose: ", Sigma_pose_Ant)
                
                odom = self.fill_odom(sk[0][0], sk[1][0], sk[2][0], Sigma_pose_Ant, v, w) 
                pose_stamped = self.get_pose_stamped(sk[0][0], sk[1][0], sk[2][0])
                marker = self.fill_marker(pose_stamped)


                self.pose_sim_pub.publish(pose_stamped)
                self.marker_pub.publish(marker)
                self.odom_pub.publish(odom) 
                last_time = current_time 

                rate.sleep() 



    def fill_marker(self, pose_stamped=PoseStamped()):
        # This function will fill the necessary data tu publish a marker to rviz.  
        # It receives pose_stamped which must be a [geometry_msgs/PoseStamped] message.  

        marker = Marker()
        marker.header.frame_id = pose_stamped.header.frame_id
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3; mesh_resource: 10
        marker.type = 10
        marker.id = 0
        marker.mesh_resource = "file:///home/arista/ws/src/puzzlebot_sim/meshes/MCR2_1000_0_Puzzlebot.stl"
        

        # Set the scale of the marker
        marker.scale.x = 0.2
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

 
        # Set the pose of the marker
        marker.pose.position.x = pose_stamped.pose.position.x
        marker.pose.position.y = pose_stamped.pose.position.y
        marker.pose.position.z = 0
        marker.pose.orientation.x = pose_stamped.pose.orientation.x
        marker.pose.orientation.y = pose_stamped.pose.orientation.y
        marker.pose.orientation.z = pose_stamped.pose.orientation.z
        marker.pose.orientation.w = pose_stamped.pose.orientation.w

        return marker



    def get_pose_stamped(self, x, y, yaw): 

        # x, y and yaw are the robot's position (x,y) and orientation (yaw) 
        # Write the data as a ROS PoseStamped message 

        pose_stamped = PoseStamped() 
        pose_stamped.header.frame_id = "/odom"    #This can be changed in this case I'm using a frame called odom. 
        pose_stamped.header.stamp = rospy.Time.now() 

        # Position 
        pose_stamped.pose.position.x = x 
        pose_stamped.pose.position.y = y 

        # Rotation of the mobile base frame w.r.t. "map" frame as a quaternion 
        quat = quaternion_from_euler(0,0,yaw) 
        pose_stamped.pose.orientation.x = quat[0] 
        pose_stamped.pose.orientation.y = quat[1] 
        pose_stamped.pose.orientation.z = quat[2] 
        pose_stamped.pose.orientation.w = quat[3] 
        return pose_stamped 


    def predict(self, sk_ant, Sigma_pose_Ant, v, w, wr, wl, dt):
        #Pose estimation (x,y,theta)   --->  h
        miu_k_est = np.zeros([3,1]) #Posicion estimada (prediccion kf)

        miu_k_est[0,0] = sk_ant[0,0] + dt * v*np.cos(sk_ant[2,0])
        miu_k_est[1,0] = sk_ant[1,0] + dt * v*np.sin(sk_ant[2,0]) 
        miu_k_est[2,0] = sk_ant[2,0] + dt * w
        miu_k_est[2,0] = np.arctan2(np.sin(miu_k_est[2,0]), np.cos(miu_k_est[2,0]))

        # Calculate 
        Hk = np.zeros([3,3])
        Hk[0,0] = 1.0
        Hk[0,1] = 0.0
        Hk[0,2] = -dt*v* np.sin(sk_ant[2,0])
        Hk[1,0] = 0.0
        Hk[1,1] = 1.0
        Hk[1,2] = dt*v* np.cos(sk_ant[2,0])
        Hk[2,0] = 0.0
        Hk[2,1] = 0.0 
        Hk[2,2] = 1.0
        Hk_T = Hk.transpose()

        # Calculate Covariance associated with the wheel speeds
        kr = 0.5
        kl = 0.5
        Sigma_w = np.zeros([2,2])
        Sigma_w[0,0] = kr*abs(wr)
        Sigma_w[0,1] = 0.0
        Sigma_w[1,0] = 0.0
        Sigma_w[1,1] = kl*abs(wl)

        Nabla_w = np.zeros([3,2])
        Nabla_w[0,0] = np.cos(sk_ant[2,0])
        Nabla_w[0,1] = np.cos(sk_ant[2,0])
        Nabla_w[1,0] = np.sin(sk_ant[2,0])
        Nabla_w[1,1] = np.sin(sk_ant[2,0])
        Nabla_w[2,0] =  2/self.L 
        Nabla_w[2,1] = -2/self.L
        Nabla_w = Nabla_w * (0.5*self.r*dt)

        Nabla_w_T = Nabla_w.transpose()

        Qk = Nabla_w.dot(Sigma_w).dot(Nabla_w_T)

        Sigma_pose_est = Hk.dot(Sigma_pose_Ant).dot(Hk_T) + Qk      

        return miu_k_est, Sigma_pose_est

    def update(self, miu_k_est, Sigma_pose_est, z, Rk):
        z_est = np.zeros([2,1])
        Gk = np.zeros([2,3])


        #print("miu_k_est: ", miu_k_est)
        

        delta_x = self.mx - miu_k_est[0,0]
        delta_y = self.my - miu_k_est[1,0]
        p = delta_x**2 + delta_y**2

        #print("delta_x: ", delta_x)
        #print("delta_y: ", delta_y)
        #print("p: ", p)
        #print("mx: ", self.mx)
        #print("my: ", self.my)
        


        #Obsarvation model 
        z_est[0,0] = np.sqrt(p)
        z_est[1,0] = np.arctan2(delta_y,delta_x) - miu_k_est[2,0]

        #print("z: ", z_est)

        #Linearise the observation model
        Gk[0,0] = -delta_x / np.sqrt(p)
        Gk[0,1] = -delta_y / np.sqrt(p)
        Gk[0,2] = 0
        Gk[1,0] = delta_y / p
        Gk[1,1] = -delta_x / p
        Gk[1,2] = -1

        #print("Gk: ", Gk)


        Gk_T = Gk.transpose()
        #print("Gk_T: ", Gk_T)

        
        #Uncertainty propagation
    
        Zk = Gk.dot(Sigma_pose_est).dot(Gk_T) + Rk

        #print("Zk: ", Zk)
        #print("Sigma_pose_est: ", Sigma_pose_est)
        #print("Rk: ", Rk)


        Z_inv = np.linalg.inv(Zk) 
        #print("Z_inv: ", Z_inv)


        # Calculate the Kalman gain 
        K = Sigma_pose_est.dot(Gk_T).dot(Z_inv)
        #print("K: ", K)
        

        #Calculate position of the robot
        miu_k = miu_k_est + K.dot(z-z_est) 
        #print("z: ", z)
        #print("z_est: ", z_est)
        #print("miu_k: ", miu_k)       

        I = np.identity(3)

        Sigma_pose = (I - K.dot(Gk)).dot(Sigma_pose_est)

        self.received_aruco = 0

        return miu_k, Sigma_pose


    def wl_cb(self, msg): 
        self.wl = msg.data 
        self.received_wl = 1 

    def wr_cb(self, msg): 
        self.wr = msg.data 
        self.received_wr = 1 

    def X_aruco_cb(self,msg):
        self.mx = msg.data
        self.received_aruco = 1

    def Y_aruco_cb(self,msg):
        self.my = msg.data
        self.received_aruco = 1

    def aruco_z_cb(self,msg):
        self.z = msg.data
        self.z = np.array(self.z)
        self.z = np.reshape(self.z,(2,1))

        self.received_aruco = 1

    def Rk_cb(self, msg):
        self.Rk = msg.data

    def fill_odom(self,x, y, theta, Sigma_pose, v, w): 
        # (x,y) -> robot position 
        # theta -> robot orientation 
        # Sigma_pose -> 3x3 pose covariance matrix 

        odom=Odometry() 
        odom.header.stamp =rospy.Time.now() 
        odom.header.frame_id = "odom" 
        odom.child_frame_id = "base_link" 
        odom.pose.pose.position.x = x 
        odom.pose.pose.position.y = y 
        odom.pose.pose.position.z = 0.0 
        quat=quaternion_from_euler(0.0, 0.0, theta) 
        odom.pose.pose.orientation.x = quat[0] 
        odom.pose.pose.orientation.y = quat[1] 
        odom.pose.pose.orientation.z = quat[2] 
        odom.pose.pose.orientation.w = quat[3] 

        # Fill the covariance matrix 
        odom.pose.covariance[0] = Sigma_pose[0,0] 
        odom.pose.covariance[1] = Sigma_pose[0,1] 
        odom.pose.covariance[5] = Sigma_pose[0,2] 
        odom.pose.covariance[6] = Sigma_pose[1,0] 
        odom.pose.covariance[7] = Sigma_pose[1,1] 
        odom.pose.covariance[11] = Sigma_pose[1,2] 
        odom.pose.covariance[30] = Sigma_pose[2,0] 
        odom.pose.covariance[31] = Sigma_pose[2,1] 
        odom.pose.covariance[35] = Sigma_pose[2,2] 
        odom.twist.twist.linear.x = v 
        odom.twist.twist.angular.z = w 

        return odom 

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    # first thing, init a node! 
    rospy.init_node('EKF_localization')  
    print("Initializing node")
    EKFClass()  
