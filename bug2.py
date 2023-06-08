#!/usr/bin/env python  


import rospy  
from geometry_msgs.msg import Twist, PoseStamped 
from std_msgs.msg import Float32, Float64MultiArray
from sensor_msgs.msg import LaserScan   #Lidar 
import numpy as np 
import time
 

         
#This class will make the puzzlebot move to a given goal 
class GoToGoal():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 
        ############ Variables ############### 

        ##############################################################
        self.x_target=0.5 #x position of the goal 
        self.y_target=6.5#.0 #y position of the goal 

        self.x_targets = [2.0, 2.5, 2.5, 3.0, 0.0, 0.0]
        self.y_targets = [4.5, 10.0, 7.5, 2.5, 0.0, 0.0]
        ##############################################################
        self.target = 0
        #####################################################################################
        self.robot_x= 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.goal_received=1 #flag to indicate if the goal has been received 
        self.lidar_received = False #flag to indicate if the laser scan has been received 
        self.target_position_tolerance=0.08 #target position tolerance [m] 
        fw_distance = 0.4 #.38  #0.45  distance to activate the following walls behavior [m] 
        progress = 0.3 #If the robot is this close to the goal with respect to when it started following walls it will stop following walls 
        v_msg=Twist() #Robot's desired speed  
        self.wr=0 #right wheel speed [rad/s] 
        self.wl=0 #left wheel speed [rad/s] 
        self.current_state = 'GoToGoal' #Robot's current state 
        self.time_of_change = 0
        self.m = 0
        self.b = 0
        self.new_ecuation = True

        rospy.on_shutdown(self.cleanup)  
        ###******* INIT PUBLISHERS *******###  
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  

        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)  
        #rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_cb) 
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb) 
        rospy.Subscriber("sk", Float64MultiArray, self.sk_cb)

        #********** INIT NODE **********###  
        freq=10 
        rate = rospy.Rate(freq) #freq Hz  
        Dt =1.0/float(freq) #Dt is the time between one calculation and the next one 
        print("Node initialized") 
        print("Please send a Goal from rviz using the button: 2D Nav Goal") 
        print("You can also publish the goal to the (move_base_simple/goal) topic.") 

        ################ MAIN LOOP ################  
        while not rospy.is_shutdown():  

            if self.lidar_received: 
                self.closest_range, self.closest_angle = self.get_closest_object(self.lidar_msg) #get the closest object range and angle 
                thetaAO = self.get_theta_ao(self.closest_angle) 
                thetaGTG =self.get_theta_gtg(self.x_targets[self.target], self.y_targets[self.target], self.robot_x, self.robot_y, self.robot_theta) 
                thetaFWC = self.get_theta_fw(thetaAO, True)
                d_t=np.sqrt((self.x_targets[self.target]-self.robot_x)**2+(self.y_targets[self.target]-self.robot_y)**2) 

                
                if(self.new_ecuation == True):
                    A,B,C = self.compute_line_ecuation()

                if self.at_goal():  
                    print("Goal reached") 
                    print("Distance to the goal: ", self.distance)

                    time.sleep(5)

                    self.target = self.target+1


                    
                    if(self.target >= 6):
                        self.current_state = 'Stop'
                        self.target = 5
                    else:
                        self.new_ecuation = True
                        self.current_state = 'GoToGoal' 
                    #print("Stop") 
                    v_msg.linear.x = 0 
                    v_msg.angular.z = 0 

                elif self.current_state == 'GoToGoal':                  
                    if ((self.closest_range <= fw_distance) and (abs(thetaFWC - thetaGTG)<=np.pi/2) and (rospy.get_time() - self.time_of_change) >= 2.0): 
                        # Implement the following walls behavior 
                        print(" change to following walls")
                        d_H1 = np.sqrt((self.x_targets[self.target]-self.robot_x)**2+(self.y_targets[self.target]-self.robot_y)**2)
                        self.time_of_change = rospy.get_time() 
                        self.current_state = "Clockwise"

                    elif((self.closest_range <= fw_distance) and (abs(thetaFWC - thetaGTG)>np.pi/2) and (rospy.get_time() - self.time_of_change) >= 2.0):
                        # Implement the following walls behavior 
                        print(" change to following walls")
                        d_H1 = np.sqrt((self.x_targets[self.target]-self.robot_x)**2+(self.y_targets[self.target]-self.robot_y)**2)
                        self.time_of_change = rospy.get_time() 
                        self.current_state = "CounterClockwise"

                    else: 
                        print("Moving to the Goal: (x,y)", self.x_targets[self.target], self.y_targets[self.target]) 
                        v_gtg, w_gtg = self.compute_gtg_control(self.x_targets[self.target], self.y_targets[self.target], self.robot_x, self.robot_y, self.robot_theta) 
                        v_msg.linear.x = v_gtg 
                        v_msg.angular.z = w_gtg 

                elif self.current_state == 'Clockwise': 

                    if self.clear_shot(thetaAO,thetaGTG) and self.intersect_line(A,B,C, d_H1): 
                        self.current_state = 'GoToGoal' 
                        self.time_of_change = rospy.get_time() 

                        print("Change to Go to goal") 

                    else: 
                        print("Following wall: (x,y)", self.x_targets[self.target], self.y_targets[self.target]) 
                        thetaFWC=self.get_theta_fw(thetaAO, True) #If it True is passed return clockwise, else counterclockwise 
                        vFWC, wFWC = self.compute_fw_control(thetaFWC) 
                        v_msg.linear.x = vFWC 
                        v_msg.angular.z = wFWC

                elif self.current_state == 'CounterClockwise':
                    if self.clear_shot(thetaAO,thetaGTG) and self.intersect_line(A,B,C, d_H1): 
                        self.current_state = 'GoToGoal' 
                        self.time_of_change = rospy.get_time() 
                        print("Change to Go to goal") 

                    else: 
                        print("Following wall: (x,y)", self.x_targets[self.target], self.y_targets[self.target]) 
                        thetaFWCC=self.get_theta_fw(thetaAO, False) #If it True is passed return clockwise, else counterclockwise 
                        vFWCC, wFWCC = self.compute_fw_control(thetaFWCC) 
                        v_msg.linear.x = vFWCC
                        v_msg.angular.z = wFWCC


                elif self.current_state == 'Stop': 
                    print("Stop") 
                    v_msg.linear.x = 0 
                    v_msg.angular.z = 0


            print("Current Position: (x,y)",self.robot_x, self.robot_y)
            self.pub_cmd_vel.publish(v_msg)  
            rate.sleep() 


    def clear_shot(self, thetaAO, thetaGTG):
        if(abs(thetaAO-thetaGTG)<np.pi/2.0):
            return True
        else:
            return False 

    def intersect_line(self, A, B, C, d_H1):

        current_time = rospy.get_time() 
        if((current_time - self.time_of_change) >= 3):
            x_actual = self.robot_x 
            y_actual = self.robot_y


            d = abs((A*x_actual)+(B*y_actual)+C)/np.sqrt((A**2)+(B**2))
            print("d_to line: ", d)

            if(d < 0.15):
                #d_L1 = np.sqrt((self.x_targets[self.target]-self.robot_x)**2+(self.y_targets[self.target]-self.robot_y)**2)
                #if d_L1 < d_H1:
                return True
            else :
                return False
        else:
            return False



    def compute_line_ecuation(self):
        x1 = self.robot_x
        y1 = self.robot_y
        x2 = self.x_targets[self.target]
        y2 = self.y_targets[self.target]

        A = (y2 - y1) / (x2 - x1)
        B = -1
        C = -A * x1 + y1
        self.new_ecuation = False
        return A, B, C



    def sk_cb(self, msg):
        sk = msg.data
        self.robot_x = sk[0]
        self.robot_y = sk[1]
        self.robot_theta = sk[2]


    def at_goal(self): 
        #This function returns true if the robot is close enough to the goal 
        #This functions receives the goal's position and returns a boolean 
        #This functions returns a boolean 
        self.distance = np.sqrt((self.x_targets[self.target]-self.robot_x)**2+(self.y_targets[self.target]-self.robot_y)**2)
        return self.distance < self.target_position_tolerance 

    def get_closest_object(self, lidar_msg): 
        #This function returns the closest object to the robot 
        #This functions receives a ROS LaserScan message and returns the distance and direction to the closest object 
        #returns  closest_range [m], closest_angle [rad], 
        min_idx = np.argmin(lidar_msg.ranges) 
        closest_range = lidar_msg.ranges[min_idx] 
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment 

        # limit the angle to [-pi, pi] 
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle)) 
        return closest_range, closest_angle 

 
    def get_theta_gtg(self, x_target, y_target, x_robot, y_robot, theta_robot): 
        #This function returns the angle to the goal 
        theta_target=np.arctan2(y_target-y_robot,x_target-x_robot) 
        e_theta=theta_target-theta_robot 
        #limit e_theta from -pi to pi 
        #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi 
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 
        return e_theta 

 

    def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot): 
        #This function returns the linear and angular speed to reach a given goal 
        #This functions receives the goal's position (x_target, y_target) [m] 
        #  and robot's position (x_robot, y_robot, theta_robot) [m, rad] 
        #This functions returns the robot's speed (v, w) [m/s] and [rad/s] 

        kvmax = 0.25 #linear speed maximum gain  
        kwmax=0.8 #angular angular speed maximum gain 

        #kw=0.5 
        av = 2.0 #Constant to adjust the exponential's growth rate   
        aw = 2.0 #Constant to adjust the exponential's growth rate 
        ed=np.sqrt((x_target-x_robot)**2+(y_target-y_robot)**2) 

        #Compute angle to the target position 
        theta_target=np.arctan2(y_target-y_robot,x_target-x_robot) 
        e_theta=theta_target-theta_robot 

        #limit e_theta from -pi to pi 
        #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi 
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 


        #Compute the robot's angular speed 
        kw=kwmax*(1-np.exp(-aw*e_theta**2))/abs(e_theta) #Constant to change the speed  
        w=kw*e_theta 

        if abs(e_theta) > np.pi/8: 
            #we first turn to the goal 
            v=0 #linear speed  

        else: 
            # Make the linear speed gain proportional to the distance to the target position 
            kv=kvmax*(1-np.exp(-av*ed**2))/abs(ed) #Constant to change the speed  
            v=kv*ed #linear speed  

        return v,w 

 

    def get_theta_ao(self, theta_closest): 

        ##This function returns the angle for the Avoid obstacle behavior  
        # theta_closest is the angle to the closest object [rad] 

        #This functions returns the angle for the Avoid obstacle behavior [rad] 
        ############################################################ 

        thetaAO=theta_closest-np.pi 
        #limit the angle to [-pi,pi] 
        thetaAO = np.arctan2(np.sin(thetaAO),np.cos(thetaAO)) 
        return thetaAO 

 

    def get_theta_fw(self, thetaAO, clockwise): 
        ## This function computes the linear and angular speeds for the robot 
        # It receives thetaAO [rad] and clockwise [bool]

        if(clockwise):
            thetaFWC = -np.pi/2 + thetaAO
        else:
            thetaFWC = np.pi/2 +  thetaAO

        thetaFWC = np.arctan2(np.sin(thetaFWC), np.cos(thetaFWC))


        return thetaFWC

     
    def compute_fw_control(self, thetaFW): 
        ## This function computes the linear and angular speeds for the robot 
        # It receives thetaFW [rad]    
        #Compute linear and angular speeds 

        kw = 1.4
        v = 0.0  # Linear velocity
        w = kw * thetaFW  # Angular velocity
        
        if self.closest_range > 0.5:
            kw = 0.6  
            v = 0.15  
            w = kw * (thetaFW + self.closest_angle/1.5)  
        
        elif abs(thetaFW) < np.pi/4.0:
            w = kw * thetaFW  # Angular velocity
            v = 0.25  
        
        return v, w


     

    def get_angle(self, idx, angle_min, angle_increment):  
        ## This function returns the angle for a given element of the object in the lidar's frame  
        angle= angle_min + idx * angle_increment  
        # Limit the angle to [-pi,pi]  
        angle = np.arctan2(np.sin(angle),np.cos(angle))  
        return angle  

    def polar_to_cartesian(self,r,theta):  
        ## This function converts polar coordinates to cartesian coordinates  
        x = r*np.cos(theta)  
        y = r*np.sin(theta)  
        return (x,y)  

     
    def laser_cb(self, msg):   
        ## This function receives a message of type LaserScan   
        self.lidar_msg = msg  
        self.lidar_received = True  

 
    def wl_cb(self, wl):  
        ## This function receives a the left wheel speed [rad/s] 
        self.wl = wl.data 

    def wr_cb(self, wr):  
        ## This function receives a the right wheel speed.  
        self.wr = wr.data  

    def goal_cb(self, goal):  
        ## This function receives a the goal from rviz.  
        print("Goal received I'm moving to x= "+str(goal.pose.position.x)+" y= "+str(goal.pose.position.y)) 
        self.current_state = "GoToGoal" 

        # assign the goal position 
        self.x_target = goal.pose.position.x 
        self.y_target = goal.pose.position.y 

        self.compute_line_ecuation()
        self.goal_received=1 

         
    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 

 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("bug_2", anonymous=True)  
    GoToGoal()  