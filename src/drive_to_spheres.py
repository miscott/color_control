#!/usr/bin/env python

'''
Created by Mitchel Scott and Oscar De Haro with assistance from Matt Taylor
and James Irwin at Washington State Unviersity
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import radians
import time
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

def process_image(image,state):
    #convert color space from BGR to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    if state == 1:
        lower_bound = np.array([0, 100, 100])
        upper_bound = np.array([0, 255, 255])
    if state == 2:
        lower_bound = np.array([40, 100, 100])
        upper_bound = np.array([60, 255, 255])
    if state == 3:
        lower_bound = np.array([100, 100, 100])
        upper_bound = np.array([160, 255, 255])
    if state == 0:
        lower_bound = np.array([10, 100, 100])
        upper_bound = np.array([30, 255, 255])
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    cv2.imshow("image_mask", mask)
    M = cv2.moments(mask)
    location = None
    magnitude = 0
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        magnitude = M['m00']
        location = (cx-320, cy-240) #scale so that 0,0 is center of screen
        cv2.circle(image, (cx,cy), 3, (0,0,255), -1)
    cv2.imshow("processing result", image)

    cv2.waitKey(1)
    return location, magnitude
    

class Node:
    def __init__(self):
        #register a subscriber callback that receives images
        self.inital_time = time.time()
        self.final_print = True
        self.state = 1
        self.prev_location = None
        self.go_straight = False
        self.save_position = [(0,0)]
        self.position = Odometry()
        self.bridge = CvBridge()
        self.state_print()
        self.max_search_time = 20 #seconds
        self.mag_val = 11000000 #Roughly 2m from ball
        self.rate = 20.0
        self.speed = 0
        self.x_location_prev = 0
        self.x_location_dot = 0
        self.turn = 0
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', 
                                          Image, self.image_callback, 
                                          queue_size=1) #Subscribe to image
        self.movement_pub = rospy.Publisher('cmd_vel_mux/input/teleop', 
                                            Twist, queue_size=1)
        rospy.Subscriber("odom", Odometry, self.callback)

    def state_print(self):
        if self.state == 1:
            print('Searching for RED ball')
        elif self.state == 2:
            print('Searching for GREEN ball')
        elif self.state == 3:
            print('Searching for BLUE ball')
        elif self.state == 0:
            print('Searching for YELLOW ball')

    def callback(self, msg):
        self.position = msg
        x = self.position.pose.pose.position.x
        y = self.position.pose.pose.position.y
        self.save_position.append((x,y))

    def image_callback(self, ros_image):
        '''
        Convert the ros image to a format openCV can use. 
        Use the center of the ball to drive the turtle bot towards the ball
        until the magnitude is too large and the robot stops
        '''
        time_elapsed = time.time() - self.inital_time
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        location, magnitude = process_image(cv_image, self.state) 
        cmd = Twist()
        #print(self.state)
        if magnitude < self.mag_val and time_elapsed <= self.max_search_time:
            self.inital_time = time.time()
            if location == None:
                '''
                If we don't know the location of the sphere, turn clockwise
                '''
                self.turn = radians(20)
                self.speed = 0
            else:
                '''
                If we know the location of the sphere, turn until roughly in 
                center of image
                '''
                x_location = location[0] 
                self.x_location_dot = (x_location - self.x_location_prev)/self.rate
                if abs(x_location) > 200:
                    
                    self.turn = -radians(4)*np.sign(x_location)
                    self.speed = 0
                else:
                    self.turn = -0.005*x_location -0.5*self.x_location_dot
                    self.speed = 0.3
                self.x_location_prev = x_location
            
            cmd.linear.x = self.speed
            cmd.angular.z = self.turn 
            
        else:
            state = self.state + 1
            cmd = Twist()
            self.movement_pub.publish(cmd)            
            if state < 4:
                
                self.state = state
            else:
                self.state = state % 4
                
            self.state_print()
        self.movement_pub.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("Locate_Sphere")
    node = Node()
    rospy.spin()