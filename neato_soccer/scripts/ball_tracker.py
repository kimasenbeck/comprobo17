#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.twist = None
        
        rospy.Subscriber(image_topic, Image, self.process_image)
        rospy.Subscriber("bump", Image, self.process_bump)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow('video_window')

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.binary_image = cv2.inRange(self.cv_image, (0,10,150), (110,110,255))
        #self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        #self.binary_image2 = cv2.inRange(self.hsv_image, (5, 0, 0), (15, 360, 360))
    def process_bump(self, msg):
        if msg.leftFront or msg.rightFront:
            self.twist = Twist(linear=Vector3(.3,0,0), angular=Vector3())
            rospy.rate(1).sleep()
        
    def dribble(self):
        moments = cv2.moments(self.binary_image)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
        #normalize x based on the width of our image to range from -0.5 to 0.5
        self.center_x = self.center_x * 1.0/640 - 0.5
        forward = Vector3(0.0, 0.0, 0.0)
        if (self.center_x < 0.25 and self.center_x > -0.25):
            forward = Vector3(0.3, 0.0, 0.0)

        self.twist = Twist(linear=forward, angular=Vector3(0.0, 0.0, -2 * self.center_x))
        self.pub.publish(self.twist)

    def stop(self):
        """ This function publishes a twist to make the robot stop."""
        self.pub.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))

        
    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        rospy.on_shutdown(self.stop)
        while not rospy.is_shutdown():
            if not self.cv_image is None:
                print self.cv_image.shape
                cv2.imshow('video_window', self.binary_image)
                cv2.imshow('window_2', self.cv_image)
                #cv2.imshow('window_3', self.binary_image2)
                cv2.waitKey(5)
                self.dribble()
                
            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()
