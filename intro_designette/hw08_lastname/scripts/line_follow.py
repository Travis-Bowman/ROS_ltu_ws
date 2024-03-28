#!/usr/bin/env python3

# Import base ROS
import rospy

# Import OpenCV and NumPy
import cv2 as cv
import numpy as np

# Import ROS message information
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import dynamic reconfigure 
from dynamic_reconfigure.server import Server
from hw08_lastname.cfg import LineFollowDynCfgConfig


ACTIVE_WINDOWS = []

##################################
# LineFollowNode class definition
##################################
class LineFollowNode():
    def __init__(self):
        """Line following node"""

        # Initialize dynamic reconfigure
        self.enable = 0
        self.gain = 0.0
        self.speed = 0.0
        self.speed_desired = 0.0
        self.frame_skip = 1
        self.follow_type = 0

        # Initialize frame counter
        self.frame_count = 1
        
        # Initialize robot motion
        self.steer = 0.0

        # Define the image subscriber

        # Define publisher

        # Set up dynamics reconfigure
        self.srv = Server(LineFollowDynCfgConfig,
                          self.dyn_reconfig_callback)

        # Define ROS rate
        self.rate = rospy.Rate(20)  # Vehicle rate

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():

            # Build message yaw rate message and publish
            
            # Sleep for time step
            self.rate.sleep()
            
        return


    ################################
    # Dynamic Reconfigure callback
    ################################
    def dyn_reconfig_callback(self, config, level):
        self.enable = config['enable']
        self.gain = config['gain']
        self.speed_desired = config['speed']
        self.frame_skip = config['frame_skip']
        self.follow_type = config['follow_type']

        # Add HSV hue, sat and val

        self.dyn_config = config
        return config
        


    #########################
    # Camera image callback
    #########################
    def camera_callback(self, rgb_msg):

        # Check frame counter
        if( self.frame_count % self.frame_skip != 0 ):
            self.frame_count += 1
            return
        self.frame_count = 1
        
        
        # Get the camera image and make a copy
        img = CvBridge().imgmsg_to_cv2(rgb_msg, "bgr8" )


        # Convert image to a HSV image and blur

        # Threshold HSV image to binary based on range

        ##
        ## Check if no lines are observed and spin
        ## Set speed and return!
        ##

        
        ##
        ## Zig-Zag method
        ##

        ##
        ## Binning
        ##

        ##
        ## P control
        ##

                
        return
    

    ####################
    # Display an image
    ####################
    def display_image(self, title_str, img, disp_flag ):

        if( disp_flag ):
            # Display the given image
            cv.namedWindow(title_str, cv.WINDOW_NORMAL)
            cv.imshow(title_str, img)
            cv.waitKey(3)

            # Add window to active window list
            if not ( title_str in ACTIVE_WINDOWS ):
                ACTIVE_WINDOWS.append(title_str)
        else:
            if( title_str in ACTIVE_WINDOWS):
                cv.destroyWindow(title_str)
                ACTIVE_WINDOWS.remove(title_str)
        return



#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('line_folllow_node')
    print("Line Follow node initialized")
    
    # Start node
    try:
        LineFollowNode()
    except rospy.ROSInterruptException:
        pass
