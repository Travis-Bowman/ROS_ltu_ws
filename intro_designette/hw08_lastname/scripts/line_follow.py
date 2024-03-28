#!/usr/bin/env python3

# Import base ROS
import rospy

# Import OpenCV and NumPy
import cv2 as cv
import numpy as np

# Import ROS message information
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

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

        self.hsv_hue_high = 180
        self.hsv_sat_high = 255
        self.hsv_val_high = 255

        self.hsv_hue_low = 79
        self.hsv_sat_low = 112
        self.hsv_val_low = 112

        self.image_thresh = 15

        # Initialize frame counter
        self.frame_count = 1
        
        # Initialize robot motion
        self.steer = 0.0

        self.bridge = CvBridge()

        # Define the image subscriber
        rospy.Subscriber("/camera/image_raw", Image, self.camera_callback)
        # Define publisher
        self.move_commanded = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.hsv_view = rospy.Publisher("/camera/hsv_image", Image, queue_size=10)

        self.move_command = Twist()

        self.state = "middle"

        #Start_stop state
        self.current_start_stop = False

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
    
    def split_image(self, cv_image):
        # returns 3 images as an array: [left, middle, right]
        h, w = cv_image.shape # cv_image.shape returns a tuple h, w. 
        w_third = w // 3 
        left = cv_image[:, :w_third] #
        middle = cv_image[:, w_third: 2 * w_third]
        right = cv_image[:, 2 * w_third:]
        return [left, middle, right]
    
    def state_machine_run(self, left_count, middle_count, right_count, cv_image):
        # publish left, middle, or right relative to camera (e.g. left is to the left of the camera)
        max_count = max(left_count, middle_count, right_count)# returms the highest value between each slice

        if max_count < 800:

            while True:
                self.move_command.angular.z = 0.3
                self.move_command.linear.x = 0.0
                self.move_commanded.publish(self.move_command)# Sends the command
                print("spinning")# Used for Debugging
                self.update_pixel_counts(self,cv_image) # Updating the pixel count
                if middle_count > 1000: # Looking for the the line to be re-centered
                    break

        if max_count == left_count: #limiting the state change 
            self.move_command.angular.z = 0.3
            self.move_command.linear.x = 0.3
            print("left") # Used for Debugging
            print(max_count) # Used for Debugging
            self.move_commanded.publish(self.move_command)# Sends the command
            self.state = "left" # Updates the state

        elif max_count == middle_count: #limiting the state change
            self.move_command.angular.z = 0.0
            self.move_command.linear.x = 0.3
            print("middle") # Used for Debugging
            print(max_count) # Used for Debugging
            self.move_commanded.publish(self.move_command)# Sends the command
            self.state = "middle" # Updates the state

        elif max_count == right_count:
            self.move_command.angular.z = -0.3
            self.move_command.linear.x = 0.3
            print("right") # Used for Debugging
            print(max_count) # Used for Debugging
            self.move_commanded.publish(self.move_command) # Sends the command
            self.state = "right" # Updates the state

    def count_pixels(self, cv_image):        
        # return number of pixels in the image above threshold as an integer
        return np.sum(cv_image > self.image_thresh) #np.sum returns the number of pixels in the image cv_image that have values greater than the threshold self.image_thresh.

    ################################
    # Dynamic Reconfigure callback
    ################################
    def dyn_reconfig_callback(self, config, level):
        try:
            self.enable = config['enable']
            self.gain = config['gain']
            self.speed_desired = config['speed']
            self.frame_skip = config['frame_skip']
            self.follow_type = config['follow_type']

            self.hsv_hue_high = config['hsv_hue_high']
            self.hsv_sat_high = config['hsv_sat_high']
            self.hsv_val_high = config['hsv_val_high']

            self.hsv_hue_low = config['hsv_hue_low']
            self.hsv_sat_low = config['hsv_sat_low']
            self.hsv_val_low = config['hsv_val_low']

            #self.thres_value = config['thres_value']

            self.dyn_config = config
        except Exception as e:
            rospy.logerr(f"Error in dynamic reconfigure callback: {e}")
        return config
    
    def HSV_mask(self, cv_image):
        # Convert the image from BGR to HSV colorspace
        frame_HSV = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        # Create a mask that is positive for all pixels between the specified HSV thresholds
        mask = cv.inRange(frame_HSV, (self.hsv_hue_low, self.hsv_sat_low, self.hsv_val_low), (self.hsv_hue_high, self.hsv_sat_high, self.hsv_val_high))

        return mask
        


    #########################
    # Camera image callback
    #########################
    def camera_callback(self, img_msg):
        # Need to convert the ROS Image message to a CV2 Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))        

        # Apply threshold operation to the image and display
        thresh=self.HSV_mask(cv_image)
        display_thresh = self.bridge.cv2_to_imgmsg(thresh, encoding="mono8")        
        self.hsv_view.publish(display_thresh)
        # Split the image into three pieces
        images=self.split_image(thresh)
        # Publish the side with the highest number pixels exceeding threshold
        if len(images)==3:
            self.state_machine_run(self.count_pixels(images[0]),self.count_pixels(images[1]),self.count_pixels(images[2]), cv_image)


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
