import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

import numpy as np

from geometry_msgs.msg import Twist


LINEAR_SPEED = 0.2
KP = 1.5/100



class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber') # this is calling the constructor class of "NODE" class
        # Defines a ros2 subscriber that gets the ROS2 images
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', # the ros2 topic we are interested in
            self.listener_callback, 
            10) # QOS history 
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def get_contour_data(self, mask):
        """
        Return the centroid of the largest contour in the binary image 'mask'
        """
        # MIN_AREA_TRACK = 50
        MIN_AREA_TRACK = 1000 # minimum area of contours to be considered

        # get list of contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        line = {} # dictionary of lines

        for contour in contours:
            M = cv2.moments(contour)

            if (M['m00'] > MIN_AREA_TRACK):
                # Contour is part of tracks
                line['x'] = int(M["m10"]/M["m00"])
                line['y'] = int(M["m01"]/M["m00"])
        return (line) # returning a tuple



    def listener_callback(self, data):
        # use cv bridge to convert images from ROS2 format to opencv format
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        hsv_image = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Create Blue Binary mask
        lower_blue = np.array([100,50,50])
        upper_blue = np.array([130,255,255])
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        
        blue_segmented_image = cv2.bitwise_and(current_frame, current_frame, mask=blue_mask)

        line = self.get_contour_data(blue_mask)
        # Display the image with the line centroid AND
        # Command Robot to move so that centroid is in the center of the camera frame
        cmd = Twist()
        _, width, _ = blue_segmented_image.shape # get width of the camera image

        if line:
            x = line['x']

            error = x - width//2

            cmd.linear.x = LINEAR_SPEED
            cv2.circle(blue_segmented_image, (line['x'], line['y']), 5, (0, 0, 255), 7)
            
            # Determine the speed to turn and get the line in th e center of the camera
            cmd.angular.z = float(error) * -KP
            print("Error: {} | Angular Z: {}, ".format(error, cmd.angular.z))

        # Send the command to execute
        self.publisher.publish(cmd)

        cv2.imshow("Camera Feed, Current Frame", current_frame)
        cv2.imshow("Camera Feed, Blue", blue_segmented_image)
        cv2.waitKey(1)




def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()