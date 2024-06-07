#!/usr/bin/python3

import rclpy
from rclpy import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray


class BuoyClassifier(Node):
    
    def __init__(self):
        super().__init__("buoy classifier")
        
        self.create_timer(0.5, self.publish_buoy_positions)
        
        self.color_image_listener = self.create_subscription(msg_type=Image, topic="/color/image_raw", callback=self.color_image_callback, qos_profile=10) 
        self.depth_image_listener = self.create_subscription(msg_type=Image, topic="/depth/image_rect_raw", callback=self.depth_image_callback, qos_profile=10)
        
        self.buoy_position_publisher = self.create_publisher(msg_type=Float64MultiArray, topic="/buoy_positions", qos_profile=10)
        
        self.cur_color_image = None
        self.cur_depth_image = None 
    
    
    def color_image_callback(self, msg):
        """"""
        pass
    
    def depth_image_callback(self, msg):
        """"""
        pass
    
    
    
    def publish_buoy_positions(self):
        """
        Takes in 2 images, the color image and the depth image and uses them to output what the buoy positions that it can see are.
        This is published to the buoy_position publisher with the following form: TODO
        """
        pass
    
    

def main():
    rclpy.init()
    buoy_classifier = BuoyClassifier()
    rclpy.spin(buoy_classifier)


if __name__ == "__main__": 
    main()