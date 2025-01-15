#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError # (2)!

from sensor_msgs.msg import Image # (3)!

from pathlib import Path # (1)!

class ObjectDetection(Node):

    def __init__(self):
        super().__init__("object_detection")

        self.base_image_path = Path.home().joinpath("myrosdata/object_detection/")
        self.base_image_path.mkdir(parents=True, exist_ok=True) # (5)!
        
        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic="/camera/image_raw",
            callback=self.camera_callback,
            qos_profile=10
        )

        self.waiting_for_image = True # (7)!
    
    def camera_callback(self, img_data): # (14)!
        cvbridge_interface = CvBridge() # (6)!
        try:
            self.cv_img = cvbridge_interface.imgmsg_to_cv2(
                img_data, desired_encoding="bgr8"
            ) # (16)!
        except CvBridgeError as e:
            self.get_logger().info(f"{e}")

        if self.waiting_for_image: # (17)!
            height, width, channels = self.cv_img.shape

            self.get_logger().info(
                f"Obtained an image of height {height}px and width {width}px."
            )

            self.show_image(img_name = "step1_original")  

            self.get_logger().info(
                "IMPORTANT: Close the image pop-up window to exit."
            )
        
            cv2.waitKey(0) # (13)!
            self.waiting_for_image = False
            cv2.destroyAllWindows() # (20)!

    def show_image(self, img_name, save_img=True): # (8)!
        self.full_image_path = self.base_image_path.joinpath(
            f"{img_name}.jpg") # (9)!

        self.get_logger().info("Opening the image in a new window...")
        cv2.imshow(img_name, self.cv_img) # (10)!
        if save_img:
            self.save_image()
    
    def save_image(self): # (8)!
        self.get_logger().info(f"Saving the image...")
        
        cv2.imwrite(str(self.full_image_path), self.cv_img) # (11)!
        
        self.get_logger().info(
            f"\nSaved an image to '{self.full_image_path}'\n"
            f"  - image dims: {self.cv_img.shape[0]}x{self.cv_img.shape[1]}px\n"
            f"  - file size: {self.full_image_path.stat().st_size} bytes") # (12)!
        
def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    while node.waiting_for_image:
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()     