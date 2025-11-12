#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node 

import cv2
from cv_bridge import CvBridge, CvBridgeError 

from sensor_msgs.msg import Image 

from pathlib import Path 

class ObjectDetection(Node):

    def __init__(self): 
        super().__init__("object_detection")

        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic="/camera/image_raw",
            callback=self.camera_callback,
            qos_profile=10
        )

        self.waiting_for_image = True 

    def camera_callback(self, img_data): 
        cvbridge_interface = CvBridge() 
        try:
            cv_img = cvbridge_interface.imgmsg_to_cv2(
                img_data, desired_encoding="bgr8"
            ) 
        except CvBridgeError as e:
            self.get_logger().warning(f"{e}")

        if self.waiting_for_image: 
            height, width, channels = cv_img.shape

            self.get_logger().info(
                f"Obtained an image of height {height}px and width {width}px."
            )

            self.show_image(img=cv_img, img_name="step1_original")

            self.waiting_for_image = False 
            cv2.destroyAllWindows() 

    def show_image(self, img, img_name, save_img=True): 

        self.get_logger().info(f"Opening {img_name}...")
        cv2.imshow(img_name, img) 

        cv2.waitKey(4000) 

        if save_img: 
            self.save_image(img, img_name)

    def save_image(self, img, img_name): 
        self.get_logger().info(f"Saving the image...")

        base_image_path = Path.home().joinpath("object_detection")
        base_image_path.mkdir(parents=True, exist_ok=True) 
        full_image_path = base_image_path.joinpath(
            f"{img_name}.jpg") 

        cv2.imwrite(str(full_image_path), img) 

        self.get_logger().info(
            f"\nSaved an image to '{full_image_path}'\n"
            f"  - image dims: {img.shape[0]}x{img.shape[1]}px\n"
            f"  - file size: {full_image_path.stat().st_size} bytes"
        ) 

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    while node.waiting_for_image:
        rclpy.spin_once(node) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    