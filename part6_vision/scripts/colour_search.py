#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

# Import image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import necessary ROS interface types:
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped

class ColourSearch(Node):

    def __init__(self):
        super().__init__("colour_search")
        
        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic="/camera/image_raw",
            callback=self.camera_callback,
            qos_profile=10
        )
        
        self.vel_pub = self.create_publisher(
            msg_type=TwistStamped,
            topic="/cmd_vel",
            qos_profile=10
        )

        self.timer = self.create_timer(
            timer_period_sec=1/5,
            callback=self.timer_callback
        )

        self.cvbridge_interface = CvBridge()

        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.vel_cmd = TwistStamped()
        self.vel_cmd.twist.angular.z = self.turn_vel_fast

        self.move_rate = "" # fast, slow, stop
        self.stop_counter = 0

        self.shutdown = False
        
        self.m00 = 0
        self.m00_min = 10000

    def shutdown_ops(self):
        self.get_logger().info(
            "Shutting down..."
        )
        cv2.destroyAllWindows()
        for i in range(5):
            self.vel_pub.publish(TwistStamped())
        self.shutdown = True
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"{e}")
        
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        #         BLUE             GREEN           RED             TURQUOISE
        lowers = [(115, 224, 100), (50, 150, 100), ( 0, 185, 100), ( 75, 150, 100)]
        uppers = [(130, 255, 255), (65, 255, 255), (10, 255, 255), (100, 255, 255)]

        for i in range(4):
            if i == 0:
                mask = cv2.inRange(hsv_img, lowers[i], uppers[i])
            else:
                mask += cv2.inRange(hsv_img, lowers[i], uppers[i])

        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)
    
    def timer_callback(self):
        if self.stop_counter > 0:
            self.stop_counter -= 1

        if self.m00 > self.m00_min:
            # blob detected
            if self.cy >= 560-100 and self.cy <= 560+100:
                if self.move_rate == 'slow':
                    self.move_rate = 'stop'
                    self.stop_counter = 30
            else:
                self.move_rate = 'slow'
        else:
            self.move_rate = 'fast'
            
        if self.move_rate == 'fast':
            self.get_logger().info(
                "\nMOVING FAST:\n"
                "I can't see anything at the moment, scanning the area..."
            )
            self.vel_cmd.twist.angular.z = self.turn_vel_fast
            
        elif self.move_rate == 'slow':
            self.get_logger().info(
                f"\nMOVING SLOW:\n"
                f"A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels."
            )
            self.vel_cmd.twist.angular.z = self.turn_vel_slow
        
        elif self.move_rate == 'stop' and self.stop_counter > 0:
            self.get_logger().info(
                f"\nSTOPPED:\n"
                f"The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels... Counting down: {self.stop_counter}"
            )
            self.vel_cmd.twist.angular.z = 0.0
        
        else:
            self.get_logger().info(
                f"\nMOVING SLOW:\n"
                f"A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels."
            )
            self.vel_cmd.twist.angular.z = self.turn_vel_slow
        
        self.vel_pub.publish(self.vel_cmd)
            
def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = ColourSearch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f"{node.get_name()} received a shutdown request (Ctrl+C)"
        )
    finally:
        node.shutdown_ops()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
