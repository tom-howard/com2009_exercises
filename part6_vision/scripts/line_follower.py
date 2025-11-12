#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped

class LineFollower(Node):
    
    def __init__(self):
        super().__init__("line_follower")
        
        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic="/camera/image_raw",
            callback=self.camera_callback,
            qos_profile=10,
        )
        
        self.vel_pub = self.create_publisher(
            msg_type=TwistStamped,
            topic="/cmd_vel",
            qos_profile=10
        )
        
        self.vel_cmd = TwistStamped()
        self.shutdown = False
        
    def shutdown_ops(self):
        self.get_logger().info(
            "Shutting down..."
        )
        cv2.destroyAllWindows()
        for i in range(5):
            self.vel_pub.publish(TwistStamped())
        self.shutdown = True

    def camera_callback(self, img_data):
        cvbridge_interface = CvBridge()
        try:
            cv_img = cvbridge_interface.imgmsg_to_cv2(
                img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"{e}")

        cv2.imshow("camera image", cv_img)

        height, width, _ = cv_img.shape
        crop_width = 1800
        crop_height = int(height / 5)
        crop_y0 = int((width / 2) - (crop_width / 2))
        crop_y1 = crop_y0 + crop_width
        crop_z0 = height - 200 - crop_height
        crop_z1 = crop_z0 + crop_height
        cropped_img = cv_img[
            crop_z0:crop_z1, crop_y0:crop_y1
        ]
        cv2.imshow("cropped_image", cropped_img)

        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        lower = (145, 180, 100)
        upper = (165, 255, 255)
        line_mask = cv2.inRange(
            hsv_img, lower, upper
        )
        line_isolated = cv2.bitwise_and(
            cropped_img, cropped_img, mask = line_mask
        )
        
        cv2.imshow("filtered line", line_isolated) 

        m = cv2.moments(line_mask)
        cy = m['m10'] / (m['m00'] + 1e-5)
        cz = m['m01'] / (m['m00'] + 1e-5)

        res = cv2.bitwise_and(cropped_img, cropped_img, mask = line_mask)

        cv2.circle(res, (int(cy), int(cz)), 10, (255, 0, 0), 2)
        cv2.imshow("filtered image", res)

        cv2.waitKey(1)

        kp = -0.0001
        reference_input = width / 2
        feedback_signal = cy
        error = feedback_signal - reference_input 

        ang_vel = kp * error
        if ang_vel < -1.82:
            ang_vel = -1.82
        elif ang_vel > 1.82:
            ang_vel = 1.82
        self.get_logger().info(
            f"Error = {error:.1f} pixels | Control Signal = {ang_vel:.2f} rad/s",
            throttle_duration_sec=0.5
        )
        self.vel_cmd.twist.linear.x = 0.1
        self.vel_cmd.twist.angular.z = ang_vel
        self.vel_pub.publish(self.vel_cmd)

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = LineFollower()
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
