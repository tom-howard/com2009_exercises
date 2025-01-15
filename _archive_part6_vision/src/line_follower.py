#!/usr/bin/env python3
# All credit to Sabeethan Kanagasingham for this one,
# Thanks Sabee!
# December 2022

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from tb3 import Tb3Move

class LineFollower():
    def __init__(self):
        node_name = "line_follower"
        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(5)

        self.cvbridge_interface = CvBridge()
        self.img_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.camera_cb)
        self.robot_controller = Tb3Move()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_cb(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(
                img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

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

        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        lower = (145, 180, 100)
        upper = (165, 255, 255)
        line_mask = cv2.inRange(
            hsv_img, lower, upper
        )
        line_isolated = cv2.bitwise_and(
            cropped_img, cropped_img, mask = line_mask
        )
        
        m = cv2.moments(line_mask)
        cy = m['m10'] / (m['m00'] + 1e-5)
        cz = m['m01'] / (m['m00'] + 1e-5)

        res = cv2.bitwise_and(cropped_img, cropped_img, mask = line_mask)

        if(cy < 10 or cz > 130):
            # detect the stop zone
            # recrop to view more of red region.
            crop_width = 800
            crop_height = int(height / 2)    
            crop_z0 = height - crop_height - 10
            crop_z1 = crop_z0 + crop_height
            cropped_img = cv_img[
                crop_z0:crop_z1, crop_y0:crop_y1
            ]
            hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
            
            lower_threshold_red = (0, 224, 100)
            upper_threshold_red = (10, 255, 255)
            mask_red = cv2.inRange(hsv_img, lower_threshold_red, upper_threshold_red)
            res = cv2.bitwise_and(cropped_img, cropped_img, mask = mask_red)

            m = cv2.moments(mask_red)            
            cz = m['m01'] / (m['m00'] + 1e-5) 
            cy = m['m10'] / (m['m00'] + 1e-5)

            cv2.circle(res, (int(cy), int(cz)), 10, (255, 0, 0), 2)
            cv2.imshow("filtered image", res)
            cv2.waitKey(1)
            
            ang_vel = 0.0
            if(cz > 10 and cz < 400 ):
                # red region ahead. 
                print(f"Stop Zone Ahead (cz = {cz:.1f} pixels)...")
                fwd_vel = 0.1
                self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
                self.robot_controller.publish()
            else:
                # reached mid of red zone or no red found. 
                fwd_vel = 0.0
                self.robot_controller.stop()
                print("Robot stopped")
        else:
            # follow the pink line.
            cv2.circle(res, (int(cy), int(cz)), 10, (255, 0, 0), 2)
            cv2.imshow("filtered image", res)
            cv2.waitKey(1)
            
            reference_input = width / 2
            feedback_signal = cy
            error = feedback_signal - reference_input
            
            kp = -0.0025

            fwd_vel = 0.2
            ang_vel = kp * error

            if ang_vel < -1.82:
                ang_vel = -1.82
            elif ang_vel > 1.82:
                ang_vel = 1.82

            print(f"Error = {error:.1f} pixels | Control Signal = {ang_vel:.2f} rad/s")
            
            self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
            self.robot_controller.publish()

    def main(self):
        while not self.ctrl_c:
            self.rate.sleep()

if __name__ == '__main__':
    node = LineFollower()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass