#!/usr/bin/env python3
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import tf
import math
import cv2
import numpy as np

class Robot_Control:
    def __init__(self):
        self.object = []
        self.cam_pose1 = []
        self.boxes = []
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.pos = PointStamped()
        self.object_box = []
        moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
        self.posit = []
        self.arm = MoveGroupCommander("arm")
        self.gripper = MoveGroupCommander("gripper")

        self.fx = 589.3664541825391  # 相机内参：焦距 fx，注意根据实际相机调整

        self.arm.set_goal_position_tolerance(0.0005)
        self.arm.set_goal_orientation_tolerance(0.0005)
        self.arm.set_max_acceleration_scaling_factor(0.9)
        self.arm.set_max_velocity_scaling_factor(0.9)
       
        self.gripper.set_goal_position_tolerance(0.01)
        self.gripper.set_goal_orientation_tolerance(0.05)
        self.gripper.set_max_acceleration_scaling_factor(0.9)
        self.gripper.set_max_velocity_scaling_factor(0.9)
       
        # ROS subscribers to receive camera images
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.rgb_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback, queue_size=1)
       
        self.joint = [math.radians(0), math.radians(0), math.radians(-90), math.radians(-0), math.radians(90), math.radians(0)]  # Initial joint angles
        self.arm.set_joint_value_target(self.joint)
        self.arm.go(wait=True)

        target_pose = self.arm.get_current_pose()
        print(target_pose)
        target_pose.header.frame_id = 'base'
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.z -= 0.214
        target_pose.pose.position.y += 0.05
        target_pose.pose.position.x += 0.03
        self.arm.set_pose_target(target_pose)
        self.arm.go(wait=True)
       
        rospy.sleep(1)
        print("arm is ready")

    # Define sort_boxes method
    def sort_boxes(self):
        print("Sorting boxes...")
        while self.posit == []:
            pass
        positions = self.posit
        print(positions)
        for center_x, center_y, label in positions:
            print(center_x, center_y, label)
            self.object_box.append(label)
            self.object.append([center_x, center_y])
        self.calc_pose()

    def calc_pose(self):
        fx = 589.3664541825391  # Camera intrinsic parameter
        for i in range(len(self.object)):
            x = int(self.object[i][0])  # Get detected object center x-coordinate
            y = int(self.object[i][1])  # Get detected object center y-coordinate
            z = self.get_depth(x, y)  # Get depth value
            self.cam_pose1.append([(x - 320) * z / fx, (y - 240) * z / fx, z])  # Calculate camera coordinates
            print("Camera coordinates:", self.cam_pose1[i])

        for i in range(len(self.cam_pose1)):
            self.gripper.set_joint_value_target([0, 0, 0, 0, math.radians(25), 0])
            self.gripper.go()
           
            # Set position in camera frame
            self.pos.header.frame_id = 'camera_rgb_optical_frame'
            self.pos.point.x, self.pos.point.y, self.pos.point.z = self.cam_pose1[i][0], self.cam_pose1[i][1], self.cam_pose1[i][2]
            print("Camera coordinates:", self.pos.point)
           
            # Transform camera coordinates to robot base frame
            self.point_tool = self.listener.transformPoint('base', self.pos)
            print("Base coordinates:", self.point_tool)

            target_pose = self.arm.get_current_pose()
            target_pose.header.frame_id = 'base'
            target_pose.header.stamp = rospy.Time.now()

            if self.object_box[i] == '长方体':  # If object is a cube
                self.point_tool.point.y -= 0.02  # Adjust for the cube's shape
           
            target_pose.pose.position.x = self.point_tool.point.x
            target_pose.pose.position.y = self.point_tool.point.y
            target_pose.pose.position.z -= 0.215
            print("Target coordinates:", target_pose.pose.position)
           
            self.arm.set_pose_target(target_pose)
            self.arm.go(wait=True)
           
            target_pose.header.stamp = rospy.Time.now()
            if self.object_box[i] == '长方体' or self.object_box[i] == '球体':
                self.point_tool.point.z += 0.01   
            target_pose.pose.position.z = self.point_tool.point.z + 0.16
           
            # Move closer to the object
            self.arm.set_pose_target(target_pose)
            self.arm.go(wait=True)

            # Grasping operation
            self.gripper.set_joint_value_target([0, 0, 0, 0, math.radians(11.5), 0])
            self.gripper.go()
           
            target_pose.header.frame_id = 'base'
            target_pose.header.stamp = rospy.Time.now()     
            target_pose.pose.position.z += 0.1  # Lift object
            self.arm.set_pose_target(target_pose)
            self.arm.go(wait=True)
           
            # Move object to target location
            target_pose = self.arm.get_current_pose()
            target_pose.header.frame_id = 'base'
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.y = -0.1
            target_pose.pose.position.x = 0.13 + i * 0.04
            target_pose.pose.position.z -= 0.215
            self.arm.set_pose_target(target_pose)
            self.arm.go()

            # Open gripper after placing object
            self.gripper.set_joint_value_target([0, 0, 0, 0, math.radians(45), 0])
            self.gripper.go()
           
            # Return arm to initial position
            self.arm.set_joint_value_target(self.joint)
            self.arm.go(wait=True)
           
            target_pose = self.arm.get_current_pose()
            target_pose.header.frame_id = 'base'
            target_pose.header.stamp = rospy.Time.now()     
            target_pose.pose.position.z -= 0.214
            target_pose.pose.position.y += 0.05
            target_pose.pose.position.x += 0.03
            self.arm.set_pose_target(target_pose)
            self.arm.go(wait=True)
       
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def detect_color_squares(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        images = image.copy()

        # Define color ranges
        blue_lower = np.array([118, 254, 254])
        blue_upper = np.array([122, 255, 255])
        black_lower = np.array([0, 0, 0])  # Expanded black range
        black_upper = np.array([180, 255, 80])  # Expanded black range for higher value
        white_lower = np.array([0, 0, 200])  # White color range
        white_upper = np.array([180, 50, 255])  # White upper range
        yellow_lower = np.array([20, 150, 150])
        yellow_upper = np.array([40, 255, 255])

        # Create masks for the colors
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
        black_mask = cv2.inRange(hsv, black_lower, black_upper)  # Added black mask
        white_mask = cv2.inRange(hsv, white_lower, white_upper)  # Added white mask
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)  # Added yellow mask

        # Find contours for each color
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        black_contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        white_contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        positions = []

        # Process blue square
        for contour in blue_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w * h > 100:  # Filter small detections
                center_x = x + w // 2
                center_y = y + h // 2
                positions.append((center_x, center_y, '球体'))
                cv2.rectangle(images, (x, y), (x + w, y + h), (255, 0, 0), 2)
                label = "blue"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.8
                color = (255, 0, 0)
                thickness = 2
                text_size = cv2.getTextSize(label, font, font_scale, thickness)[0]
                text_x = x + (w - text_size[0]) // 2
                text_y = y - 10
                cv2.putText(images, label, (text_x, text_y), font, font_scale, color, thickness)

        # Process black square (only detect 2.5 cm x 2.5 cm cubes)
        for contour in yellow_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w * h > 100:
                center_x = x + w // 2
                center_y = y + h // 2
                positions.append((center_x, center_y, '圆柱体'))
                cv2.rectangle(images, (x, y), (x + w, y + h), (0, 255, 255), 2)
                label = "yellow"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.8
                color = (0, 255, 255)
                thickness = 2
                text_size = cv2.getTextSize(label, font, font_scale, thickness)[0]
                text_x = x + (w - text_size[0]) // 2
                text_y = y - 10
                cv2.putText(images, label, (text_x, text_y), font, font_scale, color, thickness)
               
        for contour in black_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w * h > 900 and abs(w - h) < 10:  # Square condition: width and height are close
                center_x = x + w // 2
                center_y = y + h // 2
                positions.append((center_x, center_y, '正方体'))
                # Draw the rectangle
                cv2.rectangle(images, (x, y), (x + w, y + h), (0, 0, 0), 2)
                label = "black"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.8
                color = (0, 0, 0)  # Black color for text
                thickness = 2
                # Positioning the label slightly above the rectangle
                text_size = cv2.getTextSize(label, font, font_scale, thickness)[0]
                text_x = x + (w - text_size[0]) // 2  # Center the text horizontally
                text_y = y - 10  # Place text above the rectangle
                # Put the label text on the image
                cv2.putText(images, label, (text_x, text_y), font, font_scale, color, thickness)


        # Process white square
        for contour in white_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w * h > 1680:  # Filter small detections
                center_x = x + w // 2
                center_y = y + h // 2
                positions.append((center_x, center_y, '白色方块'))
                cv2.rectangle(images, (x, y), (x + w, y + h), (255, 255, 255), 2)  # White box
                label = "white"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.8
                color = (255, 255, 255)
                thickness = 2
                text_size = cv2.getTextSize(label, font, font_scale, thickness)[0]
                text_x = x + (w - text_size[0]) // 2
                text_y = y - 10
                cv2.putText(images, label, (text_x, text_y), font, font_scale, color, thickness)

        # Process yellow square       
        cv2.imshow("image", images)
        cv2.waitKey(1)
        return positions

    def get_depth(self, x, y):
        s = []
        for item in range(-10, 10):
            for jtem in range(-10, 10):
                if self.depth_img[jtem + y, item + x] != 0:
                    s.append(self.depth_img[jtem + y, item + x])
        return self.depth_img[0 + y, 0 + x]

    def rgb_callback(self, data):
        self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.posit = self.detect_color_squares(self.rgb_img)

    def depth_callback(self, data):
        self.depth_img = self.bridge.imgmsg_to_cv2(data, data.encoding)

if __name__ == '__main__':
    try:
        rospy.init_node('robot', anonymous=True)
        robot = Robot_Control()
        robot.sort_boxes()
        rospy.on_shutdown()
    except rospy.ROSInterruptException:
        pass
