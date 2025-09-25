#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys, math, tf, cv2, numpy as np
import moveit_commander
from moveit_commander import MoveGroupCommander
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Robot_Control:
    def __init__(self):
        self.object = []
        self.cam_pose1 = []
        self.boxes = []
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.pos = PointStamped()
        self.object_box = []
        self.posit = []
        self.rgb_img = None
        self.depth_img = None

        # 相机内参（请按实际相机更新）
        self.fx = 589.3664541825391
        self.cx = 320.0
        self.cy = 240.0

        moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
        rospy.on_shutdown(moveit_commander.roscpp_shutdown)

        self.arm = MoveGroupCommander("arm")
        self.gripper = MoveGroupCommander("gripper")

        # 运动学/动态学参数
        self.arm.set_goal_position_tolerance(0.0005)
        self.arm.set_goal_orientation_tolerance(0.0005)
        self.arm.set_max_acceleration_scaling_factor(0.9)
        self.arm.set_max_velocity_scaling_factor(0.9)

        self.gripper.set_goal_position_tolerance(0.01)
        self.gripper.set_goal_orientation_tolerance(0.05)
        self.gripper.set_max_acceleration_scaling_factor(0.9)
        self.gripper.set_max_velocity_scaling_factor(0.9)

        # 订阅图像（如有对齐的深度，建议订阅对齐话题）
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.rgb_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback, queue_size=1)

        # 初始关节位姿
        self.joint = [math.radians(0), math.radians(0), math.radians(-90), math.radians(0), math.radians(90), math.radians(0)]
        self.arm.set_joint_value_target(self.joint)
        self.arm.go(wait=True)

        # 准备位姿（相对当前位姿的微调只做一次）
        target_pose = self.arm.get_current_pose()
        target_pose.header.frame_id = 'base'
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.z -= 0.214
        target_pose.pose.position.y += 0.05
        target_pose.pose.position.x += 0.03
        self.arm.set_pose_target(target_pose)
        self.arm.go(wait=True)

        rospy.sleep(1.0)
        rospy.loginfo("arm is ready")

    # 颜色识别与排序：蓝→黄→黑→白（球体→圆柱体→正方体→白色方块）
    def sort_boxes(self):
        rospy.loginfo("Sorting boxes...")
        # 等待第一帧识别结果
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and self.posit == []:
            rate.sleep()

        positions = list(self.posit)

        priority = {
            '球体': 0,      # 蓝色
            '圆柱体': 1,    # 黄色
            '正方体': 2,    # 黑色
            '白色方块': 3,  # 白色
        }
        positions = sorted(positions, key=lambda p: priority.get(p[2], 99))

        for center_x, center_y, label in positions:
            self.object_box.append(label)
            self.object.append([center_x, center_y])

        self.calc_pose()

    def calc_pose(self):
        # 两段式高度
        approach_offset = 0.12   # 接近高度：物体上方 12 cm
        grasp_offset_default = 0.01  # 着落高度：高于表面约 1 cm

        for i in range(len(self.object)):
            # 开爪
            self.gripper.set_joint_value_target([0, 0, 0, 0, math.radians(25), 0])
            self.gripper.go(wait=True)

            x_pix = int(self.object[i][0])
            y_pix = int(self.object[i][1])

            # 用稳健的深度
            z = self.get_depth(x_pix, y_pix)
            if not np.isfinite(z) or z <= 0:
                rospy.logwarn("Depth invalid at {},{} -> skip this object".format(x_pix, y_pix))
                continue

            # 像素坐标 -> 相机坐标
            Xc = (x_pix - self.cx) * z / self.fx
            Yc = (y_pix - self.cy) * z / self.fx
            Zc = z

            # 填入 PointStamped（相机坐标系）
            self.pos.header.frame_id = 'camera_rgb_optical_frame'
            self.pos.header.stamp = rospy.Time(0)  # 让 TF 找最近的
            self.pos.point.x = Xc
            self.pos.point.y = Yc
            self.pos.point.z = Zc

            # 变换到 base
            try:
                self.listener.waitForTransform('base', self.pos.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
                point_tool = self.listener.transformPoint('base', self.pos)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("TF transform failed: {}".format(str(e)))
                continue

            # 形状补偿（黑/长方体 y 向微调；黑色方块再做轻微下压补偿）
            label = self.object_box[i]
            px = point_tool.point.x
            py = point_tool.point.y
            pz = point_tool.point.z

            if label in ('长方体', '正方体'):
                py -= 0.02  # 爪子居中补偿

            grasp_offset = grasp_offset_default
            pz_for_grasp = pz
            if label == '正方体':
                grasp_offset = 0.005         # 更贴近
                pz_for_grasp = pz - 0.005    # 抵消黑色表面深度偏小

            # 1) 先到接近高度（正上方）
            target_pose = self.arm.get_current_pose()
            target_pose.header.frame_id = 'base'
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = px
            target_pose.pose.position.y = py
            target_pose.pose.position.z = pz_for_grasp + approach_offset
            self.arm.set_pose_target(target_pose)
            self.arm.go(wait=True)

            # 2) 直线着落到抓取高度
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.z = pz_for_grasp + grasp_offset
            self.arm.set_pose_target(target_pose)
            self.arm.go(wait=True)

            # 3) 夹取
            self.gripper.set_joint_value_target([0, 0, 0, 0, math.radians(11.5), 0])
            self.gripper.go(wait=True)

            # 4) 抬起回到接近高度
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.z = pz_for_grasp + approach_offset
            self.arm.set_pose_target(target_pose)
            self.arm.go(wait=True)

            # 5) 搬运到放置区
            place = self.arm.get_current_pose()
            place.header.frame_id = 'base'
            place.header.stamp = rospy.Time.now()
            place.pose.position.y = -0.1
            place.pose.position.x = 0.13 + i * 0.04
            # 放置区高度：直接用当前抬起高度，随后再下放
            self.arm.set_pose_target(place)
            self.arm.go(wait=True)

            # 下放一点再松爪
            place.header.stamp = rospy.Time.now()
            place.pose.position.z = max(0.02, place.pose.position.z - 0.10)
            self.arm.set_pose_target(place)
            self.arm.go(wait=True)

            # 6) 松爪
            self.gripper.set_joint_value_target([0, 0, 0, 0, math.radians(45), 0])
            self.gripper.go(wait=True)

            # 7) 回初始位姿
            self.arm.set_joint_value_target(self.joint)
            self.arm.go(wait=True)

            # 8) 回到准备位
            ready_pose = self.arm.get_current_pose()
            ready_pose.header.frame_id = 'base'
            ready_pose.header.stamp = rospy.Time.now()
            ready_pose.pose.position.z -= 0.214
            ready_pose.pose.position.y += 0.05
            ready_pose.pose.position.x += 0.03
            self.arm.set_pose_target(ready_pose)
            self.arm.go(wait=True)

        # 结束
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    # —— 颜色检测：与你原版一致（略微整理） ——
    def detect_color_squares(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        images = image.copy()

        blue_lower,  blue_upper  = np.array([118, 254, 254]), np.array([122, 255, 255])
        black_lower, black_upper = np.array([0, 0, 0]),      np.array([180, 255, 80])
        white_lower, white_upper = np.array([0, 0, 200]),    np.array([180, 50, 255])
        yellow_lower,yellow_upper= np.array([20, 150, 150]), np.array([40, 255, 255])

        blue_mask   = cv2.inRange(hsv, blue_lower, blue_upper)
        black_mask  = cv2.inRange(hsv, black_lower, black_upper)
        white_mask  = cv2.inRange(hsv, white_lower, white_upper)
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        positions = []

        # 蓝 -> '球体'
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            if w * h > 100:
                cx, cy = x + w // 2, y + h // 2
                positions.append((cx, cy, '球体'))
                cv2.rectangle(images, (x, y), (x+w, y+h), (255, 0, 0), 2)
                cv2.putText(images, "blue", (x + w//2 - 20, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2)

        # 黑 -> '正方体'
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            if w * h > 680 and abs(w - h) < 8:
                cx, cy = x + w // 2, y + h // 2
                positions.append((cx, cy, '正方体'))
                cv2.rectangle(images, (x, y), (x+w, y+h), (0, 0, 0), 2)
                cv2.putText(images, "black", (x + w//2 - 25, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 2)

        # 白 -> '白色方块'
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            if w * h > 1850:
                cx, cy = x + w // 2, y + h // 2
                positions.append((cx, cy, '白色方块'))
                cv2.rectangle(images, (x, y), (x+w, y+h), (255, 255, 255), 2)
                cv2.putText(images, "white", (x + w//2 - 25, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)

        # 黄 -> '圆柱体'
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            if w * h > 100:
                cx, cy = x + w // 2, y + h // 2
                positions.append((cx, cy, '圆柱体'))
                cv2.rectangle(images, (x, y), (x+w, y+h), (0, 255, 255), 2)
                cv2.putText(images, "yellow", (x + w//2 - 30, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

        cv2.imshow("image", images)
        cv2.waitKey(1)
        return positions

    # —— 稳健深度：邻域非零中位数 ——
    def get_depth(self, x, y):
        if self.depth_img is None:
            return 0.0
        h, w = self.depth_img.shape[:2]
        x0, x1 = max(0, x-10), min(w-1, x+10)
        y0, y1 = max(0, y-10), min(h-1, y+10)
        patch = self.depth_img[y0:y1+1, x0:x1+1]
        nonzero = patch[patch > 0]
        if nonzero.size == 0:
            return 0.0
        return float(np.median(nonzero))

    def rgb_callback(self, data):
        try:
            self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.posit = self.detect_color_squares(self.rgb_img)
        except CvBridgeError as e:
            rospy.logwarn("CvBridge rgb error: {}".format(str(e)))

    def depth_callback(self, data):
        try:
            # 注意：若是 16UC1 深度，桥接可直接得到以毫米/米为单位的 float/uint16
            self.depth_img = self.bridge.imgmsg_to_cv2(data, data.encoding)
            # 若为 16UC1，需要转米：self.depth_img = self.depth_img.astype(np.float32) / 1000.0
        except CvBridgeError as e:
            rospy.logwarn("CvBridge depth error: {}".format(str(e)))

if __name__ == '__main__':
    try:
        rospy.init_node('robot', anonymous=True)
        robot = Robot_Control()
        robot.sort_boxes()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

