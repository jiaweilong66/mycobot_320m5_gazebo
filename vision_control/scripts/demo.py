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
    self.arm.set_goal_position_tolerance(0.0005)
    self.arm.set_goal_orientation_tolerance(0.0005)
    self.arm.set_max_acceleration_scaling_factor(0.9)
    self.arm.set_max_velocity_scaling_factor(0.9)
    
    self.gripper.set_goal_position_tolerance(0.01)
    self.gripper.set_goal_orientation_tolerance(0.05)
    self.gripper.set_max_acceleration_scaling_factor(0.9)
    self.gripper.set_max_velocity_scaling_factor(0.9)
    ###定义话题接收回调函数
    self.joint = [math.radians(0), math.radians(0), math.radians(-90), math.radians(-0), math.radians(90), math.radians(0)]#初始化关节角度
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
    
  def sort_boxes(self):
    self.calc_pose()
  def calc_pose(self):
    self.cam_pose1.append([0.23,0.05,0.23]) #计算相机坐标
    self.cam_pose1.append([0.18,0.05,0.23]) #计算相机坐标
    self.cam_pose1.append([0.15,0.13,0.25]) #计算相机坐标
    self.cam_pose1.append([0.22,0.13,0.23]) #计算相机坐标
    for i in range(len(self.cam_pose1)):
      
      self.gripper.set_joint_value_target([0,0,0,0,math.radians(25),0])
      self.gripper.go()
      #先X、Y对准，在Z向移动，防止与波罗发生碰撞
      target_pose = self.arm.get_current_pose()
      target_pose.header.frame_id = 'base'
      target_pose.header.stamp = rospy.Time.now()    
      target_pose.pose.position.x = self.cam_pose1[i][0]
      target_pose.pose.position.y = self.cam_pose1[i][1]-0.05
      target_pose.pose.position.z -= 0.215
      print("基座下目标坐标",target_pose.pose.position)
      # 对准
      self.arm.set_pose_target(target_pose)
      self.arm.go(wait=True)
      target_pose.header.stamp = rospy.Time.now()     
      target_pose.pose.position.z = self.cam_pose1[i][2]-0.215+0.16
      # 靠近
      self.arm.set_pose_target(target_pose)
      self.arm.go(wait=True)
      # 抓取
      self.gripper.set_joint_value_target([0,0,0,0,math.radians(11.5),0])
      self.gripper.go()
      target_pose.header.frame_id = 'base'
      target_pose.header.stamp = rospy.Time.now()     
      target_pose.pose.position.z += 0.1
      # 上提
      self.arm.set_pose_target(target_pose)
      self.arm.go(wait=True)
      # 移动到目标位置
      target_pose = self.arm.get_current_pose()
      target_pose.header.frame_id = 'base'
      target_pose.header.stamp = rospy.Time.now()  
      target_pose.pose.position.y = -0.1
      target_pose.pose.position.x = 0.13 + i*0.04
      target_pose.pose.position.z -= 0.215
      # 移动到目标位置
      self.arm.set_pose_target(target_pose)
      self.arm.go()
      self.gripper.set_joint_value_target([0,0,0,0,math.radians(45),0])
      self.gripper.go()
      self.arm.set_joint_value_target(self.joint)
      self.arm.go(wait=True)
     
      
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
  def detect_color_squares(self,image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    images = image.copy()
    # 定义颜色范围
    blue_lower = np.array([118, 254, 254])
    blue_upper = np.array([122, 255, 255])
    red_lower1 = np.array([0, 254, 254])
    red_upper1 = np.array([2, 255, 255])
    red_lower2 = np.array([0, 254, 254])
    red_upper2 = np.array([2, 255, 255])
    green_lower = np.array([58, 254, 254])
    green_upper = np.array([62, 255, 255])
    purple_lower = np.array([133, 254, 202])
    purple_upper = np.array([137, 255, 206])

    # 创建掩码
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    purple_mask = cv2.inRange(hsv, purple_lower, purple_upper)

    # 查找轮廓
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_contours, _ = cv2.findContours(red_mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    purple_contours, _ = cv2.findContours(purple_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    positions = []

    # # 处理蓝色方块
    for contour in blue_contours:
        x, y, w, h = cv2.boundingRect(contour)
        if w * h > 100:  # 过滤小的检测结果
            center_x = x + w // 2
            center_y = y + h // 2
            positions.append((center_x, center_y, '球体'))
            cv2.rectangle(images, (x, y), (x + w, y + h), (255, 0, 0), 2)

    # # 处理红色方块
    for contour in red_contours:
        x, y, w, h = cv2.boundingRect(contour)
        if w * h > 100:  # 过滤小的检测结果
            center_x = x + w // 2
            center_y = y + h // 2
            positions.append((center_x, center_y, '长方体'))
            cv2.rectangle(images, (x, y), (x + w, y + h), (255, 0, 0), 2)

    # 处理绿色方块
    for contour in green_contours:
        x, y, w, h = cv2.boundingRect(contour)
        if w * h > 100:  # 过滤小的检测结果
            center_x = x + w // 2
            center_y = y + h // 2
            positions.append((center_x, center_y, '正方体'))
            cv2.rectangle(images, (x, y), (x + w, y + h), (255, 0, 0), 2)
    #  # 处理紫色方块
    for contour in purple_contours:
        x, y, w, h = cv2.boundingRect(contour)
        if w * h > 100:  # 过滤小的检测结果
            center_x = x + w // 2
            center_y = y + h // 2
            positions.append((center_x, center_y, '圆柱体'))
            cv2.rectangle(images, (x, y), (x + w, y + h), (255, 0, 0), 2)
    cv2.imshow("image", images)
    cv2.waitKey(1)
    return positions
  def get_depth(self,x,y):
    ##多点取平均值
    s = []
    for item in range(-10,10):
      for jtem in range(-10,10):
        if self.depth_img[jtem + y,item + x] != 0:
          s.append(self.depth_img[jtem + y,item + x])
      return self.depth_img[0 + y,0 + x]
  def rgb_callback(self,data):
    self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    self.posit = self.detect_color_squares(self.rgb_img)
  def depth_callback(self,data):
    self.depth_img = self.bridge.imgmsg_to_cv2(data, data.encoding)
if __name__ == '__main__':
    try:
      rospy.init_node('robot', anonymous=True)
      robot = Robot_Control()
      # input('continue')
      # rospy.sleep(1)
      robot.sort_boxes()
      # rospy.sleep(1)
      rospy.on_shutdown()
    except rospy.ROSInterruptException:
        pass