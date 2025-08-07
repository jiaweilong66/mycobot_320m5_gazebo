#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This file obtains the joint angle of the MyCobot320 manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control.launch] related script for MyCobot320.
Passable parameters:
    port: serial port string. Defaults is auto-detected or '/dev/ttyUSB0'
    baud: serial port baudrate. Defaults is 115200.
"""

import math
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import serial.tools.list_ports
from pymycobot import MyCobot320

# 全局变量
mc = None
pub_arm_command = None
pub_gripper_command = None

# 上一次有效映射值
last_mapped_gripper = None
last_valid_angles = None

# 夹爪映射常量
GRIPPER_MIN_ANGLE = 0      
GRIPPER_MAX_ANGLE = 100    
GAZEBO_MIN_POSITION = -60.0  
GAZEBO_MAX_POSITION = 60.0   

# 夹爪ID
GRIPPER_ID = 14

def find_available_port():
    """自动检测可用串口，优先选择USB转串口设备"""
    ports = serial.tools.list_ports.comports()
    
    # 优先级关键词列表（从高到低）
    priority_keywords = ['ACM', 'USB', 'Arduino', 'CH340', 'CP210', 'FTDI']
    
    # 按优先级查找串口
    for keyword in priority_keywords:
        for port in ports:
            if (keyword in port.device.upper() or 
                keyword in port.description.upper() or 
                keyword in str(port.hwid).upper()):
                rospy.loginfo(f"找到优先串口: {port.device} ({port.description})")
                return port.device
    
    # 如果没找到优先设备，返回第一个可用串口
    if ports:
        selected_port = ports[0].device
        rospy.loginfo(f"使用第一个可用串口: {selected_port} ({ports[0].description})")
        return selected_port
    
    # 没有找到任何串口
    rospy.logwarn("未找到任何可用串口，使用默认值")
    return "/dev/ttyUSB0"

def list_available_ports():
    """列出所有可用串口信息"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        rospy.loginfo("没有找到可用串口")
        return
    
    rospy.loginfo("=== 可用串口列表 ===")
    for i, port in enumerate(ports):
        rospy.loginfo(f"{i+1}. 设备: {port.device}")
        rospy.loginfo(f"   描述: {port.description}")
        rospy.loginfo(f"   硬件ID: {port.hwid}")
        rospy.loginfo("   ---")

def is_valid_gripper_angle(angle):
    """检查夹爪角度是否有效"""
    if angle is None:
        return False
    if isinstance(angle, (int, float)):
        # 检查是否在合理范围内，排除异常值
        if 0 <= angle <= 100:
            return True
        # 排除明显的异常值
        if angle == 65535 or angle == -1 or angle < -100 or angle > 1000:
            return False
    return False

def callback(data):
    """ROS回调函数，处理关节状态消息"""
    global last_mapped_gripper, last_valid_angles

    # MyCobot320关节名称（根据实际URDF文件调整）
    expected_arm_joint_names = [
        "joint2_to_joint1",
        "joint3_to_joint2", 
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "joint6output_to_joint6"
    ]
    expected_gripper_joint_names = ["gripper_controller"]

    # 获取真实机械臂关节角度（增强重试机制）
    angles = None
    for retry in range(5):  # 增加重试次数
        try:
            angles = mc.get_angles()
            if angles is not None and isinstance(angles, (list, tuple)) and len(angles) == 6:
                # 检查角度是否合理
                if all(isinstance(a, (int, float)) and -180 <= a <= 180 for a in angles):
                    last_valid_angles = angles[:]  # 保存有效角度
                    break
            time.sleep(0.02)  # 短暂等待后重试
        except Exception as e:
            time.sleep(0.02)
            continue
    
    # 如果获取失败，使用上次有效值
    if angles is None or not isinstance(angles, (list, tuple)) or len(angles) != 6:
        if last_valid_angles is not None:
            angles = last_valid_angles
            rospy.logwarn_throttle(2, "使用上次有效关节角度")
        else:
            rospy.logwarn_throttle(2, "无法获取关节角度，跳过此次更新")
            return

    # 获取夹爪角度（更稳定的读取方式）
    gripper_angle = None
    valid_gripper_found = False
    
    # 尝试多次读取夹爪角度
    for retry in range(3):
        try:
            raw_angle = mc.get_pro_gripper_angle(GRIPPER_ID)
            if is_valid_gripper_angle(raw_angle):
                gripper_angle = raw_angle
                valid_gripper_found = True
                break
            time.sleep(0.05)
        except:
            time.sleep(0.05)
            continue
    
    # 如果没有获取到有效的夹爪角度，使用上次的值
    if not valid_gripper_found:
        if last_mapped_gripper is not None:
            # 从映射值反推回原始角度用于显示
            gripper_angle = ((last_mapped_gripper - GAZEBO_MIN_POSITION) / 
                           (GAZEBO_MAX_POSITION - GAZEBO_MIN_POSITION)) * \
                          (GRIPPER_MAX_ANGLE - GRIPPER_MIN_ANGLE) + GRIPPER_MIN_ANGLE
        else:
            gripper_angle = 50  # 默认中间值

    # 验证并过滤joint_states数据
    filtered_arm_positions = []
    filtered_gripper_positions = []
    
    for name, pos in zip(data.name, data.position):
        if name in expected_arm_joint_names:
            filtered_arm_positions.append(round(math.degrees(pos), 2))
        elif name in expected_gripper_joint_names:
            filtered_gripper_positions.append(round(math.degrees(pos), 2))

    # 验证数据长度
    if len(filtered_arm_positions) != len(expected_arm_joint_names):
        rospy.logerr(f"接收到{len(filtered_arm_positions)}个机械臂位置，期望{len(expected_arm_joint_names)}个")
        return
    if len(filtered_gripper_positions) != len(expected_gripper_joint_names):
        rospy.logerr(f"接收到{len(filtered_gripper_positions)}个夹爪位置，期望{len(expected_gripper_joint_names)}个")
        return

    # ARM控制：角度转换成弧度列表
    arm_angles_rad = [v * (math.pi / 180) for v in angles]

    # 发布机械臂轨迹到Gazebo
    arm_traj = JointTrajectory()
    arm_traj.header.stamp = rospy.Time.now()
    arm_traj.joint_names = expected_arm_joint_names
    
    pt = JointTrajectoryPoint()
    pt.positions = arm_angles_rad
    pt.velocities = [0.0] * len(expected_arm_joint_names)
    pt.accelerations = [0.0] * len(expected_arm_joint_names)
    pt.time_from_start = rospy.Duration(0.5)
    arm_traj.points.append(pt)
    
    pub_arm_command.publish(arm_traj)

    # GRIPPER控制：处理夹爪角度映射
    if valid_gripper_found and is_valid_gripper_angle(gripper_angle):
        # 正常映射
        mapped_gripper = ((gripper_angle - GRIPPER_MIN_ANGLE) /
                         (GRIPPER_MAX_ANGLE - GRIPPER_MIN_ANGLE)) * \
                        (GAZEBO_MAX_POSITION - GAZEBO_MIN_POSITION) + GAZEBO_MIN_POSITION
        
        # 限制范围
        mapped_gripper = max(GAZEBO_MIN_POSITION, min(GAZEBO_MAX_POSITION, mapped_gripper))
        
        # 更新有效映射值
        last_mapped_gripper = mapped_gripper
        
        print(f"Valid gripper: {gripper_angle} -> {mapped_gripper:.1f}°")
    else:
        # 使用上次有效值或默认值
        if last_mapped_gripper is not None:
            mapped_gripper = last_mapped_gripper
        else:
            mapped_gripper = GAZEBO_MIN_POSITION  # 默认关闭状态
        
        print(f"Invalid gripper data, using last valid: {mapped_gripper:.1f}°")

    # 发布夹爪轨迹到Gazebo
    gripper_traj = JointTrajectory()
    gripper_traj.header.stamp = rospy.Time.now()
    gripper_traj.joint_names = expected_gripper_joint_names
    
    gp = JointTrajectoryPoint()
    gp.positions = [math.radians(mapped_gripper)]  # 转换为弧度
    gp.velocities = [0.0]
    gp.accelerations = [0.0]
    gp.time_from_start = rospy.Duration(0.5)
    gripper_traj.points.append(gp)
    
    pub_gripper_command.publish(gripper_traj)

def initialize_gripper():
    """简化的夹爪初始化"""
    try:
        rospy.loginfo("尝试初始化夹爪...")
        
        # 检查夹爪是否存在
        version = mc.get_pro_gripper(GRIPPER_ID, 1)
        if version == -1:
            rospy.logwarn("未检测到Pro力控夹爪，跳过夹爪初始化")
            return False
        
        rospy.loginfo("Pro力控夹爪检测成功，跳过复杂初始化避免冲突")
        return True
        
    except Exception as e:
        rospy.logwarn(f"夹爪初始化失败: {e}")
        return False

def listener():
    """主函数"""
    global mc, pub_arm_command, pub_gripper_command

    rospy.init_node("mycobot320_gazebo_sync", anonymous=True)
    
    # 显示所有可用串口
    list_available_ports()
    
    # 获取串口参数
    port = rospy.get_param("~port", find_available_port())
    baud = rospy.get_param("~baud", 115200)
    
    rospy.loginfo(f"使用串口: {port}, 波特率: {baud}")

    try:
        # 初始化MyCobot320连接
        mc = MyCobot320(port, baud)
        rospy.loginfo("MyCobot320连接成功")
        
        # 完全释放舵机，不重新激活
        rospy.loginfo("释放所有舵机，保持可手动操作状态...")
        mc.release_all_servos()
        time.sleep(1)
        rospy.loginfo("舵机已释放，现在可以手动摆动机械臂")
        
        # 简化夹爪初始化
        gripper_initialized = initialize_gripper()
        if gripper_initialized:
            rospy.loginfo("夹爪检测成功")
        else:
            rospy.logwarn("未检测到夹爪或初始化失败")
        
    except Exception as e:
        rospy.logerr(f"MyCobot320连接失败: {e}")
        rospy.logerr("请检查:")
        rospy.logerr("1. 机械臂是否正确连接到电脑")
        rospy.logerr("2. 串口权限是否正确 (sudo chmod 666 /dev/ttyACM0)")
        rospy.logerr("3. 是否有其他程序占用串口")
        rospy.logerr("4. 机械臂是否上电")
        return

    # 初始化ROS发布者和订阅者
    pub_arm_command = rospy.Publisher("/arm_controller/command",
                                      JointTrajectory, queue_size=10)
    pub_gripper_command = rospy.Publisher("/gripper_controller/command",
                                          JointTrajectory, queue_size=10)
    rospy.Subscriber("/joint_states", JointState, callback)

    rospy.loginfo("MyCobot320 Gazebo同步控制节点已启动")
    rospy.loginfo("开始同步真实机械臂状态到Gazebo仿真...")
    rospy.loginfo(f"夹爪映射范围: {GAZEBO_MIN_POSITION}° 到 {GAZEBO_MAX_POSITION}°")
    rospy.loginfo("✅ 舵机保持释放状态，您可以手动摆动机械臂")
    
    # 注册退出处理函数
    def cleanup():
        if mc is not None:
            try:
                rospy.loginfo("程序退出，确保舵机释放...")
                mc.release_all_servos()
                rospy.loginfo("已释放所有舵机")
            except:
                pass
    
    rospy.on_shutdown(cleanup)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("接收到退出信号，正在关闭...")
    finally:
        cleanup()

if __name__ == "__main__":
    listener()
