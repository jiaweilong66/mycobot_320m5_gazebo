#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slider_control_320.py 
双模式滑块控制脚本 - MyCobot320版本：
  1: 滑块 -> Gazebo 控制器  
  2: 滑块 -> 真实 MyCobot320 机械臂
使用异步执行和频率控制优化性能，减少卡顿
支持自动串口检测和智能端口选择
专为Pro力控夹爪优化，兼容状态控制备选方案
"""
import time
import math
import threading
import queue
from collections import deque
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import serial.tools.list_ports
from pymycobot import MyCobot320

# 全局变量
mc = None
mode = 2
pub_arm = None
pub_gripper = None

# 优化参数
ANGLE_THRESHOLD = 3.0           # 角度变化阈值(度)
GRIPPER_THRESHOLD = 5.0         # 夹爪角度变化阈值(度)  
MAX_COMMAND_RATE = 10.0         # 最大命令频率(Hz)
COMMAND_QUEUE_SIZE = 5          # 命令队列大小

# 安全角度限制 (度)
JOINT_LIMITS = [
    (-180, 180),  # joint1
    (-180, 180),  # joint2
    (-180, 180),  # joint3
    (-180, 180),  # joint4
    (-180, 180),  # joint5
    (-180, 180),  # joint6
]

GRIPPER_LIMITS = (-62, 62)  # 夹爪角度限制

# 夹爪配置 - MyCobot320 Pro力控夹爪
GRIPPER_ID = 14             # 确保是整数
GRIPPER_MIN_ANGLE = 0.0     # 夹爪最小角度（关闭）
GRIPPER_MAX_ANGLE = 100.0   # 夹爪最大角度（打开）
# Gazebo滑块的实际角度范围（根据实际测试结果）
GAZEBO_MIN_POSITION = 0.0    # Gazebo滑块最小位置（滑块=0，对应夹爪关闭0°）
GAZEBO_MAX_POSITION = 57.3   # Gazebo滑块最大位置（滑块=最大，对应夹爪打开100°）   

# 状态记录
last_angles = None
last_gripper_angle = None
last_command_time = 0
command_queue = queue.Queue(maxsize=COMMAND_QUEUE_SIZE)
is_executing = False

# 超限警告控制
last_warning_time = {}  # 每个关节的最后警告时间
WARNING_INTERVAL = 3.0  # 警告间隔(秒)

# 统计信息
stats = {
    'total_messages': 0,
    'commands_sent': 0,
    'commands_skipped': 0,
    'limit_violations': 0,
    'errors': 0
}

# 期望的关节名称
ARM_JOINTS = [
    "joint2_to_joint1",
    "joint3_to_joint2", 
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]
GRIPPER_JOINT = "gripper_controller"

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
                rospy.loginfo(f"[串口检测] 找到优先串口: {port.device} ({port.description})")
                return port.device
    
    # 如果没找到优先设备，返回第一个可用串口
    if ports:
        selected_port = ports[0].device
        rospy.loginfo(f"[串口检测] 使用第一个可用串口: {selected_port} ({ports[0].description})")
        return selected_port
    
    # 没有找到任何串口
    rospy.logwarn("[串口检测] 未找到任何可用串口，使用默认值")
    return "/dev/ttyUSB0"

def list_available_ports():
    """列出所有可用串口信息"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        rospy.loginfo("[串口检测] 没有找到可用串口")
        return
    
    rospy.loginfo("[串口检测] === 可用串口列表 ===")
    for i, port in enumerate(ports):
        rospy.loginfo(f"[串口检测] {i+1}. 设备: {port.device}")
        rospy.loginfo(f"[串口检测]    描述: {port.description}")
        rospy.loginfo(f"[串口检测]    硬件ID: {port.hwid}")
        rospy.loginfo("[串口检测]    ---")

def test_port_connectivity(port, baud):
    """测试串口连接性"""
    try:
        test_mc = MyCobot320(port, baud)
        time.sleep(1.0)  # 给设备时间初始化
        
        # 尝试获取角度来测试连接
        angles = test_mc.get_angles()
        test_mc.release_all_servos()
        
        rospy.loginfo(f"[串口测试] ✅ 端口 {port} 连接成功，当前角度: {angles}")
        return True
        
    except Exception as e:
        rospy.logwarn(f"[串口测试] ❌ 端口 {port} 连接失败: {e}")
        return False

def smart_port_selection():
    """智能串口选择：检测所有可用端口并测试连接性"""
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        rospy.logwarn("[智能选择] 未找到任何串口设备")
        return "/dev/ttyUSB0"
    
    rospy.loginfo("[智能选择] 开始智能串口选择...")
    
    # 定义优先级关键词
    priority_keywords = ['ACM', 'USB', 'Arduino', 'CH340', 'CP210', 'FTDI']
    
    # 首先尝试高优先级端口
    for keyword in priority_keywords:
        for port in ports:
            if (keyword in port.device.upper() or 
                keyword in port.description.upper() or 
                keyword in str(port.hwid).upper()):
                
                rospy.loginfo(f"[智能选择] 正在测试高优先级端口: {port.device}")
                if test_port_connectivity(port.device, 115200):
                    return port.device
    
    # 如果高优先级端口都失败，尝试所有端口
    rospy.loginfo("[智能选择] 高优先级端口测试失败，尝试所有可用端口...")
    for port in ports:
        rospy.loginfo(f"[智能选择] 正在测试端口: {port.device}")
        if test_port_connectivity(port.device, 115200):
            return port.device
    
    # 所有端口都失败
    rospy.logerr("[智能选择] 所有端口测试失败，使用默认端口")
    return "/dev/ttyUSB0"

class RobotCommand:
    """机器人命令类"""
    def __init__(self, cmd_type, data, timestamp=None):
        self.type = cmd_type  # 'angles' or 'gripper'
        self.data = data
        self.timestamp = timestamp or time.time()

def is_mycobot_connected():
    """检查MyCobot320是否连接"""
    global mc
    try:
        if mc is None:
            return False
        mc.get_angles()
        return True
    except Exception as e:
        return False

def check_angle_limits(angles, gripper_angle):
    """检查角度是否在安全范围内"""
    global last_warning_time, stats
    
    current_time = time.time()
    violations = []
    
    # 检查关节角度限制
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
        if angle < min_limit or angle > max_limit:
            joint_key = f"joint{i+1}"
            
            # 控制警告频率
            if joint_key not in last_warning_time or current_time - last_warning_time[joint_key] > WARNING_INTERVAL:
                violations.append(f"关节{i+1}: {angle:.1f}° (限制: {min_limit}°~{max_limit}°)")
                last_warning_time[joint_key] = current_time
                stats['limit_violations'] += 1
    
    # 检查夹爪角度限制
    min_grip, max_grip = GRIPPER_LIMITS
    if gripper_angle < min_grip or gripper_angle > max_grip:
        gripper_key = "gripper"
        
        if gripper_key not in last_warning_time or current_time - last_warning_time[gripper_key] > WARNING_INTERVAL:
            violations.append(f"夹爪: {gripper_angle:.1f}° (限制: {min_grip}°~{max_grip}°)")
            last_warning_time[gripper_key] = current_time
            stats['limit_violations'] += 1
    
    # 打印警告信息
    if violations:
        rospy.logwarn(f"[slider_control] ⚠️  角度超限:")
        for violation in violations:
            rospy.logwarn(f"[slider_control]    {violation}")
    
    return len(violations) == 0

def clamp_angles(angles, gripper_angle):
    """将角度限制在安全范围内"""
    # 限制关节角度
    clamped_angles = []
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
        clamped_angle = max(min_limit, min(max_limit, angle))
        clamped_angles.append(clamped_angle)
    
    # 限制夹爪角度
    min_grip, max_grip = GRIPPER_LIMITS
    clamped_gripper = max(min_grip, min(max_grip, gripper_angle))
    
    return clamped_angles, clamped_gripper

def calculate_angle_difference(angles1, angles2):
    """计算角度差异"""
    if angles1 is None or angles2 is None:
        return float('inf')
    return sum(abs(a - b) for a, b in zip(angles1, angles2))

def should_send_command(new_angles, new_gripper_angle):
    """判断是否应该发送命令"""
    global last_angles, last_gripper_angle, last_command_time
    
    current_time = time.time()
    
    # 频率限制
    if current_time - last_command_time < 1.0 / MAX_COMMAND_RATE:
        return False, "频率限制"
    
    # 角度变化检查
    angle_diff = calculate_angle_difference(new_angles, last_angles)
    gripper_diff = abs(new_gripper_angle - last_gripper_angle) if last_gripper_angle is not None else float('inf')
    
    if angle_diff < ANGLE_THRESHOLD and gripper_diff < GRIPPER_THRESHOLD:
        return False, f"角度变化太小 (臂:{angle_diff:.1f}°, 夹爪:{gripper_diff:.1f}°)"
    
    return True, "允许发送"

def add_command_to_queue(command):
    """添加命令到队列"""
    try:
        command_queue.put_nowait(command)
        return True
    except queue.Full:
        # 队列满时移除最旧命令
        try:
            old_command = command_queue.get_nowait()
            command_queue.put_nowait(command)
            return True
        except queue.Empty:
            return False

def initialize_gripper():
    """初始化Pro力控夹爪 - 简化版本"""
    global mc
    
    try:
        rospy.loginfo("[夹爪] 尝试初始化Pro力控夹爪...")
        
        # 检查夹爪是否存在
        version = mc.get_pro_gripper(1, GRIPPER_ID)
        if version == -1:
            rospy.logwarn("[夹爪] 未检测到Pro力控夹爪")
            return False
        
        rospy.loginfo(f"[夹爪] ✅ Pro力控夹爪检测成功，版本: {version}")
        
        # 尝试获取当前角度
        try:
            current_angle = mc.get_pro_gripper_angle(GRIPPER_ID)
            rospy.loginfo(f"[夹爪] 当前夹爪角度: {current_angle}°")
        except Exception as e:
            rospy.logwarn(f"[夹爪] 无法获取夹爪角度: {e}")
        
        return True
        
    except Exception as e:
        rospy.logwarn(f"[夹爪] 初始化失败: {e}")
        return False

def set_gripper_angle_320(angle):
    """设置MyCobot320夹爪角度 - 参考力控夹爪逻辑"""
    global mc
    
    # 将Gazebo角度(-60~60)映射到夹爪角度(0~100)
    mapped_angle = ((angle - GAZEBO_MIN_POSITION) / 
                   (GAZEBO_MAX_POSITION - GAZEBO_MIN_POSITION)) * \
                  (GRIPPER_MAX_ANGLE - GRIPPER_MIN_ANGLE) + GRIPPER_MIN_ANGLE
    
    # 限制到0-100范围
    mapped_angle = max(0, min(100, mapped_angle))
    mapped_angle = int(round(mapped_angle))
    
    try:
        # 尝试使用Pro力控夹爪角度设置 (只传递2个参数)
        try:
            mc.set_pro_gripper_angle(mapped_angle, GRIPPER_ID)
            rospy.logdebug(f"[夹爪] Pro力控夹爪设置成功: 真实夹爪={mapped_angle}°")
            return True
        except TypeError as te:
            rospy.logwarn(f"[夹爪] set_pro_gripper_angle参数错误: {te}")
            # 备选方案：使用基础夹爪状态控制
            # 根据映射角度判断开关状态
            if mapped_angle >= 90:
                mc.set_gripper_state(1, 80)  # 状态1 = 打开
                rospy.loginfo(f"[夹爪] 状态控制: 打开 (真实夹爪角度: {mapped_angle}°)")
            elif mapped_angle <= 10:
                mc.set_gripper_state(0, 80)  # 状态0 = 关闭
                rospy.loginfo(f"[夹爪] 状态控制: 关闭 (真实夹爪角度: {mapped_angle}°)")
            else:
                rospy.logwarn(f"[夹爪] 中间角度 {mapped_angle}° 无法用状态控制")
            return True
            
    except Exception as e:
        rospy.logwarn(f"[夹爪] 设置夹爪失败: {e}")
        return False

def command_executor():
    """异步命令执行线程"""
    global is_executing, last_angles, last_gripper_angle, last_command_time, stats
    
    while not rospy.is_shutdown():
        try:
            # 获取命令，超时1秒
            command = command_queue.get(timeout=1.0)
            
            if not is_mycobot_connected():
                rospy.logwarn("[slider_control] MyCobot320未连接，跳过命令")
                stats['errors'] += 1
                continue
            
            is_executing = True
            
            try:
                if command.type == 'angles':
                    # 发送角度命令
                    mc.send_angles(command.data, 50)  # MyCobot320推荐速度
                    last_angles = command.data.copy()
                    rospy.logdebug(f"[slider_control] 发送角度: {[round(a,1) for a in command.data]}")
                    
                elif command.type == 'gripper':
                    # 发送夹爪命令
                    gripper_angle = command.data  # 这是Gazebo角度
                    
                    # 计算映射后的夹爪角度用于显示
                    mapped_angle = ((gripper_angle - GAZEBO_MIN_POSITION) / 
                                   (GAZEBO_MAX_POSITION - GAZEBO_MIN_POSITION)) * \
                                  (GRIPPER_MAX_ANGLE - GRIPPER_MIN_ANGLE) + GRIPPER_MIN_ANGLE
                    mapped_angle = max(0, min(100, mapped_angle))
                    
                    rospy.loginfo(f"[slider_control] 🤏 执行夹爪命令: Gazebo={gripper_angle:.1f}° → 真实夹爪={mapped_angle:.1f}°")
                    
                    if set_gripper_angle_320(gripper_angle):
                        last_gripper_angle = gripper_angle
                        rospy.loginfo(f"[slider_control] ✅ 夹爪控制成功: 真实夹爪={mapped_angle:.1f}°")
                    else:
                        rospy.logwarn(f"[slider_control] ❌ 夹爪控制失败")
                        stats['errors'] += 1
                
                stats['commands_sent'] += 1
                last_command_time = time.time()
                
            except Exception as e:
                rospy.logerr(f"[slider_control] 命令执行失败: {e}")
                stats['errors'] += 1
                
            finally:
                is_executing = False
                command_queue.task_done()
                
        except queue.Empty:
            continue
        except Exception as e:
            rospy.logerr(f"[slider_control] 命令执行器错误: {e}")
            is_executing = False

def callback(msg: JointState):
    """优化的回调函数"""
    global stats
    
    stats['total_messages'] += 1
    
    # 快速解析关节数据
    arm_deg = [0.0] * len(ARM_JOINTS)
    grip_deg = 0.0
    
    name_to_deg = {name: math.degrees(pos) for name, pos in zip(msg.name, msg.position)}
    
    # 提取臂关节角度
    for i, joint_name in enumerate(ARM_JOINTS):
        if joint_name in name_to_deg:
            arm_deg[i] = round(name_to_deg[joint_name], 1)
    
    # 提取夹爪角度
    if GRIPPER_JOINT in name_to_deg:
        grip_deg = round(name_to_deg[GRIPPER_JOINT], 1)
    
    # 检查角度限制
    check_angle_limits(arm_deg, grip_deg)
    
    if mode == 1:
        # Gazebo模式
        clamped_arm, clamped_grip = clamp_angles(arm_deg, grip_deg)
        publish_to_gazebo(clamped_arm, clamped_grip)
        
    elif mode == 2:
        # 真实机械臂模式 - 异步处理
        should_send, reason = should_send_command(arm_deg, grip_deg)
        
        if should_send:
            # 对真实机械臂进行角度限制
            clamped_arm, clamped_grip = clamp_angles(arm_deg, grip_deg)
            
            # 添加臂关节命令
            arm_command = RobotCommand('angles', clamped_arm)
            if add_command_to_queue(arm_command):
                # 添加夹爪命令（如果角度变化足够大）
                gripper_diff = abs(clamped_grip - last_gripper_angle) if last_gripper_angle is not None else float('inf')
                if gripper_diff >= GRIPPER_THRESHOLD:
                    gripper_command = RobotCommand('gripper', clamped_grip)
                    add_command_to_queue(gripper_command)
            else:
                stats['commands_skipped'] += 1
        else:
            stats['commands_skipped'] += 1
            rospy.logdebug(f"[slider_control] 跳过命令: {reason}")

def publish_to_gazebo(arm_deg, grip_deg):
    """发布到Gazebo"""
    global pub_arm, pub_gripper
    
    try:
        # 臂关节轨迹
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(d) for d in arm_deg]
        pt.time_from_start = rospy.Duration(0.2)
        traj.points = [pt]
        pub_arm.publish(traj)
        
        # 夹爪轨迹
        traj_g = JointTrajectory()
        traj_g.header.stamp = rospy.Time.now()
        traj_g.joint_names = [GRIPPER_JOINT]
        ptg = JointTrajectoryPoint()
        ptg.positions = [math.radians(grip_deg)]
        ptg.time_from_start = rospy.Duration(0.2)
        traj_g.points = [ptg]
        pub_gripper.publish(traj_g)
        
        rospy.logdebug(f"[Gazebo] 发布: 臂{[round(a,1) for a in arm_deg]}, 夹爪{grip_deg:.1f}°")
        
    except Exception as e:
        rospy.logwarn(f"[Gazebo] 发布失败: {e}")

def initialize_mycobot():
    """初始化MyCobot320连接"""
    global mc
    
    # 显示所有可用串口
    list_available_ports()
    
    # 获取串口参数，优先使用ROS参数，其次自动检测
    port = rospy.get_param("~port", None)
    baud = rospy.get_param("~baud", 115200)
    
    if port is None:
        # 没有指定端口，使用智能选择
        rospy.loginfo("[slider_control] 未指定串口，启动智能串口选择...")
        port = smart_port_selection()
    else:
        # 指定了端口，但仍然测试连接性
        rospy.loginfo(f"[slider_control] 使用指定串口: {port}")
        if not test_port_connectivity(port, baud):
            rospy.logwarn(f"[slider_control] 指定串口 {port} 连接失败，尝试自动检测...")
            port = smart_port_selection()
    
    rospy.loginfo(f"[slider_control] 最终选择串口: {port} @ {baud}")
    
    try:
        mc = MyCobot320(port, baud)
        time.sleep(2.0)
        
        # 测试连接
        current_angles = mc.get_angles()
        rospy.loginfo(f"[slider_control] ✅ MyCobot320连接成功!")
        rospy.loginfo(f"[slider_control] 当前角度: {current_angles}")
        
        # 初始化夹爪
        gripper_ok = initialize_gripper()
        if gripper_ok:
            rospy.loginfo("[slider_control] ✅ 夹爪初始化成功")
            
            # 测试夹爪控制 - 改为调用主函数
            rospy.loginfo("[slider_control] 🧪 测试夹爪控制...")
            test_success = set_gripper_angle_320(0.0)  # 测试中间位置，调用主函数
            if test_success:
                rospy.loginfo("[slider_control] ✅ 夹爪控制测试成功")
            else:
                rospy.logwarn("[slider_control] ⚠️  夹爪控制测试失败")
        else:
            rospy.logwarn("[slider_control] ⚠️  夹爪未检测到或初始化失败，将跳过夹爪控制")
        
        mc.release_all_servos()
        time.sleep(0.5)
        
        return True
        
    except Exception as e:
        rospy.logerr(f"[slider_control] ❌ MyCobot320初始化失败: {e}")
        rospy.logerr("[slider_control] 请检查:")
        rospy.logerr("[slider_control] 1. 机械臂是否正确连接到电脑")
        rospy.logerr("[slider_control] 2. 串口权限是否正确 (sudo chmod 666 /dev/ttyACM* 或 /dev/ttyUSB*)")
        rospy.logerr("[slider_control] 3. 是否有其他程序占用串口")
        rospy.logerr("[slider_control] 4. 机械臂是否已开机并正常工作")
        return False

def print_stats():
    """打印统计信息"""
    if stats['total_messages'] > 0:
        efficiency = (stats['commands_sent'] / stats['total_messages']) * 100
        rospy.loginfo(f"[slider_control] 📊 统计: 消息:{stats['total_messages']}, "
                      f"发送:{stats['commands_sent']}, 跳过:{stats['commands_skipped']}, "
                      f"超限:{stats['limit_violations']}, 错误:{stats['errors']}, "
                      f"效率:{efficiency:.1f}%")

def main():
    global mc, mode, pub_arm, pub_gripper
    
    rospy.init_node("slider_control_320", anonymous=True)
    
    # 模式选择
    print("\n" + "="*60)
    print("🎮 MyCobot320 滑块控制器")
    print("="*60)
    print("选择控制模式:")
    print("  1: 滑块 → Gazebo仿真")
    print("  2: 滑块 → 真实 MyCobot320 (优化版+自动串口检测)")
    print("="*60)
    inp = input("请输入 1 或 2 (默认 2): ").strip()
    
    mode = 1 if inp == "1" else 2
    mode_name = "Gazebo仿真" if mode == 1 else "真实 MyCobot320"
    
    rospy.loginfo(f"[slider_control] 🎯 控制模式: {mode_name}")
    rospy.loginfo(f"[slider_control] ⚙️  配置: 角度阈值={ANGLE_THRESHOLD}°, "
                  f"最大频率={MAX_COMMAND_RATE}Hz, 队列大小={COMMAND_QUEUE_SIZE}")
    rospy.loginfo(f"[slider_control] 🛡️  安全限制: 关节±180°, 夹爪{GRIPPER_LIMITS[0]}°~{GRIPPER_LIMITS[1]}°")
    
    # 初始化发布器
    pub_arm = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
    pub_gripper = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=1)
    rospy.loginfo("[slider_control] 📡 ROS发布器初始化完成")
    
    if mode == 2:
        # 初始化真实机械臂
        if not initialize_mycobot():
            rospy.logerr("[slider_control] ❌ MyCobot320初始化失败，退出")
            return
        
        # 启动命令执行线程
        executor_thread = threading.Thread(target=command_executor, daemon=True)
        executor_thread.start()
        rospy.loginfo("[slider_control] 🔄 异步命令执行器已启动")
    
    # 订阅关节状态
    rospy.Subscriber("/joint_states", JointState, callback, queue_size=1)
    rospy.loginfo("[slider_control] 📥 已订阅 /joint_states 话题")
    
    rospy.loginfo(f"[slider_control] 🚀 {mode_name}控制器启动成功，等待滑块输入...")
    rospy.loginfo("[slider_control] 💡 Tips:")
    rospy.loginfo("[slider_control]    - 角度超限时会自动限制并显示警告")
    rospy.loginfo("[slider_control]    - 支持智能串口检测和端口选择")
    rospy.loginfo("[slider_control]    - 使用异步执行减少延迟和卡顿")
    if mode == 2:
        rospy.loginfo("[slider_control]    - 夹爪使用键盘控制的已验证逻辑")
        rospy.loginfo("[slider_control]    - 滑块角度直接映射到夹爪0-100范围")
        rospy.loginfo("[slider_control]    - 如果夹爪无反应，请检查控制台的夹爪日志")
    rospy.loginfo("[slider_control]    - 按 Ctrl+C 安全退出")
    
    # 定期打印统计信息
    def stats_timer():
        while not rospy.is_shutdown():
            time.sleep(15.0)  # 每15秒打印一次
            print_stats()
    
    stats_thread = threading.Thread(target=stats_timer, daemon=True)
    stats_thread.start()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[slider_control] 🛑 收到中断信号，正在关闭...")
    finally:
        print_stats()  # 最终统计
        if mc is not None:
            try:
                mc.release_all_servos()
                rospy.loginfo("[slider_control] ✅ 已释放所有舵机")
            except:
                pass
        rospy.loginfo("[slider_control] 👋 程序已安全退出")

if __name__ == "__main__":
    main()
