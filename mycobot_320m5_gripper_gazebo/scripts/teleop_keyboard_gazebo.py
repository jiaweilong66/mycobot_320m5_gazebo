#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import rospy
import time
import threading
import queue
import termios
import tty
import sys
import select
import serial.tools.list_ports
from pymycobot import MyCobot320
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray

# ========================= 全局变量 =========================
mc = None
pub_arm_command = None
pub_gripper_command = None
home_pose = [0, 0, 0, 0, 0, 0]

# 命令队列和控制标志
command_queue = queue.Queue(maxsize=10)
is_executing = False
last_command_time = 0
MIN_COMMAND_INTERVAL = 0.05  # 最小命令间隔(秒)

# 关节和夹爪配置
ARM_JOINTS = [
    "joint2_to_joint1",
    "joint3_to_joint2", 
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]
GRIPPER_JOINT = "gripper_controller"
GRIPPER_ID = 14

# 安全角度限制 (度)
JOINT_LIMITS = [
    (-180, 180),  # joint1
    (-180, 180),  # joint2  
    (-180, 180),  # joint3
    (-180, 180),  # joint4
    (-180, 180),  # joint5
    (-180, 180),  # joint6
]

# 夹爪映射常量
GRIPPER_MIN_ANGLE = 0      
GRIPPER_MAX_ANGLE = 100    
GAZEBO_MIN_POSITION = -60.0  
GAZEBO_MAX_POSITION = 60.0   

# 控制参数
ANGLE_STEP = 5.0            # 每次按键的角度步长
FAST_STEP = 15.0            # 快速移动步长
MOVEMENT_SPEED = 50         # 机械臂移动速度

# 配置选项
ENABLE_GAZEBO_SYNC = True   # 是否同步控制Gazebo
ENABLE_GRIPPER = True       # 是否启用夹爪控制

# 状态变量
last_valid_angles = None
last_gripper_angle = None

# ========================= 串口检测 =========================
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

# ========================= 安全检查 =========================
def check_angle_limits(angles):
    """检查角度是否在安全范围内"""
    violations = []
    
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
        if angle < min_limit or angle > max_limit:
            violations.append(f"关节{i+1}: {angle:.1f}° (限制: {min_limit}°~{max_limit}°)")
    
    if violations:
        rospy.logwarn("⚠️  角度超限:")
        for violation in violations:
            rospy.logwarn(f"    {violation}")
        return False
    
    return True

def clamp_angles(angles):
    """将角度限制在安全范围内"""
    clamped_angles = []
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JOINT_LIMITS)):
        clamped_angle = max(min_limit, min(max_limit, angle))
        clamped_angles.append(clamped_angle)
    return clamped_angles

# ========================= Gazebo发布 =========================
def publish_to_gazebo(angles, gripper_angle=None):
    """发布到Gazebo仿真环境"""
    global pub_arm_command, pub_gripper_command
    
    if not ENABLE_GAZEBO_SYNC:
        return
    
    try:
        # 发布机械臂轨迹
        if pub_arm_command is None:
            pub_arm_command = rospy.Publisher("/arm_controller/command", 
                                              JointTrajectory, queue_size=1)
        
        arm_traj = JointTrajectory()
        arm_traj.header.stamp = rospy.Time.now()
        arm_traj.joint_names = ARM_JOINTS
        
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(a) for a in angles]
        pt.velocities = [0.0] * len(ARM_JOINTS)
        pt.accelerations = [0.0] * len(ARM_JOINTS)
        pt.time_from_start = rospy.Duration(0.5)
        arm_traj.points.append(pt)
        
        pub_arm_command.publish(arm_traj)
        rospy.logdebug(f"📤 Gazebo机械臂: {[round(a, 1) for a in angles]}")
        
        # 发布夹爪轨迹
        if gripper_angle is not None and ENABLE_GRIPPER:
            if pub_gripper_command is None:
                pub_gripper_command = rospy.Publisher("/gripper_controller/command",
                                                      JointTrajectory, queue_size=1)
            
            # 夹爪角度映射：0-100° -> -60°到60°
            mapped_gripper = ((gripper_angle - GRIPPER_MIN_ANGLE) /
                             (GRIPPER_MAX_ANGLE - GRIPPER_MIN_ANGLE)) * \
                            (GAZEBO_MAX_POSITION - GAZEBO_MIN_POSITION) + GAZEBO_MIN_POSITION
            
            # 限制范围
            mapped_gripper = max(GAZEBO_MIN_POSITION, min(GAZEBO_MAX_POSITION, mapped_gripper))
            
            gripper_traj = JointTrajectory()
            gripper_traj.header.stamp = rospy.Time.now()
            gripper_traj.joint_names = [GRIPPER_JOINT]
            
            gp = JointTrajectoryPoint()
            gp.positions = [math.radians(mapped_gripper)]
            gp.velocities = [0.0]
            gp.accelerations = [0.0]
            gp.time_from_start = rospy.Duration(0.5)
            gripper_traj.points.append(gp)
            
            pub_gripper_command.publish(gripper_traj)
            rospy.loginfo(f"📤 Gazebo夹爪: {gripper_angle}° -> {mapped_gripper:.1f}° (弧度: {math.radians(mapped_gripper):.3f})")
            
    except Exception as e:
        rospy.logwarn(f"Gazebo发布失败: {e}")

# ========================= 夹爪控制 =========================
def initialize_gripper():
    """初始化夹爪"""
    global mc
    
    if not ENABLE_GRIPPER:
        return False
        
    try:
        rospy.loginfo("尝试初始化Pro力控夹爪...")
        
        # 检查夹爪是否存在
        version = mc.get_pro_gripper(GRIPPER_ID, 1)
        if version == -1:
            rospy.logwarn("未检测到Pro力控夹爪")
            return False
        
        rospy.loginfo("Pro力控夹爪检测成功")
        return True
        
    except Exception as e:
        rospy.logwarn(f"夹爪初始化失败: {e}")
        return False

def get_current_gripper_angle():
    """获取当前夹爪角度"""
    global mc, last_gripper_angle
    
    if not ENABLE_GRIPPER:
        return 50  # 默认值
        
    try:
        angle = mc.get_pro_gripper_angle(GRIPPER_ID)
        if angle is not None and 0 <= angle <= 100:
            last_gripper_angle = angle
            return angle
        elif last_gripper_angle is not None:
            return last_gripper_angle
        else:
            return 50  # 默认中间值
    except:
        return last_gripper_angle if last_gripper_angle is not None else 50

def set_gripper_angle(angle):
    """设置夹爪角度"""
    global mc
    
    if not ENABLE_GRIPPER:
        return False
        
    try:
        # 限制角度范围
        angle = max(0, min(100, angle))
        
        # 尝试使用Pro夹爪角度设置 (修正参数个数)
        try:
            mc.set_pro_gripper_angle(GRIPPER_ID, angle)
            rospy.logdebug(f"Pro夹爪角度设置成功: {angle}°")
            return True
        except TypeError as te:
            rospy.logwarn(f"set_pro_gripper_angle参数错误: {te}")
            # 备选方案：使用基础夹爪状态控制
            # 根据用户反馈：角度0=关闭，角度100=打开
            if angle >= 90:
                mc.set_gripper_state(1, 80)  # 状态1 = 打开
                rospy.loginfo(f"夹爪状态控制: 打开 (角度: {angle})")
            elif angle <= 10:
                mc.set_gripper_state(0, 80)  # 状态0 = 关闭
                rospy.loginfo(f"夹爪状态控制: 关闭 (角度: {angle})")
            else:
                rospy.logwarn(f"中间角度 {angle}° 无法用状态控制，请使用0-10°(关)或90-100°(开)")
            return True
            
    except Exception as e:
        rospy.logwarn(f"设置夹爪失败: {e}")
        return False

def set_gripper_state_simple(state):
    """简化的夹爪状态控制 (开/关)"""
    global mc
    
    if not ENABLE_GRIPPER:
        return False
        
    try:
        # 根据用户反馈修正状态对应关系：
        # state: 0=关闭, 1=打开
        mc.set_gripper_state(state, 80)
        rospy.loginfo(f"🤏 夹爪{'打开' if state else '关闭'}")
        return True
    except Exception as e:
        rospy.logwarn(f"设置夹爪状态失败: {e}")
        return False

# ========================= 命令执行线程 =========================
def command_executor():
    """异步执行命令的线程"""
    global is_executing, last_command_time, last_valid_angles
    
    while not rospy.is_shutdown():
        try:
            # 从队列获取命令，超时1秒
            command = command_queue.get(timeout=1.0)
            
            current_time = time.time()
            # 检查命令间隔，避免发送过于频繁
            if current_time - last_command_time < MIN_COMMAND_INTERVAL:
                time.sleep(MIN_COMMAND_INTERVAL - (current_time - last_command_time))
            
            is_executing = True
            
            if command['type'] == 'angles':
                angles = command['data']
                
                # 安全检查和角度限制
                clamped_angles = clamp_angles(angles)
                
                try:
                    # 发送角度到真实机械臂
                    mc.send_angles(clamped_angles, MOVEMENT_SPEED)
                    last_valid_angles = clamped_angles.copy()
                    
                    # 同步到Gazebo
                    current_gripper = get_current_gripper_angle()
                    publish_to_gazebo(clamped_angles, current_gripper)
                    
                    rospy.loginfo(f"📤 机械臂移动: {[round(a, 1) for a in clamped_angles]}")
                    
                except Exception as e:
                    rospy.logwarn(f"执行角度命令失败: {e}")
                    
            elif command['type'] == 'gripper':
                angle = command['data']
                
                try:
                    rospy.loginfo(f"🔧 执行夹爪命令: {angle}°")
                    
                    if set_gripper_angle(angle):
                        rospy.loginfo(f"✅ 真实夹爪控制成功: {angle}°")
                        
                        # 同步到Gazebo
                        if last_valid_angles is not None:
                            publish_to_gazebo(last_valid_angles, angle)
                            rospy.loginfo(f"📤 已同步夹爪到Gazebo: {angle}°")
                        else:
                            rospy.logwarn("⚠️  无有效机械臂角度，跳过Gazebo夹爪同步")
                        
                        rospy.loginfo(f"🤏 夹爪移动完成: {angle}°")
                    else:
                        rospy.logwarn("❌ 夹爪控制失败，请检查夹爪连接")
                    
                except Exception as e:
                    rospy.logwarn(f"执行夹爪命令失败: {e}")
            
            last_command_time = time.time()
            is_executing = False
            command_queue.task_done()
            
        except queue.Empty:
            # 队列为空，继续等待
            continue
        except Exception as e:
            rospy.logerr(f"命令执行器错误: {e}")
            is_executing = False

def add_command_to_queue(command_type, data):
    """添加命令到队列"""
    command = {'type': command_type, 'data': data}
    
    try:
        # 非阻塞添加，如果队列满则丢弃最旧命令
        command_queue.put_nowait(command)
    except queue.Full:
        try:
            command_queue.get_nowait()  # 移除最旧命令
            command_queue.put_nowait(command)  # 添加新命令
        except queue.Empty:
            pass

# ========================= 键盘输入 =========================
class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.prev = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.prev)

def get_key_non_blocking():
    """非阻塞获取按键"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

# ========================= 键盘控制逻辑 =========================
def print_help():
    """打印帮助信息"""
    help_text = """
MyCobot320 键盘控制器
=====================

🔧 关节控制 (普通步长: {}°, 快速步长: {}°):
  w/s: joint1 +/-     W/S: joint1 +/-  (快速)
  e/d: joint2 +/-     E/D: joint2 +/-  (快速)
  r/f: joint3 +/-     R/F: joint3 +/-  (快速)  
  t/g: joint4 +/-     T/G: joint4 +/-  (快速)
  y/h: joint5 +/-     Y/H: joint5 +/-  (快速)
  u/j: joint6 +/-     U/J: joint6 +/-  (快速)

🤏 夹爪控制{}:
  o: 夹爪完全打开 (角度模式: 100° / 状态模式: 打开)
  p: 夹爪完全关闭 (角度模式: 0° / 状态模式: 关闭)
  [/]: 夹爪开启/关闭 10° (仅角度模式支持) 
  
🏠 特殊命令:
  1: 回到初始位置 (所有关节0°)
  2: 获取当前角度
  3: 切换Gazebo同步 (当前: {})
  4: 诊断Gazebo连接状态
  
❌ 退出:
  q: 退出程序

⚠️  安全提示:
- 角度限制: ±180°
- 程序会自动限制超出范围的角度
- 按键命令会异步执行，减少卡顿
""".format(
        ANGLE_STEP, 
        FAST_STEP,
        " (已启用)" if ENABLE_GRIPPER else " (未启用)",
        "开启" if ENABLE_GAZEBO_SYNC else "关闭"
    )
    print(help_text)

def teleop_keyboard():
    """键盘控制主循环"""
    global home_pose, mc, ENABLE_GAZEBO_SYNC, last_valid_angles
    
    # 获取当前角度作为起始点
    try:
        current_angles = mc.get_angles()
        if current_angles and len(current_angles) == 6:
            angle_list = current_angles.copy()
            last_valid_angles = angle_list.copy()  # 确保全局变量被设置
            rospy.loginfo(f"当前角度: {[round(a, 1) for a in angle_list]}")
        else:
            angle_list = home_pose.copy()
            last_valid_angles = home_pose.copy()  # 使用初始位置
            rospy.logwarn("无法获取当前角度，使用初始位置")
    except:
        angle_list = home_pose.copy()
        last_valid_angles = home_pose.copy()  # 使用初始位置
        rospy.logwarn("获取角度失败，使用初始位置")
    
    current_gripper = get_current_gripper_angle()
    rospy.loginfo(f"当前夹爪角度: {current_gripper}°")
    
    print_help()
    
    # 启动命令执行线程
    executor_thread = threading.Thread(target=command_executor, daemon=True)
    executor_thread.start()
    
    with RawTerminal():
        while not rospy.is_shutdown():
            key = get_key_non_blocking()
            
            if key is None:
                time.sleep(0.01)  # 短暂休眠减少CPU占用
                continue
                
            if key == 'q':
                print("\n退出程序...")
                break
            
            # 初始化回家
            if key == '1':
                angle_list = home_pose.copy()
                add_command_to_queue('angles', angle_list)
                print("🏠 回到初始位置")
                continue
            
            # 获取当前角度
            if key == '2':
                try:
                    current = mc.get_angles()
                    if current and len(current) == 6:
                        angle_list = current.copy()
                        print(f"📍 当前角度: {[round(a, 1) for a in angle_list]}")
                    else:
                        print("❌ 无法获取当前角度")
                except Exception as e:
                    print(f"❌ 获取角度失败: {e}")
                continue
            
            # 切换Gazebo同步
            if key == '3':
                ENABLE_GAZEBO_SYNC = not ENABLE_GAZEBO_SYNC
                print(f"🔄 Gazebo同步: {'开启' if ENABLE_GAZEBO_SYNC else '关闭'}")
                continue
            
            # 诊断Gazebo连接状态
            if key == '4':
                print("\n🔍 Gazebo连接诊断:")
                try:
                    if pub_arm_command:
                        arm_subs = pub_arm_command.get_num_connections()
                        print(f"  机械臂控制器订阅者: {arm_subs}")
                    else:
                        print(f"  机械臂控制器: 未初始化")
                    
                    if pub_gripper_command:
                        gripper_subs = pub_gripper_command.get_num_connections()
                        print(f"  夹爪控制器订阅者: {gripper_subs}")
                    else:
                        print(f"  夹爪控制器: 未初始化")
                    
                    print(f"  Gazebo同步状态: {'开启' if ENABLE_GAZEBO_SYNC else '关闭'}")
                    print(f"  夹爪支持: {'是' if ENABLE_GRIPPER else '否'}")
                    
                    # 测试发布一个示例夹爪命令
                    if ENABLE_GAZEBO_SYNC and ENABLE_GRIPPER:
                        current_gripper = get_current_gripper_angle()
                        print(f"  当前夹爪角度: {current_gripper}°")
                        if last_valid_angles:
                            publish_to_gazebo(last_valid_angles, current_gripper)
                            print(f"  ✅ 发送测试夹爪命令到Gazebo")
                        else:
                            print(f"  ⚠️  没有有效的机械臂角度用于测试")
                            
                except Exception as e:
                    print(f"  ❌ 诊断失败: {e}")
                print("")
                continue
            
            # 夹爪控制
            if ENABLE_GRIPPER:
                if key == 'o':
                    # o = 打开夹爪 (根据用户反馈，应该是角度100/状态1)
                    current_gripper = 100
                    add_command_to_queue('gripper', current_gripper)
                    print(f"🤏 夹爪打开: {current_gripper}°")
                    continue
                elif key == 'p':
                    # p = 关闭夹爪 (根据用户反馈，应该是角度0/状态0)
                    current_gripper = 0
                    add_command_to_queue('gripper', current_gripper)
                    print(f"🤏 夹爪关闭: {current_gripper}°")
                    continue
                elif key == '[':
                    # 渐进控制 - 夹爪更加打开
                    current_gripper = min(100, current_gripper + 10)
                    add_command_to_queue('gripper', current_gripper)
                    print(f"🤏 夹爪开启: {current_gripper}°")
                    continue
                elif key == ']':
                    # 渐进控制 - 夹爪更加关闭
                    current_gripper = max(0, current_gripper - 10)
                    add_command_to_queue('gripper', current_gripper)
                    print(f"🤏 夹爪关闭: {current_gripper}°")
                    continue
            
            # 关节运动映射 (普通步长)
            normal_mapping = {
                'w': (0, +ANGLE_STEP), 's': (0, -ANGLE_STEP),
                'e': (1, +ANGLE_STEP), 'd': (1, -ANGLE_STEP),
                'r': (2, +ANGLE_STEP), 'f': (2, -ANGLE_STEP),
                't': (3, +ANGLE_STEP), 'g': (3, -ANGLE_STEP),
                'y': (4, +ANGLE_STEP), 'h': (4, -ANGLE_STEP),
                'u': (5, +ANGLE_STEP), 'j': (5, -ANGLE_STEP),
            }
            
            # 关节运动映射 (快速步长)
            fast_mapping = {
                'W': (0, +FAST_STEP), 'S': (0, -FAST_STEP),
                'E': (1, +FAST_STEP), 'D': (1, -FAST_STEP),
                'R': (2, +FAST_STEP), 'F': (2, -FAST_STEP),
                'T': (3, +FAST_STEP), 'G': (3, -FAST_STEP),
                'Y': (4, +FAST_STEP), 'H': (4, -FAST_STEP),
                'U': (5, +FAST_STEP), 'J': (5, -FAST_STEP),
            }
            
            # 检查普通步长映射
            if key in normal_mapping:
                idx, step = normal_mapping[key]
                angle_list[idx] += step
                add_command_to_queue('angles', angle_list.copy())
                print(f"🔧 关节{idx+1}: {angle_list[idx]:.1f}° (步长: {step:.1f}°)")
                continue
            
            # 检查快速步长映射
            if key in fast_mapping:
                idx, step = fast_mapping[key]
                angle_list[idx] += step
                add_command_to_queue('angles', angle_list.copy())
                print(f"🚀 关节{idx+1}: {angle_list[idx]:.1f}° (快速步长: {step:.1f}°)")
                continue

# ========================= 主函数 =========================
def main():
    global mc
    
    rospy.init_node("mycobot320_keyboard_controller", anonymous=True)
    
    print("MyCobot320 键盘控制器启动中...")
    
    # 显示所有可用串口
    list_available_ports()
    
    # 获取串口参数
    port = rospy.get_param("~port", find_available_port())
    baud = rospy.get_param("~baud", 115200)
    
    rospy.loginfo(f"使用串口: {port}, 波特率: {baud}")
    
    try:
        # 初始化MyCobot320连接
        mc = MyCobot320(port, baud)
        rospy.loginfo("✅ MyCobot320连接成功")
        
        # 释放舵机，允许手动操作
        mc.release_all_servos()
        time.sleep(0.5)
        rospy.loginfo("🔓 舵机已释放")
        
        # 初始化夹爪
        gripper_ok = initialize_gripper()
        if gripper_ok:
            rospy.loginfo("✅ 夹爪初始化成功")
        else:
            rospy.logwarn("⚠️  夹爪未连接或初始化失败")
        
    except Exception as e:
        rospy.logerr(f"❌ MyCobot320连接失败: {e}")
        rospy.logerr("请检查:")
        rospy.logerr("1. 机械臂是否正确连接到电脑")
        rospy.logerr("2. 串口权限是否正确 (sudo chmod 666 /dev/ttyACM0)")
        rospy.logerr("3. 是否有其他程序占用串口")
        rospy.logerr("4. 机械臂是否上电")
        return
    
    print(f"""
🎮 MyCobot320 键盘控制器已就绪!
✅ 机械臂连接: 成功
🤏 夹爪支持: {'是' if ENABLE_GRIPPER else '否'}
🎮 Gazebo同步: {'开启' if ENABLE_GAZEBO_SYNC else '关闭'}
⚡ 异步执行: 已启用
🛡️ 安全限制: 已启用

按任意键开始控制 (按 'q' 退出)...
""")
    
    # 注册退出处理函数
    def cleanup():
        if mc is not None:
            try:
                rospy.loginfo("🔄 程序退出，释放舵机...")
                mc.release_all_servos()
                rospy.loginfo("✅ 舵机已释放")
            except:
                pass
    
    rospy.on_shutdown(cleanup)
    
    try:
        teleop_keyboard()
    except KeyboardInterrupt:
        rospy.loginfo("🛑 接收到中断信号，正在关闭...")
    finally:
        cleanup()

if __name__ == "__main__":
    main()
