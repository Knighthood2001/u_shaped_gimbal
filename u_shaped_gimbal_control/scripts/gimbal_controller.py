#!/usr/bin/env python

import rospy
import math
import time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class UGimbalController:
    def __init__(self):
        rospy.init_node('u_gimbal_controller', anonymous=True)
        
        # 创建发布器来控制关节位置
        self.joint_pub = rospy.Publisher('/gimbal_controller/command', Float64MultiArray, queue_size=10)
        
        # 订阅关节状态
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        self.current_positions = [0.0, 0.0]  # [joint_low, joint_up]
        self.joint_names = ['joint_low', 'joint_up']
        
        rospy.sleep(2)  # 等待系统初始化
        print("U型云台控制器已启动")
        
    def joint_state_callback(self, msg):
        """更新当前关节位置"""
        try:
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.current_positions[i] = msg.position[idx]
        except Exception as e:
            rospy.logwarn("获取关节状态失败: %s", str(e))
    
    def set_joint_positions(self, low_angle, up_angle):
        """设置两个关节的角度（弧度）"""
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [low_angle, up_angle]
        self.joint_pub.publish(cmd_msg)
        print(f"设置位置: 下关节={math.degrees(low_angle):.1f}°, 上关节={math.degrees(up_angle):.1f}°")
    
    def sinusoidal_motion(self, amplitude=1.0, frequency=0.5, duration=10.0):
        """正弦波运动演示"""
        start_time = time.time()
        rate = rospy.Rate(50)  # 50Hz
        
        print(f"开始正弦波运动: 幅度={math.degrees(amplitude):.1f}°, 频率={frequency}Hz, 时长={duration}秒")
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration:
            elapsed = time.time() - start_time
            # 两个关节做不同相位和频率的正弦运动
            low_angle = amplitude * math.sin(2 * math.pi * frequency * elapsed)
            up_angle = amplitude * math.cos(2 * math.pi * frequency * elapsed)
            
            self.set_joint_positions(low_angle, up_angle)
            rate.sleep()
    
    def step_motion(self):
        """步进运动演示"""
        steps = [
            (0.0, 0.0),           # 初始位置
            (math.pi/4, 0.0),     # 下关节旋转45度
            (math.pi/4, math.pi/4), # 上关节也旋转45度
            (0.0, math.pi/4),     # 下关节回到0度
            (0.0, 0.0)           # 回到初始位置
        ]
        
        print("开始步进运动演示...")
        for i, (low_angle, up_angle) in enumerate(steps):
            print(f"步骤 {i+1}: 下关节={math.degrees(low_angle):.1f}°, 上关节={math.degrees(up_angle):.1f}°")
            self.set_joint_positions(low_angle, up_angle)
            rospy.sleep(2)  # 每个位置停留2秒
    
    def interactive_control(self):
        """交互式控制"""
        print("\n=== U型云台交互控制 ===")
        print("命令:")
        print("  s - 步进运动演示")
        print("  w - 正弦波运动演示") 
        print("  l [角度] - 设置下关节角度(度)")
        print("  u [角度] - 设置上关节角度(度)")
        print("  r - 重置到零位")
        print("  q - 退出")
        
        while not rospy.is_shutdown():
            try:
                user_input = input("> ").strip().split()
                if not user_input:
                    continue
                    
                cmd = user_input[0].lower()
                
                if cmd == 'q':
                    break
                elif cmd == 'r':
                    self.set_joint_positions(0.0, 0.0)
                elif cmd == 's':
                    self.step_motion()
                elif cmd == 'w':
                    self.sinusoidal_motion(amplitude=math.pi/3, frequency=0.3, duration=15.0)
                elif cmd == 'l' and len(user_input) > 1:
                    angle_deg = float(user_input[1])
                    self.set_joint_positions(math.radians(angle_deg), self.current_positions[1])
                elif cmd == 'u' and len(user_input) > 1:
                    angle_deg = float(user_input[1])
                    self.set_joint_positions(self.current_positions[0], math.radians(angle_deg))
                else:
                    print("未知命令")
                    
            except ValueError:
                print("请输入有效的数字")
            except KeyboardInterrupt:
                break

def main():
    try:
        controller = UGimbalController()
        
        # 初始归零
        controller.set_joint_positions(0.0, 0.0)
        rospy.sleep(1)
        
        # 启动交互式控制
        controller.interactive_control()
        
        # 程序结束前回到零位
        controller.set_joint_positions(0.0, 0.0)
        print("云台控制程序结束")
        
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()