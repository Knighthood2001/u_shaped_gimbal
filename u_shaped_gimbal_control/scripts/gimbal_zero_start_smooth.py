import rospy
import math
import argparse
from std_msgs.msg import Float64

"""
这个代码是控制云台从起始角度逐渐过渡到目标角度，并控制云台结构，绕Z轴（水平）和Y轴（垂直）旋转，支持命令行输入目标角度。
注意起始角度是0，因此每次运行，他最开始都会先闪现到0度，然后再慢慢转动到目标角度。
"""

def angle_to_radian(angle_deg):
    """将角度（度）转换为弧度"""
    return math.radians(angle_deg)

def publish_smooth_trajectory(controller_topic, start_rad, target_rad, total_time=5.0, step_interval=0.1):
    """
    平滑发布轨迹：从起始角度逐步过渡到目标角度
    controller_topic: 控制器话题
    start_rad: 起始角度（弧度）
    target_rad: 目标角度（弧度）
    total_time: 总转动时间（秒），时间越长转动越慢
    step_interval: 每步间隔时间（秒），控制发布频率
    """
    pub = rospy.Publisher(controller_topic, Float64, queue_size=10)
    # 计算总步数和每步角度增量
    total_steps = int(total_time / step_interval)
    if total_steps == 0:
        total_steps = 1  # 避免除以0
    angle_increment = (target_rad - start_rad) / total_steps  # 每步的角度增量（弧度）
    
    rospy.loginfo(
        f"开始平滑转动（总耗时{total_time}秒），从{start_rad:.2f}弧度到{target_rad:.2f}弧度"
    )
    
    # 分步发布中间角度
    current_rad = start_rad
    for _ in range(total_steps):
        if rospy.is_shutdown():
            break
        pub.publish(Float64(data=current_rad))
        current_rad += angle_increment  # 更新当前角度
        rospy.sleep(step_interval)  # 等待下一步
    
    # 最后一步确保到达目标角度（避免累积误差）
    pub.publish(Float64(data=target_rad))
    rospy.loginfo(f"{controller_topic} 已到达目标角度")

def main():
    parser = argparse.ArgumentParser(description="云台关节平滑控制脚本（支持命令行指定角度和转动时间）")
    # 位置参数：Z轴和Y轴目标角度（度），可选，默认20度和30度
    parser.add_argument(
        "z_angle", 
        type=float, 
        nargs='?', 
        default=20.0, 
        help="Z轴（水平）目标角度（度），默认20.0度"
    )
    parser.add_argument(
        "y_angle", 
        type=float, 
        nargs='?', 
        default=30.0, 
        help="Y轴（垂直）目标角度（度），默认30.0度"
    )
    # 可选参数：Z轴和Y轴转动总时间（秒），通过--前缀指定，默认5秒
    parser.add_argument(
        "z_time", 
        type=float, 
        nargs='?', 
        default=2.0, 
        help="Z轴转动总时间（秒），默认2.0秒"
    )
    parser.add_argument(
        "y_time", 
        type=float, 
        nargs='?', 
        default=2.0, 
        help="Y轴转动总时间（秒），默认2.0秒"
    )
    
    args = parser.parse_args()

    rospy.init_node('joint_smooth_controller')
    
    # 假设起始角度为0弧度（可根据实际初始位置调整，例如通过订阅/joint_states获取）
    start_rad = 0.0
    
    # 1. 平滑控制Z轴旋转（使用命令行输入的角度和时间）
    z_target_deg = args.z_angle
    z_target_rad = angle_to_radian(z_target_deg)
    rospy.loginfo(
        f"Z轴目标：{z_target_deg}度（{z_target_rad:.2f}弧度），转动时间：{args.z_time}秒"
    )
    publish_smooth_trajectory(
        controller_topic="/root_to_low_position_controller/command",
        start_rad=start_rad,
        target_rad=z_target_rad,
        total_time=args.z_time  # 传入Z轴转动时间
    )
    
    # 等待Z轴完成后再控制Y轴
    rospy.sleep(0.5)
    
    # 2. 平滑控制Y轴旋转（使用命令行输入的角度和时间）
    y_target_deg = args.y_angle
    y_target_rad = angle_to_radian(y_target_deg)
    rospy.loginfo(
        f"Y轴目标：{y_target_deg}度（{y_target_rad:.2f}弧度），转动时间：{args.y_time}秒"
    )
    publish_smooth_trajectory(
        controller_topic="/low_to_high_position_controller/command",
        start_rad=start_rad,
        target_rad=y_target_rad,
        total_time=args.y_time  # 传入Y轴转动时间
    )
    
    rospy.loginfo("所有转动完成")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"错误：{str(e)}")