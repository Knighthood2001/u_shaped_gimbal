import rospy
import math
import argparse
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState  # 用于获取关节当前状态


def angle_to_radian(angle_deg):
    """将角度（度）转换为弧度"""
    return math.radians(angle_deg)


def radian_to_angle(radian):
    """将弧度转换为角度（度），用于计算角度差"""
    return math.degrees(radian)


def get_current_joint_radian(joint_name, timeout=5.0):
    """
    获取指定关节的当前角度（弧度）
    joint_name: 关节名称（需与URDF中定义一致）
    timeout: 等待关节状态消息的超时时间（秒）
    返回：关节当前角度（弧度），失败则返回None
    """
    try:
        joint_state = rospy.wait_for_message('/joint_states', JointState, timeout=timeout)
        if joint_name in joint_state.name:
            idx = joint_state.name.index(joint_name)
            return joint_state.position[idx]  # 关节状态消息存储的是弧度
        else:
            rospy.logerr(f"未找到关节名称: {joint_name}，请通过 'rostopic echo /joint_states' 确认URDF关节名")
            return None
    except rospy.ROSException:
        rospy.logerr(f"获取关节状态超时（{timeout}秒），请确保机器人节点已启动并发布 /joint_states 话题")
        return None


def publish_smooth_trajectory(controller_topic, start_rad, target_rad, total_time=5.0, step_interval=0.1):
    """
    平滑发布轨迹：从起始角度逐步过渡到目标角度（核心逻辑不变，接收总时间参数）
    controller_topic: 控制器话题
    start_rad: 起始角度（弧度）
    target_rad: 目标角度（弧度）
    total_time: 总转动时间（秒，由“角度差÷速度”计算得出）
    step_interval: 每步间隔时间（秒），控制发布频率（默认0.1秒，即10Hz，兼顾平滑与性能）
    """
    pub = rospy.Publisher(controller_topic, Float64, queue_size=10)
    # 计算总步数（避免除以0，最小1步）
    total_steps = int(total_time / step_interval) if total_time > 0 else 1
    # 每步角度增量（弧度）：匀速过渡的核心
    angle_increment = (target_rad - start_rad) / total_steps
    
    rospy.loginfo(
        f"开始匀速转动：从{radian_to_angle(start_rad):.2f}度到{radian_to_angle(target_rad):.2f}度，"
        f"总耗时{total_time:.2f}秒，速率{abs(radian_to_angle(angle_increment)/step_interval):.2f}度/秒"
    )
    
    current_rad = start_rad
    for _ in range(total_steps):
        if rospy.is_shutdown():
            break
        pub.publish(Float64(data=current_rad))
        current_rad += angle_increment
        rospy.sleep(step_interval)
    
    # 最后一步确保精准到达目标角度（消除累积误差）
    pub.publish(Float64(data=target_rad))
    rospy.loginfo(f"[{controller_topic}] 已到达目标角度")


def main():
    # 1. 解析命令行参数：将“转动时间”改为“转动速度”（度/秒）
    parser = argparse.ArgumentParser(description="云台关节匀速控制脚本（从当前位置+指定速度转动）")
    # 位置参数1：Z轴目标角度（度），默认20度
    parser.add_argument(
        "z_angle", 
        type=float, 
        nargs='?', 
        default=20.0, 
        help="Z轴（水平旋转）目标角度（度），默认值：20.0度"
    )
    # 位置参数2：Y轴目标角度（度），默认30度
    parser.add_argument(
        "y_angle", 
        type=float, 
        nargs='?', 
        default=30.0, 
        help="Y轴（垂直旋转）目标角度（度），默认值：30.0度"
    )
    # 位置参数3：Z轴转动速度（度/秒），默认10度/秒（安全速率，可根据机械性能调整）
    parser.add_argument(
        "z_speed", 
        type=float, 
        nargs='?', 
        default=10.0, 
        help="Z轴转动速度（度/秒），默认值：10.0度/秒（建议不超过20度/秒，避免抖动）"
    )
    # 位置参数4：Y轴转动速度（度/秒），默认10度/秒
    parser.add_argument(
        "y_speed", 
        type=float, 
        nargs='?', 
        default=10.0, 
        help="Y轴转动速度（度/秒），默认值：10.0度/秒"
    )
    
    args = parser.parse_args()

    # 2. 初始化ROS节点（节点名含“speed_control”，明确功能）
    rospy.init_node('joint_speed_control_from_current')
    
    # 3. 获取关节当前角度（核心：从当前位置开始转动）
    # ！！！必须替换为你的URDF实际关节名！！！通过 "rostopic echo /joint_states" 查看name字段
    z_joint_name = "joint_low"   # Z轴（水平）关节名
    y_joint_name = "joint_up"    # Y轴（垂直）关节名
    
    # 获取Z轴当前角度（弧度）
    z_start_rad = get_current_joint_radian(z_joint_name)
    if z_start_rad is None:
        rospy.logerr("Z轴当前角度获取失败，退出控制流程")
        return
    
    # 获取Y轴当前角度（弧度）
    y_start_rad = get_current_joint_radian(y_joint_name)
    if y_start_rad is None:
        rospy.logerr("Y轴当前角度获取失败，退出控制流程")
        return
    
    # 打印当前状态，方便调试
    rospy.loginfo(f"\n当前角度：Z轴 {radian_to_angle(z_start_rad):.2f}度，Y轴 {radian_to_angle(y_start_rad):.2f}度")
    rospy.loginfo(f"目标角度：Z轴 {args.z_angle:.2f}度，Y轴 {args.y_angle:.2f}度")
    rospy.loginfo(f"转动速度：Z轴 {args.z_speed:.2f}度/秒，Y轴 {args.y_speed:.2f}度/秒\n")

    # 4. 控制Z轴匀速转动（核心：速度→总时间的计算）
    # 计算Z轴角度差（度）：取绝对值，避免负速度
    z_angle_diff = abs(args.z_angle - radian_to_angle(z_start_rad))
    # 角度差过小（<0.1度）：无需转动，避免机械微动
    if z_angle_diff < 0.1:
        rospy.loginfo(f"Z轴角度差（{z_angle_diff:.2f}度）小于阈值，无需转动")
    else:
        # 总转动时间 = 角度差 ÷ 速度（秒）：匀速的核心逻辑
        z_total_time = z_angle_diff / args.z_speed
        # 转换目标角度为弧度
        z_target_rad = angle_to_radian(args.z_angle)
        # 发布平滑轨迹
        publish_smooth_trajectory(
            controller_topic="/root_to_low_position_controller/command",
            start_rad=z_start_rad,
            target_rad=z_target_rad,
            total_time=z_total_time  # 传入计算出的总时间
        )
    
    # 等待Z轴转动完成（可选：根据机械结构是否需要先后顺序，可调整为0.3-1秒）
    rospy.sleep(0.5)

    # 5. 控制Y轴匀速转动（逻辑与Z轴一致）
    y_angle_diff = abs(args.y_angle - radian_to_angle(y_start_rad))
    if y_angle_diff < 0.1:
        rospy.loginfo(f"Y轴角度差（{y_angle_diff:.2f}度）小于阈值，无需转动")
    else:
        y_total_time = y_angle_diff / args.y_speed
        y_target_rad = angle_to_radian(args.y_angle)
        publish_smooth_trajectory(
            controller_topic="/low_to_high_position_controller/command",
            start_rad=y_start_rad,
            target_rad=y_target_rad,
            total_time=y_total_time
        )

    rospy.loginfo("所有关节匀速控制流程完成")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断，控制流程终止")
    except Exception as e:
        rospy.logerr(f"控制过程中发生未知错误：{str(e)}")