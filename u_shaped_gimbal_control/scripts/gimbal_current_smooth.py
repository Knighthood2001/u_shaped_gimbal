import rospy
import math
import argparse
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState  # 用于获取关节当前状态


def angle_to_radian(angle_deg):
    """将角度（度）转换为弧度"""
    return math.radians(angle_deg)


def get_current_joint_radian(joint_name, timeout=5.0):
    """
    获取指定关节的当前角度（弧度）
    joint_name: 关节名称（需与URDF中定义一致）
    timeout: 等待关节状态消息的超时时间（秒）
    返回：关节当前角度（弧度），失败则返回None
    """
    try:
        # 等待关节状态消息（同步获取，确保拿到最新数据）
        joint_state = rospy.wait_for_message('/joint_states', JointState, timeout=timeout)
        # 查找目标关节在消息中的索引
        if joint_name in joint_state.name:
            idx = joint_state.name.index(joint_name)
            return joint_state.position[idx]  # 直接返回弧度（关节状态消息中存储的是弧度）
        else:
            rospy.logerr(f"未找到关节名称: {joint_name}，请检查URDF定义")
            return None
    except rospy.ROSException:
        rospy.logerr(f"获取关节状态超时（{timeout}秒），请确保关节状态话题已发布")
        return None


def publish_smooth_trajectory(controller_topic, start_rad, target_rad, total_time=5.0, step_interval=0.1):
    """
    平滑发布轨迹：从起始角度逐步过渡到目标角度
    controller_topic: 控制器话题
    start_rad: 起始角度（弧度，从当前位置获取）
    target_rad: 目标角度（弧度）
    total_time: 总转动时间（秒），时间越长转动越慢
    step_interval: 每步间隔时间（秒），控制发布频率
    """
    pub = rospy.Publisher(controller_topic, Float64, queue_size=10)
    total_steps = int(total_time / step_interval)
    if total_steps == 0:
        total_steps = 1  # 避免除以0
    angle_increment = (target_rad - start_rad) / total_steps  # 每步角度增量（弧度）
    
    rospy.loginfo(
        f"开始平滑转动（总耗时{total_time}秒），从{start_rad:.2f}弧度到{target_rad:.2f}弧度"
    )
    
    current_rad = start_rad
    for _ in range(total_steps):
        if rospy.is_shutdown():
            break
        pub.publish(Float64(data=current_rad))
        current_rad += angle_increment
        rospy.sleep(step_interval)
    
    pub.publish(Float64(data=target_rad))  # 最后一步确保到达目标
    rospy.loginfo(f"{controller_topic} 已到达目标角度")


def main():
    parser = argparse.ArgumentParser(description="云台关节平滑控制脚本（从当前位置开始转动）")
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

    rospy.init_node('joint_smooth_controller_from_current')
    
    # -------------------------- 核心修改：获取当前角度作为起始点 --------------------------
    # 关节名称（必须替换为你的URDF中实际的关节名称！通过rostopic echo /joint_states查看）
    z_joint_name = "joint_low"  # Z轴旋转关节的名称
    y_joint_name = "joint_up"  # Y轴旋转关节的名称
    
    # 获取Z轴当前角度（弧度）
    z_start_rad = get_current_joint_radian(z_joint_name)
    if z_start_rad is None:
        rospy.logerr("无法获取Z轴当前角度，退出控制")
        return
    
    # 获取Y轴当前角度（弧度）
    y_start_rad = get_current_joint_radian(y_joint_name)
    if y_start_rad is None:
        rospy.logerr("无法获取Y轴当前角度，退出控制")
        return
    # ----------------------------------------------------------------------------------

    # 1. 平滑控制Z轴（从当前角度开始）
    z_target_rad = angle_to_radian(args.z_angle)
    rospy.loginfo(
        f"Z轴目标：{args.z_angle}度（{z_target_rad:.2f}弧度），转动时间：{args.z_time}秒"
    )
    publish_smooth_trajectory(
        controller_topic="/root_to_low_position_controller/command",
        start_rad=z_start_rad,  # 用当前角度作为起始
        target_rad=z_target_rad,
        total_time=args.z_time
    )
    
    rospy.sleep(0.5)  # 等待Z轴转动完成
    
    # 2. 平滑控制Y轴（从当前角度开始）
    y_target_rad = angle_to_radian(args.y_angle)
    rospy.loginfo(
        f"Y轴目标：{args.y_angle}度（{y_target_rad:.2f}弧度），转动时间：{args.y_time}秒"
    )
    publish_smooth_trajectory(
        controller_topic="/low_to_high_position_controller/command",
        start_rad=y_start_rad,  # 用当前角度作为起始
        target_rad=y_target_rad,
        total_time=args.y_time
    )
    
    rospy.loginfo("所有转动完成")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"错误：{str(e)}")