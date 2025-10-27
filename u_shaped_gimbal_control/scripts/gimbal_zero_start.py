import rospy
import math
import argparse  # 导入处理命令行参数的模块
from std_msgs.msg import Float64

"""
功能: 控制云台结构，绕Z轴（水平）和Y轴（垂直）旋转，支持命令行输入目标角度
运行示例:
  1. 自定义角度：python scripts/joint_control.py 10 15 （Z轴10度，Y轴15度）
  2. 使用默认角度：python scripts/joint_control.py （默认Z轴20度，Y轴30度）
"""

def angle_to_radian(angle_deg):
    """将角度（度）转换为弧度"""
    return math.radians(angle_deg)

def publish_command(controller_topic, target_radian, duration=2.0):
    """
    向指定控制器发布目标角度命令，持续duration秒后退出
    controller_topic: 控制器话题
    target_radian: 目标弧度
    duration: 发布持续时间（确保关节有足够时间到达目标位置）
    """
    pub = rospy.Publisher(controller_topic, Float64, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz发布频率
    start_time = rospy.get_time()
    
    # 持续发布duration秒，或节点关闭时退出
    while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
        pub.publish(Float64(data=target_radian))
        rate.sleep()
    rospy.loginfo(f"已向{controller_topic}发布命令{duration}秒")

def main():
    # -------------------------- 新增：解析命令行参数 --------------------------
    parser = argparse.ArgumentParser(description="云台关节控制脚本，支持命令行输入Z轴和Y轴目标角度（度）")
    # 添加Z轴角度参数（positional argument，位置参数），类型为float，默认值20.0
    parser.add_argument(
        "z_angle",  # 参数名（在代码中通过args.z_angle获取）
        type=float, 
        nargs='?',  # 支持"可选输入"（不输入时用默认值）
        default=20.0, 
        help="Z轴（水平旋转）目标角度（度），默认20.0度"
    )
    # 添加Y轴角度参数，类型为float，默认值30.0
    parser.add_argument(
        "y_angle", 
        type=float, 
        nargs='?', 
        default=30.0, 
        help="Y轴（垂直旋转）目标角度（度），默认30.0度"
    )
    # 解析参数（args对象将包含z_angle和y_angle属性）
    args = parser.parse_args()
    # --------------------------------------------------------------------------

    # 初始化ROS节点
    rospy.init_node('joint_sequence_controller')
    
    # 1. 控制Z轴旋转（使用命令行输入的z_angle，或默认值）
    z_target_deg = args.z_angle  # 从命令行参数获取Z轴目标角度
    z_target_rad = angle_to_radian(z_target_deg)
    rospy.loginfo(f"开始控制Z轴(水平旋转): {z_target_deg}度")
    publish_command(
        controller_topic="/root_to_low_position_controller/command",
        target_radian=z_target_rad,
        duration=3.0
    )
    
    # 等待1秒，确保Z轴运动完成后再控制Y轴
    rospy.sleep(1.0)
    
    # 2. 控制Y轴旋转（使用命令行输入的y_angle，或默认值）
    y_target_deg = args.y_angle  # 从命令行参数获取Y轴目标角度
    y_target_rad = angle_to_radian(y_target_deg)
    rospy.loginfo(f"开始控制Y轴(垂直旋转): {y_target_deg}度")
    publish_command(
        controller_topic="/low_to_high_position_controller/command",
        target_radian=y_target_rad,
        duration=3.0
    )
    
    rospy.loginfo("所有控制命令发布完成")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"错误：{str(e)}")