// 必须定义此宏才能使用M_PI（圆周率）
#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <iomanip>  // 用于格式化浮点数输出

// -------------------------- 角度转换工具函数 --------------------------
/**
 * @brief 角度（度）转弧度
 * @param deg 角度值（度）
 * @return 弧度值
 */
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

/**
 * @brief 弧度转角度（度）
 * @param rad 弧度值
 * @return 角度值（度）
 */
inline double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

// -------------------------- 获取关节当前弧度 --------------------------
/**
 * @brief 从/joint_states话题获取指定关节的当前角度（弧度）
 * @param joint_name 目标关节名称（需与URDF一致）
 * @param timeout 超时时间（秒）
 * @return 关节当前弧度（失败返回NaN）
 */
double getCurrentJointRadian(const std::string& joint_name, double timeout = 5.0) {
    // 同步等待关节状态消息（超时返回空指针）
    sensor_msgs::JointState::ConstPtr joint_state = 
        ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", 
                                                           ros::Duration(timeout));
    
    // 检查消息是否获取成功
    if (!joint_state) {
        ROS_ERROR_STREAM("获取关节状态超时（" << timeout << "秒），请确认/joint_states话题已发布");
        return std::nan("");  // 返回NaN表示失败
    }

    // 查找目标关节在消息中的索引
    for (size_t i = 0; i < joint_state->name.size(); ++i) {
        if (joint_state->name[i] == joint_name) {
            // 关节状态消息中position存储的是弧度
            return joint_state->position[i];
        }
    }

    // 未找到关节
    ROS_ERROR_STREAM("未找到关节名称：" << joint_name << "，请通过 rostopic echo /joint_states 确认URDF关节名");
    return std::nan("");
}

// -------------------------- 平滑发布轨迹 --------------------------
/**
 * @brief 匀速发布轨迹（从起始弧度到目标弧度）
 * @param nh ROS节点句柄
 * @param controller_topic 控制器话题（如"/root_to_low_position_controller/command"）
 * @param start_rad 起始角度（弧度）
 * @param target_rad 目标角度（弧度）
 * @param total_time 总转动时间（秒）
 * @param step_interval 每步间隔时间（秒，默认0.1秒）
 */
void publishSmoothTrajectory(ros::NodeHandle& nh,
                             const std::string& controller_topic,
                             double start_rad,
                             double target_rad,
                             double total_time,
                             double step_interval = 0.1) {
    // 创建发布者（队列大小10，与Python一致）
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(controller_topic, 10);
    // 等待发布者与订阅者建立连接（确保消息能被接收）
    ros::Duration(0.5).sleep();

    // 计算总步数（避免除以0，最小1步）
    int total_steps = (total_time > 0) ? static_cast<int>(total_time / step_interval) : 1;
    // 每步角度增量（弧度）
    double angle_increment = (target_rad - start_rad) / total_steps;

    // 打印转动信息（与Python日志格式对齐）
    ROS_INFO_STREAM(std::fixed << std::setprecision(2)
                   << "开始匀速转动：从" << rad2deg(start_rad) << "度到" << rad2deg(target_rad) << "度，"
                   << "总耗时" << total_time << "秒，速率" << std::abs(rad2deg(angle_increment) / step_interval) << "度/秒");

    // 发布中间角度
    double current_rad = start_rad;
    std_msgs::Float64 msg;  // 复用消息对象，减少创建开销
    for (int i = 0; i < total_steps && ros::ok(); ++i) {
        msg.data = current_rad;
        pub.publish(msg);
        
        // 更新当前角度，等待下一步
        current_rad += angle_increment;
        ros::Duration(step_interval).sleep();
    }

    // 最后发布一次目标角度（消除累积误差）
    msg.data = target_rad;
    pub.publish(msg);
    ROS_INFO_STREAM("[" << controller_topic << "] 已到达目标角度");
}

// -------------------------- 主函数 --------------------------
int main(int argc, char** argv) {
    // 1. 初始化ROS节点（与Python节点名一致）
    ros::init(argc, argv, "joint_speed_control_from_current");
    ros::NodeHandle nh;

    // 2. 解析命令行参数（与Python位置参数逻辑一致，支持默认值）
    double z_angle = 20.0;    // Z轴目标角度（度）默认值
    double y_angle = 30.0;    // Y轴目标角度（度）默认值
    double z_speed = 10.0;    // Z轴速度（度/秒）默认值
    double y_speed = 10.0;    // Y轴速度（度/秒）默认值

    // 根据argc数量赋值（argv[0]是程序名，从argv[1]开始是参数）
    if (argc >= 2) z_angle = atof(argv[1]);
    if (argc >= 3) y_angle = atof(argv[2]);
    if (argc >= 4) z_speed = atof(argv[3]);
    if (argc >= 5) y_speed = atof(argv[4]);

    // 3. 定义关节名称（！！！必须替换为你的URDF实际关节名！！！）
    const std::string z_joint_name = "joint_low";  // Z轴（水平）关节名
    const std::string y_joint_name = "joint_up";   // Y轴（垂直）关节名

    // 4. 获取当前关节角度（弧度）
    double z_start_rad = getCurrentJointRadian(z_joint_name);
    double y_start_rad = getCurrentJointRadian(y_joint_name);

    // 检查角度获取是否成功（NaN表示失败）
    if (std::isnan(z_start_rad)) {
        ROS_ERROR("Z轴当前角度获取失败，退出控制流程");
        return 1;
    }
    if (std::isnan(y_start_rad)) {
        ROS_ERROR("Y轴当前角度获取失败，退出控制流程");
        return 1;
    }

    // 5. 打印调试信息（格式与Python一致，保留2位小数）
    ROS_INFO_STREAM("\n" << std::fixed << std::setprecision(2)
                   << "当前角度：Z轴 " << rad2deg(z_start_rad) << "度，Y轴 " << rad2deg(y_start_rad) << "度\n"
                   << "目标角度：Z轴 " << z_angle << "度，Y轴 " << y_angle << "度\n"
                   << "转动速度：Z轴 " << z_speed << "度/秒，Y轴 " << y_speed << "度/秒\n");

    // 6. 控制Z轴匀速转动
    double z_angle_diff = std::abs(z_angle - rad2deg(z_start_rad));  // 角度差（度）
    if (z_angle_diff < 0.1) {  // 角度差小于0.1度，无需转动
        ROS_INFO_STREAM(std::fixed << std::setprecision(2)
                       << "Z轴角度差（" << z_angle_diff << "度）小于阈值，无需转动");
    } else {
        double z_total_time = z_angle_diff / z_speed;  // 总转动时间 = 角度差 / 速度
        double z_target_rad = deg2rad(z_angle);        // 目标角度转弧度
        publishSmoothTrajectory(nh,
                                "/root_to_low_position_controller/command",
                                z_start_rad,
                                z_target_rad,
                                z_total_time);
    }

    // 等待Z轴转动完成（与Python一致，等待0.5秒）
    ros::Duration(0.5).sleep();

    // 7. 控制Y轴匀速转动（逻辑与Z轴一致）
    double y_angle_diff = std::abs(y_angle - rad2deg(y_start_rad));
    if (y_angle_diff < 0.1) {
        ROS_INFO_STREAM(std::fixed << std::setprecision(2)
                       << "Y轴角度差（" << y_angle_diff << "度）小于阈值，无需转动");
    } else {
        double y_total_time = y_angle_diff / y_speed;
        double y_target_rad = deg2rad(y_angle);
        publishSmoothTrajectory(nh,
                                "/low_to_high_position_controller/command",
                                y_start_rad,
                                y_target_rad,
                                y_total_time);
    }

    // 8. 完成日志
    ROS_INFO_STREAM("\n所有关节匀速控制流程完成");

    // 保持节点运行（可选，此处控制流程已结束，可直接返回）
    ros::spinOnce();
    return 0;
}