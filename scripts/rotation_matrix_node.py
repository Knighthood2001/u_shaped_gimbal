#!/usr/bin/env python
import rospy
import tf
import numpy as np

"""
运行rviz后可以使用这个代码
获取两个坐标系间的4x4齐次变换矩阵

运行结果如下：
[INFO] [1760577775.030178, 52125.452000]: 
base_link -> link_front 的4x4齐次变换矩阵:
[1.0000, 0.0000, 0.0000, 0.0000]
[0.0000, 0.7725, 0.6350, -0.2793]
[0.0000, -0.6350, 0.7725, 0.1090]
[0.0000, 0.0000, 0.0000, 1.0000]

"""
def get_4x4_transform_matrix():
    # 初始化节点
    rospy.init_node('4x4_transform_matrix_node', anonymous=True)
    
    # 定义需要查询的两个坐标系（base_link -> link_front 的变换）
    source_frame = "base_link"    # 源坐标系
    target_frame = "link_front"  # 目标坐标系（结果表示：target = 4x4矩阵 * source）
    
    # 创建tf监听器（监听tf树中的变换）
    listener = tf.TransformListener()
    
    # 循环频率（10Hz）
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        try:
            # 等待两个坐标系之间的变换可用（超时时间1秒）
            listener.waitForTransform(
                target_frame,   # 目标坐标系
                source_frame,   # 源坐标系
                rospy.Time(0),  # 0表示获取最新的变换
                rospy.Duration(1.0)  # 超时时间
            )
            
            # 获取变换：(平移向量(x,y,z), 旋转四元数(x,y,z,w))
            (trans, quat) = listener.lookupTransform(
                target_frame, 
                source_frame, 
                rospy.Time(0)
            )
            
            # 1. 生成4x4旋转矩阵（前3x3为旋转，第四行/列为默认值）
            # quaternion_matrix仅处理旋转，平移需要手动填充
            transform_matrix_4x4 = tf.transformations.quaternion_matrix(quat)
            
            # 2. 填充平移向量到4x4矩阵的第4列（前3行）
            # 4x4矩阵结构：[R[0][0], R[0][1], R[0][2], trans.x]
            #              [R[1][0], R[1][1], R[1][2], trans.y]
            #              [R[2][0], R[2][1], R[2][2], trans.z]
            #              [0      , 0      , 0      , 1      ]
            transform_matrix_4x4[0, 3] = trans[0]  # x方向平移
            transform_matrix_4x4[1, 3] = trans[1]  # y方向平移
            transform_matrix_4x4[2, 3] = trans[2]  # z方向平移
            
            # 3. 打印完整4x4变换矩阵
            rospy.loginfo("\n%s -> %s 的4x4齐次变换矩阵:", source_frame, target_frame)
            for row in transform_matrix_4x4:
                print("[" + ", ".join(f"{x:.4f}" for x in row) + "]")
            print("\n")
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # 处理变换查询异常
            rospy.logwarn("获取变换失败: %s", e)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        get_4x4_transform_matrix()
    except rospy.ROSInterruptException:
        pass
