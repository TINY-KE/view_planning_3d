#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64MultiArray
from functools import partial

# joint_values_gpmp 


def joint_values_callback(arm, data):

    # print("运行到行号：",sys._getframe().f_lineno)

    # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
    # x0: [-3.0847299992, -1.76330430583, 1.85520868669, 0.433015552185, -2.67246194478, 0.469249715246, 4.0008642662]
    joint_positions = [data[0], data[1], data[2], data[3], data[4], data[5], data[6]]
    arm.set_joint_value_target(joint_positions)
    arm.go()
    rospy.loginfo("success")






class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
        # 设置机械臂和夹爪的允许误差值
        arm.set_goal_joint_tolerance(0.001)
        # arm.set_planner_id("RRTConnectkConfigDefault")
        arm.set_planner_id("RRTstar")
        reference_frame = 'base_link'

        # sub
        rospy.Subscriber('/joint_values_gpmp', Float64MultiArray, partial(joint_values_callback,arm))
        rospy.spin()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)



    # 设置场景物体的颜色
    def setColor(self, name, r, g, b, a = 0.9):
        # 初始化moveit颜色对象
        color = ObjectColor()
        
        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # 更新颜色字典
        self.colors[name] = color

    # 将颜色设置发送并应用到moveit场景当中
    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()

        # 需要设置规划场景是否有差异     
        p.is_diff = True
        
        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # 发布场景物体颜色设置
        self.scene_pub.publish(p)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass
