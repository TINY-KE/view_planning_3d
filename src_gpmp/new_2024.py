#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose


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
 


         #gpmp规划结果 0, size:7, values: 
        # [0.351940230481, 0.793240799264, -3.05627426035, 1.50352067777, 0.0920026679796, -0.706146545288, 2.66064558025];
        joint_positions = [0.351940230481, 0.793240799264, -3.05627426035, 1.50352067777, 0.0920026679796, -0.706146545288, 2.66064558025];
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("0 success")

        #  #gpmp规划结果 1, size:7, values: 
        # [0.367511493849, 0.852895885528, -3.06707732717, 1.5053245551, 0.0920760686587, -0.655856142936, 2.65196387781];
        
        #  #gpmp规划结果 2, size:7, values: 
        # [0.407531880218, 1.00898492105, -3.08935571892, 1.51655844333, 0.0906488117557, -0.509939853408, 2.62704770996];
        #  #gpmp规划结果 3, size:7, values: 
        # [0.455342821739, 1.22744724819, -3.11235625246, 1.53046515022, 0.0919765578251, -0.304362291405, 2.58863410034];
        #  #gpmp规划结果 4, size:7, values: 
        # [0.490314686097, 1.46377401801, -3.13259524207, 1.5466386165, 0.107637257079, -0.0833491890951, 2.54305046495];
        #  #gpmp规划结果 5, size:7, values: 
        # [0.491200926693, 1.67713531102, -3.15776341919, 1.57114874748, 0.150865907235, 0.107208262677, 2.49866189828];
        joint_positions = [0.491200926693, 1.67713531102, -3.15776341919, 1.57114874748, 0.150865907235, 0.107208262677, 2.49866189828];
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("5 success")


        #  #gpmp规划结果 6, size:7, values: 
        # [0.440338310191, 1.83805004914, -3.19554861838, 1.61382478632, 0.229759151414, 0.230446622175, 2.46310230496];
        #  #gpmp规划结果 7, size:7, values: 
        # [0.331582705898, 1.93324462935, -3.23869971586, 1.6797442885, 0.346268772419, 0.27040917826, 2.44086282541];
        #  #gpmp规划结果 8, size:7, values: 
        # [0.178307880459, 1.9676144123, -3.26401703831, 1.75959656808, 0.496676601614, 0.238632726172, 2.43103702827];
        #  #gpmp规划结果 9, size:7, values: 
        # [0.00731006476561, 1.95631184869, -3.24811132698, 1.83309769997, 0.670354423694, 0.159248824889, 2.42993122141];
        #  #gpmp规划结果 10, size:7, values: 
        # [-0.151479341545, 1.9150495867, -3.18407495471, 1.87995123855, 0.846856310491, 0.0533898652573, 2.43257691504];
        #  #gpmp规划结果 11, size:7, values: 
        # [-0.274058888279, 1.85324520372, -3.08280207367, 1.89009690504, 0.998795613681, -0.0671588324758, 2.43428549301];
        joint_positions = [-0.274058888279, 1.85324520372, -3.08280207367, 1.89009690504, 0.998795613681, -0.0671588324758, 2.43428549301];
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("11 success")

        #  #gpmp规划结果 12, size:7, values: 
        # [-0.347882404738, 1.77207648111, -2.96365350409, 1.86501762188, 1.09973872516, -0.195872640925, 2.43352662513];
        #  #gpmp规划结果 13, size:7, values: 
        # [-0.373900742283, 1.66700966669, -2.84540644971, 1.81402325203, 1.1328313979, -0.326574716602, 2.43318445006];
        #  #gpmp规划结果 14, size:7, values: 
        # [-0.365654183525, 1.53475182955, -2.74182574714, 1.7493569175, 1.09794304577, -0.452134645864, 2.43831665067];
        #  #gpmp规划结果 15, size:7, values: 
        # [-0.340638836258, 1.38331074372, -2.66180242614, 1.68332834234, 1.01275817367, -0.56403720903, 2.45093633702];
        #  #gpmp规划结果 16, size:7, values: 
        # [-0.313307583835, 1.23490328262, -2.60881268896, 1.62603787088, 0.908403715, -0.653729192578, 2.46872105019];
        #  #gpmp规划结果 17, size:7, values: 
        # [-0.293311999339, 1.12203450384, -2.58085181101, 1.58554421194, 0.822270847028, -0.712806726269, 2.48540193947];
        #  #gpmp规划结果 18, size:7, values: 
        # [-0.286756010521, 1.07274557777, -2.57588829602, 1.57456602876, 0.791562891684, -0.733216523738, 2.49196509205];
        joint_positions = [-0.286756010521, 1.07274557777, -2.57588829602, 1.57456602876, 0.791562891684, -0.733216523738, 2.49196509205];
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("18 success")



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
