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






        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）

        
        
        # # x0: [-3.0847299992, -1.76330430583, 1.85520868669, 0.433015552185, -2.67246194478, 0.469249715246, 4.0008642662]
        # joint_positions = [-3.0847299992, -1.76330430583, 1.85520868669, 0.433015552185, -2.67246194478, 0.469249715246, 4.0008642662]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.loginfo("0 success")


        # # x1: [-3.04402376537, -1.69884396073, 1.8702978783, 0.382855931446, -2.67168260047, 0.354481125083, 3.97018391619]
        # joint_positions = [-3.04402376537, -1.69884396073, 1.8702978783, 0.382855931446, -2.67168260047, 0.354481125083, 3.97018391619]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.loginfo("1 success")


        # # x2: [-2.95116569275, -1.52304376833, 1.90034534245, 0.257925461332, -2.67164417053, 0.0598456237564, 3.90139668332]
        # joint_positions = [-2.95116569275, -1.52304376833, 1.90034534245, 0.257925461332, -2.67164417053, 0.0598456237564, 3.90139668332]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.loginfo("2 success")

        # # x3: [-2.84016953283, -1.27365861661, 1.92510001382, 0.0904561656591, -2.67786271753, -0.341063948932, 3.83080612055]
        # joint_positions = [-2.84016953283, -1.27365861661, 1.92510001382, 0.0904561656591, -2.67786271753, -0.341063948932, 3.83080612055]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.loginfo("3 success")
        

        # # x4: [-2.72134526088, -1.01280662375, 1.92956702004, -0.101649336678, -2.69809448942, -0.775884855127, 3.79328166499]
        # joint_positions = [-2.72134526088, -1.01280662375, 1.92956702004, -0.101649336678, -2.69809448942, -0.775884855127, 3.79328166499]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.loginfo("4 success")


        # # x5: [-2.58567851684, -0.825155147096, 1.90303806418, -0.310497240648, -2.73869858, -1.17574498534, 3.8167529028]
        # joint_positions = [-2.58567851684, -0.825155147096, 1.90303806418, -0.310497240648, -2.73869858, -1.17574498534, 3.8167529028]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.loginfo("5 success")


        # # x6: [-2.43355648621, -0.792987639679, 1.84241239677, -0.519925109608, -2.80024776011, -1.48505123901, 3.91971597317]
        # joint_positions = [-2.43355648621, -0.792987639679, 1.84241239677, -0.519925109608, -2.80024776011, -1.48505123901, 3.91971597317]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.loginfo("6 success")


        # # x7: [-2.3260014257, -0.919852532252, 1.7643924379, -0.685561568672, -2.87463717754, -1.67098533434, 4.10573271335]    
        # joint_positions = [-2.3260014257, -0.919852532252, 1.7643924379, -0.685561568672, -2.87463717754, -1.67098533434, 4.10573271335]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.loginfo("7 success")


        # # x8: [-2.33211407107, -1.16731986662, 1.6766113255, -0.777686948915, -2.9656928462, -1.69687193109, 4.35956934715]
        # joint_positions = [-2.33211407107, -1.16731986662, 1.6766113255, -0.777686948915, -2.9656928462, -1.69687193109, 4.35956934715]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.loginfo("8 success")


        # # x9: [-2.31896464051, -1.45548138621, 1.58346161741, -0.963275514931, -3.07103813556, -1.39172520186, 4.60656367297]
        # joint_positions = [-2.31896464051, -1.45548138621, 1.58346161741, -0.963275514931, -3.07103813556, -1.39172520186, 4.60656367297]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.loginfo("9 success")


        # # x10: [-2.0276392339, -1.5961786118, 1.54495801145, -1.37076960904, -3.12070893865, -0.859890926582, 4.70640728912]
        # # joint_positions = [-2.0276392339, -1.5961786118, 1.54495801145, -1.37076960904, -3.12070893865, -0.859890926582, 4.70640728912]
        # joint_positions = [-2.018431036907354, -1.4999897089911451, 1.4046777996889483, -1.3707039693409548, -3.0999924865261397, -0.8560425214202461, 4.8622166345079165]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.loginfo("10 success")
        # # 新x10: [-2.02750724615, -1.59799481955, 1.54757207374, -1.37065803169, -3.12111444241, -0.860010740512, 4.70351089598]







        # Value x0: (N5Eigen6MatrixIdLin1ELi1ELi0ELin1ELi1EEE) [
        #     -3.08473355446;
        #         -1.76330283406;
        #         1.85520856819;
        #         0.433013790715;
        #         -2.67246341465;
        #         0.469250576039;
        #         4.00086430033
        # ]
        joint_positions = [-3.08473355446, -1.76330283406, 1.85520856819, 0.433013790715, -2.67246341465, 0.469250576039, 4.00086430033]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("0 success")



        # Value x1: (N5Eigen6MatrixIdLin1ELi1ELi0ELin1ELi1EEE) [
        #     -3.17844550908;
        #         -1.72649508867;
        #         1.89459539244;
        #         0.22970737266;
        #         -2.74396899574;
        #         0.333936571011;
        #         3.99942638755
        # ]
        # gpmp真实生成的 
        joint_positions = [-3.17844550908, -1.72649508867, 1.89459539244, 0.2297737266, -2.74396899574, 0.333936571011, 3.99942638755]
        # 用于调参urdf范围的
        # joint_positions = [-3.07844550908, -1.72649508867, 1.89459539244, 0.22970737266, -2.74396899574, 0.333936571011, 3.99942638755]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("1 success")


        # Value x2: (N5Eigen6MatrixIdLin1ELi1ELi0ELin1ELi1EEE) [
        #     -3.30888840986;
        #         -1.69307465205;
        #         1.99715243665;
        #         -0.25647974396;
        #         -2.88367284051;
        #         -0.0632907731752;
        #         4.00797316936
        # ]
        # gpmp真实生成的 
        joint_positions = [-3.30888840986, -1.69307465205, 1.99715243665, -0.25647974396, -2.88367284051, -0.0632907731752, 4.00797316936]
        # 用于调参urdf范围的
        # joint_positions = [-3.00888840986, -1.69307465205, 1.99715243665, -0.25647974396, -2.88367284051, -0.0632907731752, 4.00797316936]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("2 success")


        # Value x3: (N5Eigen6MatrixIdLin1ELi1ELi0ELin1ELi1EEE) [
        #     -3.31258973057;
        #         -1.64595837721;
        #         2.12898341235;
        #         -0.829428476807;
        #         -2.99423717138;
        #         -0.670308720113;
        #         4.01324159671
        # ]
        joint_positions = [-3.31258973057, -1.64595837721, 2.12898341235, -0.829428476807, -2.99423717138, -0.670308720113, 4.01324159671]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("3 success")


        # Value x4: (N5Eigen6MatrixIdLin1ELi1ELi0ELin1ELi1EEE) [
        #     -3.17351353224;
        #         -1.44521556188;
        #         2.25258137864;
        #         -1.2945228808;
        #         -3.02919741971;
        #         -1.35859039658;
        #         3.99173047443
        # ]
        joint_positions = [-3.17351353224, -1.44521556188, 2.25258137864, -1.2945228808, -3.02919741971, -1.35859039658, 3.99173047443]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("4 success")


        # Value x5: (N5Eigen6MatrixIdLin1ELi1ELi0ELin1ELi1EEE) [
        #     -2.91842937898;
        #         -1.19542995153;
        #         2.27179136829;
        #         -1.56103894136;
        #         -3.03683632611;
        #         -1.97920417923;
        #         3.97742307692
        # ]
        joint_positions = [-2.91842937898, -1.19542995153, 2.27179136829, -1.56103894136, -3.03683632611, -1.97920417923, 3.97742307692]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("5 success")


        # Value x6: (N5Eigen6MatrixIdLin1ELi1ELi0ELin1ELi1EEE) [
        #     -2.55680149679;
        #         -1.04432833315;
        #         2.11922933233;
        #         -1.60245667655;
        #         -3.05896591984;
        #         -2.37220456659;
        #         4.03041062825
        # ]
        joint_positions = [-2.55680149679, -1.04432833315, 2.11922933233, -1.60245667655, -3.05896591984, -2.37220456659, 4.03041062825]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("6 success")


        # Value x7: (N5Eigen6MatrixIdLin1ELi1ELi0ELin1ELi1EEE) [
        #     -2.20648944191;
        #         -1.0766044864;
        #         1.87794236265;
        #         -1.4302296471;
        #         -3.08312790324;
        #         -2.48374645349;
        #         4.18102806791
        # ]
        joint_positions = [-2.20648944191, -1.0766044864, 1.87794236265, -1.4302296471, -3.08312790324, -2.48374645349, 4.18102806791]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("7 success")


        # Value x8: (N5Eigen6MatrixIdLin1ELi1ELi0ELin1ELi1EEE) [
        #     -2.03703794849;
        #         -1.25481188567;
        #         1.68762864943;
        #         -1.15786958531;
        #         -3.09736490122;
        #         -2.29429273749;
        #         4.40587847588
        # ]
        joint_positions = [-2.03703794849, -1.25481188567, 1.68762864943, -1.15786958531, -3.09736490122, -2.29429273749, 4.40587847588]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("8 success")


        # Value x9: (N5Eigen6MatrixIdLin1ELi1ELi0ELin1ELi1EEE) [
        #     -2.09927246032;
        #         -1.47847052262;
        #         1.58087848749;
        #         -1.08036710294;
        #         -3.11041089845;
        #         -1.63415604036;
        #         4.62122869264
        # ]
        joint_positions = [-2.09927246032, -1.47847052262, 1.58087848749, -1.08036710294, -3.11041089845, -1.63415604036, 4.62122869264]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("9 success")


        # Value x10: (N5Eigen6MatrixIdLin1ELi1ELi0ELin1ELi1EEE) [
        #     -2.02748471766;
        #         -1.59569874652;
        #         1.54425665921;
        #         -1.3708116248;
        #         -3.12061153762;
        #         -0.860078342368;
        #         4.70719837076
        # ]
        joint_positions = [-2.02748471766, -1.59569874652, 1.54425665921, -1.3708116248, -3.12061153762, -0.860078342368, 4.70719837076]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("10 success")
        


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
