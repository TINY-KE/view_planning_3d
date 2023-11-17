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

        
        # 开始规划,0
        #  gpmp计算结果 0, size:7, values: 1.43321052869e-07, -0.406363067491, 1.43319067198e-07, 0.324361318292, 1.43319477539e-07, 0.923783666006, 1.4331912416e-07
        joint_positions = [1.43321052869e-07, -0.406363067491, 1.43319067198e-07, 0.324361318292, 1.43319477539e-07, 0.923783666006, 1.4331912416e-07]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("0 success")

        # 开始规划,1
        # gpmp计算结果 1, size:7, values: -1.03195597012e-06, -0.405814864096, -1.03195805775e-06, 0.324294836728, -1.0319576265e-06, 0.923212950685, -1.03195799804e-06
        joint_positions = [-1.03195597012e-06, -0.405814864096, -1.03195805775e-06, 0.324294836728, -1.0319576265e-06, 0.923212950685, -1.03195799804e-06]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("1 success")

        
        # 开始规划,2
        # gpmp计算结果 2, size:7, values: -0.018889446028, -0.186335098683, -0.0188894460304, 0.312854539981, -0.0188894460299, 0.722346040451, -0.0188894460303
        joint_positions = [-0.018889446028, -0.186335098683, -0.0188894460304, 0.312854539981, -0.0188894460299, 0.722346040451, -0.0188894460303]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("2 success")

        # 开始规划,3
        # gpmp计算结果 3, size:7, values: -0.0405386937881, 0.0227981297846, -0.0405386937907, 0.303098076807, -0.0405386937902, 0.533031627103, -0.0405386937906
        joint_positions = [-0.0405386937881, 0.0227981297846, -0.0405386937907, 0.303098076807, -0.0405386937902, 0.533031627103, -0.0405386937906]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("3 success")

        # 开始规划,4
        # gpmp计算结果 4, size:7, values: -0.0655728150577, 0.218918115446, -0.0655728150608, 0.29490004682, -0.0655728150603, 0.357228974904, -0.0655728150608
        joint_positions = [-0.0655728150577, 0.218918115446, -0.0655728150608, 0.29490004682, -0.0655728150603, 0.357228974904, -0.0655728150608]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("4 success")

        # 开始规划,5
        # gpmp计算结果 5, size:7, values: -0.0942781155388, 0.399563173504, -0.0942781155422, 0.287945592738, -0.0942781155417, 0.19638429602, -0.0942781155422
        joint_positions = [-0.0942781155388, 0.399563173504, -0.0942781155422, 0.287945592738, -0.0942781155417, 0.19638429602, -0.0942781155422]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.loginfo("5 success")
        
        # 开始规划,6
        # gpmp计算结果 6, size:7, values: -0.126061143887, 0.562224397533, -0.12606114389, 0.281934339907, -0.12606114389, 0.052008902016, -0.12606114389
        # 开始规划,7
        # gpmp计算结果 7, size:7, values: -0.0866124718012, 0.586547266172, -0.0866124718045, 0.274670950096, -0.0866124718039, 0.0188349095707, -0.0866124718044
        # 开始规划,8
        # gpmp计算结果 8, size:7, values: -0.0430961756211, 0.579360738301, -0.043096175624, 0.267033415877, -0.0430961756234, 0.0108274092083, -0.043096175624
        # 开始规划,9
        # gpmp计算结果 9, size:7, values: -1.87004868813e-09, 0.535599504842, -1.87268253761e-09, 0.259214870702, -1.87203986549e-09, 0.0324931005179, -1.8728265466e-09
        # 开始规划,10
        # gpmp计算结果 10, size:7, values: 0.0430961757854, 0.579364563231, 0.0430961757829, 0.267033051949, 0.0430961757835, 0.0108236091126, 0.0430961757825
        # 开始规划,11
        # gpmp计算结果 11, size:7, values: 0.0866127582115, 0.586500223865, 0.0866127582092, 0.274675412373, 0.0866127582097, 0.0188816217133, 0.0866127582085
        # 开始规划,12
        # gpmp计算结果 12, size:7, values: 0.126056897036, 0.562579528725, 0.126056897034, 0.281900975968, 0.126056897034, 0.0516568506791, 0.126056897033
        # 开始规划,13
        # gpmp计算结果 13, size:7, values: 0.126064739661, 0.562146207938, 0.126064739659, 0.281942246379, 0.12606473966, 0.0520874341837, 0.126064739658
        # 开始规划,14
        # gpmp计算结果 14, size:7, values: 0.0942776675344, 0.399573213392, 0.0942776675331, 0.287944512154, 0.0942776675333, 0.19637409319, 0.0942776675321
        # 开始规划,15
        # gpmp计算结果 15, size:7, values: 0.0655728657224, 0.218915812146, 0.0655728657212, 0.294900324091, 0.0655728657213, 0.357231369069, 0.0655728657203
        # 开始规划,16
        # gpmp计算结果 16, size:7, values: 0.0405385612457, 0.0228171755766, 0.0405385612446, 0.303095630687, 0.0405385612448, 0.533011550918, 0.0405385612439
        # 开始规划,17
        # gpmp计算结果 17, size:7, values: 0.0188902523513, -0.186575038082, 0.0188902523503, 0.312886265924, 0.0188902523505, 0.722600616892, 0.0188902523497
        # 开始规划,18
        # gpmp计算结果 18, size:7, values: 1.03126219485e-06, -0.403846498744, 1.03126129132e-06, 0.324027798048, 1.03126146519e-06, 0.921112182162, 1.03126080801e-06
        # 开始规划,19
        # gpmp计算结果 19, size:7, values: -1.52487817365e-07, -0.00190684663475, -1.52488676759e-07, 0.000250522936598, -1.52488511217e-07, 0.00202024018867, -1.52489136043e-07


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