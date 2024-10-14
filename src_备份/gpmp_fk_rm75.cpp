// #!/usr/bin/env python
// # -*- coding: utf-8 -*-

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <gpmp2/kinematics/GaussianPriorWorkspaceOrientationArm.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/kinematics/Arm.h>
#include <gpmp2/kinematics/GaussianPriorWorkspacePoseArm.h>
// #include <gpmp2/kinematics/RobotModel.h>



#include <iostream>
#include <cmath>

// zhjd： arm轨迹优化专用
#include <gpmp2/planner/TrajUtils.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h>

#include <gtsam/inference/Symbol.h>


#include <gtsam/slam/PriorFactor.h>
#include <gpmp2/gp/GaussianProcessPriorPose2Vector.h>
#include <gpmp2/geometry/numericalDerivativeDynamic.h>

#include <cmath>
#include <algorithm>

// sdf
#include <gpmp2/obstacle/ObstacleSDFFactor.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGP.h>

// 优化器
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <gpmp2/kinematics/GaussianPriorWorkspacePoseArm.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>

// 查看arm轨迹是否避障
#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/planner/BatchTrajOptimizer.h>


// 关节之间的连续性？ GP prior
#include <gpmp2/gp/GaussianProcessPriorLinear.h>

// ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include "sdf.h"


// moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>



using namespace std;
using namespace gtsam;
using namespace gpmp2;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_random_demo", ros::init_options::AnonymousName);
    // 创建一个异步的自旋线程（spinning thread）
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // 连接move_group节点中的机械臂实例组，这里的组名arm是我们之前在setup assistant中设置的
    moveit::planning_interface::MoveGroupInterface move_group("arm");


    // 随机产生一个目标位置
    // move_group.setRandomTarget();
    // move_group.setMaxVelocityScalingFactor(0.05);
    // move_group.setMaxAccelerationScalingFactor(0.05); 
    // arm.set_goal_joint_tolerance(0.001)
    // # arm.set_planner_id("RRTConnectkConfigDefault")
    // arm.set_planner_id("RRTstar")
    move_group.setGoalJointTolerance(0.001);
    move_group.setPlannerId("RRTstar");

    // 开始运动规划，并且让机械臂移动到目标位置
    // move_group.move();
    

        
        
//         # x0: [-3.0847299992, -1.76330430583, 1.85520868669, 0.433015552185, -2.67246194478, 0.469249715246, 4.0008642662]
//         joint_positions = [-3.0847299992, -1.76330430583, 1.85520868669, 0.433015552185, -2.67246194478, 0.469249715246, 4.0008642662]
//         arm.set_joint_value_target(joint_positions)
//         arm.go()
//         rospy.loginfo("0 success")


//         # x1: [-3.04402376537, -1.69884396073, 1.8702978783, 0.382855931446, -2.67168260047, 0.354481125083, 3.97018391619]
//         joint_positions = [-3.04402376537, -1.69884396073, 1.8702978783, 0.382855931446, -2.67168260047, 0.354481125083, 3.97018391619]
//         arm.set_joint_value_target(joint_positions)
//         arm.go()
//         rospy.loginfo("1 success")


//         # x2: [-2.95116569275, -1.52304376833, 1.90034534245, 0.257925461332, -2.67164417053, 0.0598456237564, 3.90139668332]
//         joint_positions = [-2.95116569275, -1.52304376833, 1.90034534245, 0.257925461332, -2.67164417053, 0.0598456237564, 3.90139668332]
//         arm.set_joint_value_target(joint_positions)
//         arm.go()
//         rospy.loginfo("2 success")

//         # x3: [-2.84016953283, -1.27365861661, 1.92510001382, 0.0904561656591, -2.67786271753, -0.341063948932, 3.83080612055]
//         joint_positions = [-2.84016953283, -1.27365861661, 1.92510001382, 0.0904561656591, -2.67786271753, -0.341063948932, 3.83080612055]
//         arm.set_joint_value_target(joint_positions)
//         arm.go()
//         rospy.loginfo("3 success")
        

//         # x4: [-2.72134526088, -1.01280662375, 1.92956702004, -0.101649336678, -2.69809448942, -0.775884855127, 3.79328166499]
//         joint_positions = [-2.72134526088, -1.01280662375, 1.92956702004, -0.101649336678, -2.69809448942, -0.775884855127, 3.79328166499]
//         arm.set_joint_value_target(joint_positions)
//         arm.go()
//         rospy.loginfo("4 success")


//         # x5: [-2.58567851684, -0.825155147096, 1.90303806418, -0.310497240648, -2.73869858, -1.17574498534, 3.8167529028]
//         joint_positions = [-2.58567851684, -0.825155147096, 1.90303806418, -0.310497240648, -2.73869858, -1.17574498534, 3.8167529028]
//         arm.set_joint_value_target(joint_positions)
//         arm.go()
//         rospy.loginfo("5 success")


//         # x6: [-2.43355648621, -0.792987639679, 1.84241239677, -0.519925109608, -2.80024776011, -1.48505123901, 3.91971597317]
//         joint_positions = [-2.43355648621, -0.792987639679, 1.84241239677, -0.519925109608, -2.80024776011, -1.48505123901, 3.91971597317]
//         arm.set_joint_value_target(joint_positions)
//         arm.go()
//         rospy.loginfo("6 success")


//         # x7: [-2.3260014257, -0.919852532252, 1.7643924379, -0.685561568672, -2.87463717754, -1.67098533434, 4.10573271335]    
//         joint_positions = [-2.3260014257, -0.919852532252, 1.7643924379, -0.685561568672, -2.87463717754, -1.67098533434, 4.10573271335]
//         arm.set_joint_value_target(joint_positions)
//         arm.go()
//         rospy.loginfo("7 success")


//         # x8: [-2.33211407107, -1.16731986662, 1.6766113255, -0.777686948915, -2.9656928462, -1.69687193109, 4.35956934715]
//         joint_positions = [-2.33211407107, -1.16731986662, 1.6766113255, -0.777686948915, -2.9656928462, -1.69687193109, 4.35956934715]
//         arm.set_joint_value_target(joint_positions)
//         arm.go()
//         rospy.loginfo("8 success")


//         # x9: [-2.31896464051, -1.45548138621, 1.58346161741, -0.963275514931, -3.07103813556, -1.39172520186, 4.60656367297]
//         joint_positions = [-2.31896464051, -1.45548138621, 1.58346161741, -0.963275514931, -3.07103813556, -1.39172520186, 4.60656367297]
//         arm.set_joint_value_target(joint_positions)
//         arm.go()
//         rospy.loginfo("9 success")


//         # x10: [-2.0276392339, -1.5961786118, 1.54495801145, -1.37076960904, -3.12070893865, -0.859890926582, 4.70640728912]
//         # joint_positions = [-2.0276392339, -1.5961786118, 1.54495801145, -1.37076960904, -3.12070893865, -0.859890926582, 4.70640728912]
//         joint_positions = [-2.018431036907354, -1.4999897089911451, 1.4046777996889483, -1.3707039693409548, -3.0999924865261397, -0.8560425214202461, 4.8622166345079165]
//         arm.set_joint_value_target(joint_positions)
//         arm.go()
//         rospy.loginfo("10 success")
//         # 新x10: [-2.02750724615, -1.59799481955, 1.54757207374, -1.37065803169, -3.12111444241, -0.860010740512, 4.70351089598]



        


//         # 关闭并退出moveit
//         moveit_commander.roscpp_shutdown()
//         moveit_commander.os._exit(0)



//     # 设置场景物体的颜色
//     def setColor(self, name, r, g, b, a = 0.9):
//         # 初始化moveit颜色对象
//         color = ObjectColor()
        
//         # 设置颜色值
//         color.id = name
//         color.color.r = r
//         color.color.g = g
//         color.color.b = b
//         color.color.a = a
        
//         # 更新颜色字典
//         self.colors[name] = color

//     # 将颜色设置发送并应用到moveit场景当中
//     def sendColors(self):
//         # 初始化规划场景对象
//         p = PlanningScene()

//         # 需要设置规划场景是否有差异     
//         p.is_diff = True
        
//         # 从颜色字典中取出颜色设置
//         for color in self.colors.values():
//             p.object_colors.append(color)
        
//         # 发布场景物体颜色设置
//         self.scene_pub.publish(p)

// if __name__ == "__main__":
//     try:
//         MoveItFkDemo()
//     except rospy.ROSInterruptException:
//         pass




        ros::waitForShutdown();
}
