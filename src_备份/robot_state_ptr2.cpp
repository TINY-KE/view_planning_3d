// 源程序的网页  https://blog.csdn.net/qq_26565435/article/details/90449047

//#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "RobotModelAndRobotState");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // 首先实例化一个RobotModelLoader对象，该对象将在ROS参数服务器上查找机器人描述并构造一个RobotModel供我们使用
//1.RobotModelPtr，robot_model指针
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model_RobotModelPtr = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model_RobotModelPtr->getModelFrame().c_str());

//2.RobotStatePtr,robot_state指针
    robot_state::RobotStatePtr kinematic_state_RobotStatePtr(new robot_state::RobotState(kinematic_model_RobotModelPtr));
    kinematic_state_RobotStatePtr->setToDefaultValues();  //将机器人的状态都设为默认值，根据setToDefaultValues()定义，默认位置为0或者是最大界限和最小界限之间。
    // 定义一个robot_state::JointModelGroup类型的指针joint_model_group，包含规划组arm的信息。
    const robot_state::JointModelGroup* joint_model_group = kinematic_model_RobotModelPtr->getJointModelGroup("arm");


// 3.获取每个关节的名称和角度值
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();  //按照它们存储在内存中的顺序获取构成此状态的变量的名称，并将其存放在可变数组joint_name中。
    // 以上进行的一切操作均没有和move_group进行通讯，因此机器人不会动。  TODO: setToDefaultValues()也没有动吗？ 
    std::vector<double> joint_values;
    // 根据copyJointGroupPositions()，按照在规划组中变量的顺序，将“**当前状态**”下规划组中的各个关节的位置值复制到另一个地方。
    // 注意，这不一定是RobotState本身的连续内存块，因此我们复制而不是返回指针。
    kinematic_state_RobotStatePtr->copyJointGroupPositions(joint_model_group, joint_values);
    // 然后输出每个关节的名称及其当前状态下的位置值。
    for (int i = 0; i < joint_names.size(); i++){
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

// 4.关节限制
    // joint_values[0] = 5.57;
    // joint_values[1] = 1.07;
    // setJointGroupPositions()，更新规划组中各关节的位置，但这个函数不会强制执行关节限制。  TODO: 是不是指：不会执行
    kinematic_state_RobotStatePtr->setJointGroupPositions(joint_model_group, joint_values);
    ROS_INFO_STREAM("Current state is " << (kinematic_state_RobotStatePtr->satisfiesBounds() ? "valid" : "not valid"));
    kinematic_state_RobotStatePtr->printStatePositions();
    
    // enforceBounds()，调用该函数后就能对关节进行限制。具体可见终端输出结果。
    kinematic_state_RobotStatePtr->enforceBounds();
    ROS_INFO_STREAM("Current state is " << (kinematic_state_RobotStatePtr->satisfiesBounds() ? "valid" : "not valid"));
    kinematic_state_RobotStatePtr->printStatePositions();
    ROS_INFO_STREAM("Over bound is enforced to be valid .");
    const Eigen::Isometry3d& end_effector_state_init = kinematic_state_RobotStatePtr->getGlobalLinkTransform("Link7");

    ROS_INFO_STREAM("Init Translation: \n" << end_effector_state_init.translation() << "\n");
    ROS_INFO_STREAM("Init Rotation: \n" << end_effector_state_init.rotation() << "\n");

// 5.正向运动学
    kinematic_state_RobotStatePtr->setToRandomPositions(joint_model_group);   //将目前规划组中的各个关节值设置为随机值。
    kinematic_state_RobotStatePtr->printStatePositions();
    
    // const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("gripper_frame");
    // 然后计算末端执行器（当前规划组中最末端，本文为gripper_frame）在随机关节值状态下的正向运动学齐次变换矩阵（平移+旋转），并输出。
    const Eigen::Isometry3d& end_effector_state = kinematic_state_RobotStatePtr->getGlobalLinkTransform("Link7");

    ROS_INFO_STREAM("Random Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Random Rotation: \n" << end_effector_state.rotation() << "\n");

//6. 逆运动学
    // 先为可视化做准备。
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    // 调用setFromIK解当前规划组arm的逆运动学问题，返回一个bool量。在解决该问题之前，需要如下条件：
    // end_effector_state: 末端执行器的期望位姿（一般情况下为当前规划组chain的最后一个连杆，本文为gripper_link）：也就是上面已经计算得到的齐次变换矩阵end_effector_state；
    // 10：  尝试解决IK的次数
    // 0.1s：   每次尝试的时间
    Eigen::Isometry3d end_effector_state_my = Eigen::Isometry3d::Identity();
    
    // version 1:
    // Eigen::Vector3d translation(0.0, 0.0, 0.75);
    // end_effector_state_my.pretranslate(end_effector_state.translation());
    // end_effector_state_my.rotate( end_effector_state.rotation());


    // version2:
    // x 0.107883,y -0.0381831,z 0.799186,qx -0,qy 0.427923,qz 0.019639,qw 0.903602
    Eigen::Vector3d translation(0.107883, -0.0381831, 0.799186);
    Eigen::Quaterniond quaternion(-0, 0.427923, 0.019639, 0.903602);
    end_effector_state_my.pretranslate(translation);
    end_effector_state_my.rotate( quaternion.toRotationMatrix());

    std::size_t attempts = 10;
    double timeout = 0.1;
    // kinematic_state_RobotStatePtr->setT

    bool found_ik = kinematic_state_RobotStatePtr->setFromIK(joint_model_group, end_effector_state_my /* 原本是 end_effector_state_my */, attempts, timeout);   //bug
    // 如果IK得到解，则驱动机器人按照计算得到的关节值进行运动，同时，打印计算结果。
    if (found_ik){
        ROS_INFO("Find IK solution");

        // TODO:  也就是说 这里存储着逆运动学求解的结果
        kinematic_state_RobotStatePtr->copyJointGroupPositions(joint_model_group, joint_values);
        move_group.setJointValueTarget(joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "Visualizing plan %s", success ? "" : "FAILED");
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        visual_tools.trigger();

        for (int i = 0; i < joint_names.size(); i++){
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }else{
        ROS_INFO("Did not find IK solution");
    }

// // 7.计算末端执行器的雅克比矩阵
//     // getJacobian()，
//     // 先给定一个参考点reference_point_position坐标位置，
//     // 该坐标位置是相对joint_model_group->getLinkModelNames().back()，也就是Link7而言的，
//     Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
//     // 定义一个空的jacobian矩阵。
//     Eigen::MatrixXd jacobian;
//     // 调用函数，计算并输出结果。
//     kinematic_state_RobotStatePtr->getJacobian(joint_model_group,kinematic_state_RobotStatePtr->getLinkModel(joint_model_group->getLinkModelNames().back()),reference_point_position, jacobian);
//     ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
//     ROS_INFO_STREAM("END: \n");

//     ros::shutdown();


    return 0;
}
