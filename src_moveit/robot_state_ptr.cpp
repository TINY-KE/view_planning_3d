//需要的头文件
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "get_random_pose");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    move_group.setRandomTarget();


    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (random pose) %s", success ? "" : "FAILED");

    #include <Eigen/Geometry>

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    // Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    
    text_pose.translation().z() = 0.65;

    //随机的坐标无法展示，终端显示找不到“gripper_frmae”的位置
    // geometry_msgs::PoseStamped target_pose;
    // target_pose = move_group.getPoseTarget("gripper_frame");
    
    //展示坐标和轨迹线，随机的坐标怎么展示？？？
    // visual_tools.publishAxisLabeled(target_pose.pose, "random_pose");
    visual_tools.publishText(text_pose, "GetRandomPose", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

   move_group.move();//待定

    ros::waitForShutdown();

    return 0;
}
