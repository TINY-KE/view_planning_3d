#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>

#include <gazebo_rviz_tools.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cartesian_path_demo", ros::init_options::AnonymousName);

    
    // GAZEBO MODEL POSE
    ros::NodeHandle n;


	// 创建一个异步的自旋线程（spinning thread）
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroupInterface move_group("arm");

	const robot_state::JointModelGroup* joint_model_group =
		    move_group.getCurrentState()->getJointModelGroup("arm");

	//获取终端link的名称
    std::string end_effector_link=move_group.getEndEffectorLink();
	ROS_INFO_NAMED("WAM_arm", "End effector link: %s", end_effector_link.c_str());

    //设置目标位置所使用的参考坐标系
	ROS_INFO_NAMED("WAM_arm", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    std::string pose_reference_frame="/wam/base_link";
    // std::string pose_reference_frame="world";
    move_group.setPoseReferenceFrame(pose_reference_frame);
	ROS_INFO_NAMED("WAM_arm", "New Pose Reference frame: %s", move_group.getPoseReferenceFrame().c_str());


    ros::Rate loop_rate(0.5);
    bool continuous_operation_mode = false;

    // while(ros::ok()) 
    {
        

        // joint_positions = [-0.799999592529, -1.69999958617, 1.64000036225, 1.2900001101, 1.09999985802, -0.105999705866, 2.19999970525]
        // arm.set_joint_value_target(joint_positions)
        // arm.go()
        // rospy.loginfo("0 success")

        std::vector<double> joint_positions = {-0.799999592529, -1.69999958617, 1.64000036225, 1.2900001101, 1.09999985802, -0.105999705866, 2.19999970525};
        move_group.setJointValueTarget(joint_positions);

        // 解算（plan）运动学逆解
        moveit::planning_interface::MoveGroupInterface::Plan candidate_plan;

        // 路径规划
        bool success = (move_group.plan(candidate_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        // 执行
        if(success)
        {
            move_group.execute(candidate_plan);
        }
        else
        {
            ROS_INFO("!!!!!  Failed to plan");
        }
        // move_group.move(); 

        // ros::spinOnce();
        // loop_rate.sleep(); // is stuck on loop rate, reboot roscore
        ROS_INFO("<<<<<<  Plan End \n ");
        
    }
    ROS_INFO("end service");

	return 0;
}
