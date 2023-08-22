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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cartesian_path_demo", ros::init_options::AnonymousName);

	// 创建一个异步的自旋线程（spinning thread）
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroupInterface move_group("arm");

	const robot_state::JointModelGroup* joint_model_group =
		    move_group.getCurrentState()->getJointModelGroup("arm");

	//获取终端link的名称
    std::string end_effector_link=move_group.getEndEffectorLink();
	ROS_INFO_NAMED("fabo_arm", "End effector link: %s", end_effector_link.c_str());

    //设置目标位置所使用的参考坐标系
	ROS_INFO_NAMED("fabo_arm", "Planning frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("fabo_arm", "Pose Reference frame: %s", move_group.getPoseReferenceFrame().c_str());
    std::string pose_reference_frame="base_footprint";
    move_group.setPoseReferenceFrame(pose_reference_frame);
	ROS_INFO_NAMED("fabo_arm", "New Pose Reference frame: %s", move_group.getPoseReferenceFrame().c_str());


	std::vector<geometry_msgs::Pose> waypoints;	
	

	std::cout<<"-1"<<std::endl;

	
	
	
	
	// 设置3目标位置（路点）
	geometry_msgs::Pose target_pose3;
	
	// target_pose3.orientation.x = 0.014;
	// target_pose3.orientation.y = 0.510;
	// target_pose3.orientation.z = -0.024;
	// target_pose3.orientation.w = 0.860;
	// target_pose3.orientation.x = -0.375052;
	// target_pose3.orientation.y = 0.57899;
	// target_pose3.orientation.z = 0.413084;
	// target_pose3.orientation.w = 0.594532;

	// target_pose3.position.x = 0.468;
	// target_pose3.position.y = -0.326;
	// target_pose3.position.z = 0.950;
	target_pose3.position.x = 0.39128;
	target_pose3.position.y = -0.000874643;
	target_pose3.position.z = 0.55791;

	// waypoints.push_back(target_pose3);
	




	std::cout<<"0"<<std::endl;

	move_group.setPoseTarget(target_pose3);

	// 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	std::cout<<"1"<<std::endl;
	moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
	std::cout<<"2"<<std::endl;

	ROS_INFO("Visualizing plan  (pose goal) %s", success?"":"FAILED");   
	std::cout<<"3"<<std::endl;

	//让机械臂按照规划的轨迹开始运动。
	if(success){
		std::cout<<"4"<<std::endl;
		move_group.execute(my_plan);  //

		std::cout<<"5"<<std::endl;
	}
	
	sleep(3);
	
	
	



	ros::shutdown(); 
	return 0;
}
