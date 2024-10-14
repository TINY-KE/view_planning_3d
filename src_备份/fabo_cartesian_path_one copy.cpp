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
	

	// 设置1目标位置（路点）
	geometry_msgs::Pose target_pose1;
	
	target_pose1.orientation.x = 0.014;
	target_pose1.orientation.y = 0.104;
	target_pose1.orientation.z = 0.054;
	target_pose1.orientation.w = 0.993;

	target_pose1.position.x = 0.291; 
	target_pose1.position.y = -0.400;
	target_pose1.position.z = 0.702;

	// waypoints.push_back(target_pose1);



	
	
	// 设置2目标位置（路点）
	geometry_msgs::Pose target_pose2;
	
	target_pose2.orientation.x = -0.176;
	target_pose2.orientation.y = 0.417;
	target_pose2.orientation.z = -0.077;
	target_pose2.orientation.w = 0.888;

	target_pose2.position.x = 0.440;
	target_pose2.position.y = -0.151;
	target_pose2.position.z = 0.889;

	// waypoints.push_back(target_pose2);
	
	
	// 设置3目标位置（路点）
	geometry_msgs::Pose target_pose3;
	
	target_pose3.orientation.x = 0.014;
	target_pose3.orientation.y = 0.510;
	target_pose3.orientation.z = -0.024;
	target_pose3.orientation.w = 0.860;

	target_pose3.position.x = 0.468;
	target_pose3.position.y = -0.326;
	target_pose3.position.z = 0.950;

	waypoints.push_back(target_pose3);
	
	
	// 设置一个目标位置（路点）
	geometry_msgs::Pose target_pose4;
	
	target_pose4.orientation.x = 0.553;
	target_pose4.orientation.y = 0.375;
	target_pose4.orientation.z = 0.235;
	target_pose4.orientation.w = 0.706;

	target_pose4.position.x = 0.405;
	target_pose4.position.y = -0.747;
	target_pose4.position.z = 0.69;

	waypoints.push_back(target_pose4);



	// // 笛卡尔空间下的路径规划与执行
	int i = 0;
	for(auto waypoint : waypoints){
		i ++;
		move_group.setPoseTarget(waypoint);

		// 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);

		ROS_INFO("Visualizing plan %d (pose goal) %s", i, success?"":"FAILED");   

		//让机械臂按照规划的轨迹开始运动。
		if(success)
			move_group.execute(my_plan);
		
		sleep(3);
	}
	
	



	ros::shutdown(); 
	return 0;
}
