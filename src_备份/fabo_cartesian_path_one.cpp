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

<<<<<<< HEAD
// 组合两个位姿
geometry_msgs::Pose combinePoses(const geometry_msgs::Pose& pose_goal, const geometry_msgs::Pose& pose_reference)
{
    // 转换为tf2库中的数据类型
    tf2::Transform tf_pose_goal, tf_pose_reference, tf_combined;
    tf2::Quaternion tf_quat1, tf_quat2, tf_combined_quat;
    tf2::Vector3 tf_vec1, tf_vec2, tf_combined_vec;

    // 提取第一个位姿的旋转和平移部分
    tf_pose_goal.setRotation(tf2::Quaternion(pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w));
    tf_pose_goal.setOrigin(tf2::Vector3(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z));

    // 提取第二个位姿的旋转和平移部分
    tf_pose_reference.setRotation(tf2::Quaternion(pose_reference.orientation.x, pose_reference.orientation.y, pose_reference.orientation.z, pose_reference.orientation.w));
    tf_pose_reference.setOrigin(tf2::Vector3(pose_reference.position.x, pose_reference.position.y, pose_reference.position.z));

    // 组合两个位姿
    tf_combined = tf_pose_reference.inverse() * tf_pose_goal;

    // 提取组合后的旋转和平移部分
    tf_combined_quat = tf_combined.getRotation();
    tf_combined_vec = tf_combined.getOrigin();

    // 转换回geometry_msgs中的数据类型
    geometry_msgs::Pose combined_pose;
    combined_pose.orientation.x = tf_combined_quat.x();
    combined_pose.orientation.y = tf_combined_quat.y();
    combined_pose.orientation.z = tf_combined_quat.z();
    combined_pose.orientation.w = tf_combined_quat.w();
    combined_pose.position.x = tf_combined_vec.x();
    combined_pose.position.y = tf_combined_vec.y();
    combined_pose.position.z = tf_combined_vec.z();

    return combined_pose;
}


=======
>>>>>>> 08c1266182c2a3e9324b4c06027860246ab60b14
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
	
<<<<<<< HEAD
	// 获取当前位置
	tf::TransformListener listener;
    tf::StampedTransform transform;
	geometry_msgs::Pose start_pose;
    try
    {
		float mTfDuration = 2.0;
        listener.waitForTransform("/base_footprint", "/camera_rgb_frame", ros::Time(0), ros::Duration(mTfDuration));
        listener.lookupTransform("/base_footprint", "/camera_rgb_frame", ros::Time(0), transform);
		start_pose.orientation.x = transform.getRotation().x();
		start_pose.orientation.y = transform.getRotation().y();
		start_pose.orientation.z = transform.getRotation().z();
		start_pose.orientation.w = transform.getRotation().w();
		start_pose.position.x = transform.getOrigin().x();
		start_pose.position.y = transform.getOrigin().y();
		start_pose.position.z = transform.getOrigin().z();
		// std::cout<<"start pose:"<<start_pose<<std::endl;
		waypoints.push_back(start_pose);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s -->> lost tf from /base_footprint to /camera_rgb_frame",ex.what());
    }
	


	// 设置一个目标位置（路点）
	geometry_msgs::Pose target_pose1;
	
	target_pose1.orientation.x = 0.014;
	target_pose1.orientation.y = 0.104;
	target_pose1.orientation.z = 0.054;
	target_pose1.orientation.w = 0.993;

	target_pose1.position.x = 0.291; 
	target_pose1.position.y = -0.400;
	target_pose1.position.z = 0.702;

	waypoints.push_back(target_pose1);


	// 设置一个目标位置（路点）
	geometry_msgs::Pose target_pose2;
	
	target_pose2.orientation.x = -0.176;
	target_pose2.orientation.y = 0.417;
	target_pose2.orientation.z = -0.077;
	target_pose2.orientation.w = 0.888;

	target_pose2.position.x = 0.440;
	target_pose2.position.y = -0.151;
	target_pose2.position.z = 0.889;

	waypoints.push_back(target_pose2);
	
	
	// 设置一个目标位置（路点）
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
	ROS_INFO("Start planning");
	move_group.setPlannerId("RRTstar");
	// move_group.setMaxVelocityScalingFactor(0.1);

	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.001; //05;  //步长
	double fraction = 0.0;
	int maxtries = 10000; //最大尝试规划次数
	int attempts = 0; //已经尝试规划次数 arm.set_goal_position_tolerance(0.1)
	move_group.setGoalPositionTolerance(0.1);
	while(fraction < 1.0 && attempts < maxtries){
		fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
		attempts++;
		if(attempts % 10 == 0)
		ROS_INFO("Still trying after %d attempts...", attempts);
	}

	if(fraction > 0.2){
		ROS_INFO("Path computed successfully. Moving the arm.");
		// 生成机械臂的运动规划数据
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectory;
		// 执行运动
		move_group.execute(plan);
		sleep(1);
	}
	else{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
	}
=======

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
	
	
	


>>>>>>> 08c1266182c2a3e9324b4c06027860246ab60b14

	ros::shutdown(); 
	return 0;
}
