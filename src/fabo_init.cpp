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



	// // 笛卡尔空间下的路径规划与执行
	ROS_INFO("Start planning");
	move_group.setPlannerId("RRTstar");
	// move_group.setMaxVelocityScalingFactor(0.1);

	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.001; //05;  //步长
	double fraction = 0.0;
	int maxtries = 100; //最大尝试规划次数
	int attempts = 0; //已经尝试规划次数 arm.set_goal_position_tolerance(0.1)
	move_group.setGoalPositionTolerance(0.01);
	while(fraction < 1.0 && attempts < maxtries){
		fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
		attempts++;
		if(attempts % 10 == 0)
		ROS_INFO("Still trying after %d attempts...", attempts);
	}

	if(fraction > 0.8){
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

	ros::shutdown(); 
	return 0;
}
