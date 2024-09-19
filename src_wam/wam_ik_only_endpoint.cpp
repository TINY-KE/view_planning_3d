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
	ROS_INFO_NAMED("WAM_arm", "End effector link: %s", end_effector_link.c_str());

    //设置目标位置所使用的参考坐标系
	ROS_INFO_NAMED("WAM_arm", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    std::string pose_reference_frame="/wam/base_link";
    move_group.setPoseReferenceFrame(pose_reference_frame);
	ROS_INFO_NAMED("WAM_arm", "New Pose Reference frame: %s", move_group.getPoseReferenceFrame().c_str());


	std::vector<geometry_msgs::Pose> waypoints;	
	
	// 获取当前位置
	// 方法1：【有问题】
	// geometry_msgs::Pose target_pose0_1 = move_group.getCurrentPose(pose_reference_frame).pose;
	// geometry_msgs::Pose target_pose0_2 = move_group.getCurrentPose(end_effector_link).pose;
	// geometry_msgs::Pose target_pose0 = combinePoses(target_pose0_2, target_pose0_1);
	// std::cout<<"start pose footprint:"<<target_pose0_1<<std::endl;
	// std::cout<<"start pose end:"<<target_pose0_2<<std::endl;
	// std::cout<<"start pose 0:"<<target_pose0<<std::endl;
	// 方法2：利用tf
	tf::TransformListener listener;
    tf::StampedTransform transform;
	geometry_msgs::Pose start_pose;
    
    // try
    // {
	// 	float mTfDuration = 2.0;
    //     listener.waitForTransform("/base_footprint", "/camera_rgb_frame", ros::Time(0), ros::Duration(mTfDuration));
    //     listener.lookupTransform("/base_footprint", "/camera_rgb_frame", ros::Time(0), transform);
	// 	start_pose.orientation.x = transform.getRotation().x();
	// 	start_pose.orientation.y = transform.getRotation().y();
	// 	start_pose.orientation.z = transform.getRotation().z();
	// 	start_pose.orientation.w = transform.getRotation().w();
	// 	start_pose.position.x = transform.getOrigin().x();
	// 	start_pose.position.y = transform.getOrigin().y()-0.2;
	// 	start_pose.position.z = transform.getOrigin().z();
	// 	// std::cout<<"start pose:"<<start_pose<<std::endl;
		
    // }
    // catch (tf::TransformException &ex)
    // {
    //     ROS_ERROR("%s -->> lost tf from /base_footprint to /camera_rgb_frame",ex.what());
    // }
	


    // 设置END LINK的位姿
    geometry_msgs::Pose target_pose;
	
    target_pose.position.x = 0.59;
    target_pose.position.y = -0.065619;
    target_pose.position.z = 0.15273;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.707;  //0.70288
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.707; //0.71131

	move_group.setPoseTarget(target_pose);

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
        ROS_INFO("Failed to plan");
    }
    // move_group.move(); 

    // ROS_INFO( "Reach");


	ros::shutdown(); 
	return 0;
}
