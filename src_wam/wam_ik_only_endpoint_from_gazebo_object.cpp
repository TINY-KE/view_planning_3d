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
#include "ConverterTools.h"


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

    // gazebo中的模型和link名
    // std::string object_model_name = "vase_violet";  // get物体的信息
    std::string object_model_name = "bed_2";  // get物体的信息
    std::string chassis_model_name = "mrobot";   // get底盘的信息
    std::string camera_joint_name = "mrobot::wam/wrist_palm_link";   // get机械臂末端关节的信息
    double goal_dis = 3.;


    ros::Rate loop_rate(0.5);
    bool continuous_operation_mode = false;

    while(ros::ok()) {
        if(continuous_operation_mode){
            continue;
        }
        else{
            ROS_INFO(">>>>>> Input y or Y to continuous operate, any other key to operate in turn: ");
            char key = std::cin.get();
            if(key == 'y' || key == 'Y') 
                continuous_operation_mode = true;
        }
        
        ROS_INFO(">>>>>> Start Operating: ");

        // get物体的pose
        geometry_msgs::Pose object_pose = getPose(n, object_model_name, true);
        double object_x = object_pose.position.x;
        double object_y = object_pose.position.y;
        double object_z = object_pose.position.z;
        

        // get底盘的pose
        geometry_msgs::Pose chassis_pose = getPose(n, chassis_model_name, true);

                
    
        // 获取相机末端关机的位姿，用以替代相机的目标位姿
        auto joint_pose = get_link_pose(n, camera_joint_name, "world", true);
        double camera_x = joint_pose.position.x;
        double camera_y = joint_pose.position.y;
        double camera_z = joint_pose.position.z;

        
        // 计算相机的朝向
        double deltaX = object_x - camera_x;
        double deltaY = object_y - camera_y;
        double delta_yaw = std::atan2(deltaY, deltaX);
        double deltaZ = object_z - camera_z;
        double delta_pitch = std::atan2(deltaZ, std::sqrt(deltaX*deltaX + deltaY*deltaY));
    
        tf::Quaternion q = tf::createQuaternionFromRPY(0, -1*delta_pitch, delta_yaw);

        // 设置END LINK的位姿
        geometry_msgs::Pose target_pose;

        int type = 0;
        if(type==1){
            // setPose(n, camera_model_name, camera_x, camera_y, camera_z, q.w(), q.x(), q.y(), q.z());
            target_pose.position.x = 0.59;
            target_pose.position.y = -0.065619;
            target_pose.position.z = 0.15273;
            target_pose.orientation.x = 0.0;
            target_pose.orientation.y = 0.707;  //0.70288
            target_pose.orientation.z = 0.0;
            target_pose.orientation.w = 0.707; //0.71131
        }
        else if(type==0){
            double scale = goal_dis/std::sqrt(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);

            camera_x =  (camera_x-object_x)*scale + object_x; 
            camera_y =  (camera_y-object_y)*scale + object_y; 
            camera_z =  (camera_z-object_z)*scale + object_z; 
            // setPose(n, camera_model_name, camera_x, camera_y, camera_z, q.w(), q.x(), q.y(), q.z());
            target_pose.position.x = camera_x;
            target_pose.position.y = camera_y;
            target_pose.position.z = camera_z;
            
            target_pose.orientation.x = q.x();
            target_pose.orientation.y = q.y(); 
            target_pose.orientation.z = q.z();
            target_pose.orientation.w = q.w(); 
            
            target_pose = calculateRelativePose(chassis_pose,target_pose);

            std::cout<<"calculateRelativePose"
                <<", x:"<<target_pose.position.x
                <<", y:"<<target_pose.position.y
                <<", z:"<<target_pose.position.z
                <<std::endl;
        }
        else if(type==3){  //用于测试debug
            target_pose.position.x = object_x;
            target_pose.position.y = object_y;
            target_pose.position.z = object_z;
        }

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
