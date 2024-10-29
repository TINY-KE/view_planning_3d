#include <iostream>
#include <cmath>
#include <vector>
#include "MapObject.h"
#include "core/Plane.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "gazebo_rviz_tools.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "Converter.h"
#include <thread>

// #include "gtsam/JointLimitFactorVector.h"
#include "gtsam/BboxPlaneArmLinkFactor.h"
#include "GenerateArm.h"

#include "visualize_arm_tools.h"

#include "gtsam_quadrics/geometry/BoundingBoxFactor.h"
#include "gtsam/BboxEllipsoidFactor.h"

// #include "gtsam_quadrics/geometry/BboxCameraFactor.h"

using namespace Eigen;
using namespace gtsam;
using namespace gpmp2;

typedef BboxPlaneArmLinkFactor<ArmModel> BboxPlaneArmLinkArm;

int main(int argc, char** argv){
    ros::init(argc, argv, "triangle_marker_node");
    ros::NodeHandle nh;
    Visualize_Tools* vis_tools = new Visualize_Tools(nh, "wam/base_link");
    std::thread* mptVisualizeTools;
    mptVisualizeTools = new std::thread(&Visualize_Tools::Run, vis_tools);

	// 一、 构建物体和相机

	ArmModel* arm_model = generateArm("WAMArm");
	int CameraWidth = 640;
	int CameraHeight = 480;
	float fx = 554.254691191187;
	float fy = 554.254691191187;
	float cx = 320.5;
	float cy = 240.5;
	Eigen::Matrix3d Calib;
	Calib << fx,  0,  cx,
			  0,  fy,  cy,
			  0,   0,   1;
	// 生成物体
	double x=3, y=0, z=0.5;
	double lenth=2.5, width=2, height=1;
	double yaw = 0;
	MapObject* ob = new MapObject();
	ob->Update_Twobj(x,y,z,yaw);
	ob->Update_object_size(lenth,width,height);
	ob->Update_corner();
	vis_tools->MapObjects.push_back(ob);


	// 二、启动movegroup
	// 创建一个异步的自旋线程（spinning thread）
	ros::AsyncSpinner spinner(1);
	spinner.start();
	moveit::planning_interface::MoveGroupInterface move_group("arm");
	Visualize_Arm_Tools vis_arm_tools(nh, *arm_model, move_group, CameraWidth, CameraHeight, Calib, "wam/base_link" );
	std::thread* mptVisualizeArmTools;
	mptVisualizeArmTools = new std::thread(&Visualize_Arm_Tools::Run, vis_arm_tools);

	std::string end_effector_link=move_group.getEndEffectorLink();
	ROS_INFO_NAMED("WAM_arm", "End effector link: %s", end_effector_link.c_str());

	ROS_INFO_NAMED("WAM_arm", "Planning frame: %s", move_group.getPlanningFrame().c_str());
	std::string pose_reference_frame="/wam/base_link";
	// std::string pose_reference_frame="world";
	move_group.setPoseReferenceFrame(pose_reference_frame);
	ROS_INFO_NAMED("WAM_arm", "New Pose Reference frame: %s", move_group.getPoseReferenceFrame().c_str());

	// geometry_msgs::PoseStamped end_pose = move_group.getCurrentPose();
	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("arm");

	// 创建 TF2 监听器
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);






	// 三、构建因子
	// (1) 构建视场和机器人本体的因子
	double epsilon_dist = 0.05;  // 避障球与平面的最小距离
	BboxPlaneArmLinkFactor<ArmModel> factor(0, *arm_model,
                               1.0,
                               epsilon_dist,
                               CameraWidth, CameraHeight,
                               Calib
                               );


	// 四、 计算error
	Eigen::MatrixXd H1_act;
	Eigen::Matrix<double, 7, 1> config;
	// origin zero case
	config << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;

	Eigen::VectorXd err_act = factor.evaluateError(config, &H1_act);


    // 五、
    ros::Rate r(20);
    double scale = 0.1;

    while (ros::ok())
    {
    	// 获取当前关节角度
  		std::vector<double> joint_angles = move_group.getCurrentJointValues();
  		config << joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4], joint_angles[5], joint_angles[6];

    	Eigen::MatrixXd H1_planearm;
    	Eigen::Vector4d err_planearm;

    	H1_planearm = factor.evaluateError(config, &H1_planearm);
    	std::cout<<" [debug] err_act: "<<err_act.transpose()<<std::endl;
		std::cout<<" [debug] H1_planearm: "<<H1_planearm.transpose()<<std::endl;
    	std:;cout<<"end"<<std::endl;

        // 设置目标关节量
        std::vector<double > target_joint_group_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        target_joint_group_positions[0] = joint_angles[0] - H1_planearm(1,0)*err_planearm(1)/scale;
        target_joint_group_positions[1] = joint_angles[1] - H1_planearm(1,1)*err_planearm(1)/scale;
        target_joint_group_positions[2] = joint_angles[2] - H1_planearm(1,2)*err_planearm(1)/scale;
        target_joint_group_positions[3] = joint_angles[3] - H1_planearm(1,3)*err_planearm(1)/scale;
        target_joint_group_positions[4] = joint_angles[4] - H1_planearm(1,4)*err_planearm(1)/scale;
        target_joint_group_positions[5] = joint_angles[5] - H1_planearm(1,5)*err_planearm(1)/scale;
        target_joint_group_positions[6] = joint_angles[6] - H1_planearm(1,6)*err_planearm(1)/scale;

        move_group.setJointValueTarget(target_joint_group_positions);

        // plan 和 move
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success){
            std::cout<<"规划成功"<<std::endl;
            move_group.execute(my_plan);
            std::cout << "按任意键继续..." << std::endl;

			// 等待用户按下任意键（实际上是等待按下回车键）
        	std::cout << "Press [ENTER] to continue ... , [y] to autonomous mode" << std::endl;
        	std::cout << "*****************************" << std::endl;
        	char key = getchar();

        }
        else{
            std::cout<<"规划失败"<<std::endl;
            scale = scale/3;
        }

        // std::cout << "Press [ENTER] to continue ... , [y] to autonomous mode" << std::endl;
        // char key = getchar();

        r.sleep();
    }

    

    return 0;
}