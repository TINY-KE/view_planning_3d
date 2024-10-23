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
#include "gtsam_quadrics/geometry/BboxCameraFactor.h"
#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace gtsam;
using namespace gpmp2;
using namespace gtsam_quadrics;

typedef BboxPlaneArmLinkFactor<ArmModel> BboxPlaneArmLinkArm;

int main(int argc, char** argv){
	//  一、创建ROS和movegroup
	ros::init(argc, argv, "gpmp_wam", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	ros::Rate loop_rate(2); // 设置发布频率
	// 启动movegroup
	// 创建一个异步的自旋线程（spinning thread）
	ros::AsyncSpinner spinner(1);
	spinner.start();
	moveit::planning_interface::MoveGroupInterface move_group("arm");
	std::string end_effector_link = move_group.getEndEffectorLink();
	ROS_INFO_NAMED("WAM_arm", "End effector link: %s", end_effector_link.c_str());

	ROS_INFO_NAMED("WAM_arm", "Planning frame: %s", move_group.getPlanningFrame().c_str());
	std::string pose_reference_frame = "/wam/base_link";
	// std::string pose_reference_frame="world";
	move_group.setPoseReferenceFrame(pose_reference_frame);
	ROS_INFO_NAMED("WAM_arm", "New Pose Reference frame: %s", move_group.getPoseReferenceFrame().c_str());

	// 二、生成物体、机械臂、
	//（1）生成物体
	double x = 3, y = 0, z = 0.5;
	double lenth = 2.5, width = 2, height = 1;
	double yaw = 0;
	MapObject *ob = new MapObject();
	ob->Update_Twobj(x, y, z, yaw);
	ob->Update_object_size(lenth, width, height);
	ob->Update_corner();
	//（2）生成机械臂
	ArmModel *arm_model = generateArm("WAMArm");
	//（3）生成相机参数
	int CameraWidth = 640;
	int CameraHeight = 480;
	float fx = 554.254691191187;
	float fy = 554.254691191187;
	float cx = 320.5;
	float cy = 240.5;
	Eigen::Matrix3d Calib;
	Calib << fx, 0, cx,
			0, fy, cy,
			0, 0, 1;


	// 三、可视化线程
	string default_frame = "wam/base_link";

	Visualize_Tools *vis_tools = new Visualize_Tools(nh, default_frame);
	std::thread *mptVisualizeTools;
	mptVisualizeTools = new std::thread(&Visualize_Tools::Run, vis_tools);
	vis_tools->addMapObject(ob);

	Visualize_Arm_Tools vis_arm_tools(nh, *arm_model, move_group, CameraWidth, CameraHeight, Calib, default_frame);
	std::thread *mptVisualizeArmTools;
	mptVisualizeArmTools = new std::thread(&Visualize_Arm_Tools::Run, vis_arm_tools);



	// 四、BboxCameraFactor.cpp 所需的参数
	double s = 100;
	gtsam_quadrics::AlignedBox2 gtsam_bbox(0+s, 0+s, CameraWidth-s, CameraHeight-s);   //预期的物体检测框
	double bbox_sigma = 0.001;   // bbox的权重
	Eigen::Matrix4f RobotPose = Eigen::Matrix4f::Identity();
	BboxCameraFactor factor(0,
                           bbox_sigma,
                           gtsam_bbox,
                           ob,
                           RobotPose,
                           CameraWidth, CameraHeight, Calib );

	// 五、
	ros::Rate r(0.2);

    while (ros::ok())
    {
		std::cout<<"新一轮的伺服控制："<<std::endl;

    	// 获取末端执行器的位姿
    	geometry_msgs::PoseStamped end_effector_pose = move_group.getCurrentPose();
    	std::cout<<"end_effector_pose: "<<std::endl<<end_effector_pose<<std::endl;

    	auto end_effector_pose_eigen = Converter::geometryPoseStampedtoMatrix4d(end_effector_pose);
    	Eigen::Matrix4d T_endlink_to_c;
    	std::cout<<"1"<<std::endl;
    	T_endlink_to_c << 0, 0, 1, 0.02,
						  -1, 0, 0, -0.013,
						  0, -1, 0, 0.13,  //实际为0.13(用于ros)，改为0.07(用于gpmp2 forwardKinematics)
						  0, 0, 0, 1;
    	Eigen::Matrix4d T_baselink_to_c = end_effector_pose_eigen * T_endlink_to_c;
    	std::cout<<"T_baselink_to_c: "<<std::endl<<T_baselink_to_c<<std::endl;

    	Eigen::MatrixXd H1_camerapose;
    	Eigen::Vector4d err_bbox, measure_bbox, predict_bbox;

    	// 计算error
    	err_bbox = factor.evaluateError(gtsam::Pose3(T_baselink_to_c), &H1_camerapose);
    	std::cout<<"2"<<std::endl;

    	// 可视化bbox
    	measure_bbox = factor.getMeasureBounds (gtsam::Pose3(T_baselink_to_c));
    	predict_bbox = factor.getPredictedBounds (gtsam::Pose3(T_baselink_to_c));
        double board = 200;
        cv::Mat image = cv::Mat::zeros(CameraHeight+board*2, CameraWidth+board*2, CV_8UC3);
        cv::Rect FOV_cvmat(board, board, CameraWidth, CameraHeight);
        cv::Rect measure_bbox_cvmat(measure_bbox[0]+board, measure_bbox[1]+board, measure_bbox[2]-measure_bbox[0], measure_bbox[3]-measure_bbox[1]);
        cv::Rect predict_bbox_cvmat(predict_bbox[0]+board, predict_bbox[1]+board, predict_bbox[2]-predict_bbox[0], predict_bbox[3]-predict_bbox[1]);
        cv::rectangle(image, FOV_cvmat, cv::Scalar(255, 255, 255), 2);
        cv::rectangle(image, measure_bbox_cvmat, cv::Scalar(255, 0, 0), 2);
        cv::rectangle(image, predict_bbox_cvmat, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Bounding Boxes", image);
        cv::waitKey(100);


    	// 视觉伺服
		double K = 0.1;
    	if(argc==2) {
    		K = std::stof(argv[1]);
    	}
    	geometry_msgs::Pose new_pose;
    	std::cout<<"err_bbox: "<<std::endl<<err_bbox.transpose()<<std::endl;
    	std::cout<<"H1_camerapose: "<<std::endl<<H1_camerapose<<std::endl;

    	Eigen::Matrix<double, 1, 6>  delta =  err_bbox.transpose() * H1_camerapose;
		std::cout<<"delta: "<<std::endl<<-1*delta.transpose()*K<<std::endl;

    	new_pose.position.x = end_effector_pose.pose.position.x - K*delta(0,3);
    	new_pose.position.y = end_effector_pose.pose.position.y - K*delta(0,4);
    	new_pose.position.z = end_effector_pose.pose.position.z - K*delta(0,5);

    	new_pose.orientation.w = 1;
    	new_pose.orientation.x = 0;
    	new_pose.orientation.y = 0;
    	new_pose.orientation.z = 0;
  //   	auto rpy_old = Converter::toRPY(end_effector_pose.pose.orientation.x, end_effector_pose.pose.orientation.y, end_effector_pose.pose.orientation.z, end_effector_pose.pose.orientation.w);
  //   	double r_new = rpy_old[0] - K*delta(0,0);
  //   	double p_new = rpy_old[1] - K*delta(0,1);
  //   	double y_new = rpy_old[2] - K*delta(0,2);
  //   	// new_pose.orientation. = end_effector_pose.pose.position.z + K*delta(0,2);
		// auto new_orientation = Converter::rpyToQuaternion(r_new, p_new, y_new);
  //   	new_pose.orientation.w = new_orientation.w();
  //   	new_pose.orientation.x = new_orientation.x();
  //   	new_pose.orientation.y = new_orientation.y();
  //   	new_pose.orientation.z = new_orientation.z();


    	// plan 和 move
    	move_group.setPoseTarget(new_pose);
    	std::cout<<"new_pose: "<<std::endl<<new_pose<<std::endl;

    	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    	if(success){

    		std::vector<double> joint_angles = my_plan.trajectory_.joint_trajectory.points.back().positions;
    		for (size_t i = 0; i < joint_angles.size(); ++i) {
    			ROS_INFO("Joint %lu: %f", i, joint_angles[i]);
    		}
    		std::cout<<"<<<<<<<<<<<<<< 规划成功 \n"<<std::endl;
    		move_group.execute(my_plan);
    		// loop_rate.sleep();
    		std::cout << "Press [ENTER] to continue ... , [y] to autonomous mode" << std::endl;
    		std::cout << "*****************************" << std::endl;
    		char key = getchar();
    	}
    	else{
    		// Candidates[i].position.y += 0.3;
    		// Candidates[i].position.z -= 0.1;
    		// move_group.setPoseTarget(Candidates[i]);
    		// success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    		// if(success){
    		//     std::cout<<"<<<<<<<<<<<<<< 重新规划成功 \n"<<std::endl;
    		//     move_group.execute(my_plan);
    		// }
    		// else
    		std::cout<<"<<<<<<<<<<<<<< 规划失败\n"<<std::endl;
    	}

        r.sleep();
    }

    

    return 0;
}