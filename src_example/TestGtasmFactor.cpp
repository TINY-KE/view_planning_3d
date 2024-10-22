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

    // 一、生成物体
    double x=3, y=0, z=0.5;
    double lenth=2.5, width=2, height=1;
    double yaw = 0;
    MapObject* ob = new MapObject();
    ob->Update_Twobj(x,y,z,yaw);
    ob->Update_object_size(lenth,width,height);
    ob->Update_corner();
    // int pub_num = 6;
    // while(pub_num--){
    //     vis_tools.visualize_ellipsoid(3, 0, 0.5, 
    //             2.5, 2, 1, 
    //             0, 0, 0, "world", 0);
    // }
    vis_tools->MapObjects.push_back(ob);


    // 二、构建因子
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
//    ObstacleSDFFactor(gtsam::Key poseKey, const Robot& robot,
//                    const SignedDistanceField& sdf, double cost_sigma,
//                    double epsilon)
    //    ObstacleSDFFactorArm factor(0, arm,    //gtsam::Key poseKey, const Robot& robot,
                        //    sdf,       //const SignedDistanceField& sdf,
                        //    1.0,      //double cost_sigma,
                        //    obs_eps);    //double epsilonEigen::VectorXd
//    BboxPlaneArmLink(gtsam::Key poseKey, const Robot& robot,
//                    double cost_sigma,
//                    double epsilon,
//                    int width, int height, Matrix3d calib)

	// (1) 构建视场和机器人本体的因子
	double epsilon_dist = 0.05;  // 避障球与平面的最小距离
	BboxPlaneArmLinkFactor<ArmModel> factor(0, *arm_model,
                               1.0,
                               epsilon_dist,
                               CameraWidth, CameraHeight,
                               Calib
                               );
	//(2) 构建视场和lzw椭球体的因子
	Eigen::Matrix4f RobotPose = Eigen::Matrix4f::Identity();



	double s = 100;
	gtsam_quadrics::AlignedBox2 gtsam_bbox(0+s, 0+s, CameraWidth-s, CameraHeight-s);

	// （3）构建视场和gtsam椭球体的官方因子
	// gtsam::Cal3_S2 gtsam_calib(fx, fy, 0.0, cx, cy);
	boost::shared_ptr<gtsam::Cal3_S2> gtsam_calib = boost::make_shared<gtsam::Cal3_S2>(fx, fy, 0.0, cx, cy);
	// 使用数组中的值创建噪声模型
	double sigmasArray[4] = {10.0, 10.0, 10.0, 10.0};
	gtsam::SharedNoiseModel bbox_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4(sigmasArray));
	gtsam::SharedNoiseModel bbox_noise_2 = gtsam::noiseModel::Isotropic::Sigma(4,10);
	std::cout<<"bbox_noise:"<<std::endl;
	bbox_noise->print();
	std::cout<<"bbox_noise_2:"<<std::endl;
	bbox_noise_2->print();
    // BoundingBoxFactor(const AlignedBox2& measured,
    //                 const boost::shared_ptr<gtsam::Cal3_S2>& calibration,
    //                 const gtsam::Key& poseKey, const gtsam::Key& quadricKey,
    //                 const gtsam::SharedNoiseModel& model,
    //                 const MeasurementModel& errorType = STANDARD)
	gtsam::Key Key1, Key2;
	gtsam_quadrics::BoundingBoxFactor factor3(gtsam_bbox, gtsam_calib, Key1, Key2, bbox_noise);
	Eigen::Matrix4d matrix_4x4;
	// matrix_4x4 << 1, 0, 0, x,
	// 			  0, 1, 0, y,
	// 			  0, 0, 1, z,
	// 			  0, 0, 0, 1;
	matrix_4x4 << 1, 0, 0, x,
				  0, 1, 0, y,
				  0, 0, 1, z,
				  0, 0, 0, 1;
	gtsam_quadrics::ConstrainedDualQuadric Quadric(gtsam::Pose3(matrix_4x4), gtsam::Vector3(lenth/2.0,width/2.0,height/2.0));

	// （4）构建视场和gtsam椭球体的我的因子
	BboxEllipsoidFactor<ArmModel> factor4(0, *arm_model,
						1.0,
						gtsam_bbox,
						ob,
						RobotPose,
						CameraWidth, CameraHeight,
						Calib);

	Eigen::MatrixXd H1_act;
	Eigen::Matrix<double, 7, 1> config;
	// origin zero case
	config << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;

	Eigen::VectorXd err_act = factor.evaluateError(config, &H1_act);
	std::cout<<" [debug] err_act: "<<err_act<<std::endl;
    std:;cout<<"end"<<std::endl;
    
    // 三、启动movegroup
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
    ros::Rate r(20);
    while (ros::ok())
    {
    	// 获取当前关节角度
  		std::vector<double> joint_angles = move_group.getCurrentJointValues();
  		config << joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4], joint_angles[5], joint_angles[6];

  //   	vis_arm_tools.publish_collision_spheres(config) ;
		// vis_arm_tools.publish_arm_link_spheres(config) ;

        // std::cout<<"[debug] joint_angles: ";
        // for(int i=0; i<joint_angles.size(); i++){
        //     std::cout<<joint_angles[i]<<" "<<config[i]<<std::endl;
        // }

    	// （2）


    	// （3）测试factor3
    	// 获取末端执行器的位姿
    	geometry_msgs::PoseStamped end_effector_pose = move_group.getCurrentPose();
    	auto end_effector_pose_eigen = Converter::geometryPoseStampedtoMatrix4d(end_effector_pose);
    	Eigen::Matrix4d T_endlink_to_c;
    	T_endlink_to_c << 0, 0, 1, 0.02,
						  -1, 0, 0, -0.013,
						  0, -1, 0, 0.13,  //实际为0.13(用于ros)，改为0.07(用于gpmp2 forwardKinematics)
						  0, 0, 0, 1;
    	Eigen::Matrix4d T_baselink_to_c = end_effector_pose_eigen * T_endlink_to_c;
    	Eigen::MatrixXd H1_camerapose;
    	Eigen::Vector4d err_bbox = factor3.evaluateError(gtsam::Pose3(T_baselink_to_c), Quadric, &H1_camerapose, {});
    	Eigen::Vector4d measure_bbox = factor3.getMeasureBounds (gtsam::Pose3(T_baselink_to_c), Quadric);
    	Eigen::Vector4d predict_bbox = factor3.getPredictedBounds (gtsam::Pose3(T_baselink_to_c), Quadric);

		// （4）测试factor4
    	err_bbox = factor4.evaluateError(config, &H1_camerapose);
    	measure_bbox = gtsam_bbox.vector();
    	predict_bbox = factor4.getPredictedBounds (config);

        #include <opencv2/opencv.hpp>
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

    	// std::cout<<" [debug] err_bbox:     "<<err_bbox.transpose()<<std::endl;
        // std::cout<<" [debug] measure_bbox: "<<measure_bbox.transpose()<<std::endl;
        // std::cout<<" [debug] predict_bbox: "<<predict_bbox.transpose()<<std::endl;
        // std::cout<<" [debug] H1:     \n"<<H1_camerapose<<std::endl;

    	std::cout<<"end"<<std::endl;


        // std::cout << "Press [ENTER] to continue ... , [y] to autonomous mode" << std::endl;
        // char key = getchar();

        r.sleep();
    }

    

    return 0;
}