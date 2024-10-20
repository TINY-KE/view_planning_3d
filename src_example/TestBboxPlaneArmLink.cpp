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

// #include "gtsam/BboxPlaneEllipsoidFactor.h"
// #include "gtsam/JointLimitFactorVector.h"
#include "gtsam/BboxPlaneArmLinkFactor.h"
#include "gtsam/BboxPlaneEllipsoidFactor.h"
#include "GenerateArm.h"

#include "visualize_arm_tools.h"

#include "gtsam_quadrics/geometry/BoundingBoxFactor.h"

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
    // RobotPose << 0, -1, 0, 0,
    //              1, 0, 0, 10,
    //              0, 0, 1, 0,
    //              0, 0, 0, 1;
	BboxPlaneEllipsoidFactor<ArmModel> factor2(0, *arm_model,
						1.0,
						ob,
						RobotPose,
						CameraWidth, CameraHeight,
						Calib);

    // （3）构建视场和gtsam椭球体的因子
	double s = 100;
	gtsam_quadrics::AlignedBox2 gtsam_bbox(0+s, 0+s, CameraWidth-s, CameraHeight-s);
	// gtsam::Cal3_S2 gtsam_calib(fx, fy, 0.0, cx, cy);
	boost::shared_ptr<gtsam::Cal3_S2> gtsam_calib = boost::make_shared<gtsam::Cal3_S2>(fx, fy, 0.0, cx, cy);
	// 使用数组中的值创建噪声模型
	double sigmasArray[4] = {10.0, 10.0, 10.0, 10.0};
	gtsam::SharedNoiseModel bbox_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4(sigmasArray));
    // BoundingBoxFactor(const AlignedBox2& measured,
    //                 const boost::shared_ptr<gtsam::Cal3_S2>& calibration,
    //                 const gtsam::Key& poseKey, const gtsam::Key& quadricKey,
    //                 const gtsam::SharedNoiseModel& model,
    //                 const MeasurementModel& errorType = STANDARD)
	gtsam::Key Key1, Key2;
	// gtsam_quadrics::BoundingBoxFactor factor3(gtsam_bbox, gtsam_calib, Key1, Key2, bbox_noise);
	// gtsam_quadrics::BoundingBoxFactor factor3;



    
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

        // g2o::plane* plane_in_baselink = factor.computeplane(config);
		auto planesWorld= factor2.computeplanes(config);
    	vis_tools->MapPlaneNormals.clear();
    	// vis_tools->MapPlaneNormals.push_back(planesWorld[0]->param);
    	// vis_tools->MapPlaneNormals.push_back(planesWorld[1]->param);
    	// vis_tools->MapPlaneNormals.push_back(planesWorld[2]->param);
    	// vis_tools->MapPlaneNormals.push_back(planesWorld[3]->param);

        Eigen::VectorXd err_act = factor.evaluateError(config, &H1_act);
        std::cout<<" [debug] err_act: "<<err_act<<std::endl;
        std::cout<<"end"<<std::endl;

        // std::cout << "Press [ENTER] to continue ... , [y] to autonomous mode" << std::endl;
        // char key = getchar();

        r.sleep();
    }

    

    return 0;
}