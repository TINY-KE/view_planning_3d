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
#include "gtsam/BboxPlaneArmLink.h"
#include "GenerateArm.h"

#include "visualize_arm_tools.h"

using namespace Eigen;
using namespace gtsam;
using namespace gpmp2;

typedef BboxPlaneArmLink<ArmModel> BboxPlaneArmLinkArm;

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
    double obs_eps = 0.2;
    double epsilon_dist = 0.05;
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
	BboxPlaneArmLink<ArmModel> factor(0, *arm_model,
                               1.0,
                               epsilon_dist,
                               CameraWidth, CameraHeight,
                               Calib
                               );
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
	Visualize_Arm_Tools vis_arm_tools(nh, *arm_model, move_group, "wam/base_link" );
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

        g2o::plane* plane_in_baselink = factor.computeplane(config);
        vis_tools->MapPlaneNormals.clear();
        vis_tools->MapPlaneNormals.push_back(plane_in_baselink->param);

        Eigen::VectorXd err_act = factor.evaluateError(config, &H1_act);
        std::cout<<" [debug] err_act: "<<err_act<<std::endl;
        std::cout<<"end"<<std::endl;

        // std::cout << "Press [ENTER] to continue ... , [y] to autonomous mode" << std::endl;
        // char key = getchar();

        r.sleep();
    }

    

    return 0;
}