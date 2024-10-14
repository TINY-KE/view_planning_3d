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

using namespace Eigen;
// ZHJD 移植
Matrix3Xd generateProjectionMatrix(const g2o::SE3Quat &campose_cw, const Matrix3d &Kalib) {
    Matrix3Xd identity_lefttop;
    identity_lefttop.resize(3, 4);
    identity_lefttop.col(3) = Vector3d(0, 0, 0);
    identity_lefttop.topLeftCorner<3, 3>() = Matrix3d::Identity(3, 3);

    Matrix3Xd proj_mat = Kalib * identity_lefttop;

    proj_mat = proj_mat * campose_cw.to_homogeneous_matrix();

    return proj_mat;
}

MatrixXd fromDetectionsToLines(Vector4d &detections, int miImageCols, int miImageRows) {
    bool flag_openFilter = false; // filter those lines lying on the image boundary

    double x1 = detections(0);
    double y1 = detections(1);
    double x2 = detections(2);
    double y2 = detections(3);

    Vector3d line1(1, 0, -x1);
    Vector3d line2(0, 1, -y1);
    Vector3d line3(1, 0, -x2);
    Vector3d line4(0, 1, -y2);

    // those lying on the image boundary have been marked -1
    MatrixXd line_selected(3, 0);
    MatrixXd line_selected_none(3, 0);

    int config_border_pixel = 10;
    if (!flag_openFilter || (x1 > config_border_pixel && x1 < miImageCols - config_border_pixel)) {
        line_selected.conservativeResize(3, line_selected.cols() + 1);
        line_selected.col(line_selected.cols() - 1) = line1;
    }
    if (!flag_openFilter || (y1 > config_border_pixel && y1 < miImageRows - config_border_pixel)) {
        line_selected.conservativeResize(3, line_selected.cols() + 1);
        line_selected.col(line_selected.cols() - 1) = line2;
    }
    if (!flag_openFilter || (x2 > config_border_pixel && x2 < miImageCols - config_border_pixel)) {
        line_selected.conservativeResize(3, line_selected.cols() + 1);
        line_selected.col(line_selected.cols() - 1) = line3;
    }
    if (!flag_openFilter || (y2 > config_border_pixel && y2 < miImageRows - config_border_pixel)) {
        line_selected.conservativeResize(3, line_selected.cols() + 1);
        line_selected.col(line_selected.cols() - 1) = line4;
    }

    return line_selected;
}

MatrixXd GenerateBboxPlanes(g2o::SE3Quat &campose_wc, Eigen::Vector4d &bbox, Matrix3d &calib, int CameraWidth, int CameraHeight) {
    MatrixXd planes_all(4, 0);
    // std::cout << " [debug] calib : \n " << calib << std::endl;
    // get projection matrix
    MatrixXd P = generateProjectionMatrix(campose_wc.inverse(), calib);

    MatrixXd lines = fromDetectionsToLines(bbox,CameraWidth,CameraHeight);
    MatrixXd planes = P.transpose() * lines;

    // add to matrix
    for (int m = 0; m < planes.cols(); m++) {
        planes_all.conservativeResize(planes_all.rows(), planes_all.cols() + 1);
        planes_all.col(planes_all.cols() - 1) = planes.col(m);
    }

    return planes_all;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "triangle_marker_node");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // 一、生成物体
    // SdfObject ob( 3, 0, 0.5, 
    //             2.5, 2, 1, 
    //             0, 0, 0);
    double x=3, y=0, z=0.5;
    double lenth=2.5, width=2, height=1;
    double yaw = 0;
    MapObject* ob = new MapObject();
    ob->Update_Twobj(x,y,z,yaw);
    ob->Update_object_size(lenth,width,height);


    // 二、生成平面
    g2o::SE3Quat local_wc = g2o::SE3Quat();
    int CameraWidth = 640;
    int CameraHeight = 480;
    Vector4d bbox(0+100, 0+100, 640-100, 480-100);
    float fx = 554.254691191187;
    float fy = 554.254691191187;
    float cx = 320.5;
    float cy = 240.5;
    Eigen::Matrix3d mCalib;  
    mCalib << fx,  0,  cx,
              0,  fy,  cy,
              0,   0,   1;

    Eigen::MatrixXd mPlanesParamLocal_Col = GenerateBboxPlanes(local_wc, bbox, mCalib, CameraWidth, CameraHeight);  // attention: store as 列
    Eigen::MatrixXd mPlanesParamLocal = mPlanesParamLocal_Col.transpose(); 
    
    int num = mPlanesParamLocal.rows();
    

    // 三、启动movegroup
    // 创建一个异步的自旋线程（spinning thread）
	ros::AsyncSpinner spinner(1);
	spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    std::string end_effector_link=move_group.getEndEffectorLink();
	ROS_INFO_NAMED("WAM_arm", "End effector link: %s", end_effector_link.c_str());
    	
    ROS_INFO_NAMED("WAM_arm", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    std::string pose_reference_frame="/wam/base_link";
    // std::string pose_reference_frame="world";
    move_group.setPoseReferenceFrame(pose_reference_frame);
	ROS_INFO_NAMED("WAM_arm", "New Pose Reference frame: %s", move_group.getPoseReferenceFrame().c_str());

    geometry_msgs::PoseStamped end_pose = move_group.getCurrentPose();

    for( int i=0;i<num;i++)
        {
            VectorXd vec = mPlanesParamLocal.row(i);
            g2o::plane* plane_new= new g2o::plane(vec.head(4));
            // plane_new->transform(pFrame->cam_pose_Twc);  // 转换到世界坐标系
            // planes_world.push_back(plane_new);
        }



    return 0;
}