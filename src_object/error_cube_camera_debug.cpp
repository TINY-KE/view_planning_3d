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

    // 相机z轴方向，各个平面的normal应该与camera_direction 夹角小于90度
    Eigen::Vector3d camera_direction(0,0,1);  // Z轴方向

    // add to matrix
    for (int m = 0; m < planes.cols(); m++) {
        // 获取平面的法向量 (a, b, c)
        Eigen::Vector3d normal = planes.block<3, 1>(0, m);

        // 检查法向量与z轴朝向相同，如果不是则反转法向量
        if (normal.dot(camera_direction) < 0) {
            // 反转法向量方向
            planes.col(m) = -planes.col(m);
        }

        planes_all.conservativeResize(planes_all.rows(), planes_all.cols() + 1);
        planes_all.col(planes_all.cols() - 1) = planes.col(m);
    }

    return planes_all;
}


geometry_msgs::Point transformPoint(const geometry_msgs::PoseStamped& pose, const geometry_msgs::Point& point) {
    // 提取 PoseStamped 中的位姿
    const geometry_msgs::Point& pos = pose.pose.position;   // 平移部分
    const geometry_msgs::Quaternion& quat = pose.pose.orientation; // 旋转部分

    // 将 Point 转换为 tf2::Vector3
    tf2::Vector3 point_vec(point.x, point.y, point.z);

    // 将 PoseStamped 的四元数转换为 tf2::Quaternion
    tf2::Quaternion rotation;
    tf2::fromMsg(quat, rotation);

    // 创建一个变换对象 (平移 + 旋转)
    tf2::Transform transform(rotation, tf2::Vector3(pos.x, pos.y, pos.z));

    // 使用变换对点进行变换
    tf2::Vector3 transformed_point = transform * point_vec;

    // 将结果转换回 geometry_msgs::Point
    geometry_msgs::Point result;
    result.x = transformed_point.x();
    result.y = transformed_point.y();
    result.z = transformed_point.z();

    return result;
}

geometry_msgs::Point pixelToCamera(const Eigen::Vector2d& pixel, double depth, const Eigen::Matrix3d& calib, geometry_msgs::PoseStamped& camera_pose) {
    // 获取相机内参矩阵的元素
    double fx = calib(0, 0);  // 焦距 fx
    double fy = calib(1, 1);  // 焦距 fy
    double cx = calib(0, 2);  // 光心 cx
    double cy = calib(1, 2);  // 光心 cy

    // 提取像素坐标 u, v
    double u = pixel(0);
    double v = pixel(1);

    // 将像素坐标转换为相机坐标系下的三维坐标
    double x = (u - cx) * depth / fx;
    double y = (v - cy) * depth / fy;
    double z = depth;

    // 相机坐标系下的三维点
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;

    geometry_msgs::Point p2 = transformPoint(camera_pose, p);

    return p2;
}

std::vector<std::vector<geometry_msgs::Point>> GenerateBboxPlanesTrianglePoints(geometry_msgs::PoseStamped& camera_pose, Eigen::Vector4d &bbox, Matrix3d &calib) {
    geometry_msgs::Point centor_3d;
    centor_3d.x = camera_pose.pose.position.x;
    centor_3d.y = camera_pose.pose.position.y;
    centor_3d.z = camera_pose.pose.position.z;
    
    double x1 = bbox(0);
    double y1 = bbox(1);
    double x2 = bbox(2);
    double y2 = bbox(3);

    // 1----2
    // |    |
    // 3----4

    Vector2d corner1(x1, y1);
    Vector2d corner2(x2, y1);
    Vector2d corner3(x1, y2);
    Vector2d corner4(x2, y2);
    
    double depth = 3;
    auto corner1_3d = pixelToCamera(corner1, depth, calib, camera_pose);
    auto corner2_3d = pixelToCamera(corner2, depth, calib, camera_pose);
    auto corner3_3d = pixelToCamera(corner3, depth, calib, camera_pose);
    auto corner4_3d = pixelToCamera(corner4, depth, calib, camera_pose);

    std::vector<std::vector<geometry_msgs::Point>>  planes(4, std::vector<geometry_msgs::Point>(3));
    planes[0].push_back(centor_3d);
    planes[0].push_back(corner1_3d);
    planes[0].push_back(corner3_3d);
    planes[1].push_back(centor_3d);
    planes[1].push_back(corner1_3d);
    planes[1].push_back(corner2_3d);
    planes[2].push_back(centor_3d);
    planes[2].push_back(corner2_3d);
    planes[2].push_back(corner4_3d);
    planes[3].push_back(centor_3d);
    planes[3].push_back(corner4_3d);
    planes[3].push_back(corner3_3d);

    return planes;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "triangle_marker_node");
    ros::NodeHandle nh;
    Visualize_Tools* vis_tools = new Visualize_Tools(nh);
    std::thread* mptVisualizeTools;
    mptVisualizeTools = new std::thread(&Visualize_Tools::Run, vis_tools);

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
    ob->Update_corner();
    // int pub_num = 6;
    // while(pub_num--){
    //     vis_tools.visualize_ellipsoid(3, 0, 0.5, 
    //             2.5, 2, 1, 
    //             0, 0, 0, "world", 0);
    // }
    vis_tools->MapObjects.push_back(ob);

    // 二、生成平面
    g2o::SE3Quat local_wc = g2o::SE3Quat();
    int CameraWidth = 640;
    int CameraHeight = 480;
    Vector4d bbox(0+100, 0+80, 640-100, 480-80);
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

    // geometry_msgs::PoseStamped end_pose = move_group.getCurrentPose();
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("arm");

    
    // 创建 TF2 监听器
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    ros::Rate r(20);
    while (ros::ok())
    {
        // 一、 获取当前的机器人状态和相机坐标
        // 等待获取相机在odom坐标系下的变换
        geometry_msgs::TransformStamped transform_stamped;

        transform_stamped = tf_buffer.lookupTransform("wam/base_link", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));

        // 将 TransformStamped 转换为 PoseStamped
        geometry_msgs::PoseStamped end_pose;
        end_pose.header.stamp = transform_stamped.header.stamp;
        end_pose.header.frame_id = transform_stamped.header.frame_id;

        // 设置位置
        end_pose.pose.position.x = transform_stamped.transform.translation.x;
        end_pose.pose.position.y = transform_stamped.transform.translation.y;
        end_pose.pose.position.z = transform_stamped.transform.translation.z;

        // 设置方向（四元数）
        end_pose.pose.orientation = transform_stamped.transform.rotation;

        // 输出相机在 odom 坐标系下的位姿
        ROS_INFO_STREAM("Pose of camera_rgb_frame in odom: \n" << end_pose);



        // 三、通过三角形，可视化视场平面
        std::vector<std::vector<geometry_msgs::Point>> BboxPlanesTrianglePointsInWorld = GenerateBboxPlanesTrianglePoints(end_pose,  bbox, mCalib); 
        vis_tools->BboxPlanesTrianglePointsInWorld = BboxPlanesTrianglePointsInWorld;



        // 四、计算视场平面的方程
        Eigen::Matrix4d T = Converter::Quation2Eigen(end_pose.pose.orientation.x, end_pose.pose.orientation.y, end_pose.pose.orientation.z, end_pose.pose.orientation.w, 
                                        end_pose.pose.position.x, end_pose.pose.position.y, end_pose.pose.position.z);
        Eigen::Matrix<double,3,3> R = T.block<3,3>(0,0);
        Eigen::Matrix<double,3,1> t = T.block<3,1>(0,3);
        g2o::SE3Quat cam_pose_Twc = g2o::SE3Quat(R,t);

        std::vector<g2o::plane*> planes_world;
        for( int i=0;i<num;i++)
            {
                VectorXd vec = mPlanesParamLocal.row(i);
                g2o::plane* plane_new= new g2o::plane(vec.head(4));
                plane_new->transform(cam_pose_Twc);  // 转换到世界坐标系
                planes_world.push_back(plane_new);
            }

        // 五、计算cube
        Eigen::Vector3d target_pose_eigen = (ob->mCuboid3D.corner_1 + ob->mCuboid3D.corner_7)/2.0 ;
        std::cout<<"Target Pose: "<<target_pose_eigen.transpose()<<std::endl;
        
        double distance_low = planes_world[3]->distanceToPoint(target_pose_eigen, true);
        double distance_high = planes_world[1]->distanceToPoint(target_pose_eigen, true);
        double distance_left = planes_world[0]->distanceToPoint(target_pose_eigen, true);
        double distance_right = planes_world[2]->distanceToPoint(target_pose_eigen, true);
        std::cout<<"normal 0: "<< planes_world[0]->normal().transpose()<<std::endl;
        std::cout<<"normal 1: "<< planes_world[1]->normal().transpose()<<std::endl;
        std::cout<<"normal 2: "<< planes_world[2]->normal().transpose()<<std::endl;
        std::cout<<"normal 3: "<< planes_world[3]->normal().transpose()<<std::endl;

        
        std::cout<<"Distance：          ↑"<<std::endl;
        std::cout<<"                "<<distance_high<<std::endl;
        std::cout<<"        ←  "<<distance_left<<"    "<<distance_right<<" → "<<std::endl;
        std::cout<<"                "<<distance_low<<std::endl;
        std::cout<<"                    ↓"<<std::endl;

        std::cout<<std::endl;
        std::cout<<std::endl;
        
        r.sleep();
    }
    
    

    return 0;
}