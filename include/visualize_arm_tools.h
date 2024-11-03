#ifndef VIEW_ARM_TOOLS_H
#define VIEW_ARM_TOOLS_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Float64MultiArray.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

 // gpmp2 机械臂可视化
 #include <gtsam/base/Matrix.h>
 #include <gtsam/base/Vector.h>
 #include <gtsam/geometry/Point3.h>
 #include <gtsam/nonlinear/NonlinearFactor.h>
 #include <gtsam/geometry/Pose3.h>
 #include <gpmp2/kinematics/ArmModel.h>
 #include <gpmp2/kinematics/Arm.h>
 // #include "core/Plane.h"

 using namespace gpmp2;
 using namespace gtsam;



 typedef gpmp2::ArmModel Robot;
 class Visualize_Arm_Tools {
 public:
     Visualize_Arm_Tools(ros::NodeHandle& nh, const Robot& robot, moveit::planning_interface::MoveGroupInterface& move_group, int width, int height, Eigen::Matrix3d calib, std::string default_frame )
       : default_frame_(default_frame), move_group_(move_group), robot_(robot),
                miImageCols(width),
                miImageRows(height),
                mCalib(calib)
     {
         collision_spheres_pub = nh.advertise<visualization_msgs::Marker>("collision_spheres", 1);
         arm_link_spheres_pub = nh.advertise<visualization_msgs::Marker>("arm_spheres", 1);
         bbox_plane_pub = nh.advertise<visualization_msgs::Marker>("bbox_plane", 1);

         mRobotPose = Eigen::Matrix4f::Identity();
     }

 private:
     ros::Publisher collision_spheres_pub;
     ros::Publisher arm_link_spheres_pub;
     ros::Publisher bbox_plane_pub;

     // arm: planar one, all alpha = 0
     const Robot& robot_;
     std::string default_frame_;
     moveit::planning_interface::MoveGroupInterface& move_group_;
     // 图像宽度和高度
     int miImageCols; // = Config::Get<int>("Camera.width");
     int miImageRows; // = Config::Get<int>("Camera.height");
     Eigen::Matrix3d mCalib;
     double mDepth = 6.0;   //预期的相机视场长度  1.0用于截图
     double mFOV_decrease = 145;
     Eigen::Matrix4f mRobotPose;


public:
     void setRobotPose(Eigen::Matrix4f& robot_pose) {
         mRobotPose = robot_pose;
     }
     void setFOVDepth(double mDepth_) {
         mDepth = mDepth_;
     }
     void setFOVDecrease(double mFOV_decrease_) {
         mFOV_decrease = mFOV_decrease_;
     }

 public:
     void Run()
     {
//         ros::AsyncSpinner spinner(1);
//         spinner.start();
//         moveit::planning_interface::MoveGroupInterface move_group("arm");
//         std::string end_effector_link=move_group.getEndEffectorLink();
//
//         std::string pose_reference_frame="/wam/base_link";
//         // std::string pose_reference_frame="world";
//         move_group.setPoseReferenceFrame(pose_reference_frame);

         // geometry_msgs::PoseStamped end_pose = move_group.getCurrentPose();
//         const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("arm");

         ros::Rate r(100);

         while(1){
                std::vector<double> joint_angles = move_group_.getCurrentJointValues();
                Eigen::Matrix<double, 7, 1> config;
                config << joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4], joint_angles[5], joint_angles[6];
                publish_collision_spheres(config) ;
                publish_arm_link_spheres(config) ;
                visualize_plane_triangle_bypoint(config);
                r.sleep();
             }
     }


 //发布机械臂的球体
 private:
     void publish_collision_spheres(const Eigen::Matrix<double, 7, 1>& conf) {


         // run forward kinematics of this configuration
         std::vector<Point3> sph_centers;
         std::vector<gtsam::Matrix> J_px_jp;
         robot_.sphereCenters(conf, sph_centers);

         // for each point on arm stick, get error
         for (int sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {
             // 避障球 半径
             const double total_eps = robot_.sphere_radius(sph_idx);
             // 避障球 中心坐标
             const gtsam::Point3& point = sph_centers[sph_idx];

             visualization_msgs::Marker marker;
             marker.header.frame_id = default_frame_; // Assuming the frame_id is "base_link"
             marker.header.stamp = ros::Time::now();
             marker.ns = "collision_spheres";
             marker.id = sph_idx;
             marker.type = visualization_msgs::Marker::SPHERE;
             marker.action = visualization_msgs::Marker::ADD;

             // Set position
             marker.pose.position.x = point.x();
             marker.pose.position.y = point.y();
             marker.pose.position.z = point.z();

             // Set orientation
             marker.pose.orientation.x = 0.0;
             marker.pose.orientation.y = 0.0;
             marker.pose.orientation.z = 0.0;
             marker.pose.orientation.w = 1.0;

             // Set scale (diameter)
             marker.scale.x = total_eps*2.0;
             marker.scale.y = total_eps*2.0;
             marker.scale.z = total_eps*2.0;

             // Set color (red)
             marker.color.a = 0.7;
             marker.color.r = 1.0;
             marker.color.g = 1.0;
             marker.color.b = 0.0;

             marker.lifetime = ros::Duration(); // Persistent marker

             collision_spheres_pub.publish(marker);
         }

     }


     void publish_arm_link_spheres(const Eigen::Matrix<double, 7, 1>& conf) {


         // run forward kinematics of this configuration
         // 机械臂endlink的位姿
         std::vector<Pose3> joint_pos;   //  link poses in 3D work space
         std::vector<gtsam::Matrix> J_jpx_jp;   //  et al. optional Jacobians
         robot_.fk_model().forwardKinematics(conf, {}, joint_pos);

         // for each point on arm stick, get error
         for (int sph_idx = 0; sph_idx < joint_pos.size(); sph_idx++) {

             // 各关节末端 中心坐标
             double x = joint_pos[sph_idx].x();
             double y = joint_pos[sph_idx].y();
             double z = joint_pos[sph_idx].z();

             double total_eps = 0.08;

             visualization_msgs::Marker marker;
             marker.header.frame_id = default_frame_; // Assuming the frame_id is "base_link"
             marker.header.stamp = ros::Time::now();
             marker.ns = "arm_link_spheres";
             marker.id = sph_idx;
             marker.type = visualization_msgs::Marker::SPHERE;
             marker.action = visualization_msgs::Marker::ADD;

             // Set position
             marker.pose.position.x = x;
             marker.pose.position.y = y;
             marker.pose.position.z = z;

             // Set orientation
             marker.pose.orientation.x = 0.0;
             marker.pose.orientation.y = 0.0;
             marker.pose.orientation.z = 0.0;
             marker.pose.orientation.w = 1.0;

             // Set scale (diameter)
             marker.scale.x = total_eps*2.0;
             marker.scale.y = total_eps*2.0;
             marker.scale.z = total_eps*2.0;

             // Set color (red)
             marker.color.a = 0.7;
             marker.color.r = .0;
             marker.color.g = .0;
             marker.color.b = 1.0;

             marker.lifetime = ros::Duration(); // Persistent marker

             arm_link_spheres_pub.publish(marker);
         }

     }


//发布相机视场
private:
        geometry_msgs::Point pixelToCamera(const Eigen::Vector2d& pixel) {
            // 获取相机内参矩阵的元素
            double fx = mCalib(0, 0);  // 焦距 fx
            double fy = mCalib(1, 1);  // 焦距 fy
            double cx = mCalib(0, 2);  // 光心 cx
            double cy = mCalib(1, 2);  // 光心 cy

            // 提取像素坐标 u, v
            double u = pixel(0);
            double v = pixel(1);

            // 将像素坐标转换为相机坐标系下的三维坐标
            double x = (u - cx) * mDepth / fx;
            double y = (v - cy) * mDepth / fy;
            double z = mDepth;

            // 相机坐标系下的三维点
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = z;

            // geometry_msgs::Point p2 = transformPoint(camera_pose, p);

            return p;
        }

        geometry_msgs::Point transformPointToWorld(const Eigen::Matrix4f& T_world_camera, const geometry_msgs::Point& point_camera) {

            // 将相机坐标点转换为 4D 齐次坐标形式 (x, y, z, 1)
            Eigen::Vector4f point_camera_homo(point_camera.x, point_camera.y, point_camera.z, 1.0);

            // 使用变换矩阵将相机坐标系中的点转换为世界坐标系
            Eigen::Vector4f point_world_homo = T_world_camera * point_camera_homo;

            // 将结果转换为 geometry_msgs::Point，并返回
            geometry_msgs::Point point_world;
            point_world.x = point_world_homo(0);
            point_world.y = point_world_homo(1);
            point_world.z = point_world_homo(2);

            return point_world;
        }

        std::vector<std::vector<geometry_msgs::Point>> GenerateBboxPlanesTrianglePoints(const typename Robot::Pose& conf) {

            // 1. ROBOT pose
            // 相机相对于endlink的位姿
            Eigen::Matrix4f T_endlink_to_c;
            T_endlink_to_c << 0, 0, 1, 0.02,
                              -1, 0, 0, -0.013,
                              0, -1, 0, 0.07,  //实际为0.13，改为0.07
                              0, 0, 0, 1;
            // 机械臂endlink的位姿
            std::vector<Pose3> joint_pos;   //  link poses in 3D work space
            std::vector<gtsam::Matrix> J_jpx_jp;   //  et al. optional Jacobians
            robot_.fk_model().forwardKinematics(conf, {}, joint_pos);
            Pose3 pose_end_link = joint_pos[joint_pos.size()-1];
            // 将 gtsam::Pose3 转换为 Eigen::Matrix4f
            Eigen::Matrix4f T_baselink_endlink = Eigen::Matrix4f::Identity();  // 创建 4x4 单位矩阵
            // 获取 gtsam::Pose3 的 3x3 旋转矩阵并赋值到 eigenMatrix 的左上角
            T_baselink_endlink.block<3, 3>(0, 0) = pose_end_link.rotation().matrix().cast<float>();
            // 获取 gtsam::Pose3 的 3x1 平移向量并赋值到 eigenMatrix 的右侧
            T_baselink_endlink.block<3, 1>(0, 3) << pose_end_link.x(), pose_end_link.y(), pose_end_link.z();
            Eigen::Matrix4f T_world_2_c =  mRobotPose * T_baselink_endlink * T_endlink_to_c;


            // 2. 生成相机坐标系下的三维点
            double s = mFOV_decrease;
            Eigen::Vector4d temp_bbox(0 + s, 0 + s * miImageRows / miImageCols, miImageCols - s,
                                                   miImageRows - s * miImageRows / miImageCols);

            Eigen::Vector4d bbox = temp_bbox;

            geometry_msgs::Point centor_3d;
            centor_3d.x = T_world_2_c(0, 3);
            centor_3d.y = T_world_2_c(1, 3);
            centor_3d.z = T_world_2_c(2, 3);

            double x1 = bbox(0);
            double y1 = bbox(1);
            double x2 = bbox(2);
            double y2 = bbox(3);

            // 1----2
            // |    |
            // 3----4

            Eigen::Vector2d corner1(x1, y1);
            Eigen::Vector2d corner2(x2, y1);
            Eigen::Vector2d corner3(x1, y2);
            Eigen::Vector2d corner4(x2, y2);

            // 相机坐标系下的三维点
            auto corner1_3d = transformPointToWorld(T_world_2_c, pixelToCamera(corner1));
            auto corner2_3d = transformPointToWorld(T_world_2_c, pixelToCamera(corner2));
            auto corner3_3d = transformPointToWorld(T_world_2_c, pixelToCamera(corner3));
            auto corner4_3d = transformPointToWorld(T_world_2_c, pixelToCamera(corner4));
            // std::cout<<"[debug] corner1_3d: "<<corner1_3d<<std::endl;
            // std::cout<<"[debug] corner2_3d: "<<corner2_3d<<std::endl;
            // std::cout<<"[debug] corner3_3d: "<<corner3_3d<<std::endl;
            // std::cout<<"[debug] corner4_3d: "<<corner4_3d<<std::endl;
            std::cout<<std::endl;

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


     // 以三角的形式显示平面
     void visualize_plane_triangle_bypoint(const typename Robot::Pose& conf) {

            std::vector<std::vector<geometry_msgs::Point>> BboxPlanesTrianglePointsInWorld = GenerateBboxPlanesTrianglePoints(conf);

            
            for(int i=0; i<BboxPlanesTrianglePointsInWorld.size(); i++) {
                auto points = BboxPlanesTrianglePointsInWorld[i];

                visualization_msgs::Marker marker;
                marker.header.frame_id = default_frame_;  // 使用世界坐标系
                marker.header.stamp = ros::Time::now();
                marker.ns = "endpoint_bbox_plane";
                marker.id = i;
                marker.type = visualization_msgs::Marker::TRIANGLE_LIST;  // 使用三角形列表
                // marker.type = visualization_msgs::Marker::POINTS; 
                marker.action = visualization_msgs::Marker::ADD;

                // 设置颜色和透明度
                marker.color.a = 0.2;  // 不透明
                marker.color.r = 1.0;  // 红色
                marker.color.g = 0.0;
                marker.color.b = 0.0;

                // 设置比例
                marker.scale.x = 1.0;
                marker.scale.y = 1.0;
                marker.scale.z = 1.0;

                // std::vector<geometry_msgs::Point> points;

                // // 生成平面上的点
                // points.push_back(plane[0]);
                // points.push_back(plane[1]);
                // points.push_back(plane[2]);
                marker.points = points;

                // 发布Marker消息
                bbox_plane_pub.publish(marker);
                
            }


            // DEBUG 端点
            // // 创建一个 Marker 消息
            // visualization_msgs::Marker points_marker;
            // points_marker.header.frame_id = default_frame_;  // 修改为你的坐标系
            // points_marker.header.stamp = ros::Time::now();
            // points_marker.ns = "points_ns";
            // points_marker.id = 0;
            // points_marker.type = visualization_msgs::Marker::POINTS;  // 设置 Marker 类型为 POINTS
            // points_marker.action = visualization_msgs::Marker::ADD;
            // // 设置点的颜色和大小
            // points_marker.scale.x = 0.1;  // 点的大小
            // points_marker.scale.y = 0.1;  // 点的大小
            // points_marker.color.r = 1.0;  // 红色
            // points_marker.color.g = 0.0;  // 绿色
            // points_marker.color.b = 0.0;  // 蓝色
            // points_marker.color.a = 1.0;  // 透明度
            // // 遍历点集合，添加所有点到 Marker 的 points 数组中
            // for (const auto& point_vector : BboxPlanesTrianglePointsInWorld) {
            //     for (const auto& point : point_vector) {
            //         points_marker.points.push_back(point);
            //     }
            // }
            // // 发布 Marker
            // bbox_plane_pub.publish(points_marker);

            
        }


 };


#endif //VIEW_PLANNING_GAZEBO_TOOLS_H