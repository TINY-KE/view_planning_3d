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
     Visualize_Arm_Tools(ros::NodeHandle& nh, const Robot& robot, moveit::planning_interface::MoveGroupInterface& move_group, std::string default_frame )
       : default_frame_(default_frame), move_group_(move_group), robot_(robot)
     {
         collision_spheres_pub = nh.advertise<visualization_msgs::Marker>("collision_spheres", 1);
         arm_link_spheres_pub = nh.advertise<visualization_msgs::Marker>("arm_spheres", 1);
     }

 private:
     ros::Publisher collision_spheres_pub;
     ros::Publisher arm_link_spheres_pub;
     // arm: planar one, all alpha = 0
     const Robot& robot_;
     std::string default_frame_;
     moveit::planning_interface::MoveGroupInterface& move_group_;

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

         ros::Rate r(50);

         while(1){
                std::vector<double> joint_angles = move_group_.getCurrentJointValues();
                Eigen::Matrix<double, 7, 1> config;
                config << joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4], joint_angles[5], joint_angles[6];
                publish_collision_spheres(config) ;
                r.sleep();
                publish_arm_link_spheres(config) ;
                r.sleep();
             }
     }



     void publish_collision_spheres(const Eigen::Matrix<double, 7, 1>& conf) {


         // run forward kinematics of this configuration
         std::vector<Point3> sph_centers;
         std::vector<Matrix> J_px_jp;
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
         std::vector<Matrix> J_jpx_jp;   //  et al. optional Jacobians
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


 };



#endif //VIEW_PLANNING_GAZEBO_TOOLS_H