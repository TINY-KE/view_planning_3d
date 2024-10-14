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
    g2o::plane* plane_low = new g2o::plane();
    g2o::plane* plane_high = new g2o::plane();
    g2o::plane* plane_left = new g2o::plane();
    g2o::plane* plane_right = new g2o::plane();
    Eigen::Vector3d center, normal;
    center[0] = 0;
    center[1] = 0;
    center[2] = 0.5;
    normal[0] = 0;
    normal[1] = 0;
    normal[2] = 1;
    plane_low->fromPointAndNormal(center, normal);
    center[0] = 0;
    center[1] = 0;
    center[2] = 0.9;
    normal[0] = 0;
    normal[1] = 0;
    normal[2] = -1;
    plane_high->fromPointAndNormal(center, normal);
    center[0] = 0;
    center[1] = 0;
    center[2] = 0;
    normal[0] = 1;
    normal[1] = -0.2;
    normal[2] = 0.2;
    plane_left->fromPointAndNormal(center, normal);
    center[0] = 0;
    center[1] = 0;
    center[2] = 0;
    normal[0] = -0.2;
    normal[1] = 1;
    normal[2] = 0.2;
    plane_right->fromPointAndNormal(center, normal);



    ros::Rate r(1);
    publish_plane_triangle(marker_pub, plane_low->GeneratePlaneVec(), 0);
    publish_plane_triangle(marker_pub, plane_high->GeneratePlaneVec(), 1);
    publish_plane_triangle(marker_pub, plane_left->GeneratePlaneVec(), 2);
    publish_plane_triangle(marker_pub, plane_right->GeneratePlaneVec(), 3);
    r.sleep();
    publish_plane_triangle(marker_pub, plane_low->GeneratePlaneVec(), 0);
    publish_plane_triangle(marker_pub, plane_high->GeneratePlaneVec(), 1);
    publish_plane_triangle(marker_pub, plane_left->GeneratePlaneVec(), 2);
    publish_plane_triangle(marker_pub, plane_right->GeneratePlaneVec(), 3);
    r.sleep();
    publish_plane_triangle(marker_pub, plane_low->GeneratePlaneVec(), 0);
    publish_plane_triangle(marker_pub, plane_high->GeneratePlaneVec(), 1);
    publish_plane_triangle(marker_pub, plane_left->GeneratePlaneVec(), 2);
    publish_plane_triangle(marker_pub, plane_right->GeneratePlaneVec(), 3);
    r.sleep();

    // while (ros::ok()) {
    //     // marker_pub.publish(marker);
    //     publish_plane_triangle(marker_pub, plane->GeneratePlaneVec());
    //     std::cout << "Publishing triangle marker..." << std::endl;
    //     r.sleep();
    // }



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
    


    
    while(ros::ok())
    {
        // 四、获取末端位姿
        geometry_msgs::PoseStamped end_pose = move_group.getCurrentPose();


        // 五、计算平面和末端位姿的距离
        // double plane::distanceToPoint(const Vector3d& point, bool keep_flag)
        Eigen::Vector3d end_pose_eigen(end_pose.pose.position.x, end_pose.pose.position.y, end_pose.pose.position.z);
        double distance_low = plane_low->distanceToPoint(end_pose_eigen, true);
        double distance_high = plane_high->distanceToPoint(end_pose_eigen, true);
        double distance_left = plane_left->distanceToPoint(end_pose_eigen, true);
        double distance_right = plane_right->distanceToPoint(end_pose_eigen, true);

        std::cout<<"Distance：          ↑"<<std::endl;
        std::cout<<"                "<<distance_high<<std::endl;
        std::cout<<"       ←  "<<distance_left<<"    "<<distance_right<<" → "<<std::endl;
        std::cout<<"                "<<distance_low<<std::endl;
        std::cout<<"                    ↓"<<std::endl;

        std::cout<<std::endl;
        std::cout<<std::endl;
        
        r.sleep();
    }

    return 0;
}