#ifndef VIEW_PLANNING_GAZEBO_TOOLS_H
#define VIEW_PLANNING_GAZEBO_TOOLS_H

#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Float64MultiArray.h"
#include "Object.h"
#include "ConverterTools.h"

geometry_msgs::Pose  get_link_pose(ros::NodeHandle& n, std::string link_name, std::string reference_name = "world", bool output = false){
    ros::ServiceClient get_link_sate_client = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    gazebo_msgs::GetLinkState srv;
    srv.request.link_name = link_name;
    srv.request.reference_frame = reference_name;
    geometry_msgs::Pose pose;
    if (get_link_sate_client.call(srv)) {
        // auto position = srv.response.link_state.pose.position;
        // auto orientation = srv.response.link_state.pose.orientation;
        // ROS_INFO("Position: x: %f, y: %f, z: %f", position.x, position.y, position.z);
        // ROS_INFO("Orientation: x: %f, y: %f, z: %f, w: %f", orientation.x, orientation.y, orientation.z, orientation.w);
        pose = srv.response.link_state.pose;
        if(output){
         ROS_INFO("Pose of [%s]: position(x: %f, y: %f, z: %f), orientation(x: %f, y: %f, z: %f, w: %f)", 
             link_name.c_str(), 
             pose.position.x, pose.position.y, pose.position.z,
             pose.orientation.x, pose.orientation.y, 
             pose.orientation.z, pose.orientation.w);
        }
        return pose;

    } else {
        ROS_ERROR("Failed to call service get_link_state");
        return pose;
    }
}

void get_all_models_and_links_name(ros::NodeHandle& n){
    ros::ServiceClient get_world_properties_client = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
    ros::ServiceClient get_model_properties_client = n.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");

    ros::service::waitForService("/gazebo/get_world_properties");
    
    gazebo_msgs::GetWorldProperties world_srv;
    if (get_world_properties_client.call(world_srv)) {
        for (const auto& model_name : world_srv.response.model_names) {
            gazebo_msgs::GetModelProperties model_srv;
            model_srv.request.model_name = model_name;
            if (get_model_properties_client.call(model_srv)) {
                ROS_INFO("Model: %s", model_name.c_str());
                for (const auto& link_name : model_srv.response.body_names) {
                    ROS_INFO("  Link: %s", link_name.c_str());
                }
            } else {
                ROS_ERROR("Failed to get properties for model: %s", model_name.c_str());
            }
        }
    } else {
        ROS_ERROR("Failed to call service get_world_properties");
    }
}
geometry_msgs::Pose getPose(ros::NodeHandle& n, std::string name, bool output = false)
{
    ros::ServiceClient get_model_state_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    geometry_msgs::Pose pose;
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = name; 
    get_model_state_client.call(srv);
    if(output){
         ROS_INFO("Pose of [%s]: position(x: %f, y: %f, z: %f), orientation(x: %f, y: %f, z: %f, w: %f)", 
             name.c_str(), 
             srv.response.pose.position.x, srv.response.pose.position.y, srv.response.pose.position.z,
             srv.response.pose.orientation.x, srv.response.pose.orientation.y, 
             srv.response.pose.orientation.z, srv.response.pose.orientation.w);
    }
    return srv.response.pose;
}

void setPose(ros::NodeHandle& n, std::string name, double x, double y, double z, tfScalar qw, tfScalar qx, tfScalar qy, tfScalar qz)
{
    ros::ServiceClient set_model_state_client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    gazebo_msgs::SetModelState object_state;
    
    object_state.request.model_state.model_name = name;

    
    object_state.request.model_state.pose.position.x = x;
    object_state.request.model_state.pose.position.y = y;
    object_state.request.model_state.pose.position.z = z;

    object_state.request.model_state.pose.orientation.w = qw;
    object_state.request.model_state.pose.orientation.x = qx;
    object_state.request.model_state.pose.orientation.y = qy;
    object_state.request.model_state.pose.orientation.z = qz;
    
    object_state.request.model_state.twist.linear.x = 0.0;
    object_state.request.model_state.twist.linear.y = 0.0;
    object_state.request.model_state.twist.linear.z = 0.0;
    object_state.request.model_state.twist.angular.x = 0.0;
    object_state.request.model_state.twist.angular.y = 0.0;
    object_state.request.model_state.twist.angular.z = 0.0;

    object_state.request.model_state.reference_frame = "world";

    set_model_state_client.call(object_state);
}



class Visualize_Tools{
    public:
        Visualize_Tools(ros::NodeHandle& nh){
            candidate_quaterniond_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("candidate_pose_quaterniond", 1);
            pub_field = nh.advertise<visualization_msgs::Marker>("field", 1);
            pub_sdf = nh.advertise<visualization_msgs::Marker>("sdf", 1);
            joint_values_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_values_gpmp", 10);
            candidate_pub = nh.advertise<visualization_msgs::Marker>("/candidate", 10);
            publisher_object = nh.advertise<visualization_msgs::Marker>("object", 1000);
            point_pub = nh.advertise<visualization_msgs::Marker>("/point", 10);
        }
        void visualize_geometry_pose(geometry_msgs::Pose pose, std::string frame_id = "world",  double id_num = 1,  std::string name = "no-name", bool output = false);
        void visualize_object(SdfObject& ob, std::string frame_id);
        void visualize_point(Eigen::Vector3d& p , std::string frame_id, double id);

    private:
        ros::Publisher candidate_quaterniond_pub;
        ros::Publisher pub_field;
        ros::Publisher pub_sdf;
        ros::Publisher joint_values_pub;
        ros::Publisher candidate_pub;
        ros::Publisher publisher_object;
        ros::Publisher point_pub;
};

void Visualize_Tools::visualize_point(Eigen::Vector3d& p , std::string frame_id, double id){
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "points";
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker
        marker.pose.position.x = p.x();
        marker.pose.position.y = p.y();
        marker.pose.position.z = p.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker (1x1x1 here means 1m on each side)
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        // marker.lifetime = ros::Duration();

        point_pub.publish(marker);
}


void Visualize_Tools::visualize_object(SdfObject& ob, std::string frame_id){
    //publish rviz 
        visualization_msgs::Marker marker;
        marker.id = 0;//++object_id_init;//object_id_init + i;
        float mObject_Duration=1;
        // marker.lifetime = ros::Duration(mObject_Duration);
        marker.header.frame_id= frame_id;
        marker.header.stamp=ros::Time::now();
        marker.type = visualization_msgs::Marker::LINE_LIST; //LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = 255.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
        marker.scale.x = 0.01;
        //     8------7
        //    /|     /|
        //   / |    / |
        //  5------6  |
        //  |  4---|--3
        //  | /    | /        12为x轴，，14为y轴，15为z轴
        //  1------2
        // 12 lenth ：corner_2[0] - corner_1[0]
        // 14 width ：corner_2[1] - corner_3[1]
        // 15 height：corner_2[2] - corner_6[2]
        double length_half = ob.length/2.0;
        double width_half = ob.width/2.0;
        double height_half = ob.height/2.0;

        geometry_msgs::Point p1;   p1.x = -1*length_half;   p1.y = -1*width_half;      p1.z = 0; //-1*depth_half; 
        geometry_msgs::Point p2;   p2.x = length_half;   p2.y = -1*width_half;   p2.z = 0; //-1*depth_half; 
        geometry_msgs::Point p3;   p3.x = length_half;      p3.y = width_half;   p3.z = 0; //-1*depth_half; 
        geometry_msgs::Point p4;   p4.x = -1*length_half;      p4.y = width_half;      p4.z = 0; //-1*depth_half; 
        geometry_msgs::Point p5;   p5.x = -1*length_half;   p5.y = -1*width_half;      p5.z = 2*height_half; 
        geometry_msgs::Point p6;   p6.x = length_half;   p6.y = -1*width_half;   p6.z = 2*height_half; 
        geometry_msgs::Point p7;   p7.x = length_half;      p7.y = width_half;   p7.z = 2*height_half; 
        geometry_msgs::Point p8;   p8.x = -1*length_half;      p8.y = width_half;      p8.z = 2*height_half; 
        
        
        
        marker.points.push_back(corner_to_marker(p1));
        marker.points.push_back(corner_to_marker(p2));
        marker.points.push_back(corner_to_marker(p2));
        marker.points.push_back(corner_to_marker(p3));
        marker.points.push_back(corner_to_marker(p3));
        marker.points.push_back(corner_to_marker(p4));
        marker.points.push_back(corner_to_marker(p4));
        marker.points.push_back(corner_to_marker(p1));

        marker.points.push_back(corner_to_marker(p5));
        marker.points.push_back(corner_to_marker(p1));
        marker.points.push_back(corner_to_marker(p6));
        marker.points.push_back(corner_to_marker(p2));
        marker.points.push_back(corner_to_marker(p7));
        marker.points.push_back(corner_to_marker(p3));
        marker.points.push_back(corner_to_marker(p8));
        marker.points.push_back(corner_to_marker(p4));

        marker.points.push_back(corner_to_marker(p5));
        marker.points.push_back(corner_to_marker(p6));
        marker.points.push_back(corner_to_marker(p6));
        marker.points.push_back(corner_to_marker(p7));
        marker.points.push_back(corner_to_marker(p7));
        marker.points.push_back(corner_to_marker(p8));
        marker.points.push_back(corner_to_marker(p8));
        marker.points.push_back(corner_to_marker(p5));

        publisher_object.publish(marker);
        std::cout << "publish Object rviz"<< std::endl;
}

// void Visualize_Tools::visualize_geometry_pose(geometry_msgs::Pose pose, std::string frame_id = "world",  double id_num = 1,  std::string name = "no-name", bool output = false){
void Visualize_Tools::visualize_geometry_pose(geometry_msgs::Pose pose, std::string frame_id,  double id_num,  std::string name, bool output){
    
    // ros::Publisher candidate_quaterniond_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("output", 1);
    
    

    // visualization_msgs::Marker marker;
    // marker.header.frame_id = frame_id;  // 假设您的世界坐标系为"world"
    // marker.id = id_num;
    // marker.type = visualization_msgs::Marker::ARROW;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.scale.x = 0.03;  // 箭头的尺寸
    // marker.scale.y = 0.03;
    // // marker.scale.z = 0.02;

    // // 设置箭头的起点和终点
    // geometry_msgs::Point start = eigen_to_point(candidates[i].start);
    // marker.points.push_back(start);
    // Eigen::Vector3d end_new = (candidates[i].end - candidates[i].start).normalized() * 0.2 + candidates[i].start;
    // geometry_msgs::Point end = eigen_to_point(end_new);
    // marker.points.push_back(end);

    // // 设置箭头的颜色
    // marker.color.a = 1.0;  // 不透明度
    // marker.color.r = 0.0;  // 红色分量
    // marker.color.g = 0.0;  // 绿色分量
    // marker.color.b = 1.0;  // 蓝色分量
    
    // candidate_pub.publish(marker);
    
    geometry_msgs::PoseWithCovarianceStamped target;
    target.header.frame_id = frame_id;	//设置了消息的头部信息
    //通过将四元数的分量（x、y、z、w）设置为transDockPos变量中存储的旋转四元数分量。这些分量描述了箭头方向的旋转。
    target.pose.pose.orientation.x = pose.orientation.x;
    target.pose.pose.orientation.y = pose.orientation.y;
    target.pose.pose.orientation.z = pose.orientation.z;
    target.pose.pose.orientation.w = pose.orientation.w;
    target.pose.pose.position.x = pose.position.x;
    target.pose.pose.position.y = pose.position.y;
    target.pose.pose.position.z = pose.position.z;

    if(output){
        ROS_INFO("Pose of [%s]: position(x: %f, y: %f, z: %f), orientation(x: %f, y: %f, z: %f, w: %f)", 
            name.c_str(),
            target.pose.pose.position.x, target.pose.pose.position.y, target.pose.pose.position.z,
            target.pose.pose.orientation.x, target.pose.pose.orientation.y, 
            target.pose.pose.orientation.z, target.pose.pose.orientation.w);
    }
    
    candidate_quaterniond_pub.publish(target);
}


// 函数：根据平面方程生成平面上的一个点
geometry_msgs::Point generatePointOnPlane(double a, double b, double c, double d, double x, double y) {
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    
    // 根据平面方程 ax + by + cz + d = 0 计算 z
    if (c != 0) {
        point.z = -(a * x + b * y + d) / c;
    } else {
        point.z = 0.0;  // 如果 c == 0，默认为 z = 0
    }

    return point;
}

// 以三角的形式显示平面
void publish_plane_triangle(ros::Publisher& marker_pub, Eigen::Vector4d plane_param, int id, std::string frame_id = "world") {
    double a = plane_param[0];
    double b = plane_param[1];
    double c = plane_param[2];
    double d = plane_param[3];

    // 创建一个Marker消息
    // 创建一个 Marker 消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // 使用世界坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "triangle";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;  // 使用三角形列表
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


    // 生成平面上的点
    std::vector<geometry_msgs::Point> points;
    points.push_back(generatePointOnPlane(a, b, c, d, 0, 0));
    points.push_back(generatePointOnPlane(a, b, c, d, 3.0, 0));
    points.push_back(generatePointOnPlane(a, b, c, d, 0, 3.0));
    marker.points = points;

    // 发布Marker消息
    marker_pub.publish(marker);
}


// 以三角的形式显示平面
void publish_plane_triangle_bypoint(ros::Publisher& marker_pub, std::vector<geometry_msgs::Point>& points, int id, std::string frame_id = "world") {
    
    // 创建一个 Marker 消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // 使用世界坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "triangle";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;  // 使用三角形列表
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


    // 生成平面上的点
    points.push_back(points[0]);
    points.push_back(points[1]);
    points.push_back(points[2]);
    marker.points = points;

    // 发布Marker消息
    marker_pub.publish(marker);
}

// 

#endif //VIEW_PLANNING_GAZEBO_TOOLS_H