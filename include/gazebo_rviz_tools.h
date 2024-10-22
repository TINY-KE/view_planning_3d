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
#include "MapObject.h"
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

geometry_msgs::TransformStamped getTFTransform(std::string parent_name, std::string source_name, tf::Transform& out) {
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped = tf_buffer.lookupTransform(parent_name, source_name, ros::Time(0), ros::Duration(1.0));

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

    // // 输出相机在 odom 坐标系下的位姿
    // ROS_INFO_STREAM("Pose of camera_rgb_frame in odom: \n" << end_pose);
    return transform_stamped;
}



class Visualize_Tools{
    public:
        Visualize_Tools(ros::NodeHandle& nh, std::string default_frame ):default_frame_(default_frame){
            candidate_quaterniond_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                "candidate_pose_quaterniond", 1);
            pub_field = nh.advertise<visualization_msgs::Marker>("field", 1);
            pub_sdf = nh.advertise<visualization_msgs::Marker>("sdf", 1);
            joint_values_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_values_gpmp", 10);
            candidate_pub = nh.advertise<visualization_msgs::Marker>("/candidate", 10);
            publisher_object = nh.advertise<visualization_msgs::Marker>("object", 1000);
            publisher_ellipsoid = nh.advertise<visualization_msgs::Marker>("ellipsoid", 1000);
            point_pub = nh.advertise<visualization_msgs::Marker>("/point", 10);
            bbox_plane_pub = nh.advertise<visualization_msgs::Marker>("bbox_plane", 1);
            normal_plane_pub = nh.advertise<visualization_msgs::Marker>("normal_plane", 1);
        }

        void visualize_geometry_pose(geometry_msgs::Pose pose, std::string frame_id = "world",  double id_num = 1,  std::string name = "no-name", bool output = false);
        // void visualize_MapObject(SdfObject& ob, std::string frame_id);
        void visualize_ellipsoid(MapObject* ob, std::string frame_id, double id);
        void visualize_ellipsoid(double x, double y, double z, double a, double b, double c, double roll, double pitch, double yaw, std::string frame_id, double id);
        void visualize_point(Eigen::Vector3d& p , std::string frame_id, double id);
        void visualize_plane_triangle_bypoint(std::vector<geometry_msgs::Point>& points, int id, std::string frame_id = "world");
        void visualize_plane_rectangle(Eigen::Vector4d plane_param, int id, std::string frame_id = "world");


        void Run();
        
    private:
        ros::Publisher candidate_quaterniond_pub;
        ros::Publisher pub_field;
        ros::Publisher pub_sdf;
        ros::Publisher joint_values_pub;
        ros::Publisher candidate_pub;
        ros::Publisher publisher_object;
        ros::Publisher publisher_ellipsoid;
        ros::Publisher point_pub;
        ros::Publisher bbox_plane_pub;
        ros::Publisher normal_plane_pub;
        // Visualize_Arm_Tools arm_pub;

        std::string default_frame_ = "wam/base_link";

    public:
        std::vector<std::vector<geometry_msgs::Point>> BboxPlanesTrianglePointsInWorld;

        std::vector<MapObject*> MapObjects;
        // std::vector<g2o::plane*> MapPlanes;
        std::vector<Eigen::Vector4d> MapPlaneNormals;

};


void Visualize_Tools::Run()
{   
    ros::Rate r(50);


    while(1){

        for(int i=0; i<MapObjects.size(); i++ ){
            visualize_ellipsoid( MapObjects[i], default_frame_, i);
        }

        for(int i=0; i<BboxPlanesTrianglePointsInWorld.size(); i++ ){
            visualize_plane_triangle_bypoint(BboxPlanesTrianglePointsInWorld[i], i, default_frame_);
            // std::cout << "Publishing triangle marker..." << std::endl;
        }

        for(int i=0; i<MapPlaneNormals.size(); i++) {
            visualize_plane_rectangle(MapPlaneNormals[i], i, default_frame_);
        }
        r.sleep();

    }
}

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
void  Visualize_Tools::visualize_plane_rectangle(Eigen::Vector4d plane_param, int id, std::string frame_id) {
    double a = plane_param[0];
    double b = plane_param[1];
    double c = plane_param[2];
    double d = plane_param[3];

    // 创建一个Marker消息
    // 创建一个 Marker 消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;  // 使用世界坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "rectangle";
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

    double size = 2.0;
    geometry_msgs::Point p1 = generatePointOnPlane(a, b, c, d, size, size);
    geometry_msgs::Point p2 = generatePointOnPlane(a, b, c, d, size, -1*size);
    geometry_msgs::Point p3 = generatePointOnPlane(a, b, c, d, -1*size, size);
    geometry_msgs::Point p4 = generatePointOnPlane(a, b, c, d, -1*size, -1*size);
    // 生成平面上的点
    std::vector<geometry_msgs::Point> points;
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);

    points.push_back(p4);
    points.push_back(p2);
    points.push_back(p3);

    marker.points = points;

    // 发布Marker消息
    normal_plane_pub.publish(marker);
}


// 以三角的形式显示平面
void Visualize_Tools::visualize_plane_triangle_bypoint(std::vector<geometry_msgs::Point>& points, int id, std::string frame_id) {
    
    // 创建一个 Marker 消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // 使用世界坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "triangle";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;  // 使用三角形列表
    marker.action = visualization_msgs::Marker::ADD;

    // 设置颜色和透明度
    marker.color.a = 0.12;  // 不透明
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
    bbox_plane_pub.publish(marker);
}


void Visualize_Tools::visualize_ellipsoid(MapObject* ob, std::string frame_id, double id){
    // 创建 Marker 消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "ellipsoid_visualization";
    marker.id = id;  // 唯一的 ID
    marker.type = visualization_msgs::Marker::SPHERE;  // 使用 SPHERE 类型表示椭球体
    marker.action = visualization_msgs::Marker::ADD;

    // 设置椭球体的位置
    marker.pose.position.x = ob->mCuboid3D.cuboidCenter[0];
    marker.pose.position.y = ob->mCuboid3D.cuboidCenter[1];
    marker.pose.position.z = ob->mCuboid3D.cuboidCenter[2];

    // 椭球体的比例尺寸，a, b, c 分别为 x, y, z 方向的轴长度
    marker.scale.x = ob->mCuboid3D.lenth;  // x 轴长度
    marker.scale.y = ob->mCuboid3D.width;  // y 轴长度
    marker.scale.z = ob->mCuboid3D.height;  // z 轴长度

    // 设置颜色 (RGBA)
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;  // 绿色
    marker.color.b = 0.0f;
    marker.color.a = 0.5f;  // 透明度

    // 创建四元数并设置旋转（Roll, Pitch, Yaw）
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);  // 设置旋转角度 (弧度)

    // 将四元数转换为 geometry_msgs::Quaternion 并设置到 marker 中
    marker.pose.orientation = tf2::toMsg(quat);
    // 将 Marker 发布到 ROS 主题
    publisher_ellipsoid.publish(marker);
}

void Visualize_Tools::visualize_ellipsoid(double x, double y, double z, double a, double b, double c, double roll, double pitch, double yaw, std::string frame_id, double id) {

    // 创建 Marker 消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "ellipsoid_visualization";
    marker.id = id;  // 唯一的 ID
    marker.type = visualization_msgs::Marker::SPHERE;  // 使用 SPHERE 类型表示椭球体
    marker.action = visualization_msgs::Marker::ADD;

    // 设置椭球体的位置
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    // 椭球体的比例尺寸，a, b, c 分别为 x, y, z 方向的轴长度
    marker.scale.x = a;  // x 轴长度
    marker.scale.y = b;  // y 轴长度
    marker.scale.z = c;  // z 轴长度

    // 设置颜色 (RGBA)
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;  // 绿色
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;  // 透明度

    // 创建四元数并设置旋转（Roll, Pitch, Yaw）
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);  // 设置旋转角度 (弧度)

    // 将四元数转换为 geometry_msgs::Quaternion 并设置到 marker 中
    marker.pose.orientation = tf2::toMsg(quat);
    // 将 Marker 发布到 ROS 主题
    publisher_ellipsoid.publish(marker);
}
// 

#endif //VIEW_PLANNING_GAZEBO_TOOLS_H