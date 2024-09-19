#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <math.h>
#include <assert.h>
#include <iostream>
#include <string>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include "tf/transform_datatypes.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


geometry_msgs::Pose  get_link_pose(ros::ServiceClient& get_link_client, std::string link_name, std::string reference_name = "world"){
    gazebo_msgs::GetLinkState srv;
    srv.request.link_name = link_name;
    srv.request.reference_frame = reference_name;
    geometry_msgs::Pose pose;
    if (get_link_client.call(srv)) {
        // auto position = srv.response.link_state.pose.position;
        // auto orientation = srv.response.link_state.pose.orientation;
        // ROS_INFO("Position: x: %f, y: %f, z: %f", position.x, position.y, position.z);
        // ROS_INFO("Orientation: x: %f, y: %f, z: %f, w: %f", orientation.x, orientation.y, orientation.z, orientation.w);
        pose = srv.response.link_state.pose;
        return pose;

    } else {
        ROS_ERROR("Failed to call service get_link_state");
        return pose;
    }
}
void get_all_models_and_links_name(ros::ServiceClient& get_world_properties_client, ros::ServiceClient& get_model_properties_client){
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
geometry_msgs::Pose getPose(ros::ServiceClient& get_model_state_client, std::string name)
{
    geometry_msgs::Pose pose;
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = name; 
    get_model_state_client.call(srv);
    return srv.response.pose;
}

void setPose(ros::ServiceClient& set_model_state_client, std::string name, double x, double y, double z, tfScalar qw, tfScalar qx, tfScalar qy, tfScalar qz)
{
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_set_states_publisher");
 
    ros::NodeHandle n;
    ros::ServiceClient set_model_state_client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient get_model_state_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::ServiceClient get_link_sate_client = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    ros::ServiceClient get_world_properties_client = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
    ros::ServiceClient get_model_properties_client = n.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");

    double pitch = 28.0;
    
    double distance = 2.0;
    double height = 1.0;
    double angle = 0;
    double center_x = distance;
    double center_y = 0;
    
    // 获取gazeo中所有可用的model及其link
    get_all_models_and_links_name(get_world_properties_client, get_model_properties_client);

    // get物体的信息
    std::string object_model_name = "vase_violet"; 

    // get相机的信息
    std::string camera_model_name = "mobile_base"; 

    // get机械臂末端关节的信息
    std::string camera_joint_name = "mrobot::wam/wrist_palm_link"; 

    // set相机的信息
    setPose(set_model_state_client, camera_model_name, 0, 0, height, 0, 0, 0, 1);
    // camera_state.request.model_state.model_name = camera_model_name;
    // camera_state.request.model_state.pose.position.x = distance*cos(angle) - center_x;
    // camera_state.request.model_state.pose.position.y = distance*sin(angle) - center_y;
    // camera_state.request.model_state.pose.position.z = height;
    // camera_state.request.model_state.pose.orientation.w = q_init.w();
    // camera_state.request.model_state.pose.orientation.x = q_init.x();
    // camera_state.request.model_state.pose.orientation.y = q_init.y();
    // camera_state.request.model_state.pose.orientation.z = q_init.z();
    
   
    // 初始矩阵：没用到
    // Eigen::Quaterniond q_world_to_camerabody;
    // tf::Quaternion q_init = tf::createQuaternionFromRPY(0, pitch*M_PI/180.0, angle+M_PI);
    // q_world_to_camerabody.w() = q_init.w();
    // q_world_to_camerabody.x() = q_init.x();
    // q_world_to_camerabody.y() = q_init.y();
    // q_world_to_camerabody.z() = q_init.z();
    // Eigen::Matrix3d R_world_to_camerabody;//声明一个Eigen类的3*3的旋转矩阵
    // R_world_to_camerabody = q_world_to_camerabody.normalized().toRotationMatrix(); //四元数转为旋转矩阵--先归一化再转为旋转矩阵
    // Eigen::Matrix3d R_camerabody_to_cam;
    // R_camerabody_to_cam<< 0, 0, 1, 
    //                 -1, 0, 0, 
    //                 0, -1, 0;
    // Eigen::Quaterniond q_world_to_camera = Eigen::Quaterniond ((R_world_to_camerabody * R_camerabody_to_cam )); //.inverse()  R_z_f90 * R_x_f90
    // std::cout<<"camera pose"
    //             <<",qw:"<<q_world_to_camera.w()
    //             <<",qx:"<<q_world_to_camera.x()
    //             <<",qy:"<<q_world_to_camera.y()
    //             <<",qz:"<<q_world_to_camera.z()
    //             <<std::endl<<std::endl;

    std::cout<<"INIT POSE ----------------------"<<std::endl;
    std::cout<<"按任意键继续 ----------------------"<<std::endl;
    double cout = 0;

    ros::Rate loop_rate(0.5);
    std::cin.get();
    double init_dis = 1.0;
    while(ros::ok()) {

        // get物体的信息
        auto object_pose = getPose(get_model_state_client, object_model_name);
        double object_x = object_pose.position.x;
        double object_y = object_pose.position.y;
        double object_z = object_pose.position.z;
        std::cout<<object_model_name
                <<", x:"<<object_x
                <<", y:"<<object_y
                <<", z:"<<object_z
                <<std::endl;

        // get相机的信息
        // auto camera_pose = getPose(get_client, camera_model_name);
        // double camera_x = camera_pose.position.x;
        // double camera_y = camera_pose.position.y;
        // double camera_z = camera_pose.position.z;
        // std::cout<<"/wam/wrist_palm_link pose x:"<<camera_x
        //         <<",y:"<<camera_y
        //         <<",z:"<<camera_z
        //         <<std::endl;

        // 获取相机末端关机的位姿，用以替代相机的目标位姿
        auto joint_pose = get_link_pose(get_link_sate_client, camera_joint_name);
        double camera_x = joint_pose.position.x;
        double camera_y = joint_pose.position.y;
        double camera_z = joint_pose.position.z + 0.05;
        std::cout<<camera_joint_name
                <<", x:"<<camera_x
                <<", y:"<<camera_y
                <<", z:"<<camera_z
                <<std::endl;

        
        // 计算相机的朝向
        double deltaX = object_x - camera_x;
        double deltaY = object_y - camera_y;
        double delta_yaw = std::atan2(deltaY, deltaX);
        double deltaZ = object_z - camera_z;
        double delta_pitch = std::atan2(deltaZ, std::sqrt(deltaX*deltaX + deltaY*deltaY));
        

        tf::Quaternion q = tf::createQuaternionFromRPY(0, -1*delta_pitch, delta_yaw);
        int type = 0;
        if(type==1)
            setPose(set_model_state_client, camera_model_name, camera_x, camera_y, camera_z, q.w(), q.x(), q.y(), q.z());
        else if(type==0){
            double scale = 1.0/std::sqrt(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);
            camera_x =  (camera_x-object_x)*scale + object_x; 
            camera_y =  (camera_y-object_y)*scale + object_y; 
            camera_z =  (camera_z-object_z)*scale + object_z; 
            setPose(set_model_state_client, camera_model_name, camera_x, camera_y, camera_z, q.w(), q.x(), q.y(), q.z());

        }

        // std::cout<<"pose x:"<<camera_state.request.model_state.pose.position.x
        //         <<",y:"<<camera_state.request.model_state.pose.position.y
        //         <<",z:"<<camera_state.request.model_state.pose.position.z
        //         <<",qw:"<<q.w()
        //         <<",qx:"<<q.x()
        //         <<",qy:"<<q.y()
        //         <<",qz:"<<q.z()
        //         <<std::endl;


        ROS_INFO("call service");

        cout += 0.01;
        ros::spinOnce();
        loop_rate.sleep(); // is stuck on loop rate, reboot roscore
        //ROS_INFO("loop_rate sleep over");
    }
    ROS_INFO("end service");
    return 0;
}
