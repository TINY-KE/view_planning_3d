#ifndef VIEW_PLANNING_CONVERTERTOOLS_H
#define VIEW_PLANNING_CONVERTERTOOLS_H


#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include <tf2/convert.h>
#include <geometry_msgs/Pose.h>
#include <tf2/transform_datatypes.h> // Include if needed

// #include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
// #include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

//eigen cv的convert
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/core/eigen.hpp>


tf::Quaternion RPY_to_quaternion(double roll, double pitch, double yaw, tf2::Quaternion& q){
    
    // 两者结果一样
    
    // 方法1
    tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);

    // 方法2
    // Eigen::Quaterniond q;
    // // 计算四元数
    // q = Eigen::AngleAxisd(-1*delta_yaw/180*M_PI, Eigen::Vector3d::UnitZ()) *
    //     Eigen::AngleAxisd(-1*delta_pitch/180*M_PI, Eigen::Vector3d::UnitY()) *
    //     Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
}


#endif //VIEW_PLANNING_CONVERTERTOOLS_H