#ifndef VIEW_PLANNING_OBJECTS_H
#define VIEW_PLANNING_OBJECTS_H

#include "ConverterTools.h"
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/eigen.hpp>

class SdfObject{
    
    public:
        double x,y,z;
        double length,width,height;
        double roll,pitch,yaw;

        cv::Mat pose_mat = cv::Mat::eye(4, 4, CV_32F);
 
    public:
        SdfObject(  double x_, double y_, double z_, 
                    double l_, double w_, double h_, 
                    double ro_, double po_, double yo_ ){
            // variables
            x = x_;
            y = y_;
            z = z_;
            length = l_;
            width = w_;
            height = h_;
            roll = ro_;
            pitch = po_;
            yaw = yo_;

            Eigen::Matrix3d rotation_matrix;
            rotation_matrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
                            // * Eigen::AngleAxisd(ob.pitch, Eigen::Vector3d::UnitY())
                            // * Eigen::AngleAxisd(ob.roll, Eigen::Vector3d::UnitX());

            // 坐标系变换矩阵
            Eigen::Matrix4d T;
            T   <<  1, 0, 0, x,  // 假设T为平移矩阵，将A坐标系原点(1, 1, 1)平移到B坐标系原点
                    0, 1, 0, y,
                    0, 0, 1, z,
                    0, 0, 0, 1;
            T.block<3,3>(0,0) = rotation_matrix;

            // 将 Eigen 矩阵的数据复制到 cv::Mat 中
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    pose_mat.at<double>(i, j) = T(i, j);
                }
            }

        }
};

#endif // VIEW_PLANNING_OBJECTS_H