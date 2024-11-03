//
// Created by robotlab on 24-11-1.
//
#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Matrix3d R;
    R << 0.99027311575, -0.0563912168853, 0.127197432684,
         0.0559317025402, 0.99840874929, 0.00718428790891,
         -0.127400160416, -3.82032566207e-08, 0.99185139972;

    // 定义平移向量 t
    Eigen::Vector3d t(0.0207669969149, -0.157939314235, 1.05993037165);

    // 构建 4x4 的变换矩阵 T
    Eigen::Matrix4d T_b_clink;
    T_b_clink << R(0, 0), R(0, 1), R(0, 2), t(0),
         R(1, 0), R(1, 1), R(1, 2), t(1),
         R(2, 0), R(2, 1), R(2, 2), t(2),
         0, 0, 0, 1;
    Eigen::Vector3d rpy = R.eulerAngles(2, 1, 0); // ZYX 顺序

    // 输出 RPY
    std::cout << "T_b_clink Roll: " << rpy(2)/M_PI*180 << "\n";
    std::cout << "T_b_clink Pitch: " << rpy(1)/M_PI*180 << "\n";
    std::cout << "T_b_clink Yaw: " << rpy(0)/M_PI*180 << "\n";


    Eigen::Matrix4d T_endlink_to_c;
    T_endlink_to_c <<   0, 0, 1, 0.02,
                        -1, 0, 0, -0.013,
                        0, -1, 0, 0.07, //实际为0.13(用于ros)，改为0.07(用于gpmp2 forwardKinematics)
                        0, 0, 0, 1;
    std::cout <<"T_b_c:" <<T_b_clink*T_endlink_to_c << std::endl;


    Eigen::Matrix4d T_b_c;
    T_b_c << 0.0563912168853, -0.127197432684, 0.99027311575, 0.0502093653373,
                         -0.99840874929, -0.00718428790891, 0.0559317025402, -0.169297093771,
                         3.82032566207e-08, -0.99185139972, -0.127400160416, 1.12681196692,
                         0, 0, 0, 1;

    // 世界坐标系中的 3D 点 (homogeneous coordinates)
    Eigen::Vector4d P_world(3, 0, 0.75, 1);

    // 计算相机位姿矩阵的逆 (T_world_to_camera)
    Eigen::Matrix4d T_world_to_camera = T_b_c.inverse();

    // 将 3D 点从世界坐标系转换到相机坐标系
    Eigen::Vector4d P_camera = T_world_to_camera * P_world;

    // 输出相机坐标系下的点
    std::cout << "3D point in camera coordinates: ("
              << P_camera(0) << ", " << P_camera(1) << ", " << P_camera(2) << ")" << std::endl;

    return 0;
}