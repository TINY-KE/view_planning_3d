#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include "gazebo_rviz_tools.h"

#include <vector>
// 定义常量 PI
const double PI = 3.14159265358979323846;


struct Point {
    double x;
    double y;
};

struct Ellipse {
    double a; // 椭圆长轴
    double b; // 椭圆短轴
    double xc; // 椭圆中心的 x 坐标
    double yc; // 椭圆中心的 y 坐标
};

// 计算两个向量的点积
double dotProduct(const Point& v1, const Point& v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

// 计算向量的模长
double magnitude(const Point& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}


// 计算椭圆上一点的切线与椭圆中心连线的夹角
double calculateAngle(const Ellipse& ellipse, const Point& A) {
    // 计算 AC 向量
    Point AC = { A.x - ellipse.xc, A.y - ellipse.yc };
    
    // 计算法线方向向量 (n)
    Point n = { (A.x - ellipse.xc) / (ellipse.a * ellipse.a),
                (A.y - ellipse.yc) / (ellipse.b * ellipse.b) };
    
    // 切线方向向量是法线方向的垂直向量
    Point t = { -n.y, n.x };
    
    // 计算 AC 向量和切线方向向量 t 之间的夹角
    double dot = dotProduct(AC, t);
    double magAC = magnitude(AC);
    double magT = magnitude(t);
    
    // 计算夹角的余弦值
    double cosTheta = dot / (magAC * magT);
    
    // 通过 acos 计算夹角，单位为弧度
    double theta = std::acos(cosTheta);
    
    // 将角度转换为度数
    return theta * 180.0 / M_PI;
}

// 计算射线与椭圆的交点
Point calculateIntersection(const Ellipse& ellipse, double theta_rad) {

    double a = ellipse.a ;
    double b = ellipse.b;
    double xc = ellipse.xc;
    double yc = ellipse.yc;
    // 如果射线垂直（theta = 90 或 270 度）
    if (theta_rad == M_PI_2 || theta_rad == M_PI_2 * 3) {
        std::cout << "射线垂直于x轴，交点为: (0, " << ((theta_rad == M_PI_2) ? b : -b) << ")\n";
        return Point{ 0+xc, ((theta_rad == M_PI_2) ? b : -b )+yc};
    }


    // 计算 tan(theta)
    double tan_theta = std::tan(theta_rad);

    // 计算 x 坐标
    double x = std::sqrt((a * a * b * b) / (b * b + a * a * tan_theta * tan_theta));

    // 计算 y 坐标
    double y = x * tan_theta;

    // 根据 theta 的范围判断符号
    if ((theta_rad > M_PI_2 && theta_rad < M_PI_2 * 3)) {
        x = -x;
        y = -y;
    }

    // 输出结果
    std::cout << "角度为：" << theta_rad <<
    "时， 射线与椭圆的交点为: (" << x+xc << ", " << y+yc << ")\n";

    return Point{ x+xc, y+yc };
}

// 计算椭圆上一点的切线与 x 轴的夹角
double calculateAngleWithXAxis(const Ellipse& ellipse, const Point& A) {
    // 计算法线方向向量 (n)
    Point n = { (A.x - ellipse.xc) / (ellipse.a * ellipse.a),
                (A.y - ellipse.yc) / (ellipse.b * ellipse.b) };
    
    // 切线方向向量是法线方向的垂直向量
    Point t = { -n.y, n.x };
    
    // 计算切线与 x 轴的夹角
    double angleWithXAxis = std::atan2(t.y, t.x); // 使用 atan2 得到角度
    
    // 转换为度数
    double angleInDegrees = angleWithXAxis * 180.0 / M_PI;

    // 调整角度
    angleInDegrees -= 180.0; // 减去 180 度

    // 确保角度在 0 到 360 度之间
    if (angleInDegrees < 0) {
        angleInDegrees += 360.0;
    }

    return angleInDegrees; // 转换为度数
}


int main(int argc, char** argv){

    double a, b, theta;

    // 输入椭圆的长轴和短轴
    std::cout << "请输入椭圆的长轴 a: ";
    std::cin >> a;
    std::cout << "请输入椭圆的短轴 b: ";
    std::cin >> b;

    std::vector<Point> InterS;
    
    Ellipse ellipse = { a, b, 2, 0 };

    for(int i=18; i>0; i--){
        double angle = i*10/180.0*M_PI ;
        // 计算射线与椭圆的交点
        Point intersection = calculateIntersection(ellipse, angle);
        InterS.push_back(intersection);
        // 计算切线与 AC 之间的夹角
        double yaw = calculateAngle(ellipse, intersection);
        std::cout<<"切线夹角: "<<yaw<<std::endl;
        double angleWithXAxis = calculateAngleWithXAxis(ellipse, intersection);
        std::cout<<"切线与x轴的夹角: "<<angleWithXAxis<<std::endl;
    }
    

    ros::init(argc, argv, "ellipse_debug", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    Visualize_Tools vis_tools(nh);
    
    ros::Rate loop_rate(1);  // 设置发布频率

    while(ros::ok()){
        for(int i=0; i<InterS.size(); i++){
            Eigen::Vector3d point(InterS[i].x, InterS[i].y, 0);
            vis_tools.visualize_point(point, "world", i);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }


    return 0;
}
