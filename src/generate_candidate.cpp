#include <cmath>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

// #include <active_slam_msg/object_map>

// double M_PI = 3.1415926;

//define a new class called object_cluster
class object_cluster{
    public:
        //define a default constructor
        object_cluster(){};

    public:
        //define the variables for the object cluster
        double x,y,z; //the x,y,z position of the object cluster
        double l,w,h; //the length, width, and height of the object cluster
        double ro,po,yo; //the roll, pitch, and yaw of the object cluster
};
// 计算向量夹角的函数
double angle_calculate(geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2){
    double v1_norm = sqrt(v1.x*v1.x + v1.y*v1.y + v1.z*v1.z);
    double v2_norm = sqrt(v2.x*v2.x + v2.y*v2.y + v2.z*v2.z);
    double v1_dot_v2 = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
    double cos_theta = v1_dot_v2 / (v1_norm * v2_norm);
    double theta = acos(cos_theta);
    return theta;
}


class object{
    
    public:
        double x,y,z;
        double l,w,h;
        double ro,po,yo;

 
    public:
        object(  double x_, double y_, double z_, 
                    double l_, double w_, double h_, 
                    double ro_, double po_, double yo_ ){
            // variables
            x = x_;
            y = y_;
            z = z_;
            l = l_;
            w = w_;
            h = h_;
            ro = ro_;
            po = po_;
            yo = yo_;
        }
};

class candidate{
    public:
        double x,y,z;
        double roll,pitch,yaw;
        Eigen::Vector3d start;
        Eigen::Vector3d end;
        Eigen::Quaterniond q;
    public:
        // 构造函数
        candidate(){
            x = 0;
            y = 0;
            z = 0;
            roll = 0;
            pitch = 0;
            yaw = 0;
        }
        //This code creates a candidate trajectory for the robot to follow.
        candidate(Eigen::Vector3d start_, Eigen::Vector3d end_){
            start = start_;
            end = end_;
            
            Eigen::Vector3d direction = end - start;
            direction.normalize();

            // v1
            // Eigen::Vector3d v0(1,0,0);
            // Eigen::Vector3d v1(0,1,0);
            // Eigen::Vector3d v2(0,0,1);
            // double theta = acos(v.dot(v0));
            // Eigen::Vector3d axis = v.cross(v0);
            // q = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
            // v2:
            Eigen::Quaterniond quaternion;
            quaternion.setFromTwoVectors(Eigen::Vector3d::UnitX(), direction);
            q = quaternion;
            // q = Eigen::Quaterniond(0.0, direction.x(), direction.y(), direction.z());

            // Eigen::Vector3d v1_ = q * v1;
            // Eigen::Vector3d v2_ = q * v2;
            // double theta1 = acos(v1_.dot(v1));
            // double theta2 = acos(v2_.dot(v2));
            // if(theta1 > theta2){
            //     theta = theta2;
            // }
            // else{
            //     theta = theta1;
            // }
            // if(axis(2) < 0){
            //     theta = -theta;
            // }
            
            x = start(0);
            y = start(1);
            z = start(2);
        }

};

// Eigen::Vector3d 到 geometry_msgs::Point 的转换函数
geometry_msgs::Point eigen_to_point(Eigen::Vector3d &v){
    geometry_msgs::Point p;
    p.x = v(0);
    p.y = v(1);
    p.z = v(2);
    return p;
}

int main(int argc, char** argv){


    ros::init(argc, argv, "vector_display_node");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("vector_marker", 10);
    ros::Publisher publisher_mam_rviz = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/local_nbv", 1000);
    


    object ob(  0.68888, -0.317092, 0.467195,
                0.368233, 0.400311, 0.245264,
                0.000000, 0.000000, 0.000000   );
    std::vector<candidate> candidates;


    int i = 0;


    // 60度。未来如果60度达不到，则降低坡度。
    // 距离应该是根据fov的宽度选择
    double th = 45.0/180.0*M_PI; //60的坡度
    // 计算ob的对角线长度
    double ob_diag = 1.2 * sqrt(ob.l*ob.l + ob.w*ob.w + ob.h*ob.h);
    double l = ob_diag * cos(th);
    int num = 10;
    double dth = 360/(double)num /180*M_PI;
    for(int i=0; i<num; i++){
        double x = l * cos(double(i)*dth) + ob.x;
        double y = l * sin(double(i)*dth) + ob.y;
        double z = ob_diag * sin(th)    + ob.z;
        candidate cand(Eigen::Vector3d(x, y, z), Eigen::Vector3d(ob.x, ob.y, ob.z));
        // cand.start = Eigen::Vector3d(x, y, z);
        // cand.end = Eigen::Vector3d(ob.x, ob.y, ob.z);
        std::cout<<"ob_diag: "<< ob_diag <<std::endl;
        std::cout<<"end: "<<cand.end.transpose()<<";      start: "<<cand.start.transpose()<<std::endl;
        candidates.push_back(cand);
    }


    ros::Rate rate(10);  // 发布频率为10Hz
    while (ros::ok())
    {
        // v1: 以始末坐标点的方式显示
        for (size_t i = 0; i < candidates.size(); i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";  // 假设您的世界坐标系为"world"
            marker.id = i;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.1;  // 箭头的尺寸
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            // 设置箭头的起点和终点
            geometry_msgs::Point start = eigen_to_point(candidates[i].start);
            marker.points.push_back(start);

            geometry_msgs::Point end = eigen_to_point(candidates[i].end);
            marker.points.push_back(end);

            // 设置箭头的颜色
            if(i==0){
                marker.color.a = 1.0;  // 不透明度
                marker.color.r = 1.0;  // 红色分量
                marker.color.g = 0.0;  // 绿色分量
                marker.color.b = 0.0;  // 蓝色分量
            }
            else{
                marker.color.a = 1.0;  // 不透明度
                marker.color.r = 0.0;  // 红色分量
                marker.color.g = 1.0;  // 绿色分量
                marker.color.b = 0.0;  // 蓝色分量
            }
            

            marker_pub.publish(marker);
            rate.sleep();
        }

        // v2: 以“四元数”的方式显示
        // for (size_t i = 0; i < candidates.size(); i++)
        // {
        //     geometry_msgs::PoseWithCovarianceStamped can;
        //     can.pose.pose.position.x = candidates[i].x;
        //     can.pose.pose.position.y = candidates[i].y;
        //     can.pose.pose.position.z = candidates[i].z;

        //     // Eigen::Quaterniond q_w_body = Converter::ExtractQuaterniond(T_w_basefootprint);
        //     // Eigen::Quaterniond q_body_rotate = Eigen::Quaterniond( Eigen::AngleAxisd( mGreat_angle*M_PI/180.0, Eigen::Vector3d ( 0,0,1 ) )  );     //沿 Z 轴旋转 45 度
        //     // Eigen::Quaterniond q = q_w_body * q_body_rotate;
        //     can.pose.pose.orientation.w = candidates[i].q.w();
        //     can.pose.pose.orientation.x = candidates[i].q.x();
        //     can.pose.pose.orientation.y = candidates[i].q.y();
        //     can.pose.pose.orientation.z = candidates[i].q.z();
        //     can.header.frame_id= "map";
        //     can.header.stamp=ros::Time::now();
        //     std::cout<<"end: "<<candidates[i].end.transpose()<<std::endl;

        //     publisher_mam_rviz.publish(can);
        //     rate.sleep();
        // }
    }

    return 0;
}