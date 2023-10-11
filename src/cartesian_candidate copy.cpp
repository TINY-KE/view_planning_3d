#include <cmath>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>


// moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


std::string root_frame = "base_link";
std::string target_frame = "Link7"; //"/camera_rgb_frame"
std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}




cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
    //cv::Mat cv_mat_32f;
    //cv::eigen2cv(GroundtruthPose_eigen, cv_mat_32f);
    //cv_mat_32f.convertTo(mT_body_cam, CV_32F);
}

cv::Mat toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    return cvMat.clone();
}

Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

std::vector<float> toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

//[active slam]
float bboxOverlapratio(const cv::Rect& rect1, const cv::Rect& rect2)
{
    if (rect1.width == 0 || rect1.height == 0 || rect2.width == 0 || rect2.height == 0) {
        return 0;
    }
    int overlap_area = (rect1&rect2).area();
    return (float)overlap_area/((float)(rect1.area()+rect2.area()-overlap_area));
}

float bboxOverlapratioLatter(const cv::Rect& rect1, const cv::Rect& rect2)
{
    int overlap_area = (rect1&rect2).area();
    return (float)overlap_area/((float)(rect2.area()));
}

float bboxOverlapratioFormer(const cv::Rect& rect1, const cv::Rect& rect2)
{
    int overlap_area = (rect1&rect2).area();
    return (float)overlap_area/((float)(rect1.area()));
}

//cv::Mat toCvMat(const Eigen::Matrix3d &m)
//{
//    cv::Mat cvMat(3,3,CV_32F);
//    for(int i=0;i<3;i++)
//        for(int j=0; j<3; j++)
//            cvMat.at<float>(i,j)=m(i,j);
//
//    return cvMat.clone();
//}

Eigen::Matrix4d cvMattoMatrix4d(const cv::Mat &cvMat4) {
    Eigen::Matrix4f eigenMat4f;
    Eigen::Matrix4d eigenMat4d;
    //std::cout<<"converter debug: "<<cvMat4<<std::endl;
    //M << cvMat4.at<float>(0, 0), cvMat4.at<float>(0, 1), cvMat4.at<float>(0, 2), cvMat4.at<float>(0, 3),
    //     cvMat4.at<float>(1, 0), cvMat4.at<float>(1, 1), cvMat4.at<float>(1, 2), cvMat4.at<float>(1, 3),
    //     cvMat4.at<float>(2, 0), cvMat4.at<float>(2, 1), cvMat4.at<float>(2, 2), cvMat4.at<float>(2, 3),
    //     cvMat4.at<float>(3, 0), cvMat4.at<float>(3, 1), cvMat4.at<float>(3, 2), cvMat4.at<float>(3, 3);
    cv::cv2eigen(cvMat4, eigenMat4f);
    eigenMat4d = eigenMat4f.cast<double>();
    return eigenMat4d;
}


Eigen::Matrix4d Quation2Eigen(const double qx, const double qy, const double qz, const double qw, const double tx,
                                 const double ty, const double tz) {

    Eigen::Quaterniond quaternion(Eigen::Vector4d(qx, qy, qz, qw));
    Eigen::AngleAxisd rotation_vector(quaternion);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_vector);
    T.pretranslate(Eigen::Vector3d(tx, ty, tz));
    Eigen::Matrix4d Pose_eigen = T.matrix();
    return Pose_eigen;
}

cv::Mat Quation2CvMat(const double qx, const double qy, const double qz, const double qw, const double tx, const double ty, const double tz  ) {
    return toCvMat(
            Quation2Eigen(qx, qy, qz, qw, tx, ty, tz )
    );
}

Eigen::Isometry3d  Matrix4dtoIsometry3d(const Eigen::Matrix4d &matrix) {
    Eigen::Isometry3d Iso=Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
    //不能直接赋值
    //    T1<< 1.000000e+00, 1.197624e-11, 1.704639e-10, 3.214096e-14,
    //            1.197625e-11, 1.197625e-11, 3.562503e-10, -1.998401e-15,
    //            1.704639e-10, 3.562503e-10, 1.000000e+00, -4.041212e-14,
    //                       0,            0,            0,              1;

    //----1.对各个元素赋值----
    Iso(0, 0) = matrix(0, 0), Iso(0, 1) = matrix(0, 1), Iso(0, 2) = matrix(0, 2), Iso(0, 3) = matrix(0, 3);
    Iso(1, 0) = matrix(1, 0), Iso(1, 1) = matrix(1, 1), Iso(1, 2) = matrix(1, 2), Iso(1, 3) = matrix(1, 3);
    Iso(2, 0) = matrix(2, 0), Iso(2, 1) = matrix(2, 1), Iso(2, 2) = matrix(2, 2), Iso(2, 3) = matrix(2, 3);
    Iso(3, 0) = matrix(3, 0), Iso(3, 1) = matrix(3, 1), Iso(3, 2) = matrix(3, 2), Iso(3, 3) = matrix(3, 3);

    return Iso;
}

Eigen::Matrix4d Isometry3dtoMatrix4d(const Eigen::Isometry3d &Iso ){
    return Iso.matrix();
}

Eigen::Isometry3d cvMattoIsometry3d(const cv::Mat &cvMat4){
    return Matrix4dtoIsometry3d(
            cvMattoMatrix4d( cvMat4 )
            );
}


Eigen::Quaterniond ExtractQuaterniond(const Eigen::Isometry3d &Iso){
    Eigen::Quaterniond q = Eigen::Quaterniond(Iso.rotation());
    return q;
}

Eigen::Quaterniond ExtractQuaterniond(const Eigen::Matrix4d &matrix ){
    return ExtractQuaterniond(
            Matrix4dtoIsometry3d(matrix)
    );
}

Eigen::Quaterniond ExtractQuaterniond(const cv::Mat &mat ){
    return ExtractQuaterniond(
            cvMattoIsometry3d(mat)
    );
}





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

            Eigen::Quaterniond quaternion;
            quaternion.setFromTwoVectors(Eigen::Vector3d::UnitX(), direction);
            q = quaternion;

            x = start(0);
            y = start(1);
            z = start(2);
        }

        cv::Mat GetPose(){
            // cv::Mat pose = cv::Mat::ones(4,4,CV_32F);
            // pose.at<float>(0,3) = x;
            // pose.at<float>(1,3) = y;
            // pose.at<float>(2,3) = z;
            // pose.at<float>(3,3) = 1;
            
            Eigen::AngleAxisd rotation_vector(q);
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.rotate(rotation_vector);
            T.pretranslate(Eigen::Vector3d(x, y, z));
            Eigen::Matrix4d Pose_eigen = T.matrix();

            cv::Mat cvMat(4,4,CV_32F);
            for(int i=0;i<4;i++)
                for(int j=0; j<4; j++)
                    cvMat.at<float>(i,j)=Pose_eigen(i,j);

            return cvMat.clone();
        }

        
        cv::Mat GetCameraCenter(){
            cv::Mat o = (cv::Mat_<float>(4,1) << x, y, z, 1);
            return o.clone();   
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


std::vector<candidate> CandidatesInterpolation(const std::vector<candidate> &candidates, const candidate candidate_init){
    std::vector<candidate> candidates_insert;
    
    candidates_insert.push_back(candidate_init);

    // 左
    int num = 5;
    Eigen::Vector3d line = (candidates[0].start - candidate_init.start)/(double)num;
    for(int i=0; i<num; i++){
        // candidates[i] 和 candidates[i+1] 之间的连线
        Eigen::Vector3d new_start = candidate_init.start + line*(double)i;
        candidate c1(new_start, candidate_init.end); 
        candidates_insert.push_back(c1);
    }

    
    for(int i=0; i<candidates.size()-1; i++){
        candidate c = candidates[i];
        // candidates[i] 和 candidates[i+1] 之间的连线
        Eigen::Vector3d line = (candidates[i+1].start - candidates[i].start)/3.0;
        Eigen::Vector3d new_start = candidates[i].start + line;
        candidate c1(new_start, candidates[i].end); 
        new_start = candidates[i].start + line*2.0;
        candidate c2(new_start, candidates[i].end); 

        candidates_insert.push_back(candidates[i]);
        candidates_insert.push_back(c1);
        candidates_insert.push_back(c2);
    }

    candidates_insert.push_back(candidates[candidates.size()-1]);
    

    line = (candidate_init.start - candidates[candidates.size()-1].start)/(double)num;
    for(int i=0; i<num; i++){
        // candidates[i] 和 candidates[i+1] 之间的连线
        Eigen::Vector3d new_start = candidates[candidates.size()-1].start + line*(double)i;
        candidate c1(new_start, candidate_init.end); 
        candidates_insert.push_back(c1);
    }

    candidates_insert.push_back(candidate_init);
    
    return candidates_insert;
}



void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& current_scene , object& ob)
{
    // 添加物体
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = root_frame;

    // The id of the object is used to identify it.
    collision_object.id = "object";

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = ob.l;
    primitive.dimensions[1] = ob.w;
    primitive.dimensions[2] = ob.h;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = ob.x;
    box_pose.position.y = ob.y;
    box_pose.position.z = ob.z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;


    // 创建一个障碍物的列表，把之前创建的障碍物实例加入其中
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // 所有障碍物加入列表后（这里只有一个障碍物），再把障碍物加入到当前的情景中，如果要删除障碍物，使用removeCollisionObjects(collision_objects)
    ROS_INFO_NAMED("Candidate", "Add an object into the world");
    current_scene.addCollisionObjects(collision_objects);




    // // 设置障碍物的外形、尺寸等属性   
    // shape_msgs::SolidPrimitive primitive;
    // primitive.type = primitive.CYLINDER;
    // primitive.dimensions.resize(3);
    // primitive.dimensions[0] = 0.6;
    // primitive.dimensions[1] = 0.2;

    // // 设置障碍物的位置
    // geometry_msgs::Pose pose;
    // pose.orientation.w = 1.0;
    // pose.position.x =  0.0;
    // pose.position.y = -0.4;
    // pose.position.z =  0.4;

    // // 将障碍物的属性、位置加入到障碍物的实例中
    // cylinder.primitives.push_back(primitive);
    // cylinder.primitive_poses.push_back(pose);
    // cylinder.operation = cylinder.ADD;

    // // 创建一个障碍物的列表，把之前创建的障碍物实例加入其中
    // std::vector<moveit_msgs::CollisionObject> collision_objects;
    // collision_objects.push_back(cylinder);

    // // 所有障碍物加入列表后（这里只有一个障碍物），再把障碍物加入到当前的情景中，如果要删除障碍物，使用removeCollisionObjects(collision_objects)
    // current_scene.addCollisionObjects(collision_objects);

}


int main(int argc, char** argv){


    ros::init(argc, argv, "vector_display_node");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/candidate", 10);
    ros::Publisher publisher_mam_rviz = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/local_nbv", 1000);
    ros::Publisher publisher_KF = nh.advertise<visualization_msgs::Marker>("KeyFrame", 1000);
    ros::Publisher marker_object_pub = nh.advertise<visualization_msgs::Marker>("/object_marker", 10);
    
    object ob(  0.68888+0.1, 0.0/* -0.317092 */, 0.467195,
                0.368233, 0.400311, 0.245264,
                0.000000, 0.000000, 0.000000   );
    std::vector<candidate> candidates;


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

    // 插值算法
    // 0.291, -0.400, 0.702
    candidate candidate_init(Eigen::Vector3d(0.291, -0.400, 0.702), Eigen::Vector3d(ob.x, ob.y, ob.z));
    std::vector<candidate> candidates_ok;
    candidates_ok.push_back(candidates[4]);
    candidates_ok.push_back(candidates[5]);
    candidates_ok.push_back(candidates[6]);
    std::vector<candidate> candidates_insert = CandidatesInterpolation(candidates_ok, candidate_init);



    ros::Rate rate(3);  // 发布频率为10Hz
    // while (ros::ok())
    {   
        // 显示物体
        visualization_msgs::Marker marker_object;
        marker_object.header.frame_id = root_frame;
        marker_object.type = visualization_msgs::Marker::CUBE;
        marker_object.action = visualization_msgs::Marker::ADD;
        marker_object.pose.position.x = ob.x;  // 立方体中心的x坐标
        marker_object.pose.position.y = ob.y;  // 立方体中心的y坐标
        marker_object.pose.position.z = ob.z;  // 立方体中心的z坐标
        marker_object.scale.x = ob.l;  // 立方体的尺寸：x轴方向
        marker_object.scale.y = ob.w;  // 立方体的尺寸：y轴方向
        marker_object.scale.z = ob.h;  // 立方体的尺寸：z轴方向
        marker_object.color.r = 0.0;  // 立方体的颜色：红色分量
        marker_object.color.g = 1.0;  // 立方体的颜色：绿色分量
        marker_object.color.b = 0.0;  // 立方体的颜色：蓝色分量
        marker_object.color.a = 0.8;  // 立方体的颜色透明度
        marker_object_pub.publish(marker_object);
        std::cout<<"物体cube" <<std::endl;

        // v1: 以始末坐标点的方式显示
        for (size_t i = 0; i < candidates.size(); i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = root_frame;  // 假设您的世界坐标系为"world"
            marker.id = i;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.03;  // 箭头的尺寸
            marker.scale.y = 0.03;
            // marker.scale.z = 0.02;

            // 设置箭头的起点和终点
            geometry_msgs::Point start = eigen_to_point(candidates[i].start);
            marker.points.push_back(start);
            Eigen::Vector3d end_new = (candidates[i].end - candidates[i].start).normalized() * 0.2 + candidates[i].start;
            geometry_msgs::Point end = eigen_to_point(end_new);
            marker.points.push_back(end);

            // 设置箭头的颜色
            if(i==4 || i==5 || i==6 ){
                marker.color.a = 1.0;  // 不透明度
                marker.color.r = 0.0;  // 红色分量
                marker.color.g = 1.0;  // 绿色分量
                marker.color.b = 0.0;  // 蓝色分量
                
            }
            else{
                marker.color.a = 1.0;  // 不透明度
                marker.color.r = 1.0;  // 红色分量
                marker.color.g = 1.0;  // 绿色分量
                marker.color.b = 0.0;  // 蓝色分量
            }
            marker_pub.publish(marker);
            rate.sleep();
            
        }
        for (size_t i = 0; i < candidates_insert.size(); i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = root_frame;  // 假设您的世界坐标系为"world"
            marker.id = candidates.size()+ i;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.03;  // 箭头的尺寸
            marker.scale.y = 0.03;
            // marker.scale.z = 0.02;

            // 设置箭头的起点和终点
            geometry_msgs::Point start = eigen_to_point(candidates_insert[i].start);
            marker.points.push_back(start);
            Eigen::Vector3d end_new = (candidates_insert[i].end - candidates_insert[i].start).normalized() * 0.2 + candidates_insert[i].start;
            geometry_msgs::Point end = eigen_to_point(end_new);
            marker.points.push_back(end);

            // 设置箭头的颜色
            marker.color.a = 1.0;  // 不透明度
            marker.color.r = 0.0;  // 红色分量
            marker.color.g = 0.0;  // 绿色分量
            marker.color.b = 1.0;  // 蓝色分量
            
            marker_pub.publish(marker);
            rate.sleep();
            
        }
        std::cout<<"始末向量"<<std::endl;


        // // v2: 以“四元数”的方式显示
        // for (size_t i = 0; i < candidates.size(); i++)
        // {
        //     geometry_msgs::PoseWithCovarianceStamped can;
        //     can.pose.pose.position.x = candidates[i].x;
        //     can.pose.pose.position.y = candidates[i].y;
        //     can.pose.pose.position.z = candidates[i].z;

        //     // Eigen::Quaterniond q_w_body = ExtractQuaterniond(T_w_basefootprint);
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

        // // v3: 以“四元数”的方式显示
        // for (size_t i = 0; i < candidates.size(); i++)
        // {
        //     geometry_msgs::PoseWithCovarianceStamped can;
        //     can.pose.pose.position.x = candidates[i].x;
        //     can.pose.pose.position.y = candidates[i].y;
        //     can.pose.pose.position.z = candidates[i].z;

        //     Eigen::AngleAxisd rotation_vector(candidates[i].q);
        //     Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        //     T.rotate(rotation_vector);
        //     T.pretranslate(Eigen::Vector3d(candidates[i].x, candidates[i].y, candidates[i].z));
        //     Eigen::Matrix4d Pose_eigen = T.matrix();

        //     cv::Mat cvMat(4,4,CV_32F);
        //     for(int i=0;i<4;i++)
        //         for(int j=0; j<4; j++)
        //             cvMat.at<float>(i,j)=Pose_eigen(i,j);

        //     Eigen::Quaterniond qq = ExtractQuaterniond(cvMat);

        //     can.pose.pose.orientation.w = qq.w();
        //     can.pose.pose.orientation.x = qq.x();
        //     can.pose.pose.orientation.y = qq.y();
        //     can.pose.pose.orientation.z = qq.z();

        //     can.header.frame_id= "map";
        //     can.header.stamp=ros::Time::now();

        //     // std::cout<<"end: "<<candidates[i].end.transpose()<<std::endl;
        //     std::cout<<"rate: "<<i<<std::endl;

        //     publisher_mam_rviz.publish(can);
        //     rate.sleep();
        // }


            


        // v4::  slam中的锥形
        visualization_msgs::Marker mKeyFrames;
        float d = 0.04;
        mKeyFrames.header.frame_id = root_frame;
        mKeyFrames.ns = "KeyFrames";
        mKeyFrames.id=1;
        mKeyFrames.type = visualization_msgs::Marker::LINE_LIST;
        mKeyFrames.scale.x=0.005;
        mKeyFrames.pose.orientation.w=1.0;
        mKeyFrames.action=visualization_msgs::Marker::ADD;

        mKeyFrames.color.b=1.0f;
        mKeyFrames.color.a = 1.0;

        //Camera is a pyramid. Define in camera coordinate system
        cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
        cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
        cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
        cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
        cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

        for(size_t i=0, iend=candidates.size() ;i<iend; i++)
        {
            cv::Mat Tcw = candidates[i].GetPose();
            //        std::cout<<"[rviz debug] frame id： "<< candidates[i]->mnId <<", Tcw:"<<Tcw<< std::endl;
            cv::Mat Twc = candidates[i].GetPose(); //Tcw.inv();
            cv::Mat ow = candidates[i].GetCameraCenter();
            cv::Mat p1w = Twc*p1;
            cv::Mat p2w = Twc*p2;
            cv::Mat p3w = Twc*p3;
            cv::Mat p4w = Twc*p4;

            geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
            msgs_o.x=ow.at<float>(0);
            msgs_o.y=ow.at<float>(1);
            msgs_o.z=ow.at<float>(2);
            msgs_p1.x=p1w.at<float>(0);
            msgs_p1.y=p1w.at<float>(1);
            msgs_p1.z=p1w.at<float>(2);
            msgs_p2.x=p2w.at<float>(0);
            msgs_p2.y=p2w.at<float>(1);
            msgs_p2.z=p2w.at<float>(2);
            msgs_p3.x=p3w.at<float>(0);
            msgs_p3.y=p3w.at<float>(1);
            msgs_p3.z=p3w.at<float>(2);
            msgs_p4.x=p4w.at<float>(0);
            msgs_p4.y=p4w.at<float>(1);
            msgs_p4.z=p4w.at<float>(2);

            mKeyFrames.points.push_back(msgs_o);
            mKeyFrames.points.push_back(msgs_p1);
            mKeyFrames.points.push_back(msgs_o);
            mKeyFrames.points.push_back(msgs_p2);
            mKeyFrames.points.push_back(msgs_o);
            mKeyFrames.points.push_back(msgs_p3);
            mKeyFrames.points.push_back(msgs_o);
            mKeyFrames.points.push_back(msgs_p4);
            mKeyFrames.points.push_back(msgs_p1);
            mKeyFrames.points.push_back(msgs_p2);
            mKeyFrames.points.push_back(msgs_p2);
            mKeyFrames.points.push_back(msgs_p3);
            mKeyFrames.points.push_back(msgs_p3);
            mKeyFrames.points.push_back(msgs_p4);
            mKeyFrames.points.push_back(msgs_p4);
            mKeyFrames.points.push_back(msgs_p1);
        }
        mKeyFrames.header.stamp = ros::Time::now();
        publisher_KF.publish(mKeyFrames);
        rate.sleep();
        std::cout<<"锥形相机"<<std::endl;
    }



    // 创建一个异步的自旋线程（spinning thread）
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroupInterface move_group("arm");

	const robot_state::JointModelGroup* joint_model_group =
		    move_group.getCurrentState()->getJointModelGroup("arm");

	//获取终端link的名称
    std::string end_effector_link=move_group.getEndEffectorLink();
	ROS_INFO_NAMED("fabo_arm", "End effector link: %s", end_effector_link.c_str());

    //设置目标位置所使用的参考坐标系
	ROS_INFO_NAMED("fabo_arm", "Planning frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("fabo_arm", "Pose Reference frame: %s", move_group.getPoseReferenceFrame().c_str());
    std::string pose_reference_frame=root_frame;
    move_group.setPoseReferenceFrame(pose_reference_frame);
	ROS_INFO_NAMED("fabo_arm", "New Pose Reference frame: %s", move_group.getPoseReferenceFrame().c_str());

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    addCollisionObjects(planning_scene_interface, ob);

	std::vector<geometry_msgs::Pose> waypoints;	
	
	// 获取当前位置
	tf::TransformListener listener;
    tf::StampedTransform transform;
	geometry_msgs::Pose start_pose;
    try
    {
		float mTfDuration = 2.0;
        listener.waitForTransform(root_frame, target_frame, ros::Time(0), ros::Duration(mTfDuration));
        listener.lookupTransform(root_frame, target_frame, ros::Time(0), transform);
		start_pose.orientation.x = transform.getRotation().x();
		start_pose.orientation.y = transform.getRotation().y();
		start_pose.orientation.z = transform.getRotation().z();
		start_pose.orientation.w = transform.getRotation().w();
		start_pose.position.x = transform.getOrigin().x();
		start_pose.position.y = transform.getOrigin().y();
		start_pose.position.z = transform.getOrigin().z();
		// std::cout<<"start pose:"<<start_pose<<std::endl;
		waypoints.push_back(start_pose);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s -->> lost tf from /base_footprint to /camera_rgb_frame",ex.what());
    }
	
    // 添加candidates
	
    // 将candidate全部添加到waypoints中
    for(auto cand: candidates_insert){
	    geometry_msgs::Pose target_pose;
        target_pose.orientation.x = cand.q.x();
        target_pose.orientation.y = cand.q.y();
        target_pose.orientation.z = cand.q.z();
        target_pose.orientation.w = cand.q.w();

        target_pose.position.x = cand.x;
        target_pose.position.y = cand.y;
        target_pose.position.z = cand.z;

    	waypoints.push_back(target_pose);
    }

    // 将初始pose重新加入
    waypoints.push_back(waypoints[0]);
	


	// 笛卡尔空间下的路径规划与执行
	ROS_INFO("Start planning");
	
	// 笛卡尔空间下的路径规划
	move_group.setMaxVelocityScalingFactor(0.1);

	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 2.0;
	const double eef_step = 0.01; //05;  //步长
	double fraction = 0.0;
	int maxtries = 100; //最大尝试规划次数
	int attempts = 0; //已经尝试规划次数
	while(fraction < 1.0 && attempts < maxtries){
		fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
		attempts++;
		if(attempts % 10 == 0)
		ROS_INFO("Still trying after %d attempts...", attempts);
	}

	if(fraction == 1){
		ROS_INFO("Path computed successfully. Moving the arm.");
		// 生成机械臂的运动规划数据
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectory;
		// 执行运动
		move_group.execute(plan);
		sleep(1);
	}
	else{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
	}

    return 0;
}