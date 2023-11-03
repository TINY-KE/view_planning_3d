
#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <gpmp2/kinematics/GaussianPriorWorkspaceOrientationArm.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/kinematics/Arm.h>
#include <gpmp2/kinematics/GaussianPriorWorkspacePoseArm.h>
// #include <gpmp2/kinematics/RobotModel.h>



#include <iostream>
#include <cmath>

// zhjd： arm轨迹优化专用
#include <gpmp2/planner/TrajUtils.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h>

#include <gtsam/inference/Symbol.h>


#include <gtsam/slam/PriorFactor.h>
#include <gpmp2/gp/GaussianProcessPriorPose2Vector.h>
#include <gpmp2/geometry/numericalDerivativeDynamic.h>

#include <cmath>
#include <algorithm>

// sdf
#include <gpmp2/obstacle/ObstacleSDFFactor.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGP.h>

// 优化器
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <gpmp2/kinematics/GaussianPriorWorkspacePoseArm.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>

// 查看arm轨迹是否避障
#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/planner/BatchTrajOptimizer.h>


// 关节之间的连续性？ GP prior
#include <gpmp2/gp/GaussianProcessPriorLinear.h>

// ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Float64MultiArray.h"

// #include "sdf.h"


// moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// tf
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include "Converter.h"

using namespace std;
using namespace gtsam;
using namespace gpmp2;

bool field_type = 0;

// 定义调试宏
#define DEBUG

typedef ObstacleSDFFactor<ArmModel> ObstacleSDFFactorArm;
typedef ObstacleSDFFactorGP<ArmModel, GaussianProcessInterpolatorLinear>   ObstacleSDFFactorGPArm;



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

        // cv::Mat GetPose(){
        //     // cv::Mat pose = cv::Mat::ones(4,4,CV_32F);
        //     // pose.at<float>(0,3) = x;
        //     // pose.at<float>(1,3) = y;
        //     // pose.at<float>(2,3) = z;
        //     // pose.at<float>(3,3) = 1;
            
        //     Eigen::AngleAxisd rotation_vector(q);
        //     Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        //     T.rotate(rotation_vector);
        //     T.pretranslate(Eigen::Vector3d(x, y, z));
        //     Eigen::Matrix4d Pose_eigen = T.matrix();

        //     cv::Mat cvMat(4,4,CV_32F);
        //     for(int i=0;i<4;i++)
        //         for(int j=0; j<4; j++)
        //             cvMat.at<float>(i,j)=Pose_eigen(i,j);

        //     return cvMat.clone();
        // }

        
        // cv::Mat GetCameraCenter(){
        //     cv::Mat o = (cv::Mat_<float>(4,1) << x, y, z, 1);
        //     return o.clone();   
        // }

};




/**
 * @brief Add obstacle to map and corner
 * 
 * @param position ：障碍物的中心位置，以三维坐标表示。
 * @param size ：障碍物的尺寸，以三维向量表示。
 * @param map ：表示地图的三维数组。
 * @param corner ：表示障碍物角落坐标的二维数组。
 * @return std::vector<std::vector<std::vector<int>>> 
 */


void add_obstacle(std::vector<int> position, std::vector<int> size, std::vector<Matrix>& map){
    int half_size_row = floor((size[0] - 1) / 2);
    int half_size_col = floor((size[1] - 1) / 2);
    int half_size_z = floor((size[2] - 1) / 2);
    // std::cout<<"sdf添加障碍物, map:"<<map.size()<<" "<<map[0].rows()<<" "<<map[0].cols()<<std::endl; ;
    // occupancy grid
    for (int i = position[0] - half_size_row - 1; i < position[0] + half_size_row; i++) {
        for (int j = position[1] - half_size_col - 1; j < position[1] + half_size_col; j++) {
            for (int z = position[2] - half_size_z - 1; z < position[2] + half_size_z; z++) {
                // std::cout<<"map["<<z<<"]("<<i<<","<<j<<") ";
                // std::cout<<" = "<<map[z](i,j)<<std::endl;
                map[z](i,j) = 1; 
            }
        }
    }
}


double sdf_wrapper(const SignedDistanceField& field, const Point3& p) {
    return field.getSignedDistance(p);
}

// SignedDistanceField 
void generate3Ddataset_addObstacle(std::vector<gtsam::Matrix>& dataset, const Point3& origin, const double& cell_size, const int& rows, const int& cols, const int& heights){
    
    // constructor
    // SignedDistanceField sdf(origin, cell_size, data);
    // SignedDistanceField sdf(origin, cell_size, rows, cols, heights);

    // map in form of matrix
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix;
    matrix = Eigen::MatrixXd::Zero(rows, cols);
    dataset.resize(heights);
    for (int i = 0; i < heights; i++) {
        dataset[i] = matrix;
    }


    std::cout<<"[debug]: construct sdf"<<std::endl;
    add_obstacle({170, 220, 130}, {140, 60, 5}, dataset);
    add_obstacle({105, 195, 90}, {10, 10, 80}, dataset);
    add_obstacle({235, 195, 90}, {10, 10, 80}, dataset);
    add_obstacle({105, 245, 90}, {10, 10, 80}, dataset);
    add_obstacle({235, 245, 90}, {10, 10, 80}, dataset);
    add_obstacle({250, 190, 145}, {60, 5, 190}, dataset);
    add_obstacle({250, 90, 145}, {60, 5, 190}, dataset);
    add_obstacle({200, 190, 145}, {40, 5, 190}, dataset);
    add_obstacle({250, 140, 240}, {60, 100, 5}, dataset);
    add_obstacle({250, 140, 190}, {60, 100, 5}, dataset);
    add_obstacle({250, 140, 140}, {60, 100, 5}, dataset);
    add_obstacle({250, 140, 90}, {60, 100, 5}, dataset);
    // debug
    // add_obstacle({150, 10, 10}, {20, 10, 10}, dataset);

    std::cout<<"[debug]: construct sdf End"<<std::endl;

    // 发布障碍物
    // std::cout<<"sdf dataset: "<<std::endl;
    // // for (int j = 220-1; j < 260; j++) {
    // //     std::cout<<data[129](170,j)<<" "<<std::endl;
    // // }
    // for (int z = 130-1; z < 150; z++) {
    //     std::cout<<map[z](169,219)<<" "<<std::endl;
    // }
    // std::cout<<std::endl;

    // return sdf;
}


// 计算距离最近障碍物的欧几里得（栅格）距离
std::vector<gtsam::Matrix> bwdist( const std::vector<gtsam::Matrix>& map ){
    vector<Matrix> euclidean_dist_map;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix;
    matrix = Eigen::MatrixXd::Ones(map[0].rows(), map[0].cols()) * sqrt(map[0].rows()*map[0].rows() + map[0].cols()*map[0].cols()  + map.size()*map.size());
    euclidean_dist_map.resize(map.size());
    for (int i = 0; i < map.size(); i++) {
        euclidean_dist_map[i] = matrix;
    }

    for(int i=0; i<map[0].rows(); i++){
        std::cout<<"处理第"<<i<<"行"<<std::endl;
        for(int j=0; j<map[0].cols(); j++)
            for(int k=0; k<map.size(); k++){
                
                if(map[k](i,j)){
                    euclidean_dist_map[k](i,j) = 0;
                    continue;
                }

                double min_dis = euclidean_dist_map[k](i,j); 
                
                for(int i2=0; i2<map[0].rows(); i2++)
                    for(int j2=0; j2<map[0].cols(); j2++)
                        for(int k2=0; k2<map.size(); k2++){
                            double dis;
                            if( euclidean_dist_map[k](i,j) )
                                dis =  sqrt(pow(i2-i,2) + pow(j2-j,2) + pow(k2-k,2));
                            if(dis < min_dis){
                                min_dis = dis;
                            }
                            euclidean_dist_map[k](i,j) = min_dis;
                        }
                
            }
    }

    return euclidean_dist_map;
}



// 计算地图中每个点的sdf值
std::vector<gtsam::Matrix> signedDistanceField3D(const std::vector<gtsam::Matrix>& ground_truth_map, double& cell_size){
// field_rows_(data[0].rows()), field_cols_(data[0].cols()),field_z_(data.size())    
    vector<Matrix> field;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix;
    matrix = Eigen::MatrixXd::Zero(ground_truth_map[0].rows(), ground_truth_map[0].cols());
    field.resize(ground_truth_map.size());
    for (int i = 0; i < ground_truth_map.size(); i++) {
        field[i] = matrix;
    }

    std::vector<gtsam::Matrix> map, inv_map;
    map = ground_truth_map;
    inv_map = ground_truth_map;

    std::cout<<"分为1和0 ... "<<std::endl;
    for(int i=0; i<ground_truth_map[0].rows(); i++){
        for(int j=0; j<ground_truth_map[0].cols(); j++){
            for(int k=0; k<ground_truth_map.size(); k++){
                
                // 处理map
                if(map[k](i,j)  > 0.75){
                    map[k](i,j)  = 1.0;
                }
                else{
                    map[k](i,j)  = 0.0;
                }

                // 处理inv_map
                inv_map[k](i,j) = 1.0 - map[k](i,j);
            }
        }
    }
    std::cout<<"分为1和0 end ... "<<std::endl;


    std::cout<<"计算距离最近障碍物的欧几里得（栅格）距离 ... "<<std::endl;
    // 计算距离最近障碍物的欧几里得（栅格）距离
    std::vector<gtsam::Matrix> map_dist = bwdist(map);
    std::vector<gtsam::Matrix> inv_map_dist = bwdist(inv_map);

    // 将栅格距离，乘以cell_size，变为真实距离
    for(int i=0; i<map[0].rows(); i++)
        for(int j=0; j<map[0].cols(); j++)
            for(int k=0; k<map.size(); k++){
                field[k](i,j) = map_dist[k](i,j) - inv_map_dist[k](i,j);
                field[k](i,j) *= cell_size;
            }
}

ArmModel* generateArm(string arm_type){
    ArmModel* arm_model;
    if(arm_type=="realman"){    //rm_75
        //1. 机械臂DH模型
        gtsam::Vector alpha(7);  alpha << -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, 0;
        gtsam::Vector a(7);      a << 0, 0, 0, 0, 0, 0, 0;
        gtsam::Vector d(7);      d << 0.2405, 0, 0.256, 0, 0.210, 0, 0.144;
        gtsam::Vector theta(7);  theta << 0, 0, 0, 0, 0, 0, 0;
        
        // base_pose = Pose3(Rot3(eye(3)), Point3([0,0,0]'));
        gtsam::Pose3 base_pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0));
        
        // Arm(size_t dof, const gtsam::Vector& a, const gtsam::Vector& alpha, const gtsam::Vector& d, const gtsam::Pose3& base_pose, const gtsam::Vector& theta_bias);
        Arm abs_arm(7, a, alpha, d, base_pose, theta);

        // 2.用于碰撞检测的sphere
        std::vector<std::vector<double>> spheres_data = {
                        // 1  0.2405（0坐标系到1坐标系的距离）
                        {0, 0.0, 0.2405, 0.0, 0.15/2},  //基座，所以体积最大
                        {0, 0.0, 0.16, 0.0, 0.06/2},
                        {0, 0.0, 0.08, 0.0, 0.06/2},
                        {0, 0.0, 0.0, 0.0, 0.06/2},
                        // 2 0  
                        // 3 0.256
                        {2, 0.0, 0.0, 0.0, 0.06/2},
                        {2, 0.0, 0.05, 0.0, 0.06/2},
                        {2, 0.0, 0.1, 0.0, 0.06/2},
                        {2, 0.0, 0.15, 0.0, 0.06/2},
                        {2, 0.0, 0.2, 0.0, 0.06/2},
                        // 4 0
                        // 5 0.210  
                        {4, 0.0, 0.0, 0.0, 0.06/2},
                        {4, 0.0, 0.05, 0.0, 0.06/2},
                        {4, 0.0, 0.1, 0.0, 0.06/2},
                        {4, 0.0, 0.15, 0.0, 0.06/2},
                        // 6 0
                        // 7 0.144
                    };

        BodySphereVector body_spheres;
        body_spheres.clear();
        for(int i=0; i<spheres_data.size(); i++){
            BodySphere sphere(spheres_data[i][0], spheres_data[i][4], Point3(spheres_data[i][1], spheres_data[i][2], spheres_data[i][3]));
            body_spheres.push_back(sphere);
        }

        // 3.生成机械臂模型
        // ArmModel arm = ArmModel(Arm(2, a, alpha, d), BodySphereVector());
        arm_model = new ArmModel(abs_arm, body_spheres);
        std::cout<<"构建realman机械臂模型成功"<<std::endl;
    }
    else if(arm_type=="WAMArm"){ //WAM
        //1. 机械臂DH模型
        gtsam::Vector alpha(7);  alpha << -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, 0;
        gtsam::Vector a(7);      a << 0, 0, 0.045, -0.045, 0, 0, 0;
        gtsam::Vector d(7);      d << 0,0,0.55,0,0.3,0,0.06;
        gtsam::Vector theta(7);  theta << 0, 0, 0, 0, 0, 0, 0;
        
        gtsam::Pose3 base_pose = gtsam::Pose3(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0));
        
        // Arm(size_t dof, const gtsam::Vector& a, const gtsam::Vector& alpha, const gtsam::Vector& d, const gtsam::Pose3& base_pose, const gtsam::Vector& theta_bias);
        Arm abs_arm(7, a, alpha, d, base_pose, theta);

        // 2.用于碰撞检测的sphere
        std::vector<std::vector<double>> spheres_data = {
                        // 1  0.2405（0坐标系到1坐标系的距离）
                        {0, 0.0,  0.0,  0.0, 0.15},  //基座，所以体积最大
                        // 2 0  
                        {1, 0.0,  0.0,  0.2, 0.06},
                        {1, 0.0,  0.0,  0.3, 0.06},
                        {1, 0.0,  0.0,  0.4, 0.06},
                        {1, 0.0,  0.0,  0.5, 0.06},
                        // 3 0.256
                        {2, 0.0, 0.0, 0.0, 0.06},
                        // 4 0
                        {3, 0.0, 0.0, 0.1, 0.06},
                        {3, 0.0, 0.0, 0.2, 0.06},
                        {3, 0.0, 0.0, 0.3, 0.06},
                        // 5 0.210  
                        // 6 0
                        {5, 0.0, 0.0, 0.1, 0.01},
                        // 7 0.144
                        {6, 0.1,  -0.025, 0.08, 0.04},
                        {6, 0.1,  0.025, 0.08, 0.04},
                        {6, -0.1,  0, 0.08, 0.04},
                        {6, 0.15, -0.025, 0.13, 0.04},
                        {6, 0.15,  0.025, 0.13, 0.04},
                        {6, -0.15,  0,     0.13, 0.04}

                    };

        BodySphereVector body_spheres;
        body_spheres.clear();
        for(int i=0; i<spheres_data.size(); i++){
            BodySphere sphere(spheres_data[i][0], spheres_data[i][4], Point3(spheres_data[i][1], spheres_data[i][2], spheres_data[i][3]));
            body_spheres.push_back(sphere);
        }

        // 3.生成机械臂模型
        // ArmModel arm = ArmModel(Arm(2, a, alpha, d), BodySphereVector());
        arm_model = new ArmModel(abs_arm, body_spheres);
        std::cout<<"构建WAMArm机械臂模型成功"<<std::endl;
    }
    return arm_model;
}




/* 一个虚拟的物体，并构建候选点 */

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

// Eigen::Vector3d 到 geometry_msgs::Point 的转换函数
geometry_msgs::Point eigen_to_point(Eigen::Vector3d &v){
    geometry_msgs::Point p;
    p.x = v(0);
    p.y = v(1);
    p.z = v(2);
    return p;
}

std::vector<candidate>  generate_candidates(const object& ob)
{
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
    // 获取当前位置
	tf::TransformListener listener;
    tf::StampedTransform transform;
	geometry_msgs::Pose start_pose;
    std::string root_frame = "base_link";
    std::string target_frame = "Link7"; //"/camera_rgb_frame"
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
		std::cout<<"start pose:"<<start_pose<<std::endl;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s -->> lost tf from /base_footprint to /camera_rgb_frame",ex.what());
    }

    // 0.291, -0.400, 0.702
    // candidate candidate_init(Eigen::Vector3d(0.291, -0.400, 0.702), Eigen::Vector3d(ob.x, ob.y, ob.z));
    candidate candidate_init(Eigen::Vector3d(start_pose.position.x, start_pose.position.y, start_pose.position.z), Eigen::Vector3d(ob.x, ob.y, ob.z));

    std::vector<candidate> candidates_ok;
    candidates_ok.push_back(candidates[4]);
    candidates_ok.push_back(candidates[5]);
    candidates_ok.push_back(candidates[6]);
    std::vector<candidate> candidates_insert = CandidatesInterpolation(candidates_ok, candidate_init);

    return candidates_insert;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "gpmp_realman");
    ros::NodeHandle nh;

    ros::Publisher pub_field = nh.advertise<visualization_msgs::Marker>("field", 1);
    ros::Publisher pub_sdf = nh.advertise<visualization_msgs::Marker>("sdf", 1);
    ros::Publisher joint_values_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_values_gpmp", 10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/candidate", 10);


    ros::Rate loop_rate(5);  // 设置发布频率

    // 零、生成候选点
    object ob(  0.68888+0.1, 0.0/* -0.317092 */, 0.467195-0.5,
                0.368233, 0.400311, 0.245264,
                0.000000, 0.000000, 0.000000   );
    std::vector<candidate>  candidates = generate_candidates(ob);
    for (size_t i = 0; i < candidates.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";  // 假设您的世界坐标系为"world"
        marker.id = candidates.size()+ i;
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
        marker.color.a = 1.0;  // 不透明度
        marker.color.r = 0.0;  // 红色分量
        marker.color.g = 0.0;  // 绿色分量
        marker.color.b = 1.0;  // 蓝色分量
        
        marker_pub.publish(marker);
        loop_rate.sleep();
    }

    // 一、构建机械臂模型
    // ArmModel* arm_model = generateArm("WAMArm");
        ArmModel* arm_model = generateArm("realman");
    
    #ifdef DEBUG
        std::cout<<"[debug arm]: 自由度"<<arm_model->dof()<<std::endl;
    #endif
    

    
    
    

    // 二、指定始末状态
    // TODO:  用moveit计算初始位置，当前先使用全0的初始值
    // gtsam::Vector start_conf = (Vector(7) << -3.084730575741016, -1.763304599691998, 1.8552083929655296, 0.43301604856981246, -2.672461979658843, 0.46925065047728776, 4.000864693936108).finished();
    // gtsam::Vector end_conf = (Vector(7) << -2.018431036907354, -1.4999897089911451, 1.4046777996889483, -1.3707039693409548, -3.0999924865261397, -0.8560425214202461, 4.8622166345079165).finished();
    gtsam::Vector start_conf = (Vector(7) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
    gtsam::Vector end_conf = (Vector(7) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();

    gtsam::Vector start_vel = (Vector(7) << 0, 0, 0, 0, 0, 0, 0).finished();
    gtsam::Vector end_vel = (Vector(7) << 0, 0, 0, 0, 0, 0, 0).finished();
    
    // TODO:  将candidates转变为rot3
    // auto jposes = arm_model->fk_model().forwardKinematicsPose(start_conf); //???
    // Rot3 traj_orien = Rot3::Ypr(jposes(0, 6), jposes(1, 6), jposes(2, 6));
    // 已知空间中的两点，怎么计算向量的yaw-z pitch-y roll-x
    std::vector<Rot3> candidates_rot3;
    std::vector<Pose3> candidates_pose3;
    int i = 0;
    for(auto candidate: candidates)
    {
        cv::Point3f v =  cv::Point3f(candidate.end.x(),  candidate.end.y(),  candidate.end.z()) - cv::Point3f(candidate.start.x(),  candidate.start.y(),  candidate.start.z());
        float angle_yaw = std::atan2(v.y, v.x);  // 绕Z轴的旋转角度
        float angle_pitch = std::atan2(-v.z, v.x);  // 绕y轴的旋转角度

        #ifdef DEBUG
        std::cout<<"候选点" << i <<"的目标朝向偏转（ypr）："<<angle_yaw<<","<< angle_pitch<<", 0"<< std::endl;
        i++;
        // 朝向角度：1.57184,0.0207701,1.62101  说明先再绕着z转了90度，又绕着x轴转了90度。
        #endif

        // // 计算旋转矩阵
        // Eigen::Matrix3d rotation_matrix;
        // rotation_matrix = Eigen::AngleAxisd(angle_yaw, Eigen::Vector3d::UnitZ())
        //                 * Eigen::AngleAxisd(angle_pitch, Eigen::Vector3d::UnitY());

        // //转换成  cvmat矩阵
        // cv::Mat rotate_mat = Converter::toCvMat(rotation_matrix);
        // cv::Mat t_mat = (cv::Mat_<float>(3, 1) << x, y, 0);
        // cv::Mat T_world_to_baselink = cv::Mat::eye(4, 4, CV_32F);
        // rotate_mat.copyTo(T_world_to_baselink.rowRange(0, 3).colRange(0, 3));
        // t_mat.copyTo(T_world_to_baselink.rowRange(0, 3).col(3));
        // cv::Mat Camera_mat = T_world_to_baselink * mT_basefootprint_cam;

        Rot3 traj_orien = Rot3::Ypr(angle_yaw, angle_pitch, 0.0);
        Pose3 traj_pose = Pose3(traj_orien, Point3(candidate.start.x(), candidate.start.y(), candidate.start.z()));

        candidates_rot3.push_back(traj_orien);
        candidates_pose3.push_back(traj_pose);
    }
    
    
    auto jposes = arm_model->fk_model().forwardKinematicsPose(end_conf);
    Pose3 end_pose = Pose3(Rot3::Ypr(jposes(0, 6), jposes(1, 6), jposes(2, 6)), Point3(jposes(3, 6), jposes(4, 6), jposes(5, 6)));
    
    #ifdef DEBUG
        end_pose.print("终点位姿：\n");
        std::cout<<std::endl<<std::endl;
    #endif

    


    int total_time_sec = candidates.size() * 0.2; //2;
    int total_time_step = candidates.size();  //
    int check_inter = 0;
    double delta_t = real(total_time_sec) / real(total_time_step);
    double total_check_step = (check_inter + 1)*total_time_step;


    // 三、障碍物sdf
    // params
    Point3 origin(-1.5, -1.5, -1.5);
    double cell_size = 0.01;
    const int cols = 300;
    const int rows = 300;
    const int heights = 300;
    vector<Matrix> dataset;   // 用于存储地图数据。有障碍物的是1，无障碍物的是0
    // SignedDistanceField sdf = 
    generate3Ddataset_addObstacle(dataset,origin,cell_size,cols,rows,heights);

    std::cout<<"calculating signed distance field ... "<<std::endl;
    std::vector<gtsam::Matrix> field;
    if(field_type)
        field = signedDistanceField3D(dataset, cell_size);
    else{
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix;
        matrix = Eigen::MatrixXd::Zero(rows, cols);
        field.resize(heights);
        for (int i = 0; i < heights; i++) {
            field[i] = matrix;
        }

        std::cout<<"读取本地txt"<<std::endl;
        std::string filename = "/home/zhjd/ws_3d_vp/src/view_planning/txt_sdf/sdf.txt";
        // filename的数据格式：
        // 针对filed第1行的1~300列，将每一竖排的栅格数据放在txt的一行中，以空格分隔
        // 因此txt第一行，代表field的0行0列，
        // txt第二行，代表field的0行1列，
        std::ifstream file(filename); // 打开文件

        if (file.is_open()) {
            std::string line;
            int i =0;
            while (std::getline(file, line)) { // 每一行代表一竖列
                i ++;
                int row =  (i-1) % cols;
                int col = (i-1) / rows;
                
                std::istringstream iss(line); // 使用字符串流处理当前行
                double number;
                int z = 0;
                while (iss >> number) { // 从字符串流中逐个读取数字
                    // std::cout<<"z"<<z<<"  ,row"<<row<<"  ,col"<<col<<"  ,number"<<number<<std::endl;
                    field[z](row, col) = number;
                    z++;
                }
            }

            file.close(); // 关闭文件
        } else {
            std::cout << "无法打开文件：" << filename << std::endl;
        }
        std::cout<<"结束读取本地txt"<<std::endl;

    }

    std::cout<<"calculating signed distance field done"<<std::endl;
    SignedDistanceField sdf(origin, cell_size, field);
    #ifdef DEBUG
        std::cout<<"[debug sdf origin x]:"<<origin.x()<<std::endl;
        std::cout<<"[debug sdf origin y]:"<<origin.y()<<std::endl;
        std::cout<<"[debug sdf origin z]:"<<origin.z()<<std::endl;
        std::cout<<"[debug sdf cell_size]:"<<cell_size<<std::endl;
        std::cout<<"[debug sdf height]:"<<field.size()<<std::endl;
        std::cout<<"[debug sdf length]:"<<field[0].rows()<<std::endl;
        std::cout<<"[debug sdf width]:"<<field[0].cols()<<std::endl;
        
        std::cout<<"[debug sdf origin x]:"<<sdf.origin().x()<<std::endl;
        std::cout<<"[debug sdf origin y]:"<<sdf.origin().y()<<std::endl;
        std::cout<<"[debug sdf origin z]:"<<sdf.origin().z()<<std::endl;
        std::cout<<"[debug sdf cell_size]:"<<sdf.cell_size()<<std::endl;
        std::cout<<"[debug sdf height]:"<<sdf.z_count()<<std::endl;
        std::cout<<"[debug sdf length]:"<<sdf.x_count()<<std::endl;
        std::cout<<"[debug sdf width]:"<<sdf.y_count()<<std::endl;
    #endif
    // SignedDistanceField sdf(origin, cell_size, rows, cols, heights);


    // sdf可视化
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";  // 设置坐标系，根据需要进行修改
    marker.header.stamp = ros::Time::now();
    marker.ns = "sdf_field";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;  // 设置点的尺寸，根据需要进行修改
    marker.scale.y = 0.1;
    marker.color.r = 1.0;  // 设置点的颜色，根据需要进行修改
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for (int z=0; z<sdf.z_count(); z++) 
        for (int r=0; r<sdf.y_count(); r++) 
            for (int c=0; c<sdf.x_count(); c++) {
            // std::cout<<"point coordinate: "<<c<<","<<r<<","<<z<<std::endl;
            geometry_msgs::Point point;
            point.x = r*sdf.cell_size()+sdf.origin().x();  // 假设矩阵中的数据是点的坐标
            point.y = c*sdf.cell_size()+sdf.origin().y();
            point.z = z*sdf.cell_size()+sdf.origin().z();
            if(dataset[z](r, c))
                marker.points.push_back(point);
    }
    std::cout<<"marker_field赋值完毕"<<std::endl;

    visualization_msgs::Marker marker2;
    marker2.header.frame_id = "base_link";  // 设置坐标系，根据需要进行修改
    marker2.header.stamp = ros::Time::now();
    marker2.ns = "sdf_field";
    marker2.id = 1;
    marker2.type = visualization_msgs::Marker::POINTS;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose.orientation.w = 1.0;
    marker2.scale.x = 0.1;  // 设置点的尺寸，根据需要进行修改
    marker2.scale.y = 0.1;
    marker2.color.r = 0.0;  // 设置点的颜色，根据需要进行修改
    marker2.color.g = 1.0;
    marker2.color.b = 0.0;
    marker2.color.a = 1.0;

    for (int z=10; z<sdf.z_count()-10; z++) 
        for (int r=10; r<sdf.y_count()-10; r++) 
            for (int c=10; c<sdf.x_count()-10; c++) {
            // std::cout<<"point coordinate: "<<c<<","<<r<<","<<z<<std::endl;
            geometry_msgs::Point point;
            point.x = r*sdf.cell_size()+sdf.origin().x();  // 假设矩阵中的数据是点的坐标
            point.y = c*sdf.cell_size()+sdf.origin().y();
            point.z = z*sdf.cell_size()+sdf.origin().z();
            if(sdf.getSignedDistance(Point3(point.x, point.y, point.z))<0.05)
                marker2.points.push_back(point);
    }
    std::cout<<"marker_sdf赋值完毕"<<std::endl;


    
    #ifdef DEBUG
        // debug sdf内容
        // access
        SignedDistanceField::float_index idx;
        idx = sdf.convertPoint3toCell(Point3(0, 0, 0));
        idx = sdf.convertPoint3toCell(Point3(0.18, -0.18, 0.07));   // tri-linear interpolation
        idx = boost::make_tuple(1.0, 2.0, 3.0);


        // gradient
        Vector3 grad_act, grad_exp;
        Point3 p;
        p = Point3(-0.13, -0.14, 0.06);
        idx = sdf.convertPoint3toCell(p);   // tri-linear interpolation
        double signed_distance = sdf.getSignedDistance(p, grad_act);    //matlab  0.5951
        std::cout<<"[debug sdf]:(-0.13, -0.14, 0.06) sdf值： "<<signed_distance<<std::endl;
        grad_exp = numericalDerivative11(boost::function<double(const Point3&)>(
            boost::bind(sdf_wrapper, sdf, _1)), p, 1e-6);

        p = Point3(0.18, 0.12, 0.01);
        signed_distance = sdf.getSignedDistance(p, grad_act);       //matlab  0.2773
        std::cout<<"[debug sdf]:(0.18, 0.12, 0.01) sdf值： "<<signed_distance<<std::endl;
        grad_exp = numericalDerivative11(boost::function<double(const Point3&)>(
            boost::bind(sdf_wrapper, sdf, _1)), p, 1e-6);
        std::cout<<"[debug sdf]:(0.18, 0.12, 0.01) sdf梯度"<<grad_act<<std::endl;
        std::cout<<"[debug sdf]:(0.18, 0.12, 0.01) 数值梯度"<<grad_exp<<std::endl;

        
    #endif

    int counter = 0;  // 定义计数器变量
    while (ros::ok()) {
        // std::cout<<"point num:"<<marker.points.size()<<std::endl;
        marker.header.stamp = ros::Time::now();  // 更新时间戳
        pub_field.publish(marker);
        pub_sdf.publish(marker2);

        if (counter > 15) {
            // 达到指定次数后，取消定时器
            ROS_INFO("发送sdf marker");
            break;
        }
        counter++;  // 每次触发定时器，计数器加1
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    // 四、轨迹初值
    gtsam::Values init_values = gpmp2::initArmTrajStraightLine(start_conf, end_conf, total_time_step);
    // std::cout<<"轨迹初值，10个插值："<<std::endl;
    // 【已经验证过了，没有问题】： init_values.print();
    



    // 五、优化
    // 1.传感器模型
    Eigen::MatrixXd Qc = 0.1 * Eigen::MatrixXd::Identity(arm_model->dof(), arm_model->dof());
    #ifdef DEBUG
        std::cout<<"[debug协方差矩阵]:"<<std::endl<<Qc<<std::endl<<std::endl;
    #endif
    noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);// noiseModel是命名空间，Gaussian是类，Covariance是类的成员函数
    #ifdef DEBUG
        Qc_model->print("[debug高斯噪声模型]:\n");
        std::cout<<std::endl<<std::endl;
    #endif

    // 2.图优化
    // % algo settings
    double obs_sigma = 0.005;
    double epsilon_dist = 0.15;
    double fix_sigma = 1e-4;
    double end_pose_sigma = 1e-4;
    double orien_sigma = 1e-2;
    
    NonlinearFactorGraph graph;
    for(int i=0; i<=total_time_step; i++){
        Key key_pos = symbol('x', i);
        Key key_vel = symbol('v', i);

        if(i==0){
            // 2.1 起始位置约束
            graph.add(PriorFactor<Vector>(key_pos, start_conf,  noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
            graph.add(PriorFactor<Vector>(key_vel, start_vel,   noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
        }
        else if(i==total_time_step){
            // 2.2 终止位置约束
            // goal pose for end effector in workspace
            graph.add(GaussianPriorWorkspacePoseArm(key_pos, *arm_model, arm_model->dof()-1, end_pose, noiseModel::Isotropic::Sigma(6, end_pose_sigma)));
            // 为什么是dof-1
            
            // fix goal velocity
            graph.add(PriorFactor<Vector>(key_vel, end_vel, noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
        }
        else{
            // 2.3 运动学约束 fix end effector orientation in workspace to be horizontal
            graph.add(GaussianPriorWorkspaceOrientationArm(key_pos, *arm_model, arm_model->dof()-1, candidates_rot3[i]/* traj_orien */, noiseModel::Isotropic::Sigma(3, orien_sigma)));
        }

        if(i>0){
            // 初始化变量
            // key_pos1 = symbol('x', i-1);
            // key_pos2 = symbol('x', i);
            // key_vel1 = symbol('v', i-1);
            // key_vel2 = symbol('v', i);
            Key key_pos1 = symbol('x', i-1);
            Key key_pos2 = symbol('x', i);
            Key key_vel1 = symbol('v', i-1);
            Key key_vel2 = symbol('v', i);
            
            // % GP prior
            // 学习/home/zhjd/work/gpmp2/gpmp2/gp/tests/testGaussianProcessPriorLinear.cpp
            graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model));
            
            // % unary obstacle factor
            graph.add(ObstacleSDFFactorArm(key_pos, *arm_model, sdf, obs_sigma, epsilon_dist));
            
            // // % interpolated obstacle factor

            if(check_inter){
                for(int j=1; j<=check_inter; j++){
                    double tau = j * (total_time_sec / total_check_step);
                    graph.add(ObstacleSDFFactorGPArm(key_pos1, key_vel1, key_pos2, key_vel2, *arm_model, sdf, obs_sigma, epsilon_dist, Qc_model, delta_t, tau));
                }
            }
        }
        
    }
    // matlab程序
    // parameters = LevenbergMarquardtParams;
    // parameters.setVerbosity('ERROR');
    // parameters.setlambdaInitial(1000.0);
    // optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters);

    bool type = true;
    
    Values results;
    Values cout_results;
    // if(type){
    LevenbergMarquardtParams parameters;
    parameters.setVerbosity("ERROR");  //setVerbosity("TERMINATION"); //.setVerbosity("ERROR");
    parameters.setAbsoluteErrorTol(1e-12);
    parameters.setlambdaInitial(1000.0);
    LevenbergMarquardtOptimizer optimizer(graph, init_values, parameters);
    // LevenbergMarquardtOptimizer类的定义
    // #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
    // virtual class LevenbergMarquardtOptimizer : gtsam::NonlinearOptimizer {
    //     LevenbergMarquardtOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialValues);
    //     LevenbergMarquardtOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialValues, const gtsam::LevenbergMarquardtParams& params);
    //     double lambda() const;
    //     void print(string str) const;
    // };
    results = optimizer.optimize();
    // }
    // else{
    //     DoglegParams parameters;
    //     parameters.setVerbosity("ERROR");  //setVerbosity("TERMINATION"); //.setVerbosity("ERROR");
    //     DoglegOptimizer optimizer(graph, init_values, parameters);
    //     results = optimizer.optimize();
    //     // cout_results = optimizer.values();
    // }
    
    
    // Values results = optimizer.values();
    
  
    

    // /home/zhjd/work/gpmp2/gpmp2/planner/BatchTrajOptimizer.cpp
    // 282,16:       cout << "newError: " << opt->error() << endl;
    // 推测这个文件没用到

    // 结论：用到的都是下面这文件
    // /home/zhjd/software/gtsam-4.0.3/gtsam/nonlinear/NonlinearOptimizer.cpp
    // 101,15:       cout << "newError: " << error() << endl;
    // /home/zhjd/software/gtsam-4.0.3/gtsam/nonlinear/NonlinearOptimizer.cpp
    // 180,16:       cout << "errorThreshold: " << newError << " < " << errorThreshold << endl;  
    // 、、、
    

    std::cout << "Optimization complete" << std::endl;
    // std::cout << "results="<<std::endl << results << std::endl;
    std::cout << "initial error=" << graph.error(init_values) << std::endl;
    std::cout << "final error=" << graph.error(results) << std::endl;
    std::cout << "Optimization Result:"  <<std::endl;
    // results.print();
    // std::cout << "Init values:"  <<std::endl;
    // init_values.print();

    TrajOptimizerSetting opt_setting = TrajOptimizerSetting(arm_model->dof());
    opt_setting.set_total_step(total_time_step);
    opt_setting.set_total_time(total_time_sec);
    opt_setting.set_epsilon(epsilon_dist);
    opt_setting.set_cost_sigma(obs_sigma);
    opt_setting.set_obs_check_inter(check_inter);
    // opt_setting.set_conf_prior_model(noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma));
    // opt_setting.set_vel_prior_model(noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma));
    opt_setting.set_conf_prior_model(fix_sigma);
    opt_setting.set_vel_prior_model(fix_sigma);
    opt_setting.set_Qc_model(Qc);
    if(CollisionCost3DArm(*arm_model, sdf, results, opt_setting)){
        std::cout<<"Trajectory is in collision!"<<std::endl;
        return 0;
    }
    else
        std::cout<<"Trajectory is collision free."<<std::endl;



    // 五、moveit控制及rviz可视化
    // 启动movegroup
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    
    // 关节量 
    bool pub_form = true;
    std::vector<double /* std::vector<double> */ > target_joint_group_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};;
    Vector target_joint_group_positions_eigen;
    for(int i=0; i<=total_time_step; i++){
        // target_joint_group_positions.clear();
        std::cout<<"开始规划,"<<i<<std::endl;

        target_joint_group_positions_eigen = results.at<Vector>(symbol('x', i));
        
        target_joint_group_positions[0] = (double(target_joint_group_positions_eigen[0]));
        target_joint_group_positions[1] = (double(target_joint_group_positions_eigen[1]));
        target_joint_group_positions[2] = (double(target_joint_group_positions_eigen[2]));
        target_joint_group_positions[3] = (double(target_joint_group_positions_eigen[3]));
        target_joint_group_positions[4] = (double(target_joint_group_positions_eigen[4]));
        target_joint_group_positions[5] = (double(target_joint_group_positions_eigen[5]));
        target_joint_group_positions[6] = (double(target_joint_group_positions_eigen[6]));
        std::cout<<" gpmp计算结果 "<<i <<", size:"<< target_joint_group_positions.size()
                                        <<", values: "<< target_joint_group_positions[0] 
                                        <<", "<< target_joint_group_positions[1] 
                                        <<", "<< target_joint_group_positions[2]
                                        <<", "<< target_joint_group_positions[3]
                                        <<", "<< target_joint_group_positions[4]
                                        <<", "<< target_joint_group_positions[5]
                                        <<", "<< target_joint_group_positions[6]        <<std::endl;

        
        if(pub_form){
            // 通过rostopic发送出去
            std_msgs::Float64MultiArray msg;
            std::vector<double> data = {1.0, 2.0, 3.0, 4.0};
            msg.data = data;
            joint_values_pub.publish(msg);
        }
        else{   // 直接发送给moveit
            // 规划限制
            // move_group.setMaxVelocityScalingFactor(0.05);
            // move_group.setMaxAccelerationScalingFactor(0.05); 
            // arm.set_goal_joint_tolerance(0.001)
            // # arm.set_planner_id("RRTConnectkConfigDefault")
            // arm.set_planner_id("RRTstar")
            move_group.setGoalJointTolerance(0.01);
            move_group.setPlannerId("RRTstar");
            std::cout<<"设置joint values, "<<i<<std::endl;

            // 设置目标关节量
            move_group.setJointValueTarget(target_joint_group_positions);
            std::cout<<"开始执行,"<<i<<std::endl;
            move_group.move();
            std::cout<<"规划成功,"<<i<<std::endl;

            // 规划
            // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            // // moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
            // // ROS_INFO_NAMED("gpmp_realman", "Visualizing plan %d (joint space goal) %s", i,  success ? "" : "FAILED");
            // if(success){
            //     std::cout<<"规划成功"<<std::endl;
            //     // move_group.execute(my_plan);
            //     move_group.move();

            // }
            // else
            //     std::cout<<"规划失败"<<std::endl;
        }


        

    }

    return 0;
}