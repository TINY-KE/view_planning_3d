
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

// #include "sdf.h"

using namespace std;
using namespace gtsam;
using namespace gpmp2;

bool field_type = 0;

// 定义调试宏
// #define DEBUG

typedef ObstacleSDFFactor<ArmModel> ObstacleSDFFactorArm;
typedef ObstacleSDFFactorGP<ArmModel, GaussianProcessInterpolatorLinear>   ObstacleSDFFactorGPArm;


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
    // add_obstacle({200, 190, 145}, {40, 5, 190}, dataset);  //原程序中，额外添加的一块独立挡板
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

int main(int argc, char** argv){
    ros::init(argc, argv, "marker_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub_field = nh.advertise<visualization_msgs::Marker>("field", 1);
    ros::Publisher pub_sdf = nh.advertise<visualization_msgs::Marker>("sdf", 1);

    ros::Rate loop_rate(1);  // 设置发布频率



    // 一、构建机械臂模型
    // ArmModel* arm_model = generateArm("WAMArm");
    ArmModel* arm_model = generateArm("realman");
    
    #ifdef DEBUG
        std::cout<<"[debug arm]: 自由度"<<arm_model->dof()<<std::endl;
    #endif
    

    
    
    

    // 二、指定始末状态
    
    // gtsam::Vector start_conf = (Vector(7) << -0.0, 0.94, 0, 1.6, 0, -0.919, 1.55).finished();
    // gtsam::Vector end_conf = (Vector(7) << -0.8, -1.70, 1.64, 1.29, 1.1, -0.106, 2.2).finished();
    
    gtsam::Vector start_conf = (Vector(7) << -3.084730575741016, -1.763304599691998, 1.8552083929655296, 0.43301604856981246, -2.672461979658843, 0.46925065047728776, 4.000864693936108).finished();
    gtsam::Vector end_conf = (Vector(7) << -2.018431036907354, -1.4999897089911451, 1.4046777996889483, -1.3707039693409548, -3.0999924865261397, -0.8560425214202461, 4.8622166345079165).finished();

    gtsam::Vector start_vel = (Vector(7) << 0, 0, 0, 0, 0, 0, 0).finished();
    gtsam::Vector end_vel = (Vector(7) << 0, 0, 0, 0, 0, 0, 0).finished();

    auto jposes = arm_model->fk_model().forwardKinematicsPose(start_conf); //???
    Rot3 traj_orien = Rot3::Ypr(jposes(0, 6), jposes(1, 6), jposes(2, 6));
    //     jposes =

    //          0    0.0000    0.0000    3.1416    3.1416    3.1416    1.5718
    //          0    0.9400    0.9400    0.6016    0.6016    1.5206    0.0208
    //    -1.5708    0.0000   -1.5708    3.1416    1.5708    3.1416    1.6210
    //          0         0    0.4707    0.5078    0.6776    0.6776    0.7375
    //          0         0    0.0000    0.0000    0.0000    0.0000    0.0000
    //          0         0    0.2880    0.3135    0.0662    0.0662    0.0632

    // [
    //  	-0.00104354   0.0501717     0.99874;
    //     	   0.999784   0.0207948 5.22585e-17;
    //     	 -0.0207686    0.998524  -0.0501826
    //   ]
    #ifdef DEBUG
        std::cout<< "目标朝向偏转（ypr）："<<jposes(0, 6)<<","<< jposes(1, 6)<<","<<jposes(2, 6)<< std::endl;
        // 朝向角度：1.57184,0.0207701,1.62101  说明先再绕着z转了90度，又绕着x轴转了90度。
        std::cout<< "目标朝向矩阵："<<traj_orien << std::endl;
    #endif
    
    jposes = arm_model->fk_model().forwardKinematicsPose(end_conf);
    Pose3 end_pose = Pose3(Rot3::Ypr(jposes(0, 6), jposes(1, 6), jposes(2, 6)), Point3(jposes(3, 6), jposes(4, 6), jposes(5, 6)));
    // jposes =

    // -0.8000    2.3416    0.7619   -0.5183   -0.6841   -0.4912    2.6695
    //         0   -1.4416    0.0686   -0.1050    0.9866    0.9976   -0.0562
    // -1.5708    3.1416    3.0121   -1.6731    2.9563   -1.5943    1.5836
    //         0         0   -0.3475   -0.3864   -0.2358   -0.2358   -0.2085
    //         0         0    0.4222    0.4444    0.7021    0.7021    0.7555
    //         0         0   -0.0740   -0.0787   -0.1091   -0.1091   -0.1099

    // R:
    // [
    //     -0.889217  0.0558144   0.454069;
    //         0.454028  -0.014138   0.890875;
    //         0.0561433   0.998341 -0.0127696
    // ]
    // [-0.208508, 0.755514, -0.10991]';K>> 
    #ifdef DEBUG
        end_pose.print("终点位姿：\n");
        std::cout<<std::endl<<std::endl;
    #endif



    int total_time_sec = 2;
    int total_time_step = 10;
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
    

    std::cout<<"calculating signed distance field done"<<std::endl;
    // SignedDistanceField sdf(origin, cell_size, field);
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

    for (int z=0; z<300; z++) 
        for (int r=0; r<300; r++) 
            for (int c=0; c<300; c++) {
            // std::cout<<"point coordinate: "<<c<<","<<r<<","<<z<<std::endl;
            geometry_msgs::Point point;
            point.x = r*cell_size+origin.x();  // 假设矩阵中的数据是点的坐标
            point.y = c*cell_size+origin.y();
            point.z = z*cell_size+origin.z();
            if(dataset[z](r, c))
                marker.points.push_back(point);
    }
    std::cout<<"marker_field赋值完毕"<<std::endl;

    


    
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

        counter++;  // 每次触发定时器，计数器加1
        ros::spinOnce();
        loop_rate.sleep();
    }


    // 五、moveit控制及rviz可视化

    return 0;
}