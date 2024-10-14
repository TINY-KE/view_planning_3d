
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

// ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace gtsam;
using namespace gpmp2;

class Dataset {
    public:
        Dataset(){};
        ~Dataset(){};

    public:
        int cols;
        int rows;
        int z;
        float origin_x;
        float origin_y;
        float origin_z;
        float cell_size;
        // float map;
        std::vector<std::vector<std::vector<int>>> map;
        int corner_idx;
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
std::vector<std::vector<std::vector<int>>> add_obstacle(std::vector<int> position, std::vector<int> size, 
                        std::vector<std::vector<std::vector<int>>>& map, std::vector<std::vector<int>>& corner) {
    int half_size_row = (size[0] - 1) / 2;
    int half_size_col = (size[1] - 1) / 2;
    int half_size_z = (size[2] - 1) / 2;

    // occupancy grid
    for (int i = position[0] - half_size_row - 1; i < position[0] + half_size_row; i++) {
        for (int j = position[1] - half_size_col - 1; j < position[1] + half_size_col; j++) {
            for (int k = position[2] - half_size_z - 1; k < position[2] + half_size_z; k++) {
                map[i][j][k] = 1;
            }
        }
    }

    // corner
    std::vector<int> temp = {
        position[0] - half_size_row - 1,
        position[0] + half_size_row - 1,
        position[1] - half_size_col - 1,
        position[1] + half_size_col - 1,
        position[2] - half_size_z - 1,
        position[2] + half_size_z - 1
    };
    if (corner.empty()) {
        corner.push_back(temp);
    } else {
        corner.push_back(temp);
    }
}

void add_obstacle(std::vector<int> position, std::vector<int> size, std::vector<Matrix>& map){
    int half_size_row = floor((size[0] - 1) / 2);
    int half_size_col = floor((size[1] - 1) / 2);
    int half_size_z = floor((size[2] - 1) / 2);
    std::cout<<"sdf添加障碍物, size:"<<map.size()<<" "<<map[0].rows()<<" "<<map[0].cols()<<std::endl; ;
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

// Dataset generate3Ddataset(const std::string& dataset_str) {
//     Dataset dataset;
//     // dataset 2: desk dataset for WAM WAMDeskDataset
//     // params
//     dataset.cols = 300;
//     dataset.rows = 300;
//     dataset.z = 300;
//     dataset.origin_x = -1.5;
//     dataset.origin_y = -1.5;
//     dataset.origin_z = -1.5;
//     dataset.cell_size = 0.01;
//     // map
//     dataset.map = Eigen::ArrayXXXi::Zero(dataset.rows, dataset.cols, dataset.z);
//     // obstacles
//     dataset.corner_idx = Eigen::ArrayXXXi();
//     add_obstacle({170, 220, 130}, {140, 60, 5}, dataset.map, dataset.corner_idx);
//     add_obstacle({105, 195, 90}, {10, 10, 80}, dataset.map, dataset.corner_idx);
//     add_obstacle({235, 195, 90}, {10, 10, 80}, dataset.map, dataset.corner_idx);
//     add_obstacle({105, 245, 90}, {10, 10, 80}, dataset.map, dataset.corner_idx);
//     add_obstacle({235, 245, 90}, {10, 10, 80}, dataset.map, dataset.corner_idx);
//     add_obstacle({250, 190, 145}, {60, 5, 190}, dataset.map, dataset.corner_idx);
//     add_obstacle({250, 90, 145}, {60, 5, 190}, dataset.map, dataset.corner_idx);
//     add_obstacle({200, 190, 145}, {40, 5, 190}, dataset.map, dataset.corner_idx);
//     add_obstacle({250, 140, 240}, {60, 100, 5}, dataset.map, dataset.corner_idx);
//     add_obstacle({250, 140, 190}, {60, 100, 5}, dataset.map, dataset.corner_idx);
//     add_obstacle({250, 140, 140}, {60, 100, 5}, dataset.map, dataset.corner_idx);
//     add_obstacle({250, 140, 90}, {60, 100, 5}, dataset.map, dataset.corner_idx);
    
//     return dataset;
// }

double sdf_wrapper(const SignedDistanceField& field, const Point3& p) {
  return field.getSignedDistance(p);
}

void generate_sdf(){
    vector<Matrix> data;

    // params
    // data.resize(300);
    // Point3 origin(-0.2, -0.2, -0.1);
    // const double cell_size = 0.1;
    // const int cols = 5; 
    // const int rows = 5; 
    // const int heights = 3; 
    // Point3 origin(-1.5, -1.5, -1.5);
    // double cell_size = 0.01;
    Point3 origin(-1.5, -1.5, -1.5);
    double cell_size = 0.01;
    const int cols = 300;
    const int rows = 300;
    const int heights = 300;
    
    // map in form of matrix
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix;
    // Eigen::Matrix<double, rows, cols> matrix = Eigen::Matrix<double, rows, cols>::Zero();
    matrix = Eigen::MatrixXd::Zero(rows, cols);
    data.resize(heights);
    for (int i = 0; i < heights; i++) {
        data[i] = matrix;
    }
    // std::cout<<"[debug]:zere "<<matrix<<std::endl;


    // data[0] <<
    //     1.7321, 1.4142, 1.4142, 1.4142, 1.7321,
    //     1.4142, 1, 1, 1, 1.4142,
    //     1.4142, 1, 1, 1, 1.4142,
    //     1.4142, 1, 1, 1, 1.4142,
    //     1.7321, 1.4142, 1.4142, 1.4142, 1.7321;
    // data[1] <<
    //     // 1.4142, 1, 1, 1, 1.4142,
    //     0, 0, 0, 0, 0,
    //     1, 0, 0, 0, 1,
    //     1, 0, 0, 0, 1,
    //     1, 0, 0, 0, 1,
    //     1.4142, 1, 1, 1,4142;
    // data[2] <<
    //     1.7321, 1.4142, 1.4142, 1.4142, 1.7321,
    //     1.4142, 1, 1, 1, 1.4142,
    //     1.4142, 1, 1, 1, 1.4142,
    //     1.4142, 1, 1, 1, 1.4142,
    //     1.7321, 1.4142, 1.4142, 1.4142, 1.7321;
    // std::cout<<"[debug]:data 0 "<<data[0]<<std::endl;
    // std::cout<<"[debug]:data 1: "<<data[1]<<std::endl;
    // std::cout<<"[debug]:data 2 "<<data[2]<<std::endl;
    // int z = 1;
    // int i = 0;
    // for (int j = 0; j < 5; j++) {
    //     data[z](i,j) = j; 
    // }
    // std::cout<<"[debug]:data 1: "<<data[1]<<std::endl;

    std::cout<<"[debug]: construct sdf"<<std::endl;
    add_obstacle({170, 220, 130}, {140, 60, 5}, data);
    add_obstacle({105, 195, 90}, {10, 10, 80}, data);
    add_obstacle({235, 195, 90}, {10, 10, 80}, data);
    add_obstacle({105, 245, 90}, {10, 10, 80}, data);
    add_obstacle({235, 245, 90}, {10, 10, 80}, data);
    add_obstacle({250, 190, 145}, {60, 5, 190}, data);
    add_obstacle({250, 90, 145}, {60, 5, 190}, data);
    add_obstacle({200, 190, 145}, {40, 5, 190}, data);
    add_obstacle({250, 140, 240}, {60, 100, 5}, data);
    add_obstacle({250, 140, 190}, {60, 100, 5}, data);
    add_obstacle({250, 140, 140}, {60, 100, 5}, data);
    add_obstacle({250, 140, 90}, {60, 100, 5}, data);
    std::cout<<"[debug]: construct sdf End"<<std::endl;

    std::cout<<"sdf dataset: "<<std::endl;
    // for (int j = 220-1; j < 260; j++) {
    //     std::cout<<data[129](170,j)<<" "<<std::endl;
    // }
    for (int z = 130-1; z < 150; z++) {
        std::cout<<data[z](169,219)<<" "<<std::endl;
    }
    std::cout<<std::endl;

    // constructor
    SignedDistanceField sdf(origin, cell_size, data);

    // size
    std::cout<<"[debug]:sdf size "<<sdf.x_count()<<","<<sdf.y_count()<<","<<sdf.z_count()<<","<<std::endl;
    // EXPECT_LONGS_EQUAL(5, sdf.x_count());
    // EXPECT_LONGS_EQUAL(5, sdf.y_count());
    // EXPECT_LONGS_EQUAL(3, sdf.z_count());
    // EXPECT_DOUBLES_EQUAL(0.1, sdf.cell_size(), 1e-9);
    std::cout<<"[debug]:sdf cell size "<<sdf.cell_size()<<","<<std::endl;
    std::cout<<"[debug]:sdf origin"<<sdf.origin().x()<<","<<sdf.origin().y()<<","<<sdf.origin().z()<<","<<std::endl;
    // EXPECT(assert_equal(origin, sdf.origin()));

    // access
    SignedDistanceField::float_index idx;
    idx = sdf.convertPoint3toCell(Point3(0, 0, 0));
    std::cout<<"[debug]:idx "<<idx.get<0>()<<","<<idx.get<1>()<<","<<idx.get<2>()<<","<<std::endl;
    // EXPECT_DOUBLES_EQUAL(2, idx.get<0>(), 1e-9);
    // EXPECT_DOUBLES_EQUAL(2, idx.get<1>(), 1e-9);
    // EXPECT_DOUBLES_EQUAL(1, idx.get<2>(), 1e-9);
    // EXPECT_DOUBLES_EQUAL(0, sdf.signed_distance(idx), 1e-9)
    idx = sdf.convertPoint3toCell(Point3(0.18, -0.18, 0.07));   // tri-linear interpolation
    std::cout<<"[debug]:idx "<<idx.get<0>()<<","<<idx.get<1>()<<","<<idx.get<2>()<<","<<std::endl;
    // EXPECT_DOUBLES_EQUAL(0.2, idx.get<0>(), 1e-9);
    // EXPECT_DOUBLES_EQUAL(3.8, idx.get<1>(), 1e-9);
    // EXPECT_DOUBLES_EQUAL(1.7, idx.get<2>(), 1e-9);
    // EXPECT_DOUBLES_EQUAL(1.488288, sdf.signed_distance(idx), 1e-9)
    std::cout<<"[debug]:sdf signed distance "<<sdf.signed_distance(idx)<<","<<std::endl;
    idx = boost::make_tuple(1.0, 2.0, 3.0);
    // EXPECT(assert_equal(Point3(0.0, -0.1, 0.2), sdf.convertCelltoPoint3(idx)));

    // gradient
    Vector3 grad_act, grad_exp;
    Point3 p;
    p = Point3(-0.13, -0.14, 0.06);
    idx = sdf.convertPoint3toCell(p);   // tri-linear interpolation
    std::cout<<"[debug]:idx 3 "<<idx.get<0>()<<","<<idx.get<1>()<<","<<idx.get<2>()<<","<<std::endl;
    double signed_distance = sdf.getSignedDistance(p, grad_act);
    std::cout<<"[debug]:signed_distance 1: "<<sdf.getSignedDistance(Point3(-0.13, -0.14, 0.06))<<",,,,,,,,,"<<std::endl;
    std::cout<<"[debug]:signed_distance 2: "<<sdf.getSignedDistance(Point3(0,0, 0))<<",,,,,,,,,"<<std::endl;
    grad_exp = numericalDerivative11(boost::function<double(const Point3&)>(
        boost::bind(sdf_wrapper, sdf, _1)), p, 1e-6);
    // EXPECT(assert_equal(grad_exp, grad_act, 1e-6));

    p = Point3(0.18, 0.12, 0.01);
    signed_distance = sdf.getSignedDistance(p, grad_act);
    std::cout<<"[debug]:signed_distance "<<signed_distance<<","<<std::endl;
    grad_exp = numericalDerivative11(boost::function<double(const Point3&)>(
        boost::bind(sdf_wrapper, sdf, _1)), p, 1e-6);
    // EXPECT(assert_equal(grad_exp, grad_act, 1e-6));
    std::cout<<"[debug]:grad_act"<<grad_act<<std::endl;
    std::cout<<"[debug]:grad_exp"<<grad_exp<<std::endl;
}

int main(int argc, char** argv){
    // 一、构建机械臂模型
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
    ArmModel arm_model(abs_arm, body_spheres);
    std::cout<<"构建机械臂模型成功"<<std::endl;

    
    
    
    // 二、指定始末状态
    gtsam::Vector end_conf = (Vector(7) << -0.8, -1.70, 1.64, 1.29, 1.1, -0.106, 2.2).finished();
    gtsam::Vector start_conf = (Vector(7) << -0.0, 0.94, 0, 1.6, 0, -0.919, 1.55).finished();
    gtsam::Vector start_vel = (Vector(7) << 0, 0, 0, 0, 0, 0, 0).finished();
    gtsam::Vector end_vel = (Vector(7) << 0, 0, 0, 0, 0, 0, 0).finished();

    auto jposes = arm_model.fk_model().forwardKinematicsPose(start_conf); //???
    // std::cout<< jposes << std::endl;  //七个关节的位姿
    Rot3 traj_orien = Rot3::Ypr(jposes(0, 6), jposes(1, 6), jposes(2, 6));
    std::cout<< "朝向角度："<<jposes(0, 6)<<","<< jposes(1, 6)<<","<<jposes(2, 6)<< std::endl;
    // 朝向角度：1.57184,0.0207701,1.62101  说明先再绕着z转了90度，又绕着z轴转了90度。
    std::cout<< "朝向矩阵："<<traj_orien << std::endl;
    
    jposes = arm_model.fk_model().forwardKinematicsPose(end_conf);
    Pose3 end_pose = Pose3(Rot3::Ypr(jposes(0, 6), jposes(1, 6), jposes(2, 6)), Point3(jposes(3, 6), jposes(4, 6), jposes(5, 6)));

    int total_time_sec = 2;
    int total_time_step = 10;
    int check_inter = 5;
    double delta_t = real(total_time_sec) / real(total_time_step);
    double total_check_step = (check_inter + 1)*total_time_step;



    // 三、障碍物sdf

    generate_sdf();


    
    
    // 四、轨迹初值
    gtsam::Values init_values = gpmp2::initArmTrajStraightLine(start_conf, end_conf, total_time_step);
    // std::cout<<"轨迹初值，10个插值："<<std::endl;
    // init_values.print();



    // 五、优化
    // 1.传感器模型
    Eigen::MatrixXd Qc = 0.1 * Eigen::MatrixXd::Identity(arm_model.dof(), arm_model.dof());
    // std::cout<<"协方差矩阵："<<std::endl<<Qc<<std::endl;
    
    noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);// noiseModel是命名空间，Gaussian是类，Covariance是类的成员函数
    // std::cout<<"高斯噪声模型："<<std::endl<<Qc_model<<std::endl;

    // 2.图优化
    // % algo settings
    double obs_sigma = 0.005;
    double epsilon_dist = 0.15;
    double fix_sigma = 1e-4;
    double end_pose_sigma = 1e-4;
    double orien_sigma = 1e-2;

    NonlinearFactorGraph graph;
    for(int i=0; i<=total_time_step; i++){
        Key key_pos = Symbol('x', i);
        Key key_vel = Symbol('v', i);

        

        if(i==0){
            // 2.1 起始位置约束
            graph.add(PriorFactor<Vector>(key_pos, start_conf,  noiseModel::Isotropic::Sigma(arm_model.dof(), fix_sigma)));
            graph.add(PriorFactor<Vector>(key_vel, start_vel,   noiseModel::Isotropic::Sigma(arm_model.dof(), fix_sigma)));
        }
        else if(i==total_time_step){
            // 2.2 终止位置约束
            // goal pose for end effector in workspace
            graph.add(GaussianPriorWorkspacePoseArm(key_pos, arm_model, arm_model.dof()-1, end_pose, noiseModel::Isotropic::Sigma(arm_model.dof()-1, end_pose_sigma)));
            // 为什么是dof-1
            
            // fix goal velocity
            graph.add(PriorFactor<Vector>(key_vel, end_vel, noiseModel::Isotropic::Sigma(arm_model.dof(), fix_sigma)));
        }
        else{
            // 2.3 运动学约束 fix end effector orientation in workspace to be horizontal
            graph.add(GaussianPriorWorkspaceOrientationArm(key_pos, arm_model, arm_model.dof()-1, traj_orien, noiseModel::Isotropic::Sigma(3, orien_sigma)));
        }

        if(i>0){
            
        }
        
    }
    // matlab程序
    // parameters = LevenbergMarquardtParams;
    // parameters.setVerbosity('ERROR');
    // parameters.setlambdaInitial(1000.0);
    // optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters);

    bool type = false;
    Values results;
    if(type){
        LevenbergMarquardtParams parameters;
        parameters.setVerbosity("ERROR");  //setVerbosity("TERMINATION"); //.setVerbosity("ERROR");
        // parameters.setAbsoluteErrorTol(1e-12);
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
    }
    else{
        DoglegParams parameters;
        parameters.setVerbosity("ERROR");  //setVerbosity("TERMINATION"); //.setVerbosity("ERROR");
        DoglegOptimizer optimizer(graph, init_values, parameters);
        results = optimizer.optimize();
    }
    
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

    std::cout << "initial error=" << graph.error(init_values) << std::endl;
    std::cout << "final error=" << graph.error(results) << std::endl;


    // 五、moveit控制及rviz可视化

    return 0;
}