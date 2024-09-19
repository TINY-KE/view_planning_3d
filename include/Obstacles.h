#include <iostream>
#include <cmath>
#include <cmath>
#include <algorithm>
#include <CppUnitLite/TestHarness.h>

// gtsam插件
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>


using namespace std;
using namespace gtsam;
using namespace gpmp2;

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
