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


// gpmp的封装优化器
#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/planner/BatchTrajOptimizer.h>
// #include <gpmp2/kinematics/RobotModel.h>


// GPMP的官方因子图
#include <gpmp2/gp/GaussianProcessPriorLinear.h>   // 关节之间的连续性？  GP prior
#include <gpmp2/obstacle/ObstacleSDFFactor.h>// sdf
#include <gpmp2/obstacle/ObstacleSDFFactorGP.h>
#include <gpmp2/kinematics/GaussianPriorWorkspaceOrientationArm.h>
#include <gpmp2/kinematics/GaussianPriorWorkspacePoseArm.h>


// zhjd： arm轨迹优化专用
#include <gpmp2/planner/TrajUtils.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h>



#include <gpmp2/gp/GaussianProcessPriorPose2Vector.h>
#include <gpmp2/geometry/numericalDerivativeDynamic.h>



// ros
#include <ros/ros.h>




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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>



#include "GenerateArm.h"
#include "Candidate.h"
#include "Obstacles.h"
#include "ConverterTools.h"
#include "gazebo_rviz_tools.h"

using namespace std;
using namespace gtsam;
using namespace gpmp2;

bool field_type = 0;

// 定义调试宏
#define DEBUG

typedef ObstacleSDFFactor<ArmModel> ObstacleSDFFactorArm;
typedef ObstacleSDFFactorGP<ArmModel, GaussianProcessInterpolatorLinear>   ObstacleSDFFactorGPArm;

enum pose_input_type {
    WAM_DEBUG = 0,
    GET_FROM_IK_CIRCLE = 1
};

int type = WAM_DEBUG;

enum opt_type {
    LM = 1,  //LEVENBERG_MARQUARDT
    GN = 2,  //GaussNewton
    DOGLEG = 3
};

int main(int argc, char** argv){
    ros::init(argc, argv, "gpmp_wam", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    Visualize_Tools vis_tools(nh);
    ros::Rate loop_rate(2);  // 设置发布频率


    SdfObject ob( 3, 0, 0.5, 
                2.5, 2, 1, 
                0, 0, 0);
    std::vector<geometry_msgs::Pose> FootPrints;
    std::vector<geometry_msgs::Pose>  Candidates = GenerateCandidates_ellipse_by_circle(ob, FootPrints, 3.5);

    // while(ros::ok()){
    //     for(int i=0; i<Candidates.size(); i++){
    //         vis_tools.visualize_object(ob, "world");

    //         vis_tools.visualize_geometry_pose(Candidates[i], "world", i, "candidate"+to_string(i), true);
    //         ros::spinOnce();
    //         loop_rate.sleep();
    //     }
    // }


    // 零、启动movegroup
    // 创建一个异步的自旋线程（spinning thread）
	ros::AsyncSpinner spinner(1);
	spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    std::string end_effector_link=move_group.getEndEffectorLink();
	ROS_INFO_NAMED("WAM_arm", "End effector link: %s", end_effector_link.c_str());
    	
    ROS_INFO_NAMED("WAM_arm", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    std::string pose_reference_frame="/wam/base_link";
    // std::string pose_reference_frame="world";
    move_group.setPoseReferenceFrame(pose_reference_frame);
	ROS_INFO_NAMED("WAM_arm", "New Pose Reference frame: %s", move_group.getPoseReferenceFrame().c_str());

    // move_group.setGoalJointTolerance(0.01);
    // move_group.setPlannerId("RRTstar");



    // 一、构建机械臂模型
    ArmModel* arm_model = generateArm("WAMArm");
    

    // 二、指定始末状态
    // TODO:  用moveit计算初始位置，当前先使用全0的初始值
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    gtsam::Vector start_conf = stdVectorToGtsamVector(current_joint_values);
    
    move_group.setPoseTarget(Candidates[i]);
    //print pose target
    std::cout<<">>>>>>> target pose "<< i<<", x:"<<Candidates[i].position.x<<", y:"<<Candidates[i].position.y<<", z:"<<Candidates[i].position.z
            <<", qx:"<<Candidates[i].orientation.x<<", qy:"<<Candidates[i].orientation.y<<", qz:"<<Candidates[i].orientation.z<<", qw:"<<Candidates[i].orientation.w
            <<std::endl;

    // plan 和 move
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    setPose(nh, "mrobot", FootPrints[i].position.x, FootPrints[i].position.y, FootPrints[i].position.z, FootPrints[i].orientation.w, FootPrints[i].orientation.x, FootPrints[i].orientation.y, FootPrints[i].orientation.z);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
        std::cout<<"<<<<<<<<<<<<<< 规划成功 \n"<<std::endl;
        move_group.execute(my_plan);
        // loop_rate.sleep();
    gtsam::Vector start_conf = (Vector(7) << 1.941154, -0.644881, 0.063626, 1.696963, -3.159852, 1.301093, -0.217955).finished();
    gtsam::Vector end_conf = (Vector(7) << -0.0,0.94,0,1.6,0,-0.919,1.55).finished();

    gtsam::Vector start_vel = (Vector(7) << 0, 0, 0, 0, 0, 0, 0).finished();
    gtsam::Vector end_vel = (Vector(7) << 0, 0, 0, 0, 0, 0, 0).finished();  
    
    auto jposes = arm_model->fk_model().forwardKinematicsPose(start_conf); 
    Rot3 traj_orien = Rot3::Ypr(jposes(0, 6), jposes(1, 6), jposes(2, 6));
    jposes = arm_model->fk_model().forwardKinematicsPose(end_conf);
    Pose3 end_pose = Pose3(Rot3::Ypr(jposes(0, 6), jposes(1, 6), jposes(2, 6)), Point3(jposes(3, 6), jposes(4, 6), jposes(5, 6)));


    // 三、障碍物sdf


    

    

    // 四、轨迹初值  
    double total_time_sec = 2;
    double total_time_step = Candidates.size();
    double check_inter = 5;
    double delta_t = total_time_sec / total_time_step;
    double total_check_step = (check_inter + 1.0)*total_time_step;
    // version1： 直接使用直线插值
    gtsam::Values init_values = gpmp2::initArmTrajStraightLine(start_conf, end_conf, total_time_step);
    


    // 五、优化
    // 1.传感器模型
    Eigen::MatrixXd Qc = 0.1 * Eigen::MatrixXd::Identity(arm_model->dof(), arm_model->dof());
    noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);// noiseModel是命名空间，Gaussian是类，Covariance是类的成员函数


    // 2.图优化
    // % algo settings
    double obs_sigma = 0.005; 
    double epsilon_dist = 0.15; 
    double fix_sigma = 1e-4; //固定的位姿，包括初始的位姿
    double init_pose_sigma = 10.0/100.0;  //过程中的位姿
    double end_pose_sigma = 1e-4; //固定的位姿，包括结束时的位姿

    double orien_sigma = 1e-2;  //过程中的方向。 TODO: 为什么是其他噪声模型的100倍？？？
    
    NonlinearFactorGraph graph;
    for(int i=0; i<total_time_step; i++){
        Key key_pos = symbol('x', i);
        Key key_vel = symbol('v', i);
        
        if(i==0){
            // 2.1 起始位置约束
            // 删除的原因：原程序是直接限制了关节的角度，这个可以在我之后的程序中开启，
            graph.add(PriorFactor<Vector>(key_pos, start_conf,  noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
        
            // 初始位置的速度
            graph.add(PriorFactor<Vector>(key_vel, start_vel,   noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
        }
        else if(i==total_time_step){
            // 2.2 终止位置约束
            // goal pose for end effector in workspace
            graph.add(GaussianPriorWorkspacePoseArm(key_pos, *arm_model, arm_model->dof()-1, end_pose, noiseModel::Isotropic::Sigma(6, end_pose_sigma)));
                        
            // fix goal velocity
            graph.add(PriorFactor<Vector>(key_vel, end_vel, noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
        }
        else{
            // 2.3 运动学约束 fix end effector orientation in workspace to be horizontal
            graph.add(GaussianPriorWorkspaceOrientationArm(key_pos, *arm_model, arm_model->dof()-1, traj_orien, noiseModel::Isotropic::Sigma(3, orien_sigma)));
        }

        if(i>0){
            // 初始化变量
            Key key_pos1 = symbol('x', i-1);
            Key key_pos2 = symbol('x', i);
            Key key_vel1 = symbol('v', i-1);
            Key key_vel2 = symbol('v', i);
            
            // % GP prior
            // 学习/home/zhjd/work/gpmp2/gpmp2/gp/tests/testGaussianProcessPriorLinear.cpp
            graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model));
            
            // // % unary obstacle factor
            // graph.add(ObstacleSDFFactorArm(key_pos, *arm_model, sdf, obs_sigma, epsilon_dist));
            
            // // % interpolated obstacle factor
            // if(check_inter){
            //     for(int j=1; j<=check_inter; j++){
            //         double tau = j * (total_time_sec / total_check_step);
            //         graph.add(ObstacleSDFFactorGPArm(key_pos1, key_vel1, key_pos2, key_vel2, *arm_model, sdf, obs_sigma, epsilon_dist, Qc_model, delta_t, tau));
            //     }
            // }
        }
        
    }
    

    int opt_type = LM;
    Values results;
    if(opt_type == LM){
        LevenbergMarquardtParams parameters;
        parameters.setVerbosity("ERROR"); // SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
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
        results = optimizer.optimize();   
    }
    else if(opt_type == GN){
        GaussNewtonParams parameters;
        parameters.setVerbosity("ERROR");  //setVerbosity("TERMINATION"); //.setVerbosity("ERROR"); SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
        GaussNewtonOptimizer optimizer(graph, init_values, parameters);
        results = optimizer.optimize();
    }
    else if(opt_type == DOGLEG){
        DoglegParams parameters;
        parameters.setVerbosity("ERROR");  //setVerbosity("TERMINATION"); //.setVerbosity("ERROR");
        DoglegOptimizer optimizer(graph, init_values, parameters);
        results = optimizer.optimize();
        // cout_results = optimizer.values();
    }    

    std::cout << "Optimization complete" << std::endl;
    // std::cout << "results="<<std::endl << results << std::endl;
    std::cout << "initial error=" << graph.error(init_values) << std::endl;
    std::cout << "final error=" << graph.error(results) << std::endl;
    std::cout << "Optimization Result:"  <<std::endl;
    results.print();
    std::cout << "Init values:"  <<std::endl;
    init_values.print();

    










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




    //  六、moveit控制及rviz可视化
    
    


    // 关节量 
    bool pub_form = true;
    bool not_init_succuss = true;
    for(int i=0; i<Candidates.size(); i++){


        move_group.setPoseTarget(Candidates[i]);

        //print pose target
        std::cout<<">>>>>>> target pose "<< i<<", x:"<<Candidates[i].position.x<<", y:"<<Candidates[i].position.y<<", z:"<<Candidates[i].position.z
                <<", qx:"<<Candidates[i].orientation.x<<", qy:"<<Candidates[i].orientation.y<<", qz:"<<Candidates[i].orientation.z<<", qw:"<<Candidates[i].orientation.w
                <<std::endl;

        // plan 和 move
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        setPose(nh, "mrobot", FootPrints[i].position.x, FootPrints[i].position.y, FootPrints[i].position.z, FootPrints[i].orientation.w, FootPrints[i].orientation.x, FootPrints[i].orientation.y, FootPrints[i].orientation.z);
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success){
            std::cout<<"<<<<<<<<<<<<<< 规划成功 \n"<<std::endl;
            move_group.execute(my_plan);
            // loop_rate.sleep();
        }
        else{
            // Candidates[i].position.y += 0.3;
            // Candidates[i].position.z -= 0.1;
            // move_group.setPoseTarget(Candidates[i]);
            // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            
            // if(success){
            //     std::cout<<"<<<<<<<<<<<<<< 重新规划成功 \n"<<std::endl;
            //     move_group.execute(my_plan);
            // }
            // else
                std::cout<<"<<<<<<<<<<<<<< 规划失败\n"<<std::endl;
        }
        if(not_init_succuss){
            std::cin.get();
            // not_init_succuss = false;
        }
    }
            





    return 0;
}