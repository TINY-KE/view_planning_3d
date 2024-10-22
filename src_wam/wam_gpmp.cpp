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
#include "Converter.h"

#include "gtsam/BboxEllipsoidFactor.h"
#include "gtsam/BboxPlaneArmLinkFactor.h"
#include "visualize_arm_tools.h"
#include "gazebo_rviz_tools.h"
#include <thread>
#include "MapObject.h"


using namespace std;
using namespace gtsam;
using namespace gpmp2;

bool field_type = 0;

// 定义调试宏
#define DEBUG

typedef ObstacleSDFFactor<ArmModel> ObstacleSDFFactorArm;
typedef ObstacleSDFFactorGP<ArmModel, GaussianProcessInterpolatorLinear> ObstacleSDFFactorGPArm;

enum pose_input_type {
    WAM_DEBUG = 0,
    GET_FROM_IK_CIRCLE = 1
};

int type = WAM_DEBUG;

enum opt_type {
    LM = 1, //LEVENBERG_MARQUARDT
    GN = 2, //GaussNewton
    DOGLEG = 3
};

int main(int argc, char **argv) {
    //  一、创建ROS和movegroup
    ros::init(argc, argv, "gpmp_wam", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Rate loop_rate(2); // 设置发布频率
    // 启动movegroup
    // 创建一个异步的自旋线程（spinning thread）
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    std::string end_effector_link = move_group.getEndEffectorLink();
    ROS_INFO_NAMED("WAM_arm", "End effector link: %s", end_effector_link.c_str());

    ROS_INFO_NAMED("WAM_arm", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    std::string pose_reference_frame = "/wam/base_link";
    // std::string pose_reference_frame="world";
    move_group.setPoseReferenceFrame(pose_reference_frame);
    ROS_INFO_NAMED("WAM_arm", "New Pose Reference frame: %s", move_group.getPoseReferenceFrame().c_str());

    //二、生成物体、机械臂、
    // （1）生成物体
    double x = 3, y = 0, z = 0.5;
    double lenth = 2.5, width = 2, height = 1;
    double yaw = 0;
    MapObject *ob = new MapObject();
    ob->Update_Twobj(x, y, z, yaw);
    ob->Update_object_size(lenth, width, height);
    ob->Update_corner();
    //（2）生成机械臂
    ArmModel *arm_model = generateArm("WAMArm");
    //（3）生成相机参数
    int CameraWidth = 640;
    int CameraHeight = 480;
    float fx = 554.254691191187;
    float fy = 554.254691191187;
    float cx = 320.5;
    float cy = 240.5;
    Eigen::Matrix3d Calib;
    Calib << fx, 0, cx,
            0, fy, cy,
            0, 0, 1;


    // 三、可视化线程
    string default_frame = "world";

    Visualize_Tools *vis_tools = new Visualize_Tools(nh, default_frame);
    std::thread *mptVisualizeTools;
    mptVisualizeTools = new std::thread(&Visualize_Tools::Run, vis_tools);

    Visualize_Arm_Tools vis_arm_tools(nh, *arm_model, move_group, CameraWidth, CameraHeight, Calib, default_frame);
    std::thread *mptVisualizeArmTools;
    mptVisualizeArmTools = new std::thread(&Visualize_Arm_Tools::Run, vis_arm_tools);


    //四、生成FootPrints候选点
    std::vector<geometry_msgs::Pose> FootPrints; //FootPrints候选位姿
    std::vector<geometry_msgs::Pose> Candidates = GenerateCandidates_ellipse_by_circle(*ob, FootPrints, 3.5);


    //五、确定起始位姿和中间差值
    int type_start = 0;
    gtsam::Vector start_conf, end_conf;
    gtsam::Vector start_vel, end_vel;
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    // start_conf = stdVectorToGtsamVector(current_joint_values);
    start_conf = (Vector(7) << 2.43078, -0.627707 ,  0.13426,   1.70191,  -0.10452,  -1.27042,   2.40398).finished();
    end_conf = (Vector(7) << 2.43078, -0.627707 ,  0.13426,   1.70191,  -0.10452,  -1.27042,   2.40398).finished();

    start_vel = (Vector(7) << 0, 0, 0, 0, 0, 0, 0).finished();
    end_vel = (Vector(7) << 0, 0, 0, 0, 0, 0, 0).finished();

    double total_time_sec = 2;
    double total_time_step = FootPrints.size();
    double check_inter = 5;
    double delta_t = total_time_sec / total_time_step;
    double total_check_step = (check_inter + 1.0)*total_time_step;
    // version1： 直接使用直线插值
    gtsam::Values init_values = gpmp2::initArmTrajStraightLine(start_conf, end_conf, total_time_step);


    // 六、各因子的启动参数
    // 视场和机械臂本体
    double obs_sigma = 0.005;   // 障碍物因子的权重
    double epsilon_dist = 0.1;
    // bbox和二次曲线
    double s = 100;
    gtsam_quadrics::AlignedBox2 gtsam_bbox(0+s, 0+s, CameraWidth-s, CameraHeight-s);   //预期的物体检测框
    double bbox_sigma = 0.1;   // bbox的权重
    //初始位姿
    double fix_sigma = 1e-4; //固定的位姿，包括初始的位姿
    double orien_sigma = 1e-2;  //过程中的方向。 TODO: 为什么是其他噪声模型的100倍？？？
    // 机械臂之间的关节尽可能的小
    Eigen::MatrixXd Qc = 0.1 * Eigen::MatrixXd::Identity(arm_model->dof(), arm_model->dof());
    noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);// noiseModel是命名空间，Gaussian是类，Covariance是类的成员函数


    // 六. 构建图
    // % algo settings

    NonlinearFactorGraph graph;
    for(int i=0; i<FootPrints.size(); i++){
        Key key_pos = symbol('x', i);
        Key key_vel = symbol('v', i);

        if(i==0){
            // 起始位置约束
            graph.add(PriorFactor<Vector>(key_pos, start_conf,  noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));

            // 初始位置的速度
            graph.add(PriorFactor<Vector>(key_vel, start_vel,   noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
        }
        else if(i==FootPrints.size()-1){
            // 2.2 终止位置约束
            // goal pose for end effector in workspace
            // graph.add(GaussianPriorWorkspacePoseArm(key_pos, *arm_model, arm_model->dof()-1, end_pose, noiseModel::Isotropic::Sigma(6, end_pose_sigma)));
            graph.add(PriorFactor<Vector>(key_pos, end_conf,  noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));

            // fix goal velocity
            graph.add(PriorFactor<Vector>(key_vel, end_vel, noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
        }
        else{
            // 下视场和机械臂本体
            BboxPlaneArmLinkFactor<ArmModel>(key_pos, *arm_model,obs_sigma, epsilon_dist,  CameraWidth, CameraHeight, Calib);

            // BBOX和二次曲线
            Eigen::Matrix4f RobotPose = Eigen::Matrix4f::Identity();
            // RobotPose = Converter::geometryPosetoMatrix4d(FootPrints[i]);
            BboxEllipsoidFactor<ArmModel>(key_pos, *arm_model, bbox_sigma, gtsam_bbox, ob, RobotPose, CameraWidth, CameraHeight,Calib);
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


    // int opt_type = LM;
    // Values results;
    // if(opt_type == LM){
    //     LevenbergMarquardtParams parameters;
    //     parameters.setVerbosity("ERROR"); // SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
    //     parameters.setAbsoluteErrorTol(1e-12);
    //     parameters.setlambdaInitial(1000.0);
    //     LevenbergMarquardtOptimizer optimizer(graph, init_values, parameters);
    //     // LevenbergMarquardtOptimizer类的定义
    //     // #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
    //     // virtual class LevenbergMarquardtOptimizer : gtsam::NonlinearOptimizer {
    //     //     LevenbergMarquardtOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialValues);
    //     //     LevenbergMarquardtOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialValues, const gtsam::LevenbergMarquardtParams& params);
    //     //     double lambda() const;
    //     //     void print(string str) const;
    //     results = optimizer.optimize();
    // }
    // else if(opt_type == GN){
    //     GaussNewtonParams parameters;
    //     parameters.setVerbosity("ERROR");  //setVerbosity("TERMINATION"); //.setVerbosity("ERROR"); SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
    //     GaussNewtonOptimizer optimizer(graph, init_values, parameters);
    //     results = optimizer.optimize();
    // }
    // else if(opt_type == DOGLEG){
    //     DoglegParams parameters;
    //     parameters.setVerbosity("ERROR");  //setVerbosity("TERMINATION"); //.setVerbosity("ERROR");
    //     DoglegOptimizer optimizer(graph, init_values, parameters);
    //     results = optimizer.optimize();
    //     // cout_results = optimizer.values();
    // }
    //
    // std::cout << "Optimization complete" << std::endl;
    // // std::cout << "results="<<std::endl << results << std::endl;
    // std::cout << "initial error=" << graph.error(init_values) << std::endl;
    // std::cout << "final error=" << graph.error(results) << std::endl;
    // std::cout << "Optimization Result:"  <<std::endl;
    // results.print();
    // std::cout << "Init values:"  <<std::endl;
    // init_values.print();


    // TrajOptimizerSetting opt_setting = TrajOptimizerSetting(arm_model->dof());
    // opt_setting.set_total_step(total_time_step);
    // opt_setting.set_total_time(total_time_sec);
    // opt_setting.set_epsilon(epsilon_dist);
    // opt_setting.set_cost_sigma(obs_sigma);
    // opt_setting.set_obs_check_inter(check_inter);
    // // opt_setting.set_conf_prior_model(noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma));
    // // opt_setting.set_vel_prior_model(noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma));
    // opt_setting.set_conf_prior_model(fix_sigma);
    // opt_setting.set_vel_prior_model(fix_sigma);
    // opt_setting.set_Qc_model(Qc);
    // if(CollisionCost3DArm(*arm_model, sdf, results, opt_setting)){
    //     std::cout<<"Trajectory is in collision!"<<std::endl;
    //     return 0;
    // }
    // else
    //     std::cout<<"Trajectory is collision free."<<std::endl;


    //  六、moveit控制及rviz可视化


    // move_group.setGoalJointTolerance(0.01);
    // move_group.setPlannerId("RRTstar");

    //
    // // 关节量
    // bool pub_form = true;
    // bool not_init_succuss = true;
    // for (int i = 0; i < Candidates.size(); i++) {
    //     move_group.setPoseTarget(Candidates[i]);
    //
    //     //print pose target
    //     std::cout << ">>>>>>> target pose " << i << ", x:" << Candidates[i].position.x << ", y:" << Candidates[i].
    //             position.y << ", z:" << Candidates[i].position.z
    //             << ", qx:" << Candidates[i].orientation.x << ", qy:" << Candidates[i].orientation.y << ", qz:" <<
    //             Candidates[i].orientation.z << ", qw:" << Candidates[i].orientation.w
    //             << std::endl;
    //
    //     // plan 和 move
    //     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //     setPose(nh, "mrobot", FootPrints[i].position.x, FootPrints[i].position.y, FootPrints[i].position.z,
    //             FootPrints[i].orientation.w, FootPrints[i].orientation.x, FootPrints[i].orientation.y,
    //             FootPrints[i].orientation.z);
    //     bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     if (success) {
    //         std::vector<double> joint_angles = my_plan.trajectory_.joint_trajectory.points.back().positions;
    //         for (size_t i = 0; i < joint_angles.size(); ++i) {
    //             ROS_INFO("Joint %lu: %f", i, joint_angles[i]);
    //         }
    //         std::cout << "<<<<<<<<<<<<<< 规划成功 \n" << std::endl;
    //         move_group.execute(my_plan);
    //         // loop_rate.sleep();
    //     } else {
    //         // Candidates[i].position.y += 0.3;
    //         // Candidates[i].position.z -= 0.1;
    //         // move_group.setPoseTarget(Candidates[i]);
    //         // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //
    //         // if(success){
    //         //     std::cout<<"<<<<<<<<<<<<<< 重新规划成功 \n"<<std::endl;
    //         //     move_group.execute(my_plan);
    //         // }
    //         // else
    //         std::cout << "<<<<<<<<<<<<<< 规划失败\n" << std::endl;
    //     }
    //     if (not_init_succuss) {
    //         std::cin.get();
    //         // not_init_succuss = false;
    //     }
    // }


    return 0;
}
