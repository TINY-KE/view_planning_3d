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
#include "gtsam_quadrics/geometry/BboxCameraFactor.h"
#include "gtsam_quadrics/geometry/HeightCameraFactor.h"
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

gtsam::Values initCameraPose_my(std::vector<geometry_msgs::Pose> Candidates) {
    Values init_values;

    // init pose
    for (size_t i = 0; i < Candidates.size(); i++) {
        auto pose_eigen = Converter::geometryPosetoMatrix4d(Candidates[i]);
        Pose3 pose(pose_eigen);
        init_values.insert(Symbol('x', i), pose);
    }

    // init vel as avg vel
    // Vector avg_vel = (end_conf - init_conf) / static_cast<double>(total_step);
    // for (size_t i = 0; i <= total_step; i++)
    //     init_values.insert(Symbol('v', i), avg_vel);

    return init_values;
}


gtsam::Values initArmTrajStraightLine_my(const Vector& init_conf,
                                      const Vector& end_conf,
                                      size_t total_step) {
    Values init_values;

    // init pose
    for (size_t i = 0; i <= total_step; i++) {
        Vector conf;
        if (i == 0)
            conf = init_conf;
        else if (i == total_step)
            conf = end_conf;
        else
            conf =
                static_cast<double>(i) / static_cast<double>(total_step) * end_conf +
                (1.0 - static_cast<double>(i) / static_cast<double>(total_step)) *
                    init_conf;

        init_values.insert(Symbol('x', i), conf);
    }

    // init vel as avg vel
    // Vector avg_vel = (end_conf - init_conf) / static_cast<double>(total_step);
    // for (size_t i = 0; i <= total_step; i++)
    //     init_values.insert(Symbol('v', i), avg_vel);

    return init_values;
}

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
    string default_frame = "wam/base_link";

    Visualize_Tools *vis_tools = new Visualize_Tools(nh, default_frame);
    std::thread *mptVisualizeTools;
    mptVisualizeTools = new std::thread(&Visualize_Tools::Run, vis_tools);
    vis_tools->addMapObject(ob);

    Visualize_Arm_Tools vis_arm_tools(nh, *arm_model, move_group, CameraWidth, CameraHeight, Calib, default_frame);
    std::thread *mptVisualizeArmTools;
    mptVisualizeArmTools = new std::thread(&Visualize_Arm_Tools::Run, vis_arm_tools);


    //四、生成FootPrints候选点
    std::vector<geometry_msgs::Pose> FootPrints; //FootPrints候选位姿
    std::vector<geometry_msgs::Pose> Candidates = GenerateCandidates_ellipse_by_circle(*ob, FootPrints, 3.0, 10, 30);



    //五、确定起始位姿和中间差值
    gtsam::Values init_values = initCameraPose_my(Candidates);
    std::cout<<"FootPrints size:"<< FootPrints.size() <<std::endl;
    std::cout<<"init_values size:"<< init_values.size() <<std::endl;
    // init_values.print("init_values \n");

    // 六、各因子的启动参数

    // bbox和二次曲线
    double s = 100;
    gtsam_quadrics::AlignedBox2 gtsam_bbox(0+s, 0+s, CameraWidth-s, CameraHeight-s);   //预期的物体检测框
    double bbox_sigma = 0.1;   // bbox的权重
    double height_sigma = 0.001;   // bbox的权重


    // 六. 构建图
    // % algo settings
    std::vector<BboxCameraFactor>  bbox_factors;
    std::vector<BboxPlaneArmLinkFactor<ArmModel>>  lowplane_factors;


    NonlinearFactorGraph graph;
    for(int i=0; i<FootPrints.size(); i++){
        Key key_pos = symbol('x', i);
        Key key_vel = symbol('v', i);

        // if(i==0){
        //     // 起始位置约束
        //     graph.add(PriorFactor<Vector>(key_pos, start_conf,  noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
        //
        //     // 初始位置的速度
        //     graph.add(PriorFactor<Vector>(key_vel, start_vel,   noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
        // }
        // else if(i==FootPrints.size()-1){
        //     // 2.2 终止位置约束
        //     // goal pose for end effector in workspace
        //     // graph.add(GaussianPriorWorkspacePoseArm(key_pos, *arm_model, arm_model->dof()-1, end_pose, noiseModel::Isotropic::Sigma(6, end_pose_sigma)));
        //     graph.add(PriorFactor<Vector>(key_pos, end_conf,  noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
        //
        //     // fix goal velocity
        //     graph.add(PriorFactor<Vector>(key_vel, end_vel, noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
        // }
        // else
        {


            // BBOX和二次曲线
            Eigen::Matrix4f RobotPose = Eigen::Matrix4f::Identity();
            RobotPose = Converter::geometryPosetoMatrix4d(FootPrints[i]).cast<float>();;
            BboxCameraFactor factor(key_pos,
                           bbox_sigma,
                           gtsam_bbox,
                           ob,
                           RobotPose,
                           CameraWidth, CameraHeight, Calib );
            graph.add(factor);
            bbox_factors.push_back(factor);
            // factor_debug.visulize(start_conf);

            HeightCameraFactor factor2(key_pos,
                           height_sigma,
                           gtsam_bbox,
                           ob,
                           RobotPose,
                           CameraWidth, CameraHeight, Calib );
            graph.add(factor2);


        }


    }

    int opt_type = LM;
    if(argc==2) {
        double k = std::stof(argv[1]);
        if(k==1)
            opt_type = LM;
        else if(k==2)
            opt_type = GN;
        else if(k==3)
            opt_type = DOGLEG;
    }
    Values results;
    if(opt_type == LM){
        LevenbergMarquardtParams parameters;
        parameters.setVerbosity("ERROR"); // SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
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



//    // 七、第二步优化
//    int type_start = 0;
//    gtsam::Vector start_conf, end_conf;
//    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
//    // start_conf = stdVectorToGtsamVector(current_joint_values);
//    start_conf = (Vector(7) << 2.43078, -0.627707 ,  0.13426,   1.70191,  -0.10452,  -1.27042,   2.40398).finished();
//    // start_conf = (Vector(7) << 0,0,0,0,0,0,0).finished();
//    end_conf = (Vector(7) << 2.43078, -0.627707 ,  0.13426,   1.70191,  -0.10452,  -1.27042,   2.40398).finished();
//    // end_conf = (Vector(7) << 0,0,0,0,0,0,0).finished();
//
//    double total_time_step = FootPrints.size()-1;
//    double total_time_sec = 2 * (FootPrints.size()-1);
//    double check_inter = 5;
//    // 使用直线插值
//    gtsam::Values second_values = initArmTrajStraightLine_my(start_conf, end_conf, total_time_step);
//
//
//    // 八、设置第二步优化的参数
//    // 视场和机械臂本体
//    double obs_sigma = 0.005;   // 障碍物因子的权重
//    double epsilon_dist = 0.15;
//    //初始位姿
//    double pose_sigma = 1e-2; //固定的位姿，包括初始的位姿
//    double orien_sigma = 1e-2;  //过程中的方向。 TODO: 为什么是其他噪声模型的100倍？？？
//    // 机械臂之间的关节尽可能的小
//    Eigen::MatrixXd Qc = 1 * Eigen::MatrixXd::Identity(arm_model->dof(), arm_model->dof());
//    noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);// noiseModel是命名空间，Gaussian是类，Covariance是类的成员函数
//
//
//
//    //
//    NonlinearFactorGraph graph2;
//    for(int i=0; i<FootPrints.size(); i++){
//        Key key_pos = symbol('x', i);
//        Key key_vel = symbol('v', i);
//
//        // if(i==0){
//        //     // 起始位置约束
//        //     graph.add(PriorFactor<Vector>(key_pos, start_conf,  noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
//        //
//        //     // 初始位置的速度
//        //     graph.add(PriorFactor<Vector>(key_vel, start_vel,   noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
//        // }
//        // else if(i==FootPrints.size()-1){
//        //     // 2.2 终止位置约束
//        //     // goal pose for end effector in workspace
//        //     // graph.add(GaussianPriorWorkspacePoseArm(key_pos, *arm_model, arm_model->dof()-1, end_pose, noiseModel::Isotropic::Sigma(6, end_pose_sigma)));
//        //     graph.add(PriorFactor<Vector>(key_pos, end_conf,  noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
//        //
//        //     // fix goal velocity
//        //     graph.add(PriorFactor<Vector>(key_vel, end_vel, noiseModel::Isotropic::Sigma(arm_model->dof(), fix_sigma)));
//        // }
//        // else
//        {
//
//            auto target_camera_pose = results.at<Pose3>(symbol('x', i));
//            Eigen::Matrix4d T_cameralink_to_camera_matrix;
//            T_cameralink_to_camera_matrix << 0, 0, 1, 0.02,
//                                            -1, 0, 0, -0.013,
//                                            0, -1, 0, 0.0,  //实际为0.13，改为0.07
//                                            0, 0, 0, 1;
//            bbox_factors[i].visulize(target_camera_pose);
//            Eigen::Matrix4d target_cameralink_pose_matrix = target_camera_pose.matrix().cast<double>()      *    T_cameralink_to_camera_matrix.inverse();
//
//            Pose3 traj_pose = Pose3(target_cameralink_pose_matrix);
//            Rot3  traj_orien = traj_pose.rotation();
//
//            // GaussianPriorWorkspacePoseArm factor_pose(key_pos, *arm_model, arm_model->dof()-1, traj_pose, noiseModel::Isotropic::Sigma(6, pose_sigma));
//            // graph2.add(factor_pose);
//
//            GaussianPriorWorkspaceOrientationArm factor_ori(key_pos, *arm_model, arm_model->dof()-1, traj_orien, noiseModel::Isotropic::Sigma(3, orien_sigma));
//            graph2.add(factor_ori);
//        }
//
//
//    }
//
//
//    Values results2;
//    if(opt_type == LM){
//        LevenbergMarquardtParams parameters;
//        parameters.setVerbosity("ERROR"); // SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
//        // parameters.setAbsoluteErrorTol(1e-12);
//        parameters.setlambdaInitial(1000.0);
//        LevenbergMarquardtOptimizer optimizer(graph2, second_values, parameters);
//        // LevenbergMarquardtOptimizer类的定义
//        // #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
//        // virtual class LevenbergMarquardtOptimizer : gtsam::NonlinearOptimizer {
//        //     LevenbergMarquardtOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialValues);
//        //     LevenbergMarquardtOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialValues, const gtsam::LevenbergMarquardtParams& params);
//        //     double lambda() const;
//        //     void print(string str) const;
//        results2 = optimizer.optimize();
//    }
//    else if(opt_type == GN){
//        GaussNewtonParams parameters;
//        parameters.setVerbosity("ERROR");  //setVerbosity("TERMINATION"); //.setVerbosity("ERROR"); SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
//        GaussNewtonOptimizer optimizer(graph2, second_values, parameters);
//        results2 = optimizer.optimize();
//    }
//    else if(opt_type == DOGLEG){
//        DoglegParams parameters;
//        parameters.setVerbosity("ERROR");  //setVerbosity("TERMINATION"); //.setVerbosity("ERROR");
//        DoglegOptimizer optimizer(graph2, second_values, parameters);
//        results2 = optimizer.optimize();
//        // cout_results = optimizer.values();
//    }


//      九、moveit控制及rviz可视化

     // 关节量
     bool pub_form = true;
     std::vector<double > target_joint_group_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};;
     Vector target_joint_group_positions_eigen;
     for(int i=0; i<FootPrints.size(); i++){

         setPose(nh, "mrobot", FootPrints[i].position.x, FootPrints[i].position.y, FootPrints[i].position.z, FootPrints[i].orientation.w, FootPrints[i].orientation.x, FootPrints[i].orientation.y, FootPrints[i].orientation.z);

         // target_joint_group_positions.clear();
         // std::cout<<"开始规划,"<<i<<std::endl;

         auto target_camera_pose = results.at<Pose3>(symbol('x', i));
         Eigen::Matrix4d T_cameralink_to_camera_matrix;
         T_cameralink_to_camera_matrix << 0, 0, 1, 0.02,
                                         -1, 0, 0, -0.013,
                                         0, -1, 0, 0.0,  //实际为0.13，改为0.07
                                         0, 0, 0, 1;
         bbox_factors[i].visulize(target_camera_pose);
         auto target_camera_pose_ros = Converter::Matrix4dtoGeometryPose(target_camera_pose.matrix().cast<double>()
                                                                         *T_cameralink_to_camera_matrix.inverse());

         // plan 和 move
         move_group.setPoseTarget(target_camera_pose_ros);
         std::cout<<"相机高度："<< target_camera_pose_ros.position.z<<std::endl;
         moveit::planning_interface::MoveGroupInterface::Plan my_plan;
         bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
         if(success){

             std::vector<double> joint_angles = my_plan.trajectory_.joint_trajectory.points.back().positions;
             for (size_t i = 0; i < joint_angles.size(); ++i) {
                 ROS_INFO("Joint %lu: %f", i, joint_angles[i]);
             }
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

         std::cout << "Press [ENTER] to continue ... , [y] to autonomous mode" << std::endl;
         std::cout << "*****************************" << std::endl;
         char key = getchar();

     }

//    // 关节量
//    bool pub_form = true;
//    std::vector<double > target_joint_group_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};;
//    Vector target_joint_group_positions_eigen;
//    for(int i=0; i<FootPrints.size(); i++){
//
//        setPose(nh, "mrobot", FootPrints[i].position.x, FootPrints[i].position.y, FootPrints[i].position.z, FootPrints[i].orientation.w, FootPrints[i].orientation.x, FootPrints[i].orientation.y, FootPrints[i].orientation.z);
//
//        // target_joint_group_positions.clear();
//        // std::cout<<"开始规划,"<<i<<std::endl;
//
//        target_joint_group_positions_eigen = results2.at<Vector>(symbol('x', i));
//
//        target_joint_group_positions[0] = (double(target_joint_group_positions_eigen[0]));
//        target_joint_group_positions[1] = (double(target_joint_group_positions_eigen[1]));
//        target_joint_group_positions[2] = (double(target_joint_group_positions_eigen[2]));
//        target_joint_group_positions[3] = (double(target_joint_group_positions_eigen[3]));
//        target_joint_group_positions[4] = (double(target_joint_group_positions_eigen[4]));
//        target_joint_group_positions[5] = (double(target_joint_group_positions_eigen[5]));
//        target_joint_group_positions[6] = (double(target_joint_group_positions_eigen[6]));
//
//        std::cout<<"设置joint values, "<<i<<std::endl;
//        for (int i = 0; i < target_joint_group_positions.size(); i++){
//            ROS_INFO("   Joint %d: %f", i, target_joint_group_positions[i]);
//        }
//        // 设置目标关节量
//        move_group.setJointValueTarget(target_joint_group_positions);
//
//
//        // plan 和 move
//        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//        if(success){
//            std::cout<<"规划成功"<<std::endl;
//            move_group.execute(my_plan);
//
//        }
//        else
//            std::cout<<"规划失败"<<std::endl;
//
//        std::cout << "Press [any key] to continue ... " << std::endl;
//        std::cout << "*****************************" << std::endl;
//        char key = getchar();
//    }


    return 0;
}
