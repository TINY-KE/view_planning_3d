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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Float64MultiArray.h"



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
#include "Converter.h"


#include "GenerateArm.h"
#include "Candidate.h"
#include "Obstacles.h"
#include "ConverterTools.h"


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



std::vector<candidate>  generate_candidates(const SdfObject& ob)
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
    ros::Publisher candidate_pub = nh.advertise<visualization_msgs::Marker>("/candidate", 10);
    ros::Publisher candidate_quaterniond_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("output", 1);


    // 启动movegroup
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    ros::Rate loop_rate(5);  // 设置发布频率

    // 零、生成候选点
    SdfObject ob(  0.68888+0.1, 0.0/* -0.317092 */, 0.467195-0.5,
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
        
        candidate_pub.publish(marker);
        
        geometry_msgs::PoseWithCovarianceStamped target;
        target.header.frame_id = "base_link";	//设置了消息的头部信息
        //通过将四元数的分量（x、y、z、w）设置为transDockPos变量中存储的旋转四元数分量。这些分量描述了箭头方向的旋转。
        target.pose.pose.orientation.x = candidates[i].oritention_quaterniond.x();
        target.pose.pose.orientation.y = candidates[i].oritention_quaterniond.y();
        target.pose.pose.orientation.z = candidates[i].oritention_quaterniond.z();
        target.pose.pose.orientation.w = candidates[i].oritention_quaterniond.w();
        target.pose.pose.position.x = candidates[i].centor_x;
        target.pose.pose.position.y = candidates[i].centor_y;
        target.pose.pose.position.z = candidates[i].centor_z;

        candidate_quaterniond_pub.publish(target);

        loop_rate.sleep();
    }




    // 一、构建机械臂模型
    ArmModel* arm_model = generateArm("WAMArm");
    
    // #ifdef DEBUG
    //     std::cout<<"[debug arm]: 自由度"<<arm_model->dof()<<std::endl;
    // #endif
    

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
        #ifdef DEBUG
            std::cout<<"候选点" << i <<"的目标朝向偏转（ypr）："<<candidate.yaw<<","<< candidate.pitch<<", 0"<< std::endl;
            i++;
            // 朝向角度：1.57184,0.0207701,1.62101  说明先再绕着z转了90度，又绕着x轴转了90度。
        #endif


        Rot3 traj_orien = Rot3::Ypr(candidate.yaw, candidate.pitch, 0.0);
        Pose3 traj_pose = Pose3(traj_orien, Point3(candidate.start.x(), candidate.start.y(), candidate.start.z()));

        candidates_rot3.push_back(traj_orien);
        candidates_pose3.push_back(traj_pose);
    }
    
    
    



    int total_time_sec = candidates.size() * 0.2; //2;
    int total_time_step = candidates.size();  //
    int check_inter = 0;
    double delta_t = real(total_time_sec) / real(total_time_step);
    double total_check_step = (check_inter + 1)*total_time_step;



    // 四、轨迹初值  TODO: 使用moveit compute_ik或者其他机器人工具箱，计算candidates的运动学逆解来作为轨迹初值。问题在于candidates大部分是无法解出逆解的，需要进行筛选
    // version1： 直接使用直线插值
    gtsam::Values init_values; // = gpmp2::initArmTrajStraightLine(start_conf, end_conf, total_time_step);
    // version2： 使用moveit compute_ik或者其他机器人工具箱
    //1.RobotModelPtr，robot_model指针
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model_RobotModelPtr = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model_RobotModelPtr->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state_RobotStatePtr(new robot_state::RobotState(kinematic_model_RobotModelPtr));
    kinematic_state_RobotStatePtr->setToDefaultValues();  //将机器人的状态都设为默认值，根据setToDefaultValues()定义，默认位置为0或者是最大界限和最小界限之间。
    const robot_state::JointModelGroup* joint_model_group = kinematic_model_RobotModelPtr->getJointModelGroup("arm");
    int candidate_num=0;
    init_values.clear();
    for(auto candidate: candidates){

        // 调用setFromIK解当前规划组arm的逆运动学问题，返回一个bool量。在解决该问题之前，需要如下条件：
        // end_effector_state: 末端执行器的期望位姿（一般情况下为当前规划组chain的最后一个连杆，本文为gripper_link）：也就是上面已经计算得到的齐次变换矩阵end_effector_state；
        Eigen::Isometry3d end_effector_state_my = Eigen::Isometry3d::Identity();
        Eigen::Vector3d translation(candidate.centor_x, candidate.centor_y, candidate.centor_z);
        end_effector_state_my.pretranslate(translation);
        Eigen::Matrix3d rotation_matrix = candidate.oritention_quaterniond.toRotationMatrix();
        end_effector_state_my.rotate(rotation_matrix);
        
        // 10：  尝试解决IK的次数
        // 0.1s：   每次尝试的时间
        std::size_t attempts = 4;
        double timeout = 0.1;
        std::cout<<"候选点"<<candidate_num+1<<"的位姿"
                            <<",x "<<candidate.centor_x
                            <<",y "<<candidate.centor_y
                            <<",z "<<candidate.centor_z
                            <<",qx "<<candidate.oritention_quaterniond.x()
                            <<",qy "<<candidate.oritention_quaterniond.y()
                            <<",qz "<<candidate.oritention_quaterniond.z()
                            <<",qw "<<candidate.oritention_quaterniond.w()<<std::endl;
        bool found_ik = kinematic_state_RobotStatePtr->setFromIK
                        (joint_model_group, end_effector_state_my, attempts, timeout);   //bug

        // 如果IK得到解，则驱动机器人按照计算得到的关节值进行运动，同时，打印计算结果。
        if (found_ik){
            std::vector<double> joint_values;
            kinematic_state_RobotStatePtr->copyJointGroupPositions(joint_model_group, joint_values);
            Eigen::VectorXd joint_values_gpmp(7), avg_vel_gpmp(7);
            ROS_INFO_STREAM("Find IK solution for "<<candidate_num<<" and success to store");
            std::cout<<"joint values: ";
            for(int i=0; i<joint_values.size(); i++){
                joint_values_gpmp[i] = joint_values[i];   
                std::cout<<joint_values[i]<<",";
                avg_vel_gpmp[i] = 0; 
            }
            std::cout<<std::endl;
            std::cout<<std::endl;
            
            init_values.insert(Symbol('x', candidate_num),  joint_values_gpmp);
            // TODO: 自己插入一个为0的速度。之后根据候选点ik之间的变化量，计算速度  Vector avg_vel = (end_conf - init_conf) / static_cast<double>(total_step);
            init_values.insert(Symbol('v', candidate_num),  avg_vel_gpmp);


        }else{
            ROS_INFO("Did not find IK solution");
        }
        candidate_num++;
    }
    ROS_ERROR_STREAM("Find IK: "<<candidate_num<<"/"<<candidates.size());

    return 0;
}