#include <iostream>
#include <cmath>
#include <cmath>
#include <algorithm>
#include <CppUnitLite/TestHarness.h>
#include <thread>

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


// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "Viewer.h"
#include "MapObject.h"
#include "GenerateArm.h"
#include "ConverterTools.h"
#include "Converter.h"
#include "ViewPlanning.h"
#include "Map.h"
#include "GazeboTools.h"
#include "TeleopMobile.h"

class ObjectMap;


using namespace std;
using namespace gtsam;
using namespace gpmp2;

bool field_type = 0;

// 定义调试宏
#define DEBUG


enum pose_input_type {
    WAM_DEBUG = 0,
    GET_FROM_IK_CIRCLE = 1
};

int type = WAM_DEBUG;



int main(int argc, char **argv) {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);  // 参看：https://www.jianshu.com/p/bb128a6b413e
    memcpy(&raw, &cooked, sizeof(struct termios)); // 将cooked中的内容拷贝到raw中
    raw.c_lflag &=~ (ICANON | ECHO);
    //将ICANON和ECHO按位或“|” 之后取反，再将raw.c_lflag和其相与，将结果赋值给raw.c_lflag
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    //raw.c_cc是控制字符  https://www.gwduan.com/web/computer/history/unix-prog/terminal.html#special-char
    tcsetattr(kfd, TCSANOW, &raw);

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
    std::vector<double> init_joint_group = {0, 0, 0, 0.17, 0, 0, 0};;
    move_group.setJointValueTarget(init_joint_group);
    moveit::planning_interface::MoveGroupInterface::Plan init_plan;
    bool success = (move_group.plan(init_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.execute(init_plan);


    //二、生成机械臂、相机参数
    //（1）生成机械臂
    ArmModel *arm_model = generateArm("WAMArm");
    //（2）生成相机参数
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
    // (3) 规划轨迹中差值的数量
    int circle_divides = 240;
    // （4）最佳视场
    int FovDecrease = 120;
    // 地图
    ObjectMap *map = new ObjectMap(nh);
    std::thread *mptMap;
    mptMap = new std::thread(&ObjectMap::Run, map);

    // 三、键盘线程
    TeleopMobile *teleop_mobile_tool = new TeleopMobile(nh, 0, 0, 0, 0, 0, 0);
    std::thread *mptTeleopMobile;
    mptTeleopMobile = new std::thread(&TeleopMobile::Run, teleop_mobile_tool);

    // 三、可视化线程
    string default_frame = "wam/base_link";

    Visualize_Tools *vis_tools = new Visualize_Tools(nh, map, default_frame);
    std::thread *mptVisualizeTools;
    mptVisualizeTools = new std::thread(&Visualize_Tools::Run, vis_tools);

    Visualize_Arm_Tools vis_arm_tools(nh, *arm_model, move_group, CameraWidth, CameraHeight, Calib, default_frame);
    std::thread *mptVisualizeArmTools;
    mptVisualizeArmTools = new std::thread(&Visualize_Arm_Tools::Run, vis_arm_tools);
    vis_arm_tools.setFOVDecrease(FovDecrease);


    //四、生成ViewPlanning
    ViewPlanningTool view_planning(arm_model, circle_divides, CameraWidth, CameraHeight, FovDecrease, Calib, vis_tools);


    while (ros::ok()) {
        // std::cout << "Press [any key] to start a view-planning... " << std::endl;
        // std::cout << "*****************************" << std::endl;

        char key;
        read(kfd, &key, 1);

        if (key == KEYCODE_s) {
            std::string traj_save_path = "/home/robotlab/ws_3d_vp/src/view_planning_3d/";
            map->save_traj_node(traj_save_path);
        }

        if (key != KEYCODE_ENTER)
            continue;

        std::cout << "Start a view-planning... " << std::endl;
        // 五、生成FootPrints候选点
        auto objects = map->getMapObjects();

        MapObject *target_object;
        double min_dis = 10000;
        geometry_msgs::Pose robot_pose = getPose(nh, "mrobot");
        double robot_pose_x = robot_pose.position.x, robot_pose_y = robot_pose.position.y;
        for(auto ob: objects) {
            if(ob->explored)
                continue;

            double dis = sqrt( (robot_pose_x - ob->mCuboid3D.cuboidCenter.x()) * (robot_pose_x - ob->mCuboid3D.cuboidCenter.x()) +
                                    (robot_pose_y - ob->mCuboid3D.cuboidCenter.y()) * (robot_pose_y - ob->mCuboid3D.cuboidCenter.y())
                            );
            std::cout<<"object-robot dis: "<<dis<<std::endl;
            if(dis<min_dis) {
                min_dis = dis;
                target_object = ob;
            }
        }


        Values arm_results;
        std::vector<geometry_msgs::Pose> FootPrints;
        view_planning.planning(target_object, robot_pose_x, robot_pose_y, arm_results, FootPrints);

        // 确认是否抵达起点
        key = '!';
        read(kfd, &key, 1);

        if (key == KEYCODE_ENTER) {
            // 六、原地旋转
            int rotate_divides = 90;
            bool turn_left = true;
            rotate_90degrees(nh, move_group, FootPrints[0], turn_left, rotate_divides);


            // 九、moveit控制及rviz可视化

            //高度
            double use_visulize = 1;
            std::vector<double> target_joint_group_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};;
            Vector target_joint_group_positions_eigen;
            for (int i = 0; i < FootPrints.size(); i++) {
                setPose(nh, "mrobot", FootPrints[i].position.x, FootPrints[i].position.y, FootPrints[i].position.z,
                        FootPrints[i].orientation.w, FootPrints[i].orientation.x, FootPrints[i].orientation.y,
                        FootPrints[i].orientation.z);

                // target_joint_group_positions.clear();
                // std::cout<<"开始运动执行,"<<i<<std::endl;


                target_joint_group_positions_eigen = arm_results.at<Vector>(symbol('x', i));

                if (use_visulize) {
                    view_planning.visualize(i, target_joint_group_positions_eigen);
                }

                target_joint_group_positions[0] = (double(target_joint_group_positions_eigen[0]));
                target_joint_group_positions[1] = (double(target_joint_group_positions_eigen[1]));
                target_joint_group_positions[2] = (double(target_joint_group_positions_eigen[2]));
                target_joint_group_positions[3] = (double(target_joint_group_positions_eigen[3]));
                target_joint_group_positions[4] = (double(target_joint_group_positions_eigen[4]));
                target_joint_group_positions[5] = (double(target_joint_group_positions_eigen[5]));
                target_joint_group_positions[6] = (double(target_joint_group_positions_eigen[6]));

                std::cout << "设置joint values, " << i << std::endl;
                for (int j = 0; j < target_joint_group_positions.size(); j++) {
                    ROS_INFO("   Joint %d: %f", j, target_joint_group_positions[j]);
                }
                // 设置目标关节量
                move_group.setJointValueTarget(target_joint_group_positions);

                // plan 和 move
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if (success) {
                    std::cout << "Joint运动执行成功" << std::endl;
                    move_group.execute(my_plan);
                } else
                    std::cout << "Joint运动执行失败" << std::endl;
            }

            // 标记物体已经explore完毕
            target_object->explored = true;

            // 转身回来
            rotate_divides = 90;
            turn_left = false;
            rotate_90degrees(nh, move_group, FootPrints[0], turn_left, rotate_divides);

            
        }

    }

    // 等待 ROS 节点关闭（当 Ctrl+C 被捕获时）
    // ros::waitForShutdown();

    return 0;
}




