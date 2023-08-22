
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <cmath>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>



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

            // v1
            // Eigen::Vector3d v0(1,0,0);
            // Eigen::Vector3d v1(0,1,0);
            // Eigen::Vector3d v2(0,0,1);
            // double theta = acos(v.dot(v0));
            // Eigen::Vector3d axis = v.cross(v0);
            // q = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
            // v2:
            Eigen::Quaterniond quaternion;
            quaternion.setFromTwoVectors(Eigen::Vector3d::UnitX(), direction);
            q = quaternion;
            // q = Eigen::Quaterniond(0.0, direction.x(), direction.y(), direction.z());

            // Eigen::Vector3d v1_ = q * v1;
            // Eigen::Vector3d v2_ = q * v2;
            // double theta1 = acos(v1_.dot(v1));
            // double theta2 = acos(v2_.dot(v2));
            // if(theta1 > theta2){
            //     theta = theta2;
            // }
            // else{
            //     theta = theta1;
            // }
            // if(axis(2) < 0){
            //     theta = -theta;
            // }
            
            x = start(0);
            y = start(1);
            z = start(2);
        }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "candidate_ik_fabo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "arm";

  // 用想要控制的目标planning group，初始化.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // PlanningSceneInterface 用于在 virtual world" scene 添加或移除 物体
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  // 原始指针Raw pointers， 用来指代planning group（即此处的arm），以提高性能。
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);





  // Visualization
  // ^^^^^^^^^^^^^
  // MoveItVisualTools包提供了许多在RViz中可视化对象、机器人和轨迹的功能，以及调试工具，如脚本的逐步内省
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script via buttons and keyboard shortcuts in RViz、
  //  远程控制是一种内省工具，允许用户通过RViz中的按钮和键盘快捷键逐步完成高级脚本
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  // RViz提供了许多类型的标记，在这个演示中，我们将使用文本、圆柱体和球体
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Candidate ik fabo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  // 批量发布用于减少发送到RViz进行大型可视化的消息数量
  visual_tools.trigger();

  // Getting Basic Information
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("Candidate", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("Candidate", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("Candidate", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));



  // 添加物体
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "object";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.368233;
  primitive.dimensions[1] = 0.400311;
  primitive.dimensions[2] = 0.245264;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.68888;
  box_pose.position.y = -0.317092;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("Candidate", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);



  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 首先，我们将创建一个指针，引用当前机器人的状态。RobotState是包含所有当前位置/速度/加速度数据的对象
  // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  // move_group.setPlanningTime(10.0);

  // publisher
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("vector_marker", 10);
  ros::Publisher publisher_mam_rviz = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/local_nbv", 1000);
  
  // 代观测的物体
  object ob(  0.68888, -0.317092, 0.467195,
              0.368233, 0.400311, 0.245264,
              0.000000, 0.000000, 0.000000   );

  // 生成候选视点
  // 60度。未来如果60度达不到，则降低坡度。
  // 距离应该是根据fov的宽度选择
  double th = 45.0/180.0*M_PI; //60的坡度
  // 计算ob的对角线长度
  double ob_diag = 1.2 * sqrt(ob.l*ob.l + ob.w*ob.w + ob.h*ob.h);
  double l = ob_diag * cos(th);
  int num = 10;
  double dth = 360/(double)num /180*M_PI;
  std::vector<candidate> candidates;
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

  for (size_t i = 0; i < candidates.size(); i++)
  {
    // 设置运动学逆解的末端位姿
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = static_cast<double>(candidates[i].x);
    target_pose1.position.y = static_cast<double>(candidates[i].y);
    target_pose1.position.z = static_cast<double>(candidates[i].z);
    target_pose1.orientation.x = static_cast<double>(candidates[i].q.x());
    target_pose1.orientation.y = static_cast<double>(candidates[i].q.y());
    target_pose1.orientation.z = static_cast<double>(candidates[i].q.z());
    target_pose1.orientation.w = static_cast<double>(candidates[i].q.w());

    move_group.setPoseTarget(target_pose1);

    // 解算（plan）运动学逆解
    moveit::planning_interface::MoveGroupInterface::Plan candidate_plan;

    bool success = (move_group.plan(candidate_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("Candidate", "Visualizing plan %d (pose goal) %s", i,success ? "" : "FAILED");


    // We can also visualize the plan as a line with markers in RViz.
    // ROS_INFO_NAMED("Candidate", "Visualizing plan %d as trajectory line", i);
    // visual_tools.publishAxisLabeled(target_pose1, "pose1");
    // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    /* Uncomment below line when working with a real robot */
    // move_group.move(); 

  }
  

  ros::shutdown();
  return 0;
}
