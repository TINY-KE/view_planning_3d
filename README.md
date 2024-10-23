# 常用指令
+ 启动gazebo和moveit
    source ws_gazebo_moveit/devel/setup.bash   &&  roslaunch wam_arm_moveit wam_kinectv1_bringup_moveit_onlyrobot.launch 
+ 视点规划
    rosrun view_planning  gpmp_wam_getfromik 
+ rosrun tf tf_echo /wam/wrist_palm_link /camera_rgb_optical_frame
    At time 0.000
    - Translation: [0.020, -0.013, 0.130]
    - Rotation: in Quaternion [-0.500, 0.500, -0.500, 0.500]
                in RPY (radian) [-1.571, -0.000, -1.571]
                in RPY (degree) [-90.000, -0.000, -90.000]
        旋转矩阵 [0 0 1 0.02; -1 0 0 -0.013; 0 -1 0 0.13; 0 0 0 1]
        GTSAM:
        Rot3 R = Rot3::RzRyRx(-1.571,  -0.000, -1.571);
        Point3 t(0.020, -0.013, 0.130);
        Pose3 pose_camera_to_endlink(R, t);  // 第一个位姿 (T1)

# 利用椭圆+圆+moveit ik，实现环绕观测 commit 72c828759ba615f3a0c04c02b9d2e436e54966b6
+ gpmp_wam_Reproduce_Matlab
    cpp复现matlab机械臂手臂朝上
+ wam_getfromik_debug
    完成了利用椭圆和圆（GenerateCandidates_ellipse_by_circle）实现候选点，并通过moveit ik 计算关节角度。


# 准备融合g2o和gtsam     commit 7410dc96d6aaab6a2c1b2b8d82a7846a85f4583b
+ 因为wam机械臂会摔倒，因此引入了link wam/base_link的重量为1000
+ 添加Converter.cc  MapObject.cpp


# 第二次准备融合g2o和gtsam     commit 18cb4a9540ca6c993d42a0e3c0928603b907bc1d
+ 添加到

# 通过三角面可视化相机视场，并实现了点面距离的计算  commit b9a5d4d88226eb853f41507cc2122e989dc5ffd2
+ 删除了Thirdparty中的g2o，但是Ellipsoid.h还是到导入两个g2o头文件，但是我已经删除了啊？ 为什么还能找到并编译。
    #include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
    #include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
  姑且可以认为融合了g2o和gtsam
+ 实现了一点和相机视场平面（四个包围面）的距离计算；
+ 通过三角面，在rviz中可视化相机视场


# 计算点和视场包络面的距离，并构建误差因子  commit 211e71a33b8799756cd2af9a3b4b33c44e274f12
+ 建立专门的可视化线程
+ 实现了计算椭球体近端点与视场四个平面距离（四个平面的normal与相机z轴相同）的计算
+ 创建 BboxPlaneArmLink.h; ERROR为最小阈值距离减去各关节点与''视场下平面''的距离； ERROR导数与平面的normal相同；
    + 待： 构建视场下平面；


# 自制本体视场避遮挡gtsam因子, 可视化避障球 commit 375b6b6c578e97281db4b3c7431a8d30b6683b93
+ 验证BboxPlaneArmLink的Error基本正确
+ 修改WAM机械臂的DH参数和去掉了抓持爪子的避障球
+ 构建新类Visualize_Arm_Tools，可视化机械臂的避障球和各关节坐标   

# 自制本体视场和椭球体的gtsam因子, 失败  commit c177660de487b4f09f0f226999b70c40c0fb4c1e
+ 生成BboxPlaneEllipsoidFactor因子，
+ 在TestBboxPlaneArmLink.cpp中验证了bbox平面的准确性
+ 构建evaluateError。
+ 在Visualize_Arm_Tools，可视化bbox平面？？ 
+ 待解决 误差和偏导·+问题：引入lzw的椭球。椭球会不会和平面的距离计算函数（拉格朗日）会不会和gtsam冲突？

# 移植改造验证官方椭球体的gtsam因子，构建了BboxEllipsoidFactor commit 5aceced2d56ed9d8d39bb1e829a769bd30a4908c
+ 通过cv::imshow验证了ERROR的准确性
+ 验证了偏导(bbox_Error/Pose3)是4x6的矩阵
 *H1 = db_dC * dC_dx;
    + db_dC 为4x9, 误差向量 对 圆锥二次曲面参数（dual conic parameters） 的导数：
    + dC_dx 为9x6,  dual conic 参数 相对于 相机位姿（pose）参数 的导数：
+ 构建了BoundingBoxFactor因子，需要输入“机器人位姿”和“物体位姿”

# 对椭球轨迹进行优化
+ 编写wam_gpmp将我的三个创新点因子，联合优化，但有问题。
+ 问题分析：
    + BboxEllipsoidFactor： 雅可比链式法则太长
    + BboxPlaneArmLinkFactor： 雅可比推导失败



# 三维二次曲面 Class ConstrainedDualQuadric
+ 

# 平面二次曲线 Class DualConic
+ gtsam::Matrix33 dC_;  ///< 3x3 matrix of the quadratic equation
+ 

# 二次曲面的投影工具  Class QuadricCamera
+ QuadricCamera::project(quadric, camera_pose, calibration_, &dC_dq, dC_dx);
+ 其中 Eigen::Matrix<double, 9, 6> dC_dx 代表二次曲面对相机位姿的偏导？？？ 
+ 

#
+ 待：验证gpmp能否输入（GenerateCandidates_ellipse_by_circle）的结果，从而使的规划结果更顺滑。
+ 待：暂时，在底盘的结果上加上扰动，代表底盘的运动。验证效果之后，再改回来

# 安装的依赖项
sudo apt-get install ros-noetic-rviz-visual-tools  ros-noetic-moveit-visual-tools


# 待：删掉plane.h中对g2o库的调用