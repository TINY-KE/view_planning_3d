# 常用指令
+ 启动gazebo和moveit
    source ws_gazebo_moveit/devel/setup.bash   &&  roslaunch wam_arm_moveit wam_kinectv1_bringup_moveit_onlyrobot.launch 
+ 视点规划
    rosrun view_planning  gpmp_wam_getfromik 


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


# 计算平面和视场包络面的距离
+ 建立专门的可视化线程
+ 实现了计算椭球体近端点与视场四个平面距离（四个平面的normal与相机z轴相同）的计算
+ 创建 BboxPlaneArmLink.h; ERROR为最小阈值距离减去各关节点与''视场下平面''的距离； ERROR导数与平面的normal相同；
    + 待： 构建视场下平面；


# 自制gtsam因子 
+ 待：验证BboxPlaneArmLink
+ 问题：引入lzw的椭球。椭球会不会和平面的距离计算函数（拉格朗日）会不会和gtsam冲突？
+ 构建BboxPlaneEllipsoid


+ 待：验证gpmp能否输入（GenerateCandidates_ellipse_by_circle）的结果，从而使的规划结果更顺滑。
+ 待：暂时，在底盘的结果上加上扰动，代表底盘的运动。验证效果之后，再改回来

# 安装的依赖项
sudo apt-get install ros-noetic-rviz-visual-tools  ros-noetic-moveit-visual-tools


