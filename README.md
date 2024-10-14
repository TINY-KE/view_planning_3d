# 常用指令
+ 启动gazebo和moveit
    source ws_gazebo_moveit/devel/setup.bash   &&  roslaunch wam_arm_moveit wam_kinectv1_bringup_moveit_onlyrobot.launch 
+ 视点规划
    rosrun view_planning  gpmp_wam_getfromik 

# 利用椭圆+圆+moveit ik，实现环绕观测
+ gpmp_wam_Reproduce_Matlab
    cpp复现matlab机械臂手臂朝上
+ gpmp_wam_getfromik_debug 
    完成了利用椭圆和圆（GenerateCandidates_ellipse_by_circle）实现候选点，并通过moveit ik 计算关节角度。

#  
+ 待：验证gpmp能否输入（GenerateCandidates_ellipse_by_circle）的结果，从而使的规划结果更顺滑。
+ 待：验证cube四方体8角与平面距离的error
+ 待：引入预设的椭球，
+ 待：暂时，在底盘的结果上加上扰动，代表底盘的运动。验证效果之后，再改回来

# 安装的依赖项
sudo apt-get install ros-noetic-rviz-visual-tools  ros-noetic-moveit-visual-tools


