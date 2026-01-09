
sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default2 https://gitee.com/zhenshenglee/colcon-mixin-repository/raw/master/index.yaml
colcon mixin update default2

cd src
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y(鱼香肉丝 自动安装依赖)

colcon build --mixin release --parallel-workers 8
colcon build --mixin release --executor sequential

wget https://github.com/ricab/scope_guard/blob/main/scope_guard.hpp
ros2 run hello_moveit hello_moveit --ros-args --log-level DEBUG
moveit_ros_planning_interface
pandoc input.rst -f rst -t markdown -o output.md
ros2 pkg create <package_name> --dependencies rclcpp std_msgs

gz sdf -p robot.urdf > robot.sdf
colcon build --cmake-args -DBUILD_TESTING=ON

git submodule add -b humble https://github.com/moveit/moveit2.git
如果目录存在.gitmodules则
git submodule add -f -b humble https://github.com/moveit/moveit2.git
git push origin master 
更新子库
git submodule update init

colcon build --packages-select moveit_ros_planning_interface
//1、在仿真环境下运行
ros2 launch jaka_zu5_gazebo jaka_zu5_bringup_gazebo.launch.py
//2、启动move_group节点和move_group_interface节点
ros2 launch moveit_arm_control run_move_group_interface.launch.py
//发送笛卡尔坐标系
ros2 service call /move_to_location_position shared_message/srv/MoveToLocationPosition "{pose:[0,0,-0.013,0,0,0],loc_type: 0}"
ros2 service call /move_to_location_position shared_message/srv/MoveToLocationPosition "{pose:[-0.082,0.014,-0.300,0,0,0],loc_type: 0}"
//发送关节坐标
ros2 service call /move_to_location_position shared_message/srv/MoveToLocationPosition "{pose:[-4.108, 1.565, 1.689, 3.633, -5.367, 1.120],loc_type: 2}"
//发送tcp目标
ros2 service call /move_to_location_position shared_message/srv/MoveToLocationPosition "{pose:[0.084,-0.0092,0.119,3.137,3.135,3.13],loc_type: 1}"
接受到消息必须两个lanch同时运行
ros2 launch moveit_arm_control move_group_interface_cpp.launch.py
ros2 launch moveit_arm_control run_move_group.launch.py
带有debug信息
colcon build --packages-select moveit_ros_move_group --cmake-args -DCMAKE_BUILD_TYPE=Debug
节点不显示的时候
ros2 node list --no-daemon