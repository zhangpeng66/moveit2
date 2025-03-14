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