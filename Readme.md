rosdep install --from-paths src --ignore-src -r -y

rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

git submodule add https://github.com/rst-tu-dortmund/teb_local_planner.git

git submodule init

git submodule update

catkin build