rosdep install --from-paths src --ignore-src -r -y

rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

git submodule add <-repo->

git submodule init

git submodule update

catkin build

. src/cartographer/scripts/install_abseil.sh