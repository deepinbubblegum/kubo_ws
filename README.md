https://drive.google.com/file/d/1LRjpliiMtxM9ad4My7fY5h6kTULXZPfR/view?usp=sharing

<!-- turtlebot3 install -->
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3

<!-- Move base flex turtlebot3 -->
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_mbf amcl_demo.launch
roslaunch turtlebot3_mbf rviz.launch