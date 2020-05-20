#   安装依赖
```bash
sudo apt-get install ros-$ROS_DISTRO-arbotix ros-$ROS_DISTRO-openni-camera \
                     ros-$ROS_DISTRO-dynamixel-motor ros-$ROS_DISTRO-rosbridge-suite \
                     ros-$ROS_DISTRO-turtlebot-* ros-$ROS_DISTRO-kobuki-* ros-$ROS_DISTRO-moveit-python \
                     python-pygraph python-pygraphviz python-easygui \
                     mini-httpd ros-$ROS_DISTRO-laser-pipeline ros-$ROS_DISTRO-ar-track-alvar \
                     ros-$ROS_DISTRO-laser-filters ros-$ROS_DISTRO-rgbd-launch \
                     ros-$ROS_DISTRO-depthimage-to-laserscan ros-$ROS_DISTRO-shape-msgs \
                     ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-ros-pkgs \
                     ros-$ROS_DISTRO-gazebo-msgs ros-$ROS_DISTRO-gazebo-plugins \
                     ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-cmake-modules \
                     ros-$ROS_DISTRO-kobuki-gazebo-plugins ros-$ROS_DISTRO-kobuki-gazebo \
                     ros-$ROS_DISTRO-smach ros-$ROS_DISTRO-smach-ros ros-$ROS_DISTRO-grasping-msgs \
                     ros-$ROS_DISTRO-executive-smach ros-$ROS_DISTRO-smach-viewer \
                     ros-$ROS_DISTRO-robot-pose-publisher ros-$ROS_DISTRO-tf2-web-republisher \
                     ros-$ROS_DISTRO-move-base-msgs ros-$ROS_DISTRO-fake-localization \
                     graphviz-dev libgraphviz-dev gv python-scipy liburdfdom-tools \
                     ros-$ROS_DISTRO-laptop-battery-monitor ros-$ROS_DISTRO-ar-track-alvar* \
                     ros-$ROS_DISTRO-map-server ros-$ROS_DISTRO-move-base* \
                     ros-$ROS_DISTRO-simple-grasping ros-$ROS_DISTRO-manipulation-msgs
```

#   启动
```bash
#   启动turtlebot模型
roslaunch rbx2_tasks fake_turtlebot.launch 

#   打开rviz
rosrun rviz rviz -d `rospack find rbx2_tasks`/nav_tasks.rviz

#   启动smach可视化界面
rosrun smach_viewer smach_viewer.py
```

#   启动例程
```bash
rosrun rbx2_tasks patrol_smach_concurrence.py 
```
