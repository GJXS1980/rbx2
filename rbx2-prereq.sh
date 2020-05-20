#!/bin/sh

# Install the prerequisites for the ROS By Example code, Volume 2

sudo apt-get install ros-$ROS_DISTRO-arbotix ros-$ROS_DISTRO-openni-camera \
ros-$ROS_DISTRO-dynamixel-motor ros-$ROS_DISTRO-rosbridge-suite \
ros-$ROS_DISTRO-mjpeg-server ros-$ROS_DISTRO-rgbd-launch \
ros-$ROS_DISTRO-moveit-full ros-$ROS_DISTRO-moveit-ikfast \
ros-$ROS_DISTRO-turtlebot-* ros-$ROS_DISTRO-kobuki-* ros-$ROS_DISTRO-moveit-python \
python-pygraph python-pygraphviz python-easygui \
mini-httpd ros-$ROS_DISTRO-laser-pipeline ros-$ROS_DISTRO-ar-track-alvar \
ros-$ROS_DISTRO-laser-filters ros-$ROS_DISTRO-hokuyo-node \
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
