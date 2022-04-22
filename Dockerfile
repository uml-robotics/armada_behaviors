# Use Melodic version 
FROM ros:melodic-ros-core-bionic

SHELL [ "/bin/bash" , "-c" ]

# Install all basic deps
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    ros-melodic-catkin \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*


# bootstrap rosdep
RUN rosdep init \
&& rosdep update --rosdistro $ROS_DISTRO  \
&& rosdep fix-permissions\
&& rosdep update

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-ros-base=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*


# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-ros-base=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

#Intial Workpsace Setup
RUN source /opt/ros/melodic/setup.bash \
 && mkdir -p /home/catkin_ws/src \
 && cd /home/catkin_ws/src \
 && catkin init 

#Intial Build
RUN source /opt/ros/melodic/setup.bash \
 && cd /home/catkin_ws \
 && catkin build


#### TEST APPLICATION ##########################
# Copy Code into src workspace 
WORKDIR /home/catkin_ws/src
COPY . .

# Rebuild Workspace
RUN source /opt/ros/melodic/setup.bash \
 &&  cd /home/catkin_ws/ \
 && catkin build
