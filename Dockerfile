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
    git \
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

# Install GPD as library 
WORKDIR /home/catkin_ws/src
RUN git clone https://github.com/atenpas/gpd \
&& sed -i -e 's/PCL 1.9 REQUIRED/PCL REQUIRED/g' /home/catkin_ws/src/gpd/CMakeLists.txt
WORKDIR /home/catkin_ws/src/gpd
RUN mkdir build \
&& cd build \
&& cmake ..  \
&& make -j  \ 
sudo make install 

WORKDIR /home/catkin_ws/src
RUN git clone -b master https://github.com/atenpas/gpd_ros \
&& sed -i -e 's/PCL 1.9 REQUIRED/PCL REQUIRED/g' /home/catkin_ws/src/gpd_ros/CMakeLists.txt 

WORKDIR /home/catkin_ws/
RUN  source /opt/ros/melodic/setup.bash \
&& rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
&& catkin build \
&& source devel/setup.bash 


#### TEST APPLICATION ##########################
# Copy Code into src workspace 
WORKDIR /home/catkin_ws/src
COPY . .

# Rebuild Workspace
RUN source /opt/ros/melodic/setup.bash \
 &&  cd /home/catkin_ws/ \
 && rosdep install --from-paths src --ignore-src -r -y \
 && catkin build
