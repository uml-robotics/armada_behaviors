#From Prebuilt image 
FROM  jmeliuml/ros-ci-base

#### TEST APPLICATION ##########################
# Copy Code into src workspace 
WORKDIR /home/catkin_ws/src
COPY . .

# Rebuild Workspace
RUN source /opt/ros/melodic/setup.bash \
 &&  cd /home/catkin_ws/ \
 && rosdep install --from-paths src --ignore-src -r -y \
 && catkin build
