printmsg "Installing GPD as a library"
cd ~/$WORKSPACE/src
git clone https://github.com/atenpas/gpd
sed -i -e 's/PCL 1.9 REQUIRED/PCL REQUIRED/g' ~/$WORKSPACE/src/gpd/CMakeLists.txt
cd gpd
mkdir build && cd build
cmake ..
make -j
sudo make install

# going to need to fork this repo -----------------------------------------
printmsg "Cloning and installing the gpd_ros package"
cd ~/$WORKSPACE/src
git clone -b master https://github.com/atenpas/gpd_ros
sed -i -e 's/PCL 1.9 REQUIRED/PCL REQUIRED/g' ~/$WORKSPACE/src/gpd_ros/CMakeLists.txt
cd ~/$WORKSPACE
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
catkin build
source devel/setup.bash