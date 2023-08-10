echo "Building ROS nodes"

cd Examples/ROS/Edge_SLAM
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=RelWithDebInfo
make -j
