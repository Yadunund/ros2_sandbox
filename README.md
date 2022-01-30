# ros2_sandbox
A collection of ROS 2 packages for testing


## test_multithreaded_exeuctor
Clone this package into your workspace 
```
source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select test_multithreaded_executor
```

Terminal 1
```bash
source WORKSPACE/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp #optional
ros2 run demo_nodes_cpp talker
```

Terminal 2
```bash
source WORKSPACE/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp #optional
ros2 run test_multithreaded_executor test_executor 50 # for 50 listeners
```