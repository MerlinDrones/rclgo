# To Use

This example must be placed in a ROS2 workspace to work via ros2 run.

```bash
colcon build --packages-select rclgo_param_demo_pkg
source install/setup.bash

#Run the ros node without a yaml file
ros2 run rclgo_param_demo_pkg param_demo
# or with a yaml file
ros2 run rclgo_param_demo_pkg param_demo --params-file /absolute/path/params.yaml

ros2 param list
ros2 param get /param_demo camera.fps
ros2 param set /param_demo camera.fps 30
ros2 param describe /param_demo camera.exposure
```

Alternatively, you can build and run the binary directly.

```bash
go build ./cmd/param_demo
./param_demo &
ros2 param list
ros2 param get /param_demo camera.fps
ros2 param set /param_demo camera.fps 30
ros2 param describe /param_demo camera.exposure
```
