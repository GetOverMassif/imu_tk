
# build order
```bash
colcon build
source install/setup.bash
```

# 
ros2 run imu_calib imu_data_receiver --ros-args -p limit_time:=80 -p limit_num:=8000 -p target_path:=/home/lj/Documents/imu_tk/data/data5

limit_time limit_num target_path