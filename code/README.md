# code

launch.py is the launch file. It will launch the control code, OpenCV code, world and robot. And also bridge the ros2 data (joint state, lidar, imu, camera)

The joint_controller.py is the control. Inter "s" to start movement and "q" to stop movement. It alse subscribe lidar data to implement automatic obstacle avoidance function.

The image_listener.py is the OpneCV code. It will subscribe the camera data and apply OpenCV on it.

The slam_launch.py is using for launch SLAM toolbox with param. It dosen't work due to SLAM does not support ros2 jazzy version yet.
