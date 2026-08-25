My SOlution for problem 2

 ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=depot

 check depth views
  ros2 topic echo /oakd/rgb/preview/camera_info --once
