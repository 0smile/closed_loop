# Closed loop demo for ROS2

It is a ROS2 version of the Four-bar linkage example.

In ROS2, spawning a entity in Gazebo must be done by generating from the topic of "robot_state_publisher" or a SDF file. Luckily the origin method can still be used.



Due to URDF cannot express closed loop structure, I use some tricks to show closed loop structure in rviz.

To do so, you must have a real closed loop structure robot, or you can realize it through classic gazebo. Implementation with ignition gazebo has not been tested.

Firstly you need to break the closed loop chain, to make your URDF a open loop structure. You can easily do it through remove a joint of a closed loop.

Then, Inject this joint with <gazebo> tag in URDF file. As we spawn the model in gazebo through robot_description topic, we will get a closed loop robot in gazebo.

Next we need to use ros2_control, more specifically, gazebo_ros2_control to show the robot in rviz. It does a map from the joint state in gazebo to the ros2 topic.

With above steps, you can visualize your closed loop robot in both gazebo and rviz, and they are synchronized. An issues is when you restart the world, synchronization will lost.

Further, Moveit and Nav may can be applied on closed loop robot. However one thing should be keeped in mind: it still a open loop robot in ros2. 
  
## Konwn issue
1. If include rviz in launch file, the robot model will not shown in rviz. Launching rviz manually after gazebo can avoid this issue, still seeking for better solutions.

2. When restart the gazebo simulation world, synchronization will lost.
