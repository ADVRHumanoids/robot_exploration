# gazebo_odom

Package to provide pelvis pose wrt to world from Gazebo.

Output: TF from world to pelvis

## Launch file
*start_simulation.launch.py* is used to start the simulation with CENTAURO.

Nodes executed:
- sim_odom_connect_: to obtain ground truth odometry from Gazebo.
- Gazebo launch with urdf spawner and ros_gz_bridge
- xbot2