# Assignment3

Software architecture for the control of a mobile robot.
================================

This is software architecture for the control of the robot in the environment developed by [Student Robotics](https://studentrobotics.org) in ROS.
The software will rely on the move_base and gmapping packages for localizing the robot and plan the motion.

The architecture is able to get the user request, and let the robot execute one of the following behaviors (depending on the user’s input):
1. autonomously reach a (x,y) coordinate inserted by the user
2. let the user drive the robot with the keyboard
3. let the user drive the robot assisting them to avoid collisions

Installing and running
----------------------

In a terminale type the following commands:
```bashscript
$ sudo apt-get install ros-<your_ros_distro>-navigation
$ mkdir -p ROS_ws/src
$ cd ROS_ws/src
$ git clone https://github.com/CarmineD8/slam_gmapping.git
$ git clone https://github.com/AliceRivi14/Assignment3.git
$ cd ..
$ catkin_make
```
Add the line `‘source [ws_path]/devel/setup.bash’` in your `.bashrc` file.

To run the simulation, open two different terminal and type:
```bashscript
$ roslaunch final_assignment simulation_gmapping
```
```bashscript
$ roslaunch final_assignment move_base
```

To run the node for this project, open another terminal and type:
```bashscript
$ roslaunch final_assignment assignment
```

Now you can see the robot in the environment of Rviz and Gazebo.


Lauch file
-----------

### Simulation_gmapping ###

The simulation_gmapping.launch file allows:
* Add the description of the robot to the ROS parameter server
* Launch the simulation in Gazebo
* Launch the Rviz node, along with some additional nodes
* Generate the robot in the simulation

### Move_base ###

The move_base.launch file allows:
* Launch the move_base node
* Set the rosparam described in the .yaml file

### Assignment ###

The assignment.launch file allows:
* Launch the ?? node


Nodes
-----------

### Rviz node ###

When launching Rviz, three nodes are actually executed:

* `joint_state_publisher`: the package reads the robot_description parameter from the parameter server, finds all of the non-fixed joints and publishes a JointState message with all those joints defined. If GUI is present, the package displays the joint positions in a window as sliders.

* `robot_state_publisher`: the package uses the URDF specified by the parameter robot_description and the joint positions from the topic joint_states to calculate the forward kinematics of the robot and publish the results via `tf`.

* `rviz`

### Slam_gmapping node ###

This ROS node is contained in the `gmapping` package and provides laser-based SLAM (Simultaneous Localization and Mapping). Using `slam_gmapping`, you can create a 2-D occupancy grid map from laser and pose data collected by a mobile robot.
To use `slam_gmapping`, you need a mobile robot that provides odometry data and is equipped with a horizontally-mounted, fixed, laser range-finder. The `slam_gmapping` node will attempt to transform each incoming scan into the `odom (odometry) tf` frame.

The `slam_gmapping` node takes in `sensor_msgs/LaserScan` messages and builds a map (`nav_msgs/OccupancyGrid`). The map can be retrieved via a ROS topic or service. 

Subscriber:
* `tf (tf/tfMessage)`: transforms necessary to relate frames for laser, base, and odometry.
* `scan (sensor_msgs/LaserScan)`: laser scans to create the map.

Publisher:
* `map_metadata (nav_msgs/MapMetaData)`: get the map data from this topic, which is latched, and updated periodically. 
* `map (nav_msgs/OccupancyGrid)`: get the map data from this topic, which is latched, and updated periodically.
* `~entropy (std_msgs/Float64)`: estimate of the entropy of the distribution over the robot's pose (a higher value indicates greater uncertainty). New in 1.1.0.

Server:
* `dynamic_map (nav_msgs/GetMap)`: call this service to get the map data.

Parameters:

The values of these parameters are set according to this project.
(To verify the default parameters, please refer to the website http://wiki.ros.org/gmapping)

* `~base_frame (string, value: "link_chassis")`: the frame attached to the mobile base.
* `~map_update_interval (float, value: 5.0)`: how long (in seconds) between updates to the map. Lowering this number updates the occupancy grid more often, at the expense of greater computational load. 
* `~maxUrange (float, value: 16.0)`: the maximum usable range of the laser. A beam is cropped to this value. 
* `~sigma (float, default: 0.05)`: the sigma used by the greedy endpoint matching.
* `~kernelSize (int, value: 1)`: the kernel in which to look for a correspondence.
* `~lstep (float, value: 0.05)`: the optimization step in translation.
* `~astep (float, value: 0.03)`: the optimization step in rotation.
* `~iterations (int, default: 3)`: the number of iterations of the scanmatcher.
* `~lsigma (float, value: 0.075)`: the sigma of a beam used for likelihood computation.
* `~ogain (float, value: 3.0)`: gain to be used while evaluating the likelihood, for smoothing the resampling effects.
* `~lskip (int, value: 0)`: number of beams to skip in each scan. Take only every (n+1)th laser ray for computing a match (0 = take all rays).
* `~srr (float, value: 0.1)`: odometry error in translation as a function of translation (rho/rho).
* `~srt (float, value: 0.1)`: odometry error in translation as a function of rotation (rho/theta).
* `~str (float, value: 0.1)`: odometry error in rotation as a function of translation (theta/rho).
* `~stt (float, value: 0.1)`: odometry error in rotation as a function of rotation (theta/theta).
* `~linearUpdate (float, value: 1.0)`: process a scan each time the robot translates this far.
* `~angularUpdate (float, value: 0.2)`: process a scan each time the robot rotates this far.
* `~temporalUpdate (float, value: 3.0)`: process a scan if the last scan processed is older than the update time in seconds. A value less than zero will turn time based updates off. 
* `~resampleThreshold (float, value: 0.5)`: the Neff based resampling threshold.
* `~particles (int, value: 20)`: number of particles in the filter.
* `~xmin (float, value: -50.0)`: initial map size (in metres).
* `~ymin (float, value: -50.0)`: initial map size (in metres).
* `~xmax (float, value: 50.0)`: initial map size (in metres).
* `~ymax (float, value: 50.0)`: initial map size (in metres).
* `~delta (float, value: 0.05)`: resolution of the map (in metres per occupancy grid block).
* `~llsamplerange (float, value: 0.01)`: translational sampling range for the likelihood.
* `~llsamplestep (float, value: 0.01)`: translational sampling step for the likelihood.
* `~lasamplerange (float, value: 0.005)`: angular sampling range for the likelihood.
* `~lasamplestep (float, value: 0.005)`: angular sampling step for the likelihood.
* `~transform_publish_period (float, value: 0.0005)`:how long (in seconds) between transform publications. To disable broadcasting transforms, set to 0. 


### Move_base node ###

The `move_base` package provides an implementation of an action that, given a goal in the world, will attempt to reach it with a mobile base. Actions are services which are not executed atomically, and thus may also offer some additional tools, such as a continuous feedback and the possibility of cancel the request.
For the purpose of this project the publish/subscribe architecture will be used for using actions, by sending a message on the topic `move_base/goal`.

Subscriber:
* `move_base/goal (move_base_msgs/MoveBaseActionGoal)`: a goal for move_base to pursue in the world.

Publisher:
* `cmd_vel (geometry_msgs/Twist)`: a stream of velocity commands meant for execution by a mobile base.


### MODE1 node ###

This node represents the first mode that the user can choose. Through this code the user is asked what is the (x,y) position he wants the robot to reach. Once the coordinates have been selected, the robot is guided to the desired position, avoiding obstacles.

If the position is not reached within a certain time, the assignment is cancelled.

Subscriber:
* `scan (sensor_msgs/LaserScan)`: laser scans to create the map.
* `odom (nav_msgs/Odometry)`: odometry data from the position model.

Publisher:
* `cmd_vel (geometry_msgs/Twist)`: a stream of velocity commands meant for execution by a mobile base.
* `move_base/goal (move_base_msgs/MoveBaseActionGoal)`: a goal for move_base to pursue in the world.

This node has 10 functions:

* `bool SwitchModeCallback(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res)`:

    activates the node when the `std_srvs/SetBool` service is called.
    
* `void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)`:

    is called when a message is posted on the `/odom` topic.
    
    The pose in this message corresponds to the estimated position of the robot in the odometric frame along with an optional covariance for the certainty of that pose estimate.
    
    The `tf` software library is responsible for managing the relationships between coordinate frames relevant to the robot in a transform tree.
    
* `void ChangeState(int state)`:

    changes the machine state. There are three states representing the behaviour or the situation in which the robot finds itself
   
* `float NormalizeAngle(float angle)`:

    normalises the angle and allows calculation of the directional error with respect to the desired position
    
* `void FixYaw(float des_pos_x, float des_pos_y)`:

    represents state 0, when the robot has to correct its direction angle to the desired position. If the directional error is less than +/- 2 degrees, the robot changes its state.
    
* `void GoForward(float des_pos_x, float des_pos_y)`:

    represents state 1 of the robot, when the robot is facing the right direction, but away from the desired position. If the position error is less than 0.3, or the direction error is greater than +/- 20 degrees, the robot changes its state.
    
* `void Position()`:

    represents state 2, when the robot reaches the desired position and stops.
    
* `float RobotDistance(int min, int max, float dist_obs[])` in which:

     `min`: minimum index of the subsection of the array that we want to analyze.

    `max`: maximum index of the subsection of the array that we want to analyze.

    `dist_obs[]`: array wich contains 720 elements wich are the distance of the obstacles from the robot.

    `dist_value`: minimun distance from an obstacle in the subsection of the array.

    It is possible to use the ranges vector to see robot distance from the wall.

* `void ControlAlgorithm()`:

    determines the evolution of the robot based on the distance to obstacles and changes its status according to the position of the robot relative to the desired position.
    
    With this function the robot's speed and position are published on the `/cmd_vel` and `/move_base/goal` topics respectively.
    
* `void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)`:

    is called when a message is posted on the `/scan` topic. 

    The robot can see with a field of 180° in front of him and this field (in radiants) is divided in 720 section.
    
    
### MODE2 node ###

This node represents the second mode that the user can choose. Through this code the user can guide the robot within the environment using the keyboard.

In order to be able to move the robot via the keyboard, it may be useful to exploit the keyboard teleop program. The code of the `teleop_twist_keyboard.cpp` node has been copied into the MODE2 node and adapted to the needs of the project.

Publisher:
* `cmd_vel (geometry_msgs/Twist)`: a stream of velocity commands meant for execution by a mobile base.

This node has 2 functions:
* `bool SwitchModeCallback(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res)`:

    activates the node when the `std_srvs/SetBool` service is called.
    
* `int getch(void)`:

    manages user-generated keyboard inputs by preventing them from blocking.
    
When the node is activated, a message is printed on the screen indicating which commands are to be used to move the robot.

`
Reading from the keyboard and Publishing to Twist!
  ---------------------------
  Moving around:
     u    i    o
     j    k    l
     m    ,    .
  For Holonomic mode (strafing), hold down the shift key:
  ---------------------------
     U    I    O
     J    K    L
     M    <    >
  t : up (+z)
  b : down (-z)
  anything else : stop
  q/z : increase/decrease max speeds by 10%
  w/x : increase/decrease only linear speed by 10%
  e/c : increase/decrease only angular speed by 10%
  CTRL-C to quit
`

The commands for moving the robot are as follows:

`u` to turn left

`i` to go straight ahead

`o` to turn right

`j` to turn counterclockwise

`k` to stop

`l` to turn clockwise

`m` to turn left going backwards

`,` to go straight back

`.` to turn right going backwards

The other commands described in the message can be used to change the speed of the robot.


### UI node ###

The UI node represents the project’s user interface. This node constantly waits for an input from the user, who can ask which mode you want to run based on the robot’s behavior.


This node has one function:

* `int MODE()`:

    print a integer request message and return the integer given in input by the user.
    



Pseudocode
------------------------

###  ###

```pseudocode
float RobotDistance(min, max, dist_obs[]){
  calculate the minimum distance from an obstacle in a range of 720 elements
  return the distant value
}  


void LaserCallback(scan){
  calculate the min distance of the robot from the wall in the left, right and front position with the function RobotDistance

  if there is obstacles in the front of the robot
     if robot is closer to the obstacles to the left
        turn the robot on the right
     else if the robot is closer to the obstacles to the right
        turn the robot on the left
  else go the robot forward
}

int main (){
  initializing control_node and the NodeHandle
  definition of service, publisher and subscriber
}
```

### UI_node ###

```pseudocode
char Input(){
  print a character request message and return the character given in input by the user
}  

void ScanCallback(msg){
  send the request to change the velocity and reset the position
}

int main (){
  initializing ui_node and the NodeHandle
  definition of client and subscriber
} 
```
