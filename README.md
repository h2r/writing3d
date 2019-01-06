# writing3d
ROS package for the 3D writing project

### Arm motion control and planning
Run the planner
```
./moveit_planner.py right_arm
```
There are several ways to move the arm. First, you can set a pose target for the end effector, with quaternion or not:
```
./moveit_client right_arm -g x y z --ee
./moveit_client right_arm -g x y z qx qy qz qw --ee
```
If the `--ee` is not supplied, the list of coordinates will be interpreted as joint space goal pose.

Second, you can supply a file of YAML format with a list of pose targets (waypoints).
The YAML file can also be just a list of 7 target joint positions (joint space goal).
```
./moveit_client right_arm -f <path_to_file>
```
If you want to move the arm manually (without planning),
```
./movo_pose_pubilsher -i <list of joint indices> -v <list of joint values>
```
If you supply the option `-c`, the indices will be for cartesian coordinates (i.e. x y z). Right now for some reason the
cartesian goal doesn't work after `moveit_client` is used to move the arm.

### Write a character
Right now, just run
```
./writer.py
```
