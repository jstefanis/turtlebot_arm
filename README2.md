# PhantomX Pincher arm under ROS indigo 
<p align="center">
<img src="https://github.com/NathanCrombez/PhantomXPincherArmROS/blob/master/images/pincher.jpg" align ="middle" width="60%" height="60%" title="header">
</p>

## Packages installation 

__Arbotix__:
ArbotiX ROS drivers to interface with the ArbotiX board.
Installation from source:

	cd ~/ros/indigo/catkin_ws/src
	git clone https://github.com/NathanCrombez/arbotix_ros
	cd .. && catkin_make


__Turtlebot Arm__:
The turtlebot_arm package provides bringup, description, and utilities for using the turtlebot arm. 
The Kinetic version provides code for the PhantomX Pincher. The Kinectic version works with ROS Indigo except "turtlebot_arm_object_manipulation" and "turtlebot_arm_block_manipulation".
Installation from source:

	cd ~/ros/indigo/catkin_ws/src
	git clone  https://github.com/NathanCrombez/turtlebot_arm
	cd .. && catkin_make

	
	
	

## arbotix

The arm architecture is described in a yaml file. USB port, number of joints, limits...

	port: /dev/ttyUSB0
	read_rate: 15
	write_rate: 25
	joints: {
  		arm_shoulder_pan_joint: {id: 1, neutral: 205, max_angle: 180, min_angle: -60, max_speed: 90},
		arm_shoulder_lift_joint: {id: 2, max_angle: 150, min_angle: -150, max_speed: 90},
		arm_elbow_flex_joint: {id: 3, max_angle: 150, min_angle: -150, max_speed: 90},
 		arm_wrist_flex_joint: {id: 4, max_angle: 100, min_angle: -100, max_speed: 90},
		gripper_joint: {id: 5, max_angle: 0, min_angle: -180, max_speed: 90}
	}
	controllers: {
 		arm_controller: {type: follow_controller, joints: [arm_shoulder_pan_joint, arm_shoulder_lift_joint,  arm_elbow_flex_joint, arm_wrist_flex_joint], action_name: arm_controller/follow_joint_trajectory,  onboard: False }
	}
	
see ROS by examples vol.2 for further information.

## turtlebot_arm ROS package
__Content:__

- turtlebot_arm _bringup: TODO
- turtlebot_arm_description: TODO
- turtlebot_arm_kinect_calibration: TODO
- turtlebot_arm_moveit_config: TODO
- turtlebot_arm_block_manipulation: TODO

__1. Getting started:__

1.1 Command the arm from arbotix gui :

	#Edit turtlebot_arm_bringup launch file and set the correct USB port
	roslaunch turtlebot_arm_bringup arm.launch
	rosrun arbotix_python arbotix_gui

<p align="center">
<img src="https://github.com/NathanCrombez/PhantomXPincherArmROS/blob/master/images/arbotix_gui.png" width="45%" height="45%" title="arbotix gui">
</p>


__2. Path planning:__

2.1 - Path planning simulation :

	roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch 

<p align="center">	
<img src="https://github.com/NathanCrombez/PhantomXPincherArmROS/blob/master/images/pathplanning.png " width="50%" height="50%" title="rviz path planning">
</p>


2.2 - Path planning with a real arm :

	#Edit turtlebot_arm_moveit launch file and pass the argument "sim" to "false".
	roslaunch turtlebot_arm_bringup arm.launch
	roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch 
	
Video:

[![YOUTUBE VIDEO](https://img.youtube.com/vi/srDlH51DyNc/0.jpg)](https://youtu.be/srDlH51DyNc)




__3. Kinect/Arm calibration:__

3.1 Requirements:
- Calibration checkerboard (7x6 27mm) : [here](http://wiki.ros.org/turtlebot_kinect_arm_calibration/Tutorials/CalibratingKinectToTurtleBotArm?action=AttachFile&do=get&target=check_7x6_27mm.pdf) 
- The complete checkerboard have to be in the kinect field of view
- Be sure that the arm can reach every point on the checkerboard

<p align="center">
<img src="https://github.com/NathanCrombez/PhantomXPincherArmROS/blob/master/images/setup.jpg" width="50%" height="50%" title="setup">
</p>

3.2 Calibration:

Start up the kinect and the arm

	roslaunch freenect_launch freenect.launch #Use factory-calibration registration
	roslaunch turtlebot_arm_bringup myarm.launch #launch perso TODO: why do we need a fake_joint_pubisher ??
	
Launch the calibration program

	roslaunch turtlebot_arm_kinect_calibration calibrate.launch
	
First, this should detect the checkerboard and pop up the image shown below, with the calibration pattern edges overlaid and four points marked on the image. 

<p align="center">
<img src="https://github.com/NathanCrombez/PhantomXPincherArmROS/blob/master/images/cornersdetection.png" width="50%" height="50%" title="cornersdetection">
</p>

As the terminal instructions say,  move the right part of the open gripper to the four specified points like on the following photos. 


	[ INFO] [1507636573.094797721]: [calibrate] Initialized.
	[ INFO] [1507636573.464054472]: [calibrate] Got image info!
	Got an image callback!
	Is the checkerboard correct? 
	Move edge of gripper to point 1 in image and press Enter
	[....]
	
<p align="center">	
<img src="https://github.com/NathanCrombez/PhantomXPincherArmROS/blob/master/images/corner1.jpg" width="50%" height="50%" title="corner 1">
<img src="https://github.com/NathanCrombez/PhantomXPincherArmROS/blob/master/images/corner2.jpg" width="50%" height="50%" title="corner 2">
<img src="https://github.com/NathanCrombez/PhantomXPincherArmROS/blob/master/images/corner3.jpg" width="50%" height="50%" title="corner 3">
</p>

After moving to the fourth point, the calibration script will output something like the following: 


	Resulting transform (camera frame -> fixed frame): 
	  0.103775   0.992602  0.0630181  -0.202959
	  0.841407 -0.0538294  -0.537714   0.153275
	 -0.530344   0.108825  -0.840769    1.01224
		 0          0          0          1

	Resulting transform (fixed frame -> camera frame): 
	  -0.530344    0.108825   -0.840769     1.01224
	  -0.103775   -0.992602  -0.0630182    0.157959
	  -0.841407   0.0538295    0.537714   -0.153275
	4.59135e-41 4.58869e-41 4.58869e-41           1

	Static transform publisher (use for external kinect): 
	rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms
	rosrun tf static_transform_publisher 0.424262 0.0548834 0.943436 -0.480766 -0.00262613 0.874737 0.0607612 /base_link /camera_link 100

	URDF output (use for kinect on robot): 
	<?xml version="1.0"?>
	<robot>
		<property name="turtlebot_calib_cam_x" value="0.424262" />
		<property name="turtlebot_calib_cam_y" value="0.0548834" />
		<property name="turtlebot_calib_cam_z" value="0.943436" />
		<property name="turtlebot_calib_cam_rr" value="-0.116664" />
		<property name="turtlebot_calib_cam_rp" value="0.998702" />
		<property name="turtlebot_calib_cam_ry" value="2.9392" />
		<property name="turtlebot_kinect_frame_name" value="base_link" />
	</robot>

Run the static transform output with the script:

	rosrun tf static_transform_publisher 0.424262 0.0548834 0.943436 -0.480766 -0.00262613 0.874737 0.0607612 /base_link /camera_link 100

As long as this command runs, the static_transform_publisher will publish the transform between the arm frame and the kinect frame. If you move the physical camera, you will need to recalibrate again. 


Whichever visualization option you chose, the kinect pointcloud  should line up with the actual position of the arm in rviz, as shown in the image below. 

<p align="center">
<img src="https://github.com/NathanCrombez/PhantomXPincherArmROS/blob/master/images/aftercalibframes.png " width="50%" height="50%" title="after calibration frames">
<img src="https://github.com/NathanCrombez/PhantomXPincherArmROS/blob/master/images/aftercalib.png " width="50%" height="50%" title="after calibration">
<img src="https://github.com/NathanCrombez/PhantomXPincherArmROS/blob/master/images/aftercalibrealtruth.jpg" width="50%" height="50%" title="after calib real truth">
</p>



__4. Block manipulation:__

Explanations: TODO

	rosrun tf static_transform_publisher 0.424262 0.0548834 0.943436 -0.480766 -0.00262613 0.874737 0.0607612 /base_link /camera_link 100
	roslaunch block_manip_complete.launch
	
	
Video :

 [![YOUTUBE VIDEO](https://img.youtube.com/vi/zvuJRbIJwj8/0.jpg)](https://youtu.be/zvuJRbIJwj8)




































	