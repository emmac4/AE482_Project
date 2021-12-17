# AE 482 Project:  Pixel Art Bot
-----------------------------------

## Instructions
------------------

-Make sure to source the ROS workspace before doing any of the following by running the 
following command in the terminal while in your workspace directory: 
'''
{
	 'source devel/setup.bash'
	 }
	 '''

-Launch the Gazebo world by running the following code in terminal: 
	 $ roslaunch project_gazebo ur3_project.launch
  this launches the Gazebo simulator, including the Gazebo world we designed
  with the robot arm, camera, and blocks. This also starts the roscore.

-To run the script that moves the robot and contains the sensor subscribers:
	'rosrun project_gazebo sort_exec.py'
   this script does several things: 
   1. interfaces with *blob_search.py* which uses OpenCV to read image data from camera and locate the blocks of specified colors
   2. Passes block locations given from *blob_search.py* into *sort_func.py* which calculates inverse kinematics then forward kinematics
   3. Gives the UR3 arm move commands based on the forward kinematics calculated
