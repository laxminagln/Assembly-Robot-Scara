## Design and Development of a Robotic Arm for Automated Screw Assembly

This readme file outlines the details related to the files present in "Team5_Supporting" folder. The requirement is to submit all the supporting files, such as readme, code, CAD, simulation files, photos, and videos associate with the case study of our team.

### Fusion 360 Files
The subfolder Fusion 360 Files contain the step files of each component we designed for our robotic arm using Fusion 360. It has the full model as well with the file name as - Robotic_Arm.step.

 - Base Structure - base_structure.step
 - Link connecting Base Structure and Base Plate - base_link.step
 - Base Plate - base_plate.step
 - Rods fixed to Base Plate - rod_1.step, rod_2.step, rod_3.step, rod_4.step
 - Middle Plate which moves up and down - middle_plate.step
 - Top Plate - top_plate.step
 - Link connecting Middle Plate and Arm 1 - gripper_link_1.step
 - Link connecting Arm 1 and Arm 2 - gripper_link_2.step
 - End Effector motor housing and screw driver shaft extension - gripper.step
 - Robot Arm full model (no motors/ couplers/ ball bearings) - Robotic_Arm.step
 - Conveyor and Sprocket (stl files) - Conveyor v1.stl, sprocket v1.stl

### Photos
The subfolder Photos contain the images backing up our implementation.

 - The design of arm, part highlights from different perspectives - Arm_Design.png
 - The circuit diagram including all electronic components - Circuit_Diagram.png
 - The assembled circuit - Circuit_Assembly.jpg
 - Image of Robot Arm after Assembly, taken in Grid - Robot_Arm.jpg
 - Clear image of Robot Arm - Robot_Arm_wo_bg.png
 - Trajectory Planning plots (position vs time, velocity vs time, acceleration vs time) - Position_Time.png, Velocity_Time.png, Acceleration_Time.png

### Videos
The subfolder Videos contain the videos backing up our implementation.

 - The video of whole circuit assembly and circuit testing of robot arm - Circuit_Assembly.mp4
 - Simulation of Robot Arm highlighting horizontal and vertical movements created in Matlab - Matlab_Simulation.mp4
 - Trajectory Simulation of Robot Arm created using Matlab - Trajectory_Planning_Simulation.mp4

### Codes
The subfolder Codes contain the code files for our robotic arm implementation and simulations.

 - Matlab code file for simulating trajectory and generating corresponding plots - Trajectory_Planning.m
 - Arduino code for the working of Robot Arm (uses inverse kinematics) - robot_arm/robot_arm.ino
 - Matlab code file for simulating the horizontal and vertical movements - Simulation Code/simulation.m