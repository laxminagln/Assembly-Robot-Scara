## Design and Development of a Robotic Arm for Automated Screw Assembly

This repo outlines the details related to the Assembly Scara Robot project. 

### Fusion360
The subfolder Fusion360 contains the step files of each component designed for the robotic arm using Fusion 360. It also has the full model, with the file name Robotic_Arm.step.

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
 - Conveyor and Sprocket (STL files) - Conveyor v1.stl, sprocket v1.stl

### Image
The subfolder Image contains the images backing up the implementation.

 - The design of the arm, part highlights from different perspectives - Arm_Design.png
 - The circuit diagram including all electronic components - Circuit_Diagram.png
 - The assembled circuit - Circuit_Assembly.jpg
 - Image of Robot Arm after Assembly, taken in Grid - Robot_Arm.jpg
 - Clear image of Robot Arm - Robot_Arm_wo_bg.png
 - Trajectory Planning plots (position vs time, velocity vs time, acceleration vs time) - Position_Time.png, Velocity_Time.png, Acceleration_Time.png

### Video
The subfolder Video contains the videos backing up the implementation.

 - The video of the whole assembly and working of the robot arm - Working_Video.mp4
 - Simulation of Robot Arm highlighting horizontal and vertical movements created in Matlab - Matlab_Simulation.mp4
 - Trajectory Simulation of Robot Arm created using Matlab - Trajectory_Planning_Simulation.mp4

### Code
The subfolder Code contains the code files for our robotic arm implementation and simulations.

 - Matlab code file for simulating trajectory and generating corresponding plots - Trajectory_Planning.m
 - Arduino code for the working of Robot Arm (uses inverse kinematics) - robot_arm.ino
 - Matlab code file for simulating the horizontal and vertical movements - Simulation/simulation.m
