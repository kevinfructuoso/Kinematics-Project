## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace. Look at me, I'm Mr. Meeseeks!
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_img_vids_gifs/misc3.png
[image2]: ./misc_img_vids_gifs/forward_kinematics_reference_frames.png
[image3]: ./misc_img_vids_gifs/pose_middle_left_to_bin.png
[image4]: ./misc_img_vids_gifs/pose_middle_left.png
[image5]: ./misc_img_vids_gifs/pose_error_upper_right_to_bin.png
[image6]: ./misc_img_vids_gifs/pose_error_upper_right.png
[image7]: ./misc_img_vids_gifs/kuka_project.gif

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The first step is to define the reference frames of the kuka arm. For purposes of this analysis, the Denavit-Hartenberg (DH) convention used is defined in John J. Craig's Intro to Robotics 3rd Edition. This conventions uses four parameters to describe position/orientation of neighboring frames.

The below image displays the reference frames chosen for this analysis and code implementation. The last three joints (4, 5, and 6) form a spherical wrist. Frames 4, 5, and 6 will all be centered at the wrist center. This is required to be able to obtain an analytical solution to the Inverse Kinematics equations for the Kuka arm.

![alt text][image2]

Using these reference frames, the non-zero DH parameter table can be derived, as shown below, where G is the gripper frame.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
:-----: |  :-----: | :-----: | :-----: | :-----:
0->1    |    0     |    0    |    d1   | q1
1->2    |  -pi/2   |   a1    |    0    | -pi/2 + q2
2->3    |    0     |   a2    |    0    | q3
3->4    |  -pi/2   |   -a3   |    0    | q4
4->5    |   pi/2   |    0    |    d4   | q5
5->6    |  -pi/2   |    0    |    0    | q6
6->G    |    0     |    0    |    dG   | 0


From the kr210.urdf.xacro file, the following table details the relative coordinates from one joint to the next.

Links   | x       | y       | z 
:-----: | :-----: | :-----: | :-----:
0->1    |    0    |    0    | 0.33 
1->2    |  0.35   |    0    | 0.42
2->3    |    0    |    0    | 1.25
3->4    |  0.96   |    0    | -0.054 
4->5    |  0.54   |    0    | 0 
5->6    |  0.193  |    0    | 0 
6->G    |  0.11   |    0    | 0 


With the values from the above table, most of the DH parameter table can be directly filled in.

Links   |alpha(i-1)|  a(i-1) |  d(i-1) | theta(i)
:-----: |  :-----: | :-----: | :-----: | :-----:
0->1    |    0     |    0    |   0.75  | q1
1->2    |  -pi/2   |   0.35  |    0    | -pi/2 + q2
2->3    |    0     |   1.25  |    0    | q3
3->4    |  -pi/2   |  -0.054 |    0    | q4
4->5    |   pi/2   |    0    |    1.5  | q5
5->6    |  -pi/2   |    0    |    0    | q6
6->G    |    0     |    0    |   0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Using the defined DH parameter table from the above section, each individual transformation matrix can be defined. 
/*7 tables...*/


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The position of the wrist center can be determined given the position of the gripper according to the following equation. The wrist center will be used to determine the first three joint angles 1, 2, and 3. Note that the orientation of the gripper must be corrected with respect to the base link.

*equation here!*

After solving for the wrist center position from the above equation, joint angle 1 can be derived by the equation:

*equation here!*

The image below helps visualize joint angles 2 and 3. Using some fancy geometry skills, the below equations can be determined for joint angles 2 and 3.

![alt text][image1]
*equation here!*

Once joint angles 1-3 are found, they can be used to generate the equations to solve for joint angles 4-6. To isolate equations for joints 4-6, the rotation matrix from link 3 to link 6 must be derived. In order to simplify down to joint angles 4-6, the recently solved angles 1-3 can be substituted in the following equation.

*equation here!* 

The result is a matrix of equations that make up the rotation matrix from link 3 to link 6. That matrix is shown below.

*insert matrix*

By definition, the product of individual rotations between respective links must be equal to the the overall pose rotation of the gripper link. Therefore:

*equation here!*

By substituting the gripper correction matrix and  the solved joint angles 1-3, the result is the rotation matrix from link 3 to link 6 solved as constants. By setting the matrix of equations equal to these constants, the below equations can be derived to solve for joint angles 4-6.

*last equations here!* 

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The IK_server.py implements the above analysis to compute the inverse kinematics of the Kuka arm. The homogeneous transform is used to compute the forward kinematics to determine the position error. All of the heavy matrix computations are done with only symobls before the ROS node begins running. When the time comes, the results simply have the appropriate values substituted in to complete the calculations. This was done to optimize run time during/between the pick and place motions.

With the above analysis and techniques, the Kuka arm was able to successfully complete 10/10 pick and place trials. The tests can be seen in the .gif below.

<p align="center">
  <img width="460" height="300" src="./misc_img_vids_gifs/kuka_project.gif">
</p>

![alt text][image7]

The magnitude of the position error was calculated using forward kinematics from the solved joint angles. The calculations of this project has a consistent position error of approximately 1 mm. The below figure is an example of the plotted errors for indivdual motions. The error calculations are commented out in the final submission in order to optimize the execution time of the code.

![alt text][image3]

One way to improve the project is to optimize the motions between poses. As can be seen in the .gif, there is some wasted motion where the arm will rotate more than necessary to reach a position. Logic could be added to be smarter about the solved angles to complete the poses since there are multiple viable solutions. This would also optimize the execution time of each test run as well.
