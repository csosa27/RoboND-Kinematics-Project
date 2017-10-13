## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/forward_kinematics.JPG
[image2]: ./misc_images/kuka_KR210_sketch.jpg
[image3]: ./misc_images/kinematics_analysis_WC.jpg
[image4]: ./misc_images/ik_equations_v1_3.JPG
[image5]: ./misc_images/works.JPG
[image6]: ./misc_images/works3.JPG
[image7]: ./misc_images/error.JPG

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The forward_kinematics.launch file was used to generate the kinematic sketch (**Figure** **2**) with assistance from Lesson 2 and Project Module.

![alt text][image1]

###### **Figure**  **1** : Model represented by the foward_kinematics.launch file

![alt text][image2]
###### **Figure**  **2** : Sketch to display links with offsets, lengths, and joint axes.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Obtained the DH Parameters performing a kinematic sketch (**Figure** **2**) and analysis like the one explained in Lesson 2 and later in the Project Module.

Using the [kr210.urdf.xacro](https://github.com/csosa27/RoboND-Kinematics-Project/tree/master/RoboND-Kinematics-Project\kuka_arm\urdf) file the below DH Parameter table was generated. Values were obtained by looking for the joints section in the xacro file; there using the sketch from **Figure** **2** distances from joint to joint were obtained and used as a(i-1) and d(i) values repective to their axis as provided in the Figure. Some values, like d(EE) might need to obtained by the sum of multiple joint xyz values, in this case, the x value of joint 6 and the x value of the gripper joint. 

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | - 0.054 | 1.5 | q4
4->5 |   pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The Inverse Position Kinematics were addressed in this snippet from the [IK_debug.py]:

		# Compensate for rotation discrepancy between DH parameters and Gazebo
		R_corr = R_z(pi) * R_y(-pi/2)
		R_rpy = R_z(y) * R_y(p) * R_x(r)
		R_rpy = R_rpy.evalf(subs={r: roll, p: pitch, y: yaw})
            
		R0_6 = R_rpy * R_corr

		# Calculate joint angles using Geometric IK method
		d_7 = dh[d7] # d7 from DH table
		wx = px - (d_7 * R0_6[0,2])
		wy = py - (d_7 * R0_6[1,2])
		wz = pz - (d_7 * R0_6[2,2])


The Inverse Orientation Kinematics were addressed by the analysis shown in the following figures:

![alt text][image3]

###### **Figure**  **3**: Representation of the triangle made up by joints 2, 3 and wrist center.

![alt text][image4]

###### **Figure**  **4**: Triangle 2-3-WC with equations used

The following code snippet shows the way the above equations were addressed to obtain theta 1-6.

		# Leveraging link distances and offsets from dh table
		a_3 = dh[a3]
		d_4 = dh[d4]
		d_1 = dh[d1]
		a_1 = dh[a1]
		a_2 = dh[a2]

		#### Finding theta 1-3
		theta1 = atan2(wy, wx)

		# equations from the inverse kinematics example (Kinematics: lesson 2 - Section 19)
		r = sqrt(wx**2 + wy**2) - a_1 #wx -> xc & wy -> yc (interpreted from top view)
		S = wz - d_1 # wz -> zc

		s_a = sqrt(a_3**2 + d_4**2)
		s_b = sqrt(r**2 + S**2)
		s_c = a_2

		# Law of Cosines to obtain angles a and b (alpha and beta, respectively)
		alpha = acos((s_c**2 + s_b**2 - s_a**2) / (2*s_c*s_b))
		beta = acos((s_c**2 + s_a**2 - s_b**2) / (2*s_c*s_a))

		theta2 = (pi/2) - alpha - atan2(S,r)
		theta3 = (pi/2) - beta - atan2(-a_3,d_4)

		#### Finding theta 4-6

		# Evaluating rotational matrix extracted from original transformation matrices using obtained theta_i
		R0_3 = T0_1 * T1_2 * T2_3
		R0_3 = R0_3[0:3, 0:3]
		R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

		R3_6 = R0_3.inv("ADJ") * R0_6 # noticed ADJ inverse method reduced EE position error to 0 on IK_debug.py
		
		# R3_6 Matrix Values
		r13 = R3_6[0,2]
		r33 = R3_6[2,2]
		r23 = R3_6[1,2]
		r21 = R3_6[1,0]
		r22 = R3_6[1,1]
		r12 = R3_6[0,1]
		r32 = R3_6[2,1]

		theta4 = atan2(r33, -r13)
		theta5 = atan2(sqrt(r13**2 + r33**2), r23)
		theta6 = atan2(-r22, r21)

The matrix displayed in **Figure** **4** further demonstrate the way rotational matrix values were extracted.
		
### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

In order to obtain the transformation and rotation matrices, I decided to utilize functions to generate all of the different matrices. This is shown in the [IK_server.py] snippet below:

	def TF_Matrix(q, alpha, a, d):
		T = Matrix([[           cos(q),           -sin(q),           0,             a],
					[sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
					[sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
					[                0,                 0,           0,             1]])
		return(T)

	def R_z(y):
		Rot_z = Matrix([[   cos(y),   -sin(y),         0],
						[   sin(y),    cos(y),         0],
						[        0,         0,         1]])
		return(Rot_z)

	def R_y(p):
		Rot_y = Matrix([[   cos(p),         0,    sin(p)],
						[        0,         1,         0],
						[  -sin(p),         0,    cos(p)]])
	
		return(Rot_y)

	def R_x(r):
		Rot_x = Matrix([[        1,         0,         0],
						[        0,    cos(r),   -sin(r)],
						[        0,    sin(r),    cos(r)]])
		return(Rot_x)

		
These allowed the code to easily create the many transformation and rotation matrices by calling the functions, while still being outside of the handle_calculate_IK function. Another advantage was to generate all the transformation and rotation matrices outside the forloop to prevent them being generated constantly which would decrease performance and effectiveness. Further, I tried to leverage the DH parameters as much as possible given that they were already created and stored. 

Possibly, due to computer performance, it was rather slow still and while I tried implementing a class structure to the code, I couldn't manage to get it to work. 

![alt text][image5]

###### **Figure**  **5** : Here it shows the gripper grabbing the can on the shelf

![alt text][image6]

###### **Figure**  **6** : Here it shows the kuka arm releasing the can on the bucket.

#### 2. Errors and considerations for future improvements.
While debugging the code many times (long times due to slow performance), I noticed that the code does not respond well when the planned path is relatively abnormal and navitates far away to grab a can or to move towards the bucket. Not sure why this happens but when normal trajectories are given the code performs well. Not sure if it'll require calibration or more statements to make it smarter and discern the correct path to take on that kind of situation. This can be seen in the following Figure:

![alt text][image7]

###### **Figure**  **7** : Here it shows the error given when abnormal planned path is provided and it's unable to solve the inverse kinematics.





