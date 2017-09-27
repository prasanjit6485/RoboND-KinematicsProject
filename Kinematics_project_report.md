## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]:./misc_images/misc2.png
[kinematicdiagram]:./misc_images/KinematicDiagram.png
[dhtable]:./misc_images/dhtable.png
[inversekinematics]:./misc_images/inversekinematics.png
[pickandplace]:./misc_images/pickandplace.png

![alt text][image1]

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

I am submitting the writeup as markdown and You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and generate Kuka KR210 schematic diagram.

![alt text][kinematicdiagram]

#### 2. Dereive DH paramter table from the above schematic diagram and generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

![alt text][dhtable]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

![alt text][inversekinematics]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

`IK_server.py` code is well commented and self explanatory. To improve the overall performance for computing inverse kinematics, compute correctional rotation matrix and inverse of rotation matrix from link 0 to link 3 outside the loop. In addition with python sympy module, by computing inverse of rotation matrix first and then substitue values improved overall end-effector offset significantly. Also, I have added a flag ERR_ANALYSIS for computing overall end-effector offset for each pose. Currently it is set to False state. Below is the error and performance analysis for particular plan:

```
[INFO] [1506520559.311973]: Printing 16 end-effector offsets:
Pose 01 - Overall End-Effector offset is: 0.000036684988862 units
Pose 02 - Overall End-Effector offset is: 0.000036684988862 units
Pose 03 - Overall End-Effector offset is: 0.000036684988861 units
Pose 04 - Overall End-Effector offset is: 0.000036684988862 units
Pose 05 - Overall End-Effector offset is: 0.000036684988861 units
Pose 06 - Overall End-Effector offset is: 0.000036684988861 units
Pose 07 - Overall End-Effector offset is: 0.000036684988861 units
Pose 08 - Overall End-Effector offset is: 0.000036684988862 units
Pose 09 - Overall End-Effector offset is: 0.000036684988862 units
Pose 10 - Overall End-Effector offset is: 0.000036684988862 units
Pose 11 - Overall End-Effector offset is: 0.000036684988862 units
Pose 12 - Overall End-Effector offset is: 0.000036684988862 units
Pose 13 - Overall End-Effector offset is: 0.000036684988862 units
Pose 14 - Overall End-Effector offset is: 0.000036684988862 units
Pose 15 - Overall End-Effector offset is: 0.000036684988862 units
Pose 16 - Overall End-Effector offset is: 0.000036684988862 units

[INFO] [1506520559.342267]: Total run time to calculate joint angles from pose is 20.3762 seconds
```

We can further improve the overall end-effector offset by effectively computing theta2 and theta3.

Added image with 10/10 pick and place cycles.
![alt text][pickandplace]

In summary, I was able to successfully complete 8/10 pick and place cycles for the Kinematic's project. Overall it was fun and challenging.

