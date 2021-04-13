# 3.Robotic-Arm-Manipulation-in-ROS-using-MoveIt-motion-planning-framework.
In this project we shall be exploring Robotic Arm Manipulation in ROS using MoveIt motion planning framework.
 
# Robotic Manipulators / Robotic Arms
These are robots designed to manipulate environment around them.
These are commonly referred to as Robotic Arms.
![image](https://user-images.githubusercontent.com/75885970/114540628-64212d00-9c73-11eb-88fb-a477eba77974.png)

This is a typical structure of a manipulator. A manipulator consists of Joints, Links, End effector and a base.

-Joints: These are the movable components of a robotic arm. Joints allow movement of links. Actuators like motors allow movement of joints.

-Links: These are rigid structure use to connect joints. Movement of links are possible with joints only.

-End-Effector: In serial robotic manipulators, the end effector means the last link of the robot. An end effector is the device at the end of a robotic arm, designed to interact with the environment. For example, the mechanism which gives gripping capability to a robotic arm.

-Base: It is a plane to which a robotic arm is connected.

 #A robotic arm is very similar to a human arm.

-our elbow and wrist are similar to joints.
-our forearm is like a link which connects two joints i.e your elbow and wrist.
-our palm and the fingers are the end-effector.
-our torso is the base for your arm.
![image](https://user-images.githubusercontent.com/75885970/114540806-9c287000-9c73-11eb-9085-1f62dc456135.png)

# About the kinematics
It is a branch of mathematics that deals with motion of a body or system of bodies.
It describes the motion of points, bodies or system of bodies without looking into forces that causes them to move.
Before diving deep let's look at why we need kinematics in the first place?

# Why Kinematics?
![image](https://user-images.githubusercontent.com/75885970/114540963-c2e6a680-9c73-11eb-949e-693cc4475415.png)
-Suppose you have a robotic arm and you want to pick a box using this arm.
-You know the position of the box in Cartesian space i.e you know the x,y,z of the box in space.
-Now in order to pick this box you need the end-effector of your robotic arm to go near the box and then pick it.
-To make this possible you need to actuate the joints of your robotic arm in such a way that the end-effector is able to reach the box.
-Now, how much you need to actuate each joints for this? This is an Inverse Kinematics problem.
-Given the position of the end-effector in Cartesian space you can calculate angle of rotation for each joints using Inverse Kinematics.
-If you have joint-angles and you need to calculate the position of your end-effector in Cartesian space you can calculate that using Forward Kinematics.

![image](https://user-images.githubusercontent.com/75885970/114541043-d98cfd80-9c73-11eb-8177-408d318f3f40.png)

# what is the Moveit?
MoveIt! is a very popular Motion Planning Framework.

It is a set of packages and tools used for Robotic Manipulation in ROS.

MoveIt! contains software for manipulation, motion planning, kinematics, collision checking etc.

It also has a RViz plugin which can be used to perform Motion Planning from RViz itself.

There are also plenty of Python APIs for Moveit! and ROS which can be used to write ROS Nodes that control Robotic Manipulators.

In this section we are going to learn how to use MoveIt! with ROS and Gazebo.

Before moving forward make sure to install MoveIt! on your system

# MoveIt! Architecture
![image](https://user-images.githubusercontent.com/75885970/114541212-0a6d3280-9c74-11eb-89ef-4b8757d62ed2.png)

# move_grou_node ?
This node is the most essential part of MoveIt! which connects all the part of the MoveIt! system together.

This node collects information from the Robot and passes it to various open source plugins available in MoveIt! which are responsible for Motion Planning, IK etc.

This node then passes the instructions from these plugins to the Robot controller which makes the robot move.

Python MoveIt! APIs to command the move_group node to perform actions such as pick/place, IK, FK, among others.

move_group uses the Planning Scene Monitor to maintain a planning scene, which is a representation of the world and the current state of the robot.

# Moveit setup Assitant 
MoveIt! Setup Assistant is a pretty handy tool which provides a GUI interface to generate configuration MoveIt! ROS package.

The MoveIt Setup Assistant is a graphical user interface for configuring any robot for use with MoveIt. Its primary function is generating a Semantic Robot Description Format (SRDF) file for your robot. Additionally, it generates other necessary configuration files for use with the MoveIt pipeline. To learn more about the SRDF, you can go through the URDF/SRDF Overview page.
![image](https://user-images.githubusercontent.com/75885970/114541476-6768e880-9c74-11eb-8d6d-35b13083c4ba.png)

-By using this moveit setup assistant , i have generated the moveit configuration files.

# RViz Interface
MoveIt! comes with a plugin for the ROS Visualizer (RViz). The plugin allows you to setup scenes in which the robot will work, generate plans, visualize the output and interact directly with a visualized robot. We will explore the plugin in this tutorial.

Lets start by visualising the planning of a robotic arm in Rviz.
![image](https://user-images.githubusercontent.com/75885970/114541681-aa2ac080-9c74-11eb-8a0a-94a74a9fc651.png)

You will see a robotic arm and a MotionPlanning display type in Rviz.

In the MotionPlanning menu go to the Planning option and select Start State and Goal State and click on plan button you will observe that the robot planning a path to a Goal State.
![image](https://user-images.githubusercontent.com/75885970/114541717-b3b42880-9c74-11eb-8941-584430a20245.png)

# Gazebo Interface
Now that I have seen how to use Setup assistant and visualize the movement of the arm in Rviz, let's find out how to make it simulate on Gazebo.

To make the arm move on Gazebo I need an interface which will take the commands from move_it and convey it to the arm in simulation.

This interface in this scenario is a controller. Controller is basically a generic control loop feedback mechanism, typically a PID controller, to control the output sent to your actuators.

I need to define controllers for the arm. Let's start:

# ros_controller configuration file for Moveit 
-In this mainly i have to setup the files for controllers, and configure all files related to controllers
A configuration file called ros_controllers.yaml has to been created inside the config folder of the pkg_moveit_ur5_1 package.

This is for MoveIt to know that there is a controller called arm_controller which takes care of the movement of the arm in Gazebo.

# joint_state_controller config file
- Creating the this file for the  providing the joint state publisher to the desired subscriber;

# Writing a controller configuration file for Gazebo
A configuration file called trajectory_control.yaml has to be created inside the config folder of the pkg_moveit_ur5_1 package.

This is for gazebo interface. Basically, i am defining the arm_controller that i called in the config file for MoveIt in step 1.

This controller when spawned/loaded in Gazebo will basically get the input from MoveIt and will move the arm accordingly.

It is equivalent to the remote control in a remote controlled car. Our hand is MoveIt giving input, the controller is the arm controller which i will be defining here and the car is the arm spawned in Gazebo.

-We need to define various parameters such as:
1)Type of the controller: If it is position based controller or velocity based controller. Position based controller means the input will be the desired position where as 2)velocity based controller means the input will be the desired velocity of the arm.

3)Joints: We need to define the joints which are being controlled by the controller.

4)stop_trajectory_duration: Controls the duration of the stop motion.

5)state_publish_rate: Frequency (in Hz) at which the controller state is published.

6)action_monitor_rate: Frequency (in Hz) at which the action goal status is monitored. This is an advanced parameter that should not require changing.

#  Writing a launch file for Gazebo trajectory controllers
This is the final step. This is where i spawn the arm in Gazebo and Rviz and launch the trajectory controller and MoveIt! interface.

Make a launch file named ur5_bringup_moveit.launch in the launch folder of pkg_moveit_ur5_1 package.
Launch the file by running the following command and plan the path.
command:"roslaunch pkg_moveit_ur5_1 ur5_bringup_moveit.launch"
![image](https://user-images.githubusercontent.com/75885970/114542580-bc592e80-9c75-11eb-8ceb-45a352b385bc.png)
![image](https://user-images.githubusercontent.com/75885970/114542631-c713c380-9c75-11eb-9ba4-721a8a408147.png)
![image](https://user-images.githubusercontent.com/75885970/114542647-cbd87780-9c75-11eb-8e82-f17e27c7df6d.png)
![image](https://user-images.githubusercontent.com/75885970/114542668-d09d2b80-9c75-11eb-8b81-57e5591d6be3.png)

-MoveIt! provides C++ and Python APIs to command the move_group node to perform actions such as Pick and Place, IK, FK etc.
-Here i have used c++ as well as python apis.

# Adding objects to the Planning Scene
In MoveIt! Setup Assistant you generate collision matrix which prevents self-collision i.e it make sure that the Motion Planning Algorithm does not plan a path in which the links of the Robotic Arm are colliding with each other.

Robotic Manipulators typically are used to interact with the environment around them. We won't want the links of the manipulator to collide with its environment also.

To achieve this we can add the object in the Planning Scene of MoveIt! so that the Motion Planning Algorithm consider the object as an obstacle and plans a path avoiding this obstacle.

But, not all objects in the environment are obstacles.
Consider a Robot whose job is to pick a package from a shelf and place it in a box.

In this case the shelf can be considered as an obstacle and the manipulator should not collide with it.

-In MoveIt! you can add the mesh files of such obstacles in the Planning Scene.

On the other hand the package on the shelf is not an obstacle. The end-effector of the arm needs to go the package to pick it. After picking the package the arm needs to ensure that the package also does not collide with the shelf.

-In MoveIt! you can dynamically attach/detach objects to the end-effector. When attached, Motion Planners will then consider the object also in Motion Planning.





