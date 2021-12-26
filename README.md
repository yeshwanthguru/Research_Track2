# Research_Track2
Name  :yeshwanth guru krishna kumar\
Reg No:S5059111\
RT2-Assignment-1\


This main branch defines the controlling the robot in vrep scene with the ros.
In this repository there are three sub branches such as **action**,**master** and **ros2** with the given packages as per the  different requirment.

## PACKAGE DESCRIPTION AND SIMULATION EXECUTION PROCESS:
## MAIN BRANCH ---> ROS CONTROLS ROBOT IN VREP ENVIRONMENT:
At initial the package is cloned from the repository 

   https://github.com/CarmineD8/rt2_assignment1.git

   and the requirment is to control a robot in the vrep scene with the ROS is developed and updated in this repository

   https://github.com/yeshwanthguru/Research_Track2.git


   package tree
 

Among the given packages there are four nodes developed 


You can execute the process with the command:

roslaunch rt2_assignment1 coppeliasim_roscontroll.launch

with this above command the package will start the nodes in a new terminal should be open and then coppelia sim to be opened during the launch of the simulation environ ment we will find the **ros loading succeed** in the terminal of VREP launch which give the visual conformation that **ROS** is linked  with the coppelia sim. 


## Our package consist of four nodes 
**go_to_point**\
**user_interface**\
**position_service**\
**state_machine** 

## **go_to_point**:
go_to_point node  will implements a service to start or stop the robot towards a point in the environment.And the service implemented in this node would be able to drive the robot towards a certain position in space of (x,y) and with a certain angle (theta)
## **user_interface**:
User interface node is to control the robot with the attributes start and stop and calls the service implemented in the state_machine node. 
## **position_service**:
The node PositionServer, which implements a random Position Server.And the service implemented in this node will gives the random values for x,y and thetain which the the values of x and y should be limited between min and max value. 
## **state_machine**:
state_machine node implements a service to start or stop the robot and call the other two service to drive the robot.



    
