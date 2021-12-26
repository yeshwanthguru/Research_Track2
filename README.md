
# Research_Track2
Name  :yeshwanth guru krishna kumar\
Reg No:S5059111\
RT2-Assignment-1 ROS ACTION SERVER INCLUDED ROBOT CONTROL IN GAZEBO SIMULATION


This action branch defines the controlling the robot in GAZEBO simulation with the ros.
Including the action server in the given package nodes .ROS Actions have a client-to-server communication relationship with a specified protocol. The actions use ROS topics to send goal messages from a client to the server. You can cancel goals using the action client. ... This function enables you to return the result message, final state of the goal and status of the server.

## PACKAGE DESCRIPTION AND SIMULATION EXECUTION PROCESS:
## ACTION BRANCH ---> ROS ACTION SERVER INCLUDED ROBOT CONTROL IN GAZEBO SIMULATION
:
At initial the package is cloned from the repository 

   https://github.com/CarmineD8/rt2_assignment1.git

   and the requirment is to control a robot in the vrep scene with the ROS is developed and updated in this repository sub branch action

   https://github.com/yeshwanthguru/Research_Track2.git

Among the given packages there are four nodes developed 


You can execute the process with the command:

                   roslaunch rt2_assignment1 sim.launch

## Our package consist of four nodes 
**go_to_point**\
**user_interface**\
**position_service**\
**state_machine** 

## **go_to_point**:
go_to_point node  will implements a service to start or stop the robot towards a point in the environment.And the service implemented in this node would be able to drive the robot towards a certain position in space of (x,y) and with a certain angle (theta) and the action server is included.
## **user_interface**:
User interface node is to control the robot with the attributes start and stop and calls the service implemented in the state_machine node. 
## **position_service**:
The node PositionServer, which implements a random Position Server.And the service implemented in this node will gives the random values for x,y and theta in which the the values of x and y should be limited between min and max value. 
## **state_machine**:
state_machine node implements a service to start or stop the robot and call the other two service to drive the robot.

## **CMakelist.txt**
CMakeLists. txt file contains a set of directives and instructions describing the project's source files and targets (executable, library, or both). When you create a new project, CLion generates CMakeLists. ... txt files to describe the build, contents, and target rules for a subdirectory of the assignment.


## **package.xml**  ##
The package manifest is an XML file called package. xml that must be included with any catkin-compliant package's root folder. This file defines properties about the package such as the package name, version numbers, authors, maintainers, and dependencies on other catkin packages.

## **srv**  ##
ROS uses a simplified service description language ("srv") for describing ROS service types. This builds directly upon the ROS msg format to enable request/response communication between nodes. Service descriptions are stored in .srv files in the srv/ subdirectory of a package. 

                      [*]command.srv
                      [*]position.srv
                      [*]Randomposition.srv

## **Code Execution process ** ##

Above the complete package Architecture package has been describes now the package execution process will be seen




       

 
 


    