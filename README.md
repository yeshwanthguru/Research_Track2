
# Research_Track2
Name  :yeshwanth guru krishna kumar\
Reg No:S5059111\
RT2-Assignment-1



## PACKAGE DESCRIPTION AND SIMULATION EXECUTION PROCESS:
## MAIN BRANCH ---> ROS AND ROS2 BRIDGING TO CONTROL A MOBILE ROBOT
At initial the package is cloned from the repository 

   https://github.com/CarmineD8/rt2_assignment1.git
package tree
   ![Peek 2021-12-26 16-11](https://user-images.githubusercontent.com/72270080/147417311-ef6c9f2c-6027-4f3a-ae81-9801f304195a.png)
And the package is clonned inside the ros1 environment workspace and then later clone the ros 2 package which is develeopen and updated in ther repository

   https://github.com/yeshwanthguru/Research_Track2.git -b ros2

Among the given packages there are four nodes developed 


## Our packages consist of four nodes two each in different workspace.

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

## **CMakelist.txt**
CMakeLists. txt file contains a set of directives and instructions describing the project's source files and targets (executable, library, or both). When you create a new project, CLion generates CMakeLists. ... txt files to describe the build, contents, and target rules for a subdirectory of the assignment.

## **package.xml**  ##
The package manifest is an XML file called package. xml that must be included with any catkin-compliant package's root folder. This file defines properties about the package such as the package name, version numbers, authors, maintainers, and dependencies on other catkin packages.

## **srv** ##
ROS uses a simplified service description language ("srv") for describing ROS service types. This builds directly upon the ROS msg format to enable request/response communication between nodes. Service descriptions are stored in .srv files in the srv/ subdirectory of a package. 

                      [*]command.srv
                      [*]position.srv
                      [*]Randomposition.srv
## **my_mapping** ##
This file is to map the ros and ros2 workspace ......

## **Code Execution process ** ##

For the complete setup some process to be followed to set the environment on following process

## **ROS_WS **##
At initial during time to setting the ros setup workspace by sourcing the terminal one with the ros source file and procedd the further process to set the environment with the ros script in one terminal and for the ros2 environment want to source ros2 environment with the source script file and clone the developed package into it and in one terminal source ros and ros2 script together with a script .Once source is done package is cloned in specific workspace to proceed the ros1_bridge

                       git clone https://github.com/ros2/ros1_bridge.git
                       
clone the above link inside the ros2_ws and give the below command to build the workspace


                      colcon build --symlink-install --packages-skip ros1_bridge  
                      
For the compile of rosbridge everything done on the basis of mapping rule contained in the package and after this in the terminal of ros12 sourced terminal 

                       
                      colcon build --packages-select ros1_bridge --cmake-force-configure
                      
                      
 Once everything is done after building the package launch the fle with the following command 
 
 
 [*]Terminal 1(ros)--------->roslaunch rt2_assignment1 ros_ros2_bridge.launch
 [*]Terminal 2(ros2)--------->ros2 run ros1_bridge dynamic_bridge
 [*]Terminal 3(ros1 and 2)--------->ros2 launch rt2_assignment1 sim_container.py
 
 
 
 


                      
 
 

 


    
