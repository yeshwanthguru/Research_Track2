
# Research_Track2
Name  :yeshwanth guru krishna kumar\
Reg No:S5059111\
RT2-Assignment-1


This main branch defines the controlling the robot in vrep scene with the ros.
In this repository there are three sub branches such as **action**,**master** and **ros2** with the given packages as per the  different requirment.

## PACKAGE DESCRIPTION AND SIMULATION EXECUTION PROCESS:
## MAIN BRANCH ---> ROS CONTROLS ROBOT IN VREP ENVIRONMENT:
At initial the package is cloned from the repository 

   https://github.com/CarmineD8/rt2_assignment1.git

   and the requirment is to control a robot in the vrep scene with the ROS is developed and updated in this repository

   https://github.com/yeshwanthguru/Research_Track2.git


   package tree
   ![Peek 2021-12-26 16-11](https://user-images.githubusercontent.com/72270080/147417311-ef6c9f2c-6027-4f3a-ae81-9801f304195a.png)

  
 

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

## **CMakelist.txt**
CMakeLists. txt file contains a set of directives and instructions describing the project's source files and targets (executable, library, or both). When you create a new project, CLion generates CMakeLists. ... txt files to describe the build, contents, and target rules for a subdirectory of the assignment.

## **dr12robot_roscontroll.ttt**
This is Vrep scene file contains the scene developed and controlled the robot with the lua script and here we used dr12 robot for this scene.This will be controlled with the ros by using this package its executing process will be described below .

## **coppeliasim_roscontroll.launch** ##
This is launch file for the vrep scene control which will execute the nodes to turn on in this section Vrep plays the simulation section and so it is linked with the ros with plugin which can be identified with the load success command int the coppelia launch terminal.And this is the launch file to activate the nodes in the ROS Side.

 ![Screenshot from 2021-12-26 21-35-47](https://user-images.githubusercontent.com/72270080/147419470-13d42ab0-6bf2-418f-b2d3-73cff6957c7f.png)



## **package.xml**  ##
The package manifest is an XML file called package. xml that must be included with any catkin-compliant package's root folder. This file defines properties about the package such as the package name, version numbers, authors, maintainers, and dependencies on other catkin packages.

## **srv**  ##
ROS uses a simplified service description language ("srv") for describing ROS service types. This builds directly upon the ROS msg format to enable request/response communication between nodes. Service descriptions are stored in .srv files in the srv/ subdirectory of a package. 

                      [*]command.srv
                      [*]position.srv
                      [*]Randomposition.srv

## **Code Execution process ** ##

Above the complete package Architecture package has been describes now the package execution process will be seen


Our Complete package with respect to the ros side we needs the nodes to be activated and it can be launched using the command in Terminal1

                  roslaunch rt2_assignment1 coppeliasim_roscontroll.launch

Once this is launched you will get the user interface to control the robot with the input 1 to start the robot and 0 to stop the robot.

In the Terminal2 we should launch the coppelia sim once the ros side process done during the sim environment launched we can see the command in the crep launch terminal that **ROS LOADING SUCCESS** once getting in to the vrep directory we can launch the \
                      
                      
                                  ./Coppeliasim.sh 

Once the coppelia sim is open you can import the scene file **dr12robot_roscontroll.ttt** from the repository with the option open scene in the coppeliasim.once the scene is loaded in the environment then should click on the play button to make the scene to run and then in the ROS side we want to give the input for the robot control once the input is given the robot simulation takes place in which this scene developed and control with the lua script with the ROS connection in the scripts which is developed with the threaded scripts .

And the final output of vrep scene is attached for reference in  screenshot format




![WhatsApp Image 2022-02-02 at 9 55 11 PM](https://user-images.githubusercontent.com/72270080/152235883-16a5a57a-a6e7-4a01-8f63-9e6a12f95d1e.jpeg)


 

 


    
