#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment1/RcAction.h>

bool start = false; 
bool goal_reached = false;
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res)
{
    if (req.command == "start")
    {
        start = true;
    }
    else
    {
        start = false;
    }
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_machine");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("/user_interface", user_interface);
    ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
    actionlib::SimpleActionClient<rt2_assignment1::RcAction> ac("go_to_point", true);

    rt2_assignment1::RandomPosition rp;
    rt2_assignment1::RcGoal goal;

    rp.request.x_max = 5.0;
    rp.request.x_min = -5.0;
    rp.request.y_max = 5.0;
    rp.request.y_min = -5.0;

    while(ros::ok())
    {
        ros::spinOnce();
        if (start)
        {
            client_rp.call(rp);
            goal.x = rp.response.x;
            goal.y = rp.response.y;
            goal.theta = rp.response.theta;

            std::cout << "\nGoing to the position: x= " << goal.x << " y= " << goal.y << " theta = " << goal.theta << std::endl;
            ac.sendGoal(goal);
            while (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ros::spinOnce();
                if (start == false)
                {
                  //cancel all the goals
                  ac.cancelAllGoals();
                  goal_reached=false;

                  break;
                }
                goal_reached=true;
            }
        }

    if(goal_reached)
    {
        std::cout << "\nPosition reached" << std::endl;
        goal_reached = false;
    }
    }
  return 0 ;
}
