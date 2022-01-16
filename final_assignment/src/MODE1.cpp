#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include "move_base_msgs/MoveBaseAction.h"



int main (int argc, char **argv)
{
  // Initializing the MODE1_node
  ros::init(argc, argv, "MODE1_node");
  ros::NodeHandle nh;

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  MoveBaseClient ac("move_base", true);
  ac.waitForServer();
  move_base_msgs::MoveBaseGoal goal;

  float x, y;
  int m;

  ros::Rate loop_rate(10);
  while(ros::ok()){
    ros::param::get("/mode", m);
    // Mode 1 activated
    if (m == 1){
      std::cout << "Choose the (x,y) position or type '00 00' to return to the user interface\n";
      std::cin >> x >> y;
      //Return to the user interface
      if( x == 00 && y == 00) {
        ros::param::set("/mode", 0);
      }
      else{
      std::cout << "Please wait for the end of the execution\n";
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.pose.orientation.w = 1.0;
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = x;
      goal.target_pose.pose.position.y = y;
      ac.sendGoal(goal);

      // Position reached before timeout
      if (ac.waitForResult(ros::Duration(30.0)))
        std::cout << "Position (" << x << ";" << y << ") reached before the timeout\n";
      else{
        std::cout << "Position not reached before timeout\n";
        ac.cancelGoal();
        std::cout << "Deleted position\n";
      }
    }
    }
    else{
      loop_rate.sleep();
      continue;
    }

    ros::spinOnce();
		loop_rate.sleep();
  }

  return 0;
}
