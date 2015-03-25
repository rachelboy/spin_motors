#include "ros/ros.h"
#include "std_msgs/Int64.h"

#include <iostream>
#include <stdio.h>
#include <string.h>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

using namespace std;

RoboteqDevice device;
int status;
string response;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void motorCallback(const std_msgs::Int64::ConstPtr& msg)
{
  int cmd = msg->data;
  int result;
  printf("- SetCommand(_GO, 1, %i)...", cmd);
  if((status = device.SetCommand(_GO, 1, cmd)) != RQ_SUCCESS)
    cout<<"failed --> "<<status<<endl;
  else
    cout<<"succeeded."<<endl;

  //Wait 10 ms before sending another command to device
  sleepms(10);

  cout<<"- GetValue(_BLSPEED, 1)...";
  if((status = device.GetValue(_BLSPEED, 1, result)) != RQ_SUCCESS)
    cout<<"failed --> "<<status<<endl;
  else
    cout<<"returned --> "<<result<<endl;
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "spin_motors");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  response = "";
  status = device.Connect("/dev/ttyACM0");

  if(status != RQ_SUCCESS)
  {
    cout<<"Error connecting to device: "<<status<<"."<<endl;
    return 1;
  }

  cout<<"- SetConfig(_MXRPM, 1, 500)...";
  if((status = device.SetConfig(_MXRPM, 1, 500)) != RQ_SUCCESS)
    cout<<"failed --> "<<status<<endl;
  else
    cout<<"succeeded."<<endl; 

  //Wait 10 ms before sending another command to device
  sleepms(10);

  cout<<"- SetConfig(_MXRPM, 2, 500)...";
  if((status = device.SetConfig(_MXRPM, 2, 500)) != RQ_SUCCESS)
    cout<<"failed --> "<<status<<endl;
  else
    cout<<"succeeded."<<endl; 

  //Wait 10 ms before sending another command to device
  sleepms(10);

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("motor_speed", 1000, motorCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}