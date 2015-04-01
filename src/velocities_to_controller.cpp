#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

using namespace std;

const int RIGHT = 1;
const int LEFT = 2;

RoboteqDevice device;
int status;
string response;

void rightCallback(const std_msgs::Float32::ConstPtr&);
void leftCallback(const std_msgs::Float32::ConstPtr&);
int velToCmd(float);
void setConfig(int, int, int);


void rightCallback(const std_msgs::Float32::ConstPtr& msg) {
  float vel = msg->data;
  int cmd = velToCmd(vel);
  printf("- SetCommand(_GO, RIGHT, %i)...", cmd);
  if((status = device.SetCommand(_GO, RIGHT, cmd)) != RQ_SUCCESS)
    cout<<"failed --> "<<status<<endl;
  else
    cout<<"succeeded."<<endl;

  // int result;
  // cout<<"- GetValue(_BLSPEED, 1)...";
  // if((status = device.GetValue(_BLSPEED, 1, result)) != RQ_SUCCESS)
  //   cout<<"failed --> "<<status<<endl;
  // else
  //   cout<<"returned --> "<<result<<endl;
}

void leftCallback(const std_msgs::Float32::ConstPtr& msg) {
  float vel = msg->data;
  int cmd = velToCmd(vel);
  printf("- SetCommand(_GO, LEFT, %i)...", cmd);
  if((status = device.SetCommand(_GO, LEFT, cmd)) != RQ_SUCCESS)
    cout<<"failed --> "<<status<<endl;
  else
    cout<<"succeeded."<<endl;
}

int velToCmd(float vel) {
  // 0.1524 m is the diameter of the wheel
  int max_motor_rpm = 5000;
  float wheel_circumfrence = 3.1415926535*0.1524;
  float wheel_rpm = vel*60/wheel_circumfrence;
  float motor_rpm = wheel_rpm*10;
  // make sure we don't go over 100% or under -100%
  float per_cmd = fmax(fmin(motor_rpm/max_motor_rpm,1.0),-1.0);
  int cmd = int(per_cmd*1000);
  return cmd;
}

void setConfig(int cmd, int chnl, int param){
  printf("- SetConfig(%i, %i, %i)...", cmd, chnl, param);
  if((status = device.SetConfig(cmd, chnl, param)) != RQ_SUCCESS)
    cout<<"failed --> "<<status<<endl;
  else
    cout<<"succeeded."<<endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "spin_motors");

  ros::NodeHandle n;

  response = "";
  status = device.Connect("/dev/ttyACM0");

  if(status != RQ_SUCCESS)
  {
    cout<<"Error connecting to device: "<<status<<"."<<endl;
    return 1;
  }


  ros::Subscriber r_sub = n.subscribe("rwheel_vtarget", 1000, rightCallback);
  ros::Subscriber l_sub = n.subscribe("lwheel_vtarget", 1000, leftCallback);



  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}