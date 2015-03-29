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
int p = -1;
int i = -1;
int d = -1;


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
  // sleepms(20);

  cout<<"- GetValue(_BLSPEED, 1)...";
  if((status = device.GetValue(_BLSPEED, 1, result)) != RQ_SUCCESS)
    cout<<"failed --> "<<status<<endl;
  else
    cout<<"returned --> "<<result<<endl;
}

void PCallback(const std_msgs::Int64::ConstPtr& msg)
{
  int cmd = msg->data;
  if(cmd != p){
    p = cmd;
    int result;
    printf("- SetCommand(_KP, 1, %i)...", cmd);
    if((status = device.SetConfig(_KP, 1, cmd)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"succeeded."<<endl;
  }
  

  //Wait 10 ms before sending another command to device
  // sleepms(20);

  // cout<<"- GetConfig(_KP, 1)...";
  // if((status = device.GetConfig(_KP, 1, result)) != RQ_SUCCESS)
  //   cout<<"failed --> "<<status<<endl;
  // else
  //   cout<<"returned --> "<<result<<endl;
}

void ICallback(const std_msgs::Int64::ConstPtr& msg)
{
  int cmd = msg->data;
  if(cmd != i) {
    i = cmd;
    int result;
    printf("- SetConfig(_KI, 1, %i)...", cmd);
    if((status = device.SetConfig(_KI, 1, cmd)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"succeeded."<<endl;
  }
  

  //Wait 10 ms before sending another command to device
  // sleepms(20);

  // cout<<"- GetConfig(_KI, 1)...";
  // if((status = device.GetConfig(_KI, 1, result)) != RQ_SUCCESS)
  //   cout<<"failed --> "<<status<<endl;
  // else
  //   cout<<"returned --> "<<result<<endl;
}

void DCallback(const std_msgs::Int64::ConstPtr& msg)
{
  int cmd = msg->data;
  if(cmd != d){
    d = cmd;
    int result;
    printf("- SetConfig(_KD, 1, %i)...", cmd);
    if((status = device.SetConfig(_KD, 1, cmd)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"succeeded."<<endl;
  }

  //Wait 10 ms before sending another command to device
  // sleepms(20);

  // cout<<"- GetConfig(_KD, 1)...";
  // if((status = device.GetConfig(_KD, 1, result)) != RQ_SUCCESS)
  //   cout<<"failed --> "<<status<<endl;
  // else
  //   cout<<"returned --> "<<result<<endl;
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

  // cout<<"Setting max RPM";
  // setConfig(_MXRPM, 1, 500);
  // //Wait 10 ms before sending another command to device
  // sleepms(20);
  // setConfig(_MXRPM, 2, 500);
  // sleepms(20);

  // cout<<"Setting amp limit";
  // setConfig(_ALIM, 1, 125);
  // sleepms(20);
  // setConfig(_ALIM, 2, 125);
  // sleepms(20);

  // cout<<"Set feedback mode to hall sensors";
  // setConfig(_BLFB,1,0);
  // sleep(20);
  // setConfig(_BLFB,2,0);
  // sleep(20);

  // cout<<"Set number of poles";
  // setConfig(_BPOL,1,30);
  // sleep(20);
  // setConfig(_BPOL,2,30);
  // sleep(20);

  // cout<<"Set encoder to feedback";
  // setConfig(_EMOD,1,18);
  // sleep(20);
  // setConfig(_EMOD,2,34);
  // sleep(20);

  // cout<<"Set encoder PPR value";
  // setConfig(_EPPR,1,1000);
  // sleep(20);
  // setConfig(_EPPR,2,1000);
  // sleep(20);

  // cout<<"Setting motor mode";
  // setConfig(_MMOD,1,0);
  // sleep(20);
  // setConfig(_MMOD,2,0);
  // sleep(20);

  ros::Subscriber s_sub = n.subscribe("motor_speed", 1000, motorCallback);
  ros::Subscriber p_sub = n.subscribe("p_value", 1000, PCallback);
  ros::Subscriber i_sub = n.subscribe("i_value", 1000, ICallback);
  ros::Subscriber d_sub = n.subscribe("d_value", 1000, DCallback);


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}