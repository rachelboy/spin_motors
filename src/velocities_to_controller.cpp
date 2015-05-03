#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>

#include "roboteq_code/RoboteqDevice.h"
#include "roboteq_code/ErrorCodes.h"
#include "roboteq_code/Constants.h"
#include "spin_motors/Encoder.h"

using namespace std;

// define motor mapping
const int RIGHT = 2;
const int LEFT = 1;

RoboteqDevice device;
int status;
int max_motor_rpm; // set in main(), with rosparam
float wheel_circumfrence; // set in main(), with rosparam
bool estop = false;

void rightCallback(const std_msgs::Float32::ConstPtr&);
void leftCallback(const std_msgs::Float32::ConstPtr&);
void estopCallback(const std_msgs::Bool::ConstPtr&);
int velToCmd(float);
void setConfig(int, int, int);


void rightCallback(const std_msgs::Float32::ConstPtr& msg) {
  /*
  set the right motor speed
  */
  if(!estop) { // if the estop is not engaged
    float vel = msg->data;
    int cmd = velToCmd(vel); // translate velocity to motor command
    printf("- SetCommand(_GO, RIGHT, %i)...", cmd);
    if((status = device.SetCommand(_GO, RIGHT, cmd)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"succeeded."<<endl;
  } else {
    // if the estop is engaged, set speed to 0
    printf("- SetCommand(_GO, RIGHT, 0)...");
    if((status = device.SetCommand(_GO, RIGHT, 0)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"succeeded."<<endl;
  }
}

void leftCallback(const std_msgs::Float32::ConstPtr& msg) {
  /* 
  set the left motor speed
  */
  if(!estop){ // if the estop is not engaged
    float vel = msg->data;
    // translate velocity to motor command, and reverse it b/c motor is reversed
    int cmd = -1*velToCmd(vel);
    printf("- SetCommand(_GO, LEFT, %i)...", cmd);
    if((status = device.SetCommand(_GO, LEFT, cmd)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"succeeded."<<endl;
  } else {
    // if the estop is engaged, set speed to 0
    printf("- SetCommand(_GO, LEFT, 0)...");
    if((status = device.SetCommand(_GO, LEFT, 0)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"succeeded."<<endl;
  }
}

void estopCallback(const std_msgs::Bool::ConstPtr& msg) {
  estop = msg->data;
}

int velToCmd(float vel) {
  /* 
  translate wheel velocity to the motor speed command
  roboteq takes speed commands as a percentage of the maximum speed
  So, 500 would be 50% maximum speed
  -500 would be 50% maximum speed, but in reverse
  */
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

  // open a connection to the roboteq
  string port_handle;
  n.param<std::string>("mc_port",port_handle,"/dev/roboteq");
  status = device.Connect(port_handle);

  if(status != RQ_SUCCESS)
  {
    cout<<"Error connecting to device: "<<status<<"."<<endl;
    return 1;
  }

  // set the maximum motor rpm (motor_rpm = 10*wheel_rpm)
  string max_motor_rpm_str; 
  n.param<std::string>("max_motor_rpm",max_motor_rpm_str,"5000");
  max_motor_rpm = atoi(max_motor_rpm_str.c_str());

  // set the wheel circumference in meters
  string wheel_circumfrence_str;
  n.param<std::string>("wheel_circumfrence",wheel_circumfrence_str,"0.47878");
  wheel_circumfrence = atof(wheel_circumfrence_str.c_str());

  ros::Subscriber r_sub = n.subscribe("rwheel_vtarget", 1000, rightCallback);
  ros::Subscriber l_sub = n.subscribe("lwheel_vtarget", 1000, leftCallback);
  ros::Subscriber estop_sub = n.subscribe("estop", 1000, estopCallback);
  ros::Publisher enc_pub = n.advertise<spin_motors::Encoder>("encoder_count_rel", 1000);

  ros::Rate loop_rate(15);

  int resp;
  int enc_r;
  int enc_l;
  while(ros::ok()) {
    // get change in right and left encoder counts and publish it
    if((resp = device.GetValue(_CR, RIGHT, enc_r)) != RQ_SUCCESS)
      cout<<"GetValue(_CR, 1) failed --> "<<resp<<endl;
    else if((resp = device.GetValue(_CR, LEFT, enc_l)) != RQ_SUCCESS)
      cout<<"GetValue(_CR, 2)failed --> "<<resp<<endl;
    else {
      spin_motors::Encoder enc_msg;
      enc_msg.right = enc_r;
      enc_msg.left = enc_l;
      enc_pub.publish(enc_msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
