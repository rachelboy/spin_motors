#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"

#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>

#include "roboteq_code/RoboteqDevice.h"
#include "roboteq_code/ErrorCodes.h"
#include "roboteq_code/Constants.h"
#include "spin_motors/Encoder.h"

using namespace std;

const int RIGHT = 1;
const int LEFT = 2;

RoboteqDevice device;
int status;
string response;
int max_motor_rpm;
float wheel_circumfrence;

void rightCallback(const std_msgs::Float32::ConstPtr&);
void leftCallback(const std_msgs::Float32::ConstPtr&);
int velToCmd(float);
void setConfig(int, int, int);


void rightCallback(const std_msgs::Float32::ConstPtr& msg) {
  float vel = msg->data;
  int cmd = velToCmd(vel);
  if((status = device.SetCommand(_GO, RIGHT, cmd)) != RQ_SUCCESS)
    printf("- SetCommand(_GO, RIGHT, %i) failed\n", cmd);

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
  if((status = device.SetCommand(_GO, LEFT, cmd)) != RQ_SUCCESS)
    printf("- SetCommand(_GO, LEFT, %i) failed\n", cmd);
}

void setP(const std_msgs::Float32::ConstPtr& msg) {
  int p = (int) 10*(msg->data);
  setConfig(_KP, RIGHT, p);
  setConfig(_KP, LEFT, p);
}

void setI(const std_msgs::Float32::ConstPtr& msg) {
  int i = (int) 10*(msg->data);
  setConfig(_KI, RIGHT, i);
  setConfig(_KI, LEFT, i);
}

void setD(const std_msgs::Float32::ConstPtr& msg) {
  int d = (int) 10*(msg->data);
  setConfig(_KD, RIGHT, d);
  setConfig(_KD, LEFT, d);
}

int velToCmd(float vel) {
  // 0.1524 m is the diameter of the wheel
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
  string port_handle;
  n.param<std::string>("mc_port",port_handle,"/dev/ttyACM0");
  status = device.Connect(port_handle);

  if(status != RQ_SUCCESS)
  {
    cout<<"Error connecting to device: "<<status<<"."<<endl;
    return 1;
  }

  string max_motor_rpm_str; 
  n.param<std::string>("max_motor_rpm",max_motor_rpm_str,"5000");
  max_motor_rpm = atoi(max_motor_rpm_str.c_str());

  string wheel_circumfrence_str;
  n.param<std::string>("wheel_circumfrence",wheel_circumfrence_str,"0.47878");
  wheel_circumfrence = atof(wheel_circumfrence_str.c_str());

  ros::Subscriber r_sub = n.subscribe("rwheel_vtarget", 1000, rightCallback);
  ros::Subscriber l_sub = n.subscribe("lwheel_vtarget", 1000, leftCallback);
  ros::Subscriber p_sub = n.subscribe("kp", 1000, setP);
  ros::Subscriber i_sub = n.subscribe("ki", 1000, setI);
  ros::Subscriber d_sub = n.subscribe("kd", 1000, setD);
  ros::Publisher enc_pub = n.advertise<spin_motors::Encoder>("encoder_count_rel", 1000);

  ros::Rate loop_rate(10);

  while(ros::ok()) {
    int status;
    int enc_r;
    int enc_l;
    if((status = device.GetValue(_CR, RIGHT, enc_r)) != RQ_SUCCESS)
      cout<<"GetValue(_CR, 1) failed --> "<<status<<endl;
    else if((status = device.GetValue(_CR, LEFT, enc_l)) != RQ_SUCCESS)
      cout<<"GetValue(_CR, 2)failed --> "<<status<<endl;
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