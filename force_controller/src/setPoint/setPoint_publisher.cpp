// ROS 
#include <ros/ros.h>

// ROS Msg Type for setPoint
#include <force_controller/force_contr_pubsub.h>

// ROS Msg Type
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"

// STD Libs
#include <string>
#define RATE 500 // Parameter to set publication rate
//---------------------------------------------------------------------------------------------------
// Can get setPiont from launch file, command line, or another publisher--this yet to be implemented.
// setPoint looks like:
//---------------------------------------------------------------------------------------------------
// #Number of Controllers
// int32 num_ctrls
// 
// # Name of the type(s) of Controller(s): force or moment
// string[] type
// 
// # Desired force/moment (3D) for dominant controller
// geometry_msgs/Vector3[] domDes
// 
// # Gains force/moment (3D) for dominant controller
// geometry_msgs/Vector3[] domGains
// 
// # Desired force/moment (3D) for subordinate controller
// geometry_msgs/Vector3[] subDes
// 
// # Gains force/moment (3D) for subordinate controller
// geometry_msgs/Vector3[] subGains
//--------------------------------------------------------------------------------------------------- 
// Command line arguments can be expected in any of the following forms:
// numCtrls mode1 desx1 desy1 desz1                                                   # argc= 6 => 1 controller with setpoint 
// numCtrls mode1 desx1 desy1 desz1 gx1 gy1 gz1                                       # argc= 9 => 1 controller with setpoint and mode
//--------------------------------------------------------------------------------------------------- 
// numCtrls mode1 desx1 desy1 desz1 mode2 desx2 desy2 desz2                           # argc=10 => 2 controller with setpoints
// numCtrls mode1 desx1 desy1 desz1 gx1 gx2 gx3 mode2 desx2 desy2 desz2               # argc=13 => 2 controller 1st one with setpoints and gains, 2nd one only with setpoint
// numCtrls mode1 desx1 desy1 desz1 gx1 gx2 gx3 mode2 desx2 desy2 desz2 gx2 gy2 gz2   # argc=16 => 2 controller with setpoints and gains
//--------------------------------------------------------------------------------------------------- 
int main(int argc, char **argv)
{
  // Start the node
  ros::init(argc, argv, "setPoint_publisher");
  ros::NodeHandle n;

  // A. Handle command line arguments
  int numCtrls=1;
  int mode1=0,  mode2=0;
  double x1=0,  x2=0;
  double y1=0,  y2=0; 
  double z1=0,  z2=0; 
  double gx1=0, gx2=0;
  double gy1=0, gy2=0; 
  double gz1=0, gz2=0;

  /*** Variable Initialization according to user input ***/
  // User only entered desired setpoint. 
  numCtrls=atof(argv[1]);

  // Dom Controller with SetPoint
  if(argc==6 && numCtrls==1)
    {
      mode1=atof(argv[2]);      
      x1   =atof(argv[3]);
      y1   =atof(argv[4]);
      z1   =atof(argv[5]);
    }
  
  // Dom Controller with SetPoint and Gains
  else if(argc==9 && numCtrls==1)
    {
      mode1=atof(argv[2]);      
      x1   =atof(argv[3]);
      y1   =atof(argv[4]);
      z1   =atof(argv[5]);
      gx1  =atof(argv[6]);
      gy1  =atof(argv[7]);
      gz1  =atof(argv[8]);
    }

  // Dom and Sub Controller with SetPoint
  else if(argc==10 && numCtrls==2)
    {
      mode1=atof(argv[2]);      
      x1   =atof(argv[3]);
      y1   =atof(argv[4]);
      z1   =atof(argv[5]);

      mode2=atof(argv[6]);
      x2   =atof(argv[7]);
      y2   =atof(argv[8]);
      z2   =atof(argv[9]);
    }

  // Dom Controller with SetPoint and Gains and SubController with SetPoint
  else if(argc==13 && numCtrls==2)
    {
      mode1=atof(argv[2]);      
      x1   =atof(argv[3]);
      y1   =atof(argv[4]);
      z1   =atof(argv[5]);
      gx1  =atof(argv[6]);
      gy1  =atof(argv[7]);
      gz1  =atof(argv[8]);

      mode2=atof(argv[9]);
      x2   =atof(argv[10]);
      y2   =atof(argv[11]);
      z2   =atof(argv[12]);
    }

  // Both Controllers with SetPoint and Gains
  else if(argc==16 && numCtrls==2)
    {
      mode1=atof(argv[2]);      
      x1   =atof(argv[3]);
      y1   =atof(argv[4]);
      z1   =atof(argv[5]);
      gx1  =atof(argv[6]);
      gy1  =atof(argv[7]);
      gz1  =atof(argv[8]);

      mode2=atof(argv[9]);
      x2   =atof(argv[10]);
      y2   =atof(argv[11]);
      z2   =atof(argv[12]);
      gx2  =atof(argv[13]);
      gy2  =atof(argv[14]);
      gz2  =atof(argv[15]);
    }

  else
    {
      ROS_INFO("Wrong number of arguments in the commmand line.");
    }

  // B. Initialize ROS data
  // Get side Parameter
  std::string side_="";
  n.param<std::string>("side", side_, "right");

  // Create the publisher 
  ros::Publisher setPoint_pub = n.advertise<force_controller::setPoint>("/" + side_ + "/force_control/setPoint",1); 

  // Populate the setPoint msg
  force_controller::setPoint sP;

  // Set Control Type, Desired set-point and gains for 1 or 2 controllers. 
  if(numCtrls < 1 || numCtrls > 2)
    {
      ROS_ERROR("Wrong number of controllers given");
      return -1;
    }
  else
    {
      // Number of Controllers
      sP.num_ctrls=numCtrls;

      /********** Dominant Controller ************/
      /*** CONTROL TYPE ***/
      // Dominant Controller Type
      if(mode1==0)           sP.domType="force";
      else if(mode1==1)      sP.domType="moment";

      /*** SETPOINT ***/
      geometry_msgs::Vector3 d;
      d.x=x1; d.y=y1; d.z=z1;
      sP.domDes.push_back(d);

      /*** GAINS ***/
      if(argc==9 || argc==13 || argc==16) {
        d.x=gx1; d.y=gy1; d.z=gz1;
        sP.domGains.push_back(d);
      }        
      
      /************ Subordinate Controller **********/
      if(numCtrls==2) // If condition not true it's okay if all are empty
        {
          /*** CONTROL TYPE ***/
          // Subordinate Controller Type
          if(mode2==0)           sP.subType="force";
          else if(mode2==1)      sP.subType="moment";

          /*** SETPOINT ***/
          d.x=x2; d.y=y2; d.z=z2;
          sP.subDes.push_back(d);

          /*** Gains ***/
          if(argc==16) {
            d.x=gx2; d.y=gy2; d.z=gz2;
            sP.subGains.push_back(d);
          }        
        }     
    }

  // Set ROS Rate for while loop
  ros::Rate rate(RATE); 

  while(ros::ok())
    {
     setPoint_pub.publish(sP);
     ros::spinOnce();
     rate.sleep();
    }

  return 0;
}
