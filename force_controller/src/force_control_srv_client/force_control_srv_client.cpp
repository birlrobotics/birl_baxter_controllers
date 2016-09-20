#include "ros/ros.h"

// ROS Service
#include <force_controller/controller.h>

// ROS Msg Type
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

// STD Libs
#include <cstdlib>
#include <stdlib.h>
#include <string>

bool runClient(int mode, int argc, char **argv, int x, int y, int z, int gx, int gy, int gz)
{
  // Local variables 
  bool res=false; 

  // Local ROS Node Creation
  ros::NodeHandle n;

  // Create the service client object which is associated with the service topic
  ros::ServiceClient client = n.serviceClient<force_controller::forceControl>("/right/force_controller",false); // what about persistence (2nd arg)??

  /*** Fill our service client data ***/
  // We then need to create a force_controller srv object that has the contents information.
  force_controller::forceControl srv;

  /*** Num of Controllers ***/
  srv.request.num_ctrls=1;         // TODO, we'll need to update when considering more than one controller

  /*** CONTROL TYPE ***/
  std::string ctrl_type;
  if(mode==0)
    ctrl_type="force";
  else if(mode==1)
    ctrl_type="moment";
  else
    {
      ROS_INFO("Wrong control type given");
      return -1;
    }

  std::vector<std::string> ctrl_type_vec;
  ctrl_type_vec.push_back(ctrl_type);
  srv.request.type=ctrl_type_vec;

  /*** Desired ***/
  geometry_msgs::Vector3 d;
  d.x=x; 
  d.y=y; 
  d.z=z;
  srv.request.desired.push_back(d);

  /*** Gains ***/
  if(argc==8)
    {
      d.x=gx;
      d.y=gy;
      d.z-gz;
      srv.request.gains.push_back(d);
    }
   
  // Begin Client Call Routine
  // Make sure service exists
  if(client.waitForExistence(ros::Duration(-1))) 
    {
      // Then check if client is properly made
      if(client.isValid())
        {            
          // Make the client call
          if ( client.call(srv) )
            {
              ROS_INFO_STREAM("The commanded joint angle update is: " 
                              << srv.response.update_angles.position[0] << " " 
                              << srv.response.update_angles.position[1] << " " 
                              << srv.response.update_angles.position[2] << " " 
                              << srv.response.update_angles.position[3] << " " 
                              << srv.response.update_angles.position[4] << " " 
                              << srv.response.update_angles.position[5] << " " 
                              << srv.response.update_angles.position[6] );
              res = true;
            }
          else
            {
              ROS_ERROR("Failed to call service force_controller");
              res = false;
            } 
        }
    }

  return res;
}


int main(int argc, char **argv)
{
  int mode=0;
  double x=0;
  double y=0; 
  double z=0;
  double gx=0;
  double gy=0;
  double gz=0;

  // Main ROS node and information
  ros::init(argc,argv,"forceControl_srvClient");
  ros::NodeHandle mn;

  /*** Variable Initialization according to user input ***/
  // User only entered desired setpoint. 
  if(argc==4)
    {
      x=atof(argv[1]);
      y=atof(argv[2]);
      z=atof(argv[3]);
      mode=0;
    }

  // User entered moded and desired setpoint.
  else if(argc==5)
    {
      mode=atof(argv[1]);
      x=atof(argv[2]);
      y=atof(argv[3]);
      z=atof(argv[4]);
      
    }
  else if(argc==8)
    {
      mode=atof(argv[1]);
      x=atof(argv[2]);
      y=atof(argv[3]);
      z=atof(argv[4]);
      gx=atof(argv[5]);
      gy=atof(argv[6]);
      gz=atof(argv[7]);      
    }
  else
    {
      x=0;
      y=0;
      z=0;
      mode=0;
    }

  /*** Begin the loop ***/
  // Set ROS Rate for while loop
  ros::Rate rate(5);
  bool res=true;

  // While everything is okay, keep calling the service client. 
  //  while( ros::ok() && !ros::isShuttingDown() ) 
  while(res) 
    {              
      res=runClient(mode,argc,argv,x,y,z,gx,gy,gz);
      rate.sleep();
    }

  return 0;
}
