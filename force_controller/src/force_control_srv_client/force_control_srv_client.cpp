#include "ros/ros.h"

// ROS Service
#include <force_controller/force_controller_service.h>

// ROS Msg Type
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

// STD Libs
#include <cstdlib>
#include <stdlib.h>
#include <string>
//---------------------------------------------------------------------------------------------------
// Can get setPoint from launch file, or command line
// setPoint looks like:
//---------------------------------------------------------------------------------------------------
// #Number of Controllers
// int32 num_ctrls
// 
// # Name of the type(s) of Controller(s): force or moment
// string[] domType
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


bool runClient(int argc, int numCtrls, int mode1, int x1, int y1, int z1, int gx1, int gy1, int gz1, int mode2, int x2, int y2, int z2, int gx2, int gy2, int gz2)
{
  // Local variables 
  bool res=false; 

  // Local ROS Node Creation
  ros::NodeHandle n;

  // Create the service client object which is associated with the service topic
  ros::ServiceClient client = n.serviceClient<force_controller::forceControl>("/right/force_control/setPoint",true); // 2nd arguments indicates whether the service connection should be persistent. 

  /*** Fill our service client data ***/
  // We then need to create a force_controller srv object that has the contents information.
  force_controller::forceControl srv;

  /*** Num of Controllers ***/
  srv.request.num_ctrls=numCtrls;         // TODO, we'll need to update when considering more than one controller

  /*** CONTROL TYPE ***/
  std::string ctrl_type;

  if(numCtrls==1)
    {
      if(mode1 == 0)
        ctrl_type="force";
      else if(mode1 == 1)
        ctrl_type="moment";
      else
        {
          ROS_INFO("Wrong control type given");
          return -1;
        }

      // Set the dominant controller type
      srv.request.domType=ctrl_type;

      /*** Dominant Desired ***/
      geometry_msgs::Vector3 d;
      d.x=x1; 
      d.y=y1; 
      d.z=z1;
      srv.request.domDes.push_back(d);

      /*** Dominant Gains ***/
      if(argc==8)
        {
          d.x=gx1;
          d.y=gy1;
          d.z-gz1;
          srv.request.domGains.push_back(d);
        }

    }

  // 2 Controllers
  else if(numCtrls==2)
   {
     if(mode2 == 0)
       ctrl_type="force";
     else if(mode2 == 2)
       ctrl_type="moment";
     else
       {
         ROS_INFO("Wrong control type given");
         return -1;
       }

     // Set the subordinate controller type
     srv.request.subType=ctrl_type;

     /*** Subordinate Desired ***/
     geometry_msgs::Vector3 d;
     d.x=x2; 
     d.y=y2; 
     d.z=z2;
     srv.request.subDes.push_back(d);

     /*** Subordiante Gains ***/
     if(argc==16)
       {
         d.x=gx2;
         d.y=gy2;
         d.z-gz2;
         srv.request.subGains.push_back(d);
       }
   }
 else
   {
     ROS_ERROR("force_srv_client: Wrong number of controllers");
     return -1;
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
             ROS_INFO_STREAM("The output is: " << srv.response.output << std::endl);
                             // << srv.response.update_angles.position[0] << " " 
                             // << srv.response.update_angles.position[1] << " " 
                             // << srv.response.update_angles.position[2] << " " 
                             // << srv.response.update_angles.position[3] << " " 
                             // << srv.response.update_angles.position[4] << " " 
                             // << srv.response.update_angles.position[5] << " " 
                             // << srv.response.update_angles.position[6] );
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
        // Main ROS node and information
        ros::init(argc,argv,"force_control_srv_client");
        ros::NodeHandle mn;

        /*** Variable Initialization according to user input ***/
        // A. Handle command line arguments
        int numCtrls=1;					// How many controllers
        int mode1=0,  mode2=0;	// mode1: what type of controller will be dominant? "force" or "moment"
        double x1=0,  x2=0;			// Des values for XYZ for dominant and subordinate controlllers
        double y1=0,  y2=0; 
        double z1=0,  z2=0; 
        double gx1=0, gx2=0;		// DesGains for XYZ for dominant and subordiante controllers
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

        // TODO: State Machine.
        // Here we need to insert the logic of the state machine, that would issue different calls for different states. For now just run a loop. 

        /*** Begin the loop ***/
        // Set ROS Rate for while loop
        ros::Rate rate(5);
        bool res=true;

        // While everything is okay, keep calling the service client. 
        //  while( ros::ok() && !ros::isShuttingDown() ) 
        while(res) 
          {              
            res=runClient(argc,numCtrls,mode1,x1,y1,z1,gx1,gy1,gz1,mode2,x2,y2,z2,gx1,gy2,gz2);
            rate.sleep();
          }

        return 0;
      }
