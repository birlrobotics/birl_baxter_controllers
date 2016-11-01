#ifndef CONTROLLER_
#define CONTROLLER_

// ROS System
#include <ros/ros.h>

// ROS Message Types
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <force_controller/setPoint.h> // setPoint desired type

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <force_controller/force_error_constantsConfig.h>
//#include "/home/vmrguser/ros/indigo/baxter_ws/devel/include/force_controller/force_error_constantsConfig.h"

// Baxter Message Types
#include <baxter_core_msgs/JointCommand.h>  // To command the joints 
#include <baxter_core_msgs/EndpointState.h> // To read the wrench at the endpoint 
#include <baxter_core_msgs/SEAJointState.h> // To read the gravitation compensation data

// Eigen Libs
#include <Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Core>

// Force Control and Kinematics
#include <force_controller/forceControl.h> // Service
#include <force_controller/kinematics.h>

// Forcing Core Files (http://processors.wiki.ti.com/index.php/Multithreaded_Debugging_Made_Easier_by_Forcing_Core_Dumps)
//include <sys/types.h>
//include <signal.h>
//include <unistd.h>

// STD Libs
#include <iostream>
#include <fstream>
#include <string>
#include <deque>
using std::string; 

//-----------------------------------------------------------------------------------------------------------------------------
// Programs design parameters
//----------------------------------------------------------------------------------------
/*** Flags for ROS Communication Objects ***/
#define JOINTS_SUB_F 1 // Subscribes to /gravity_compensation_torques to get joints, velocities, torques, and gravity compensations
#define WRENCH_SUB_F 1 // Subscribes to /endpoint_state to get the endpoint wrench
#define SETPNT_SUB_F 1 // Subscribes to /side/force_control/setPoint
//----------------------------------------------------------------------------------------
#define JOINTS_PUB_F 1 // Publishes to /joint_command to move the arm to a reference set point.
#define FILT_W_PUB_F 1 // Publishes a filtered wrench value 
//----------------------------------------------------------------------------------------
#define CTRBAS_SRV_F 0 // Publishes the control basis service server. When a client call is sent, force_control begins. 
#define DYN_RECONF_F 0 // Dynamic reconfigure flag
//----------------------------------------------------------------------------------------

/*** Inner Control Loop ***/ 
#define JNTPOS_TORQUE_CONTROLLER 1     // If true, set point is joint angles, otherwise joint torques.

/*** Time  Rates ***/
#define FC_ROS_RATE     500           // These rates (Hz) set control loop pudate cycles. Inner position control loop needs to run faster than the outer loop.
#define POS_ROS_RATE    1000 
#define TIME_OUT        0.025            // Position Controller timeout. Determines how long to run the loop before issuing a finished flag. 
                                       // If too long and robot is in contact with surface, forces will rise dangerously.
                                       // We want to add delta joint angles to the latest joint position. To do this, we act as fast as possible, or else we'll be adding dq's to old joint valus. Also, joint_command works best at a fast rate.

/*** MATH ***/
#define PI 3.141592654
//-----------------------------------------------------------------------------------------------------------------------------

namespace force_controller
{
  // Arm Parameters
  static const int LEFT = 0, RIGHT = 1;

  // Proportional Gain Parameters for joint controller Const: (0.0050)
  double k_fp0=0.2, k_fp1=0.0001, k_fp2=0.1, 
    k_mp0=0.015, k_mp1=0.0015, k_mp2=0.015; // Also sed with dynamc_reconfigure
  

  // Derivative Gain Parameters for joint controller Const: Const: 0.0025
  double dg=0.25;
  double k_fv0=dg, k_fv1=dg, k_fv2=dg, k_mv0=dg, k_mv1=dg, k_mv2=dg;

  bool force_error_constantsFlag = false;

  // These are the parameters that conform a 2nd order least squares error approximation to a function composed by Baxter Joint Angles as the independent variable and Joint Torques as a dependent variable. These parameters are then used to model ax^2 + bx^1 + cx^0 for each of the 7 joints, for both arms giving rise to a 2x7x3 structure.
  static const double COEFF[2][7][3] = {
    // Left Arm
    { {0.04528400,  0.412655, -0.102458},     // S0
      {-13.645011, 15.076427, -2.089347},     // S1
      {9.60591200, 46.063440, 53.876335},     // E0
      {0.17552900, -0.174395, -1.880203},     // E1
      {2.78838700, -3.276785,  1.355042},     // W0
      {-0.2248960,  0.726712, -0.299096},     // W1
      {-0.2951670,  1.183197, -0.832453} },   // W2
    { // Right Arm
      {-0.8811330,  0.447500,  0.108815},     // S0
      {24.7798690,-44.860449, 17.881954},     // S1
      {-4.5111360, 22.012300,-24.573247},     // E0
      { 0.1717400, -0.207249, -0.946106},     // E1
      {-2.5302830, -3.267953, -0.817067},     // W0
      {-0.5545530,  1.847279, -1.338913},     // W1
      { 0.1445150,  0.506907,  0.507274} }	  // W2
  };

  class controller
  {
  public:
   
    // Constructor
    controller(ros::NodeHandle);

    // Destructor
    ~controller() { }

    // Public Flags
    int dynamic_reconfigure_flag;

    // Public Methods
    bool force_controller();                  // **Force Controller. Main method

	  inline bool start() { return init_; }
    inline int get_rosCommunicationCtr() { return rosCommunicationCtr; }
    inline double get_fcLoopRate() {return fc_while_loop_rate_; }
    inline void rosCommunicationCtrUp() { rosCommunicationCtr++; }
    /*** Dynamic Reconfigure Callback ***/
    // void callback(force_error_constants::force_error_constantsConfig &config, uint32_t level); // placed it as global in .cpp

  private:

	  void fillJointNames();
	  Eigen::VectorXd         getTorqueOffset();
	  std::vector<double>     toVector(const geometry_msgs::Vector3& d);
	  sensor_msgs::JointState fill(Eigen::VectorXd dq);

    /*** ROS Updates for subscribers  and Parameters ***/
    void getWrenchEndpoint(const baxter_core_msgs::EndpointStateConstPtr& state);    // Eigen::Vector3d getWrenchEndpoint(....
	  void getBaxterJointState(const baxter_core_msgs::SEAJointStateConstPtr& state);  // Used to get joint positions, velocities, and efforts from Baxter. 
    void getSetPoint(const force_controller::setPointConstPtr& state);               // Used to get the desired set point for the control basis

    void updateGains();                                                              // used to change gFp_ and gMp_ after params updated with rqt_reconfig
	  void updateGains(geometry_msgs::Vector3 gain, std::string type);
    
    /*** Controllers ***/
    bool isMoveFinish(bool& result);                                                  // Used by position control to check if goal has been reached.
    bool position_controller(sensor_msgs::JointState qd, ros::Time);                  // Position Controller
    void torque_controller(Eigen::VectorXd delT, ros::Time t0);                       // Torque controller

    /*** Force Control Support Methods and Null Space Methods ***/
    // double computeError(...) // inline method below.
    Eigen::Vector3d extractWrench_Force_Moment(string type);
	  bool JacobianProduct(std::string type, Eigen::VectorXd& update);
	  bool JacobianErrorProduct(std::string type, Eigen::VectorXd& update); // changed the order of multiplication by gains to see if it improves controller.
	  bool NullSpaceProjection(std::vector<Eigen::VectorXd> updates, sensor_msgs::JointState& dq);
	  bool computePrimitiveController(Eigen::VectorXd& update, std::string type, Eigen::Vector3d setPoint, std::vector<double>& e);



    /*** Inline Methods ***/
	  inline void openFiles()
	  {
      std::ostringstream num2;
      num2 << "s" << m_ << "__" << "Joints_" << side_ << "_sim.txt";
      std::string title2 = num2.str();
      char * name2 = new char [title2.length()+1];
      std::strcpy (name2, title2.c_str());

      save_.open(name2, std::ios::out);
      exe_ = true;
	  }

	  inline void fin()
	  {
      exe_=false;
      if(save_.is_open())
        save_.close();

      joints_sub_.shutdown();
	  }


    //*******************************************************************************************
    // computeError(...)
    // Simply computes the difference between the desired amount and the actual amount.
    // type: force or moment
    // xt is the current data in force/moment
    // xd is the desired data 
    //*******************************************************************************************
	  inline double computeError(std::string type, Eigen::Vector3d xt, Eigen::Vector3d xd)
	  {
      double mag;
      int    offset = 0;
      if(type == "moment") offset = 3;

      // Compute the error between cur and des  data, save in error_ private member
      error_ = Eigen::VectorXd::Zero(6);
      for(unsigned int i=0; i<3; i++)
        error_(i+offset) = (xd(i)-xt(i)); // -1 is to help us descend the gradient error. 
        
      // Compute the derivative error using a finite difference approximation. 
      // TODO: should try the symmetric difference quotient. (error+1-error_1)/2rate
      // TODO: Can filter the derror signal.
      int position_derror_flag=0;
      if(position_derror_flag)
        {
          for(unsigned int i=0; i<3; i++)     
            derror_(i+offset) = (error_(i+offset) - error_t_1(i+offset))*fc_while_loop_rate_;          // Instead of dividing by time, multiply by the rate.
        }
      else
        {
          for(unsigned int i=0; i<3; i++)     
            derror_(i+offset) = (velocity_[0][i+offset] - velocity_[1][i+offset])*fc_while_loop_rate_; // Instead of dividing by time, multiply by the rate.
        }

      if(errorCtr_==1)
        derror_=Eigen::VectorXd::Zero(6);
      
      // Save current error to error_t_1
      error_t_1 = error_;

      // Also compute the norm. Useful to check if  error decreases over time. 
      mag = error_.norm();

      return mag;
	  }

    //*******************************************************************************************
    // Inlined Initialization function
    // Get's class' starting time. 
    // Sets a counter 
    //*******************************************************************************************
	  inline void initialize()
	  {
      to_ = ros::Time::now();
      m_ = 0;
	  }    

    /***************************************************************************** Private Members **********************************************************/
    // Node Handles
	  ros::NodeHandle node_handle_, root_handle_;          // Node handle is used with parameters and root_handle is used with services/publications/subscriptions
                                                         // Designed to work with local and global namespaces. Currently not used consistently. 

    // Publishers, subscribers, and services.
	  ros::Subscriber    joints_sub_;                      // Subscription to get joint angles. 
    ros::Subscriber    wrench_sub_;                      // Used to subscribe to the wrench endpoint. 
    ros::Subscriber    setPoint_sub_;

    ros::Publisher     joint_cmd_pub_;                   // Publication to command joints 
    ros::Publisher     filtered_wrench_pub_;             // Publish a filtered wrench number

	  ros::ServiceServer ctrl_server_;                     // Used to advertise the control basis service. 

    // Publish/Subscribe/Service Flags and Counter. 
    // Given that we are using a multithreaded method, it's good to automatically generate a counter of how many things we publish/subscribe/service.
    int rosCommunicationCtr;

    int joints_sub_flag;
    int wrench_sub_flag;
    int setPoint_sub_flag;

    int joint_cmd_pub_flag;
    int filtered_wrench_pub_flag;

    int ctrl_server_flag;
    // int dynamic_reconfigure_flag;                       // Currently dynamic_reconfigure code sits outside the class in main, so this will be publc.

    // Kinematics model pointers.
	  Kinematics::Ptr kine_model_;
	 
    // Baxter arm strings
	  std::string side_, tip_name_;
	  std::vector<std::string> joints_names_;

    // Force Controller Vars
	  Eigen::Vector3d gFp_, gMp_; // Proportional gains
    Eigen::Vector3d gFv_, gMv_; // Derivative gains
	  Eigen::VectorXd error_, error_t_1, derror_;
    double error_norm_;
    std::vector<Eigen::VectorXd> setPoint_; // Keep [0] for dominant and [1] for subordiante
    force_controller::setPoint sP_;         // Contains des values, gains for up to 2 cntrls.
    Eigen::VectorXd cur_data_, cur_data_f_;
    std::deque<Eigen::VectorXd> wrenchVec, wrenchVecF; 
	  std::vector<double> j_t_1_, jv_t_1_, tm_t_1_, tg_t_1_; // Joints, velocity, torques, gravitational torques. 
	  std::vector<std::vector<double> > joints_, velocity_, torque_, tg_;

    // Position Controller Vars (used with controller::position_controller
    sensor_msgs::JointState update_angles_;
	  baxter_core_msgs::JointCommand qgoal_; // Smooth filtered goal with alpha
	  std::vector<double> goal_, qd_, qe_; // Comes from position_controller/include/initialPose.h. 
                                         // joints_ was removed from here and instead we used the native std::vector<std::vector<double> > joints_ always using index[0] instead.

    // Position Controller Tolerance Parameters
	  double tolerance_, max_error_, alpha_;    

    // Force Controller Tolerance Parameters
    double force_error_threshold_;

    // Status Boolean Flags
	  bool init_, exe_, jo_ready_;

    // Counters
	  int n_, no_, m_, points_, errorCtr_;

    /*** Flags ***/
    // Inner Control Loop Flags
    int jntPos_Torque_InnerCtrl_Flag_;
    // Wrench Filtering Flags
    int wrenchFilteringFlag;  // in getWrenchEndpoint()
    int initialFiltering;

    // File Streams
	  std::ofstream save_;

    // Time
	  ros::Time to_;
    double timeOut_;
    double fc_while_loop_rate_;
    double pos_while_loop_rate_;
  };
}
#endif /* REPLAY_ */
