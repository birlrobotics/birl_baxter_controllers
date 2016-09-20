#include <force_controller/controller.h>
namespace force_controller
{
  //***********************************************************************************************************************************************
  // callback(...) for dynamic Reconfigure set as a global function. 
  // When the rqt_reconfigure gui is used and those parameters are changed, the config.param_name in this function will be updated. Then, these parameters need to be set to the private members of your code to record those changes. 
  //***********************************************************************************************************************************************
  void callback(force_error_constants::force_error_constantsConfig &config, uint32_t level)
  {
    // Print the updated values
    ROS_INFO("Reconfigure request: %f %f %f %f %f %f", 
             config.k_fp0,
             config.k_fp1,
             config.k_fp2,
             config.k_mp0,
             config.k_mp1,
             config.k_mp2);
  
  // Save to the corresponding data members. 
  k_fp0=config.k_fp0;
  k_fp1=config.k_fp1;
  k_fp2=config.k_fp2;
  k_mp0=config.k_mp0;
  k_mp1=config.k_mp1;
  k_mp2=config.k_mp2;

  // change the flag
  force_error_constantsFlag = true;

  }

  // Controller Class Constructor
  controller::controller(ros::NodeHandle node): node_handle_(node)
	  {
      // Initialize counters and Flags
      n_ = 0;	m_ = 0;	no_ = 0, errorCtr_=1; error_norm_=0.0;

      // Local 
      int nj;
      double gain;
      // Default parameter Values
      double alpha  = 0.50;           // Orig: 0.987512
      double oneDeg = PI/180;
      int    force_error_threshold=1;  

      /*** Get Parameter Values ***/
      // Get Parameter Values from the parameter server. Set in the roslaunch file or by hand.
      // node_handle_ access node namespace. root_handle_ access global namespace.

      // Position Controller precision and filtering
      node_handle_.param<double>("joint_precision", tolerance_, oneDeg);
      node_handle_.param<double>("filter", alpha_, alpha);  

      // Force Controller Error Tolerance
      node_handle_.param<double>("error_threshold",force_error_threshold_,force_error_threshold);

      // Strings
      node_handle_.param<std::string>("side", side_, "right");
      node_handle_.param<std::string>("tip_name", tip_name_, "right_gripper");

      // Hack: currently we cannot guarantee the order in which spinner threads are called.
      // There are occassions in which getWrenchEndpoint is called before updateJointAngles, in this case, getTorqueOffset is called, which needs joints. A segfault is issued.
      std::vector<double> tmp;
      //for(int i=0; i<7; i++) tmp.push_back(0.0);
      tmp.resize(7);
      tmp[0]=0.07; tmp[1]=-0.9; tmp[2]=1.15; tmp[3]=1.92; tmp[4]=-0.6; tmp[5]=1.01; tmp[6]=0.48;

      // Right Arm
      if(strcmp(side_.c_str(),"right")) joints_.push_back(tmp);
      // Left arm
      else 
        {
          tmp[0]=0.0; tmp[1]=-0.9; tmp[2]=1.1; tmp[3]=1.92; tmp[4]=-0.65; tmp[5]=1.00; tmp[6]=-0.48;
          joints_.push_back(tmp);
        }
          
      // Proportional Gains
      gFp_ << k_fp0, k_fp1, k_fp2; 
      gMp_ << k_mp0, k_mp1, k_mp2; 

      // Derivative Gains
      gFv_ << k_fv0, k_fv1, k_fv2; 
      gMv_ << k_mv0, k_mv1, k_mv2; 

      // Other vectors
      cur_data_   = Eigen::VectorXd::Zero(6);  cur_data_f_ = Eigen::VectorXd::Zero(6);
      error_      = Eigen::VectorXd::Zero(6);  error_t_1   = Eigen::VectorXd::Zero(6);
      derror_     = Eigen::VectorXd::Zero(6);
      // Create two entries into setPoint_
      Eigen::VectorXd t = Eigen::VectorXd::Zero(3);
      setPoint_.push_back(t); 
      setPoint_.push_back(t); 
    
      // State
      exe_ = false;	jo_ready_ = false;

      /***************************************************** Publisher, subscriber and Service Advertisement *******************************************************************/
      // Set all the flag values ros ros communication objects
      rosCommunicationCtr=0;
      joints_sub_flag         =JOINTS_SUB_F; 
      wrench_sub_flag         =WRENCH_SUB_F;
      setPoint_sub_flag       =SETPNT_SUB_F;
      joint_cmd_pub_flag      =JOINTS_PUB_F;
      filtered_wrench_pub_flag=FILT_W_PUB_F;
      ctrl_server_flag        =CTRBAS_SRV_F; 
      dynamic_reconfigure_flag=DYN_RECONF_F; 
      force_error_constantsFlag=false; 

      // 1. Subscription object to get current joint angle positions, velocities, joint torques, and gravitational compensation torques.
      if(joints_sub_flag)
        {
          joints_sub_ = root_handle_.subscribe<baxter_core_msgs::SEAJointState>("/robot/limb/" + side_ + "/gravity_compensation_torques", 1, &controller::getBaxterJointState, this);
          rosCommunicationCtr++;
        }

      // 2. Subscription object to get the wrench endpoint state. 
      if(wrench_sub_flag)
        {
          wrench_sub_ = root_handle_.subscribe<baxter_core_msgs::EndpointState>("/robot/limb/" + side_ + "/endpoint_state", 1, &controller::getWrenchEndpoint, this);
          rosCommunicationCtr++;
        }

      // 3. Subscription object to get the controller setPoint 
      if(setPoint_sub_flag)
        {
          setPoint_sub_ = root_handle_.subscribe<force_controller::setPoint>("/" + side_ + "/force_control/setPoint",1,&controller::getSetPoint,this);
          rosCommunicationCtr++;
        }

      // 4. Publication object to publish commanded joint positions throught the joint_command topic.
      if(joint_cmd_pub_flag)
        {
          joint_cmd_pub_ = node_handle_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + side_ + "/joint_command", 10, false); // May want to keep a small num of points
          ros::Duration(3.0).sleep(); ROS_INFO("Initial Pose initialized for %s arm, tolerance = %f", side_.c_str(), tolerance_);	
          rosCommunicationCtr++;
        }

      // 5. Publication object to publish filtered wrench information.
      if(filtered_wrench_pub_flag)
        {
          filtered_wrench_pub_ = node_handle_.advertise<baxter_core_msgs::EndpointState>("/robot/limb/" + side_ + "/filtered_wrench", 20, false);
          rosCommunicationCtr++;
        }

      // Create the kinematic chain/model through kdl from base to right/left gripper
	    kine_model_ = Kinematics::create(tip_name_, nj);

      // Clear torque and gravitational torque vectors.
      torque_.clear();
      tg_.clear();

      // If we have our 7DoF, then let's set the values of our joint names in joints_names_ as: s0s1e0e1w0w1w2. This is the order of /robot/limb/right/gravity_compensation_torques but not of /robot/limb/right/joint_commad
      if(nj == 7)
        {
          fillJointNames();
          init_ = true;
        }

      // Print successful exit
      ros::Duration(1.0).sleep();

      // Time Rates
      timeOut_            =TIME_OUT;
      fc_while_loop_rate_ =FC_ROS_RATE;
      pos_while_loop_rate_=POS_ROS_RATE;

      /*** Filtering ***/
      // Inner Control Loop
      jntPos_Torque_InnerCtrl_Flag_=JNTPOS_TORQUE_CONTROLLER;

        // Wrench Filtering
      wrenchFilteringFlag=1;
      initialFiltering   =1;

      ROS_INFO("Force controller on baxter's %s arm is ready", side_.c_str());
	  }
  //***********************************************************************************************************************************************
  // fillJointNames()
  // Given the string parameter for the right or left side of the arm, create a vector of joint names with an ordered set of joint angle elements. 
  //***********************************************************************************************************************************************
  void controller::fillJointNames()
  {
    joints_names_.clear();
    joints_names_.push_back(side_ + "_s0");
    joints_names_.push_back(side_ + "_s1");
    joints_names_.push_back(side_ + "_e0");
    joints_names_.push_back(side_ + "_e1");
    joints_names_.push_back(side_ + "_w0");
    joints_names_.push_back(side_ + "_w1");
    joints_names_.push_back(side_ + "_w2");

    // Resize other kinematic model variables that need to be used in computeError
    joints_.clear();
    j_t_1_.clear();
    jv_t_1_.clear();

    j_t_1_.resize(joints_names_.size());
    jv_t_1_.resize(joints_names_.size());
    tm_t_1_.resize(joints_names_.size());
    tg_t_1_.resize(joints_names_.size());

    // Initialize qe_ to a high value on the first iteration: used in position_controller
    qe_.clear();
    for(unsigned int i=0; i<joints_names_.size(); i++)
      qe_.push_back(100.0);
  }

  // Convert a 3D geometry vector and convert it into a 3D std::vector 
  std::vector<double> controller::toVector(const geometry_msgs::Vector3& d)
  {
    std::vector<double> result;
    result.clear();
    result.push_back(d.x);
    result.push_back(d.y);
    result.push_back(d.z);
    return result;
  }

  //*************************************************************************
  // fill(...)
  // Takes a delta joint angle update and add it to the current angles. TODO: make sure you know the order of the joints strings.
  // Current angles are set in getBaxterJointState where filtered values are set to joints_[0]
  //*************************************************************************
  sensor_msgs::JointState controller::fill(Eigen::VectorXd dq)
  {
    // Create a JointState structure. Composed of string[] name and float64[] pos/vel/effort.
    sensor_msgs::JointState update;
    update.name = joints_names_;
    update.position.clear();

    // Add delta joint angle updates to current angles held in joints_[0]. 
    for(unsigned int i=0; i<joints_names_.size(); i++)
      update.position.push_back(dq(i)+joints_[0][i]);

    // Clear other quantities. 
    update.velocity.clear();
    update.effort.clear();

    // Add current time stamp. 
    update.header.stamp = ros::Time::now();

    return update;
  }

  // This function records the torque offset 
  Eigen::VectorXd controller::getTorqueOffset()
  {
    // Create a 7 dimensional vector to hold torque offsets
    Eigen::VectorXd torqueOffset = Eigen::VectorXd::Zero(7);

    // Different offsets for right or left arms. 
    if(side_ == "left")
      {
        // Offset_left_ji = a_ji*x^2 + b_ji*x^1 + c_ji*x^0
        for(unsigned int i=0; i<7; i++)
          torqueOffset(i) = COEFF[LEFT][i][0]*joints_[0][i]*joints_[0][i] + COEFF[LEFT][i][1]*joints_[0][i] + COEFF[LEFT][i][2];
      }
    else if(side_ == "right")
      {
        // Offset_right_ji = a_ji*x^2 + b_ji*x^1 + c_ji*x^0
        for(unsigned int i=0; i<7; i++) 
          torqueOffset(i) = COEFF[RIGHT][i][0]*joints_[0][i]*joints_[0][i] + COEFF[RIGHT][i][1]*joints_[0][i] + COEFF[RIGHT][i][2];
      }
    else
      ROS_ERROR_STREAM_ONCE("The torque offset has not been set appropriately because the wrong arm name has been provided as: " << side_.c_str());

    return torqueOffset; // 7D vector of offsets
  }

  //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  // extractWrench_Force_Moment()
  // Extracts either force or moment from wrench based on the input string. It assumes that a subscriber for the wrench data is being called and that the data is available.
  // Input: string for force or moment.
  // Returns 3D vector.
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
  Eigen::Vector3d controller::extractWrench_Force_Moment(string type)
  {
    Eigen::Vector3d wrench = Eigen::Vector3d::Zero(); 

    // Split values into force or moment
    if(type == "force")
      {
        for(unsigned int i=0; i<3; i++)
          {
            if(wrenchFilteringFlag)
              wrench(i) = cur_data_f_(i);
            else
              wrench(i) = cur_data_(i);
          }
      }
    else if(type == "moment")
      {
        for(unsigned int i=0; i<3; i++)
          {
            if(wrenchFilteringFlag)
              wrench(i) = cur_data_f_(i+3);
            else
              wrench(i) = cur_data_(i+3);
          }
      }
    else
      ROS_ERROR("Type is incorrect");

    return wrench; 
  }

  //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  // 2 possible ways of getting the wrench endpoint:
  // Input: a baxter_core_msgs/EndPointState which contains a wrench with force/torque.
  // 1. Subscribe to the endpoint_state topic and get the wrench. 
  // 2. Get joint torques (and the gravitational torque) and then compute the endpoint wrench. Can us an offset (computed a priori) that tries to cancel the noise in Baxter's arms.  // Places result in private member cur_data_
  // Also available is the choice to filter the data using a 2nd order low-pass filter.
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  void controller::getWrenchEndpoint(const baxter_core_msgs::EndpointStateConstPtr& state)
  {  
    // Local variables. Used in 2. 
    // Eigen::Vector3d result = Eigen::Vector3d::Zero();
    Eigen::VectorXd offset = getTorqueOffset(), realTorque; // wrench; // Create 3 vectors: offset, realTorque, and wrench (force error). 
    Eigen::MatrixXd jacobian, JJt;

    // Assign the correct size for the vectors. 
    realTorque = Eigen::VectorXd::Zero(7);
    // wrench     = Eigen::VectorXd::Zero(6);
    
    // This method should only be called if the joint angles have already been read. Hence:
    // if(jo_ready_)
    //   {

    // Select a method. False if you want a Jacobian computation for torques->wrench. True to get wrench from topic.
    if(wrench_sub_flag) {
      
      // Get the force and torque from the callback argument state
      cur_data_ << state->wrench.force.x, state->wrench.force.y, state->wrench.force.z,
        state->wrench.torque.x,state->wrench.torque.y,state->wrench.torque.z;

      // Offset Removal. TODO: Pending to do any such analysis, but it would go here.
      // 
    }

    // Convert from joint torques to wrench.
    else {
      // Real Torque = sensed torque - gravitational torque - modeled offset torque. 
      for(unsigned int i=0; i<7; i++)
        realTorque(i) = torque_[0][i] - tg_[0][i]; // - offset(i);

      // Print offset and real torques
      ROS_WARN_STREAM("Torque Offset is:\n--------------------\n" << offset     << "\n--------------------\n");
      ROS_WARN_STREAM("Real Torque is:  \n--------------------\n" << realTorque << "\n--------------------\n");

      // Get Jacobian (input: joint angles and joint names, output is matrix). 
      kine_model_->getJacobian(joints_[0], joints_names_, jacobian);

      // Preparing the Pseudo Inverse: 
      JJt = jacobian * jacobian.transpose();

      // Use the Jacobian Pseudo Inverse (Moore-Penrose Inverse) to compute the wrench.
      // Equation: (J'J)^-1*J' or J'*(JJ')^-1
      // Note that in our case T=J'e, so we need to use J' instead of just J in the above equation.
      // Also, the Pseudo Inverse tends to have stability problems around singularities. A large change in joing angles will be produced, even for a small motion.  
      // Singularities represent directions of motion that the manipulator cannot achieve. Particulary in a straight arm configuration, or in the wrist when W1 aligns both W0 and W2. We can check for near singularities if the determinant is close to zero. At singular positions the Pseudo Inverse the matrix is well behaved.
      cur_data_ = (JJt.inverse()*jacobian) * realTorque;
      ROS_INFO_STREAM("\n-------------------------------\nThe current wrench value is:\n-------------------------------\n" << cur_data_ << "\n-------------------------------\n");
      // Use Eigen's least square approach: 
      // wrench = Jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(realTorque);
      for(unsigned int i=0; i<6; i++)
        {
          if(isnan(cur_data_(i)))
            {
              ROS_ERROR_STREAM("Failed to compute Jacobian Pseudo Inverse " << cur_data_(i) << ", tor " << realTorque << ", jac" << jacobian << ", j " << joints_[0][0]);
              // return cur_data_;
            }
        }
    } // End if-else for two methods.   

    /***************************************************************************** Filtering ************************************************************************************/
    // Create a vector or 6D Eigen vectors
    Eigen::VectorXd temp = Eigen::VectorXd::Zero(6);    

    if(wrenchFilteringFlag)
      {
        // If this is the first loop we need to insert two rows with equal values as the current iteration for both wrenchVec and wrenchVecF
        if(initialFiltering)
          {
            for(int i=0; i<2; i++)
              {
                // Input wrench at the end of the vector
                wrenchVec.push_back(cur_data_);               

                // Filtered wrench at the end of the vector
                wrenchVecF.push_back(cur_data_);
              }
            // Change Flag
            initialFiltering=false;
          }

        // Copy new data to wrenchVec at the end of the vector
        wrenchVec.push_back(cur_data_);
        
        // Assuming a vector of size(3:0,1,2) Element [2] is the current one (sitting at the back), element t-1 is [1], sitting in the middle, and element t-2 is[0]. 
        // The indeces are opposite to what you think it should be. 
        for(int i=0; i<6; i++)
          temp(i) = 0.018299*wrenchVec[2](i) + 0.036598*wrenchVec[1](i) + 0.018299*wrenchVec[0](i) + 1.58255*wrenchVecF[1](i) - 0.65574*wrenchVecF[0](i);

        // Add new filtered result
        wrenchVecF.push_back(temp);

        // Pop last value from these two structures to keep the size of the vector to 3 throughout
        wrenchVec.pop_front();
        wrenchVecF.pop_front();
      }

    // Set filterd value to private member
    for(int i=0; i<6; i++)
      cur_data_f_(i) = wrenchVecF[0](i);

    baxter_core_msgs::EndpointState fw;
    // Time Stamp
    fw.header.stamp = ros::Time::now();

    // Wrench
    fw.wrench.force.x  = cur_data_f_(0); 
    fw.wrench.force.y  = cur_data_f_(1); 
    fw.wrench.force.z  = cur_data_f_(2); 
    fw.wrench.torque.x = cur_data_f_(3); 
    fw.wrench.torque.y = cur_data_f_(4); 
    fw.wrench.torque.z = cur_data_f_(5); 

    // Republish. Leading to system crash..
    if(filtered_wrench_pub_flag)
      {
        filtered_wrench_pub_.publish(fw);
        // ROS_INFO_STREAM("Publishing the filtered wrench: " << fw.wrench << std::endl);
      }

    //}
  }

  //*****************************************************************************
  // getSetPoint(...)
  // Reads force_controller::setPoint which is structured as:
  // #Number of Controllers
  // int32 num_ctrls

  // # Type of dominant controller
  // string domType

  // # Desired force/moment (3D) for dominant controller
  // geometry_msgs/Vector3[] domDes

  // # Gains force/moment (3D) for dominant controller
  // geometry_msgs/Vector3[] domGains

  // # Type of subordinate controller
  // string subType

  // # Desired force/moment (3D) for subordinate controller
  // geometry_msgs/Vector3[] subDes

  // # Gains force/moment (3D) for subordinate controller
  // geometry_msgs/Vector3[] subGains
  //*****************************************************************************
  void controller::getSetPoint(const force_controller::setPointConstPtr& sP)
  {
      // Number of Controllers
      sP_.num_ctrls=sP->num_ctrls;

      if(sP_.num_ctrls<0 || sP_.num_ctrls>2)
        {
          ROS_ERROR("The number of controllers seen in getSetPoint() is less than 0 or more than 2. We can only handle 1 or 2.");
        }

      // Dominant Controller data
      sP_.domType =sP->domType;
      sP_.domDes  =sP->domDes;   
      sP_.domGains=sP->domGains; 

      // Subordinate Controller data (if any)
      sP_.subType =sP->subType;
      sP_.subDes  =sP->subDes;
      sP_.subGains=sP->subGains;
  }
  //***************************************************************************************************************************************************
  // getBaxterJointState(...)
  // 
  // This callback function works in hand with the subscription call joints_sub_ using the topic /robot/limb/"side"/gravity_compensation_torques
  // From this topic we obtain current joint angle positions, velocities, actual torques, and gravity_model_effort torques (compensation + hysteresis + crosstalk).
  // The type of the topic is: baxter_core_msgs::SEAJointState. It's strucutre which looks as follows:
  // ---
  //   std_msgs/Header header
  //   uint32 seq
  //   time stamp
  //   string frame_id
  //   string[] name
  //   ---
  //   float64[] commanded_position      
  //   float64[] commanded_velocity
  //   float64[] commanded_acceleration
  //   float64[] commanded_effort
  //   ---
  //   float64[] actual_position         // Current joint angle position in radians
  //   float64[] actual_velocity
  //   float64[] actual_effort           // This includes the inertial feed forward torques when applicable.
  //   float64[] gravity_model_effort    // This is the torque required to hold the arm against gravity returned by KDL if the arm was stationary.  
  //                                   // This does not include inertial feed forward torques (even when we have them) or any of the corrections (i.e. spring
  //   hysteresis, crosstalk, etc) we make to the KDL model.
  //   float64[] gravity_only
  //   float64[] hysteresis_model_effort
  //   float64[] crosstalk_model_effort
  //   float64 hystState
  //***************************************************************************************************************************************************
  void controller::getBaxterJointState(const baxter_core_msgs::SEAJointStateConstPtr& state)
  {
    // Local variables
    ros::Time t = ros::Time::now();
    unsigned int i, k=0;

    // Create Joint, Torque, and Gravitation values and filtered values. 
    std::vector<double> jt, jtf;    // Joints and Filtered Joints
    std::vector<double> jv, jvf;    // Joint Velocities and Filtered Joint Velocities
    std::vector<double> tt, tg;     // Torques and Filtered Torques
    std::vector<double> ttf, tgf;   // Gravitational Torques and Filtered equivalents.

    // Joint Angles
    jt.clear();		jtf.clear();
    jt.resize(joints_names_.size());
    jtf.resize(joints_names_.size());

    // Joint Velocities
    jv.clear();   jvf.clear();
    jv.resize(joints_names_.size());
    jvf.resize(joints_names_.size());

    // Regular Torques
    tt.resize(joints_names_.size());
    ttf.resize(joints_names_.size());

    // Gravitational Torques
    tg.resize(joints_names_.size());
    tgf.resize(joints_names_.size());

    // Update the current joint values, actual effort, and gravity_model_effort values through the topic data.
    ROS_INFO_ONCE("Initializing joints");
    if(exe_ && save_.is_open())
      save_ << (t - to_).toSec() << "	"; // Convert time to seconds

    // For all of our 7 Joints
    while(k < joints_names_.size())
      {
        // And again for the 7 joints
        for(i=0; i<state->name.size(); i++)
          { // Joints names order is: s0s1e0e1w0w1w2. The name list from the topic may not always be in this order. This is necessary to make sure we put the right value in the right place.            
            if(state->name[i] == joints_names_[k])
              {
                // Get current joint angles
                jt[k] = state->actual_position[i];

                // Get current joint Velocities
                jv[k] = state->actual_velocity[i];

                // Get current joint torques
                tt[k] = state->actual_effort[i];

                // Get currend end-point gravitational torque 
                tg[k] = state->gravity_model_effort[i];

                // Save joint angle information
                if(exe_ && save_.is_open())		  
                  save_ << state->actual_position[i] << "	";

                k = k + 1;
                if(k == joints_names_.size())
                  break;
              }
          }
      }

    // Make sure previous values are set before performing IIR filtering. 
    if(!jo_ready_)
      {
        // Save current values into filtered parameters to start
        for(unsigned int i=0; i<joints_names_.size(); i++)
          {
            ttf[i] = tt[i];
            tgf[i] = tg[i];

            // Create previous (unfiltered) values
            tm_t_1_[i] = tt[i];
            tg_t_1_[i] = tg[i];
            j_t_1_[i]  = jt[i];
            jv_t_1_[i] = jv[i];
          }

        // Push 2x!! (to create a 2nd order system) to private members before computing (lowpass) IIR Filtering
        torque_.push_back(tm_t_1_);
        torque_.push_back(tm_t_1_);
        
        tg_.push_back(tg_t_1_);
        tg_.push_back(tg_t_1_);
        
        joints_.push_back(jt);
        joints_.push_back(jt);

        velocity_.push_back(jv);
        velocity_.push_back(jv);
        
        jo_ready_ = true;
      }

    // IIR Low Pass Filter: create the filtered values.
    for(unsigned int i=0; i<7; i++)
      {
        jtf[i] = 0.0784*j_t_1_[i]  + 1.5622*joints_[0][i]   - 0.6413*joints_[1][i];
        jvf[i] = 0.0784*jv_t_1_[i] + 1.5622*velocity_[0][i] - 0.6413*velocity_[1][i];
        ttf[i] = 0.0784*tm_t_1_[i] + 1.5622*torque_[0][i]   - 0.6413*torque_[1][i];
        tgf[i] = 0.0784*tg_t_1_[i] + 1.5622*tg_[0][i]       - 0.6413*tg_[1][i];
        
        j_t_1_[i]  = jt[i];
        jv_t_1_[i] = jv[i];
        tm_t_1_[i] = tt[i];
        tg_t_1_[i] = tg[i];

        // Save filtered value to private members
        joints_[1][i]   = joints_[0][i];         // Make index 1, older element
        joints_[0][i]   = jtf[i];                // Save filtered val to head of vec

        velocity_[1][i] = velocity_[0][i];
        velocity_[0][i] = jvf[i];        

        torque_[1][i] = torque_[0][i];
        torque_[0][i] = ttf[i];

        tg_[1][i]     = tg_[0][i];
        tg_[0][i]     = tgf[i];
      }

    // Save torque and gravitation torque info to file.
    if(exe_ && save_.is_open())
      {
        for(unsigned int j=0; j<joints_.size(); j++)
          save_ << tt[j] << "	";
        for(unsigned int j=0; j<joints_.size(); j++)
          save_ << tg[j] << "	";
        save_ << std::endl;
      }

    ROS_INFO_STREAM_ONCE("Joints updated for: " << ttf[0] << ", "<< ttf[1] << ", "<< ttf[2]);
  }

  //****************************************************************************************************
  // updateGains(...)
  // Used to update gains at run-time through the parameter server or through rqt_reconfigure.
  //****************************************************************************************************  
  void controller::updateGains(geometry_msgs::Vector3 gain, std::string type)
  {


    if(type == "force")
      gFp_ = Eigen::Vector3d(gain.x, gain.y, gain.z);
    else if(type == "moment")
      gMp_ = Eigen::Vector3d(gain.x, gain.y, gain.z);
    else
      ROS_WARN("Could not recognize type of controller, using default gain value");

  }

  void controller::updateGains() {

    // Update with rqt_reconfigure updated parameters
    gFp_ << k_fp0, k_fp1, k_fp2;
    gMp_ << k_mp0, k_mp1, k_mp2;

    // change the flag
    force_error_constantsFlag = false;

  }

/*********************************************** JacobianProduct ***********************************************************
 ** The Jacobian product can be computed using the pseudoinverse J#, or the Jacobian Transpose Jt in the case of position control
 ** and transpose in case of force/moment.
 ** The transpose is an approximation that can work if scaled appropriately.
 ** The latter is more stable that the pseudoinverse approach which struggles near singularities.
 **
 ** For a 6x7 non-square matrix:
 ** For position control:
 ** del_x = Jacobian * del_q
 ** dq = J# * del_x
 **
 ** For force control:
 ** dq=Jt * k* del_f 
 **
 ** Note: Matrices use column-major indexing.
 *************************************************************************************************************************************/
  bool controller::JacobianProduct(/*in*/ std::string type, /*out*/Eigen::VectorXd& update)
  {
    // Initializing a 6x7 jacobian and a 6D joint vector
    Eigen::MatrixXd jacobian;                               // Eigen defaults to storing the entry in column-major.
    Eigen::VectorXd ke = Eigen::VectorXd::Zero(6), dqbis;

    // 1. Get the 6x7 Jacobian for Baxter. 
    kine_model_->getJacobian(joints_[0], joints_names_, jacobian);

    // 2. Compute error x gain 
    if(type == "force")
      {
        for(unsigned int i=0; i<3; i++)
          ke(i) = gFp_(i)*error_(i) + gFv_(i)*derror_(i);
      }
    else if(type == "moment")
      {
        for(unsigned int i=0; i<3; i++)
          ke(i+3) = gMp_(i)*error_(i+3);
      }
    else
      {
        ROS_ERROR("Could not recognize type of controller");
        return false;
      }

    // 3. Compute the delta angles=Jt*(k*wrench_error). 
    dqbis = jacobian.transpose() * ke;

    // Copy to output form
    update = dqbis;

    // Printout the delta joint angle update
    ROS_INFO_STREAM("\n-------------------------------\nDelta Joint Angle Update qs:\n-------------------------------\n" << update << "\n-------------------------------\n");
    return true;
  }


  //*******************************************************************************************************
  // Same as above, but I want to multiply by the gains after the jacobian product to see if this makes any difference. So far in my testing, arm just moves to become straight. Does not seem to work. 
  //*******************************************************************************************************
  bool controller::JacobianErrorProduct(/*in*/ std::string type, /*out*/Eigen::VectorXd& update)
  {
    // Initializing a 6x7 jacobian and a 6D joint vector
    Eigen::MatrixXd jacobian;                               // Eigen defaults to storing the entry in column-major.
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd dq_scaled = Eigen::VectorXd::Zero(7);

    // 1. Get the 6x7 Jacobian for Baxter. 
    kine_model_->getJacobian(joints_[0], joints_names_, jacobian);

    // 2. Compute the delta angles=Jt*(k*wrench_error). 
    dq = jacobian.transpose() * error_;

    // 3. Compute error x product 
    if(type == "force")
      {
        // for(unsigned int i=0; i<3; i++)
        dq_scaled = gFp_(0)*dq;// gain * outoput of product (7x1)
      }
    else if(type == "moment")
      {
        for(unsigned int i=0; i<3; i++)
          dq_scaled(i+3) = gMp_(0)*dq(i+3);
      }
    else
      {
        ROS_ERROR("Could not recognize type of controller");
        return false;
      }
    
    // Copy to output form
    update = dq_scaled;

    // Printout the delta joint angle update
    ROS_INFO_STREAM("\n-------------------------------\nDelta Joint Angle Update qs:\n-------------------------------\n" << update << "\n-------------------------------\n");
    return true;
  
}
  //************************************************************************************************************************************************************
  // computePrimitiveController(...)
  // Output: places commanded joint angles in update. 
  //************************************************************************************************************************************************************
  bool controller::computePrimitiveController(Eigen::VectorXd& update, std::string type, Eigen::Vector3d setPoint, std::vector<double>& e)
  {
    // Init: Set vectors for actual and goal data
    Eigen::Vector3d curdata;
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(joints_[0].size());

    // 1. Current data is obtained from getWrenchEndpoint
    curdata = extractWrench_Force_Moment(type);

    // 2. Error between actual and desired wrench is computed, placed in error_
    error_norm_ = computeError(type, curdata, setPoint);
    e.push_back(error_norm_);  // Keep track of norm. Helps to identify direction of controller
    ROS_ERROR_STREAM("Error Norm: " << error_norm_);
	
    // 3. Compute the product of the error and the jacobian to produce the delta joint angle update placed in dq.
    if(!JacobianProduct(type, dq)) 
      {
        ROS_ERROR("Could not get Jacobian");
        return false;
      }

    // 4. Save the result as the first vector element of update.
    // update.push_back(dq);
    update=dq;
    return true;
  }

/*******************************************************************************************************
 ** NullSpaceProjection()
 ** This function projects the output of the subordinate
 ** controller to the left null space of the dominant controller.
 ** It is important to note that the dominant output is the update
 ** produced by that controller but is not the updated position.
 ** Ie the update might be 0.4 degrees and the position might be 90.4 degrees.
 **
 ** We multiply AngleUpdate2 * null_space_matrix and
 ** place the result in NullSpaceProjMat
 **
 ** This controller consider a 6D/7D space, given that the input
 ** and output vectors will always be the change in joint
 ** coordinates, AngleUpdate2(q1,q2,q3,q4,q5,q6,q7).
 **
 ** From Platt's work, the null space operator, N, is defined as the moore-penrose generalized pseudoinverse:
 **
 ** N = I - [x_out * inv(x_out' * x_out) * x_out']: that is, the identity - *outer product / inner product).
 ** From linear algebra the fraction of outer product/inner product is a projection unto the null space of the column space.
 ** When we subtract the identity from it, we are projecting on the perpendicular space, the left null space.
 **
 ** N = I - [ 1/(q1^2 + q2^2 + q3^2 + q4^2 + q5^2 + q6) * |q1^2 q1q2 q13 ... q1q6 |
 **														|q2q1 q2^2 ...     q2q6 |
 **														|q6q1  ... ...      q6^2|
 **
 ** Notice that the identity matrix is subtracted from
 ** the outer product normalized by the dot product of x_out.
 **
 ** Variables:
 **
 ** SPECIAL CASE: there may be a time where the dominant controller is all zeros
 ** If this is the case, we need to code the value of denominator to 1, otherwise
 ** there will be a division by zero.
 **
 ** Input:  updates is a vector with two eigen vector entries: [0] for the dominant controller and [1] for the subordiante controller.
 ** Output: a sensor_msgs::JointState structure with new angle update+current angles
 ********************************************************************************************************/
  bool controller::NullSpaceProjection(std::vector<Eigen::VectorXd> updates, sensor_msgs::JointState& dq)
  {
    Eigen::MatrixXd outer; outer.resize( joints_[0].size(),joints_[0].size() );
    Eigen::VectorXd dq2 = Eigen::VectorXd::Zero(joints_[0].size());
    Eigen::VectorXd dq1 = Eigen::VectorXd::Zero(joints_[0].size());

    // 1. Compute inner product
    double inner = updates[0].dot(updates[0]);
    if(inner==0)
      inner = 1.0;

    // 2. Compute outer product
    outer = (updates[0] * updates[0].transpose()) / inner;

    // Project output of subordinate controller on null space projection of dom ctrl
    dq2 = (Eigen::MatrixXd::Identity(7, 7) - outer) * updates[1];

    // Add this project to current dominant controller output
    for(unsigned int i =0; i<joints_[0].size(); i++)
      {
        dq1(i) = updates[0](i) + dq2(i);
        if( isnan(dq1(i)) )
          {
            ROS_ERROR("NAN value at joint: %s", joints_names_[i].c_str());
            return false;
          }
      }

    // Add this joint angle update to current robot joint angles using fill and place in sensor_msgs::JointState
    dq = fill(dq1);

    return true;
  }

  // ---------------------------------------------------------------------------
  // force_controller
  // computes end-effector force/moment errors for 1 or 2 controllers using the Jacobian Transpose. If 2 controllers the output of the 2nd controller is projected unto the nullspace of the 1st controller. The joint angle update is added to the current angles and then passed to a position controller. The latter in turn has a smoothing filter that depends on variable alpha that can make the position response move closer to goal or more slowly. We check to see if the move has reached its goal. If not we continue to publish to JointCommand to get closer as long as a timeout is not reached. This timeout is short ~ 1.5 second to seek reactivity.
  // ---------------------------------------------------------------------------
  bool controller::force_controller()
  {
    ROS_INFO("\n----------------------------Entering force controller----------------------------");

    // // Local variables
    bool fin= true;
    bool ok = false;
    std::vector<double> error, js;
    std::vector<Eigen::VectorXd> dqs;

    // Set initial variables like time t0_=0.0
    // side_ = qd.header.frame_id;
    initialize();

    // Initialize vectors
    js.resize(7);                    // Will keep current 7 joint angles here
    dqs.resize(2);                   // To delta angle updates: dom and sub ctrlrs
    dqs[0]=Eigen::VectorXd::Zero(7); // TODO might want to change dqs to simply be an eigen vector. change the prototype of primitiveController and NullSpaceProjection. Is there a need for the vector and the history?
    error.clear();

    // openFiles to write down data
    openFiles();
 
    // A1. Check for gain updates:
    // From command line or roslaunch
    if(sP_.domGains.size()!=0)
        updateGains(sP_.domGains[0],sP_.domType);
    if(sP_.subGains.size()!=0)
      updateGains(sP_.subGains[0], sP_.subType);

    // Update from Dynamic Reconfigure GUI
    if(force_error_constantsFlag)
      updateGains();

    // Wait for the first joint angles readings. Otherwise cannot compute torques.
    while(!ok)
      {
        if(jo_ready_)
          ok = true;
      }

    // B. Call controllers. 
    // As long as our norm is greater than the force error threshold, continue the computation.
    string type="";
    //    do {
      // B1. Call primitive controllers. Store the delta joint angle update in dqs. 
      for(unsigned int i=0; i<sP_.num_ctrls; i++)
        {
          ROS_INFO("Calling primitive controller %d", i);

          // Extract the force/moment setpoint
          if(i==0) 
            {
              type=sP_.domType;
              setPoint_[i] << sP_.domDes[0].x, sP_.domDes[0].y, sP_.domDes[0].z;
            }
          else
            {
              type=sP_.subType;
              setPoint_[i] << sP_.subDes[0].x, sP_.subDes[0].y, sP_.subDes[0].z;
            }


          // Takes vector of eigen's: setPoint_ as an input
          // Outputs a delta joint angle in vector of eigens dqs[i] and the error. 
          // [0]=dominant controller, [1] = subordinate controller
          if(!computePrimitiveController(dqs[i], type, setPoint_[i], error))
            {
              ROS_ERROR("Could not compute angle update for type: %s", type.c_str());
              return false;
            }
        }

      // B2. 2 Controllers: call compound controllers
      if(sP_.num_ctrls > 1)
        {
          // Take 2 angle updates in dqs. Project the subordinate update to the dominant ctrl's nullspace. 
          // Places new angle update+current angles in the sensor_msgs::JointState update_angles_, ready to pass to position controller
          if(!NullSpaceProjection(dqs, update_angles_))
            {
              ROS_ERROR("Could not get null space projection");
              return false;
            }
        }
    
      // For only one (dominant controller) Add delta joint angles to current joint angles through fill, store in update_angles.position
      else
        update_angles_ = fill(dqs[0]);         

      // C. Move to desired joint angle position through a positon or torque control loop
      if(jntPos_Torque_InnerCtrl_Flag_)           
        fin=position_controller(update_angles_,to_); // Position Controller
        
      else 
        torque_controller(dqs[0],to_);              // Torque Controller        

      // If position controller did not finish properly, exit, else continue the while loop.
      // if(!fin)
      //   break;

      // // Set frequency to 
      // loopRate.sleep();
      // }  while(fin && error_norm_ > force_error_threshold_); // do while
    ROS_INFO_STREAM("isMoveFinish returns: " << fin );

    return true;	
  }

  //********************************************************************************************
  // torque_controller(...)
  // Compute error in wrench point, and then use the Jt and a gain to compute torque updates.
  // Sent to /joint_command
  //********************************************************************************************
  void controller::torque_controller(Eigen::VectorXd delT, ros::Time t0)
    {
      // Copy qd into goal_ and clear member qd_ 
      goal_.clear();	qd_.clear();                     // goal_ is of type vector<doubles> 
      for(unsigned int i=0; i<7; i++)
        goal_.push_back(delT[i]);
      qd_ = goal_;

      qgoal_.mode = qgoal_.TORQUE_MODE;               // qgoal is of type baxter_core_msgs/JointCommand. Consists of int mode, float[] name, float[] command.
      qgoal_.names = joints_names_;                   
      qgoal_.command.resize( goal_.size() );

      // Compute commanded torque: actual_torque - gravity_model_effort + delta torque.
      // Notice that the first two are vectors, so we take the top vector elements.
      for(unsigned int i=0; i< goal_.size(); i++)
        qgoal_.command[i] = torque_[0][i]-tg_[0][i]+delT[i]; // Actual torque - gravity compensation (will be added) + delta 
        // qgoal_.command[i] = torque_[0][i]+delT[i]; // Trying with gravity compensation suppresed resulted in a fast and dangerous motion dropping the arm.

      // Get current time 
      ros::Time tnow = ros::Time::now();
  
      // Publish desired filtered joint angles (arm moves)
      ROS_INFO("Commanded torque is:\
              \n------------------------------\n<%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f> at time: %f\n------------------------------\n",
               qgoal_.command[0],qgoal_.command[1],qgoal_.command[2],
               qgoal_.command[3],qgoal_.command[4],qgoal_.command[5],
               qgoal_.command[6],(tnow-t0).toSec());

      // Publish to the topic /robot/limb/right/joint_command. Baxter will move upon receiving this command. 
      joint_cmd_pub_.publish(qgoal_);
      ROS_INFO("\n--------------------------------------------- Finished Moving the %s arm ---------------------------------------------\n\n", side_.c_str());      
    }

  //********************************************************************************************************************************************************************
  // position_controller()
  // Input: JointState qd is the refernce angles where we want to go. t0 is the initial time of the entire program in the ordered set: {s0,s1,e0,e1,w0,w1,w2}
  // Try to move to the desired joint angles. This function originally devised in the position_control package. 
  // Input: 
  // - update_angles.position with desired updated positions 
  //********************************************************************************************************************************************************************
  bool controller::position_controller(sensor_msgs::JointState qd, ros::Time t0)
  {
    // Copy qd into goal_ and clear member qd_ 
    goal_.clear();	qd_.clear();                     // goal_ is of type vector<doubles> 
    for(unsigned int i=0; i<qd.position.size(); i++)
      goal_.push_back(qd.position[i]);               // push back the reference angles. qd is JointStates and has a position[]
    qd_ = goal_;

    // Create a new variable qgoal in which we filter the joing angle between the current position joints_ and the goal position goal_. If alpha is 0, we send the goal directly, if alpha is 1, we stay in our current position. This filter has the effect of speeding up or slowing down the motion of the robot. 
    qgoal_.mode = qgoal_.POSITION_MODE;              // qgoal is of type baxter_core_msgs/JointCommand. Consists of int mode, float[] name, float[] command.
    qgoal_.names = joints_names_;                    // Make sure that the order of these names is {s0,s1,e0,e1,w0,w1,w2}
    qgoal_.command.resize( goal_.size() );

    for(unsigned int i=0; i< goal_.size(); i++)
      qgoal_.command[i] = ((1-alpha_)*goal_[i]) + (alpha_ * joints_[0][i]);

    // Get current time 
    ros::Time tnow = ros::Time::now();

    // Internal Counter. Gets updated in while loop inside isMoveFinish()
    n_ = 0;

    // Publish desired filtered joint angles (arm moves). The time is the diff from now to the beginning of the demo.
    ROS_INFO("Commanded Joint Angle:\
              \n------------------------------\n<%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f> at time: %f\n------------------------------",
             qgoal_.command[0],qgoal_.command[1],qgoal_.command[2],qgoal_.command[3],
             qgoal_.command[4],qgoal_.command[5],qgoal_.command[6],(tnow-t0).toSec());

    // Publish initial joint command (/robot/limb/right/joint_command).
    // Check if goal is reached. 
    joint_cmd_pub_.publish(qgoal_);
    bool cont, isFinished = isMoveFinish(cont);
    
    // Set the position loop rate
    ros::Rate pos_rate(pos_while_loop_rate_);

    // If not finished, keep looping until finished. 
    while(!isFinished && node_handle_.ok())
      {
        // Smoothing Filter: change goal to move more slowly or quickly
        for(unsigned int i=0; i<goal_.size(); i++)
          qgoal_.command[i] = ((1-alpha_)*goal_[i]) + (alpha_ * qgoal_.command[i]);	
     
        // Print commanded joint angles again and their current time 
        ros::Time tnow = ros::Time::now();
        ROS_INFO_ONCE("Commanded Joint Angle:\
              \n------------------------------\n<%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f> at time: %f\n------------------------------",
             qgoal_.command[0],qgoal_.command[1],qgoal_.command[2],
             qgoal_.command[3],qgoal_.command[4],qgoal_.command[5],
             qgoal_.command[6],(tnow-t0).toSec());
        
        joint_cmd_pub_.publish(qgoal_);
        isFinished = isMoveFinish(cont);
        
        // Control the Position Loop Rate
        pos_rate.sleep();
      }

    // Print error to screen
    ROS_WARN("The current joint position error is: %f, %f, %f, %f, %f, %f, %f", 
             qe_[0],qe_[1],qe_[2],qe_[3],qe_[4],qe_[5],qe_[6]);

    ROS_INFO("\n---------------------------------------------\nFinished Moving the %s arm\n---------------------------------------------\n", side_.c_str());
    return isFinished;
  }

  //*************************************************************************************************************************
  // isMoveFinish(...)
  // Input: boolean output to determine if we should CONTINUE this function. 
  // Output: boolean output to determine if we have reached our goal or FINISHED.
  // 
  // Check to see if the arm has finished moving to desired joint angle position. This function is called at a given rate according to ros duration. 
  // Uses qgoal_ which is originally set in the force_controller. It's of type baxter_core_msgs/JointCommand and contains a mode, command[], and names[].
  //*************************************************************************************************************************
  bool controller::isMoveFinish(bool& cont)
  {  
    double max=0.0;
    std::vector<double> error; error.clear();
    
    // Start counter: used to measure how many times we attempt to move before reaching the goal.
    n_ = n_ + 1;

    // For the first 5 tries, return false. Helps to avoid early computation errors.
    // if( n_!=0  &&  (n_ % 5 != 0) )  
    //   {
    //     cont = false;
    //     return false;
    //   }

    // Record maximum error across all joints.
    for(unsigned int i=0; i<joints_[0].size(); i++)
      {
        // Joint error between current position and our qgoal_ var (filtered goal var)
        error.push_back( joints_[0][i] - qd_[i] );

        // Update position error threshold
        if ( fabs(error.back()) > max)
          max = error.back();
      }

    // Copy the reference qgoal_ to a local varaible q[i]. 
    // Test if error is going down for each joint.
    int ok=0, keep=0, k=0; // keep: means joints have not reached goal, ie keep trying.
    std::vector<double> q;
    q.clear();	q.resize(joints_names_.size()); 
    bool copy;

    // Mechanism to either copy current joint angles or goal.
    for(unsigned int i=0; i < joints_names_.size(); i++)
      {
        copy = true;
        // Copy goal
        if( k < qgoal_.names.size() )
          {
            // Only do the following when both names match. Ensures right order.
            if(qgoal_.names[k] == joints_names_[i]) 
              {
                q[i] = qgoal_.command[k]; 
                k = k + 1;
                copy = false;
              }
          }
        
        // Copy current joint angles only if goal info has been copied
        if(copy)
          q[i] = joints_[0][i];
      }	

    // Completely clear our reference goal joint names and command angles.
    qgoal_.names.clear();
    qgoal_.command.clear();

    // Have a new local variable goal be readied.
    goal_.clear();
    
    // If error > tolerance (test for each joint), set qgoal again. 
    for(unsigned int i=0; i<joints_[0].size(); i++)
      {	       
        // If Joint Position Error is greater than tolearnce, do again. 
        if( fabs(error[i]) > tolerance_)         // Tolerance comes from precision parameter (launch file/constructor value).
          {
            qgoal_.names.push_back( joints_names_[i] );
            qgoal_.command.push_back(q[i]);      // Adjusted in previous for/if loop
            goal_.push_back(qd_[i]);             // Orig reference angles

            // Have we reached a termination condition? Still have errors?
            if( fabs( error[i] - qe_[i] ) != 0.0 )  
              keep = keep + 1; // If keep reaches > 0, some joints did not reach goal
          }
        else
          ok = ok + 1;
      }
    qe_ = error; 

    /***** Cases ****/
    // Timeout=all robot joints do not converge to desired position.Measured from the time the force_controller starts until now. 
    ros::Time tnow = ros::Time::now();
    int maxCycles=timeOut_*fc_while_loop_rate_;

    // 1. Info printing case. Before timeout and still working towards the goal.
    if( (keep + ok == joints_[0].size()) && (ok != joints_names_.size()) && ( ((tnow - to_).toSec()) <= timeOut_ ))
      {
        ROS_INFO("Position Controller: At time: %f, keep = %d, ok = %d", (tnow - to_).toSec(), keep, ok);
        if(n_ % maxCycles == 0) ROS_WARN("Position Controller: Warning!! Max number of cycles reached in isMoveFinish().");

        cont = true;   // Continue the loop
        return false;  // We have not yet reached the goal
      }

    // 2. Timeout: either reach the best possible solution or obstructed by something. 
    //    else if( (keep <= 2) || ( ((tnow - to_).toSec()) > timeOut_ ) )
    else if( ((tnow - to_).toSec()) > timeOut_ ) 
      {
        ROS_ERROR("Position Controller: Timed Out!! At time: %f, and number of cycles: %d", (tnow - to_).toSec(), n_);
        cont = false;   // Could not get a result.        
        return true;    // Return a finish command
      }

    // 3. Success: all 7 joints are under tolerance.
    else if(ok == joints_names_.size())
      {
        ROS_INFO("Position Controller: Success!! All joint position errors are under tolerance!");
        cont = false;   // No need to continue
        return true;    // Return a finish command
      }

    // if(n_ % 200 == 0)
    //   ROS_WARN("F: time: %f, keep = %d, ok = %d", (tnow-to_).toSec(), keep, ok);	
    // cont = true;
    // return false;
  }
}  //namespace controller


int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_basis");

  // Create a node namespace. Ie for service/publication: <node_name>/topic_name or for parameters: <name_name>/param_name 
  ros::NodeHandle node("~"); 

  // Instantiate the controller
  force_controller::controller myControl(node);

  if(myControl.dynamic_reconfigure_flag)
    {
      // (i) Set up the dynamic reconfigure server
      dynamic_reconfigure::Server<force_error_constants::force_error_constantsConfig> srv;

      // (ii) Create a callback object of type force_error_constantsConfig
      dynamic_reconfigure::Server<force_error_constants::force_error_constantsConfig>::CallbackType f;

      // (iii) Bind that object to the actual callback function
      f=boost::bind(&force_controller::callback, _1, _2); // Used to pass two params to a callback.

      // Note: how to bind a private member function: If we cant callback to be a member method of the class controller, then we would need to use something like... 
      // boost::function<void (force_error_constants::force_error_constantsConfig &,int)> f2( boost::bind( &myclass::fun2, this, _1, _2 ) );
      // Still not clear on the syntax. Since it's not a member method, we make it a global and also need to use global parameters

      // (iv) Set the callback to the service server. 
      srv.setCallback(f);

      // Update the rosCommunicationCtr
      myControl.rosCommunicationCtrUp();
    }

  if(!myControl.start())
    {
      ROS_ERROR("Could not start controller, exiting");
      ros::shutdown();
      return 1;
    }
  ros::Rate rate( myControl.get_fcLoopRate() );

  /*** Different Communication Modes ***/
  while(ros::ok())
    {
      // 1. nonBlocking spinOnce
      ros::spinOnce();
      myControl.force_controller();
      rate.sleep();
    }
  
  // 2. AsyncSpinner - nonBlocking
  //ros::AsyncSpinner spinner(myControl.get_rosCommunicationCtr());
  //spinner.start();
  //ros::waitForShutdown(); 

  // 3. MultiThreadedSpinner - nonBlocking
  // ros::MultiThreadedSpinner spinner(myControl.get_rosCommunicationCtr()); // One spinner per ROS communication object: here we use it for 
                                                          // 1. Publish joint commands
                                                          // 2. Subsribe to current joint Angles
                                                          // 3. Advertice a service server
                                                          // 4. Subscribe to endpoint wrench (optional set in getWrenchEndpoint)
                                                          // 5. Dynamic Reconfigure (off)
                                                          // 6. published filtered wrench
  //spinner.spin();
  //ros::waitForShutdown(); 

  // 4. Blocking Spin
  // ros::spin();
  // ros::waitForShutdown();

  return 0;
}  
