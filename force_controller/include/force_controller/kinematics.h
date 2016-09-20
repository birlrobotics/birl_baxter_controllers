#ifndef FORCE_CONTROLLER_H_
#define FORCE_CONTROLLER_H_

// STD
#include <cstring>

// ROS Headers
#include <ros/ros.h>

// KDL Headers
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>

// URDF Header
#include <urdf/model.h>

// Eigen Headers// 
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Core>

namespace force_controller 
{

  typedef struct INFO {
    std::vector<std::string> link_names;
    std::vector<std::string> joint_names;
  } KinematicSolverInfo;

  class Kinematics 
  {
	public:
	  Kinematics();

    // Kinematics shared pointer Typdef 
    typedef boost::shared_ptr<Kinematics> Ptr;

	  bool init(std::string tip_name, int &no_jts);
    bool readJoints(urdf::Model &robot_model);

    // Jacobian Computations
	  bool getJacobian(/*in*/std::vector<double> joints,     /*in*/std::vector<std::string> names, /*out*/Eigen::MatrixXd& jacobian);
	  bool getJacPseudoInv(/*in*/std::vector<double> joints, /*in*/std::vector<std::string> names, /*out*/Eigen::MatrixXd& jacobian);

    // Locally defined function to create kdl tree
	  static Ptr create(std::string tip_name, int &no_jts)
	  {
      // Create the kinematic pointer
	    Ptr parm_kinematics = Ptr(new Kinematics());

      // Create the chain and return the number of joints. 
	    if (parm_kinematics->init(tip_name, no_jts))
        {
          ROS_INFO_ONCE("All init finished for tip: %s", tip_name.c_str());
          return parm_kinematics;
        }
	    else
	      ROS_ERROR("Couldn't initialize for tip: %s", tip_name.c_str());
	    return Ptr();
	  }

    // Return the appropriate index for a given joint name. 
    inline int getJointIndex(const std::string &name)
	  {
      for (unsigned int i = 0; i < info_.joint_names.size(); i++)
        {
          if (info_.joint_names[i] == name)
            return i;
        }
      return -1;
	  }

  private:
    // Members
	  KDL::Chain          chain_;
	  std::string         root_name_, tip_name_;
	  unsigned int        num_joints_;
	  ros::NodeHandle     nh_, nh_private_;
    KinematicSolverInfo info_;

    // Methods
	  bool loadModel(const std::string xml);

  };

}
#endif
