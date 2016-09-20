#include <cstring>
#include <ros/ros.h>
#include <force_controller/kinematics.h>

namespace force_controller 
{

  Kinematics::Kinematics(): nh_private_("~") 
  {
  }

  bool Kinematics::init(std::string tip, int &no_jts)
  {
    // Get URDF XML
    std::string urdf_xml, full_urdf_xml;
    tip_name_ = tip;
    nh_.param("urdf_xml", urdf_xml, std::string("robot_description"));
    nh_.searchParam(urdf_xml, full_urdf_xml);
    ROS_DEBUG("Reading xml file from parameter server");
    std::string result;
    if (!nh_.getParam(full_urdf_xml, result)) 
      {
        ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return false;
      }

    if (!nh_.getParam("root_name", root_name_)) 
      {
        ROS_FATAL("GenericIK: No root name found on parameter server");
        return false;
      }

    if (!loadModel(result)) 
      {
        ROS_FATAL("Could not load models!");
        return false;
      }

    no_jts=num_joints_;
    return true;
  }


  bool Kinematics::loadModel(const std::string xml)
  {
    urdf::Model robot_model;
    KDL::Tree tree;
    if (!robot_model.initString(xml)) 
      {
        ROS_FATAL("Could not initialize robot model");
        return -1;
      }
    if (!kdl_parser::treeFromString(xml, tree)) 
      {
        ROS_ERROR("Could not initialize tree object");
        return false;
      }
    if (!tree.getChain(root_name_, tip_name_, chain_)) 
      {
        ROS_ERROR("Could not initialize chain object for root_name %s and tip_name %s",root_name_.c_str(), tip_name_.c_str());
        return false;
      }
    if (!readJoints(robot_model)) 
      {
        ROS_FATAL("Could not read information about the joints");
        return false;
      }
    return true;
  }

  bool Kinematics::readJoints(urdf::Model &robot_model) 
  {
    num_joints_ = 0;
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name_);
    boost::shared_ptr<const urdf::Joint> joint;
    for (int i = 0; i < chain_.getNrOfSegments(); i++)
      while (link && link->name != root_name_) 
        {
          if (!(link->parent_joint)) 
            break;
      
          joint = robot_model.getJoint(link->parent_joint->name);
          if (!joint) 
            {
              ROS_ERROR("Could not find joint: %s", link->parent_joint->name.c_str());
              return false;
            }
          if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) 
            num_joints_++;

          link = robot_model.getLink(link->getParent()->name);
        }
    info_.joint_names.resize(num_joints_);
    info_.link_names.resize(num_joints_);

    link = robot_model.getLink(tip_name_);
    unsigned int i = 0;
    while (link && i < num_joints_) 
      {
        ROS_INFO_ONCE("Number of Joints: %d, for tip: %s", num_joints_, tip_name_.c_str());
        joint = robot_model.getJoint(link->parent_joint->name);
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) 
          {
            int index = num_joints_ - i - 1;
            info_.joint_names[index] = joint->name;
            info_.link_names[index] = link->name;
            i++;
          }
        link = robot_model.getLink(link->getParent()->name);
      }
    return true;
  }

  //****************************************************************************************************************
  // getJacobian(...)
  // Input:  joints and namespace
  // Output: jacobian matrix (6x7)
  // See more of KDLs API at:
  // Jacobians: http://people.mech.kuleuven.be/~rsmits/kdl/api/html/classKDL_1_1Jacobian.htmlxs
  // Joint Arrays: http://www.letworyinteractive.com/blendercode/db/d00/classKDL_1_1JntArray.html
  // Jacobian Solver: http://people.mech.kuleuven.be/~rsmits/kdl/api/html/classKDL_1_1ChainJntToJacSolver.html
  //****************************************************************************************************************
  bool Kinematics::getJacobian(std::vector<double> joints, std::vector<std::string> names, Eigen::MatrixXd& jacobian)
  {
    // KDL Types
    KDL::Jacobian J(7);                          // Pass the number of columns for this jacobian. 
    KDL::JntArray jntPos;                        // Crate an array. Needs to be resized. 
    KDL::ChainJntToJacSolver j_solver(chain_);   // Solver requires the kinematic chain.
    jntPos.resize(joints.size());

    // Copy joint values to the joint position.
    ROS_INFO_STREAM_ONCE("Kinematics: Joint size: " << joints.size());
    for (unsigned int j = 0; j < names.size(); j++) 
      {
        for (unsigned int i = 0; i < num_joints_; i++) 
          {
            // Given that baxter's joints sometimes may come in a different order, this helps to ensure the right values are copied. 
            if (names[j] == info_.joint_names[i])
              {
                jntPos(i) = joints[j];
                break;
              } 
          }
      }
    
    // Call the Jacobian solver: takes the joint angles and the jacobian. 
    // Calculate the jacobian expressed in the base frame of the chain, with reference point at the end effector of the *chain.
    j_solver.JntToJac(/*in*/jntPos, /*out*/J);


    // Copy to the jacobian to the output argument. 
    // The matix needs to be resized appropriately. It is also a column-major structure. 
    int row, col;
    row = J.rows(); col = J.columns();
    ROS_WARN_STREAM_ONCE("Kinematics: Jac size: rows: " << row << ", cols: " << col);

    jacobian.resize(row, col);
    jacobian.setZero(row, col); 
    for(unsigned int i=0; i<row; i++) // Copying by rows. Better if by columns but all info remains correct. 
      {
        for(unsigned int j=0; j<col; j++)
          jacobian(i, j) = J(i, j);
      }
    return true;
  }

  bool Kinematics::getJacPseudoInv(std::vector<double> joints, std::vector<std::string> names, Eigen::MatrixXd& jacobian)
  {
    Eigen::MatrixXd j, jt, temp, inv;

    getJacobian(joints, names, j);
    jt = j.transpose();
    temp = j*jt;
    inv = temp.inverse();
    jacobian = jt*inv;
    return true;
  }
}  //namespace

