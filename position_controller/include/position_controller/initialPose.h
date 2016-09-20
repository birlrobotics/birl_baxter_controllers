#ifndef POSITION_CONTROLLER_
#define POSITION_CONTROLLER_

#include <ros/ros.h>
#include <tf/tf.h>

#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


#define PI 3.141592654

namespace position_controller
{

  class moveBaxter
  {
  public:

    // Constructor inherits from node_handle_
  	moveBaxter(ros::NodeHandle node): node_handle_(node)
	  {
      double oneDeg = PI/180;

      // Set Parameter Values
      node_handle_.param<double>("precision", tolerance_, oneDeg);
      node_handle_.param<double>("filter", alpha_, 0.9950);  //original 0.987512

      // Set subscribers with their callbacks
      joint_state_sub_ = root_handle_.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1, &moveBaxter::updateJoints, this);
      move_            = root_handle_.subscribe<sensor_msgs::JointState>("/move_Baxter",        1, &moveBaxter::execute,      this);
	  }

    // Destructor
	  ~moveBaxter() {}

	private:

	  void fillJointNames();
	  void updateJoints(const sensor_msgs::JointStateConstPtr& states);
	  bool isMoveFinish(bool& result);
	  void execute(sensor_msgs::JointState qd);

    // Inlined Initialization function: Get time and publish joint commands. 
	  inline void initialize()
	  {
      fillJointNames();
      to_ = ros::Time::now();
      m_ = 0;
	
      // Create joint command publisher object
      joint_cmd_pub_ = root_handle_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + side_ + "/joint_command", 20, false);
      ros::Duration(3.0).sleep();
      ROS_INFO("Initial Pose initialized for %s arm, tolerance = %f", side_.c_str(), tolerance_);	
	  }

    // Private Objects: subscribers, publishers, and node handles. 
	  ros::Subscriber joint_state_sub_, move_;
	  ros::Publisher joint_cmd_pub_;
	  ros::NodeHandle node_handle_, root_handle_;
	 
	  std::string side_;
	  baxter_core_msgs::JointCommand qgoal_;
	  std::vector<double> joints_, goal_, qd_, qe_;
	  std::vector<std::string> joints_names_;
	  double tolerance_, max_error_, alpha_;
	  ros::Time to_, tjoints_;
	  int n_, m_;
  };

}

#endif /* POSITION_CONTROLLER_ */
