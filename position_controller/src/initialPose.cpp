#include <position_controller/initialPose.h>

namespace position_controller
{
  void moveBaxter::fillJointNames()
  {
    joints_names_.clear();
    joints_names_.push_back(side_ + "_e0");
    joints_names_.push_back(side_ + "_e1");
    joints_names_.push_back(side_ + "_s0");
    joints_names_.push_back(side_ + "_s1");
    joints_names_.push_back(side_ + "_w0");
    joints_names_.push_back(side_ + "_w1");
    joints_names_.push_back(side_ + "_w2");

    joints_.clear();
    qe_.clear();
    for(unsigned int i=0; i<joints_names_.size(); i++)
      {
        joints_.push_back(0.0);
        qe_.push_back(100.0);
      }
  }

  void moveBaxter::updateJoints(const sensor_msgs::JointStateConstPtr& states)
  {
    unsigned int i, k=0;
    tjoints_ = states->header.stamp;
    m_ = m_ + 1;
    while(k < joints_names_.size())
      {
        for(i=0; i<states->name.size(); i++)
          {
            if(states->name[i] == joints_names_[k])
              {
                joints_[k] = states->position[i];
                k = k + 1;
                if(k == joints_names_.size())
                  break;
              }
          }
      }
  }

  bool moveBaxter::isMoveFinish(bool& result)
  {  
    //this function is called 20 times in a sec
    double max=0.0;
    std::vector<double> error;
    error.clear();
    n_ = n_ + 1;

    if( n_!=0  &&  (n_ % 5 != 0) )  
      {
        result = false;
        return false;
      }

    for(unsigned int i=0; i<joints_.size(); i++)
      {
        error.push_back( joints_[i] - qd_[i] );
        if ( fabs(error.back()) > max)
          max = error.back();
      }

    int ok=0, keep=0, k=0;
    std::vector<double> q;
    q.clear();	q.resize(joints_.size());
    bool copy;
    for(unsigned int i=0; i < joints_names_.size(); i++)
      {
        copy = true;
        if( k < qgoal_.names.size() )
          {
            if(qgoal_.names[k] == joints_names_[i]) 
              {
                q[i] = qgoal_.command[k];
                k = k + 1;
                copy = false;
              }
          }

        if(copy)
          q[i] = joints_[i];
      }	
    qgoal_.names.clear();
    goal_.clear();
    qgoal_.command.clear();

    for(unsigned int i=0; i<joints_.size(); i++)
      {	
        if( fabs(error[i]) > tolerance_)
          {
            qgoal_.names.push_back( joints_names_[i] );
            qgoal_.command.push_back(q[i]);
            goal_.push_back(qd_[i]);
            if( fabs( error[i] - qe_[i] ) != 0.0 )  //Check that the error decreases
              keep = keep + 1;
          }
        else
          ok = ok + 1;
      }
    qe_ = error;
    ros::Time tnow = ros::Time::now();
    if( (keep + ok == joints_.size()) && (ok != joints_.size()) && ( ((tnow - to_).toSec()) <= 90.0 ))
      {
        if(n_ % 200 == 0)
          ROS_WARN("time: %f, keep = %d, ok = %d", tnow.toSec(), keep, ok);

        result = true;
        return false;
      }
    else if(ok == joints_.size())
      {
        //ROS_INFO("Error is under tolerance!");
        result = true;
        return true;
      }

    // Timer. If robot cannot reach goal with this level of accuracy before timeout, send error. 
    else if( (keep <= 2) || ( ((tnow - to_).toSec()) > 90.0 ) )
      {
        result = false;   //If the error does not decrease and its not near the goal, finish move but indicate that the result is not good
        ROS_ERROR("Time Out, t = %f, n = %d", tnow.toSec(), n_);
        return true;
      }

    if(n_ % 200 == 0)
      ROS_WARN("F: time: %f, keep = %d, ok = %d", tnow.toSec(), keep, ok);
	
    result = true;
    return false;
  }

  void moveBaxter::execute(sensor_msgs::JointState qd)
  {
    side_ = qd.header.frame_id;
    initialize();
	
    goal_.clear();	qd_.clear();
    for(unsigned int i=0; i<qd.position.size(); i++)
      goal_.push_back(qd.position[i]);

    // Create a new variable qgoal in which we filter the joing angle between the current position joints_ and the goal position goal_. If alpha is 0, we send the goal directly, if alpha is 1, we stay in our current position. This filter has the effect of speeding up or slowing down the motion of the robot. 
    qd_ = goal_;
    qgoal_.mode = qgoal_.POSITION_MODE;
    qgoal_.names = joints_names_;
    qgoal_.command.resize( goal_.size() );

    for(unsigned int i=0; i< goal_.size(); i++)
      qgoal_.command[i] = ((1-alpha_)*goal_[i]) + (alpha_ * joints_[i]);

    to_ = ros::Time::now();
    n_ = 0;
    ROS_INFO("Moving arm to goal position . . . to = %f", to_.toSec());
    joint_cmd_pub_.publish(qgoal_);

    bool cont, fin = isMoveFinish(cont);
    while(!fin && node_handle_.ok())
      {
        //Filter
        for(unsigned int i=0; i<goal_.size(); i++)
          qgoal_.command[i] = ((1-alpha_)*goal_[i]) + (alpha_ * qgoal_.command[i]);	

        joint_cmd_pub_.publish(qgoal_);
        fin = isMoveFinish(cont);
        ros::Duration(0.05).sleep();
      }
    ROS_WARN("Joint error: %f, %f, %f, %f, %f, %f, %f", qe_[0],qe_[1],qe_[2],qe_[3],qe_[4],qe_[5],qe_[6]);

    ROS_INFO("----------Finished moving %s arm", side_.c_str());
  }

}  //namespace position_controller


int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_controller");
  //ROS_INFO("Main start");  
  
  // Private Namespace
  ros::NodeHandle node("~");
  
  // Instantiate moveBaxter class. 
  position_controller::moveBaxter move_arm(node);
  ros::Duration(1.0).sleep();

  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();

  return 0;
}
