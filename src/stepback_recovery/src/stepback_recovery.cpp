#include <stepback_recovery/stepback_recovery.h>
#include <pluginlib/class_list_macros.hpp>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(stepback_recovery::StepBackRecovery, nav_core::RecoveryBehavior)

namespace stepback_recovery {


StepBackRecovery::StepBackRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  initialized_(false), world_model_(NULL) {} 

// initialize
//------------
void StepBackRecovery::initialize(std::string name, tf2_ros::Buffer* tf,
costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
{
  if(!initialized_)
  {
    name_ = name;
    //tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    private_nh.param("stepback_len", stepback_len_, 0.15);

    // we'll simulate every 5 cm by default
    private_nh.param("sim_granularity", sim_granularity_, 0.05);
    private_nh.param("frequency", frequency_, 20.0);

    blp_nh.param("acc_lim_x", acc_lim_x_, 2.5);
    blp_nh.param("max_vel_trans", max_vel_trans_, 0.42);
    blp_nh.param("min_vel_trans", min_vel_trans_, 0.1);
    blp_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.05);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}


StepBackRecovery::~StepBackRecovery(){
  delete world_model_;
}

// EXECUTION OF THE RECOVERY (see planner_common_params.yaml to see the order of recovery behaviours)
//---------------------------
void StepBackRecovery::runBehavior()
{
    if(!initialized_){
        ROS_ERROR("This object must be initialized before runBehavior is called");
        return;
    }

    if(global_costmap_ == NULL || local_costmap_ == NULL){
        ROS_ERROR("The costmaps passed to the StepBackRecovery object cannot be NULL. Doing nothing.");
        return;
    }
    ROS_WARN("Stepback recovery behavior started.");

    ros::Rate r(frequency_);
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    geometry_msgs::PoseStamped global_pose;
    local_costmap_->getRobotPose(global_pose);

    double start_x = global_pose.pose.position.x;
    double start_y = global_pose.pose.position.y;
    double start_theta = tf2::getYaw(global_pose.pose.orientation);
    double current_x = start_x;
    double current_y = start_y;
    double current_theta = start_theta;
    double target_x;
    double target_y;
    double target_theta = start_theta;

    if (std::fabs(angles::shortest_angular_distance(start_theta, 0.0)) <= M_PI/2){ //move to the left
      target_x = start_x - stepback_len_ * std::cos(start_theta);
    } else{ //move to the right
      target_x = start_x + stepback_len_ * std::cos(start_theta);
    }
    if (start_theta <= M_PI){ //robot faces upwards
      target_y = start_y - stepback_len_ * std::sin(start_theta);
    }else{ //robot faces downwards
      target_y = start_y + stepback_len_ * std::sin(start_theta);
    }

    //ROS_ERROR("INIZIO: start_x: %.2f, start_y: %.2f, start_theta: %.2f, current_x: %.2f, current_y: %.2f, current_theta: %.2f, target_x: %.2f, target_y: %.2f, target_theta: %.2f", start_x, start_y, start_theta, current_x, current_y, current_theta, target_x, target_y, target_theta);

    while (n.ok() && ((std::fabs(current_x-target_x) > xy_goal_tolerance_) || (std::fabs(current_y-target_y) > xy_goal_tolerance_)))
    {
      // Update Current Pose
      local_costmap_->getRobotPose(global_pose);
      current_x = global_pose.pose.position.x;
      current_y = global_pose.pose.position.y;
      current_theta = tf2::getYaw(global_pose.pose.orientation);

      //ROS_ERROR("start_x: %.2f, start_y: %.2f, start_theta: %.2f, current_x: %.2f, current_y: %.2f, current_theta: %.2f, target_x: %.2f, target_y: %.2f, target_theta: %.2f", start_x, start_y, start_theta, current_x, current_y, current_theta, target_x, target_y, target_theta);

      // distance already done
      double dist_done = std::sqrt(std::pow(current_y-start_y, 2)+std::pow(current_x-start_x, 2));

      // compute the distance left to move
      double dist_left = std::sqrt(std::pow(current_y-target_y, 2)+std::pow(current_x-target_x, 2));

      //ROS_ERROR("dist_left: %.2f", dist_left);

      // check if that velocity is legal by forward simulating
      double sim_step = 0.0;
      while (sim_step < dist_left)
      {
        double sim_theta = current_theta;
        double sim_x;
        double sim_y;
        if (std::fabs(angles::shortest_angular_distance(start_theta, 0.0))){ //move to the left
          sim_x = current_x - std::cos(current_theta) * sim_step;
        } else{ //move to the right
          sim_x = current_x + std::cos(current_theta) * sim_step;
        }
        if (start_theta <= M_PI){ //robot faces upwards
          sim_y = current_y - std::sin(current_theta) * sim_step;
        }else{ //robot faces downwards
          sim_y = current_y + std::sin(current_theta) * sim_step;
        }

        // make sure that the point is legal, if it isn't... we'll abort
        double footprint_cost = world_model_->footprintCost(sim_x, sim_y, sim_theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
        if (footprint_cost < 0.0)
        {
          ROS_ERROR("Stepback Recovery can't stepback because there is a potential collision. Cost: %.2f", footprint_cost);
          ROS_ERROR("Stepback Recovery can't stepback because there is a potential collision. Step considered: %.2f of %.2f left", sim_step, dist_left);
          return;
        }

        sim_step += sim_granularity_;
      }

      // compute the velocity that will let us stop by the time we reach the goal
      double vel = sqrt(2 * acc_lim_x_ * dist_left);

      //ROS_ERROR("VEL: %.2f, start_x: %.2f, start_y: %.2f, start_theta: %.2f, current_x: %.2f, current_y: %.2f, current_theta: %.2f, target_x: %.2f, target_y: %.2f, target_theta: %.2f", vel, start_x, start_y, start_theta, current_x, current_y, current_theta, target_x, target_y, target_theta);

      // make sure that this velocity falls within the specified limits
      vel = std::min(std::max(vel, min_vel_trans_), max_vel_trans_);

      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = -vel;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;

      vel_pub.publish(cmd_vel);

      r.sleep();
    }
}

};

