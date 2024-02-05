#include <ministepback_recovery/ministepback_recovery.h>
#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(ministepback_recovery::MiniStepBackRecovery, nav_core::RecoveryBehavior)

namespace ministepback_recovery {


MiniStepBackRecovery::MiniStepBackRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  initialized_(false), world_model_(NULL) {} 

// initialize
//------------
void MiniStepBackRecovery::initialize(std::string name, tf2_ros::Buffer* tf,
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

    private_nh.param("setpback_speed", stepback_speed_, -0.05);
    private_nh.param("stepback_time", stepback_time_, 0.4);
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


MiniStepBackRecovery::~MiniStepBackRecovery(){
  delete world_model_;
}

// EXECUTION OF THE RECOVERY (see planner_common_params.yaml to see the order of recovery behaviours)
//---------------------------
void MiniStepBackRecovery::runBehavior()
{
    if(!initialized_){
        ROS_ERROR("This object must be initialized before runBehavior is called");
        return;
    }

    if(global_costmap_ == NULL || local_costmap_ == NULL){
        ROS_ERROR("The costmaps passed to the MiniStepBackRecovery object cannot be NULL. Doing nothing.");
        return;
    }
    ROS_WARN("MiniStepback recovery behavior started. Hope it goes well...");

    ros::Rate r(frequency_);
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Just move back
    ros::Time start_time = ros::Time::now();

    while ( n.ok() && (ros::Time::now()-start_time) < ros::Duration(stepback_time_) )
    {
        // Set speed
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = stepback_speed_;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub.publish(cmd_vel);
        r.sleep();
    }
}

};

