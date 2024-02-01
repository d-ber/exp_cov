#ifndef ROTATE_RECOVERY_H_
#define ROTATE_RECOVERY_H_
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>

namespace stepback_recovery{
  /**
   * @class StepBackRecovery
   * @brief A recovery behavior that stepbacks the robot in-place to attempt to clear out space
   */
  class StepBackRecovery : public nav_core::RecoveryBehavior {
    public:
      /**
       * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
       * @param  
       * @return 
       */
      StepBackRecovery();

      /**
       * @brief  Initialization function for the StepBackRecovery recovery behavior
       * @param tf A pointer to a transform listener
       * @param global_costmap A pointer to the global_costmap used by the navigation stack 
       * @param local_costmap A pointer to the local_costmap used by the navigation stack 
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

      /**
       * @brief  Run the StepBackRecovery recovery behavior.
       */
      void runBehavior();

      /**
       * @brief  Destructor for the stepback recovery behavior
       */
      ~StepBackRecovery();

    private:
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
      costmap_2d::Costmap2D costmap_;
      std::string name_;
      /*tf::TransformListener* tf_;*/
      bool initialized_;
      double sim_granularity_, acc_lim_x_, max_vel_trans_, min_vel_trans_, xy_goal_tolerance_, frequency_;
      double stepback_len_;
      base_local_planner::CostmapModel* world_model_;
  };
};
#endif  

