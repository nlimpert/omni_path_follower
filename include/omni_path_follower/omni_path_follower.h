#ifndef OMNI_PATH_FOLLOWER_H
#define OMNI_PATH_FOLLOWER_H

/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/goal_functions.h>
#include <Eigen/Dense>
#include <omni_path_follower/pose_se2.h>
#include <omni_path_follower/misc.h>

#include <boost/shared_ptr.hpp>

// transforms
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// dynamic reconfigure
#include <omni_path_follower/PathFollowerReconfigureConfig.h>
#include <dynamic_reconfigure/server.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


using std::string;

namespace omni_path_follower
{

class PathFollower : public nav_core::BaseLocalPlanner
{
public:

  PathFollower();

  /** overridden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros);
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
  bool isGoalReached();
  bool setPlan(const std::vector< geometry_msgs::PoseStamped > &plan);

  /**
   * @brief Prune global plan such that already passed poses are cut off
   *
   * The pose of the robot is transformed into the frame of the global plan by taking the most recent tf transform.
   * If no valid transformation can be found, the method returns \c false.
   * The global plan is pruned until the distance to the robot is at least \c dist_behind_robot.
   * If no pose within the specified treshold \c dist_behind_robot can be found,
   * nothing will be pruned and the method returns \c false.
   *
   * Note: this method is taken from the teb_local_planner package!
   *
   * @remarks Do not choose \c dist_behind_robot too small (not smaller the cellsize of the map), otherwise nothing will be pruned.
   * @param tf A reference to a transform listener
   * @param global_pose The global pose of the robot
   * @param[in,out] global_plan The plan to be transformed
   * @param dist_behind_robot Distance behind the robot that should be kept [meters]
   * @return \c true if the plan is pruned, \c false in case of a transform exception or if no pose cannot be found inside the threshold
   */
 bool pruneGlobalPlan(const tf::TransformListener& tf, const tf::Stamped<tf::Pose>& global_pose,
                      std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot=1);

 void reconfigureCB(PathFollowerReconfigureConfig& config, uint32_t level);

 bool updateTrajectoryIfNeeded(geometry_msgs::Twist& twist);

private:
  double angle_k_;
  double goal_threshold_linear_;
  double goal_threshold_angular_;
  double pot_min_dist_;
  double min_vel_lin_;
  double max_vel_lin_;
  double k_trans_;
  double k_rot_;
  double max_vel_lin_at_goal_;
  double acc_lim_lin_;
  double acc_lim_theta_;
  double max_vel_theta_;
  double min_vel_theta_;
  double rot_to_goal_pose_dist_;
  double rotate_from_obstacles_k_;
  double goal_k_;
  double obstacle_k_;
  double cutoff_factor_at_goal_;
  double ang_trans_k_;

  double lookahead_distance_;

  float acc_lin_inc_;
  float acc_lin_dec_;
  float acc_theta_inc_;
  float acc_theta_dec_;

  float loop_time_;

  bool rotate_to_path_;
  bool rotate_at_start_;
  bool visualize_;
  base_local_planner::OdometryHelperRos odom_helper_; //!< Provides an interface to receive the current velocity from the robot
  boost::shared_ptr< dynamic_reconfigure::Server<PathFollowerReconfigureConfig> > dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime

  geometry_msgs::Pose last_waypoint_;
  geometry_msgs::Pose next_waypoint_;
  int path_length_;
  int path_index_;

  std::string global_frame_;

  PoseSE2 robot_pose_; //!< Store current robot pose
  PoseSE2 robot_goal_; //!< Store current robot goal

  geometry_msgs::Twist target_vel_; //!< Store target robot translational and angular velocity (vx, vy, omega)
  geometry_msgs::Twist new_robot_vel_; //!< Store current robot translational and angular velocity (vx, vy, omega)
  geometry_msgs::Twist last_vel_; //!< Store current robot translational and angular velocity (vx, vy, omega)

  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped goal_;

  // TODO: change to TF2!
  tf::TransformListener* tfl_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;

  bool initialized_;
  bool goal_reached_;
  ros::Publisher marker_pub;

  boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;

  // members taken from assisted_teleop
  int num_th_samples_, num_x_samples_;
  double theta_range_;
  double collision_trans_speed_, collision_rot_speed_;
  std::vector<geometry_msgs::Point> footprint_spec_; //!< Store the footprint of the robot
  double robot_inscribed_radius_; //!< The radius of the inscribed circle of the robot (collision possible)
  double robot_circumscribed_radius_; //!< The radius of the circumscribed circle of the robot

  double sign(double x)  { return x < 0.0 ? -1.0 : +1.0; };
  float calculate_translation(float current, float desired);
  float calculate_rotation( float current, float desired );

  bool transformTwist(Eigen::Vector3f &twist_in, Eigen::Vector3f &twist_out);

};

} //namespace omni_path_follower

#endif
