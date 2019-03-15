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

//#define EPSILON 0.000001

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

// bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
//           const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame, std::vector<geometry_msgs::PoseStamped>& transformed_plan);
// /**
//   * @brief  Transforms the global plan of the robot from the planner frame to the local frame (modified).
//   *
//   * The method replaces transformGlobalPlan as defined in base_local_planner/goal_functions.h
//   * such that the index of the current goal pose is returned as well as
//   * the transformation between the global plan and the planning frame.
//   * @param tf A reference to a transform listener
//   * @param global_plan The plan to be transformed
//   * @param global_pose The global pose of the robot
//   * @param costmap A reference to the costmap being used so the window size for transforming can be computed
//   * @param global_frame The frame to transform the plan to
//   * @param max_plan_length Specify maximum length (cumulative Euclidean distances) of the transformed plan [if <=0: disabled; the length is also bounded by the local costmap size!]
//   * @param[out] transformed_plan Populated with the transformed plan
//   * @param[out] current_goal_idx Index of the current (local) goal pose in the global plan
//   * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
//   * @return \c true if the global plan is transformed, \c false otherwise
//   */
// bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
//                          const tf::Stamped<tf::Pose>& global_pose,  const costmap_2d::Costmap2D& costmap,
//                          const std::string& global_frame, double max_plan_length, std::vector<geometry_msgs::PoseStamped>& transformed_plan,
//                          int* current_goal_idx = NULL, tf::StampedTransform* tf_plan_to_global = NULL) const;

 void reconfigureCB(PathFollowerReconfigureConfig& config, uint32_t level);

private:
// bool posesEqual(geometry_msgs::PoseStamped first, geometry_msgs::PoseStamped second);
//  double in_path_vel_;
//  double to_path_k_;
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
  tf::TransformListener* tfl_;
  costmap_2d::Costmap2DROS* costmap_ros_;

  bool initialized_;
  bool goal_reached_;
  ros::Publisher marker_pub;

  double sign(double x)  { return x < 0.0 ? -1.0 : +1.0; };
  float calculate_translation(float current, float desired);
  float calculate_rotation( float current, float desired );
};

} //namespace omni_path_follower

#endif
