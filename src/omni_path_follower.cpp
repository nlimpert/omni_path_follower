#include <omni_path_follower/omni_path_follower.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(omni_path_follower::PathFollower, nav_core::BaseLocalPlanner)

namespace omni_path_follower
{
  PathFollower::PathFollower() :
     initialized_(false)
   {
    double controller_frequency = 5;
    ros::NodeHandle nh_move_base("~/");
    nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);

    loop_time_ = 1 / controller_frequency;
   }

  void PathFollower::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
  {
    tfl_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.

    ros::NodeHandle nh("~/" + name);

    nh.param("angle_k", angle_k_, 1.5);
    nh.param("goal_threshold_linear", goal_threshold_linear_, 0.05);
    nh.param("visualize", visualize_, false);
    nh.param("max_vel_lin", max_vel_lin_, 1.0);
    nh.param("min_vel_lin", min_vel_lin_, 1.0);
    nh.param("max_vel_lin_at_goal", max_vel_lin_at_goal_, 0.1);
    nh.param("max_vel_theta", max_vel_theta_, 1.0);
    nh.param("min_vel_theta", min_vel_theta_, 0.1);
    nh.param("acc_lim_lin", acc_lim_lin_, 1.0);
    nh.param("acc_lim_theta", acc_lim_theta_, 1.0);
    nh.param("rot_to_goal_pose_dist", rot_to_goal_pose_dist_, 2.0);
    nh.param("angle_k", angle_k_, 1.0);
    nh.param("goal_k", goal_k_, 130.0);
    nh.param("obstacle_k", obstacle_k_, 1.0);
    nh.param("rotate_from_obstacles_k", rotate_from_obstacles_k_, 130.0);
    nh.param("pot_min_dist", pot_min_dist_, 0.2);
    nh.param("cutoff_factor_at_goal", cutoff_factor_at_goal_, 0.1);
    nh.param("goal_threshold_linear", goal_threshold_linear_, 0.05);
    nh.param("goal_threshold_angular", goal_threshold_angular_, 0.05);
    nh.param("lookahead_distance", lookahead_distance_, 1.0);

    nh.param("num_th_samples", num_th_samples_, 20);
    nh.param("num_x_samples", num_x_samples_, 10);
    nh.param("theta_range", theta_range_, 0.7);
    nh.param("translational_collision_speed", collision_trans_speed_, 0.0);
    nh.param("rotational_collision_speed", collision_rot_speed_, 0.0);

    global_frame_ = costmap_ros_->getGlobalFrameID();
    footprint_spec_ = costmap_ros_->getRobotFootprint();

    //initialize empty global plan
    std::vector<geometry_msgs::PoseStamped> empty_plan;
    empty_plan.push_back(geometry_msgs::PoseStamped());
    global_plan_ = empty_plan;


    costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);

    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_);

    ROS_INFO("Calculated MinAndMaxDistances: %f %f", robot_inscribed_radius_, robot_circumscribed_radius_);

    dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<PathFollowerReconfigureConfig> >(nh);
    dynamic_reconfigure::Server<PathFollowerReconfigureConfig>::CallbackType cb = boost::bind(&PathFollower::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);

    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("omni_path_follower_vis", 1);
    goal_reached_ = false;
    initialized_ = true;

    ROS_INFO("omni_path_follower initialized, loop time: %f", loop_time_);
    return;
  }

  bool PathFollower::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
  {
    // check if plugin initialized
    if(!initialized_)
    {
      ROS_ERROR("omni_path_follower has not been initialized, please call initialize() before using this planner");
      return false;
    }

    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    goal_reached_ = false;

    // Get robot pose
    tf::Stamped<tf::Pose> robot_pose;
    costmap_ros_->getRobotPose(robot_pose);
    robot_pose_ = PoseSE2(robot_pose);

    // prune global plan to cut off parts of the past (spatially before the robot)
    pruneGlobalPlan(*tfl_, robot_pose, global_plan_);

    // Transform global plan to the frame of interest (w.r.t. the local costmap)
    std::vector<geometry_msgs::PoseStamped> transformed_plan;

    if(!base_local_planner::transformGlobalPlan(*tfl_, global_plan_, robot_pose, *costmap_ros_->getCostmap(), costmap_ros_->getGlobalFrameID(), transformed_plan))
    {
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }
    int global_plan_pose_in_lookahead = transformed_plan.size() - 1;
//    double d = 0.0;

    for (int i = transformed_plan.size() - 1; i > 0; i--) {
      // find a pose in the global path that's not further than lookahead_distance_
      double x_square = (robot_pose_.x() - transformed_plan[i].pose.position.x) * (robot_pose_.x() - transformed_plan[i].pose.position.x);
      double y_square = (robot_pose_.y() - transformed_plan[i].pose.position.y) * (robot_pose_.y() - transformed_plan[i].pose.position.y);
      double d = sqrt(x_square + y_square);
//      ROS_INFO("%i %f %f and %f", i , transformed_plan[i].pose.position.x ,transformed_plan[i].pose.position.y, d);
      if (d < lookahead_distance_) {
        global_plan_pose_in_lookahead = i;
//        ROS_INFO("took %i at distance %f", global_plan_pose_in_lookahead, d);
        break;
      }
    }

    // check if global goal is reached
    tf::Stamped<tf::Pose> global_goal;
    tf::poseStampedMsgToTF(transformed_plan.back(), global_goal);
    double dx = global_goal.getOrigin().getX() - robot_pose_.x();
    double dy = global_goal.getOrigin().getY() - robot_pose_.y();
    double delta_orient = g2o::normalize_theta( tf::getYaw(global_goal.getRotation()) - robot_pose_.theta());

    bool xy_tolerance_reached = fabs(std::sqrt(dx*dx+dy*dy)) < goal_threshold_linear_;
    bool ori_tolerance_reached = fabs(delta_orient) < goal_threshold_angular_;

    if(xy_tolerance_reached && ori_tolerance_reached)
    {
      ROS_INFO_NAMED("omni_path_follower", "GOAL Reached!");
      goal_reached_ = true;
      return true;
    }

    // Get current goal point (last point of the transformed plan)
    tf::Stamped<tf::Pose> local_goal;
    tf::poseStampedMsgToTF(transformed_plan[global_plan_pose_in_lookahead], local_goal);

    robot_goal_.x() = local_goal.getOrigin().getX();
    robot_goal_.y() = local_goal.getOrigin().getY();

    double robot_ori = tf::getYaw(robot_pose.getRotation());

    Eigen::Vector2d vec_goalrobot(robot_goal_.x() - robot_pose.getOrigin().getX(),
                                  robot_goal_.y() - robot_pose.getOrigin().getY());

//    float att_x = 0., att_y = 0.;
//    float att_phi = 0.0;

    // add current goal as an attraction point to target
//    att_x = vec_goalrobot[0];
//    att_y = vec_goalrobot[1];
//    att_phi = angles::normalize_angle(atan2(att_y, att_x) - robot_ori);

    // determine whether we selected the last waypoint in order to scale down the rep force.
    bool last_waypoint_selected_ =
        fabs(robot_goal_.x() - global_plan_.back().pose.position.x) < 0.000001 &&
        fabs(robot_goal_.y() - global_plan_.back().pose.position.y) < 0.000001;

    float target_vel_x = 0.;
    float target_vel_y = 0.;

    float cur_max_vel = max_vel_lin_;

    // is the currently selected goal the goal of the global path?
    if (last_waypoint_selected_) {
      target_vel_x = fabs(dx) > 0.5 ? 0.5 : std::max(fabs(dx), max_vel_lin_at_goal_);
      target_vel_y = fabs(dy) > 0.5 ? 0.5 : std::max(fabs(dy), max_vel_lin_at_goal_);
    } else {
      target_vel_x = cur_max_vel;
      target_vel_y = cur_max_vel;
    }

    double goal_phi = angles::shortest_angular_distance(robot_ori, atan2(vec_goalrobot[1], vec_goalrobot[0]));

    float target_ori = 0.;

    // rotate to goal pose when we are getting close
    if (fabs(dx) < rot_to_goal_pose_dist_ && fabs(dy) < rot_to_goal_pose_dist_) {
      target_ori = angles::shortest_angular_distance(robot_ori,tf::getYaw(global_goal.getRotation()));
    } else {
      target_ori = goal_phi;
    }

    float drive_x = vec_goalrobot[0];
    float drive_y = vec_goalrobot[1];
    float drive_phi = angles::normalize_angle(atan2(drive_y, drive_x) - robot_ori - target_ori * ang_trans_k_);

    float drive_part_x = 0.f;
    float drive_part_y = 0.f;

    drive_part_x = std::cos( drive_phi );
    drive_part_y = std::sin( drive_phi );

    target_vel_.linear.x = drive_part_x * target_vel_x;
    target_vel_.linear.y = drive_part_y * target_vel_y;
    target_vel_.angular.z = angle_k_ * target_ori;

    updateTrajectoryIfNeeded(target_vel_);

    // limit angular vel
    if (fabs(target_vel_.angular.z) > 0.00001 ) {
      target_vel_.angular.z = std::min(fabs(target_vel_.angular.z), max_vel_theta_) *
          (target_vel_.angular.z / fabs(target_vel_.angular.z));
    }

//    //first, we'll check the trajectory that the user sent in... if its legal... we'll just follow it
//    if(checkTrajectory(desired_vel[0], desired_vel[1], desired_vel[2], true)){
//      geometry_msgs::Twist cmd;
//      cmd.linear.x = desired_vel[0];
//      cmd.linear.y = desired_vel[1];
//      cmd.angular.z = desired_vel[2];
//      pub_.publish(cmd);
//      r.sleep();
//      continue;
//    }

    cmd_vel.linear.x = calculate_translation(last_vel_.linear.x, target_vel_.linear.x);
    cmd_vel.linear.y = calculate_translation(last_vel_.linear.y, target_vel_.linear.y);
    cmd_vel.angular.z = calculate_rotation(last_vel_.angular.z, target_vel_.angular.z);

    last_vel_.linear.x = cmd_vel.linear.x;
    last_vel_.linear.y = cmd_vel.linear.y;
    last_vel_.angular.z = cmd_vel.angular.z;

    if (visualize_ == true) {
      visualization_msgs::Marker goal_marker, att_marker, result_marker_vel;
      visualization_msgs::MarkerArray markers;

      // add repelling marker
      goal_marker.header.frame_id = "/map";
      goal_marker.header.stamp = ros::Time::now();

      goal_marker.type = visualization_msgs::Marker::CYLINDER;
      goal_marker.action = visualization_msgs::Marker::ADD;
      goal_marker.scale.x = 0.1;
      goal_marker.scale.y = 0.1;
      goal_marker.scale.z = 0.3;
      goal_marker.color.a = 0.5;
      goal_marker.id = 0;
      goal_marker.color.r = 1.0;
      goal_marker.color.g = 0.0;
      goal_marker.color.b = 0.0;

      goal_marker.pose.position.x = robot_goal_.x();
      goal_marker.pose.position.y = robot_goal_.y();

      markers.markers.push_back(goal_marker);

      marker_pub.publish(markers);
    }

    return true;
  }

  bool PathFollower::pruneGlobalPlan(const tf::TransformListener& tf, const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
  {
    if (global_plan.empty())
      return true;

    try
    {
      // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
      tf::StampedTransform global_to_plan_transform;
      tf.lookupTransform(global_plan.front().header.frame_id, global_pose.frame_id_, ros::Time(0), global_to_plan_transform);
      tf::Stamped<tf::Pose> robot;
      robot.setData( global_to_plan_transform * global_pose );

      double dist_thresh_sq = dist_behind_robot*dist_behind_robot;

      // iterate plan until a pose close the robot is found
      std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
      std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
      while (it != global_plan.end())
      {
        double dx = robot.getOrigin().x() - it->pose.position.x;
        double dy = robot.getOrigin().y() - it->pose.position.y;
//        ROS_INFO("robot_pose: %f %f vs %f %f", robot_pose_.x(), robot_pose_.y(), robot.getOrigin().x(), robot.getOrigin().y());
//        ROS_INFO("d: %f %f it: %f %f", dx, dy, it->pose.position.x,it->pose.position.y);

        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < dist_thresh_sq)
        {
           erase_end = it;
           break;
        }
        ++it;
      }
      if (erase_end == global_plan.end())
        return false;

      if (erase_end != global_plan.begin())
        global_plan.erase(global_plan.begin(), erase_end);
    }
    catch (const tf::TransformException& ex)
    {
      ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
      return false;
    }
    return true;
  }

  void PathFollower::reconfigureCB(PathFollowerReconfigureConfig& config, uint32_t level)
  {
    visualize_               = config.visualize;
    pot_min_dist_            = config.pot_min_dist;
    max_vel_lin_             = config.max_vel_lin;
    min_vel_lin_             = config.min_vel_lin;
    max_vel_lin_at_goal_     = config.max_vel_lin_at_goal;
    max_vel_theta_           = config.max_vel_theta;
    min_vel_theta_           = config.min_vel_theta;
    lookahead_distance_      = config.lookahead_distance;
    acc_lim_lin_             = config.acc_lim_lin;
    acc_lim_theta_           = config.acc_lim_theta;
    rot_to_goal_pose_dist_   = config.rot_to_goal_pose_dist;
    angle_k_                 = config.angle_k;
    goal_k_                  = config.goal_k;
    obstacle_k_              = config.obstacle_k;
    rotate_from_obstacles_k_ = config.rotate_from_obstacles_k;
    cutoff_factor_at_goal_   = config.cutoff_factor_at_goal;
    goal_threshold_linear_   = config.goal_threshold_linear;
    goal_threshold_angular_  = config.goal_threshold_angular;
    ang_trans_k_             = config.ang_trans_k;

    acc_lin_inc_   = acc_lim_lin_ * loop_time_;
    acc_theta_inc_ = acc_lim_theta_ * loop_time_;
    acc_lin_dec_   = acc_lim_lin_ * loop_time_ * 10.;
    acc_theta_dec_ = acc_lim_theta_ * loop_time_ * 10.;

  }

  bool PathFollower::isGoalReached()
  {
    return goal_reached_;
  }

  bool PathFollower::setPlan(const std::vector< geometry_msgs::PoseStamped > &plan)
  {
    if(!initialized_)
    {
      ROS_ERROR("path follower: planner has not been initialized");
      return false;
    }

    ROS_DEBUG("path follower: got plan");
    global_plan_  = plan;

    path_index_ = 0;
    path_length_ = global_plan_.size();
    last_waypoint_ = global_plan_.at(path_index_).pose;
    next_waypoint_ = global_plan_.at(path_index_ + 1).pose;

    goal_reached_ = false;
    return true;
  }

  /** Implementation of Calculate Translation Function.
   * This method is taken from the Fawkes plugin "colli":
   * https://www.fawkesrobotics.org/
   *
   * @param current The current translation of the robot
   * @param desired The desired translation of the robot
   * @return the new translation
   */
  float PathFollower::calculate_translation(float current, float desired)
  {
    float exec_trans = 0.0;

    if (desired < current) {

      if (current > 0.0) {
        exec_trans = current - acc_lin_dec_;
        exec_trans = std::max( exec_trans, desired );

      } else if (current < 0.0) {
        exec_trans = current - acc_lin_inc_;
        exec_trans = std::max( exec_trans, desired );

      }  else {
        exec_trans = std::max( -acc_lin_inc_, desired );
      }

    } else if (desired > current) {

      if (current > 0.0) {
        exec_trans = current + acc_lin_inc_;
        exec_trans = std::min( exec_trans, desired );

      } else if (current < 0.0) {
        exec_trans = current + acc_lin_dec_;
        exec_trans = std::min( exec_trans, desired );

      } else {
        exec_trans = std::min( acc_lin_inc_, desired );
      }

    } else {
      // nothing to change
      exec_trans = desired;
    }
    return exec_trans;
  }

  /** Implementation of Calculate Rotation Function.
   * This method is taken from the Fawkes plugin "colli":
   * https://www.fawkesrobotics.org/
   *
   * @param current The current rotation of the robot
   * @param desired The desired rotation of the robot
   * @return the new rotation
   */
  float PathFollower::calculate_rotation( float current, float desired )
  {
    float exec_rot = 0.0;

    if (desired < current) {

      if (current > 0.0) {
        // decrease right rot
        exec_rot = current - acc_theta_dec_;
        exec_rot = std::max( exec_rot, desired );

      } else if (current < 0.0) {
        // increase left rot
        exec_rot = current - acc_theta_inc_;
        exec_rot = std::max( exec_rot, desired );

      } else {
        // current == 0;
        exec_rot = std::max( -acc_theta_inc_, desired );
      }

    } else if (desired > current) {
      if (current > 0.0) {
        // increase right rot
        exec_rot = current + acc_theta_inc_;
        exec_rot = std::min( exec_rot, desired );

      } else if (current < 0.0) {
        // decrease left rot
        exec_rot = current + acc_theta_dec_;
        exec_rot = std::min( exec_rot, desired );

      } else {
        // current == 0
        exec_rot = std::min( acc_theta_inc_, desired );
      }

    } else {
      // nothing to change!!!
      exec_rot = desired;
    }

    return exec_rot;
  }

bool PathFollower::updateTrajectoryIfNeeded(geometry_msgs::Twist& twist)
{
  // The following functionality is essentially taken from navigation_experimental/assisted_teleop

  //TODO: find a more elegant solution here :-(
  Eigen::Vector3f vel = Eigen::Vector3f::Zero();
  vel[0] = twist.linear.x;
  vel[1] = twist.linear.y;
  vel[2] = twist.angular.z;

  Eigen::Vector3f transformed_vel = Eigen::Vector3f::Zero();
  transformTwist(vel, transformed_vel);

  //first, we'll check the trajectory that the user sent in... if its legal... we'll just follow it
  double cost = costmap_model_->footprintCost(transformed_vel[0], transformed_vel[1], transformed_vel[2], footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_);

//  if(costmap_model_->footprintCost(twist.linear.x, twist.linear.y, twist.angular.z, footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_) > 0 ){
  if(cost >= 0.0 ){
    return true;
  }

  // TODO: We have to consider the actual loop time here since currently we're checking
  // for m/s while the loop time is much lower.
  ROS_WARN("Trajectory not feasible: %f %f %f -> %f", vel[0], vel[1], vel[2], cost);

  Eigen::Vector3f desired_vel = Eigen::Vector3f::Zero();
  desired_vel[0] = twist.linear.x;
  desired_vel[1] = twist.linear.y;
  desired_vel[2] = twist.angular.z;

  double dth = (theta_range_) / double(num_th_samples_);
  double dx = twist.linear.x / double(num_x_samples_);
  double start_th = twist.angular.z - theta_range_ / 2.0;

  Eigen::Vector3f best = Eigen::Vector3f::Zero();
  double best_dist = DBL_MAX;
  bool trajectory_found = false;

  //if we don't have a valid trajectory... we'll start checking others in the angular range specified
  for(int i = 0; i < num_x_samples_; ++i){
    Eigen::Vector3f check_vel = Eigen::Vector3f::Zero();
    check_vel[0] = desired_vel[0] - i * dx;
    check_vel[1] = desired_vel[1];
    check_vel[2] = start_th;
    transformTwist(check_vel, transformed_vel);
    for(int j = 0; j < num_th_samples_; ++j){
      check_vel[2] = start_th + j * dth;
      if(costmap_model_->footprintCost(transformed_vel[0], transformed_vel[1], transformed_vel[2], footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius_) >= 0.0 ){
        //if we have a legal trajectory, we'll score it based on its distance to our desired velocity
        Eigen::Vector3f diffs = (desired_vel - check_vel);
        double sq_dist = diffs[0] * diffs[0] + diffs[1] * diffs[1] + diffs[2] * diffs[2];

        //if we have a trajectory that is better than our best one so far, we'll take it
        if(sq_dist < best_dist){
          best = check_vel;
          best_dist = sq_dist;
          trajectory_found = true;
        }
      }
    }
  }

  //check if best is still zero, if it is... scale the original trajectory based on the collision_speed requested
  //but we only need to do this if the user has set a non-zero collision speed
  if(!trajectory_found && (collision_trans_speed_ > 0.0 || collision_rot_speed_ > 0.0)){
    double trans_scaling_factor = 0.0;
    double rot_scaling_factor = 0.0;
    double scaling_factor = 0.0;

    if(fabs(desired_vel[0]) > 0 && fabs(desired_vel[1]) > 0)
      trans_scaling_factor = std::min(collision_trans_speed_ / fabs(desired_vel[0]), collision_trans_speed_ / fabs(desired_vel[1]));
    else if(fabs(desired_vel[0]) > 0)
      trans_scaling_factor = collision_trans_speed_ / (fabs(desired_vel[0]));
    else if(fabs(desired_vel[1]) > 0)
      trans_scaling_factor = collision_trans_speed_ / (fabs(desired_vel[1]));

    if(fabs(desired_vel[2]) > 0)
      rot_scaling_factor = collision_rot_speed_ / (fabs(desired_vel[2]));

    if(collision_trans_speed_ > 0.0 && collision_rot_speed_ > 0.0)
      scaling_factor = std::min(trans_scaling_factor, rot_scaling_factor);
    else if(collision_trans_speed_ > 0.0)
      scaling_factor = trans_scaling_factor;
    else if(collision_rot_speed_ > 0.0)
      scaling_factor = rot_scaling_factor;

    //apply the scaling factor
    best = scaling_factor * best;
  }

  twist.linear.x = best[0];
  twist.linear.y = best[1];
  twist.angular.z = best[2];

  return true;
}

bool PathFollower::transformTwist(Eigen::Vector3f& twist_in, Eigen::Vector3f &twist_out)
{
  //TODO: find a more elegant solution here :-(
  geometry_msgs::PoseStamped stamped_in, stamped_out;

  stamped_in.header.frame_id = costmap_ros_->getBaseFrameID();
  stamped_in.header.stamp = ros::Time::now();
  stamped_in.pose.position.x = twist_in[0];
  stamped_in.pose.position.y = twist_in[1];

  tf::Quaternion quat = tf::createQuaternionFromYaw(twist_in[2]);
  stamped_in.pose.orientation.x = quat.getX();
  stamped_in.pose.orientation.y = quat.getY();
  stamped_in.pose.orientation.z = quat.getZ();
  stamped_in.pose.orientation.w = quat.getW();

  try{
    tfl_->transformPose (costmap_ros_->getGlobalFrameID(), ros::Time(0), stamped_in, costmap_ros_->getBaseFrameID(), stamped_out);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return false;
  }

  quat.setX(stamped_out.pose.orientation.x);
  quat.setY(stamped_out.pose.orientation.y);
  quat.setZ(stamped_out.pose.orientation.z);
  quat.setW(stamped_out.pose.orientation.w);

  twist_out[0] = stamped_out.pose.position.x;
  twist_out[1] = stamped_out.pose.position.y;
  twist_out[2] = tf::getYaw(quat);
  return true;
}

}
