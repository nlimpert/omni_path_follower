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

    global_frame_ = costmap_ros_->getGlobalFrameID();

    //initialize empty global plan
    std::vector<geometry_msgs::PoseStamped> empty_plan;
    empty_plan.push_back(geometry_msgs::PoseStamped());
    global_plan_ = empty_plan;

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

    // Get robot velocity

    // prune global plan to cut off parts of the past (spatially before the robot)
    pruneGlobalPlan(*tfl_, robot_pose, global_plan_);

    // Transform global plan to the frame of interest (w.r.t. the local costmap)
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    int goal_idx;
    tf::StampedTransform tf_plan_to_global;
    if(!base_local_planner::transformGlobalPlan(*tfl_, global_plan_, robot_pose, *costmap_ros_->getCostmap(), costmap_ros_->getGlobalFrameID(), transformed_plan))
    {
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }
    // check if global goal is reached
    tf::Stamped<tf::Pose> global_goal;
    tf::poseStampedMsgToTF(transformed_plan.back(), global_goal);
    double dx = global_goal.getOrigin().getX() - robot_pose_.x();
    double dy = global_goal.getOrigin().getY() - robot_pose_.y();
    double delta_orient = g2o::normalize_theta( tf::getYaw(global_goal.getRotation()) - robot_pose_.theta() );

    bool xy_tolerance_reached = fabs(std::sqrt(dx*dx+dy*dy)) < goal_threshold_linear_;
    bool ori_tolerance_reached = fabs(delta_orient) < goal_threshold_angular_;

    if(xy_tolerance_reached && ori_tolerance_reached)
    {
      ROS_INFO_NAMED("omni_path_follower", "GOAL Reached!");
      goal_reached_ = true;
      return true;
    }

    // Get current goal point (last point of the transformed plan)
    tf::Stamped<tf::Pose> goal_point, goal_point_2;
    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
    tf::poseStampedMsgToTF(global_plan_.back(), goal_point_2);

    robot_goal_.x() = goal_point.getOrigin().getX();
    robot_goal_.y() = goal_point.getOrigin().getY();

    // Setup potential field method
    unsigned int width = costmap_ros_->getCostmap()->getSizeInCellsX();
    unsigned int height = costmap_ros_->getCostmap()->getSizeInCellsY();
    double cur_world_x = 0.0;
    double cur_world_y = 0.0;
    double cur_robot_x = robot_pose.getOrigin().x();
    double cur_robot_y = robot_pose.getOrigin().y();
    float cur_min_dist = 100000.0;

    double robot_ori = tf::getYaw(robot_pose.getRotation());

    Eigen::Vector2d vec_goalrobot(robot_goal_.x() - robot_pose.getOrigin().getX(),
                                  robot_goal_.y() - robot_pose.getOrigin().getY());

    float rep_x = 0.0, rep_y = 0.0;
    float rep_phi = 0.0;

    float att_x = 0., att_y = 0.;
    float att_phi = 0.0;

    for (int posX = 0; posX < width; ++posX) {
      for (int posY = 0; posY < height; ++posY) {
        costmap_ros_->getCostmap()->mapToWorld(posX, posY, cur_world_x, cur_world_y);
        if (costmap_ros_->getCostmap()->getCost(posX, posY) != costmap_2d::FREE_SPACE) {
          double dx = cur_world_x - cur_robot_x;
          double dy = cur_world_y - cur_robot_y;

          if (fabs(dx) >= 0.01 && fabs(dy) >= 0.01) {
            //Assign a lower factor for objects at larger distances
            float factor = 1.f / ( (dx*dx + dy*dy) * (dx*dx + dy*dy) );

            float d = sqrt( dx * dx + dy * dy );
            if (d < cur_min_dist) {
                cur_min_dist = d;
            }

            // add current obstacle as an repelling point to target
            rep_x -= factor * dx;
            rep_y -= factor * dy;
          }
        }
      }
    }
    rep_phi = angles::normalize_angle(atan2(rep_y, rep_x) - robot_ori);

    // add current goal as an attraction point to target
    att_x = vec_goalrobot[0];
    att_y = vec_goalrobot[1];
    att_phi = angles::normalize_angle(atan2(att_y, att_x) - robot_ori);

    // determine whether we selected the last waypoint in order to scale down the rep force.
    bool last_waypoint_selected_ =
        fabs(robot_goal_.x() - global_plan_.back().pose.position.x) < 0.000001 &&
        fabs(robot_goal_.y() - global_plan_.back().pose.position.y) < 0.000001;

    if (fabs(cur_min_dist) < 1.0 && last_waypoint_selected_ == false) {
      rep_x *= obstacle_k_;
      rep_y *= obstacle_k_;
    } else {
      rep_x = 0.;
      rep_y = 0.;
    }

    att_x *= goal_k_;
    att_y *= goal_k_;

    float target_vel_x = 0.;
    float target_vel_y = 0.;

    float cur_max_vel = max_vel_lin_;

    if (fabs(cur_min_dist) < pot_min_dist_) {
      // consider a minimum feasible velocity
      cur_max_vel = std::max(0.1, cur_max_vel / 2.0);
    }

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
      target_ori = angles::normalize_angle(goal_phi + rotate_from_obstacles_k_ * rep_phi);
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

    // limit angular vel
    if (fabs(target_vel_.angular.z) > 0.00001 ) {
      target_vel_.angular.z = std::min(fabs(target_vel_.angular.z), max_vel_theta_) *
          (target_vel_.angular.z / fabs(target_vel_.angular.z));
    }

    float weight_ang = fabs(target_vel_.angular.z) / max_vel_theta_;
    ROS_INFO_THROTTLE(2, "weight_ang: %f", weight_ang);

    ROS_INFO_THROTTLE(2, "pre:\tx: %f y: %f",
                      target_vel_.linear.x,
                      target_vel_.linear.y);

    cmd_vel.linear.x = calculate_translation(last_vel_.linear.x, target_vel_.linear.x);
    cmd_vel.linear.y = calculate_translation(last_vel_.linear.y, target_vel_.linear.y);
    cmd_vel.angular.z = calculate_rotation(last_vel_.angular.z, target_vel_.angular.z);

    last_vel_.linear.x = cmd_vel.linear.x;
    last_vel_.linear.y = cmd_vel.linear.y;
    last_vel_.angular.z = cmd_vel.angular.z;

    if (visualize_ == true) {
      visualization_msgs::Marker rep_marker, att_marker, result_marker_vel;
      visualization_msgs::MarkerArray markers;

      // add repelling marker
      rep_marker.header.frame_id = "/base_link";
      rep_marker.header.stamp = ros::Time::now();

      rep_marker.type = visualization_msgs::Marker::ARROW;
      rep_marker.action = visualization_msgs::Marker::ADD;
      rep_marker.scale.x = 0.1;
      rep_marker.scale.y = 0.1;
      rep_marker.scale.z = 0.1;
      rep_marker.color.a = 0.5;
      geometry_msgs::Point cur_point;
      rep_marker.id = 0;
      rep_marker.color.r = 1.0;
      rep_marker.color.g = 0.0;
      rep_marker.color.b = 0.0;

      cur_point.x = 0.0;
      cur_point.y = 0.0;
      rep_marker.points.push_back(cur_point);
      cur_point.x = std::cos( rep_phi ) * 5.0;
      cur_point.y = std::sin( rep_phi ) * 5.0;
      rep_marker.points.push_back(cur_point);

      markers.markers.push_back(rep_marker);

      // add attractive marker
      att_marker.header.frame_id = "/map";
      att_marker.header.stamp = ros::Time::now();

      att_marker.type = visualization_msgs::Marker::ARROW;
      att_marker.action = visualization_msgs::Marker::ADD;
      att_marker.scale.x = 0.1;
      att_marker.scale.y = 0.1;
      att_marker.scale.z = 0.1;
      att_marker.color.a = 0.5;

      att_marker.id = 1;
      att_marker.color.r = 0.0;
      att_marker.color.g = 1.0;
      att_marker.color.b = 0.0;

      cur_point.x = robot_pose_.x();
      cur_point.y = robot_pose_.y();
      att_marker.points.push_back(cur_point);
      cur_point.x = robot_goal_.x();
      cur_point.y = robot_goal_.y();
      att_marker.points.push_back(cur_point);

      markers.markers.push_back(att_marker);

      // add result marker
      result_marker_vel.header.frame_id = "/base_link";
      result_marker_vel.header.stamp = ros::Time::now();

      result_marker_vel.type = visualization_msgs::Marker::ARROW;
      result_marker_vel.action = visualization_msgs::Marker::ADD;
      result_marker_vel.scale.x = 0.1;
      result_marker_vel.scale.y = 0.1;
      result_marker_vel.scale.z = 0.1;
      result_marker_vel.color.a = 1;

      result_marker_vel.id = 2;
      result_marker_vel.color.r = 0.0;
      result_marker_vel.color.g = 0.0;
      result_marker_vel.color.b = 1.0;

      cur_point.x = 0.0;
      cur_point.y = 0.0;
      result_marker_vel.points.push_back(cur_point);
      cur_point.x = target_vel_.linear.x;
      cur_point.y = target_vel_.linear.y;
      result_marker_vel.points.push_back(cur_point);

      markers.markers.push_back(result_marker_vel);

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

}
