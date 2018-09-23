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

//  bool PathFollower::posesEqual(geometry_msgs::PoseStamped first, geometry_msgs::PoseStamped second)
//  {
//    //    first.pose.position - second.pose.position;
//    if(fabs(first.pose.position.x - second.pose.position.x) < EPSILON &&
//       fabs(first.pose.position.y - second.pose.position.y) < EPSILON &&
//       fabs(first.pose.position.z - second.pose.position.z) < EPSILON &&
//       fabs(first.pose.orientation.x - second.pose.orientation.x) < EPSILON &&
//       fabs(first.pose.orientation.y - second.pose.orientation.y) < EPSILON &&
//       fabs(first.pose.orientation.z - second.pose.orientation.z) < EPSILON &&
//       fabs(first.pose.orientation.w - second.pose.orientation.w) < EPSILON)
//      return true;

//    return false;
//  }

  void PathFollower::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
  {
    tfl_ = tf;
    costmap_ros_ = costmap_ros;
    ros::NodeHandle nh("~/" + name);

//    in_path_vel_ = 0.2;
//    to_path_k_ = 0.5;
    angle_k_ = 1.5;
    goal_threshold_linear_ = 0.05;
    max_vel_lin_ = 0.;
    max_vel_lin_at_goal_ = 0.;
    rot_to_goal_pose_dist_ = 0.;
    pot_min_dist_ = 0.;
    goal_k_ = 0.;
    acc_lin_inc_ = 0.;
    acc_theta_inc_ = 0.;

//    rotate_to_path_ = true;
//    rotate_at_start_ = false;
    global_frame_ = costmap_ros_->getGlobalFrameID();

//    //initialize empty global plan
//    std::vector<geometry_msgs::PoseStamped> empty_plan;
//    empty_plan.push_back(geometry_msgs::PoseStamped());
//    global_plan_ = empty_plan;

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
      ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
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
//    tf::Stamped<tf::Pose> robot_vel_tf;
//    odom_helper_.getRobotVel(robot_vel_tf);
//    robot_vel_.linear.x = robot_vel_tf.getOrigin().getX();
//    robot_vel_.linear.y = robot_vel_tf.getOrigin().getY();
//    robot_vel_.angular.z = tf::getYaw(robot_vel_tf.getRotation());

    // prune global plan to cut off parts of the past (spatially before the robot)
    pruneGlobalPlan(*tfl_, robot_pose, global_plan_);

    // Transform global plan to the frame of interest (w.r.t. the local costmap)
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    int goal_idx;
    tf::StampedTransform tf_plan_to_global;
    if (!transformGlobalPlan(*tfl_, global_plan_, robot_pose, *costmap_ros_->getCostmap(), global_frame_, 0.5,
                             transformed_plan, &goal_idx, &tf_plan_to_global))
    {
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    // check if global goal is reached
    tf::Stamped<tf::Pose> global_goal;
    tf::poseStampedMsgToTF(global_plan_.back(), global_goal);
    global_goal.setData( tf_plan_to_global * global_goal );
    double dx = global_goal.getOrigin().getX() - robot_pose_.x();
    double dy = global_goal.getOrigin().getY() - robot_pose_.y();
    double delta_orient = g2o::normalize_theta( tf::getYaw(global_goal.getRotation()) - robot_pose_.theta() );
    if(fabs(std::sqrt(dx*dx+dy*dy)) < goal_threshold_linear_
      && fabs(delta_orient) < goal_threshold_angular_)
    {
      ROS_INFO_NAMED("omni_path_follower", "GOAL Reached!");
      goal_reached_ = true;
      return true;
    }

    // Get current goal point (last point of the transformed plan)
    tf::Stamped<tf::Pose> goal_point;
    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
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
//    float cur_min_x = 0.;
//    float cur_min_y = 0.;

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
//                cur_min_x = dx;
//                cur_min_y = dy;
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

    if (fabs(cur_min_dist) < 0.3 && last_waypoint_selected_ == false) {
//      float factor = std::min(1. / fabs(cur_min_dist - 0.3), 0.5);
//      ROS_INFO("factor: %f", factor);
      rep_x *= obstacle_k_;
      rep_y *= obstacle_k_;
//      rep_x *= factor;
//      rep_y *= factor;
//      cmd_vel.linear.x += drive_part_x * factor;
//      cmd_vel.linear.y += drive_part_y * factor;
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
      rep_x *= cutoff_factor_at_goal_;
      rep_y *= cutoff_factor_at_goal_;
    } else {
      target_vel_x = cur_max_vel;
      target_vel_y = cur_max_vel;
    }

//    ROS_INFO("rep_phi: %f\tatt_phi: %f", rep_phi, att_phi);

    double goal_phi = angles::shortest_angular_distance(robot_ori, atan2(vec_goalrobot[1], vec_goalrobot[0]));

    float target_ori = 0.;

    // rotate to goal pose when we are getting close
    if (fabs(dx) < rot_to_goal_pose_dist_ && fabs(dy) < rot_to_goal_pose_dist_) {
      target_ori = angles::shortest_angular_distance(robot_ori,tf::getYaw(global_goal.getRotation()));
//    } else if (true) {
//      new_ori = target_phi;
    } else {
      target_ori = angles::normalize_angle(goal_phi + rotate_from_obstacles_k_ * rep_phi);
    }

    float drive_x = att_x + rep_x;
    float drive_y = att_y + rep_y;
    float drive_phi = angles::normalize_angle(atan2(drive_y, drive_x) - robot_ori);

    float drive_part_x = 0.f;
    float drive_part_y = 0.f;

    drive_part_x = std::cos( drive_phi );
    drive_part_y = std::sin( drive_phi );

    double d_phi = angles::shortest_angular_distance(rep_phi, goal_phi);

//    ROS_INFO("vec_nextrob: %f\t%f", vec_nextrob[0], vec_nextrob[1]);
//    ROS_INFO("goal_phi: %f", goal_phi);
//    ROS_INFO("target_phi: %f", target_phi);
//    ROS_INFO("d_phi: %f min_dist: %f", d_phi, cur_min_dist);
//    ROS_INFO("proposed: %f\t%f", drive_part_x * 0.2, drive_part_y * 0.2);
//    ROS_INFO("");

    //rotate velocity into robot frame



//    float vel_x = fabs(dx) > 0.5 ? 0.5 : std::max(fabs(dx), 0.1);
//    float vel_y = fabs(dy) > 0.5 ? 0.5 : std::max(fabs(dy), 0.1);

//    ROS_INFO("vel: %f %f", vel_x, vel_y);
//    ROS_INFO("d: %f %f", dx, dy);


//    cmd_vel.linear.x = cos(goal_phi) * vel_x;
//    cmd_vel.linear.y = sin(goal_phi) * vel_y;

//    if (fabs(cur_min_dist) < 0.5 && last_waypoint_selected_ == false) {
//      float factor = std::min(1. / fabs(cur_min_dist - 0.3), 0.5);
//      ROS_INFO("factor: %f", factor);
//      cmd_vel.linear.x += drive_part_x * factor;
//      cmd_vel.linear.y += drive_part_y * factor;
//    }

    target_vel_.linear.x = drive_part_x * target_vel_x;
    target_vel_.linear.y = drive_part_y * target_vel_y;
    target_vel_.angular.z = angle_k_ * target_ori;

    // limit angular vel
    if (fabs(target_vel_.angular.z) > 0.00001 ) {
      target_vel_.angular.z = std::min(fabs(target_vel_.angular.z), max_vel_theta_) *
          (target_vel_.angular.z / fabs(target_vel_.angular.z));
    }

//    float d_vel_lin_x = new_robot_vel_.linear.x - last_robot_vel_.linear.x;
//    float d_vel_lin_y = new_robot_vel_.linear.y - last_robot_vel_.linear.y;
//    float d_vel_theta = new_robot_vel_.angular.z - last_robot_vel_.angular.z;

//    ROS_INFO("d: %f\t%f\t%f", d_vel_lin_x, d_vel_lin_y, d_vel_theta);

//    // consider acceleration limits
//    if (fabs(d_vel_lin_x) > acc_lim_lin_ * loop_time_) {
//      cmd_vel.linear.x = last_robot_vel_.linear.x + (acc_lim_lin_ * loop_time_ * sign(d_vel_lin_x));
//    } else {
//      cmd_vel.linear.x = new_robot_vel_.linear.x;
//    }
//    if (fabs(d_vel_lin_y) > acc_lim_lin_ * loop_time_) {
//      cmd_vel.linear.y = last_robot_vel_.linear.y + (acc_lim_lin_ * loop_time_ * sign(d_vel_lin_y));
//    } else {
//      cmd_vel.linear.y = new_robot_vel_.linear.y;
//    }

//    if (fabs(d_vel_theta) > acc_lim_theta_ * loop_time_) {
//      cmd_vel.angular.z = last_robot_vel_.angular.z + (acc_lim_theta_ * loop_time_ * sign(d_vel_theta));
//    } else {
//      cmd_vel.angular.z = new_robot_vel_.angular.z;
//    }

//    last_robot_vel_.linear.x = cmd_vel.linear.x;
//    last_robot_vel_.linear.y = cmd_vel.linear.y;
//    last_robot_vel_.angular.z = cmd_vel.angular.z;

//    float d_v_x     = target_vel_.linear.x  - last_vel_.linear.x;
//    float d_v_y     = target_vel_.linear.y  - last_vel_.linear.y;
//    float d_v_theta = target_vel_.angular.z - last_vel_.angular.z;

    //    cmd_vel.linear.x = drive_part_x * target_vel_x;
    //    cmd_vel.linear.y = drive_part_y * target_vel_y;
    //    cmd_vel.angular.z = angle_k_ * target_ori;

    ROS_INFO("X");
    cmd_vel.linear.x = calculate_translation(last_vel_.linear.x, target_vel_.linear.x);

    ROS_INFO("Y");
    cmd_vel.linear.y = calculate_translation(last_vel_.linear.y, target_vel_.linear.y);

    ROS_INFO("ang_Z");
    cmd_vel.angular.z = calculate_rotation(last_vel_.angular.z, target_vel_.angular.z);


    ROS_INFO("DONE CALCULATE_TRANSLATION");

    last_vel_.linear.x = cmd_vel.linear.x;
    last_vel_.linear.y = cmd_vel.linear.y;
    last_vel_.angular.z = cmd_vel.angular.z;


    if (visualize_ == true) {
      visualization_msgs::Marker rep_marker, att_marker;
      visualization_msgs::MarkerArray markers;

      rep_marker.header.frame_id = "/base_link";
      rep_marker.header.stamp = ros::Time::now();

      rep_marker.type = visualization_msgs::Marker::ARROW;
      rep_marker.action = visualization_msgs::Marker::ADD;
      rep_marker.scale.x = 0.1;
      rep_marker.scale.y = 0.1;
      rep_marker.scale.z = 0.1;
      rep_marker.color.a = 1.0;
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

      att_marker.header.frame_id = "/base_link";
      att_marker.header.stamp = ros::Time::now();

      att_marker.type = visualization_msgs::Marker::ARROW;
      att_marker.action = visualization_msgs::Marker::ADD;
      att_marker.scale.x = 0.1;
      att_marker.scale.y = 0.1;
      att_marker.scale.z = 0.1;
      att_marker.color.a = 1.0;

      att_marker.id = 1;
      att_marker.color.r = 0.0;
      att_marker.color.g = 1.0;
      att_marker.color.b = 0.0;

      cur_point.x = 0.0;
      cur_point.y = 0.0;
      att_marker.points.push_back(cur_point);
      cur_point.x = std::cos( att_phi ) * 5.0;
      cur_point.y = std::sin( att_phi ) * 5.0;
      att_marker.points.push_back(cur_point);

      markers.markers.push_back(att_marker);

//      goal_marker.header.frame_id = "/map";
//      goal_marker.header.stamp = ros::Time::now();

//      goal_marker.type = visualization_msgs::Marker::ARROW;
//      goal_marker.action = visualization_msgs::Marker::ADD;
//      goal_marker.scale.x = 0.1;
//      goal_marker.scale.y = 0.1;
//      goal_marker.scale.z = 0.1;
//      goal_marker.color.a = 1.0; // Don't forget to set the alpha!

//      goal_marker.id = 2;
//      goal_marker.color.r = 0.0;
//      goal_marker.color.g = 0.0;
//      goal_marker.color.b = 1.0;

//      cur_point.x = robot_goal_.x();
//      cur_point.y = robot_goal_.y();
//      goal_marker.points.push_back(cur_point);

//      float cur_ori = 0.;
//      if (fabs(dx) < rot_to_goal_pose_dist_ && fabs(dy) < rot_to_goal_pose_dist_) {
//        cur_ori = tf::getYaw(global_goal.getRotation());
//      } else {
//        cur_ori = angles::normalize_angle(goal_phi + rotate_from_obstacles_k_ * rep_phi);
//      }
//      cur_point.x = robot_goal_.x() + std::cos( cur_ori );
//      cur_point.y = robot_goal_.y() + std::sin( cur_ori );
//      goal_marker.points.push_back(cur_point);

//      markers.markers.push_back(goal_marker);

      marker_pub.publish(markers);
    }


    // TODO: Reached goal
//    if(fabs(std::sqrt(dx*dx+dy*dy)) < cfg_.goal_tolerance.xy_goal_tolerance
//      && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance)
//    {
//      goal_reached_ = true;
//      return true;
//    }

//    Eigen::Vector2d vec_nextrob(robot_pose.getOrigin().getX() - next_waypoint_.position.x,
//                                robot_pose.getOrigin().getY() - next_waypoint_.position.y);

//    double robot_angle = tf::getYaw(robot_pose.getRotation());
//    double path_angle = atan2(vec_lastnext[1],vec_lastnext[0]);
//    double delta_angle = angles::shortest_angular_distance(path_angle,robot_angle);

//    double len_lastnext = vec_lastnext.norm();



//    ROS_INFO("desired: %f\t%f", vec_nextrob[0], vec_nextrob[1]);
//    ROS_INFO("target:  %f\t%f", target_x, target_y);

//    //shortest distance from robot to path
//    double cross = vec_lastnext[0]*vec_lastrob[1] - vec_lastnext[1]*vec_lastrob[0];
//    double to_path_dist = cross/len_lastnext;   //TODO norm = 0?!

//    //distance along path between last and next waypoint
//    double along_path_dist = len_lastnext - vec_lastnext.dot(vec_lastrob)/len_lastnext;

//    double to_path_vel = - to_path_k_ * to_path_dist;
//    double in_path_vel = in_path_vel_;

//    //if we are in the last path segment, slow down
//    if(path_index_ == (path_length_ - 2))
//    {
//      in_path_vel = in_path_vel_ / len_lastnext * along_path_dist;

//      double path_dist = hypot(to_path_dist, along_path_dist);

//      if(path_dist < goal_threshold_)
//      {
//        cmd_vel = zero_vel;
//        goal_reached_ = true;
//        return true;
//      }
//    }

//    //rotate velocity into robot frame
//    cmd_vel.linear.x = cos(delta_angle)*in_path_vel + sin(delta_angle)*to_path_vel;
//    cmd_vel.linear.y = -sin(delta_angle)*in_path_vel + cos(delta_angle)*to_path_vel;
//    cmd_vel.angular.z = -angle_k_ * delta_angle;

//    //check if we need to switch to the next path segment
//    double dot = (-1*vec_lastnext).dot(vec_nextrob);
//    if(dot < 0 && path_index_ < (path_length_ - 2))
//    {
//      ROS_DEBUG("path_follower: next waypoint");
//      path_index_ += 1;
//      last_waypoint_ = global_plan_.at(path_index_).pose;
//      next_waypoint_ = global_plan_.at(path_index_+1).pose;
//    }

    return true;
  }

  bool PathFollower::transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                    const tf::Stamped<tf::Pose>& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                    std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, tf::StampedTransform* tf_plan_to_global) const
  {
    // this method is a slightly modified version of base_local_planner/goal_functions.h

    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    transformed_plan.clear();

    try
    {
      if (global_plan.empty())
      {
        ROS_ERROR("Received plan with zero length");
        *current_goal_idx = 0;
        return false;
      }

      // get plan_to_global_transform from plan frame to global_frame
      tf::StampedTransform plan_to_global_transform;
      tf.waitForTransform(global_frame, ros::Time::now(),
      plan_pose.header.frame_id, plan_pose.header.stamp,
      plan_pose.header.frame_id, ros::Duration(0.5));
      tf.lookupTransform(global_frame, ros::Time(),
      plan_pose.header.frame_id, plan_pose.header.stamp,
      plan_pose.header.frame_id, plan_to_global_transform);

      //let's get the pose of the robot in the frame of the plan
      tf::Stamped<tf::Pose> robot_pose;
      tf.transformPose(plan_pose.header.frame_id, global_pose, robot_pose);

      //we'll discard points on the plan that are outside the local costmap
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                       costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
      dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                             // located on the border of the local costmap


      int i = 0;
      double sq_dist_threshold = dist_threshold * dist_threshold;
      double sq_dist = 1e10;

      //we need to loop to a point on the plan that is within a certain distance of the robot
      while(i < (int)global_plan.size())
      {
        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
        if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold) // find first distance that is greater
        {
          sq_dist = new_sq_dist;
          break;
        }
        sq_dist = new_sq_dist;
        ++i;
      }

      tf::Stamped<tf::Pose> tf_pose;
      geometry_msgs::PoseStamped newer_pose;

      double plan_length = 0; // check cumulative Euclidean distance along the plan

      //now we'll transform until points are outside of our distance threshold
      while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
      {
        const geometry_msgs::PoseStamped& pose = global_plan[i];
        tf::poseStampedMsgToTF(pose, tf_pose);
        tf_pose.setData(plan_to_global_transform * tf_pose);
        tf_pose.stamp_ = plan_to_global_transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        tf::poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);

        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;

        // caclulate distance to previous pose
        if (i>0 && max_plan_length>0)
          plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);

        ++i;
      }

      // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
      // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
      if (transformed_plan.empty())
      {
        tf::poseStampedMsgToTF(global_plan.back(), tf_pose);
        tf_pose.setData(plan_to_global_transform * tf_pose);
        tf_pose.stamp_ = plan_to_global_transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        tf::poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);

        // Return the index of the current goal point (inside the distance threshold)
        if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
      }
      else
      {
        // Return the index of the current goal point (inside the distance threshold)
        if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
      }

      // Return the transformation from the global plan to the global planning frame if desired
      if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
    }
    catch(tf::LookupException& ex)
    {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex)
    {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex)
    {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (global_plan.size() > 0)
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
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
    max_vel_lin_at_goal_     = config.max_vel_lin_at_goal;
    max_vel_theta_           = config.max_vel_theta;
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

    acc_lin_inc_   = acc_lim_lin_ * loop_time_;
    acc_theta_inc_ = acc_lim_theta_ * loop_time_;
    acc_lin_dec_   = acc_lim_lin_ * loop_time_;
    acc_theta_dec_ = acc_lim_theta_ * loop_time_;
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
//    last_robot_vel_.linear.x = 0.;
//    last_robot_vel_.linear.y = 0.;
//    last_robot_vel_.angular.z = 0.;
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

    ROS_INFO("call: %f %f %f %f %f", current, desired, acc_lin_inc_, acc_lin_dec_, loop_time_);

    if (desired < current) {

      if (current > 0.0) {
        // decrease forward speed
        ROS_INFO("%i", __LINE__);

        exec_trans = current - acc_lin_dec_;
        exec_trans = std::max( exec_trans, desired );

      } else if (current < 0.0) {
        // increase backward speed
        ROS_INFO("%i", __LINE__);

        exec_trans = current - acc_lin_inc_;
        exec_trans = std::max( exec_trans, desired );

      }  else {
        // current == 0;
        ROS_INFO("%i", __LINE__);

        exec_trans = std::max( -acc_lin_inc_, desired );
      }

    } else if (desired > current) {

      if (current > 0.0) {
        // increase forward speed
        ROS_INFO("%i", __LINE__);

        exec_trans = current + acc_lin_inc_;
        exec_trans = std::min( exec_trans, desired );

      } else if (current < 0.0) {
        // decrease backward speed
        ROS_INFO("%i", __LINE__);

        exec_trans = current + acc_lin_dec_;
        exec_trans = std::min( exec_trans, desired );

      } else {
        // current == 0
        ROS_INFO("%i", __LINE__);

        exec_trans = std::min( acc_lin_inc_, desired );
        ROS_INFO("acc_lin_inc_, desired: %f %f", acc_lin_inc_, desired);
      }

    } else {
      // nothing to change!!!
      exec_trans = desired;
    }

    ROS_INFO("return %f", exec_trans);
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

    return exec_rot*loop_time_;
  }

}
