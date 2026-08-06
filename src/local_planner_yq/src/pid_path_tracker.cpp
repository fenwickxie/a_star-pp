#include "pid_path_tracker.h"

#include <algorithm>

PidPathTracker::PidPathTracker(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : nh_(nh), private_nh_(private_nh)
{
  loadParams();
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 10);
  global_path_sub_ = nh_.subscribe(global_path_topic_, 1, &PidPathTracker::pathCallback, this);
}

void PidPathTracker::loadParams()
{
  private_nh_.param<std::string>("world_frame", world_frame_, world_frame_);
  private_nh_.param<std::string>("base_frame", base_frame_, base_frame_);
  private_nh_.param<std::string>("global_path_topic", global_path_topic_, global_path_topic_);
  private_nh_.param<std::string>("cmd_topic", cmd_topic_, cmd_topic_);

  private_nh_.param<bool>("publish_vel", publish_vel_, publish_vel_);
  private_nh_.param<double>("lookahead_distance", lookahead_distance_, lookahead_distance_);
  private_nh_.param<double>("goal_dist_threshold", goal_dist_threshold_, goal_dist_threshold_);

  private_nh_.param<bool>("align_while_plan", align_while_plan_, align_while_plan_);

  double angle_max_deg = angle_max_ * 180.0 / M_PI;
  double angle_align_deg = angle_align_ * 180.0 / M_PI;
  double angle_goal_stop_deg = angle_goal_stop_ * 180.0 / M_PI;
  private_nh_.param<double>("angle_max_deg", angle_max_deg, angle_max_deg);
  private_nh_.param<double>("angle_align_deg", angle_align_deg, angle_align_deg);
  private_nh_.param<double>("angle_goal_stop_deg", angle_goal_stop_deg, angle_goal_stop_deg);
  angle_max_ = angle_max_deg * M_PI / 180.0;
  angle_align_ = angle_align_deg * M_PI / 180.0;
  angle_goal_stop_ = angle_goal_stop_deg * M_PI / 180.0;

  private_nh_.param<bool>("enable_backward_tracking", enable_backward_tracking_, enable_backward_tracking_);
  private_nh_.param<double>("desired_vel", desired_vel_, desired_vel_);
  private_nh_.param<double>("max_ang_vel", max_ang_vel_, max_ang_vel_);

  double critical_angle_deg = critical_angle_ * 180.0 / M_PI;
  double curvature_scaling_angle_deg = curvature_scaling_angle_ * 180.0 / M_PI;
  private_nh_.param<double>("critical_angle_deg", critical_angle_deg, critical_angle_deg);
  private_nh_.param<double>("curvature_scaling_angle_deg", curvature_scaling_angle_deg, curvature_scaling_angle_deg);
  critical_angle_ = critical_angle_deg * M_PI / 180.0;
  curvature_scaling_angle_ = curvature_scaling_angle_deg * M_PI / 180.0;

  private_nh_.param<double>("pow_k", pow_k_, pow_k_);
  private_nh_.param<double>("min_vel", min_vel_, min_vel_);
  private_nh_.param<double>("dock_blend_threshold", dock_blend_threshold_, dock_blend_threshold_);

  private_nh_.param<int>("pure_pursuit_mode", pure_pursuit_mode_, pure_pursuit_mode_);

  ROS_INFO("[pid_path_tracker] world_frame=%s, base_frame=%s", world_frame_.c_str(), base_frame_.c_str());
  ROS_INFO("[pid_path_tracker] lookahead=%.2f, desired_vel=%.2f, mode=%d", lookahead_distance_, desired_vel_, pure_pursuit_mode_);
}

void PidPathTracker::pathCallback(const nav_msgs::PathConstPtr& msg)
{
  if (!msg || msg->poses.empty()) {
    ROS_WARN("[pid_path_tracker] received empty path");
    return;
  }

  global_path_ = *msg;
  has_global_path_ = true;

  align_goal_ = global_path_.poses.back();

  has_goal_yaw_ = false;
  const auto& q = align_goal_.pose.orientation;
  const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (q_norm > 1e-3) {
    goal_yaw_ = tf::getYaw(q);
    has_goal_yaw_ = true;
  } else {
    // Compatible with existing a_star_test path format where goal yaw is encoded in pose.position.z.
    goal_yaw_ = align_goal_.pose.position.z;
    has_goal_yaw_ = true;
  }

  state_ = State::ALIGNING;
  ROS_INFO("[pid_path_tracker] new path accepted, points=%zu", global_path_.poses.size());
}

double PidPathTracker::wrapAngle(double ang)
{
  while (ang > M_PI) {
    ang -= 2.0 * M_PI;
  }
  while (ang < -M_PI) {
    ang += 2.0 * M_PI;
  }
  return ang;
}

double PidPathTracker::clamp(double v, double lo, double hi)
{
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

bool PidPathTracker::getRobotPose(Pose2D& pose)
{
  tf::StampedTransform transform;
  try {
    tf_.waitForTransform(world_frame_, base_frame_, ros::Time(0), ros::Duration(0.2));
    tf_.lookupTransform(world_frame_, base_frame_, ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN_THROTTLE(1.0, "[pid_path_tracker] TF error: %s", ex.what());
    return false;
  }

  pose.x = transform.getOrigin().x();
  pose.y = transform.getOrigin().y();
  pose.yaw = tf::getYaw(transform.getRotation());
  return true;
}

bool PidPathTracker::getLocalGoal(const geometry_msgs::PoseStamped& goal_map, Pose2D& goal_base)
{
  Pose2D robot;
  if (!getRobotPose(robot)) {
    return false;
  }

  const double dx = goal_map.pose.position.x - robot.x;
  const double dy = goal_map.pose.position.y - robot.y;
  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);

  goal_base.x = c * dx + s * dy;
  goal_base.y = -s * dx + c * dy;
  goal_base.yaw = wrapAngle(goal_yaw_ - robot.yaw);
  return true;
}

bool PidPathTracker::getLookAheadGoal(geometry_msgs::PoseStamped& goal_map)
{
  if (!has_global_path_ || global_path_.poses.empty()) {
    return false;
  }

  Pose2D robot;
  if (!getRobotPose(robot)) {
    return false;
  }

  int closest_idx = 0;
  double min_dist = 1e9;
  for (size_t i = 0; i < global_path_.poses.size(); ++i) {
    const auto& p = global_path_.poses[i].pose.position;
    const double d = std::hypot(p.x - robot.x, p.y - robot.y);
    if (d < min_dist) {
      min_dist = d;
      closest_idx = static_cast<int>(i);
    }
  }

  for (size_t i = static_cast<size_t>(closest_idx + 1); i < global_path_.poses.size(); ++i) {
    const auto& p = global_path_.poses[i].pose.position;
    const double d = std::hypot(p.x - robot.x, p.y - robot.y);
    if (d >= lookahead_distance_) {
      goal_map = global_path_.poses[i];
      return true;
    }
  }

  goal_map = global_path_.poses.back();
  return true;
}

bool PidPathTracker::isGoalReached(double threshold)
{
  if (!has_global_path_ || global_path_.poses.empty()) {
    return false;
  }

  Pose2D goal_base;
  if (!getLocalGoal(global_path_.poses.back(), goal_base)) {
    return false;
  }
  return std::hypot(goal_base.x, goal_base.y) < threshold;
}

geometry_msgs::Twist PidPathTracker::computeCommandMode0(const Pose2D& local_goal) const
{
  geometry_msgs::Twist cmd;
  const double x = local_goal.x;
  const double y = local_goal.y;
  const double dist = std::hypot(x, y);
  const double dist_sq = x * x + y * y;

  double v_raw = desired_vel_;
  if (lookahead_distance_ > 1e-6) {
    v_raw = desired_vel_ * dist / lookahead_distance_;
  }
  const double abs_vel = clamp(v_raw, 0.0, desired_vel_);

  if (!enable_backward_tracking_) {
    if (x < 0.0) {
      cmd.linear.x = 0.0;
      const double ang = std::atan2(y, x);
      cmd.angular.z = (ang > 0.0) ? 0.4 : -0.4;
      return cmd;
    }
    cmd.linear.x = abs_vel;
  } else {
    cmd.linear.x = (x >= 0.0) ? abs_vel : -abs_vel;
  }

  if (dist_sq < 1e-5) {
    cmd.angular.z = 0.0;
  } else {
    cmd.angular.z = 2.0 * cmd.linear.x * y / dist_sq;
    cmd.angular.z = clamp(cmd.angular.z, -max_ang_vel_, max_ang_vel_);
  }
  return cmd;
}

geometry_msgs::Twist PidPathTracker::computeCommandMode1(const Pose2D& local_goal) const
{
  geometry_msgs::Twist cmd;
  const double x = local_goal.x;
  const double y = local_goal.y;
  const double L = std::hypot(x, y);

  int lin_vel_sign = (x >= 0.0) ? 1 : -1;
  double delta_theta = std::atan2(y, x);

  if (lin_vel_sign < 0 && enable_backward_tracking_) {
    if (delta_theta > 0.0) {
      delta_theta -= M_PI;
    } else {
      delta_theta += M_PI;
    }
  }

  const int ang_vel_sign = (delta_theta >= 0.0) ? 1 : -1;
  const double abs_delta = std::fabs(delta_theta);
  const double radius = (L * L) / (2.0 * (std::fabs(y) + 1e-4));
  const double radius_min = L / (2.0 * std::sin(curvature_scaling_angle_));

  if (abs_delta >= critical_angle_ && L > dock_blend_threshold_) {
    cmd.linear.x = 0.0;
    cmd.angular.z = ang_vel_sign * max_ang_vel_;
  } else if (abs_delta >= curvature_scaling_angle_ && L > dock_blend_threshold_) {
    cmd.linear.x = lin_vel_sign * desired_vel_ * (radius / radius_min);
    cmd.angular.z = ang_vel_sign * std::fabs(cmd.linear.x) / radius;
    cmd.angular.z = clamp(cmd.angular.z, -max_ang_vel_, max_ang_vel_);
  } else {
    cmd.linear.x = lin_vel_sign * desired_vel_;
    cmd.angular.z = ang_vel_sign * std::fabs(cmd.linear.x) / radius;
    cmd.angular.z = clamp(cmd.angular.z, -max_ang_vel_, max_ang_vel_);
  }

  const double ratio = clamp(L / lookahead_distance_, 0.0, 1.0);
  const double vel_scaled = cmd.linear.x * std::pow(ratio, pow_k_);
  if (vel_scaled > 0.0) {
    cmd.linear.x = std::max(vel_scaled, min_vel_);
  } else if (vel_scaled < 0.0) {
    cmd.linear.x = std::min(vel_scaled, -min_vel_);
  } else {
    cmd.linear.x = 0.0;
  }

  if (L < dock_blend_threshold_) {
    cmd.angular.z = 0.0;
  }
  return cmd;
}

bool PidPathTracker::handleAligning()
{
  Pose2D local_goal;
  if (!getLocalGoal(align_goal_, local_goal)) {
    state_ = State::FAILED;
    return false;
  }

  double angle_diff = std::atan2(local_goal.y, local_goal.x);
  if (enable_backward_tracking_ && local_goal.x < 0.0) {
    if (angle_diff > 0.0) {
      angle_diff -= M_PI;
    } else {
      angle_diff += M_PI;
    }
  }

  const double abs_err = std::fabs(angle_diff);
  if (abs_err < angle_align_) {
    state_ = State::TRACKING;
    return true;
  }

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z = (angle_diff > 0.0) ? max_ang_vel_ : -max_ang_vel_;

  if (!publish_vel_) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
  }
  cmd_pub_.publish(cmd);
  return true;
}

bool PidPathTracker::handleTracking()
{
  if (isGoalReached(goal_dist_threshold_)) {
    state_ = State::ALIGNING_TO_GOAL;
    stopControl();
    return true;
  }

  if (!getLookAheadGoal(align_goal_)) {
    state_ = State::FAILED;
    return false;
  }

  Pose2D local_goal;
  if (!getLocalGoal(align_goal_, local_goal)) {
    state_ = State::FAILED;
    return false;
  }

  if (align_while_plan_) {
    double angle_diff = std::atan2(local_goal.y, local_goal.x);
    if (enable_backward_tracking_ && local_goal.x < 0.0) {
      if (angle_diff > 0.0) {
        angle_diff -= M_PI;
      } else {
        angle_diff += M_PI;
      }
    }

    if (std::fabs(angle_diff) > angle_max_) {
      state_ = State::ALIGNING;
      return true;
    }
  }

  geometry_msgs::Twist cmd;
  if (pure_pursuit_mode_ == 1) {
    cmd = computeCommandMode1(local_goal);
  } else {
    cmd = computeCommandMode0(local_goal);
  }

  if (!publish_vel_) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
  }
  cmd_pub_.publish(cmd);
  return true;
}

bool PidPathTracker::handleAligningToGoal()
{
  if (!has_goal_yaw_) {
    state_ = State::GOAL_REACHED;
    stopControl();
    return true;
  }

  Pose2D robot;
  if (!getRobotPose(robot)) {
    state_ = State::FAILED;
    return false;
  }

  const double yaw_err = wrapAngle(goal_yaw_ - robot.yaw);
  if (std::fabs(yaw_err) < angle_goal_stop_) {
    stopControl();
    state_ = State::GOAL_REACHED;
    ROS_INFO_THROTTLE(1.0, "[pid_path_tracker] goal reached");
    return true;
  }

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z = (yaw_err > 0.0) ? max_ang_vel_ * 0.6 : -max_ang_vel_ * 0.6;

  if (!publish_vel_) {
    cmd.angular.z = 0.0;
  }
  cmd_pub_.publish(cmd);
  return true;
}

void PidPathTracker::stopControl()
{
  geometry_msgs::Twist stop;
  stop.linear.x = 0.0;
  stop.angular.z = 0.0;
  cmd_pub_.publish(stop);
}

void PidPathTracker::runOnce()
{
  if (!has_global_path_) {
    return;
  }

  switch (state_) {
    case State::ALIGNING:
      handleAligning();
      break;
    case State::TRACKING:
      handleTracking();
      break;
    case State::ALIGNING_TO_GOAL:
      handleAligningToGoal();
      break;
    case State::GOAL_REACHED:
      stopControl();
      break;
    case State::FAILED:
    case State::IDLE:
    default:
      stopControl();
      break;
  }
}