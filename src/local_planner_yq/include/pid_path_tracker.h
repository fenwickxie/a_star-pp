#ifndef PID_PATH_TRACKER_H
#define PID_PATH_TRACKER_H

#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"

class PidPathTracker
{
public:
  enum class State
  {
    IDLE,
    ALIGNING,
    TRACKING,
    ALIGNING_TO_GOAL,
    GOAL_REACHED,
    FAILED
  };

  PidPathTracker(ros::NodeHandle nh, ros::NodeHandle private_nh);

  void runOnce();

private:
  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  void loadParams();
  void pathCallback(const nav_msgs::PathConstPtr& msg);

  bool getRobotPose(Pose2D& pose);
  bool getLocalGoal(const geometry_msgs::PoseStamped& goal_map, Pose2D& goal_base);
  bool getLookAheadGoal(geometry_msgs::PoseStamped& goal_map);
  bool isGoalReached(double threshold);

  bool handleAligning();
  bool handleTracking();
  bool handleAligningToGoal();

  geometry_msgs::Twist computeCommandMode0(const Pose2D& local_goal) const;
  geometry_msgs::Twist computeCommandMode1(const Pose2D& local_goal) const;
  void stopControl();

  static double wrapAngle(double ang);
  static double clamp(double v, double lo, double hi);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf::TransformListener tf_;

  ros::Subscriber global_path_sub_;
  ros::Publisher cmd_pub_;

  nav_msgs::Path global_path_;
  geometry_msgs::PoseStamped align_goal_;

  bool has_global_path_{false};
  bool publish_vel_{true};
  State state_{State::IDLE};

  std::string world_frame_{"map"};
  std::string base_frame_{"base_link"};
  std::string global_path_topic_{"/global_path"};
  std::string cmd_topic_{"/cmd_vel"};

  double lookahead_distance_{0.8};
  double goal_dist_threshold_{0.10};

  bool align_while_plan_{true};
  double angle_max_{20.0 * M_PI / 180.0};
  double angle_align_{5.0 * M_PI / 180.0};
  double angle_goal_stop_{3.0 * M_PI / 180.0};

  bool enable_backward_tracking_{true};
  double desired_vel_{0.35};
  double max_ang_vel_{0.8};
  double critical_angle_{45.0 * M_PI / 180.0};
  double curvature_scaling_angle_{25.0 * M_PI / 180.0};
  double pow_k_{2.0};
  double min_vel_{0.04};
  double dock_blend_threshold_{0.1};

  int pure_pursuit_mode_{1};

  double goal_yaw_{0.0};
  bool has_goal_yaw_{false};
};

#endif