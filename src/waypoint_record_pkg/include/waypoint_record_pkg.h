#ifndef WAYPOINT_RECORD_PKG_H
#define WAYPOINT_RECORD_PKG_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include <signal.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <unistd.h>
#include <fstream>
#include "jsoncpp/json/json.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>
#include "tf/transform_listener.h"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/time.h>
#include <ros/duration.h>
#include <time.h>
#include <sys/time.h>

#include <vector>
#include <deque>
#include<cmath>
#include <thread>

#include <signal.h>
#include <thread>


class PathRecord {

public:
 PathRecord() = default;
 PathRecord(ros::NodeHandle nh, ros::NodeHandle private_nh);

 void PoseCallback(const nav_msgs::Odometry::ConstPtr& msg);

 void GeomPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

 void ctrl_thread(void);

private:


ros::NodeHandle nh_;
ros::NodeHandle private_nh_;

ros::Subscriber pose_sub_;
ros::Subscriber geo_pose_sub_;

ros::Publisher path_pub_;

std::ofstream csvfile_;

double record_therold_;

std::string pose_topic_ = "hello";
std::string geopose_topic_ =  " ";
std::string path_file_;
std::string world_frame_ = "/map";

nav_msgs::Odometry robot_pose_;

nav_msgs::Path path_;




};




















#endif