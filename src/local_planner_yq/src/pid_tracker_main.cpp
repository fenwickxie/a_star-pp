#include "pid_path_tracker.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pid_path_tracker");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  PidPathTracker tracker(nh, private_nh);

  double loop_rate = 30.0;
  private_nh.param<double>("loop_rate", loop_rate, loop_rate);
  ros::Rate rate(loop_rate);

  while (ros::ok()) {
    tracker.runOnce();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}