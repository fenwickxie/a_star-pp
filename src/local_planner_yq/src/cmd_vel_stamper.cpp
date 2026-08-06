#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"

class CmdVelStamper
{
public:
  CmdVelStamper(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
  {
    private_nh.param<std::string>("input_topic", input_topic_, std::string("/cmd_vel_unstamped"));
    private_nh.param<std::string>("output_topic", output_topic_, std::string("/cmd_vel"));
    private_nh.param<std::string>("frame_id", frame_id_, std::string("base_link"));

    sub_ = nh.subscribe(input_topic_, 20, &CmdVelStamper::twistCallback, this);
    pub_ = nh.advertise<geometry_msgs::TwistStamped>(output_topic_, 20);
    ROS_INFO("[cmd_vel_stamper] %s -> %s", input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void twistCallback(const geometry_msgs::TwistConstPtr& msg)
  {
    geometry_msgs::TwistStamped out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = frame_id_;
    out.twist = *msg;
    pub_.publish(out);
  }

private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_stamper");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  CmdVelStamper node(nh, private_nh);
  ros::spin();
  return 0;
}
