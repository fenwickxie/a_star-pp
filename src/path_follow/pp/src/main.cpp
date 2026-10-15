
#include "diff_purepursuit.h"



ros::Publisher velpub;

//Ctrl C handle
void signalHandle(int sig )
{
   geometry_msgs::Twist ttvel;
   ttvel.linear.x = 0;
   ttvel.angular.z = 0;
   velpub.publish(ttvel);

   ROS_INFO("Ctrl_C Have Enter !!! ");
   ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diff_purepursuit");

    ros::NodeHandle node;
    ros::NodeHandle private_node("~");


    ros::NodeHandle nh;
    velpub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    
    //glog
    google::InitGoogleLogging(argv[0]);
    google::SetLogDestination(google::GLOG_INFO, "/home/pyq/catkin_ws/src/local_planner_yq/log/"); 

    //gflags
    google::ParseCommandLineFlags(&argc, &argv, true);

    PurePursuit solution(node, private_node);

    signal(SIGINT, &signalHandle);//Ctrl C handle


    LOG(INFO)<<"Start PurePursuit !!!";
    ros::Rate r(50);

    while(ros::ok())
    {
        solution.purepursuit_control();

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}