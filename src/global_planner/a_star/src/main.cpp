#include "a_star_test.h"




int main(int argc, char * argv[])
{

    ros::init(argc, argv, "a_star_test");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    AStarTest m_a_star(nh, private_nh);
    ros::TimerEvent tevent;
    ROS_INFO("Start ROS");
    ros::Rate rate(2);
 
    while(ros::ok())
    {
        m_a_star.PlanningLoop();

        ros::spinOnce();

        rate.sleep();
        

    }

    return 0;
}












