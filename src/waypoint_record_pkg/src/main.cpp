#include "waypoint_record_pkg.h"

PathRecord::PathRecord(const ros::NodeHandle nh, const ros::NodeHandle private_nh_):nh_(nh), private_nh_(private_nh_)
{

    private_nh_.getParam("pose_topic", pose_topic_);
    private_nh_.getParam("geopose_topic", geopose_topic_);
    private_nh_.getParam("path_file",  path_file_);
    private_nh_.getParam("record_therold",  record_therold_);
    private_nh_.getParam("world_frame",  world_frame_);



    pose_sub_ = nh_.subscribe(pose_topic_, 5, &PathRecord::PoseCallback,this);//定位消息类型1
    geo_pose_sub_ = nh_.subscribe(geopose_topic_, 5, &PathRecord::GeomPoseCallback,this);//定位消息类型2

    path_pub_ = nh_.advertise<nav_msgs::Path>("/pur_path", 5, this);

    time_t time_seconds = time(0);
    struct tm* now_time = localtime(&time_seconds);
    printf("Reach localtime is %d-%d-%d %d:%d:%d\n", now_time->tm_year + 1900, now_time->tm_mon + 1,
	now_time->tm_mday, now_time->tm_hour, now_time->tm_min, now_time->tm_sec);

    //打开文件名
    csvfile_.open(path_file_ + std::to_string(now_time->tm_year + 1900) + "_" 
    + std::to_string(now_time->tm_mon + 1) + "_" + std::to_string(now_time->tm_mday)+ "_" 
    + std::to_string(now_time->tm_hour)+"_" + std::to_string(now_time->tm_min)+ "_" 
    + std::to_string(now_time->tm_sec)+"_"+"pursuit_path.csv",std::ios::out|std::ios::app); 

    //内容标题
    csvfile_<<"X"<<","<<"Y"<<","<<"Z"<<"\n";

    ROS_INFO(" pose topic is %s\n", pose_topic_.c_str());


   // path_.poses.resize(1);


}

 void PathRecord::GeomPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
 {

    if(path_.poses.empty()) 
    {
        path_.poses.push_back(*msg);
    }


    auto dis  = sqrt(pow(msg->pose.position.x - path_.poses[path_.poses.size()-1].pose.position.x, 2) +
      pow(msg->pose.position.y - path_.poses[path_.poses.size()-1].pose.position.y, 2));

   // std::cout<<"dis is :"<<dis<<std::endl;

   
    //距离太近的点不写入
    if(dis > record_therold_)
    {

        path_.poses.push_back(*msg);

        path_.header.frame_id = robot_pose_.header.frame_id;
        path_.header.stamp = ros::Time::now();

        path_pub_.publish(path_);

        //写入文件
        if(csvfile_)
        {
            csvfile_<<msg->pose.position.x<<","<<msg->pose.position.y<<","<<msg->pose.position.z<<"\n";

        }

    }


 }





 void PathRecord::PoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
 {

    robot_pose_ = *msg;


    if(path_.poses.empty()) 
    {
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.frame_id = robot_pose_.header.frame_id;
        temp_pose.header.stamp = ros::Time::now();
        temp_pose.pose = robot_pose_.pose.pose;

        path_.poses.push_back(temp_pose);
    }
    
    
    
    auto dis  = sqrt(pow(robot_pose_.pose.pose.position.x - path_.poses[path_.poses.size()-1].pose.position.x, 2) +
      pow(robot_pose_.pose.pose.position.y - path_.poses[path_.poses.size()-1].pose.position.y, 2));

   // std::cout<<"dis is :"<<dis<<std::endl;

   
    //距离太近的点不写入
    if(dis > record_therold_)
    {
        geometry_msgs::PoseStamped tem_pose;

        tem_pose.header.frame_id = robot_pose_.header.frame_id;
        tem_pose.header.stamp = ros::Time::now();
        tem_pose.pose = robot_pose_.pose.pose;


        path_.poses.push_back(tem_pose);

        path_.header.frame_id = robot_pose_.header.frame_id;
        path_.header.stamp = ros::Time::now();

        path_pub_.publish(path_);

        //写入文件
        if(csvfile_)
        {
            csvfile_<<msg->pose.pose.position.x<<","<<msg->pose.pose.position.y<<","<<msg->pose.pose.position.z<<"\n";

        }

    }


 }








int main(int argc , char** argv)
{

    ros::init(argc, argv, "waypoint_record_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    PathRecord solution(nh, private_nh);

    ros::Rate rr(50);
    while(ros::ok())
    {

  
        //for text
        
        ros::spinOnce();
        rr.sleep();
        
    }

    return 0;

}