#ifndef _DIFF_PUREPURSUIT_H
#define _DIFF_PUREPURSUIT_H

#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Path.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#define GLOG_USE_GLOG_EXPORT
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include <signal.h>
#include <sstream>
#include <fstream>
#include <string>
#include <iostream>
#include "nav_msgs/Path.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include <thread>
#include <mutex>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

//using Lkf    = 1.0;
//using k      = 0.1;

using Pose3d = Eigen::Vector3d;
using Pose2d = Eigen::Vector2d;
using Vpath = std::vector<Pose3d>;

class PurePursuit{

public:
    PurePursuit(ros::NodeHandle nh, ros::NodeHandle private_nh);
    virtual ~PurePursuit();

    void initParams(void);

    Pose3d getBasePose(void){return base_pose_;}

    void calcu_target_goal(Pose3d& base_pose, Vpath& global_path, int& index);

    void purepursuit_control(void);

    void globalPathCallback(const nav_msgs::PathConstPtr& msgs);
    void baseposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msgs); 
    void cmdvelCallback(const geometry_msgs::TwistConstPtr& msgs);

    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void getBasePose(Pose3d& base_pose);

    bool transPoseTBaseCoordination(const Pose3d& base_pose, Pose3d &pathPoint);

    void setPath(nav_msgs::PathConstPtr& path);

    bool checkReachGoal(Pose3d& base_pose, Vpath& global_path);

    void VisualKFPointMark( visualization_msgs::Marker& KfPontMark, Pose3d& pose);

    void StopRobot(void);

    
    void reset(void);

    //在机器人去第一个目标点之前 先将机器人旋转到目标点角度对齐，即目标点相对机器人坐标系的偏航角为20，
    //尤其对其第一个目标点在机器人坐标系的后方时，先让机器人转过来。
    //对于狭窄空间有用

    bool FirstKfAngleAlign(Pose3d& firstAimPoint_in_robot);

    void ReadCSVpath(std::string file);

    void StopObstacleHandle(void);

    //到达位置后做航向对齐
    bool AlignYaw(void);




private:
    bool is_initial_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber global_path_sub_;
    ros::Subscriber base_pose_sub_;
    ros::Subscriber vel_sub_;

    ros::Subscriber scan_sub_;//当前侦激光雷达，单线雷达数据

    ros::Publisher vel_pub_;
    ros::Publisher KfPointMarkPub_;
    ros::Publisher GlobalPathLastPointPub_;

    ros::Publisher readpath_pub_;

    ros::Publisher reach_pub_;

    geometry_msgs::Twist sub_cmd_vel_;

    tf::TransformListener tf_;
   

    Pose3d base_pose_;
    Pose3d last_base_pose_;

    nav_msgs::Path global_path_;

    Vpath global_piont_path_;


    int last_kf_point_index_;//上一次路径点中，距离机器人位姿最近的满足前视距离点的在路径点集合中的索引

    double yaw_to_omega_coffe_;
    double linear_vel_p_coffe_;

    double forward_sight_dis_;//前视距离

    double forward_sight_dis_coffie_;
    double robot_ref_vel_;//线速度

    double thresold_reach_;

    std::string base_frame_;
    std::string world_frame_;

    bool sub_path_;

    visualization_msgs::Marker KfPontMark_;

    int GetPathNum_;

    int RecodrPathNum_;

    int RecordPlanState_;//记录规划状态，主要为了有新路径抢占

    bool FollowPathComplish_;//路径跟踪是否完成  0-not complish  1-complish

    bool ResetPlan_;

    bool IsFirstKfAimPoint_;

    int last_last_kf_point_;


    //2024.8.16 
    std::string path_flie_;

    //读取文件中的路径或订阅路径
    bool read_path_file_ = false;

    nav_msgs::Path readpath_;

    bool loop_path_ = false;
    
    std::vector<double> nearst_path_point_;

    sensor_msgs::LaserScan scan_;

    bool stop_obstacle_trig_ = false;

    std::thread stopob_th_;

    geometry_msgs::Twist vel;

    std::mutex m_mutex_;  

    bool attiude_aglin_ = true;

    double cur_goal_yaw_;//目标航向角
    double diff_yaw_cur_to_goal_ = 0;//机器人和目标点航向差

    double stopyaw_ = 2.0;//度

    std_msgs::Bool reach_flag_;

    bool attitude_state_ = false;

    std::string gloalplath_topic_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr NearCloudStack_;
    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    ros::Subscriber poincloud_sub_; // 订阅点云
    
        std::string pointcloude_topic_;
        double ob_thre_ = 1.2;
        double max_angular_=0.3;
        bool enable_scan_stop_ = true;
        double scan_stop_dis_thre_ = 0.30;
        int scan_stop_index_min_ = 165;
        int scan_stop_index_max_ = 195;
        double scan_stop_dis_min_valid_ = 0.05;





    

};







#endif 
