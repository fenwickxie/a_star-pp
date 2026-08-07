#include "diff_purepursuit.h"


/*********************
 * 1.需优化一个地方，当当前找到的前视点和全局路径的终点之间的距离小于前视距离时，
 * 下一次找前视点不用像之前那样计算了，直接把最后一个点作为下一次的前视点 
 * 并且也要满足机器人与前视点之间距离要小于前视距离时才找下一个这个规则--10mon.12.2020
 * 
 * *******************/



PurePursuit::PurePursuit(ros::NodeHandle nh, ros::NodeHandle private_nh):nh_(nh), private_nh_(private_nh)
{
    is_initial_ = false;
    if(!is_initial_)
    {
        initParams();
        is_initial_ = true;

    }

    if(!read_path_file_)global_path_sub_ = nh_.subscribe("/global_path", 1, &PurePursuit::globalPathCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 10,&PurePursuit::ScanCallback, this);
  
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 50, this);
    KfPointMarkPub_ = nh_.advertise<visualization_msgs::Marker>("kf_point_makr", 10, this);
    GlobalPathLastPointPub_ = nh_.advertise<visualization_msgs::Marker>("PahtLastPoint", 10, this);

    readpath_pub_ = nh_.advertise<nav_msgs::Path>("readpath", 5, this);

    reach_pub_ = nh_.advertise<std_msgs::Bool>("/navi_finish", 1, this);
    poincloud_sub_ =
        nh_.subscribe(pointcloude_topic_, 10, &PurePursuit::PointCloudCallback,
                      this); // 多线点云话题
    NearCloudStack_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    
    

    


    
    if(read_path_file_)
    {
        //从文件中读取路径
        ReadCSVpath(path_flie_);

        loop_path_ = true;

    }

    reach_flag_.data = false;

   stopob_th_ = std::thread(&PurePursuit::StopObstacleHandle, this);
    stopob_th_.detach();
}



void PurePursuit::StopObstacleHandle(void) {

  while (1) {
    // std::cout<<"1111111111111"<<std::endl;
    sleep(0.001);
    m_mutex_.lock();
    if (enable_scan_stop_ && scan_.ranges.size() > 0) {
      auto ScanNearstPointIterator =
          std::min_element(scan_.ranges.begin(), scan_.ranges.end()); //

      int NearstPointIndex =
          std::distance(scan_.ranges.begin(), ScanNearstPointIterator);

      auto closet_angle =
          scan_.angle_min + scan_.angle_increment * NearstPointIndex;

      if ((NearstPointIndex > scan_stop_index_min_ &&
           NearstPointIndex <
               scan_stop_index_max_)) // 根据单线雷达来设计，机器人正前方雷达线是第180根，则左右各扩展一部分，再根据距离判断停障
      {
        if (scan_.ranges[NearstPointIndex] < scan_stop_dis_thre_ &&
            scan_.ranges[NearstPointIndex] > scan_stop_dis_min_valid_) {
          std::cout << "NearstPointIndex is :" << NearstPointIndex << std::endl;
          // std::cout<<"closet_angle is :"<<closet_angle * 180.0 /
          // M_PI<<std::endl; std::cout<<"closet_DISTANCE is
          // :"<<scan_.ranges[NearstPointIndex]<<std::endl;
          {

            stop_obstacle_trig_ = true;
            // ROS_INFO("OBSTACLE  DECECTE");
          }
        }

            } else {
                stop_obstacle_trig_ = false;
                //  ROS_INFO("NO OBSTACLE  DECECTE");
            }
        } else if (!enable_scan_stop_) {
            stop_obstacle_trig_ = false;
    }

    /*************************处理点云雷达
     * 判断停障****************************** */
    if (NearCloudStack_->size() > 0) {
      pcl::PointXYZI point;
      int cloudsize = NearCloudStack_->size();
      getBasePose(base_pose_);
      // for(int i  = 0; i < cloudsize; i++)
      // {
      //     point = NearCloudStack_.points[i];

      //     float dis = sqrt((point.x - base_pose_(0)) * (point.x -
      //     base_pose_(0)) + (point.y - base_pose_(1)) * (point.y -
      //     base_pose_(1)));

      //     if(dis < ob_thre_)
      //     {
      //         NearCloudStack_->push_back(point);
      //         //若进来的点云是单帧，且是激光雷达坐标系下，则直接计算障碍物距离机器人的距离

      //     }

      // }
    //   注意这里不要在循环里面修改标志位，容易不停跳动
        bool obs_found=false;
      for (int i = 0; i < cloudsize; i++) {
        point = NearCloudStack_->points[i];
        // double new_x = -point.y;
        // double new_y = point.x;
        double new_x = point.x;
        double new_y = point.y;

        // 当前振
        float dis = sqrt((point.x) * (point.x) + (point.y) * (point.y));
        // float dis = sqrt((point.x - base_pose_(0)) * (point.x -
        // base_pose_(0)) + (point.y - base_pose_(1)) * (point.y -
        // base_pose_(1)));

        // 再计算

        // 排除地面的点云以及无效的全为0的点云
        if (dis < ob_thre_ && point.z > 0.1 && new_x != 0 && new_y != 0)

        {
          // double theta = atan2(point.y, point.x);
          //注意：小车前进方向为雷达y轴方向，这里求解的是与y轴的夹角
          double theta = atan2(new_x, new_y);

          if (fabs(theta) < 0.5) {
            ROS_INFO("FOUND OBS");

            LOG(INFO) << "The index i is " << i;
            LOG(INFO) << "The theta  is " << theta;
            LOG(INFO) << "point x is " << new_x << " point y is " << new_y
                      << " point z is " << point.z;

            // stop_obstacle_trig_ = true; // 停止障碍物
            obs_found=true;
            break;//找到就停止
            // NearCloudStack_->push_back(point);
            // 若进来的点云是单帧，且是激光雷达坐标系下，则直接计算障碍物距离机器人的距离
            // 同时可以设置有障碍物
          }

        } else {
          obs_found=false;
        //   stop_obstacle_trig_ = false;
        }
      }
      stop_obstacle_trig_=obs_found;
    }
    // LOG(INFO) << "The stop_obstacle_trig_ is " << stop_obstacle_trig_;


    m_mutex_.unlock();
  }
}

void PurePursuit::PointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {

  NearCloudStack_->clear();
  pcl::fromROSMsg(*msg, *NearCloudStack_);
}





void PurePursuit::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    
    scan_ = *msg;


}


void PurePursuit::initParams(void)
{
    private_nh_.getParam("forward_sight_dis", forward_sight_dis_);
    private_nh_.getParam("forward_sight_dis_coffie", forward_sight_dis_coffie_);
    private_nh_.getParam("robot_ref_vel", robot_ref_vel_);//线速度
    private_nh_.getParam("base_frame", base_frame_);
    private_nh_.getParam("world_frame", world_frame_);
    private_nh_.getParam("yaw_to_omega_coffe", yaw_to_omega_coffe_);
    private_nh_.getParam("linear_vel_p_coffe", linear_vel_p_coffe_);
    private_nh_.getParam("thresold_reach", thresold_reach_);
    private_nh_.getParam("path_flie", path_flie_);
    private_nh_.getParam("read_path_file", read_path_file_);
    private_nh_.getParam("pointcloude_topic", pointcloude_topic_);
  private_nh_.getParam("ob_thre", ob_thre_);
  private_nh_.getParam("max_angular", max_angular_);
    private_nh_.getParam("enable_scan_stop", enable_scan_stop_);
    private_nh_.getParam("scan_stop_dis_thre", scan_stop_dis_thre_);
    private_nh_.getParam("scan_stop_index_min", scan_stop_index_min_);
    private_nh_.getParam("scan_stop_index_max", scan_stop_index_max_);
    private_nh_.getParam("scan_stop_dis_min_valid", scan_stop_dis_min_valid_);


    

    ROS_INFO("robot_ref_vel_ : %f", robot_ref_vel_);
    ROS_INFO("forward_sight_dis : %f", forward_sight_dis_);

    //参考线速度根据权重系数加入到前视距离中
   // forward_sight_dis_ = forward_sight_dis_ + forward_sight_dis_coffie_ * robot_ref_vel_;

    last_kf_point_index_ = 0;
    last_last_kf_point_ = 0;
    base_pose_.setZero();
    sub_path_ = false;
    GetPathNum_ = 0;
    RecodrPathNum_ = 0;
    RecordPlanState_ = 1;//inital  no path state
    ResetPlan_ = 0;

    IsFirstKfAimPoint_ = false;


    

}


void PurePursuit::ReadCSVpath(std::string file )
{
    ROS_INFO("read path");
    std::ifstream  pathfile_(path_flie_);
    if (!pathfile_.is_open()) {
        std::cerr << "Error: Could not open file." << std::endl;
        return ;
    }
    std::string line;
    while(std::getline(pathfile_, line))
    {
       

        Pose3d point;
        std::stringstream ss(line);
        std::string cell;

        int i = 0;
        while (std::getline(ss, cell, ',')) 
        {
            point(i) = std::stod(cell);
            i++;

        }
        global_piont_path_.push_back(point);

    }

  //  std::cout<<"global_piont_path_ size is :"<<global_piont_path_.size()<<std::endl;


    if(global_piont_path_.size() > 2)
    {
        //路径读取成功，则rviz可视化发布出来
        readpath_.header.frame_id = world_frame_;

        for(auto point : global_piont_path_)
        {
            geometry_msgs::PoseStamped po;

            po.header.frame_id = world_frame_;
            po.pose.position.x = point(0);
            po.pose.position.y = point(1);

            readpath_.poses.push_back(po);

        }

        readpath_pub_.publish(readpath_);

    }
}


PurePursuit::~PurePursuit()
{
  
   
}



//定位成功下，查询坐标系tf  map->base_footprint得到机器人位姿
void PurePursuit::getBasePose(Pose3d& base_pose)
{
    tf::StampedTransform transform;
    tf_.waitForTransform(world_frame_, base_frame_, ros::Time(0), ros::Duration(1.0));
    try{

        tf_.lookupTransform(world_frame_, base_frame_, ros::Time(0), transform );//base到map的变换，即Tmap_base
    }
    catch(tf::LookupException& ex)
    {
          ROS_ERROR("%s",ex.what());
    }

    base_pose(0) = transform.getOrigin().getX();
    base_pose(1) = transform.getOrigin().getY();
    base_pose(2) = tf::getYaw(transform.getRotation());

    //std::cout<<" X: "<< base_pose(0)<<std::endl;
   // std::cout<<" Y: "<< base_pose(1)<<std::endl;

   // std::cout<<" Z: "<< base_pose(2)<<std::endl;

//    LOG(INFO)<<"Get base pose is : "<<base_pose_[0]<<", "\
//    <<base_pose_[1]<<", "<<base_pose_[2] * 180 / M_PI;

}



// Vpath = std::vector<Pose3d>;
// Vpath global_piont_path_;
void PurePursuit::globalPathCallback(const nav_msgs::PathConstPtr& msgs)
{
    //ROS_INFO("Get path");
    //reset();

    reach_flag_.data = false;
    //订阅到新路径时，修改为如下，比reset()少了一个停止机器人
    last_kf_point_index_ = 0;
    last_last_kf_point_ = 0;
    IsFirstKfAimPoint_ = false;
    if(!global_piont_path_.empty())global_piont_path_.clear();

    //目标点航向，a_star_test中赋值的
    cur_goal_yaw_ = msgs->poses[msgs->poses.size()-1].pose.position.z;


    for(int i = 0; i < msgs->poses.size(); i++)
    {
        Pose3d point;

        point(0) = msgs->poses.at(i).pose.position.x;
        point(1) = msgs->poses.at(i).pose.position.y;
       // point(2) = tf::getYaw(msgs->poses.at(i).pose.orientation);

        global_piont_path_.push_back(point);

      //  LOG(INFO)<<"path is "<<"x= "<< point(0)<<", "<<"y= "<< point(1)<<", "<<"yaw ="<< point(2);
    }
    sub_path_ = true;

    ResetPlan_ = 1;


    visualization_msgs::Marker pathlastpoint;
     //这里传入的坐标是相对于map坐标系的一个路径点,所以写map
    pathlastpoint.header.frame_id = world_frame_;
    pathlastpoint.header.stamp = ros::Time::now();
    pathlastpoint.ns = "my_namespace";//随便写一个
    pathlastpoint.id = 0;
    pathlastpoint.type = visualization_msgs::Marker::SPHERE;
    pathlastpoint.action = visualization_msgs::Marker::ADD;
    pathlastpoint.pose.position.x = global_piont_path_.back()(0);
    pathlastpoint.pose.position.y = global_piont_path_.back()(1);
    pathlastpoint.pose.position.z = 0;
    pathlastpoint.scale.x = 0.1;// 0.2m
    pathlastpoint.scale.y = 0.1;//0.2m
    pathlastpoint.scale.z = 0.1;//0.2m
    pathlastpoint.color.a = 1.0;// 必须设置为1 否则不显示标记
    pathlastpoint.color.r = 2.0;
    pathlastpoint.color.g = 4.0;
    pathlastpoint.color.b = 6.0;

    GlobalPathLastPointPub_.publish(pathlastpoint);



  
   if(global_piont_path_.size()>2) LOG(INFO)<<" Global Path Nums is : "<<global_piont_path_.size();

}








//not use
void PurePursuit::baseposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msgs)
{
    base_pose_[0] = msgs->pose.pose.position.x;
    base_pose_[1] = msgs->pose.pose.position.y;
    base_pose_[2] = tf::getYaw(msgs->pose.pose.orientation); 

   // LOG(INFO)<<"Get base pose is : "<<base_pose_[0]<<", "\
   // <<base_pose_[1]<<", "<<base_pose_[2] * 180 / M_PI;
    //ROS_INFO("base yaw is : %f",  base_pose_[2] * 180 / M_PI);

}

//not use
void PurePursuit::cmdvelCallback(const geometry_msgs::TwistConstPtr& msgs)
{
    sub_cmd_vel_ = *msgs;
    //LOG(INFO)<<"Sub Robot Velocity is : "<<sub_cmd_vel_;

}

void PurePursuit::reset(void)
{
     // PurePursuit::StopRobot();
     // sub_path_ = false;
      last_kf_point_index_ = 0;
       last_last_kf_point_ = 0;
       IsFirstKfAimPoint_ = false;
       
      if(!global_piont_path_.empty())global_piont_path_.clear();
}


//no use
bool PurePursuit::FirstKfAngleAlign(Pose3d& firstAimPoint_in_robot)
{
    // geometry_msgs::Twist vel_msg;
    // Pose3d basepose;

    // getBasePose(basepose);



    // vel_msg.angular.z = atan2(firstAimPoint_in_robot(1), firstAimPoint_in_robot(0));
    

    // vel_pub_.publish(vel_msg);

    return true;

}








//将当前找到的这个路径点转到机器人当前坐标系下，即要得到这个路径点在机器人坐标系中的坐标
bool PurePursuit::checkReachGoal(Pose3d& base_pose, Vpath& global_path)
{
    double dx, dy;
    if(!global_piont_path_.empty())
    {
        dy = global_path[global_path.size() - 1](1) - base_pose(1);
        dx = global_path[global_path.size() - 1](0) - base_pose(0);

        FollowPathComplish_ = 1;
        
        auto dis = std::sqrt(dy * dy + dx * dx);
        printf("dis is : %f\n", dis);

        if(dis < thresold_reach_)return true; 
    }
    return false;
}


//前视点的显示
void PurePursuit::VisualKFPointMark( visualization_msgs::Marker& KfPontMark, Pose3d& pose)
{
    //这里传入的坐标是相对于机器人本体base_footprint坐标系的一个路径点,所以写base_footprint
    KfPontMark.header.frame_id = base_frame_;
    KfPontMark.header.stamp = ros::Time::now();
    KfPontMark.ns = "my_namespace";//随便写一个
    KfPontMark.id = 0;
    KfPontMark.type = visualization_msgs::Marker::SPHERE;
    KfPontMark.action = visualization_msgs::Marker::ADD;
    KfPontMark.pose.position.x = pose(0);
    KfPontMark.pose.position.y = pose(1);
    KfPontMark.pose.position.z = 0;
    KfPontMark.scale.x = 0.2;// 0.2m
    KfPontMark.scale.y = 0.2;//0.2m
    KfPontMark.scale.z = 0.2;//0.2m
    KfPontMark.color.a = 1.0;// 必须设置为1 否则不显示标记
    KfPontMark.color.r = 5.0;
    KfPontMark.color.g = 1.0;
    KfPontMark.color.b = 0.0;

    KfPointMarkPub_.publish(KfPontMark);

}


void PurePursuit::StopRobot(void)
{
    geometry_msgs::Twist temvel;
    temvel.angular.z = 0;
    temvel.linear.x = 0;

    vel_pub_.publish(temvel);
}



//将找到的前视点的坐标转到机器人的坐标系下
//base_pose--机器人当前map系下坐标   pathPoint--找到的map系下前视点
 bool PurePursuit::transPoseTBaseCoordination(const Pose3d& base_pose, Pose3d& pathPoint)
 {

    Eigen::Matrix3d mat;
    Eigen::Vector3d vec;

    //base_footprint到map系的齐次坐标变换矩阵
    mat<<cos(base_pose(2))  ,-sin(base_pose(2)) ,  base_pose(0),
          sin(base_pose(2)) ,  cos(base_pose(2)), base_pose(1) ,
                   0        ,         0         ,              1;

    vec<<pathPoint(0), pathPoint(1), 1;

    //要得到路径点在机器人坐标中的坐标，所以这里要取逆，得到map系到base_footprint的齐次坐标变换
    //再乘以这个路径点的坐标(map系下),得到该路径点在base_footprint系的坐标表示
    vec = mat.inverse() * vec ;
    pathPoint(0) = vec(0);
    pathPoint(1) = vec(1);

   // LOG(INFO)<<"pathPoint is :"<<pathPoint(0)<<", "<<pathPoint(1);
    return true;

 }


//
/***************
 计算给点的全局路径点上满足前视距离的最近的路径点
 Vpath = std::vector<Pose3d>;
 Pose3d = Eigen::Vector3d;

*params:
index--路径点上机器人还未走过的路径点中，距离机器人最近的点在路径点中的索引, 是上一次的索引引用
global_path--全局路径点集合
base_pose--机器人当前位姿

计算机器人未走过的路径中，距离机器人最近的满足前视距离的点
 **************/
void PurePursuit::calcu_target_goal(Pose3d& base_pose, Vpath& global_path, int& index)
{
    
    double dy, dx;
    double sum_dis;

    nearst_path_point_.clear();

    //计算未走过的路径中即路径点转到机器人系后，x是大于0的，距离机器人最近的点的索引,
    //没到倒数第二个点
    if(loop_path_ || (index + 1) < global_path.size() - 1)
    {
        getBasePose(base_pose_);//get base pose in map

        //遍历整个路径，找到距离机器人最近的路径点坐标, 问题--走过的路径点也会遍历 ,这对于循环遍历一个环形轨迹有用
        //if(!loop_path_ || nearst_path_point_.size() < 2)//循环路径时，仅计算一次
        {
             for(auto p = global_path.begin(); p != global_path.end(); p++)
            {
                dy =  (*p)(0) - base_pose(0);
                dx =  (*p)(1) - base_pose(1);
                nearst_path_point_.push_back(std::sqrt(dy * dy + dx * dx));
                
            }
            //ROS_INFO("nearst_path_point_nearst_path_point_nearst_path_point_nearst_path_point_nearst_path_point_");
        }
       
       // ROS_INFO("nearst_path_point_ SIZE is %d", nearst_path_point_.size());

        auto NearstPointIterator = std::min_element(nearst_path_point_.begin(), nearst_path_point_.end());
        //得到最小值在容器中的索引，索引是从0起
        int NearstPointIndex = std::distance(nearst_path_point_.begin(), NearstPointIterator);
        // or  auto NearstPointIndex = NearstPointIterator - nearst_path_point.begin();
        
        //这里加入代码，已经找到距离机器人最近的路径点，将全局路径中在这个索引之前的路径点从全局路径中删除
        // 减少计算量和避免找到走过的点的可能

        



        //从上面计算的最近点开始，向路径后面依次遍历，直到找到距离机器人距离值第一个大于或等于前视距离的点的索引
        //这里计算距离，需要通过路径点之间的距离累加得到        
        for( auto p = (global_path.begin() + NearstPointIndex) ; p != global_path.end(); p++)
        {
            //if(sum_dis < forward_sight_dis_ && (loop_path_ || (NearstPointIndex + 1) < global_path.size() - 1))
            if(loop_path_ || (NearstPointIndex + 1) < global_path.size() - 1)
            {
                // auto delx = (*(p+1))(0) - (*p)(0);//计算相邻路径点间的2范数
                // auto dely = (*(p+1))(1) - (*p)(1);
                // sum_dis += std::sqrt(delx * delx + dely * dely);

                // NearstPointIndex++;//从最近点往后取点
                //是逐个路径点计算与机器人的欧式距离，而不是逐个点之间线段累加到前视距离
                if(nearst_path_point_[NearstPointIndex] >= forward_sight_dis_)
                {
                    index = NearstPointIndex;

                   // ROS_INFO("INDEX IS %d", index);
                    return ;//找到返回

                }

                NearstPointIndex++;




            }
            else if(!loop_path_ && NearstPointIndex == global_path.size() - 1 - 1)
            {
                index =  global_path.size() - 1 - 1;
               // LOG(INFO)<<"Nearst kf Point is Path Second End Point is : "<<index;
                return;
            }
            else if(!loop_path_ && NearstPointIndex == global_path.size() - 1)
            {
                // path end point
                index =  global_path.size() - 1;
               // LOG(INFO)<<"Nearst kf Point is Path End Point is : "<<index;
                return;
            }
            else if(sum_dis >= forward_sight_dis_)
            {
                index = NearstPointIndex;
               // LOG(INFO)<<"Nearst kf Point is : "<<index;
                return;
            }
        }
    }
    else if(index + 1 ==  global_path.size() - 1)
    {
        index = global_path.size() - 1 - 1;
      //  LOG(INFO)<<"Index is last 2";
        return;
    }
    else if(index  ==  global_path.size() - 1)
    {
        index = global_path.size() - 1;
     //   LOG(INFO)<<"Index is last point";
        return;
    }

}


 bool PurePursuit::AlignYaw(void)
 {
    //四元数转欧拉角
   


    float theta =  base_pose_(2) - cur_goal_yaw_  ;
    ROS_INFO("cur_goal_yaw_ IS : %f", cur_goal_yaw_* 180 / M_PI);
     ROS_INFO("base_pose__yaw_ IS : %f", base_pose_(2)* 180 / M_PI);
    theta = theta * 180 / M_PI;//转为角度制

    if(theta < 0) theta += 360.0; 

    // std::cout<<"diff theta:"<<theta<<std::endl;


    if(fabs(theta) <= stopyaw_)
    {
      return true;
    }

    if(theta > 180)
    {
        //逆时针
        vel.angular.z = 0.2;
        vel.linear.x = 0;
        vel_pub_.publish(vel);
      //  ROS_INFO("1111");
    }
    else if(theta <= 180.0)
    {
        //顺时针
        vel.angular.z = -0.2;
        vel.linear.x = 0;
        vel_pub_.publish(vel);
       // ROS_INFO("22222");

    }

    diff_yaw_cur_to_goal_ = theta;//degree
   
    //ROS_INFO("Diff yaw is : %f",theta );
    printf("dis_theta is : %f\n",theta );
  
   
    

  return false;



 }


/***************
 * 
 * 
 *Calculate Control Velocity
 * 
 *
 * **************/
void PurePursuit::purepursuit_control(void)
{
    // use to publish velocity
    double angle, dy, dx;
    double point_yaw;
    Pose3d tmpKFPoint;

    int lastest_index_kf = -1;

    if(stop_obstacle_trig_)
    {
        StopRobot();
        return;
    }
    
   
  
    if(read_path_file_  || sub_path_ )//是否获得路径
    {

        getBasePose(base_pose_);

        //机器人位置和路径终点(认为目标点)距离小于设定值时，则认为到达目标点
        if(!loop_path_ && attitude_state_ ||checkReachGoal(base_pose_, global_piont_path_))
        {
            //LOG(INFO)<<"Reach Goal";
           // StopRobot();
            //vel.angular.z = 0;
           // vel.linear.x = 0;
            //sub_path_ = false;
            reset();//global_piont_path_.clear()
            
            if(attiude_aglin_)//如果需要航向对齐
            {
                attitude_state_ = true;
                if(AlignYaw())
                {
                    attitude_state_ = false;
                    StopRobot();
                    sub_path_ = false;

                    //发布到达话题，通知上装
                    reach_flag_.data = true;
                    for(int i = 0; i < 1; i++)reach_pub_.publish(reach_flag_);
                    reach_flag_.data = false;

                    ROS_INFO("Reach Goal !!!");
                }
                else{
                    return;
                }
            }


            sub_path_ = false;

            return;
          
        }

        if(!global_piont_path_.empty())
        {
            //当前视点是倒数第二个全局路径点时，下一次算前视点就不用算了，直接将最后一个路径点索引给到下一个视点即可。
            //但是caution，也是要等到机器人与倒数第二个路径点小于前视点时才会到下一个前视点 
            if(!loop_path_ && last_kf_point_index_== global_piont_path_.size() -1 -1)
            {
                if(sqrt( pow(base_pose_(0) - global_piont_path_[last_kf_point_index_](0), 2) 
                + pow(base_pose_(1) - global_piont_path_[last_kf_point_index_](1), 2))
                < forward_sight_dis_)
                {
                    last_kf_point_index_ = global_piont_path_.size() - 1;
                }
            }
            else 
            {
                //计算路径中，距离机器人最近且满足前视距离的点的索引
                calcu_target_goal(base_pose_, global_piont_path_, last_kf_point_index_);
            } 
            
            //

            if(loop_path_ || last_kf_point_index_ <= global_piont_path_.size() - 1)//not end point
            {
                    getBasePose(base_pose_);

                    tmpKFPoint(0) = global_piont_path_[last_kf_point_index_](0);
                    tmpKFPoint(1) = global_piont_path_[last_kf_point_index_](1);

                    //将这个路径点转到机器人坐标系base_footprint下
                    transPoseTBaseCoordination(base_pose_, tmpKFPoint);

                    //可视化当前的前视点
                    VisualKFPointMark(KfPontMark_, tmpKFPoint);

                    

                    //计算前视点在当前机器人下的相对角度
                    //如果不转换到机器人坐标系可以这样: 
                    //机器人和路径点的世界坐标，
                    //atan2(robot_y - pathpoint_y, robot_x - pathpoint_x ) - robot_yaw
                    angle = atan2(tmpKFPoint(1), tmpKFPoint(0));
                   // std::cout<<"angle is : "<<angle * 180 / M_PI<<std::endl;

                    //让机器人的朝向和第一个目标点之间的偏角足够小后才开始给线性速度，
                    //主要为了第一个目标点在机器人的后方，机器人需要一个转弯空间，
                    //这对于狭窄空间有帮助
                    if(last_kf_point_index_ >= 1  && abs(angle) > 3.0 * M_PI / 180  && IsFirstKfAimPoint_ == false)
                    {
                        
                        if(angle > 0 )
                        {
                            vel.angular.z = 0.3;
                           // ROS_INFO("3333");
                        }
                        else 
                        {
                            vel.angular.z = -0.3;
                           // ROS_INFO("4444");
                            
                        }

                        
                        vel.linear.x = 0;
                       // std::cout<<"is first kf"<<std::endl;


                    }
                    else if(last_kf_point_index_ >= 1)
                    {

                        float ld2 = pow(tmpKFPoint(0), 2) +  pow(tmpKFPoint(1), 2);
                        
                        

                        //vel.angular.z = -yaw_to_omega_coffe_ * angle;
                        //
                        vel.angular.z = 2 * robot_ref_vel_ * tmpKFPoint(1) / ld2; //差速底盘纯跟踪模型解算角速度

                        vel.linear.x = static_cast<float>(robot_ref_vel_);
                        IsFirstKfAimPoint_ = true; //////////////////
                       // std::cout<<"Not first kf"<<std::endl;

                    }


                    //this diff chasis model  ,just like turtle2 follow turtle1 example
                   

                    //10.12
                    if(!loop_path_ && checkReachGoal(base_pose_, global_piont_path_))
                    {
                        LOG(INFO)<<"Reach Goal";
                        vel.angular.z = 0;
                        vel.linear.x = 0;
                        StopRobot();
                      if(!attiude_aglin_)  sub_path_ = false;
                        reset();//global_piont_path_.clear()
                        return;
                    
                    }                  
            }
            else{
                vel.angular.z = 0;
                vel.linear.x = 0;
              //  LOG(INFO)<<"kf_index beyond path";
                last_kf_point_index_ = 0;
                
            }

            //判断速度是否是无效值和无穷值
            if(std::isinf(vel.linear.x ) || std::isnan(vel.linear.x ))
            {
              //  LOG(INFO)<<"Linear Vel is not Val";
            
                if(std::isinf(vel.angular.z ) || std::isnan(vel.angular.z))
                {
              //      LOG(INFO)<<"Angular Vel is not Val";
                }
            }
            else
            {
                vel_pub_.publish(vel);//publish Velocity to Robot
            } 
        }

        //last_last_kf_point_ = last_kf_point_index_;
    }
}

