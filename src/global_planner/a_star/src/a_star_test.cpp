#include "a_star_test.h"
#include "ros/node_handle.h"
#include "bspline.h"






//TODO  
AStarTest::~AStarTest()
{
    
    delete[] my_cellmap_;
}



//TODO  
void AStarTest::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& p_map)
{

  if(havedMap_)return;

   ROS_INFO("ENTER MAP TOPIC");
   havedMap_ = true;
   m_map_ = *p_map;
   num_map_ = p_map->info.width * p_map->info.height;//grib numbers

   GridSize_X_ = p_map->info.width;
   GridSize_Y_ = p_map->info.height;
   origin_X_ = p_map->info.origin.position.x;
   origin_Y_ = p_map->info.origin.position.y;

 
   my_cellmap_->grid = new Node[num_map_];//为栅格申请内存////////////////////////////////////////////
   my_cellmap_->resolution = p_map->info.resolution;
   my_cellmap_->width = p_map->info.width;
   my_cellmap_->height = p_map->info.height;
   my_cellmap_->orignx = p_map->info.origin.position.x;
   my_cellmap_->origny = p_map->info.origin.position.y;

    std::cout<<"1111"<<std::endl;
  
   
    calCacheFreeCellDist(p_map, my_cellmap_); 
    std::cout<<"22222"<<std::endl;
    MakeCostmap2d(p_map);//膨胀

   std::cout<<"333333"<<std::endl;
   PubCostMap2d(p_map);

  


   std::cout<<"num_map: "<<num_map_<<std::endl;
   std::cout<<"p_map->info.width: "<<p_map->info.width<<std::endl;
   std::cout<<"p_map->info.height: "<<p_map->info.height<<std::endl;

   resolution_ = p_map->info.resolution;

   grid_data = p_map->data;


    
   map_sub.shutdown();
}

 
void AStarTest::MakeCostmap2d(const nav_msgs::OccupancyGridConstPtr& msg)
{
    std::vector<unsigned int> checkindex;
    for(unsigned int tmp_index = 0; tmp_index < msg->data.size(); tmp_index++)//遍历静态地图的每一个栅格，如果是占用就对其进行膨胀处理
    {
       // checkindex.clear();//
        VistEightNeighbourHood(tmp_index, checkindex);//找到当前栅格膨胀半径范围内的空闲栅格
    }

     if(!checkindex.empty())//把上面找到的栅格设置为膨胀值
    {     
        for(auto index : checkindex)//把这些栅格设置为膨胀值
        {
            (my_cellmap_->grid + index)->cell_state_ = CELLSTATE::INFLATION_OCCUPY;
            (my_cellmap_->grid + index)->visiable = 0;
        }
     }
}


void AStarTest::VistEightNeighbourHood(unsigned int& index, std::vector<unsigned int>& checkindex)
{
    
    if(((my_cellmap_->grid + index)->cell_state_) == CELLSTATE::OCCUPY)//occupy
   {
       unsigned int gridx, gridy, tempindex;
      
       //计算栅格坐标
       GetGridCoordinateFromIndex(gridx, gridy, index);//
        // ROS_INFO(" gridx, gridy is : %d, %d", gridx, gridy);

        //膨胀半径对应的栅格数
       int inflation_grid = inflation_radius_ / resolution_;

        //是否越界
       if(BeyondGridMapScope(gridx, gridy))
        {    
            //以当前占用栅格为中心 膨胀的范围是膨胀半径对应的栅格范围，遍历8邻域
            for(int i = -inflation_grid; i < inflation_grid + 1; i++)
            {
                for(int j = -inflation_grid; j < inflation_grid + 1; j++)
                {
                    if(i!= 0  && j!=0)//不是本身
                    {
                        auto tmx = gridx;//这里必须新建栅格
                        auto tmy = gridy;

                        tmx += i;
                        tmy += j;

                       // ROS_INFO("gridx , gridy is : %d, %d", gridx, gridy);
                        //地图内且未访问过
                        tempindex = GetIndex(tmx, tmy);//地图索引
                        //未越界且之前没访问过
                        if(BeyondGridMapScope(tmx, tmy) &&
                        (my_cellmap_->grid + tempindex)->vist == VISTMARK::NOTMARK)
                        {
                         //ROS_INFO("FIND not mark");   
                            (my_cellmap_->grid + tempindex)->vist = VISTMARK::MARKED;

                            //如果不是占用 则加入容器，因为这个容器保存一个占用栅格周围8临域未被占用的栅格，用于膨胀，要设置成占用
                            if((my_cellmap_->grid +tempindex)->cell_state_== CELLSTATE::FREE)
                                checkindex.push_back(tempindex);//这里保存的栅格是后面要设置为膨胀层的栅格，先放到容器内，后面统一赋值为膨胀层像素值
                        }
                    }
                }
            }
          //  auto tmpindex = GetIndex(gridx, gridy);
           // VistEightNeighbourHood(tmpindex, checkindex);
        }
   }

}

void AStarTest::PubCostMap2d(const nav_msgs::OccupancyGridConstPtr& msg)
{
     nav_msgs::OccupancyGrid costmap;
    

    costmap.header.stamp = ros::Time::now();
    costmap.header.frame_id = world_frame_;
    costmap.info.resolution = msg->info.resolution;
    costmap.info.width = msg->info.width;
    costmap.info.height = msg->info.height;
    costmap.info.origin = msg->info.origin;

    for(int index = 0; index <  msg->info.width * msg->info.height ; index++)
    {
       costmap.data.push_back((my_cellmap_->grid + index)->cell_state_);

       //ROS_INFO("%d", (my_cellmap_->grid + index)->cell_state_);
     
    }

    ROS_INFO("PUBLISH COSTMAP!!!" );

    costmap_pub_.publish(costmap);


}


void AStarTest::VisAstarPath(nav_msgs::Path& msg)
{

    visualization_msgs::Marker path_vis;
    path_vis.header.stamp =  ros::Time::now();
    path_vis.header.frame_id = world_frame_;
    path_vis.id = 0;
    path_vis.type = visualization_msgs::Marker::LINE_STRIP;
    path_vis.scale.x = 0.2;
    path_vis.scale.y = 0.2;
    path_vis.scale.z = 0.2;
    path_vis.pose.orientation.x = 0.0;
    path_vis.pose.orientation.y = 0.0;
    path_vis.pose.orientation.z = 0.0;
    path_vis.pose.orientation.w = 1.0;

    path_vis.color.a = 1.0;
    path_vis.color.r = 0.0;
    path_vis.color.g = 1.0;
    path_vis.color.b = 0.0;

    geometry_msgs::Point pt;

    path_vis.points.reserve(msg.poses.size());
    for(auto pos : msg.poses)
    {
        pt.x = pos.pose.position.x;
        pt.y = pos.pose.position.y;
        pt.z = pos.pose.position.z;

        path_vis.points.push_back(pt);

    }

   if(path_vis.points.size()>1) path_vis_pub_.publish(path_vis);

}

void AStarTest::timerCallback(const ros::TimerEvent& event) {

        
        maxx_path_time_--;
        //ROS_INFO("Timer  work --");
        if(getNewGoal_ && maxx_path_time_ < 1)
        {

            ROS_INFO("Timer fired!");
            overtiem_ = true;//规划超时
        }
       


}

//TODO  
AStarTest::AStarTest(ros::NodeHandle  nh, ros::NodeHandle private_nh):nh_(nh), private_nh_(private_nh)
{
    ROS_INFO("constructor function");
    
    private_nh_.getParam("base_frame", base_frame_);
    private_nh_.getParam("world_frame", world_frame_);
    private_nh_.getParam("inflation_radius", inflation_radius_);
    private_nh_.getParam("dis_occ", dis_occ_);
    private_nh_.getParam("map_topic", map_topic_);
    private_nh_.getParam("goal_topic", goal_topic_);


    //timer_ = nh_.createTimer(ros::Duration(1.0), &AStarTest::timerCallback, this); // 1秒触发一次

    my_cellmap_ = new map_t();

    my_cellmap_->grid = (Node*)nullptr;


    startpoint_init_ = false;
    use_location_startpoint_ = false;
    double yaw_type;
    geometry_msgs::Quaternion quat;

   //no use
    base_pose_sub_ = nh_.subscribe("/pose_stamped_odom", 10, &AStarTest::baseposeCallback, this);



    local_pub = nh_.advertise<std_msgs::Float32MultiArray>("/curr_state", 1, this);//pubish to python casadi mpc
    referance_path_pub = nh_.advertise<std_msgs::Float32MultiArray>("/surf_predict_pub",1, this);//pubish to python casadi mpc

    //可视化全局路径
    path_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/global_path_vis", 5, this);

   // map_sub = m_nh->subscribe("/yq_costmap2d", 5, &AStarTest::map_callback, this);//
    map_sub = nh_.subscribe(map_topic_, 1, &AStarTest::map_callback, this);//
    
    goal_sub = nh_.subscribe(goal_topic_, 10, &AStarTest::goal_callback, this);
  //  m_pub = m_nh->advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
    path_pub = nh_.advertise<nav_msgs::Path>("/global_path", 1, this);

    costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/costmap", 1, this);

   
    // if(!use_location_startpoint_)
    // {
    //    // param_nh.getParam("start_point_x", start_point_.pose.position.x);
    //    // param_nh.getParam("start_point_y", start_point_.pose.position.y);
    //    // param_nh.getParam("start_point_z", start_point_.pose.position.z);
    //   //  param_nh.getParam("angle", yaw_type);

        

    //     quat = tf::createQuaternionMsgFromYaw(yaw_type);
    //     start_point_.pose.orientation.x = quat.x;
    //     start_point_.pose.orientation.y = quat.y;
    //     start_point_.pose.orientation.z = quat.z;
    //     start_point_.pose.orientation.w = quat.w;

    //     std::cout<<"start_point_x"<< start_point_.pose.position.x<<std::endl;
    //     std::cout<<"start_point_y"<< start_point_.pose.position.y<<std::endl;

    // }

    getNewGoal_ = false;//新的目标点

    reachGoal_ = false;//已经到达目标点
    havedMap_ = false;

   // planning_time_ = m_nh->createTimer(ros::Duration(1.0), &AStarTest::PlanningLoop, this);
   //   std::cout<<"reachGoal_"<<reachGoal_<<std::endl;


    timeout_handle_th_ = std::thread(&AStarTest::TimeoutHandle, this, nh_ );
    timeout_handle_th_.detach();
}


void AStarTest::TimeoutHandle(ros::NodeHandle nh)
{

    timer_ = nh.createTimer(ros::Duration(1.0), &AStarTest::timerCallback, this);
    ros::Rate rate(50);
    while (ros::ok()) {
        ros::spinOnce();  // 必须要调用才能处理回调
        rate.sleep();
    }


}

void AStarTest::PlanningLoop(void)
{

   // ROS_INFO("PLanning Loop");
  
    //std::cout<<"reachGoal_"<<reachGoal_<<std::endl;
   // std::cout<<"havedMap_"<<havedMap_<<std::endl;
    
   if( havedMap_==true && getNewGoal_)
   {
        nav_msgs::Path path_node;
        geometry_msgs::PoseStamped cur_pose;
        ros::Time cur_time;
        
        Node *now_node;
        bool search_result ;

       // ROS_INFO("Start PLanning Loop");

        std::vector<geometry_msgs::PoseStamped> move_plan;

       // goal_point_ = *m_goal;
      //  getNewGoal_ = true;//
    
      //  std::cout<<"x is"<<goal_point_.pose.position.x<<std::endl;
       // std::cout<<"y is"<<goal_point_.pose.position.y<<std::endl;

        get_node(goal_point_, p_goal);

        //  if(UpdateBasePose(start_point_))
        //      search_result =  path_search(start_point_, goal_point_, move_plan);
        //  else 
        //      search_result = false;
        
        if(getNewGoal_)
        {
            ROS_INFO("getNewGoal_ : %d",getNewGoal_);
            overtiem_ = false;
          
      
          
            if(UpdateBasePose(start_point_))
            {  
            
                search_result =  path_search(start_point_, goal_point_, move_plan);

            }
                
            else 
            {
                search_result = false;

            }
            

        } 
       // ROS_INFO("search_result : %d",search_result);
       // ROS_INFO("111111111111111111111111111111111!");

        if(getNewGoal_ && search_result == true )//准备发布path
        {
            ROS_INFO("Global Planning Success !!!");

            ROS_INFO("Waitting Bspline Smooth Path ......");
            maxx_path_time_ = 30;
             overtiem_ = false;
            search_result = false;
            getNewGoal_ = false;

            auto it = &p_goal;//从目标点开始,通过前驱节点回到起始点 将路径打印发布出来
            path_node.poses.clear();
            while(it != nullptr)
            {
                    cur_time = ros::Time::now();
                   
                    now_node = it ;


                    path_node.header.stamp = cur_time;
                    path_node.header.frame_id = world_frame_;

                    cur_pose.header.stamp = cur_time;
                    cur_pose.pose.position.x = now_node->x_index * resolution_ + m_map_.info.origin.position.x;//根据索引求map下实际坐标
                    cur_pose.pose.position.y = now_node->y_index * resolution_ + m_map_.info.origin.position.y;
                    cur_pose.pose.position.z = 0;

                    path_node.poses.push_back(cur_pose);
                

                    it = it -> prev_node;//取出该点得父节点

                    //std::cout<<"x is "<<cur_pose.pose.position.x<<std::endl;
                    //std::cout<<"y is "<<cur_pose.pose.position.y<<std::endl;

            }
            std::reverse(path_node.poses.begin(), path_node.poses.end());//要得到从起点到终点的路径

        //ROS_INFO("BSPLINE ");


            #if 1    //bspline 
            /**********************************/
            std::vector<Eigen::Vector2d> roarse_path;
            roarse_path.clear();
            roarse_path.reserve(path_node.poses.size());

            for(int j = 0; j <  path_node.poses.size(); j++)
            {
                roarse_path.push_back(Eigen::Vector2d( path_node.poses[j].pose.position.x, path_node.poses[j].pose.position.y));

            }
           if(roarse_path.size() > 2)  roarse_path = getBspline(roarse_path);
           else 
           {
             ROS_INFO("Bspling Smooth Failed!!!");

             return ;
           }



           
            path_node.poses.clear();
            path_node.poses.reserve(roarse_path.size());
            geometry_msgs::PoseStamped bs_pose;
             for(int k = 0; k <  roarse_path.size() ; k++)
             {

               
                bs_pose.header.stamp = cur_time;
                bs_pose.pose.position.x = roarse_path[k](0);//根据索引求map下实际坐标
                bs_pose.pose.position.y = roarse_path[k](1);
                bs_pose.pose.position.z = 0;

                path_node.poses.push_back(bs_pose);

             }
            
            //路径最后一个点的z值传递目标点的航向
            path_node.poses[path_node.poses.size()-1].pose.position.z = goal_yaw_;



             /**********************************/
        

            VisAstarPath(path_node);//可视化路径

            referance_path_.data.clear();
            referance_path_.data.reserve(roarse_path.size());
            //std::reverse(roarse_path.begin(), roarse_path.end());
            for(int i  = 0; i < roarse_path.size(); i++)
            {

                referance_path_.data.push_back(roarse_path[i](0));
                referance_path_.data.push_back(roarse_path[i](1));
                referance_path_.data.push_back(0);
                referance_path_.data.push_back(0);
                referance_path_.data.push_back(0);
            }
            referance_path_pub.publish(referance_path_);
            ROS_INFO("Smooth Finish!!!");
             #endif

            path_pub.publish(path_node);//发布全局路径，nav_path

            
            reachGoal_ = false;


             referance_path_.data.clear();

            for(int i = 0; i < num_map_; i++)
            {
                my_cellmap_->grid[i].F = 0;
                my_cellmap_->grid[i].G = 0;
                my_cellmap_->grid[i].H = 0;
                my_cellmap_->grid[i].prev_node = nullptr;

            }
            
            //释放搜索时申请的内存
         //   realsenode(openlist);
          //  realsenode(closelist);//将点从列表里删除，为下一次规划准备
        }
        else if(getNewGoal_ && search_result == false)
        {

            ROS_INFO("Global Planning Failed,Try  it again");
            maxx_path_time_ = 30;
             overtiem_ = false;
            getNewGoal_ = false;
            reachGoal_ = false;
            referance_path_.data.clear();

            for(int i = 0; i < num_map_; i++)
            {
                my_cellmap_->grid[i].F = 0;
                my_cellmap_->grid[i].G = 0;
                my_cellmap_->grid[i].H = 0;
                my_cellmap_->grid[i].prev_node = nullptr;

            }
           
           // realsenode(openlist);//将点从列表里删除，为下一次规划准备
           // realsenode(closelist);//将点从列表里删除，为下一次规划准备

            //openlist.clear();
            

            //ROS_INFO("Search Fail !!!");
        }

    }


}


//TODO  no use
void AStarTest::baseposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs)
{
    //if(startpoint_init_ == false)
    {
        //start_point_.pose.position.x = msgs->pose.pose.position.x;
       // start_point_.pose.position.y = msgs->pose.pose.position.y;
        startpoint_init_ = true;
        //std::cout<<"start_point_x"<< start_point_.pose.position.x<<std::endl;
       // std::cout<<"start_point_y"<< start_point_.pose.position.y<<std::endl;
    }
   
    start_point_ = *msgs;


    /***************publish to python casadi mpc**********************/
    std_msgs::Float32MultiArray local_car;
    local_car.data.push_back(msgs->pose.position.x);
    local_car.data.push_back(msgs->pose.position.y);
    local_car.data.push_back(msgs->pose.position.z);
    //欧拉角转四元数
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msgs->pose.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换
    local_car.data.push_back(yaw);
    local_car.data.push_back(roll);
    local_car.data.push_back(pitch);
    local_pub.publish(local_car);
    /***************publish to python casadi mpc**********************/




    //ROS_INFO("PUBLISH  local_car !!!!!!!!!!!");






    
}



//TODO  定位
 bool AStarTest::UpdateBasePose( geometry_msgs::PoseStamped& point)
 {


    tf::StampedTransform transform;
    tf_listener_.waitForTransform(world_frame_, base_frame_, ros::Time(0), ros::Duration(1.0));
    try{

        tf_listener_.lookupTransform(world_frame_, base_frame_, ros::Time(0), transform );
    }
    catch(tf::LookupException& ex)
    {
        
        //  ROS_ERROR("%s",ex.what());
          return false;
    }

    point.pose.position.x = transform.getOrigin().getX();
    point.pose.position.y = transform.getOrigin().getY();

    return true;

 }






//TODO  
void AStarTest::goal_callback(const geometry_msgs::PoseStamped::ConstPtr& m_goal)
{

    nav_msgs::Path path_node;
    geometry_msgs::PoseStamped cur_pose;
    ros::Time cur_time;
    
    bool search_result;

    ROS_INFO("SUB GOAL");

    openlist.clear();
    std::vector<geometry_msgs::PoseStamped> move_plan;

    goal_point_ = *m_goal;
    getNewGoal_ = true;//

    //获取目标点航向
    tf2::Quaternion quat(
    m_goal->pose.orientation.x,
    m_goal->pose.orientation.y,
    m_goal->pose.orientation.z,
    m_goal->pose.orientation.w); 

    double roll, pitch, yaw;
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    goal_yaw_ = yaw;
    //获取目标点航向
   
    std::cout<<"x is"<<goal_point_.pose.position.x<<std::endl;
    std::cout<<"y is"<<goal_point_.pose.position.y<<std::endl;
    std::cout<<"yaw is"<<goal_yaw_<<std::endl;
    get_node(goal_point_, p_goal);


   // ROS_INFO("p_goal state is : %d\n", (p_goal.cell_state_));

    //************************* */
    if((p_goal.cell_state_ == CELLSTATE::OCCUPY) ||  (p_goal.cell_state_ == CELLSTATE::INFLATION_OCCUPY) )//目标点选的在障碍物或膨胀区内，不可规划
    {

        ROS_INFO("Select Goal is Inavalible!!!!!!!! ");
        getNewGoal_ = false;//

        UpdateBasePose(goal_point_);//再将目标点设置为当前定位

        return;
    }


    maxx_path_time_ = 30;
    overtiem_ = false;
    //   ros::TimerEvent tevent;
    // PlanningLoop(tevent);
    #if 0 //only astar 不在回调函数中进行算法

        if(UpdateBasePose(start_point_))
            search_result =  path_search(start_point_, goal_point_, move_plan);
        else 
            search_result = false;
        

    if(search_result == true)//准备发布path
    {
        auto it = &p_goal;//从目标点开始,通过前驱节点回到起始点 将路径打印发布出来
        while(it != nullptr)
        {
                cur_time = ros::Time::now();
                Node *now_node;
                now_node = it ;


                path_node.header.stamp = cur_time;
                path_node.header.frame_id = "/map";

                cur_pose.header.stamp = cur_time;
                cur_pose.pose.position.x = now_node->x_index * resolution_ + m_map_.info.origin.position.x;//根据索引求map下实际坐标
                cur_pose.pose.position.y = now_node->y_index * resolution_ + m_map_.info.origin.position.y;
                cur_pose.pose.position.z = 0;

                path_node.poses.push_back(cur_pose);
            

                it = it -> prev_node;//取出该点得父节点

                //std::cout<<"x is "<<cur_pose.pose.position.x<<std::endl;
                //std::cout<<"y is "<<cur_pose.pose.position.y<<std::endl;

        }
        std::reverse(path_node.poses.begin(), path_node.poses.end());//要得到从起点到终点的路径
        path_pub.publish(path_node);

        //释放搜索时申请的内存
        realsenode(openlist);
        //realsenode(closelist);//将点从列表里删除，为下一次规划准备
    }
    else
    {
        realsenode(openlist);//将点从列表里删除，为下一次规划准备
        //realsenode(closelist);//将点从列表里删除，为下一次规划准备

        ROS_INFO("Search Fail !!!");
    }
    
   #endif
}



//TODO  
bool AStarTest::get_node(int x_index, int y_index, Node& p_node)
{
    p_node.x_index = x_index;
    p_node.y_index = y_index;

    auto index_t = p_node.x_index + p_node.y_index * m_map_.info.width;
    //auto index_t = p_node.x_index * m_map_.info.width + p_node.y_index;
    p_node.index = index_t;
    p_node.cell_state_ =  (my_cellmap_->grid + index_t)->cell_state_;
    if( m_map_.data[index_t] == 0)
    {
        p_node.visiable = 1;//free
        //ROS_INFO("Free");

    }
    else 
    {
        p_node.visiable = 0;//occupy or unkown
        //ROS_INFO("Occupy Grid");
    }



p_node.H = abs((abs(goal_point_.pose.position.y - m_map_.info.origin.position.y) / resolution_)- p_node.y_index) +\
abs((abs(goal_point_.pose.position.x - m_map_.info.origin.position.x) / resolution_)- p_node.x_index);

//std::cout<<"p_node.H"<<p_node.H<<std::endl;
 return 0;
}


//TODO  
bool AStarTest::get_node(geometry_msgs::PoseStamped& p_point, Node& p_node)
{
    p_node.x_index = (p_point.pose.position.x - m_map_.info.origin.position.x) / resolution_;
    p_node.y_index = (p_point.pose.position.y - m_map_.info.origin.position.y) / resolution_;

    auto index_t =  p_node.x_index + p_node.y_index * m_map_.info.width;
    //auto index_t = p_node.x_index * m_map_.info.width + p_node.y_index;
    p_node.index = index_t;
    p_node.cell_state_ =  (my_cellmap_->grid + index_t)->cell_state_;
    if( m_map_.data[index_t] == 0)
    {
        p_node.visiable = 1;//free
       
        //ROS_INFO("Free");

    }
    else 
    {
        p_node.visiable = 0;//occupy or -1unkown
      
        //ROS_INFO("Occupy Grid");
    }

    p_node.H = abs((abs(goal_point_.pose.position.y - m_map_.info.origin.position.y) / resolution_)- p_node.y_index) +\
    abs((abs(goal_point_.pose.position.x - m_map_.info.origin.position.x) / resolution_)- p_node.x_index);

    //std::cout<<"p_node.H"<<p_node.H<<std::endl;
    return true;
}



//TODO  
//find is or not free grid
bool AStarTest::is_free(Node* test_node)
{
    if(test_node->cell_state_== CELLSTATE::FREE)return true;
    else if(test_node->cell_state_ == CELLSTATE::OCCUPY || test_node->cell_state_ == CELLSTATE::INFLATION_OCCUPY )return false;
    
    return false;

}




//TODO  
bool AStarTest::find_node(std::vector<Node*>nodelist, Node* t_node)
{
    int num;
    for(auto node : nodelist )
    {
        if(node->x_index == t_node->x_index && node->y_index == t_node->y_index)
        {
            // num++;
            // std::cout<<"num"<<num<<std::endl;
            // std::cout<<"node->x_index"<<node->x_index<<std::endl;
            // std::cout<<"nt_node->x_index"<<t_node->x_index<<std::endl;
            // std::cout<<"node->y_index"<<node->y_index<<std::endl;
            // std::cout<<"t_node->y_index"<<t_node->y_index<<std::endl;
            return true;
        }
       
    }
    return false;
}


//no use
void AStarTest::HandleMap(const nav_msgs::OccupancyGrid::ConstPtr& static_map, map_t* &cellmap)
{

     //栈顶是栈中距离最小的栅格
    std::priority_queue<Node> candidatecell;//重载为小顶堆  即栈顶是距离值最小的栅格
    std::cout<<"candidatecell.size() : "<<candidatecell.size()<<std::endl;
   
    for(int i = 0; i < num_map_; i++)
    {
        if(static_map->data[i] == CELLSTATE::OCCUPY)//要先膨胀的地图
        {
           cellmap->grid[i].nearest_occ_dist_ = 0;
           cellmap->grid[i].index = i;
         //  cellmap->grid[i].cell_state_ = CELLSTATE::INFLATION_OCCUPY;
           cellmap->grid[i].marked_ = 1;
           cellmap->grid[i].cell_state_ = CELLSTATE::OCCUPY;
           cellmap->grid[i].source_index_ = i;

         //  candidatecell.push(cellmap->grid[i]);
            
        }
        else if(static_map->data[i] == CELLSTATE::FREE)//开始计算空闲栅格到最近的障碍物栅格距离
        {
           cellmap->grid[i].nearest_occ_dist_ = (double)std::max(cellmap->height, cellmap->width) * cellmap->resolution;
           cellmap->grid[i].index = i;
           cellmap->grid[i].cell_state_ = CELLSTATE::FREE;
           cellmap->grid[i].marked_ = 0; 
        }
        
    }



}




//计算每个空闲栅格距离最近的障碍物栅格的实际距离
//TODO  
void AStarTest::calCacheFreeCellDist(const nav_msgs::OccupancyGrid::ConstPtr& static_map, map_t* &cellmap)
{
    //栈顶是栈中距离最小的栅格
    std::priority_queue<Node> candidatecell;//重载为小顶堆  即栈顶是距离值最小的栅格
    std::cout<<"candidatecell.size() : "<<candidatecell.size()<<std::endl;
   
    for(int i = 0; i < num_map_; i++)
    {

        
        if(static_map->data[i] == CELLSTATE::OCCUPY)//要先膨胀的地图
        {
           cellmap->grid[i].nearest_occ_dist_ = 0;
           cellmap->grid[i].index = i;
         //  cellmap->grid[i].cell_state_ = CELLSTATE::INFLATION_OCCUPY;
           cellmap->grid[i].marked_ = 1;
           cellmap->grid[i].cell_state_ = CELLSTATE::OCCUPY;
           cellmap->grid[i].source_index_ = i;

           candidatecell.push(cellmap->grid[i]);
            
        }
        else if(static_map->data[i] == CELLSTATE::FREE)//开始计算空闲栅格到最近的障碍物栅格距离
        {
           cellmap->grid[i].nearest_occ_dist_ = (double)std::max(cellmap->height, cellmap->width) * cellmap->resolution;
           cellmap->grid[i].index = i;
           cellmap->grid[i].cell_state_ = CELLSTATE::FREE;
           cellmap->grid[i].marked_ = 0; 
        }
        
    }

    unsigned int tmp_index = 0;

    //开始计算空闲栅格距离最近障碍物的距离
   // std::cout<<"candidatecell.size() : "<<candidatecell.size()<<std::endl;

    #if 1
    while(!candidatecell.empty())
    {
        Node current_node = candidatecell.top();//弹出栈顶
        candidatecell.pop();
        std::cout<<"candidatecell.size() : "<<candidatecell.size()<<std::endl;
      
        //开始八邻域搜索
        for(int i = -1; i <= 1; i++)
        {
            for(int j = -1; j <= 1; j++)
            {
                if(i != 0 && j != 0)//不是栅格本身
                {
                   
                    unsigned int x_index =0,  y_index=0;

                    GetGridCoordinateFromIndex(x_index, y_index, current_node.index);
                    x_index = x_index + i;
                    y_index = y_index + j;

                    if(!BeyondGridMapScope(x_index, y_index))
                    {

                       // candidatecell.pop();
                        continue;
                    }
                   

                    tmp_index = GetIndex(x_index, y_index);

                    //如果已经计算过了距离 则开始下一次
                    if(cellmap->grid[tmp_index].marked_ == 1)
                    {
                        //candidatecell.pop();
                        continue;
                    }

                    //
                    double square = (double)sqrt(i*i + j*j);
                    //当前的栅格是障碍物栅格
                    if(cellmap->grid[current_node.index].cell_state_ == CELLSTATE::OCCUPY)
                    {
                        if(cellmap->grid[tmp_index].cell_state_ == CELLSTATE::FREE) 
                        {
                           
                            cellmap->grid[tmp_index].nearest_occ_dist_ = square * cellmap->resolution;
                            //std::cout<<"distance is : "<<cellmap->grid[tmp_index].nearest_occ_dist_<<std::endl;


                            cellmap->grid[tmp_index].marked_ = 1;
                            cellmap->grid[tmp_index].source_index_ = current_node.index;
                            cellmap->grid[tmp_index].index = tmp_index;

                            candidatecell.push(cellmap->grid[tmp_index]); 
                        }
                    }
                    else if(cellmap->grid[current_node.index].cell_state_ == CELLSTATE::FREE)
                    {
                        //当前的栅格是前面遍历时新加入到优先队列中的空闲栅格

                        //说明障碍物栅格已经遍历完了，开始遍历开始搜索时加入的空闲栅格了
                        cellmap->grid[tmp_index].nearest_occ_dist_ = square * cellmap->resolution + 
                        cellmap->grid[current_node.index].nearest_occ_dist_;  

                        //std::cout<<"distance is : "<<cellmap->grid[current_node.index].nearest_occ_dist_<<std::endl;

                        cellmap->grid[tmp_index].marked_ = 1;
                        cellmap->grid[tmp_index].source_index_ = current_node.source_index_;//源栅格是障碍物栅格，虽然是间接的
                        cellmap->grid[tmp_index].index = tmp_index;

                        candidatecell.push(cellmap->grid[tmp_index]); 
                    }
                  
                }
            }
            // std::cout<<"candidatecell.size() : "<<candidatecell.size()<<std::endl;
        }
       
    }
    #endif
    
}






//TODO  
bool AStarTest::path_search( geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    Node* curr_ptr;
   
    get_node(start, p_start);
    
    p_start.G = 0;
    p_start.F = p_start.G + p_start.H;
    p_start.prev_node = nullptr;

    openlist.clear();

    openlist.push_back(&p_start);

    while(openlist.size() != 0)
    {
     
        if(overtiem_) 
        {
            overtiem_ = false;
           return false;
        }
        

        //滑动窗口 开放表里保持最新的100个点 多了就从头部删除，但是数据还在只是从容器中删除了
        // if(openlist.size() > num_map_)
        // {

        //    // openlist.erase(openlist.begin(), openlist.begin());

        // }

        curr_ptr = openlist.front();//

        //find min F node 获取此时开发列表里F最小的节点
        for(std::vector<Node*>::iterator node = openlist.begin(); node != openlist.end(); node++)
        {
            if(curr_ptr == nullptr)
            {  
                curr_ptr = *node;
            }
            else if(curr_ptr->F >= (*node)->F)
            {
                curr_ptr = *node;

            }
        }

        //cur_time = ros::Time::now();
        //最小的F节点从开发列表里删除加入到关闭列表里
      //  curr_ptr->marked_ = 1;
        openlist.erase(std::find(openlist.begin(), openlist.end(), curr_ptr));

     // closelist.push_back(curr_ptr);
        // path_node.header.stamp = cur_time;
        // path_node.header.frame_id = "/map";
        // cur_pose.header.stamp = cur_time;
        // cur_pose.pose.position.x = curr_ptr->x_index * resolution_ + m_map_.info.origin.position.x;
        // cur_pose.pose.position.y = curr_ptr->y_index * resolution_ + m_map_.info.origin.position.y;
        // cur_pose.pose.position.z = 0;
        // path_node.poses.push_back(cur_pose);
        //path_pub.publish(path_node);
        //这里定义到达目标点的精度标准
       
        if( std::sqrt(std::pow(curr_ptr->x_index - p_goal.x_index,2)  +  
            std::pow(curr_ptr->y_index - p_goal.y_index,2)) <  
              std::sqrt(SEARCH_THRESHOLD*SEARCH_THRESHOLD) )
        {
            int delta_x, delta_y;

            delta_x = p_goal.x_index - curr_ptr->x_index;
            delta_y = p_goal.y_index - curr_ptr->y_index;

            //std::cout<<"delta_x is "<<delta_x<<std::endl;//计算到达精度
            //std::cout<<"delta_y is"<<delta_y<<std::endl;

            p_goal.prev_node = curr_ptr;//目标点的前驱点设置为现在的点


           // closelist.push_back(&p_goal);//将目标点放入关闭队列  搜索结束

          //  ROS_INFO("Search Finish");
            return true;
         
        }


       //开始搜索该节点周围得8个栅格，周围8个格子考虑效率 步长为5个栅格
        for(int i = -SEARCH_STEP; i < SEARCH_STEP + 1; i = i + SEARCH_STEP)
        {
            for(int j = -SEARCH_STEP; j < SEARCH_STEP + 1; j = j + SEARCH_STEP)
            {
                if(i == 0 && j == 0 )// current//自己不算在内
                {
                    continue;
                }

                //if(i!=0 && j!=0)continue;//不走斜线 只考察四邻域
                
                //Node* s_node = new Node;//创建一个邻居节点


               int grid_index =((curr_ptr->y_index)+j) * GridSize_X_ + (curr_ptr->x_index)+ i;
               Node* s_node;
               if(grid_index < num_map_) 
               {
                //ROS_INFO("New create node");
                s_node = &(my_cellmap_->grid[grid_index]);//从之前创建的内存中获取 指针，
               }
               else continue;
              
               
                get_node((curr_ptr->x_index) + i, (curr_ptr->y_index) + j, *s_node);
               // ROS_INFO("PATH_SEARCH RUN");
                if(!is_free(s_node))
                {

                    continue;

                }

                //查询该栅格距离最近障碍物的距离，小于阈值则不采用该栅格,防止路径在占用栅格稀疏区域时导致路径进入稀疏的占用栅格附件
                if((my_cellmap_->grid[s_node->index].nearest_occ_dist_ < dis_occ_))
                {
                    continue;
                }
             
                

                
                
                /*这里要注意，代价的计算应该与实际的步长一致，不能随便说是10，14, 因为还有一个曼哈段距离H,H的计算是根据实际步长得来的，F的值是G+H, 
                这样两个的计算就不是基于给的步长这个尺度了，所以前后左右的是一个步长，对角线的是1.4个步长（即根号2个步长）*/
           
               


                    //传统A*贴边
               double G_cost = curr_ptr->G + ((i != 0 && j != 0) ? (SEARCH_STEP * 1.4) : SEARCH_STEP); 
                 
                 //对中效果
                //  double G_cost = curr_ptr->G + ((i != 0 && j != 0) ? (1 * SEARCH_STEP * 1.4 ) : SEARCH_STEP) + 4*(1-1/(1+exp(-my_cellmap_->grid[s_node->index].nearest_occ_dist_)));
                //double G_cost = curr_ptr->G + ((i != 0 && j != 0) ? (1 * SEARCH_STEP * 1.4 ) : SEARCH_STEP) + 80000* exp(-my_cellmap_->grid[s_node->index].nearest_occ_dist_);
               
               
               
             
                //std::cout<<"G_cost"<<G_cost<<std::endl;

                //bool f_node = find_node(openlist, s_node);//该栅格是不是在开放列表
                //std::cout<<"F_Node is "<<f_node<<std::endl;
                //if(f_node == false)//如果不在开放列表,计算F ,并将本节点设为父节点
                if(s_node->G == 0)
                {
                    s_node->G = G_cost;
                    s_node->F = s_node->G + 1 * s_node->H;
                   // s_node->F = s_node->G ;
                    s_node->prev_node = curr_ptr;

                    openlist.push_back(s_node);
                   // ROS_INFO("Add Node");
                }
                else if(G_cost < s_node->G)//如果已经开放列表,current_ptr这个点到达起始点得G更小，就更新G，并从新计算F，然后前驱点设置为current_ptr
              //  else if(G_cost < s_node->nearest_occ_dist_)
                {
                    s_node->G = G_cost;
                    s_node->F = s_node->G + 1 * s_node->H;
                    //s_node->F = s_node->G ;
                    s_node->prev_node = curr_ptr;
                }
            }

        }
    } 

    
    // for(std::vector<Node*>::iterator node = closelist.begin(); node != closelist.end(); node++)
    // {

    //     std::cout<<"x_index is:"<<(*node)->x_index<<"y_index is:"<<(*node)->y_index<<std::endl;

    // }
  
    return false;   
}


////TODO  
 void AStarTest::realsenode(std::vector<Node*>& list_node)
 {
    // for(int i = 0; i < list_node.size(); i++)
    // {
    //   if(list_node[i] != nullptr)  delete list_node[i];
    // }

     
    if(list_node.size() > 0)
    {
       // ROS_INFO("list_node.size() IS %d", list_node.size());
        delete[] list_node[0];
        
    }
   

    list_node.clear();

    ROS_INFO("Delete sucess");
    //  for(auto it = list_node.begin(); it != list_node.end();)//注意这里不要使用++，erase本身就返回下一个的迭代器
    //  {
         
    //     it = list_node.erase(it);
    //     delete *it;
 
    //  }

 }






//TODO  
