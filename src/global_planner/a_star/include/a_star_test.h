#ifndef A_STAR_TEST_H
#define A_STAR_TEST_H

#include <iostream>
#include <string>
#include "math.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <vector>
#include <functional>
#include <set>
#include <algorithm>
#include"visualization_msgs/MarkerArray.h"
#include "nav_msgs/Path.h"
#include "time.h"

#include "ros/timer.h"
#include "ros/console.h"
#include <queue>
#include "std_msgs/Float32MultiArray.h"

#include <nav_msgs/Odometry.h>
#include <thread>
#include <mutex>



#define SEARCH_STEP   1

#define SEARCH_THRESHOLD  SEARCH_STEP * 1
//这里的Node理解为栅格
struct Node{


Node(){
G = 0;
H = 0;
F = 0;

};

double G;
double H;
double F;



unsigned int index;//这个栅格本身的索引  
unsigned int x_index;
unsigned int y_index;
Node* prev_node = nullptr; 
int visiable; //0-free 100-occupy -1-unkown
unsigned char marked_;//是否在计算距离最近障碍物距离时已经访问过
 int16_t cell_state_ = -1;//free=1  occ=255;

double nearest_occ_dist_;//空闲栅格距离最近的障碍物的实际距离

int source_index_;//计算最近距离时，这个栅格距离最近的障碍物栅格的索引

bool vist;

bool operator<(const Node& a )const {
 return this->nearest_occ_dist_ > a.nearest_occ_dist_;
}

};


// bool Node::operator<(const Node& a )
// {
   
// }


////////////////////////////////////
//地图类型
 struct map_t{

    int height, width;
    double orignx, origny;
    double resolution;
    Node * grid;
   
   
};



/////////////////////////////////////



class AStarTest{


    
public:



//栅格的访问情况
 enum VISTMARK{
NOTMARK =0,
MARKED = 2
};

    //AStarTest();

    AStarTest(ros::NodeHandle nh, ros::NodeHandle private_nh);

    ~AStarTest();

    enum CELLSTATE{
        FREE = 0,
        INFLATION_OCCUPY = 180,
        OCCUPY = 100,
        UNKNOWN = -1

    };




    bool get_node(geometry_msgs::PoseStamped& p_point, Node& p_node);
    bool get_node(int x_index, int y_index, Node& p_node);

    bool path_search( geometry_msgs::PoseStamped& start,  geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& p_map);

    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& m_goal);

    void baseposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs); 

    bool UpdateBasePose( geometry_msgs::PoseStamped& point);//use tf to update robot pose

    bool is_free(Node* test_node);

    bool find_node(std::vector<Node*>nodelist, Node* t_node);

    void realsenode(std::vector<Node*>& list_node);

    void calCacheFreeCellDist(const nav_msgs::OccupancyGrid::ConstPtr& static_map, map_t* &cellmap);

    void HandleMap(const nav_msgs::OccupancyGrid::ConstPtr& static_map, map_t* &cellmap);

    void PlanningLoop(void);




    //#define getindex(x, y)  x * GridSize_X_ + y;
//Grid to Map Index  获得地图索引
//map里的data[index], 需要得到地图索引查询
inline unsigned int  GetIndex(unsigned int& gridX, unsigned int& gridY)
{
    return gridY * GridSize_X_ + gridX;
}



//Map index to Grid   由地图索引得到栅格坐标，注意先计算gridy 
inline void GetGridCoordinateFromIndex(unsigned int& gridx, unsigned int& gridy, unsigned int& index)
{
    gridy = index / GridSize_X_;
    gridx = index - gridy * GridSize_X_;
   

}


//World to Grid   世界坐标转栅格坐标
inline void GetGridCoordinateFromWorld(double& x, double& y, unsigned int& gridx, unsigned int& gridy)
{
    gridx = static_cast<unsigned int>((x - origin_X_) / resolution_);
    gridy = static_cast<unsigned int>((y - origin_Y_) / resolution_);
}

//Grid To world  栅格坐标转世界坐标
inline void GridCoordinateToWorld(unsigned int& gx, unsigned int& gy, double& x, double& y)
{
    x = origin_X_ + (gx + 0.5) * resolution_;
    y = origin_Y_ + (gy + 0.5) * resolution_;
}

//栅格坐标是否越界
inline bool BeyondGridMapScope(unsigned int& gridx, unsigned int& gridy)
{
    if(gridx < GridSize_X_ && gridy < GridSize_Y_)
    return true;
    
    return false;
}


void MakeCostmap2d(const nav_msgs::OccupancyGridConstPtr& msg);
void VistEightNeighbourHood(unsigned int& index, std::vector<unsigned int>& checkindex);

void PubCostMap2d(const nav_msgs::OccupancyGridConstPtr& msg);


void VisAstarPath(nav_msgs::Path& msg);

void timerCallback(const ros::TimerEvent& event) ;

void TimeoutHandle(ros::NodeHandle nh);

bool GoalVaild(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:

    nav_msgs::OccupancyGrid m_map_;

    std::vector<int8_t> grid_data;
    //std::vector<uint8_t> grid_data;

    std::vector<Node*> openlist;
    std::vector<Node*> closelist;

    Node p_start ;
    Node p_goal ;

    ros::Subscriber map_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber base_pose_sub_;

    ros::Publisher m_pub;
    ros::Publisher path_pub;
    ros::Publisher local_pub;
    ros::Publisher referance_path_pub;
    ros::Publisher costmap_pub_;
    ros::Publisher path_vis_pub_;
    visualization_msgs::Marker m_path;

    uint32_t num_map_;

    geometry_msgs::PoseStamped start_point_;
    geometry_msgs::PoseStamped goal_point_;

    std::string base_frame_ = "/base_link";
    std::string world_frame_ = "/map";

    tf::TransformListener tf_listener_;

    float resolution_ = 0.05;
    int GridSize_X_, GridSize_Y_ ; //栅格尺度的x y大小
    double origin_X_, origin_Y_;


    bool startpoint_init_;
    bool use_location_startpoint_;

    //std::unordered_map<int, Node>freecelldist_:

    map_t* my_cellmap_;

    //12.29
    bool getNewGoal_;
    bool reachGoal_;
    bool havedMap_;

    ::ros::Timer planning_time_;



    std_msgs::Float32MultiArray referance_path_;

    double inflation_radius_ = 0.3;//代价地图的膨胀半径

    double goal_yaw_;
    
    double dis_occ_ = 0.8;//距离最近障碍物的阈值，在8邻域选择时用于过滤距离机器人太近的栅格

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    std::string map_topic_ = "/map";
    std::string goal_topic_ = "/move_base_simple/goal";

    bool overtiem_ = false;
    int maxx_path_time_ = 20;


    ros::Timer timer_ ; 



    std::thread timeout_handle_th_;
    std::mutex m_mutex_; 

};

















#endif