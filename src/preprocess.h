#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE{AVIA = 1, VELO16, OUST64}; //{1, 2, 3}
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
//枚举类型：表示特征点的类型
//正常点，Nor, 
//可能的平面点，Poss_Plane,
//确定的平面点， Real_Plane, 
//有跨越的平面点，Edge_Jump, 
//边上的平面点，Edge_Plane, 
//线段 这个也许当了无效点？，也就是空间中的小线段？Wire, 
//无效点 程序中未使用ZeroPoint
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
//枚举类型：位置标识
enum Surround{Prev, Next};//前一个/后一个
//枚举类型：表示有跨越边的类型
//Nr_nor, 正常
//Nr_zero,0
//Nr_180,180
//Nr_inf, 无穷大，跳变较远？
//Nr_blind 在盲区？
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

//通过orgtype存放一些激光雷达点的一些其他属性
struct orgtype
{
  double range;//点云在xy平面距离雷达中心的距离
  double dista; //当前点与后一个点之间的距离的平方
  //假设雷达原点为O 前一个点为M 当前点为A 后一个点为N
  double angle[2];//这个是角OAM和OAN的cos直
  double intersect;//这是角MAN的cos值
  E_jump edj[2];//前后两点的类型
  Feature ftype;//点类型
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;//默认为正常点
    intersect = 2;
  }
};
//velodyne数据结构
namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;//4D点坐标类型 添加 XYZ 三维坐标
      float intensity;//强度
      float time;//时间
      uint16_t ring;//点所属的圈数
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW//进行内存对其    // Eigen 宏，用于创建新的对齐操作符
  };
}  // namespace velodyne_ros
//注册velodyne_ros的point类型
//通过 POINT_CLOUD_REGISTER_POINT_STRUCT 宏注册了 velodyne_ros::Point 类型，告诉 PCL 如何解析这种类型的点云数据
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;//强度
      uint32_t t;//时间
      uint16_t reflectivity;//反射率
      uint8_t  ring;//所属圈数
      uint16_t ambient;//没用到
      uint32_t range;//距离
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();//构造函数
  ~Preprocess();//析构函数
  
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);//对livox自定义的livox_ros_driver::CustomMsg::ConstPtr类型数据的处理函数
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);//对ROS的msg格式的处理函数
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);//

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;//定义全部点，边缘点，平面点
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar//用于存储单个激光点的其他的特征，最大128线上的点
  float time_unit_scale;
  //雷达类型，采样间隔，扫描线数，扫描频率
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
  double blind;//定义盲区（会被直接滤除）
  bool feature_enabled, given_offset_time;//是否提取特征点，是否进行时间偏移
  ros::Publisher pub_full, pub_surf, pub_corn;//发布全部点、平面点、角点（边缘点）
    

  private:
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
