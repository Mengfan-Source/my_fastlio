#include "preprocess.h"

#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()//构造函数，在创建类对象时赋值的默认值
  :feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
  //默认：不进行特征提取，雷达类型选择AVIA，盲区距离为0.01 采样间隔为1
{
  inf_bound = 10;//有效点集合，大于10m则是盲区
  N_SCANS   = 6;//多线激光雷达的线数
  SCAN_RATE = 10;//扫描频率10HZ
  group_size = 8;///8个点为一组
  disA = 0.01;//点集合的距离阈值，用于判断是否为平面
  disA = 0.1; // 点集合的距离阈值，用于判断是否为平面，应该是B？？？？？
  p2l_ratio = 225;//点到线的距离阈值，需要大于这个值才能判断组成面
  limit_maxmid =6.25;//中点到左侧的距离变化率范围
  limit_midmin =6.25;//中点到右侧的距离变化率范围
  limit_maxmin = 3.24;//左侧到右侧的距离变化率范围
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;//点与点距离超过两倍则认为遮挡
  edgeb = 0.1;//点与点距离超过0.1m则认为遮挡
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;//三个点如果角度大于172.5度，且比例小于1.2倍，则认为是平面
  given_offset_time = false;//是否提供时间偏移。默认不提供

  jump_up_limit = cos(jump_up_limit/180*M_PI);//角度大于170度的点则跳过，认为在...
  jump_down_limit = cos(jump_down_limit/180*M_PI);//角度小于8度的点跳过
  cos160 = cos(cos160/180*M_PI);//夹角限制
  smallp_intersect = cos(smallp_intersect/180*M_PI);//三个点如果角度大于172.5度，且比例小于1.2倍，则认为是平面
}

Preprocess::~Preprocess() {}//析构函数
//设置参数用的函数，在程序中并未使用，主要时设置程序运行参数，如是否提取特征点、雷达种类、盲区范围，采样间隔。
void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;//设置采样间隔：即每隔point_filter_num个点取1个点
}
//预处理函数，主要包含了不同激光雷达的处理，在laser-mapping.cpp中被调用，从而拿到处理后的点云。并初步完成了筛选。
//输入：livox_ros_driver::CustomMsg
//输出：pcl::PointXYZINormal：float x, y, z, intensity, normal_x,normal_y,normal_z, curvature;
//其中的Normal结构体：表示给定点所在样本曲面上的法线方向，以及对应曲率的测量值，用第四个元素来占位，兼容SSE和高效计算。用户访问法向量的第一个坐标，可以通过points[i].data_n[0]或者points[i].normal[0]或者points[i].normal_x，但曲率不能被存储在同一个结构体中，因为它会被普通的数据操作覆盖掉
void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{  
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (time_unit)
  {
    case SEC:
      time_unit_scale = 1.e3f;//1000
      break;
    case MS:
      time_unit_scale = 1.f;//1
      break;
    case US:
      time_unit_scale = 1.e-3f;//0.001
      break;
    case NS:
      time_unit_scale = 1.e-6f;//0.000001
      break;
    default:
      time_unit_scale = 1.f;//否则按照MS执行
      break;
  }

  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;
  
  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}
//对livox雷达的预处理：拿到livox
void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  //清除之前的点云缓存
  pl_surf.clear();//清除之前的平面点点云缓存
  pl_corn.clear();//清除之前的角点点云缓存
  pl_full.clear();//清除之前的全部点点云缓存
  double t1 = omp_get_wtime();//后面没用到，这是在此处记录一个时间戳
  int plsize = msg->point_num;//一帧中的点云总个数
  // cout<<"plsie: "<<plsize<<endl;

  pl_corn.reserve(plsize);//分配空间
  pl_surf.reserve(plsize);//分配空间
  pl_full.resize(plsize);//分配空间

  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);//每一个SCAN（每条扫描线）保存的点云数量
  }
  uint valid_num = 0;//有效的点云数
  //如果进行特征提取（fast-lio2默认不进行特征提取）
  if (feature_enabled)
  {//遍历一帧数据中的每个点（1-plsize而不是0到plsize）
    for(uint i=1; i<plsize; i++)
    {
      //只取线数在0～N_SCANS内并且回波次序为0或者1的点云
      //激光雷达的回波次序通常指的是激光束在发送后被目标反射并返回的次序。在回波次序中，通常有两个主要值：0 和 1。
    /*回波次序为 0：
        表示激光束发送后，目标的反射回波是第一个到达激光雷达的。
        这通常意味着目标离激光雷达比较近，因此回波的时间很短。
        在一些激光雷达系统中，回波次序为 0 可能与近距离目标的探测相关。
      回波次序为 1：
        表示激光束发送后，目标的反射回波是第二个或后面的到达激光雷达的。
        这通常意味着目标离激光雷达比较远，导致回波的时间较长。
        回波次序的信息对于激光雷达在环境中定位和建图非常重要。通过检测回波的次序，系统可以推断目标的距离，并进一步分析目标的位置、形状等信息。在某些应用中，这种信息对于识别静态和动态目标、避障、导航等方面都很有用。不同的激光雷达系统和应用可能采用不同的回波次序策略*/
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        pl_full[i].x = msg->points[i].x;//点云x轴
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        //使用曲率作为每个激光点的时间
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points

        bool is_new = false;
        //只有当前点和上一个点的间距足够大（>1e-7(好像也不是很大)），才将当前点认为是有用的点，分别加入到pl_buff对应line的buffer中
        if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
            || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
            || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
        {
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }
    }
    static int count = 0;
    static double time = 0.0;
    count ++;
    double t0 = omp_get_wtime();
    //对每个line中的点进行分别处理（pl_buff中的line）
    for(int j=0; j<N_SCANS; j++)
    {
      //如果当前line中的点云过小（点的数量过小），则直接跳过，继续处理下一条line
      if(pl_buff[j].size() <= 5) continue;
      pcl::PointCloud<PointType> &pl = pl_buff[j];
      plsize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(plsize);
      plsize--;
      for(uint i=0; i<plsize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);//计算前后两个间隔点的距离
      }
      //因为i最后一个点没有i+1，所以单独求了一个range，没有distance
      types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
      give_feature(pl, types);//给特征
      // pl_surf += pl;
    }
    time += omp_get_wtime() - t0;
    printf("Feature extraction time: %lf \n", time / count);
  }
  //如果不进行特征提取
  else
  {
    //遍历接收到的一帧数据中的每个点
    for(uint i=1; i<plsize; i++)
    {
      //只取线数在0～N_SCANS内并且回波次序为0或者1的点云
      //激光雷达的回波次序通常指的是激光束在发送后被目标反射并返回的次序。在回波次序中，通常有两个主要值：0 和 1。
    /*回波次序为 0：
        表示激光束发送后，目标的反射回波是第一个到达激光雷达的。
        这通常意味着目标离激光雷达比较近，因此回波的时间很短。
        在一些激光雷达系统中，回波次序为 0 可能与近距离目标的探测相关。
      回波次序为 1：
        表示激光束发送后，目标的反射回波是第二个或后面的到达激光雷达的。
        这通常意味着目标离激光雷达比较远，导致回波的时间较长。
        回波次序的信息对于激光雷达在环境中定位和建图非常重要。通过检测回波的次序，系统可以推断目标的距离，并进一步分析目标的位置、形状等信息。在某些应用中，这种信息对于识别静态和动态目标、避障、导航等方面都很有用。不同的激光雷达系统和应用可能采用不同的回波次序策略*/
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num ++;//有效点云数
        //等间隔降采样
        if (valid_num % point_filter_num == 0)
        {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          //使用曲率作为每个激光点的时间
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms
//只有当前点和上一个点的间距足够大（>1e-7(好像也不是很大)），才将当前点认为是有用的点，分别加入到pl_buff对应line的buffer中
          if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
              || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
              || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
              && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
          {
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }
}

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);//从ros中的sensor_msgs::PointCloud2数据格式转为pcl数据格式
  int plsize = pl_orig.size();//plsize是一帧点云的数据大小
  //reserve 是 C++ 中容器类（如 std::vector）提供的一个方法，用于预分配容器的内存空间，以减少动态分配内存的次数，从而提高程序的性能。
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  if (feature_enabled)//是否进行特征提取，如果需要进行特征提取：执行以下操作
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++)//这里是遍历总点云数据中的每个点
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (blind * blind)) continue;//设置直通滤波
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;//该点的法向量在x轴上的分量
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;//self：这个值貌似没有用到
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t * time_unit_scale;//使用曲率当作时间戳：激光雷达点的时间戳*单位时间刻度（时间戳转化为ms）
      if(pl_orig.points[i].ring < N_SCANS)//环号=激光雷达线号
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);//将这一帧激光雷达数据映射到对应的环号上
      }
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];//提取出每一个激光线上的所有点
      int linesize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      //计算相邻点之间的速度信息（差分获得）
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);//得到每个点的距离信息
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);//特征提取只在每条线上进行。
    }
  }
  else//如果不进行特征提取操作
  {
    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      //根据采样间隔进行降采样
      if (i % point_filter_num != 0) continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      
      if (range < (blind * blind)) continue;
      
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms

      pl_surf.points.push_back(added_pt);//为什么只添加到surf上？
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0)
    {
      given_offset_time = true;
    }
    else
    {
      given_offset_time = false;
      double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
      double yaw_end  = yaw_first;
      int layer_first = pl_orig.points[0].ring;
      for (uint i = plsize - 1; i > 0; i--)
      {
        if (pl_orig.points[i].ring == layer_first)
        {
          yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
          break;
        }
      }
    }

    if(feature_enabled)
    {
      for (int i = 0; i < N_SCANS; i++)
      {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
      }
      
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        int layer  = pl_orig.points[i].ring;
        if (layer >= N_SCANS) continue;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_unit_scale; // units: ms

        if (!given_offset_time)
        {
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
          if (is_first[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;
              is_first[layer]=false;
              added_pt.curvature = 0.0;
              yaw_last[layer]=yaw_angle;
              time_last[layer]=added_pt.curvature;
              continue;
          }

          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle;
          time_last[layer]=added_pt.curvature;
        }

        pl_buff[layer].points.push_back(added_pt);
      }

      for (int j = 0; j < N_SCANS; j++)
      {
        PointCloudXYZI &pl = pl_buff[j];
        int linesize = pl.size();
        if (linesize < 2) continue;
        vector<orgtype> &types = typess[j];
        types.clear();
        types.resize(linesize);
        linesize--;
        for (uint i = 0; i < linesize; i++)
        {
          types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
          vx = pl[i].x - pl[i + 1].x;
          vy = pl[i].y - pl[i + 1].y;
          vz = pl[i].z - pl[i + 1].z;
          types[i].dista = vx * vx + vy * vy + vz * vz;
        }
        types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
        give_feature(pl, types);
      }
    }
    else
    {
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
        
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;

        if (!given_offset_time)
        {
          int layer = pl_orig.points[i].ring;
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

          if (is_first[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;
              is_first[layer]=false;
              added_pt.curvature = 0.0;
              yaw_last[layer]=yaw_angle;
              time_last[layer]=added_pt.curvature;
              continue;
          }

          // compute offset time
          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle;
          time_last[layer]=added_pt.curvature;
        }

        if (i % point_filter_num == 0)
        {
          if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
          {
            pl_surf.points.push_back(added_pt);
          }
        }
      }
    }
}
//特征提取：只对每条line的点云提取特征
//输入：1.每条扫描线上的点云（单条扫描线）
//     2.这条扫描线上每个点的额外特征
void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size();//单条扫描线上的点数
  int plsize2;
  if(plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;
//点不能在盲区，从这条线非盲区的点开始,这个距离只包含xy的计算
  while(types[head].range < blind)
  {
    head++;
  }

  // Surf
  //group_size默认值为8，
  //判断当前点后面是否还够8个点，如果够的话就逐渐减少，每次减8，就一次？
  //如果plsize>group_size,plsize2 = plsize-group_size,否则plsize的值设为0
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;


  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());//当前平面法向量（初始化为0）CSDN说是法向量，我认为是方向向量
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());//上一个平面的法向量 CSDN说是法向量，我认为是方向向量

  uint i_nex = 0, i2;//i2为当前点的下一个点
  uint last_i = 0; uint last_i_nex = 0;//last_i为上一个点的保存的索引
  int last_state = 0;//为1代表上个状态为平面，否则为0
  //判断面点
  int plane_type;
//head是指当前传进来的一条线上第一个非盲区内的点
  for(uint i=head; i<plsize2; i++)//之所以将plsize2后面的点减去，是因为在进行plane_judge时，会往后多取8个点，然后在进行点的类型判断时，由于plane_judge函数会返回一个至少往后八个点的索引，因此这一帧所有的点都计算上了
  {
    //在盲区内的点不做处理
    if(types[i].range < blind)
    {
      continue;
    }
//更新i2
    i2 = i;
//求得平面，并返回类型0 1 2
//返回为1时，才认为当前点是平面上的点
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
    
    if(plane_type == 1)
    {//设置确定的平面点和可能的平面点
      for(uint j=i; j<=i_nex; j++)
      { 
        if(j!=i && j!=i_nex)
        {
          //把起始点和终止点之间的所有点设置为确定的平面点
          types[j].ftype = Real_Plane;
        }
        else
        {
          //把起始点和终止点设为可能的平面点
          types[j].ftype = Poss_Plane;
        }
      }
      
      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      //最开始last_state=0,直接跳过，
      //之后last_state =1
      //如果之前状态是平面则判断当前点是处于两平面边缘的点还是较为平坦的平面的点
      //last_direct.norm()求取的是上一次直线方向向量的范数，范数越大表示
      //范数可以用来表示方向的稳定性： 平方范数越大，表示方向向量的每个分量的值越大，方向就越偏向各个维度的正向或负向。较大的平方范数可能表示直线方向更为突出或偏离坐标轴。
      if(last_state==1 && last_direct.norm()>0.1)
      {
        //这里计算的是两个方向向量的夹角？？？但是CSDN说是两个法向量的夹角
        //两个单位方向向量相乘，若两向量平行，则值会大，若两向量垂直，则值为0
        //这是两个向量的夹角公式：得到的是两个向量夹角的余弦值
        double mod = last_direct.transpose() * curr_direct;
        if(mod>-0.707 && mod<0.707)//夹角范围在（45-135度之间）
        {
          // //修改ftype，两个面法向量夹角在45度和135度之间 认为是两平面边缘上的点（这是CSDN的注释，而我认为这两个向量是方向向量）
          types[i].ftype = Edge_Plane;//认为这两个方向向量dadaaaa
        }
        else
        {
          types[i].ftype = Real_Plane;
        }
      }
      
      i = i_nex - 1;
      last_state = 1;
    }
    else // if(plane_type == 2)
    {
      //plane_type=0或2的时候
      i = i_nex;
      last_state = 0;
    }
    // else if(plane_type == 0)
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;

    //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }

    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }

    //   }
    // }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  //判断边缘点
  for(uint i=head+3; i<plsize2; i++)
  {
    //当点在盲区或者点不属于正常点和可能的平面点时，该点直接跳过不予处理
    if(types[i].range<blind || types[i].ftype>=Real_Plane)
    {
      continue;
    }

//点与点之间不能离的太近，否则直接跳过
    if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);//由当前点组成的向量
    Eigen::Vector3d vecs[2];

    for(int j=0; j<2; j++)
    {
      int m = -1;
      if(j == 1)
      {
        m = 1;
      }
//如果当前的前/后一个点在盲区内
      if(types[i+m].range < blind)
      {
        if(types[i].range > inf_bound)//并且当前点大于10m，那么则认为跳变较远
        {
          types[i].edj[j] = Nr_inf;//赋予该点前/后两点类型为Nr_inf(跳变较远)
        }
        else//否则
        {
          types[i].edj[j] = Nr_blind;//赋予该点前/后两点类型为Nr_blind(在盲区)
        }
        continue;//直接跳出当前点
      }
      //若当前点不在盲区内
      //设置雷达坐标系原点为O，当前点为A，前/后一个点为M和N   
      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
      vecs[j] = vecs[j] - vec_a;
      //dot运算是计算点积
      //这个是角OAM和OAN的cos直
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if(types[i].angle[j] < jump_up_limit)//cos(170)
      {
        types[i].edj[j] = Nr_180;//认为M在OA延长线上
      }
      else if(types[i].angle[j] > jump_down_limit)
      {
        types[i].edj[j] = Nr_zero;//认为M在OA上
      }
    }
    //这个是角MAN的cos
    //Prev默认是0，Next是1
    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    //前一个点是正常点&&下一个点在激光线上&&当前点与后一个点的距离大于0.025m&&当前点与后一个点的距离大于当前点与前一个点距离的四倍
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      if(types[i].intersect > cos160)//角MAN要小于160度，不然就平行于激光了
      {
        if(edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    //前面点是正常点&&（当前点到雷达中心距离>10，并且后点在盲区<blind(4m)）认为这是边缘点
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
     
    }
    //前后没有正常点：
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      if(types[i].ftype == Nor)
      {
        types[i].ftype = Wire;//程序中应该没有使用，当成空间中小的线段或者无用点了
      }
    }
  }
//继续找平面点
  plsize2 = plsize-1;//plsize中存放的是当前传入扫描线中总点的数量
  double ratio;
  for(uint i=head+1; i<plsize2; i++)
  {
    //当前点、前一个点、后一个点都需要不在盲区里面，若有一个在盲区内则跳过这个点
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
    {
      continue;
    }
    //当前点与前一个点、当前点与后一个点之间的距离不能太近
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
    {
      continue;
    }
//如果当前点为正常点：Nor是默认点吗，这里判断的是上面没有判断过的点吗？？，走到这里后好像没有点满足判断条件？？？
    if(types[i].ftype == Nor)
    {
      if(types[i-1].dista > types[i].dista)
      {
        ratio = types[i-1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;
      }
//对于一个点如果MAN角度大于172.5度&&前后点的间距变化率小于1.2，则继续执行
      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
      {
        if(types[i-1].ftype == Nor)
        {
          types[i-1].ftype = Real_Plane;
        }
        if(types[i+1].ftype == Nor)
        {
          types[i+1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }
//存储平面点
  int last_surface = -1;
  for(uint j=head; j<plsize; j++)
  {
    //平面点和可能的平面点
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
    {
      if(last_surface == -1)
      {
        last_surface = j;
      }
   //通常连着好几个都是面点
   //必须在采样间隔上的平面点才使用（这里是无差别滤波，从每次找到面点开始，每几个点才取一个） 
      if(j == uint(last_surface+point_filter_num-1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {//条边较大的边缘边的点 位于平面边缘上的点
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }
      //假如赏赐找到的面点被无差别滤掉了，而此时已经到了边缘
      if(last_surface != -1)
      {
        //取上次面点到此次边缘线之间的所有点的重心当作一个面点存储进去
        PointType ap;
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.intensity /= (j-last_surface);
        ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}
//平面判断
//传入参数：
//pl：当前点所在的一帧数据的的所在单条扫描线
//types：这条扫描线上每个点的额外特征构成的数组
//i_cur:当前点在pl中的索引位置
//i_nex：猜测是下一个点
//curr_direct:当前平面的法向量
//输出一个值，若为1则表明目标点是平面点，并且把当前点所在平面的法向量赋值给传入参数curr_direct 并且i_nex的值也会改变为
int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  //0.01*sqrt(x^2+y^2)+0.1
  //基本上可以近似看成是0.1，100m的时候才到0.2？？？？，其中disA和disB是定义的系数，暂时不知道干什么用的
  //types[i_cur]得到的是sqrt(x^2+y^2)，在xy平面上的距离
  double group_dis = disA*types[i_cur].range + disB;
//作平方运算
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;//前后点距离数组
  disarr.reserve(20);
/**********************先取最近的八个点，若不存在盲区内的点，存储当前点与后一个点的距离，再取这个扫描线中其他的距离当前点近的点，若比较近，且也不存在盲区内的点，也存储与后一个点的距离*******************************************************/
//距离小  点与点之间较近，先取够八个点
  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    //但凡这八个点里面有一个盲区点就返回2
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();//盲区点的法向量设置为零向量
      return 2;//返回2 ？？？
    }
    disarr.push_back(types[i_nex].dista);//存储当前点与后一个点之间的距离
  }
  //看后续的点有没有满足条件的（满足什么条件:与当前点之间的距离小于 group_dis的点）
  for(;;)//这句话等同于while(1)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;//退出条件
  //如果在pl中，除了选中的八个点之后的点中有一个盲区点，直接返回2
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();//
      return 2;//返回2
    }
    //对于后续的点，距离i_cur太远了就直接break，不考虑了
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx*vx + vy*vy + vz*vz;
    if(two_dis >= group_dis)
    {
      break;//跳出循环体
    }
    disarr.push_back(types[i_nex].dista);//存储当前点与后一个点的距离
    i_nex++;
  }
/******************************************************************************************************/
  double leng_wid = 0;
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++)//如果进行到这，此时i_nex取值到在所定义的距离公式计算的距离限度内，这一条扫描线上选中的前后点距离的最后一个点
  {
    //对于前面所选择的点（选择存储前后相邻点距离的点）遍历计算
    //假设i_cur点为A，j点为B，i_nex点为C
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    //向量AB
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;
    //向量AB叉乘向量AC，得到垂直于AC和AB的点，大小为|AB|*|AC|*sin<AB,AC>
    //这个求出的向量的大小物理意义是：物理意义是ABC组成的平行四边形的面积
    //式中vx，vy,vz是向量AC，是移动到后面的nex和cur之间的向量
    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];
  //这个向量计算模平方的物理意义是ABC组成的平行四边形的面积的平方，（为|AC|*h，其中h是B到线AC的距离）
    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    if(lw > leng_wid)
    {
      leng_wid = lw;//寻找，最大面积的平方（因为AC是固定的，因此寻找最大面积的平方，就是寻找距离AC最远的B）
    }
  }

//此时two_dis中存储的是向量AC，即是移动到后面的nex和cur之间的向量
//|AC*AC| / (AC*AC*h*h)<225-->h<1/15=0.0667m
//(AC*AC*h*h)指的是ABC构成的平行四边形最大面积的平方

//由于上面找到的是距离AC最远的B，如果这个B与AC之间的距离小于1/15，认为这个点太近了，不好拟合为一个平面。证明这个点或许不是平面点（但是只是一条线上的数据，说明不利哦什么）
  if((two_dis*two_dis/leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();//法向量直接设为0
    return 0;//返回0
  }
//把刚刚存起来的相邻两点之间的距离数组按照从大到小的顺序排序
  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++)
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }
//相邻两点之间的距离的最小值过小，也直接返回？？？暂时不理解
  if(disarr[disarr.size()-2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }
//对AVIA雷达的单独处理
  if(lidar_type==AVIA)
  {//点与点之间距离变化太大的时候，可能与激光光束是平行的，所以也就舍弃了
    double dismax_mid = disarr[0]/disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize-2];
    if(dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  
  curr_direct << vx, vy, vz;//AC，CSDN上说是当前点的法向量，但是我觉得是方向向量
  curr_direct.normalize();//对法向量进行归一化？？？ 我觉得是对方向向量进行归一化
  return 1;//到这里出来才认为是平面点
}
//边缘特征判断，主要是为了防止LOAM里面的两种不考虑的点，进行判断是否是边缘点
//输入：一条激光线上的激光点pl，这条线上的激光点的特征数组，i要传入的点在这条线上的位置，nor_dir指的是0/1，若传进来为0指的是前一个点。为1指的是后一个点
//输出：
bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if(nor_dir == 0)//如果nor_dir方向为前一个点代表要向前判断，判断前面两个点
  {
    if(types[i-1].range<blind || types[i-2].range<blind)
    {
      return false;//如果前面的两个点有一个在盲区里面直接返回false
    }
  }
  else if(nor_dir == 1)//如果nor_dir方向为前后，代表要向后判断，判断后面两个点
  {
    if(types[i+1].range<blind || types[i+2].range<blind)
    {
      return false;
    }
  }
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;

  if(d1<d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

 //如果相邻点的距离变化大则可能是被遮挡，不把他当作边缘点
  if(d1>edgea*d2 || (d1-d2)>edgeb)
  {
    return false;
  }
  
  return true;
}
