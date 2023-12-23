#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "use-ikfom.hpp"

/// *************Preconfiguration

#define MAX_INI_COUNT (10)
//判断两个点的时间是否前后颠倒，在处理上使用curature存储激光雷达点时间戳
const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;//定义噪声协方差
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;//IMU参数输出文件
  V3D cov_acc;//加速度测量协方差
  V3D cov_gyr;//角速度测量协方差
  V3D cov_acc_scale;//
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;//角速度零偏的协方差
  V3D cov_bias_acc;//加速度零偏的协方差
  double first_lidar_time;//当前帧第一个时间点云

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;//当前帧点云未去畸变
  sensor_msgs::ImuConstPtr last_imu_;//上一帧IMU
  deque<sensor_msgs::ImuConstPtr> v_imu_;//IMU队列
  vector<Pose6D> IMUpose;//IMU位姿
  vector<M3D>    v_rot_pcl_;//未使用
  M3D Lidar_R_wrt_IMU;//lidar到IMU的旋转外参：在IMU系下lidar的位置
  V3D Lidar_T_wrt_IMU;//lidar到IMU的位置外参
  V3D mean_acc;//加速度均值，用于计算方差
  V3D mean_gyr;//角速度均值，用于计算方差
  V3D angvel_last;//上一帧角速度
  V3D acc_s_last;//上一帧加速度
  double start_timestamp_;//开始时间戳
  double last_lidar_end_time_;//上一帧雷达数据结束时间戳
  int    init_iter_num = 1;//初始化迭代次数
  bool   b_first_frame_ = true;//是否为第一帧
  bool   imu_need_init_ = true;//是否需要初始化IMU
};
//构造函数：
ImuProcess::ImuProcess()//默认是第一帧，IMU需要初始化，开始时间戳赋值为-1
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;//初始化迭代次数
  //IKFoM(Iterated Kalman Filters on Manifolds)是港大开源的一个计算高效且方便的迭代卡尔曼滤波器工具包。
  //https://github.com/hku-mars/IKFoM.git
  //"MTK" 可能是指 "Manifold Toolkit"，这是一个用于处理航空航天导航、估计与控制问题的开源 C++ 模板库
  Q = process_noise_cov();//调用use-ikfom.hpp里面的process_noise_cov完成噪声协方差的初始化
  cov_acc       = V3D(0.1, 0.1, 0.1);//加速度测量协方差矩阵初始化
  cov_gyr       = V3D(0.1, 0.1, 0.1);//角速度测量协方差矩阵初始化
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);//角速度测量零偏协方差矩阵
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);//加速度测量零偏协方差矩阵
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;//上一帧角速度初始化
  Lidar_T_wrt_IMU = Zero3d;//lidar-IMU的位置外参初始化
  Lidar_R_wrt_IMU = Eye3d;//lidar-IMU的旋转外参初始化
  last_imu_.reset(new sensor_msgs::Imu());//上一帧IMU初始化
}

ImuProcess::~ImuProcess() {}
//重置参数
void ImuProcess::Reset() 
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;//是否需要初始化IMU
  start_timestamp_  = -1;//开始时间戳
  init_iter_num     = 1;//初始化迭代次数16
  v_imu_.clear();//imu队列清空
  IMUpose.clear();//imu位姿清空
  last_imu_.reset(new sensor_msgs::Imu());//上一帧IMU初始化为原始状态
  cur_pcl_un_.reset(new PointCloudXYZI());//当前点云帧未去畸变初始化
}
//传入外参，包含R，T
void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}
//传入外参，包含T
void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}
//传入外参，包含R，T
void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}
//传入陀螺仪角速度协方差
void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}
//传入参数：测量数据
//内部主要是对x_和P_完成了初始化，这里涉及到esikfom文件内部的知识，这里暂且先不讲
void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  //1.初始化重力、陀螺仪偏差、acc和陀螺仪协方差
  //2.将加速度测量值标准化为单位重力
  //这里英国是静止初始化
  
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)//判断是否为第一帧，如果是第一帧
  {
    Reset();//重置参数
    N = 1;//将迭代次数置为1
    b_first_frame_ = false;////将是否为第一帧的标志为清零
    const auto &imu_acc = meas.imu.front()->linear_acceleration;//从common_lib.h中拿到imu初始时刻的加速度
    const auto &gyr_acc = meas.imu.front()->angular_velocity;//从common_lib.h中拿到imu初始时可的角速度
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;//第一帧加速度测量值作为初始化均值
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;//第一帧角速度测量值作为初始化均值
    first_lidar_time = meas.lidar_beg_time;//如果是第一帧数据，将第一帧数据测量值的雷达开始时间
  }
//计算方差
  for (const auto &imu : meas.imu)//拿到的所有IMU帧
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
//更新均值
    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;
    //.cwiseProduct()对应系数相乘
    //每次迭代之后均值都会发生变化，最后的方差公式中减的应该是最后的均值
    // https://blog.csdn.net/weixin_44479136/article/details/90510374 方差迭代计算公式
    //按照博客推导出来的下面方差递推公式有两种
    //按照博客推出来的是两种都对不上
    //cov_acc是一个三维数组，.cwiseProduct()方法表示对应系数相乘得到的结果仍为三维数组，例如9(1,2,3).cwiseProduct((4,5,6))=(4,10,18)
    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);
    //第二种是
    // cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - 上一次的mean_acc)  / N;
    // cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - 上一次的mean_gyr)  / N;
    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();   //在esekfom.hpp获得x_的状态
  //从common_lib.h中拿到重力，并与加速度测量均值的单位重力求出SO2的旋转矩阵类型的重力加速度
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);//此处得到的是一个3*1的三维列向量，对测量的IMU加速度的平均值单位化后，乘以重力加速度（9.81）然后再进行单位化，作为状态的重力部分（为什么？）
  
  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;//将角速度的平均值作为角速度的零偏
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);
//在esekfom.hpp获得P_的协方差矩阵
  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();//将协方差矩阵设为单位阵
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001; //将协方差矩阵的位置和旋转的协方差置为0.00001
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;//将协方差矩阵的速度和位姿的协方差置为0.00001
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;//将协方差矩阵的重力和姿态的协方差置为0.0001
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001; //将协方差矩阵的陀螺仪偏差和姿态的协方差置为0.001
  init_P(21,21) = init_P(22,22) = 0.00001; //将协方差矩阵的lidar和imu外参位移量的协方差置为0.00001
  kf_state.change_P(init_P);//将初始化协方差矩阵传入esekfom.hpp中的P_
  last_imu_ = meas.imu.back();//将最后一帧的imu数据传入last_imu_中，暂时没用到

}
//在UndistortPcl函数中不但有IMU的前向信息，还有激光雷达去畸变的问题，这一节我们围绕着IMU的正向传播展开，代码中通过迭代的形式完成了IMU数据的更新，并将acc和gyro的数据传入到ESKF中，详细的公式我们后面再来讲。
//输入：measuregroup包含激光点云数据和IMU队列信息，上一帧结束后的状态量
//状态信息
//在函数体里面执行运算的，经过去畸变处理的点云信息，运算后会传出
void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head 将上一帧尾部的IMU数据，添加到当前帧的帧头***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);//last_imu是全局变量
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();//拿到当前帧头部的IMU时间，也就是上一帧尾部的imu时间戳
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();//拿到当前帧尾部时间戳
  const double &pcl_beg_time = meas.lidar_beg_time;
  const double &pcl_end_time = meas.lidar_end_time;//PCL开始/结束时间戳
  
  /*** sort point clouds by offset time 根据点云中每个点的时间戳对点云进行重新排序 ***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  /*** Initialize IMU pose 初始化IMU位姿***/
  state_ikfom imu_state = kf_state.get_x();//获取上一次KF估计的后验状态作为本次IMU预测的初始状态
  //IMUpose是pose6d格式的
  //将初始状态加入IMUpose中，包含有时间间隔，上一帧加速度，上一帧角速度，上一帧速度，上一帧位置，上一帧旋转矩阵
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point 在每一个IMU点的前向传播***/
  //前向传播对应的参数：平均角速度、平均加速度、IMU加速度、IMU速度、IMU位置
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;//IMU旋转矩阵
  double dt = 0;////时间间隔

  input_ikfom in;//eskf传入的参数
  //这里不包括IMU队列的最后一帧IMU数据
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);//拿到当前帧的IMU数据
    auto &&tail = *(it_imu + 1);//拿到下一帧的IMU数据
    
    if (tail->header.stamp.toSec() < last_lidar_end_time_)    continue;//判断时间先后顺序，不符合直接continue
    //计算当前帧和下一帧的平均角度和平均速度
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
    

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;
    //通过重力数值对加速度进行以下微调？？？
    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;
//如果IMU开始时刻早于上次激光雷达数据的最晚时刻（因为次将上次最后一个IMU插入到此次开头了。所以会出现一次这种情况）
    if(head->header.stamp.toSec() < last_lidar_end_time_)
    {
      //从赏赐雷达时刻末尾开始传播，计算与此次IMU结尾之间的时间差
      dt = tail->header.stamp.toSec() - last_lidar_end_time_;
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
    }
    else
    {
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }
    //原始测量的中值作为更新
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    //配置协方差矩阵（12*12？）只赋值给协方差矩阵的对角元素，也就是方差
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;//猜测：就是bg的值
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;//猜测：就是ba的值
    //IMU前向传播，每次传播的时间间隔为dt
    kf_state.predict(dt, Q, in);
  //进行前向传播(IMU预测过程)之后，保留当前IMU预测过程的状态（这是在一个遍历的for循环中）
    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;//更新angvel_last:角速度测量值的平均值-IMU预测过程计算出来的零偏
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);//更新acc_avr:加速度测量值的平均值-IMU预测过程计算出来的零偏，然后转换到IMU坐标系下
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];//加上重力，得到世界坐标系下的加速度
    }
    //后一个IMU距离此次雷达开始的时间间隔
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end 计算这一次数据中最后一帧IMU数据的位置和姿态的预测部分，因为前面的for循环并不包含这次数据的最后一帧IMU数据***/
  //判断雷达结束时间是否晚于IMU，最后一个IMU时刻可能遭遇雷达末尾，也可能晚于雷达末尾
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);//保存这一帧最后一个雷达测量的结束时间，以便于下一帧使用
  
  imu_state = kf_state.get_x();//更新IMU状态，以便下一次使用
  last_imu_ = meas.imu.back();//保存最后一帧数据，以便下一帧使用
  last_lidar_end_time_ = pcl_end_time;

  /*** undistort each lidar point (backward propagation) 反向传播，激光雷达点云去畸变***/
    /*** 在处理完所有的IMU预测后，剩下的就是对激光的去畸变了 ***/
  // 基于IMU预测对lidar点云去畸变
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu<<MAT_FROM_ARRAY(head->rot);
    // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
    vel_imu<<VEC_FROM_ARRAY(head->vel);//拿到前一帧的IMU速度
    pos_imu<<VEC_FROM_ARRAY(head->pos);//前一帧的IMU位置(在全局坐标系下的位置？)
    acc_imu<<VEC_FROM_ARRAY(tail->acc);//后一帧的IMU加速度
    angvel_avr<<VEC_FROM_ARRAY(tail->gyr);//后一帧的IMU角速度

    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation变换到“结束”帧，仅使用旋转
       * Note: Compensation direction is INVERSE of Frame's moving direction注意：补偿方向与帧的移动方向相反
       * So if we want to compensate a point at timestamp-i to the frame-e：所以如果我们向补偿位于时间戳i的一个点到帧e的一个点
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame 其中T_ei在全局坐标系中表示*/
      //R_i：R_imu前一帧IMU数据相对于全局坐标系的旋转量
      M3D R_i(R_imu * Exp(angvel_avr, dt));//点所在时刻的旋转矩阵
      
      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      //T_ei是在全局坐标系下：it_pcl相对于这一帧激光雷达数据末尾时间戳的时间差
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      //conjugate指的是矩阵的共轭，对于旋转矩阵来说（单位实对陈矩阵）矩阵的共轭等于矩阵的转置
      V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!
      
      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  if(meas.imu.empty()) {return;};
  ROS_ASSERT(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }

    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
