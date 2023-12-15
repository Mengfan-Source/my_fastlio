# the preintegrated Lidar states at the time of IMU measurements in a frame
#在一帧中，IMU的测量时间过程中雷达状态的预积分量

# IMU测量相对于一帧雷达数据中第一个点的偏移时间
float64  offset_time # the offset time of IMU measurement w.r.t the first lidar point
# 全局坐标系下，在激光雷达原点的总加速度预积分量
float64[3] acc       # the preintegrated total acceleration (global frame) at the Lidar origin
# 激光雷达原点处的无偏角速度（在机体坐标系下）（测量值中去除零偏，更接近真实直）
float64[3] gyr       # the unbiased angular velocity (body frame) at the Lidar origin
#预积分速度量
float64[3] vel       # the preintegrated velocity (global frame) at the Lidar origin
#预积分位置量
float64[3] pos       # the preintegrated position (global frame) at the Lidar origin
#预积分旋转量
float64[9] rot       # the preintegrated rotation (global frame) at the Lidar origin