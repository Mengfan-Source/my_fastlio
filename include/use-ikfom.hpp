#ifndef USE_IKFOM_H
#define USE_IKFOM_H

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2; 
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;

MTK_BUILD_MANIFOLD(state_ikfom,
((vect3, pos))
((SO3, rot))
((SO3, offset_R_L_I))
((vect3, offset_T_L_I))
((vect3, vel))
((vect3, bg))
((vect3, ba))
((S2, grav))
);

MTK_BUILD_MANIFOLD(input_ikfom,
((vect3, acc))
((vect3, gyro))
);

MTK_BUILD_MANIFOLD(process_noise_ikfom,
((vect3, ng))
((vect3, na))
((vect3, nbg))
((vect3, nba))
);

MTK::get_cov<process_noise_ikfom>::type process_noise_cov()
{
	MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
	MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);// 0.03
	MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na, 0.0001); // *dt 0.01 0.01 * dt * dt 0.05
	MTK::setDiagonal<process_noise_ikfom, vect3, 6>(cov, &process_noise_ikfom::nbg, 0.00001); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
	MTK::setDiagonal<process_noise_ikfom, vect3, 9>(cov, &process_noise_ikfom::nba, 0.00001);   //0.001 0.05 0.0001/out 0.01
	return cov;
}

//double L_offset_to_I[3] = {0.04165, 0.02326, -0.0284}; // Avia 
//vect3 Lidar_offset_to_IMU(L_offset_to_I, 3)；
//fast-lio2论文公式(2)，其实这里的f就是将IMU的积分方程组成矩阵形式然后再去计算
Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in)
{
	//将IMU积分方程矩阵初始化为0，这里的24分别对应
	//速度(3)
	//角速度(3)
	//外参偏置T(3)
	//外参偏置R(3)
	//加速度(3)
	//角速度零偏(3)
	//加速度零偏(3)
	//位置(3)
	Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
	//得到IMU的角速度：返回的是omega = *this-s.bg(自己：真值=测量值-零偏)
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);
	//得到IMU的加速度，并转到世界坐标系中，其中s.rot表示的是body(IMU)在世界坐标系中的位置
	//根据公式这里得到的是a-g？  a-g = R（acc-ba）真值=测量值-零偏
	//自动驾驶与机器人中的SLAM书中p68(p52)式3.4a和式3.4b：
	vect3 a_inertial = s.rot * (in.acc-s.ba); //得到IMU的加速度，并转到世界坐标系中，其中s.rot表示的是body(IMU)在世界坐标系中的位置
	for(int i = 0; i < 3; i++ ){
		res(i) = s.vel[i];//更新的速度
		res(i + 3) =  omega[i]; //更新的角速度
		//由于上式得到的a_inertial是在世界坐标系下的a-g，这里加上了g
		res(i + 12) = a_inertial[i] + s.grav[i]; //更新的加速度
		//上面说的初始化矩阵为f的矩阵，而不是状态量x的矩阵，这个是f，可以根据离散化的IMU运动方程反推在程序中状态量X
	}
	return res;
}
//对应fast-lio2论文中的公式(11)
//每一个IMU到来时要进行的前向传播过程中计算协方差矩阵预测值时所需的F_x
//这个矩阵是24*23维的，23行，23列
Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in)
{
	//这个23对应了status的维度计算(x)
	//这个status包含：pos(3),rot(3),offset_R_L_I(3),offset_T_L_I(3),vel(3),bg(3),ba(3),grav(2)
	Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
	//通过使用 block 函数，将一个 3x3 的单位矩阵（Eigen::Matrix3d::Identity()）复制到矩阵 cov 中的特定块。
	//这个操作将单位矩阵复制到 cov 矩阵的第 0 行到第 2 行、第 12 列到第 14 列的块中
	cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
	vect3 acc_;
	in.acc.boxminus(acc_, s.ba);//拿到角速度
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);//拿到加速度
	//这里的-s.rot.toRotationMatrix()是因为论文中的矩阵是逆时针旋转的
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix()*MTK::hat(acc_);
	// 将角度转到存入的矩阵中（应该与上面颠倒了）
	cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();
	Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	//将vec的2*1矩阵转为grav_matrix的3*2矩阵
	s.S2_Mx(grav_matrix, vec, 21);
	cov.template block<3, 2>(12, 21) =  grav_matrix; 
	cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity(); 
	return cov;
}


Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
	cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
	return cov;
}

vect3 SO3ToEuler(const SO3 &orient) 
{
	Eigen::Matrix<double, 3, 1> _ang;
	Eigen::Vector4d q_data = orient.coeffs().transpose();
	//scalar w=orient.coeffs[3], x=orient.coeffs[0], y=orient.coeffs[1], z=orient.coeffs[2];
	double sqw = q_data[3]*q_data[3];
	double sqx = q_data[0]*q_data[0];
	double sqy = q_data[1]*q_data[1];
	double sqz = q_data[2]*q_data[2];
	double unit = sqx + sqy + sqz + sqw; // if normalized is one, otherwise is correction factor
	double test = q_data[3]*q_data[1] - q_data[2]*q_data[0];

	if (test > 0.49999*unit) { // singularity at north pole
	
		_ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
	if (test < -0.49999*unit) { // singularity at south pole
		_ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
		
	_ang <<
			std::atan2(2*q_data[0]*q_data[3]+2*q_data[1]*q_data[2] , -sqx - sqy + sqz + sqw),
			std::asin (2*test/unit),
			std::atan2(2*q_data[2]*q_data[3]+2*q_data[1]*q_data[0] , sqx - sqy - sqz + sqw);
	double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
	vect3 euler_ang(temp, 3);
		// euler_ang[0] = roll, euler_ang[1] = pitch, euler_ang[2] = yaw
	return euler_ang;
}

#endif