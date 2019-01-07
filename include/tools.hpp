#pragma once

#include <math.h> 
#include <Eigen/Dense>
#define M_PI       3.14159265358979323846

namespace tools
{
	// 计算反对称矩阵
	inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& w)
	{
		Eigen::Matrix3d w_hat;
		w_hat(0, 0) = 0;
		w_hat(0, 1) = -w(2);
		w_hat(0, 2) = w(1);
		w_hat(1, 0) = w(2);
		w_hat(1, 1) = 0;
		w_hat(1, 2) = -w(0);
		w_hat(2, 0) = -w(1);
		w_hat(2, 1) = w(0);
		w_hat(2, 2) = 0;
		return w_hat;
	}


	// 旋转矢量转为四元数
	inline Eigen::Quaterniond smallAngleQuaternion(
		const Eigen::Vector3d& dtheta)
	{
		Eigen::Quaterniond q;
		Eigen::Vector3d dq = dtheta / 2.0;

		double dq_square_norm = dq.squaredNorm();

		if (dq_square_norm <= 1)
		{
			q.x() = dq(0);
			q.y() = dq(1);
			q.z() = dq(2);
			q.w() = std::sqrt(1 - dq_square_norm);
		}
		else
		{
			q.x() = dq(0) / std::sqrt(1 + dq_square_norm);
			q.y() = dq(1) / std::sqrt(1 + dq_square_norm);
			q.z() = dq(2) / std::sqrt(1 + dq_square_norm);
			q.w() = 1 / std::sqrt(1 + dq_square_norm);
		}
		//q = q.conjugate();
		return q;
	}

	// 左乘矩阵
	template <typename Derived>
	inline Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
	{
		Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
		Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
		ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
		ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
		return ans;
	}

	// 右乘矩阵
	template <typename Derived>
	inline Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
	{
		Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
		Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
		ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
		ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
		return ans;
	}

	// 旋转矩阵到欧拉角
	inline Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
	{
		Eigen::Vector3d n = R.col(0);
		Eigen::Vector3d o = R.col(1);
		Eigen::Vector3d a = R.col(2);

		Eigen::Vector3d ypr(3);
		double y = atan2(n(1), n(0));
		double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
		double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
		ypr(0) = y;
		ypr(1) = p;
		ypr(2) = r;

		return ypr / M_PI * 180.0;
	}

	// 欧拉角到旋转矩阵
	template <typename Derived>
	inline Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
	{
		typedef typename Derived::Scalar Scalar_t;

		Scalar_t y = ypr(0) / 180.0 * M_PI;
		Scalar_t p = ypr(1) / 180.0 * M_PI;
		Scalar_t r = ypr(2) / 180.0 * M_PI;

		Eigen::Matrix<Scalar_t, 3, 3> Rz;
		Rz << cos(y), -sin(y), 0,
			sin(y), cos(y), 0,
			0, 0, 1;

		Eigen::Matrix<Scalar_t, 3, 3> Ry;
		Ry << cos(p), 0., sin(p),
			0., 1., 0.,
			-sin(p), 0., cos(p);

		Eigen::Matrix<Scalar_t, 3, 3> Rx;
		Rx << 1., 0., 0.,
			0., cos(r), -sin(r),
			0., sin(r), cos(r);

		return Rz * Ry * Rx;
	}





}














