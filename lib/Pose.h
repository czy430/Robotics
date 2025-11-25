#ifndef POSE_N
#define POSE_N

#include "PoseMatrix.h"
#include "Vector.h"
#include "PoseMatrix.h"
#include "Quaternion.h"
#include "HomoVector.h"
#include "HomoMatrix.h"

// 绕x轴旋转
PoseMatrix Rotate_X (double angle)
{
	PoseMatrix res;

	angle *= PI / 180.0;
	res.Matrix[0][0] = 1.0;
	res.Matrix[1][1] = res.Matrix[2][2] = cos(angle);
	res.Matrix[1][2] = -(res.Matrix[2][1] = sin(angle));
	
	return res;
}

// 绕y轴旋转
PoseMatrix Rotate_Y (double angle)
{
	PoseMatrix res;

	angle *= PI / 180.0;
	res.Matrix[0][0] = res.Matrix[2][2] = cos(angle);
	res.Matrix[2][0] = -(res.Matrix[0][2] = sin(angle));
	res.Matrix[1][1] = 1.0;

	return res;
}

// 绕z轴旋转
PoseMatrix Rotate_Z (double angle)
{
	PoseMatrix res;

	angle *= PI / 180.0;
	res.Matrix[0][0] = res.Matrix[1][1] = cos(angle);
	res.Matrix[0][1] = -(res.Matrix[1][0] = sin(angle));
	res.Matrix[2][2] = 1.0;

	return res;
}

// 轴-角到姿态矩阵
PoseMatrix Rotate (const Vector & k, double angle)
{
	PoseMatrix res;

	angle *= PI / 180.0;
	res.Matrix[0][0] = k.Vec[0] * k.Vec[0] * (1.0 - cos(angle)) + cos(angle);
	res.Matrix[0][1] = k.Vec[1] * k.Vec[0] * (1.0 - cos(angle)) - k.Vec[2] * cos(angle);
	res.Matrix[0][2] = k.Vec[2] * k.Vec[0] * (1.0 - cos(angle)) + k.Vec[1] * cos(angle);
	res.Matrix[1][0] = k.Vec[0] * k.Vec[1] * (1.0 - cos(angle)) + k.Vec[2] * cos(angle);
	res.Matrix[1][1] = k.Vec[1] * k.Vec[1] * (1.0 - cos(angle)) + cos(angle);
	res.Matrix[1][2] = k.Vec[2] * k.Vec[1] * (1.0 - cos(angle)) - k.Vec[0] * cos(angle);
	res.Matrix[2][0] = k.Vec[0] * k.Vec[2] * (1.0 - cos(angle)) - k.Vec[1] * cos(angle);
	res.Matrix[2][1] = k.Vec[1] * k.Vec[2] * (1.0 - cos(angle)) + k.Vec[0] * cos(angle);
	res.Matrix[2][2] = k.Vec[2] * k.Vec[2] * (1.0 - cos(angle)) + cos(angle);

	return res;
}
// 姿态矩阵到轴-角
std::pair<Vector, double> InvRotate (const PoseMatrix & R)
{
	Vector k;
	double angle = acos((Trace(R) - 1.0) / 2.0);
	k.Vec[0] = (R.Matrix[2][1] - R.Matrix[1][2]) / 2.0 / sin(angle);
	k.Vec[1] = (R.Matrix[0][2] - R.Matrix[2][0]) / 2.0 / sin(angle);
	k.Vec[2] = (R.Matrix[1][0] - R.Matrix[0][1]) / 2.0 / sin(angle);
	angle *= 180.0 / PI;
	return std::make_pair(k, angle);
}
// 动轴欧拉角到姿态矩阵
PoseMatrix EulerAngleToPoseMatrix (const double & alpha, const double & beta, const double & gamma, const std::string & s)
{
	PoseMatrix res(1.0);

	if (s[0] == 'X') res *= Rotate_X(alpha);
	else if (s[0] == 'Y') res *= Rotate_Y(alpha);
	else if (s[0] == 'Z') res *= Rotate_Z(alpha);

	if (s[1] == 'X') res *= Rotate_X(beta);
	else if (s[1] == 'Y') res *= Rotate_Y(beta);
	else if (s[1] == 'Z') res *= Rotate_Z(beta);

	if (s[2] == 'X') res *= Rotate_X(gamma);
	else if (s[2] == 'Y') res *= Rotate_Y(gamma);
	else if (s[2] == 'Z') res *= Rotate_Z(gamma);

	return res;
}

// 定轴欧拉角到姿态矩阵
PoseMatrix _EulerAngleToPoseMatrix (const double & alpha, const double & beta, const double & gamma, const std::string & s)
{
	PoseMatrix res(1.0);

	if (s[2] == 'X') res *= Rotate_X(gamma);
	else if (s[2] == 'Y') res *= Rotate_Y(gamma);
	else if (s[2] == 'Z') res *= Rotate_Z(gamma);

	if (s[1] == 'X') res *= Rotate_X(beta);
	else if (s[1] == 'Y') res *= Rotate_Y(beta);
	else if (s[1] == 'Z') res *= Rotate_Z(beta);

	if (s[0] == 'X') res *= Rotate_X(alpha);
	else if (s[0] == 'Y') res *= Rotate_Y(alpha);
	else if (s[0] == 'Z') res *= Rotate_Z(alpha);

	return res;
}

// 姿态矩阵到欧拉角
Vector PoseMatrixtoEulerAngle (const PoseMatrix & R, const std::string & s)
{
	Vector res;
	if (s == "XYZ")
	{
		if (fabs(R.Matrix[0][2] + 1.0) < eps)
		{
			res.Vec[1] = -PI / 2;
			res.Vec[0] = -atan2(R.Matrix[1][0], R.Matrix[1][1]);
			res.Vec[2] = 0.0;
		}
		else if (fabs(R.Matrix[0][2] - 1.0) < eps)
		{
			res.Vec[1] = PI / 2;
			res.Vec[0] = atan2(R.Matrix[1][0], R.Matrix[1][1]);
			res.Vec[2] = 0.0;
		}
		else
		{
			res.Vec[1] = asin(R.Matrix[0][2]);
			res.Vec[0] = atan2(-R.Matrix[1][2], R.Matrix[2][2]);
			res.Vec[2] = atan2(-R.Matrix[0][1], R.Matrix[0][0]);
		}
	}
	else if (s == "XZY")
	{
		res.Vec[1] = -asin(R.Matrix[0][1]);
    	res.Vec[0] = atan2(R.Matrix[2][1], R.Matrix[1][1]);
    	res.Vec[2] = atan2(R.Matrix[0][2], R.Matrix[0][0]);
	}
	else if (s == "YXZ")
	{
		res.Vec[1] = -asin(R.Matrix[1][2]);
    	res.Vec[0] = atan2(-R.Matrix[0][2], R.Matrix[2][2]);
    	res.Vec[2] = atan2(-R.Matrix[1][0], R.Matrix[1][1]);
	}
	else if (s == "YZX")
	{
		res.Vec[1] = asin(R.Matrix[1][0]);
    	res.Vec[0] = atan2(-R.Matrix[2][0], R.Matrix[0][0]);
    	res.Vec[2] = atan2(-R.Matrix[1][2], R.Matrix[1][1]);
	}
	else if (s == "ZXY")
	{
		res.Vec[1] = asin(R.Matrix[2][1]);
    	res.Vec[0] = atan2(-R.Matrix[0][1], R.Matrix[1][1]);
    	res.Vec[2] = atan2(-R.Matrix[2][0], R.Matrix[2][2]);
	}
	else if (s == "ZYX")
	{
		res.Vec[1] = sin(R.Matrix[2][0]);
    	res.Vec[0] = atan2(R.Matrix[1][0], R.Matrix[0][0]);
    	res.Vec[2] = atan2(R.Matrix[2][1], R.Matrix[2][2]);
	}
	else if (s == "XYX")
	{
		res.Vec[1] = sin(R.Matrix[0][0]);
    	res.Vec[0] = atan2(R.Matrix[1][0], -R.Matrix[2][0]);
    	res.Vec[2] = atan2(R.Matrix[0][1], R.Matrix[0][2]);
	}
	else if (s == "XZX")
	{
		res.Vec[1] = sin(R.Matrix[0][0]);
    	res.Vec[0] = atan2(R.Matrix[2][0], R.Matrix[1][0]);
    	res.Vec[2] = atan2(R.Matrix[0][2], -R.Matrix[0][1]);
	}
	else if (s == "YXY")
	{
		res.Vec[1] = sin(R.Matrix[1][1]);
    	res.Vec[0] = atan2(R.Matrix[0][1], R.Matrix[2][1]);
    	res.Vec[2] = atan2(R.Matrix[1][0], -R.Matrix[1][2]);
	}
	else if (s == "YZY")
	{
		res.Vec[1] = sin(R.Matrix[1][1]);
    	res.Vec[0] = atan2(R.Matrix[2][1], -R.Matrix[0][1]);
    	res.Vec[2] = atan2(R.Matrix[1][2], R.Matrix[1][0]);
	}
	else if (s == "ZXZ")
	{
		res.Vec[1] = sin(R.Matrix[2][2]);
    	res.Vec[0] = atan2(R.Matrix[0][2], -R.Matrix[1][2]);
    	res.Vec[2] = atan2(R.Matrix[2][0], R.Matrix[2][1]);
	}
	else if (s == "ZYZ")
	{
		res.Vec[1] = sin(R.Matrix[2][2]);
    	res.Vec[0] = atan2(R.Matrix[1][2], R.Matrix[0][2]);
    	res.Vec[2] = atan2(R.Matrix[2][1], -R.Matrix[2][0]);
	}
	return res;
}
// 轴-角到单位四元数
Quaternion RotateToQuaternion (const Vector & k, double angle)
{
	angle *= PI / 180.0;
	return Quaternion(cos(angle / 2), k * sin(angle / 2));
}
// 单位四元数到旋转变换矩阵
PoseMatrix QuaternionToPoseMatrix (const Quaternion & Q)
{
	PoseMatrix res;
	
	res.Matrix[0][0] = Q.eta * Q.eta + Q.epsilon.Vec[0] * Q.epsilon.Vec[0] - Q.epsilon.Vec[1] * Q.epsilon.Vec[1] - Q.epsilon.Vec[2] * Q.epsilon.Vec[2];
	res.Matrix[1][1] = Q.eta * Q.eta - Q.epsilon.Vec[0] * Q.epsilon.Vec[0] + Q.epsilon.Vec[1] * Q.epsilon.Vec[1] - Q.epsilon.Vec[2] * Q.epsilon.Vec[2];
	res.Matrix[2][2] = Q.eta * Q.eta - Q.epsilon.Vec[0] * Q.epsilon.Vec[0] - Q.epsilon.Vec[1] * Q.epsilon.Vec[1] + Q.epsilon.Vec[2] * Q.epsilon.Vec[2];

	res.Matrix[0][1] = (Q.epsilon.Vec[0] * Q.epsilon.Vec[1] - Q.epsilon.Vec[2] * Q.eta) * 2.0;
	res.Matrix[1][0] = (Q.epsilon.Vec[0] * Q.epsilon.Vec[1] + Q.epsilon.Vec[2] * Q.eta) * 2.0;

	res.Matrix[0][2] = (Q.epsilon.Vec[0] * Q.epsilon.Vec[2] + Q.epsilon.Vec[1] * Q.eta) * 2.0;
	res.Matrix[2][0] = (Q.epsilon.Vec[0] * Q.epsilon.Vec[2] - Q.epsilon.Vec[1] * Q.eta) * 2.0;

	res.Matrix[1][2] = (Q.epsilon.Vec[1] * Q.epsilon.Vec[2] - Q.epsilon.Vec[0] * Q.eta) * 2.0;
	res.Matrix[2][1] = (Q.epsilon.Vec[1] * Q.epsilon.Vec[2] + Q.epsilon.Vec[0] * Q.eta) * 2.0;

	return res;
}

// 旋转变换矩阵到单位四元数
Quaternion PoseMatrixToQuaternion (const PoseMatrix & R)
{
	Quaternion res;
	
	if (1 + R.Matrix[0][0] + R.Matrix[1][1] + R.Matrix[2][2] != 0)
	{
		res.eta = sqrt(1 + R.Matrix[0][0] + R.Matrix[1][1] + R.Matrix[2][2]) / 2;
	}

	return res;
}

HomoMatrix Translate_X (const double & x)
{
    HomoMatrix res(1.0);
    res.Matrix[0][HOMO - 1] = x;
    return res;
}

HomoMatrix Translate_Y (const double & y)
{
    HomoMatrix res(1.0);
    res.Matrix[1][HOMO - 1] = y;
    return res;
}

HomoMatrix Translate_Z (const double & z)
{
    HomoMatrix res(1.0);
    res.Matrix[2][HOMO - 1] = z;
    return res;
}

HomoMatrix Translate (const Vector & p)
{
    PoseMatrix res(1.0);
    return HomoMatrix(res, p);
}

HomoMatrix HomoRotate_X (const double & angle)
{
    Vector res;
    return HomoMatrix(Rotate_X(angle), res);
}

HomoMatrix HomoRotate_Y (const double & angle)
{
    Vector res;
    return HomoMatrix(Rotate_Y(angle), res);
}

HomoMatrix HomoRotate_Z (const double & angle)
{
    Vector res;
    return HomoMatrix(Rotate_Z(angle), res);
}

HomoMatrix HomoRotate (const Vector & k, const double & angle)
{
    Vector res;
    return HomoMatrix(Rotate(k, angle), res);
}

// D-H法
HomoMatrix DH (const double & theta, const double & d, const double a, const double & alpha)
{
	return HomoRotate_Z(theta) * Translate_Z(d) * Translate_X(a) * HomoRotate_X(alpha);
}

#endif
