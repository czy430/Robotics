#ifndef VELOCITY_N
#define VELOCITY_N

#include "Global.h"
#include "PoseMatrix.h"
#include "Vector.h"
#include "PoseMatrix.h"
#include "Quaternion.h"
#include "HomoVector.h"
#include "HomoMatrix.h"

// 欧拉角速度到刚体角速度
Vector EulerAngVelotoAngVelo (const double & alpha, const double & beta, 
    const double & dotalpha, const double & dotbeta, const double & dottheta, const std::string & s)
{
	PoseMatrix R;
	Vector res[3], ret;
	
	res[0].Vec[s[0] - 'X'] = dotalpha;
	res[1].Vec[s[1] - 'X'] = dotbeta;
	res[2].Vec[s[2] - 'X'] = dottheta;

	ret = res[0];

	if (s[0] == 'X') R = Rotate_X(alpha);
	else if (s[0] == 'Y') R = Rotate_Y(alpha);
	else if (s[0] == 'Z') R = Rotate_Z(alpha);

	ret += R * res[1];

	if (s[1] == 'X') R *= Rotate_X(beta);
	else if (s[1] == 'Y') R *= Rotate_Y(beta);
	else if (s[1] == 'Z') R *= Rotate_Z(beta);

	ret += R * res[2];

	return ret;
}
// 刚体角速度到欧拉角速度
Vector AngVelotoEulerAngVelo (const Vector & omega, 
    const double & alpha, const double & beta, const std::string & s)
{
	PoseMatrix R;
	Vector res[3], ret;
	
	res[0].Vec[s[0] - 'X'] = 1;
	res[1].Vec[s[1] - 'X'] = 1;
	res[2].Vec[s[2] - 'X'] = 1;

	if (s[0] == 'X') R = Rotate_X(alpha);
	else if (s[0] == 'Y') R = Rotate_Y(alpha);
	else if (s[0] == 'Z') R = Rotate_Z(alpha);

	res[1] = R * res[1];

	if (s[1] == 'X') R *= Rotate_X(beta);
	else if (s[1] == 'Y') R *= Rotate_Y(beta);
	else if (s[1] == 'Z') R *= Rotate_Z(beta);

	res[2] = R * res[2];

	PoseMatrix J(res[0], res[1], res[2]);
	J = Inv(J);
	ret = J * omega;
	
	return ret;
}

// 轴角微分到刚体角速度
Vector DiffAxisAngletoAngVelo (const Vector & k, const Vector & dotk, 
    const double & phi, const double & dotphi)
{
	return k * dotphi - Cross_Product_Matrix(k) * dotk * (1 - cos(phi)) - dotk * sin(phi);
}
// 刚体角速度到轴角微分
std::pair<Vector, double> AngVelotoDiffAxisAngle (const PoseMatrix & crossk, const Vector & k, 
    const double & phi, const Vector & omega)
{
	Vector dotk = (crossk - crossk * crossk * (1 / tan(phi / 2.0))) * omega * 0.5;
	double dotphi = DotProduct(k, omega);
	return std::make_pair(dotk, dotphi);
}

#endif
