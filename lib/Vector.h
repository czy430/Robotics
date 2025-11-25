#ifndef VECTOR_H
#define VECTOR_H

#include "Global.h"

struct Vector
{
	double Vec[POSE];
    
	Vector () { memset(Vec, 0, sizeof(Vec)); }
	
    Vector (const double & a, const double & b, const double & c)
    {
        Vec[0] = a;
        Vec[1] = b;
        Vec[2] = c;
    }

    Vector operator + (const Vector & a) const
    {
        Vector res;

        for (int i = 0; i < POSE; ++i) res.Vec[i] = Vec[i] + a.Vec[i];

        return res;
    }
    Vector operator += (const Vector & a)
    {
        *this = *this + a;
        return *this;
    }
    Vector operator - (const Vector & a) const
    {
        Vector res;

        for (int i = 0; i < POSE; ++i) res.Vec[i] = Vec[i] - a.Vec[i];

        return res;
    }
    Vector operator -= (const Vector & a)
    {
        *this = *this - a;
        return *this;
    }
    // 外积
    Vector operator * (const Vector & a) const
    {
        Vector res;

        res.Vec[0] = Vec[1] * a.Vec[2] - Vec[2] * a.Vec[1];
        res.Vec[1] = Vec[2] * a.Vec[0] - Vec[0] * a.Vec[2];
        res.Vec[2] = Vec[0] * a.Vec[1] - Vec[1] * a.Vec[0];

        return res;
    }
    Vector operator *= (const Vector & a)
    {
        *this = *this * a;
        return *this;
    }
    // 数乘
    Vector operator * (const double & a) const
    {
        Vector res;

        for (int i = 0; i < POSE; ++i) res.Vec[i] = Vec[i] * a;

        return res;
    }
    Vector operator *= (const double & a)
    {
        *this = *this * a;
        return *this;
    }
    Vector operator / (const double & a) const
    {
        Vector res;

        for (int i = 0; i < POSE; ++i) res.Vec[i] = Vec[i] / a;

        return res;
    }
    Vector operator /= (const double & a)
    {
        *this = *this / a;
        return *this;
    }
};
// 清空
void clear (Vector & a)
{
	memset(a.Vec, 0, sizeof(a.Vec));
}
// 内积
double DotProduct (const Vector & a, const Vector & b)
{
    return a.Vec[0] * b.Vec[0] + a.Vec[1] * b.Vec[1] + a.Vec[2] * b.Vec[2];
}
// 范数
double Norm (const Vector & a)
{
    return sqrt(a.Vec[0] * a.Vec[0] + a.Vec[1] * a.Vec[1] + a.Vec[2] * a.Vec[2]);
}
// 标准化
Vector Unit (Vector a)
{
    return a / Norm(a);
}

std::istream & operator >> (std::istream & in, Vector & a)
{
	for (int i = 0; i < POSE; ++i) in >> a.Vec[i];
	return in;
}

std::ostream & operator << (std::ostream & out, Vector & a)
{
	for (int i = 0; i < POSE; ++i)
    {
        if (fabs(a.Vec[i]) < eps) out << 0 << " ";
		else out << a.Vec[i] << " ";
    }
    out << std::endl;
	return out;
}

#endif