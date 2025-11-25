#ifndef HOMOVECTOR_N
#define HOMOVECTOR_N

#include "Global.h"
#include "Vector.h"

struct HomoVector
{
	double Vec[HOMO];
    
	HomoVector () { memset(Vec, 0, sizeof(Vec)); }

    HomoVector (const double & a, const double & b, const double & c)
    {
        Vec[0] = a;
        Vec[1] = b;
        Vec[2] = c;
    }

	HomoVector (const Vector & a)
    {
        Vec[0] = a.Vec[0];
        Vec[1] = a.Vec[1];
        Vec[2] = a.Vec[2];
        Vec[3] = 1;
    }

    void operator = (const HomoVector & a)
	{
		for (int i = 0; i < HOMO; ++i) Vec[i] = a.Vec[i];
	}

    void operator = (const Vector & a)
	{
		for (int i = 0; i < POSE; ++i) Vec[i] = a.Vec[i];
        Vec[HOMO - 1] = 0;
	}

    HomoVector operator + (const HomoVector & a) const
    {
        HomoVector res;

        for (int i = 0; i < HOMO; ++i) res.Vec[i] = Vec[i] + a.Vec[i];

        return res;
    }
    HomoVector operator += (const HomoVector & a)
    {
        *this = *this + a;
        return *this;
    }
    HomoVector operator - (const HomoVector & a) const
    {
        HomoVector res;

        for (int i = 0; i < HOMO; ++i) res.Vec[i] = Vec[i] - a.Vec[i];

        return res;
    }
    HomoVector operator -= (const HomoVector & a)
    {
        *this = *this - a;
        return *this;
    }
};

std::istream & operator >> (std::istream & in, HomoVector & a)
{
	for (int i = 0; i < HOMO; ++i) in >> a.Vec[i];
	return in;
}

std::ostream & operator << (std::ostream & out, HomoVector & a)
{
	for (int i = 0; i < HOMO; ++i)
    {
        if (fabs(a.Vec[i]) < eps) out << 0 << " ";
		else out << a.Vec[i] << " ";
    }
    out << std::endl;
	return out;
}

#endif
