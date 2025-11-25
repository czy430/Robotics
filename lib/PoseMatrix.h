#ifndef POSEMATRIX_H
#define POSEMATRIX_H

#include "Global.h"
#include "Vector.h"

struct PoseMatrix
{
	double Matrix[POSE][POSE];

	PoseMatrix () { memset(Matrix, 0, sizeof(Matrix)); }
    
    PoseMatrix (const double & a)
    {
        memset(Matrix, 0, sizeof(Matrix));
        for (int i = 0; i < POSE; ++i) Matrix[i][i] = a;
    }

	PoseMatrix (const Vector & a, const Vector & b, const Vector & c)
	{
		memset(Matrix, 0, sizeof(Matrix));
		for (int i = 0; i < POSE; ++i)
		{
			Matrix[i][0] = a.Vec[i];
			Matrix[i][1] = b.Vec[i];
			Matrix[i][2] = c.Vec[i];
		}
	}

    void operator = (const PoseMatrix & a)
	{
		for (int i = 0; i < POSE; ++i)
		{
			for (int j = 0; j < POSE; ++j)
			{
				Matrix[i][j] = a.Matrix[i][j];
			}
		}
	}

    PoseMatrix operator + (const PoseMatrix & a) const
	{
		PoseMatrix res;

		for (int i = 0; i < POSE; ++i)
		{
			for (int j = 0; j < POSE; ++j)
			{
				res.Matrix[i][j] = Matrix[i][j] + a.Matrix[i][j];
			}
		}
        
        return res;
    }

	PoseMatrix operator += (const PoseMatrix & a)
	{
		*this = *this + a;
		return *this;
	}

    PoseMatrix operator - (const PoseMatrix & a) const
	{
        PoseMatrix res;

		for (int i = 0; i < POSE; ++i)
		{
			for (int j = 0; j < POSE; ++j)
			{
				res.Matrix[i][j] = Matrix[i][j] - a.Matrix[i][j];
			}
		}
        
        return res;
	}

	PoseMatrix operator -= (const PoseMatrix & a)
	{
		*this = *this - a;
		return *this;
	}

	PoseMatrix operator * (const PoseMatrix & a) const
	{
		PoseMatrix res;

		for (int i = 0; i < POSE; ++i)
		{
			for (int j = 0; j < POSE; ++j)
			{
				for (int k = 0; k < POSE; ++k)
				{
					res.Matrix[i][j] += Matrix[i][k] * a.Matrix[k][j];
				}
			}
		}
		return res;
	}

	PoseMatrix operator *= (const PoseMatrix & a)
	{
		*this = *this * a;
		return *this;
	}

	Vector operator * (const Vector & a) const
	{
		Vector res;

		for (int i = 0; i < POSE; ++i)
		{
			for (int k = 0; k < POSE; ++k)
			{
				res.Vec[i] += Matrix[i][k] * a.Vec[k];
			}
		}

		return res;
	}
};

std::istream & operator >> (std::istream & in, PoseMatrix & a)
{
	for (int i = 0; i < POSE; ++i)
	{
		for (int j = 0; j < POSE; ++j)
		{
			in >> a.Matrix[i][j];
		}
	}
	return in;
}

std::ostream & operator << (std::ostream & out, PoseMatrix & a)
{
	for (int i = 0; i < POSE; ++i)
	{
		for (int j = 0; j < POSE; ++j)
		{
			if (fabs(a.Matrix[i][j]) < eps) out << 0 << " ";
			else out << a.Matrix[i][j] << " ";
		}
		out << std::endl;
	}
	return out;
}
//  迹
double Trace (const PoseMatrix & R)
{
    double res;
    for (int i = 0; i < POSE; ++i) res += R.Matrix[i][i];
    return res;
}
//  转置
PoseMatrix Transpose (const PoseMatrix & R)
{
    PoseMatrix res;
    
    for (int i = 0; i < POSE; ++i)
	{
		for (int j = 0; j < POSE; ++j)
		{
			res.Matrix[i][j] = R.Matrix[j][i];
		}
	}

    return res;
}
//  叉乘操作数
PoseMatrix Cross_Product_Matrix (const Vector & omega)
{
    PoseMatrix res;
    
    res.Matrix[0][1] = -omega.Vec[2];
    res.Matrix[0][2] = omega.Vec[1];
    res.Matrix[1][0] = omega.Vec[2];
    res.Matrix[1][2] = -omega.Vec[0];
    res.Matrix[2][0] = -omega.Vec[1];
    res.Matrix[2][1] = omega.Vec[0];

    return res;
}
//  求逆（使用初等行变换）
PoseMatrix Inv (PoseMatrix R)
{
	PoseMatrix _R(1.0);
	for (int i = 0; i < POSE; ++i)
    {
        int row = i;
        for (int j = i + 1; j < POSE; ++j) if (R.Matrix[j][i] > R.Matrix[row][i]) row = j;
        if (row != i) std::swap(R.Matrix[row], R.Matrix[i]), std::swap(_R.Matrix[row], _R.Matrix[i]);
        if (fabs(R.Matrix[i][i]) < eps)
		{
			std::cout << "No solution" << std::endl;
			break;
		}
        for (int j = 0; j < POSE; ++j)
        {
            if (i == j) continue;
            double res = R.Matrix[j][i] / R.Matrix[i][i];
            for (int k = i; k < POSE; ++k) R.Matrix[j][k] -= res * R.Matrix[i][k];
            for (int k = 0; k < POSE; ++k) _R.Matrix[j][k] -= res * _R.Matrix[i][k];
        }
        for (int j = 0; j < POSE; ++j) R.Matrix[i][j] /= R.Matrix[i][i], _R.Matrix[i][j] /= R.Matrix[i][i];
    }
	return _R;
}
//  求逆（利用性质，即转置）
PoseMatrix inv (const PoseMatrix & R)
{
    PoseMatrix res;
    
    for (int i = 0; i < POSE; ++i)
	{
		for (int j = 0; j < POSE; ++j)
		{
			res.Matrix[i][j] = R.Matrix[j][i];
		}
	}

    return res;
}
//  对旋转变换矩阵求导
PoseMatrix Differential (const PoseMatrix & R, const Vector & omega)
{
	return Cross_Product_Matrix(omega) * R;
}

#endif
