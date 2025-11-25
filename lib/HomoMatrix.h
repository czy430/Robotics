#ifndef HOMOMATRIX_H
#define HOMOMATRIX_H

#include "Global.h"
#include "Vector.h"
#include "PoseMatrix.h"
#include "HomoVector.h"

struct HomoMatrix
{
	double Matrix[HOMO][HOMO];

	HomoMatrix () { memset(Matrix, 0, sizeof(Matrix)); }

    HomoMatrix (const double & a)
    {
        memset(Matrix, 0, sizeof(Matrix));
        for (int i = 0; i < HOMO; ++i) Matrix[i][i] = a;
    }

    HomoMatrix (const PoseMatrix & R, const Vector & p)
    {
        memset(Matrix, 0, sizeof(Matrix));
        for (int i = 0; i < HOMO - 1; ++i)
        {
            for (int j = 0; j < HOMO - 1; ++j)
            {
                Matrix[i][j] = R.Matrix[i][j];
            }
            Matrix[i][HOMO - 1] = p.Vec[i];
        }
        Matrix[HOMO - 1][HOMO - 1] = 1.0;
    }

    void operator = (const HomoMatrix & a)
	{
		for (int i = 0; i < HOMO; ++i)
		{
			for (int j = 0; j < HOMO; ++j)
			{
				Matrix[i][j] = a.Matrix[i][j];
			}
		}
	}

	HomoMatrix operator * (const HomoMatrix & a) const
	{
		HomoMatrix res;

		for (int i = 0; i < HOMO; ++i)
		{
			for (int j = 0; j < HOMO; ++j)
			{
				for (int k = 0; k < HOMO; ++k)
				{
					res.Matrix[i][j] += Matrix[i][k] * a.Matrix[k][j];
				}
			}
		}
		return res;
	}

	HomoMatrix operator *= (const HomoMatrix & a)
	{
		*this = *this * a;
		return *this;
	}

	HomoVector operator * (const HomoVector & a) const
	{
		HomoVector res;

		for (int i = 0; i < HOMO; ++i)
		{
			for (int k = 0; k < HOMO; ++k)
			{
				res.Vec[i] += Matrix[i][k] * a.Vec[k];
			}
		}

		return res;
	}
};

std::istream & operator >> (std::istream & in, HomoMatrix & a)
{
	for (int i = 0; i < HOMO; ++i)
	{
		for (int j = 0; j < HOMO; ++j)
		{
			in >> a.Matrix[i][j];
		}
	}
	return in;
}

std::ostream & operator << (std::ostream & out, HomoMatrix & a)
{
	for (int i = 0; i < HOMO; ++i)
	{
		for (int j = 0; j < HOMO; ++j)
		{
            if (fabs(a.Matrix[i][j]) < eps) out << 0 << " ";
			else out << a.Matrix[i][j] << " ";
		}
		out << std::endl;
	}
	return out;
}

// 转置
HomoMatrix Transpose (const HomoMatrix & T)
{
    HomoMatrix res;
    
    for (int i = 0; i < HOMO; ++i)
	{
		for (int j = 0; j < HOMO; ++j)
		{
			res.Matrix[i][j] = T.Matrix[j][i];
		}
	}

    return res;
}

std::pair <PoseMatrix, Vector> split (const HomoMatrix & T)
{
	PoseMatrix R;
	
	for (int i = 0; i < POSE; ++i)
	{
		for (int j = 0; j < POSE; ++j)
		{
			R.Matrix[i][j] = T.Matrix[i][j];
		}
	}

	Vector p;

	for (int i = 0; i < POSE; ++i) p.Vec[i] = T.Matrix[i][3];

	return std::make_pair(R, p);

}

//  求逆
HomoMatrix Inv (HomoMatrix R)
{
	HomoMatrix _R(1.0);
	for (int i = 0; i < HOMO; ++i)
    {
        int row = i;
        for (int j = i + 1; j < HOMO; ++j) if (R.Matrix[j][i] > R.Matrix[row][i]) row = j;
        if (row != i) std::swap(R.Matrix[row], R.Matrix[i]), std::swap(_R.Matrix[row], _R.Matrix[i]);
        if (fabs(R.Matrix[i][i]) < eps)
		{
			std::cout << "No solution" << std::endl;
			break;
		}
        for (int j = 0; j < HOMO; ++j)
        {
            if (i == j) continue;
            double res = R.Matrix[j][i] / R.Matrix[i][i];
            for (int k = i; k < HOMO; ++k) R.Matrix[j][k] -= res * R.Matrix[i][k];
            for (int k = 0; k < HOMO; ++k) _R.Matrix[j][k] -= res * _R.Matrix[i][k];
        }
        for (int j = 0; j < HOMO; ++j) R.Matrix[i][j] /= R.Matrix[i][i], _R.Matrix[i][j] /= R.Matrix[i][i];
    }
	return _R;
}

// 求逆
HomoMatrix inv (const HomoMatrix & T)
{
	std::pair <PoseMatrix, Vector> res = split(T);

	return HomoMatrix(Transpose(res.first), Transpose(res.first) * res.second * (-1.0));
}

#endif
