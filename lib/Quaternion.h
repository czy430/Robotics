#ifndef QUATERNION_H
#define QUATERNION_H

#include "Global.h"

struct Quaternion
{
    double eta;
    Vector epsilon;

    Quaternion () { eta = 0; }
    Quaternion (double eta, Vector epsilon) : eta(eta), epsilon(epsilon) {}

    Quaternion operator * (const Quaternion & a) const
    {
        Quaternion res;
        
        res.eta = eta * a.eta - DotProduct(epsilon, a.epsilon);
        res.epsilon = epsilon * a.eta + a.epsilon * eta + epsilon * a.epsilon;

        return res;
    }
    Quaternion operator *= (const Quaternion & a)
    {
        *this = *this * a;
        return *this;
    }

};
// 共轭
Quaternion Adjoint (const Quaternion & a)
{
    return Quaternion(a.eta, a.epsilon * (-1.0));
}
// 范数
double Norm (const Quaternion & a)
{
    return a.eta * a.eta + Norm(a.epsilon);
}
// 倒数
Quaternion Inv (const Quaternion & a)
{
    double num = 1 / Norm(a);
    return Quaternion(a.eta * num * num, a.epsilon * (-1.0) * num * num);
}

#endif
