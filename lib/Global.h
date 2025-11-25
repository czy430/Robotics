#ifndef GLOBAL_H
#define GLOBAL_H

#include <cmath>
#include <vector>
#include <utility>
#include <cstring>
#include <iostream>
#include <algorithm>

const int POSE = 3;
const int HOMO = 4;
const int JACO = 6;
const double PI = 3.1415926535;
const double eps = 1e-9;

//double atan2 (double & y, double & x)
//{
//	if (x > 0 && y >= 0) return atan(y / x);
//	if (x < 0 && y >= 0) return atan(y / x) + PI;
//	if (x < 0 && y < 0) return atan(y / x) - PI;
//	if (x > 0 && y < 0) return atan(y / x);
//	if (x == 0 && y > 0) return PI / 2;
//	if (x == 0 && y < 0) return -PI / 2;
//	std::cout << "Function \"atan2\": No solution." << std::endl;
//	return 0.0;
//}

#endif
