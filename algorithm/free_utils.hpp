#pragma once
#include <cmath>
using namespace std;

#include "free_point.hpp"

inline float norm(float x0, float y0, float z0, float x1, float y1, float z1)
{
    return sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2) + pow(z0 - z1, 2));
}

inline float norm(point3D p, int dimension=2)
{
	if (dimension == 2)
	{
		return sqrt(pow(p[0], 2) + pow(p[1], 2));
	}
	else
	{
		return sqrt(pow(p[0], 2) + pow(p[1], 2) + pow(p[2], 2));
	}
}