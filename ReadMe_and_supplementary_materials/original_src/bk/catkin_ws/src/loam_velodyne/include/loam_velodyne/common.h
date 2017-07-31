#include <cmath>

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

inline double length3d(double x,
	                   double y,
	                   double z)
{
	return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}
inline float length3d(float x,
	                  float y,
	                  float z)
{
	return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}