#include <math.h>
#include <time.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <chrono>
#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

const float scanPeriod = 0.1; // time duration per scan
const int N_SCANS = 16;       // laser scan beam num
const int MAX_POINTS = 40000; // 160000 for HDL64E
const int imuQueLength = 200; // 2000 for HDL64E

const int maxIterNumOdom = 25;    // 100 for HDL64E
const int maxIterNumMapping = 10; // 20 for HDL64E

inline double rad2deg(double radians) {
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees) {
  return degrees * M_PI / 180.0;
}

inline double length3d(double x, double y, double z) {
	return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}
inline float length3d(float x, float y, float z) {
	return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

inline double length2d(double x, double y) {
	return sqrt(pow(x, 2) + pow(y, 2));
}
inline float length2d(float x, float y) {
	return sqrt(pow(x, 2) + pow(y, 2));
}
inline float sqrDis(pcl::PointXYZI p1, pcl::PointXYZI p2) {
	return pow(p1.x - p2.x, 2) +
	       pow(p1.y - p2.y, 2) +
	       pow(p1.z - p2.z, 2);
}
inline float sqrDis(float x, float y, float z) {
	return pow(x, 2) +
	       pow(y, 2) +
	       pow(z, 2);
}
inline double sqrDis(double x, double y, double z) {
	return pow(x, 2) +
	       pow(y, 2) +
	       pow(z, 2);
}

// Ref: http://tutorial.math.lamar.edu/Classes/CalcIII/EqnsOfPlanes.aspx
inline void solvePlane(const pcl::PointXYZI& tripod1,
                       const pcl::PointXYZI& tripod2,
                       const pcl::PointXYZI& tripod3,
                       float& pa, float& pb,
                       float& pc, float& pd) 
{
    pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) -
         (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
    pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) -
         (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
    pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) -
         (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
    pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

    float ps = length3d(pa, pb, pc);

    //if (ps == 0) {
    //  std::cout<<"FATAL ERROR, surfPointsFlat id: "<<i<<std::endl;
    //  std::cout<<"pa: "<<pa<<std::endl;
    //  std::cout<<"pb: "<<pb<<std::endl;
    //  std::cout<<"pc: "<<pc<<std::endl;
    //  std::cout<<"pd: "<<pd<<std::endl;
    //  std::cout<<"tripod1: "<<tripod1<<std::endl;
    //  std::cout<<"tripod2: "<<tripod2<<std::endl;
    //  std::cout<<"tripod3: "<<tripod3<<std::endl;
    //  std::cout<<"pointSearchSurfInd1[i]: "<<pointSearchSurfInd1[i]<<std::endl;
    //  std::cout<<"pointSearchSurfInd2[i]: "<<pointSearchSurfInd2[i]<<std::endl;
    //  std::cout<<"pointSearchSurfInd3[i]: "<<pointSearchSurfInd3[i]<<std::endl;
    //}

    // normal vector of the plane
    pa /= ps;
    pb /= ps;
    pc /= ps;
    pd /= ps;
}

struct FreqReport {
  FreqReport(const std::string &n) : name(n), firstTime(true) {}
  void report() {
    if (firstTime) {
      firstTime = false;
      last_time = std::chrono::system_clock::now();
      return;
    }
    auto cur_time = std::chrono::system_clock::now();
    ROS_INFO("time interval of %s = %f seconds\n",
    	    name.c_str(),
            std::chrono::duration_cast<
                std::chrono::duration<float, std::ratio<1, 1>>>(
                	cur_time - last_time).count());
    last_time = cur_time;
  }

  std::string name;
  std::chrono::system_clock::time_point last_time;
  bool firstTime;
};
