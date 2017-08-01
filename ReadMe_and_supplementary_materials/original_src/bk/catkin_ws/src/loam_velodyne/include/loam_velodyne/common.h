#include <math.h>
#include <time.h>
#include <stdio.h>
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
