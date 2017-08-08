#include <loam_velodyne/common.h>

const int skipFrameNum = 1;

bool systemInited = false;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;
double timeImuTrans = 0;

bool newCornerPointsSharp = false;
bool newCornerPointsLessSharp = false;
bool newSurfPointsFlat = false;
bool newSurfPointsLessFlat = false;
bool newLaserCloudFullRes = false;
bool newImuTrans = false;

pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>());

pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>());

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>());
// stores the original coordinates (not transformed to start time point) of feature point in current cloud
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>());

// pcl::PointCloud<pcl::PointXYZI>::Ptr pointSearchCornerLast(new pcl::PointCloud<pcl::PointXYZI>());
// pcl::PointCloud<pcl::PointXYZI>::Ptr pointSearchSurfLast(new pcl::PointCloud<pcl::PointXYZI>());

// pcl::PointCloud<pcl::PointXYZI>::Ptr pointProjCornerLast(new pcl::PointCloud<pcl::PointXYZI>());
// pcl::PointCloud<pcl::PointXYZI>::Ptr pointProjSurfLast(new pcl::PointCloud<pcl::PointXYZI>());

pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZ>());

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

int laserCloudCornerLastNum;
int laserCloudSurfLastNum;

int pointSelCornerInd[MAX_POINTS];
float pointSearchCornerInd1[MAX_POINTS];
float pointSearchCornerInd2[MAX_POINTS];

int pointSelSurfInd[MAX_POINTS];
float pointSearchSurfInd1[MAX_POINTS];
float pointSearchSurfInd2[MAX_POINTS];
float pointSearchSurfInd3[MAX_POINTS];

// rotation {x, y, z} or {pitch?, yaw?, roll?}, translation {x, y, z}
float transform[6] = {0};
float transformSum[6] = {0};

float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
float imuRollLast = 0, imuPitchLast = 0, imuYawLast = 0;
float imuShiftFromStartX = 0, imuShiftFromStartY = 0, imuShiftFromStartZ = 0;
float imuVeloFromStartX = 0, imuVeloFromStartY = 0, imuVeloFromStartZ = 0;

// ??
void TransformToStart(pcl::PointXYZI *pi, pcl::PointXYZI *po)
{
  // this is scanPeriod * relTime
  float s = 10 * (pi->intensity - int(pi->intensity));

  float rx = s * transform[0];
  float ry = s * transform[1];
  float rz = s * transform[2];
  float tx = s * transform[3];
  float ty = s * transform[4];
  float tz = s * transform[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  po->x = cos(ry) * x2 - sin(ry) * z2;
  po->y = y2;
  po->z = sin(ry) * x2 + cos(ry) * z2;
  po->intensity = pi->intensity;
}

// ??
void TransformToEnd(pcl::PointXYZI *pi, pcl::PointXYZI *po)
{
  float s = 10 * (pi->intensity - int(pi->intensity));

  float rx = s * transform[0];
  float ry = s * transform[1];
  float rz = s * transform[2];
  float tx = s * transform[3];
  float ty = s * transform[4];
  float tz = s * transform[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  float x3 = cos(ry) * x2 - sin(ry) * z2;
  float y3 = y2;
  float z3 = sin(ry) * x2 + cos(ry) * z2;

  rx = transform[0];
  ry = transform[1];
  rz = transform[2];
  tx = transform[3];
  ty = transform[4];
  tz = transform[5];

  float x4 = cos(ry) * x3 + sin(ry) * z3;
  float y4 = y3;
  float z4 = -sin(ry) * x3 + cos(ry) * z3;

  float x5 = x4;
  float y5 = cos(rx) * y4 - sin(rx) * z4;
  float z5 = sin(rx) * y4 + cos(rx) * z4;

  float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
  float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
  float z6 = z5 + tz;

  float x7 = cos(imuRollStart) * (x6 - imuShiftFromStartX)
           - sin(imuRollStart) * (y6 - imuShiftFromStartY);
  float y7 = sin(imuRollStart) * (x6 - imuShiftFromStartX)
           + cos(imuRollStart) * (y6 - imuShiftFromStartY);
  float z7 = z6 - imuShiftFromStartZ;

  float x8 = x7;
  float y8 = cos(imuPitchStart) * y7 - sin(imuPitchStart) * z7;
  float z8 = sin(imuPitchStart) * y7 + cos(imuPitchStart) * z7;

  float x9 = cos(imuYawStart) * x8 + sin(imuYawStart) * z8;
  float y9 = y8;
  float z9 = -sin(imuYawStart) * x8 + cos(imuYawStart) * z8;

  float x10 = cos(imuYawLast) * x9 - sin(imuYawLast) * z9;
  float y10 = y9;
  float z10 = sin(imuYawLast) * x9 + cos(imuYawLast) * z9;

  float x11 = x10;
  float y11 = cos(imuPitchLast) * y10 + sin(imuPitchLast) * z10;
  float z11 = -sin(imuPitchLast) * y10 + cos(imuPitchLast) * z10;

  po->x = cos(imuRollLast) * x11 + sin(imuRollLast) * y11;
  po->y = -sin(imuRollLast) * x11 + cos(imuRollLast) * y11;
  po->z = z11;
  po->intensity = int(pi->intensity);
}

void PluginIMURotation(float bcx, float bcy, float bcz,
                       float blx, float bly, float blz,
                       float alx, float aly, float alz,
                       float &acx, float &acy, float &acz)
{
  float sbcx = sin(bcx);
  float cbcx = cos(bcx);
  float sbcy = sin(bcy);
  float cbcy = cos(bcy);
  float sbcz = sin(bcz);
  float cbcz = cos(bcz);

  float sblx = sin(blx);
  float cblx = cos(blx);
  float sbly = sin(bly);
  float cbly = cos(bly);
  float sblz = sin(blz);
  float cblz = cos(blz);

  float salx = sin(alx);
  float calx = cos(alx);
  float saly = sin(aly);
  float caly = cos(aly);
  float salz = sin(alz);
  float calz = cos(alz);

  float srx = -sbcx * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly)
            - cbcx * cbcz * (calx * saly * (cbly * sblz - cblz * sblx * sbly)
            - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx)
            - cbcx * sbcz * (calx * caly * (cblz * sbly - cbly * sblx * sblz)
            - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz);
  acx = -asin(srx);

  float srycrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * saly * (cbly * sblz - cblz * sblx * sbly)
               - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx)
               - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * caly * (cblz * sbly - cbly * sblx * sblz)
               - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz)
               + cbcx * sbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);

  float crycrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * caly * (cblz * sbly - cbly * sblx * sblz)
               - calx * saly * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sblz)
               - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * saly * (cbly * sblz - cblz * sblx * sbly)
               - calx * caly * (sbly * sblz + cbly * cblz * sblx) + cblx * cblz * salx)
               + cbcx * cbcy * (salx * sblx + calx * caly * cblx * cbly + calx * cblx * saly * sbly);
  acy = atan2(srycrx / cos(acx), crycrx / cos(acx));

  float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz)
               - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx)
               - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly)
               + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx)
               - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz
               + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz)
               + calx*cblx*salz*sblz);
  float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly)
               - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx)
               + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
               + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly)
               + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly
               - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz)
               - calx*calz*cblx*sblz);
  acz = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
}

void AccumulateRotation(float cx, float cy, float cz,
                        float lx, float ly, float lz,
                        float &ox, float &oy, float &oz)
{
  float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
  ox = -asin(srx);

  float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz)
               + sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
  float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy)
               - cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz)
               + sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
  float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz)
               - cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharpMsg)
{
  timeCornerPointsSharp = cornerPointsSharpMsg->header.stamp.toSec();

  cornerPointsSharp->clear();
  pcl::fromROSMsg(*cornerPointsSharpMsg, *cornerPointsSharp);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices);
  newCornerPointsSharp = true;
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharpMsg)
{
  timeCornerPointsLessSharp = cornerPointsLessSharpMsg->header.stamp.toSec();

  cornerPointsLessSharp->clear();
  pcl::fromROSMsg(*cornerPointsLessSharpMsg, *cornerPointsLessSharp);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cornerPointsLessSharp, *cornerPointsLessSharp, indices);
  newCornerPointsLessSharp = true;
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlatMsg)
{
  timeSurfPointsFlat = surfPointsFlatMsg->header.stamp.toSec();

  surfPointsFlat->clear();
  pcl::fromROSMsg(*surfPointsFlatMsg, *surfPointsFlat);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*surfPointsFlat,*surfPointsFlat, indices);
  newSurfPointsFlat = true;
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlatMsg)
{
  timeSurfPointsLessFlat = surfPointsLessFlatMsg->header.stamp.toSec();

  surfPointsLessFlat->clear();
  pcl::fromROSMsg(*surfPointsLessFlatMsg, *surfPointsLessFlat);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*surfPointsLessFlat, *surfPointsLessFlat, indices);
  newSurfPointsLessFlat = true;
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg)
{
  timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp.toSec();

  laserCloudFullRes->clear();
  pcl::fromROSMsg(*laserCloudFullResMsg, *laserCloudFullRes);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserCloudFullRes,*laserCloudFullRes, indices);
  newLaserCloudFullRes = true;
}

void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTransMsg)
{
  timeImuTrans = imuTransMsg->header.stamp.toSec();

  imuTrans->clear();
  pcl::fromROSMsg(*imuTransMsg, *imuTrans);

  imuPitchStart = imuTrans->points[0].x;
  imuYawStart = imuTrans->points[0].y;
  imuRollStart = imuTrans->points[0].z;

  imuPitchLast = imuTrans->points[1].x;
  imuYawLast = imuTrans->points[1].y;
  imuRollLast = imuTrans->points[1].z;

  imuShiftFromStartX = imuTrans->points[2].x;
  imuShiftFromStartY = imuTrans->points[2].y;
  imuShiftFromStartZ = imuTrans->points[2].z;

  imuVeloFromStartX = imuTrans->points[3].x;
  imuVeloFromStartY = imuTrans->points[3].y;
  imuVeloFromStartZ = imuTrans->points[3].z;

  newImuTrans = true;
}


FreqReport laserOdometryFreq("laserOdometry");
FreqReport registeredLaserCloudFreq("registeredLaserCloud");
int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle nh;

  ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>
                                         ("/laser_cloud_sharp", 2, laserCloudSharpHandler);

  ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>
                                             ("/laser_cloud_less_sharp", 2, laserCloudLessSharpHandler);

  ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>
                                      ("/laser_cloud_flat", 2, laserCloudFlatHandler);

  ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>
                                          ("/laser_cloud_less_flat", 2, laserCloudLessFlatHandler);

  ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>
                                         ("/velodyne_cloud_2", 2, laserCloudFullResHandler);

  ros::Subscriber subImuTrans = nh.subscribe<sensor_msgs::PointCloud2>
                                ("/imu_trans", 5, imuTransHandler);

  ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>
                                           ("/laser_cloud_corner_last", 2);

  ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>
                                         ("/laser_cloud_surf_last", 2);

  ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
                                        ("/velodyne_cloud_3", 2);

  //ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pc1", 2);

  //ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc2", 2);

  //ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2> ("/pc3", 2);

  //ros::Publisher pub4 = nh.advertise<sensor_msgs::PointCloud2> ("/pc4", 2);

  ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);
  nav_msgs::Odometry laserOdometry;
  laserOdometry.header.frame_id = "/camera_init";
  laserOdometry.child_frame_id = "/laser_odom";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform laserOdometryTrans;
  laserOdometryTrans.frame_id_ = "/camera_init";
  laserOdometryTrans.child_frame_id_ = "/laser_odom";

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  // pointOri stores the original coordinates (not transformed to start time point) of feature point in current cloud
  // coeff.xyz stores step * diff(distance(pointSel, {edge/plane}), {pointSel.x, pointSel.y, pointSel.z}), diff means gradient
  // coeff.instance stores step * distance(pointSel, {edge/plane})
  pcl::PointXYZI pointOri, pointSel, tripod1, tripod2, tripod3, /*pointProj,*/ coeff;

  bool isDegenerate = false;
  cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

  int frameCount = skipFrameNum;
  ros::Rate rate(100);

  while (ros::ok()) {
    // not all the callbacks can be excuted
    ros::spinOnce();

    if (newCornerPointsSharp && newCornerPointsLessSharp && newSurfPointsFlat &&
        newSurfPointsLessFlat && newLaserCloudFullRes && newImuTrans &&
        fabs(timeCornerPointsSharp - timeSurfPointsLessFlat) < 0.005 &&
        fabs(timeCornerPointsLessSharp - timeSurfPointsLessFlat) < 0.005 &&
        fabs(timeSurfPointsFlat - timeSurfPointsLessFlat) < 0.005 &&
        fabs(timeLaserCloudFullRes - timeSurfPointsLessFlat) < 0.005 &&
        fabs(timeImuTrans - timeSurfPointsLessFlat) < 0.005) {

      newCornerPointsSharp = false;
      newCornerPointsLessSharp = false;
      newSurfPointsFlat = false;
      newSurfPointsLessFlat = false;
      newLaserCloudFullRes = false;
      newImuTrans = false;

      if (!systemInited) {
        // swap cornerPoints**Less**Sharp and laserCloudCornerLast
        // initialize the "last clouds" & kdtrees, publish the first clouds,
        // initialize the pitch and roll components of transformSum
        // initialize laserCloudCornerLast
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;

        // swap surfPoints**Less**Flat and laserCloudSurfLast
        // initialize laserCloudSurfLast
        laserCloudTemp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;

        // update kdtree for corner and surf
        // initialize kdtreeCornerLast kdtreeSurfLast
        kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
        kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

        // publish the first laserCloudCornerLast
        sensor_msgs::PointCloud2 laserCloudCornerLastMsg;
        pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLastMsg);
        laserCloudCornerLastMsg.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserCloudCornerLastMsg.header.frame_id = "/camera";
        pubLaserCloudCornerLast.publish(laserCloudCornerLastMsg);

        // publish the first laserCloudSurfLast
        sensor_msgs::PointCloud2 laserCloudSurfLastMsg;
        pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLastMsg);
        laserCloudSurfLastMsg.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserCloudSurfLastMsg.header.frame_id = "/camera";
        pubLaserCloudSurfLast.publish(laserCloudSurfLastMsg);

        transformSum[0] += imuPitchStart;
        //transformSum[1] += imuYawStart;
        transformSum[2] += imuRollStart;

        systemInited = true;
        continue;
      }

      // minus the predicted motion
      transform[3] -= imuVeloFromStartX * scanPeriod;
      transform[4] -= imuVeloFromStartY * scanPeriod;
      transform[5] -= imuVeloFromStartZ * scanPeriod;

      // when features are sufficient in last cloud
      if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {

        int cornerPointsSharpNum = cornerPointsSharp->points.size();
        int surfPointsFlatNum = surfPointsFlat->points.size();

        for (int iterCount = 0; iterCount < maxIterNumOdom; iterCount++) {
          laserCloudOri->clear();
          //pointSearchCornerLast->clear();
          //pointProjCornerLast->clear();
          //pointSearchSurfLast->clear();
          //pointProjSurfLast->clear();
          coeffSel->clear();
          /*
             1. deal with cornerPointsSharp, saved to laserCloudOri and coeffSel
             2. deal with surfPointsFlat, saved to laserCloudOri and coeffSel
             3. build problem and cv::solve
          */
          for (int i = 0; i < cornerPointsSharpNum; i++) {
            // transform current point to the frame of start time point
            TransformToStart(&cornerPointsSharp->points[i], &pointSel);
            // locate the nearest point in last corner points to this cornerPointsSharp point every 5 iters
            if (iterCount % 5 == 0) {
              /* necessary ?
              std::vector<int> indices;
              pcl::removeNaNFromPointCloud(*laserCloudCornerLast, *laserCloudCornerLast, indices);
              */
              // search the nearest point
              kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
              int closestPointInd = -1, minPointInd2 = -1;
              // when distance to the found nearest point pointSearchInd[0] in last cloud be < 5
              if (pointSearchSqDis[0] < 25) {
                closestPointInd = pointSearchInd[0];
                // get the scan line id of closestPointInd
                // scan_id saved in intensity = int(scan_id) + float(scanPeriod * relTime)
                int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

                float pointSqDis, minPointSqDis2 = 25; // max distance: 5

                // search forward points, find the other point in last cloud
                for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
                  // TODO: j is bounded by cornerPointsSharpNum, can it be used to index laserCloudCornerLast?
                  if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) {
                    break;
                  }
                  // squared distance between the other point and pointSel
                  pointSqDis = sqrDis(laserCloudCornerLast->points[j], pointSel);

                  // the other point must not be in the same scan
                  if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
                    if (pointSqDis < minPointSqDis2) {
                      minPointSqDis2 = pointSqDis;
                      minPointInd2 = j;
                    }
                  }
                }
                // search backward points
                for (int j = closestPointInd - 1; j >= 0; j--) {
                  if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
                    break;
                  }

                  pointSqDis = sqrDis(laserCloudCornerLast->points[j], pointSel.x);

                  if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
                    if (pointSqDis < minPointSqDis2) {
                      minPointSqDis2 = pointSqDis;
                      minPointInd2 = j;
                    }
                  }
                }
              }
              // the first point in last cloud closest to pointSel (distance < 5)
              pointSearchCornerInd1[i] = closestPointInd;
              // the second point in neaby scan within last clound closest to pointSel (distance < 5)
              pointSearchCornerInd2[i] = minPointInd2;
            }

            if (pointSearchCornerInd2[i] >= 0) { // found all two closest points
              tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
              tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

              float x0 = pointSel.x;
              float y0 = pointSel.y;
              float z0 = pointSel.z;
              float x1 = tripod1.x;
              float y1 = tripod1.y;
              float z1 = tripod1.z;
              float x2 = tripod2.x;
              float y2 = tripod2.y;
              float z2 = tripod2.z;

              // cross(pointSel - tripod1, pointSel - tripod2) =
              //  [(y0 - y1) (z0 - z2) - (y0 - y2) (z0 - z1),
              //   (x0 - x2) (z0 - z1) - (x0 - x1) (z0 - z2),
              //   (x0 - x1) (y0 - y2) - (x0 - x2) (y0 - y1)]
              // a012 = |cross(pointSel - tripod1, pointSel - tripod2)|
              float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                              * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                              + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                              * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                              + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                              * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));
              // l12 = |tripod1 - tripod2|
              float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

              // diff(ld2, x0), ld2 is the distance from pointSel to edge
              // (tripod1, tripod2) as defined below
              float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                       + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

              // diff(ld2, y0)
              float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                       - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

              // diff(ld2, z0)
              float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                       + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

              // distance from pointSel to edge (tripod1, tripod2)
              float ld2 = a012 / l12;
              /*
              pointProj = pointSel;
              pointProj.x -= la * ld2;
              pointProj.y -= lb * ld2;
              pointProj.z -= lc * ld2;
              */
              float s = 1.0; // TODO: step? weight?
              if (iterCount >= 5) {
                s = 1 - 1.8 * fabs(ld2);  // TODO: why adjust s like this?
              }

              coeff.x = s * la;
              coeff.y = s * lb;
              coeff.z = s * lc;
              coeff.intensity = s * ld2;

              // apply this correspondence only when
              // s is not too small and distance is not zero
              if (s > 0.1 && ld2 != 0) {
                laserCloudOri->push_back(cornerPointsSharp->points[i]);
                //pointSearchCornerLast->push_back(tripod1);
                //pointSearchCornerLast->push_back(tripod2);
                //pointProjCornerLast->push_back(pointProj);
                coeffSel->push_back(coeff);
              }
            }
          }

          //=================================================================
          // ROS_INFO("corner points finished, dealing with surface points");

          for (int i = 0; i < surfPointsFlatNum; i++) {
            TransformToStart(&surfPointsFlat->points[i], &pointSel);

            if (iterCount % 5 == 0) { // time to find correspondence with planar patches
              kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

              int closestPointInd = -1, minPointInd2 = -1;
              int minPointInd3 = -1; // another point closest to closestPointInd
                                     // [minPointInd2->within] /
                                     // [minPointInd3->not within] same scan
              if (pointSearchSqDis[0] < 25) { // when distance to the found
                                              // nearest point pointSearchInd[0]
                                              // in last cloud be < 5
                closestPointInd = pointSearchInd[0];
                // get the scan_id of closestPointInd
                int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);

                float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
                // search forward points, find the other point in last cloud
                for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
                  // TODO: j is bounded by surfPointsFlatNum, can it be used to index laserCloudSurfLast?
                  if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) {
                    break;
                  }

                  pointSqDis = sqrDis(laserCloudSurfLast->points[j], pointSel);

                  // within same scan
                  if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan) {
                     if (pointSqDis < minPointSqDis2) {
                       minPointSqDis2 = pointSqDis;
                       minPointInd2 = j;
                     }
                  // not within same scan
                  } else {
                     if (pointSqDis < minPointSqDis3) {
                       minPointSqDis3 = pointSqDis;
                       minPointInd3 = j;
                     }
                  }
                }
                // search backward points, find the other point in last cloud
                for (int j = closestPointInd - 1; j >= 0; j--) {
                  if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) {
                    break;
                  }

                  pointSqDis = sqrDis(laserCloudSurfLast->points[j], pointSel);

                  if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan) {
                    if (pointSqDis < minPointSqDis2) {
                      minPointSqDis2 = pointSqDis;
                      minPointInd2 = j;
                    }
                  } else {
                    if (pointSqDis < minPointSqDis3) {
                      minPointSqDis3 = pointSqDis;
                      minPointInd3 = j;
                    }
                  }
                }
              }
              // the planar patch
              pointSearchSurfInd1[i] = closestPointInd;
              pointSearchSurfInd2[i] = minPointInd2;
              pointSearchSurfInd3[i] = minPointInd3;
            }
            // found the planar patch
            if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
              tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
              tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
              tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

              float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) -
                         (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
              float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) -
                         (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
              float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) -
                         (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
              float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

              float ps = length3d(pa, pb, pc);

              if (ps == 0) {
                std::cout<<"FATAL ERROR, surfPointsFlat id: "<<i<<std::endl;
                std::cout<<"pa: "<<pa<<std::endl;
                std::cout<<"pb: "<<pb<<std::endl;
                std::cout<<"pc: "<<pc<<std::endl;
                std::cout<<"pd: "<<pd<<std::endl;
                std::cout<<"tripod1: "<<tripod1<<std::endl;
                std::cout<<"tripod2: "<<tripod2<<std::endl;
                std::cout<<"tripod3: "<<tripod3<<std::endl;
                std::cout<<"pointSearchSurfInd1[i]: "<<pointSearchSurfInd1[i]<<std::endl;
                std::cout<<"pointSearchSurfInd2[i]: "<<pointSearchSurfInd2[i]<<std::endl;
                std::cout<<"pointSearchSurfInd3[i]: "<<pointSearchSurfInd3[i]<<std::endl;
              }

              // normal vector of the plane
              pa /= ps;
              pb /= ps;
              pc /= ps;
              pd /= ps;

              // this is exactly the distance from pointSel to planar patch
              // {tripod1, tripod2, tripod3}
              float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
              /*
              pointProj = pointSel;
              pointProj.x -= pa * pd2;
              pointProj.y -= pb * pd2;
              pointProj.z -= pc * pd2;
              */
              // TODO: why adjust s like this?
              float s = 1.0;
              if (iterCount >= 5) {
                s = 1 - 1.8 * fabs(pd2) / sqrt(length3d(pointSel.x, pointSel.y, pointSel.z));
              }

              coeff.x = s * pa;
              coeff.y = s * pb;
              coeff.z = s * pc;
              coeff.intensity = s * pd2;

              if (s > 0.1 && pd2 != 0) {// apply this correspondence only when
                                        // s is not too small and distance is
                                        // not zero
                laserCloudOri->push_back(surfPointsFlat->points[i]);
                //pointSearchSurfLast->push_back(tripod1);
                //pointSearchSurfLast->push_back(tripod2);
                //pointSearchSurfLast->push_back(tripod3);
                //pointProjSurfLast->push_back(pointProj);
                coeffSel->push_back(coeff);
              }
            }
          }

          int pointSelNum = laserCloudOri->points.size();
          if (pointSelNum < 10) {
            continue;
          }

          cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
          cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
          cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
          cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
          cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
          cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

          for (int i = 0; i < pointSelNum; i++) {
            // the original coordinates (not transformed to start time point) of feature point in current cloud
            pointOri = laserCloudOri->points[i];
            // the scaled gradients: diff(distance, {x, y, z}), and the scaled distance.
            // x/y/z are coordinates of feature points in the starting frame
            coeff = coeffSel->points[i];

            float s = 1.0;

            float srx = sin(s * transform[0]);
            float crx = cos(s * transform[0]);
            float sry = sin(s * transform[1]);
            float cry = cos(s * transform[1]);
            float srz = sin(s * transform[2]);
            float crz = cos(s * transform[2]);
            float tx = s * transform[3];
            float ty = s * transform[4];
            float tz = s * transform[5];

            float arx = (-s*crx*sry*srz*pointOri.x + s*crx*crz*sry*pointOri.y + s*srx*sry*pointOri.z
                      + s*tx*crx*sry*srz - s*ty*crx*crz*sry - s*tz*srx*sry) * coeff.x
                      + (s*srx*srz*pointOri.x - s*crz*srx*pointOri.y + s*crx*pointOri.z
                      + s*ty*crz*srx - s*tz*crx - s*tx*srx*srz) * coeff.y
                      + (s*crx*cry*srz*pointOri.x - s*crx*cry*crz*pointOri.y - s*cry*srx*pointOri.z
                      + s*tz*cry*srx + s*ty*crx*cry*crz - s*tx*crx*cry*srz) * coeff.z;

            float ary = ((-s*crz*sry - s*cry*srx*srz)*pointOri.x
                      + (s*cry*crz*srx - s*sry*srz)*pointOri.y - s*crx*cry*pointOri.z
                      + tx*(s*crz*sry + s*cry*srx*srz) + ty*(s*sry*srz - s*cry*crz*srx)
                      + s*tz*crx*cry) * coeff.x
                      + ((s*cry*crz - s*srx*sry*srz)*pointOri.x
                      + (s*cry*srz + s*crz*srx*sry)*pointOri.y - s*crx*sry*pointOri.z
                      + s*tz*crx*sry - ty*(s*cry*srz + s*crz*srx*sry)
                      - tx*(s*cry*crz - s*srx*sry*srz)) * coeff.z;

            float arz = ((-s*cry*srz - s*crz*srx*sry)*pointOri.x + (s*cry*crz - s*srx*sry*srz)*pointOri.y
                      + tx*(s*cry*srz + s*crz*srx*sry) - ty*(s*cry*crz - s*srx*sry*srz)) * coeff.x
                      + (-s*crx*crz*pointOri.x - s*crx*srz*pointOri.y
                      + s*ty*crx*srz + s*tx*crx*crz) * coeff.y
                      + ((s*cry*crz*srx - s*sry*srz)*pointOri.x + (s*crz*sry + s*cry*srx*srz)*pointOri.y
                      + tx*(s*sry*srz - s*cry*crz*srx) - ty*(s*crz*sry + s*cry*srx*srz)) * coeff.z;

            float atx = -s*(cry*crz - srx*sry*srz) * coeff.x + s*crx*srz * coeff.y
                      - s*(crz*sry + cry*srx*srz) * coeff.z;

            float aty = -s*(cry*srz + crz*srx*sry) * coeff.x - s*crx*crz * coeff.y
                      - s*(sry*srz - cry*crz*srx) * coeff.z;

            float atz = s*crx*sry * coeff.x - s*srx * coeff.y - s*crx*cry * coeff.z;

            float d2 = coeff.intensity;

            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arz;
            matA.at<float>(i, 3) = atx;
            matA.at<float>(i, 4) = aty;
            matA.at<float>(i, 5) = atz;

            matB.at<float>(i, 0) = -0.05 * d2;
          }
          cv::transpose(matA, matAt);
          matAtA = matAt * matA;
          matAtB = matAt * matB;
          // TODO(qmf): check solve speed
          cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

          if (iterCount == 0) { // initialize
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {10, 10, 10, 10, 10, 10};
            for (int i = 5; i >= 0; i--) {
              if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 6; j++) {
                  matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
              } else {
                break;
              }
            }
            matP = matV.inv() * matV2;
          }

          if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;

            //ROS_INFO ("laser odometry degenerate");
          }

          //------- (bug fix: sometime the L-M optimization result matX contains NaN, which will break the whole node)
          if (std::isnan(matX.at<float>(0, 0))
            || std::isnan(matX.at<float>(1, 0))
            || std::isnan(matX.at<float>(2, 0))
            || std::isnan(matX.at<float>(3, 0))
            || std::isnan(matX.at<float>(4, 0))
            || std::isnan(matX.at<float>(5, 0)))
          {
            //ROS_INFO("[USER WARN]laser Odometry: NaN found in var \"matX\", this L-M optimization step is going to be ignored");
            ROS_INFO("iter: %d, %f, %f, %f, %f, %f, %f",
                     iterCount,
                     matX.at<float>(0, 0),
                     matX.at<float>(1, 0),
                     matX.at<float>(2, 0),
                     matX.at<float>(3, 0),
                     matX.at<float>(4, 0),
                     matX.at<float>(5, 0));
          } else{
            //ROS_INFO("[USER WARN]laser Odometry: not NaN found in var \"matX\".");
            ROS_INFO("iter: %d, Updating", iterCount);
            transform[0] += matX.at<float>(0, 0);
            transform[1] += matX.at<float>(1, 0);
            transform[2] += matX.at<float>(2, 0);
            transform[3] += matX.at<float>(3, 0);
            transform[4] += matX.at<float>(4, 0);
            transform[5] += matX.at<float>(5, 0);
          }
          //-------
          float deltaR = sqrt(
                             pow(rad2deg(matX.at<float>(0, 0)), 2)
                           + pow(rad2deg(matX.at<float>(1, 0)), 2)
                           + pow(rad2deg(matX.at<float>(2, 0)), 2));
          float deltaT = sqrt(
                             pow(matX.at<float>(3, 0) * 100, 2)
                           + pow(matX.at<float>(4, 0) * 100, 2)
                           + pow(matX.at<float>(5, 0) * 100, 2));

          if (deltaR < 0.1 && deltaT < 0.1) {
            break;
          }

          // ROS_INFO ("iter: %d, deltaR: %f, deltaT: %f", iterCount, deltaR, deltaT);
        }
      }

      // accumulate transform
      float rx, ry, rz, tx, ty, tz;
      AccumulateRotation(transformSum[0], transformSum[1], transformSum[2],
                         -transform[0], -transform[1] * 1.05, -transform[2],
                         rx, ry, rz);

      float x1 = cos(rz) * (transform[3] - imuShiftFromStartX)
               - sin(rz) * (transform[4] - imuShiftFromStartY);
      float y1 = sin(rz) * (transform[3] - imuShiftFromStartX)
               + cos(rz) * (transform[4] - imuShiftFromStartY);
      float z1 = transform[5] * 1.05 - imuShiftFromStartZ;

      float x2 = x1;
      float y2 = cos(rx) * y1 - sin(rx) * z1;
      float z2 = sin(rx) * y1 + cos(rx) * z1;

      tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
      ty = transformSum[4] - y2;
      tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

      PluginIMURotation(rx, ry, rz, imuPitchStart, imuYawStart, imuRollStart,
                        imuPitchLast, imuYawLast, imuRollLast, rx, ry, rz);

      transformSum[0] = rx;
      transformSum[1] = ry;
      transformSum[2] = rz;
      transformSum[3] = tx;
      transformSum[4] = ty;
      transformSum[5] = tz;

      geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(rz, -rx, -ry);

      laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
      laserOdometry.pose.pose.orientation.x = -geoQuat.y;
      laserOdometry.pose.pose.orientation.y = -geoQuat.z;
      laserOdometry.pose.pose.orientation.z = geoQuat.x;
      laserOdometry.pose.pose.orientation.w = geoQuat.w;
      laserOdometry.pose.pose.position.x = tx;
      laserOdometry.pose.pose.position.y = ty;
      laserOdometry.pose.pose.position.z = tz;
      pubLaserOdometry.publish(laserOdometry);
      // laserOdometryFreq.report();

      laserOdometryTrans.stamp_ = ros::Time().fromSec(timeSurfPointsLessFlat);
      laserOdometryTrans.setRotation(
        tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
      laserOdometryTrans.setOrigin(tf::Vector3(tx, ty, tz));
      tfBroadcaster.sendTransform(laserOdometryTrans);

      int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
      for (int i = 0; i < cornerPointsLessSharpNum; i++) {
        TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
      }

      int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
      for (int i = 0; i < surfPointsLessFlatNum; i++) {
        TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
      }

      frameCount++;
      if (frameCount >= skipFrameNum + 1) {
        int laserCloudFullResNum = laserCloudFullRes->points.size();
        for (int i = 0; i < laserCloudFullResNum; i++) {
          // transform all points in this sweep to end
          TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
        }
      }

      // update laserCloudCornerLast and laserCloudSurfLast with new data
      pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = cornerPointsLessSharp;
      cornerPointsLessSharp = laserCloudCornerLast;
      laserCloudCornerLast = laserCloudTemp;

      laserCloudTemp = surfPointsLessFlat;
      surfPointsLessFlat = laserCloudSurfLast;
      laserCloudSurfLast = laserCloudTemp;

      laserCloudCornerLastNum = laserCloudCornerLast->points.size();
      laserCloudSurfLastNum = laserCloudSurfLast->points.size();
      if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
        kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
        kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
      }

      if (frameCount >= skipFrameNum + 1) {
        frameCount = 0;

        sensor_msgs::PointCloud2 laserCloudCornerLastMsg;
        pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLastMsg);
        laserCloudCornerLastMsg.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserCloudCornerLastMsg.header.frame_id = "/camera";
        pubLaserCloudCornerLast.publish(laserCloudCornerLastMsg);// all transformed to sweep end

        sensor_msgs::PointCloud2 laserCloudSurfLastMsg;
        pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLastMsg);
        laserCloudSurfLastMsg.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserCloudSurfLastMsg.header.frame_id = "/camera";
        pubLaserCloudSurfLast.publish(laserCloudSurfLastMsg);// all transformed to sweep end

        sensor_msgs::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserCloudFullRes3.header.frame_id = "/camera";
        pubLaserCloudFullRes.publish(laserCloudFullRes3);// all transformed to sweep end

        // registeredLaserCloudFreq.report();
      }

      /*sensor_msgs::PointCloud2 pc12;
      pcl::toROSMsg(*pointSearchCornerLast, pc12);
      pc12.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
      pc12.header.frame_id = "/camera";
      pub1.publish(pc12);

      sensor_msgs::PointCloud2 pc22;
      pcl::toROSMsg(*pointSearchSurfLast, pc22);
      pc22.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
      pc22.header.frame_id = "/camera";
      pub2.publish(pc22);

      sensor_msgs::PointCloud2 pc32;
      pcl::toROSMsg(*pointProjCornerLast, pc32);
      pc32.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
      pc32.header.frame_id = "/camera";
      pub3.publish(pc32);

      sensor_msgs::PointCloud2 pc42;
      pcl::toROSMsg(*pointProjSurfLast, pc42);
      pc42.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
      pc42.header.frame_id = "/camera";
      pub4.publish(pc42);*/
    }

    rate.sleep();
  }

  return 0;
}
