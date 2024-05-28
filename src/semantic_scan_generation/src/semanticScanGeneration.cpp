#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace std;
using namespace cv;

const double PI = 3.1415926;

double cameraOffsetZ = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudSeg(new pcl::PointCloud<pcl::PointXYZRGB>());

const int stackNum = 400;
float lidarXStack[stackNum];
float lidarYStack[stackNum];
float lidarZStack[stackNum];
float lidarRollStack[stackNum];
float lidarPitchStack[stackNum];
float lidarYawStack[stackNum];
double odomTimeStack[stackNum];
int odomIDPointer = -1;
int imageIDPointer = 0;

bool imageInit = false;
double imageTime = 0;

bool newLaserCloud = false;
double laserCloudTime = 0;

cv_bridge::CvImageConstPtr segImageCv;

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  odomIDPointer = (odomIDPointer + 1) % stackNum;
  odomTimeStack[odomIDPointer] = rclcpp::Time(odom->header.stamp).seconds();
  lidarXStack[odomIDPointer] = odom->pose.pose.position.x;
  lidarYStack[odomIDPointer] = odom->pose.pose.position.y;
  lidarZStack[odomIDPointer] = odom->pose.pose.position.z;
  lidarRollStack[odomIDPointer] = roll;
  lidarPitchStack[odomIDPointer] = pitch;
  lidarYawStack[odomIDPointer] = yaw;
}

void semImageHandler(const sensor_msgs::msg::Image::ConstSharedPtr image)
{
  imageTime = rclcpp::Time(image->header.stamp).seconds();
  segImageCv = cv_bridge::toCvShare(image, "bgr8");

  imageInit = true;
}

void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudIn)
{
  laserCloudTime = rclcpp::Time(laserCloudIn->header.stamp).seconds();

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  newLaserCloud = true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("semanticScanGeneration");

  nh->declare_parameter<double>("cameraOffsetZ", cameraOffsetZ);

  nh->get_parameter("cameraOffsetZ", cameraOffsetZ);

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry> ("/state_estimation", 50, odomHandler);

  auto subSegImage = nh->create_subscription<sensor_msgs::msg::Image> ("/camera/semantic_image", 2, semImageHandler);

  auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2> ("/registered_scan", 2, laserCloudHandler);

  auto pubLaserCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2> ("/semantic_scan", 2);

  rclcpp::Rate rate(200);
  bool status = rclcpp::ok();
  while (status)
  {
    rclcpp::spin_some(nh);

    if (imageInit && newLaserCloud) {
      newLaserCloud = false;

      int laserCloudSize = laserCloud->points.size();
      if (laserCloudSize <= 0) continue;

      if (odomIDPointer < 0) continue;
      while (odomTimeStack[imageIDPointer] < imageTime - 0.001 &&
             imageIDPointer != (odomIDPointer + 1) % stackNum) {
        imageIDPointer = (imageIDPointer + 1) % stackNum;
      }
      if (fabs(odomTimeStack[imageIDPointer] - imageTime) > 0.001) continue;

      float lidarX = lidarXStack[imageIDPointer];
      float lidarY = lidarYStack[imageIDPointer];
      float lidarZ = lidarZStack[imageIDPointer];
      float lidarRoll = lidarRollStack[imageIDPointer];
      float lidarPitch = lidarPitchStack[imageIDPointer];
      float lidarYaw = lidarYawStack[imageIDPointer];

      int imageWidth = segImageCv->image.size().width;
      int imageHeight = segImageCv->image.size().height;

      float sinLidarRoll = sin(lidarRoll);
      float cosLidarRoll = cos(lidarRoll);
      float sinLidarPitch = sin(lidarPitch);
      float cosLidarPitch = cos(lidarPitch);
      float sinLidarYaw = sin(lidarYaw);
      float cosLidarYaw = cos(lidarYaw);

      pcl::PointXYZRGB point;
      laserCloudSeg->clear();
      for (int i = 0; i < laserCloudSize; i++) {
        float x1 = laserCloud->points[i].x - lidarX;
        float y1 = laserCloud->points[i].y - lidarY;
        float z1 = laserCloud->points[i].z - lidarZ;

        float x2 = x1 * cosLidarYaw + y1 * sinLidarYaw;
        float y2 = -x1 * sinLidarYaw + y1 * cosLidarYaw;
        float z2 = z1;

        float x3 = x2 * cosLidarPitch - z2 * sinLidarPitch;
        float y3 = y2;
        float z3 = x2 * sinLidarPitch + z2 * cosLidarPitch;

        float x4 = x3;
        float y4 = y3 * cosLidarRoll + z3 * sinLidarRoll;
        float z4 = -y3 * sinLidarRoll + z3 * cosLidarRoll - cameraOffsetZ;

        float horiDis = sqrt(x4 * x4 + y4 * y4);
        int horiPixelID = -imageWidth / (2 * PI) * atan2(y4, x4) + imageWidth / 2 + 1;
        int vertPixelID = -imageWidth / (2 * PI) * atan(z4 / horiDis) + imageHeight / 2 + 1;
        int pixelID = imageWidth * vertPixelID + horiPixelID;

        if (horiPixelID >= 0 && horiPixelID < imageWidth && vertPixelID >= 0 && vertPixelID < imageHeight) {
          point.x = laserCloud->points[i].x;
          point.y = laserCloud->points[i].y;
          point.z = laserCloud->points[i].z;
          point.b = segImageCv->image.data[3 * pixelID];
          point.g = segImageCv->image.data[3 * pixelID + 1];
          point.r = segImageCv->image.data[3 * pixelID + 2];

          laserCloudSeg->push_back(point);
        }
      }

      sensor_msgs::msg::PointCloud2 laserCloudOut;
      pcl::toROSMsg(*laserCloudSeg, laserCloudOut);
      laserCloudOut.header.stamp = rclcpp::Time(static_cast<uint64_t>(laserCloudTime * 1e9));
      laserCloudOut.header.frame_id = "map";
      pubLaserCloud->publish(laserCloudOut);
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
