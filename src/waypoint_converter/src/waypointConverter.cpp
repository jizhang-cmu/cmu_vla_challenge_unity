#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string trav_area_file_dir;
string boundary_file_dir;
double waypointXYRadius = 0.2;
double waypointProjDis = 0.5;
bool twoWayHeading = true;
double frameRate = 5.0;
bool checkTravArea = true;
bool waypointTravAdj = false;
double adjDisThre = 5.0;
double searchDisThre = 3.0;
double travDisThre = 0.1;
double extDis = 0;
int yawConfig = 0;
double speed = 1.0;
double speed2 = speed;
bool sendSpeed = true;
bool sendBoundary = true;

pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr travArea(new pcl::PointCloud<pcl::PointXYZ>());
pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeTravArea(new pcl::KdTreeFLANN<pcl::PointXYZ>());

float vehicleX = 0, vehicleY = 0, vehicleZ = 0, vehicleYaw = 0;
float waypointX = 0, waypointY = 0, waypointYaw = 0;
float waypointX2 = 0, waypointY2 = 0, waypointYaw2 = 0;

double curTime = 0, poseTime = 0, waypointTime = 0, travAreaTime = 0;
bool waypointInit = false;
bool waypointReached = false;
bool waypointAdj = false;

std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;

geometry_msgs::msg::PointStamped waypointMsgs;

rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubWaypointPtr;

// reading boundary from file function
void readBoundaryFile()
{
  FILE* boundary_file = fopen(boundary_file_dir.c_str(), "r");
  if (boundary_file == NULL) {
    sendBoundary = false;
    printf ("\nCannot read boundary.ply file.\n\n");
    return;
  }

  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(boundary_file, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(boundary_file, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  boundary->clear();
  pcl::PointXYZ point;
  int val1, val2, val3;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(boundary_file, "%f", &point.x);
    val2 = fscanf(boundary_file, "%f", &point.y);
    val3 = fscanf(boundary_file, "%f", &point.z);

    if (val1 != 1 || val2 != 1 || val3 != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    boundary->push_back(point);
  }

  fclose(boundary_file);
}

// vehicle pose callback function
void poseHandler(const nav_msgs::msg::Odometry::ConstSharedPtr pose)
{
  curTime = rclcpp::Time(pose->header.stamp).seconds();

  if (!waypointInit) return;
  if (curTime - poseTime < 1.0 / frameRate) return;
  poseTime = curTime;

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = pose->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleYaw = yaw;
  vehicleX = pose->pose.pose.position.x;
  vehicleY = pose->pose.pose.position.y;
  vehicleZ = pose->pose.pose.position.z;
  
  float disX = vehicleX - waypointX2;
  float disY = vehicleY - waypointY2;
  float dis = sqrt(disX * disX + disY * disY);

  if (dis < waypointXYRadius) waypointReached = true;
  if (dis < adjDisThre && waypointTravAdj) waypointAdj = true;

  if (waypointReached) {
    if (yawConfig == -1) waypointYaw2 = vehicleYaw;
    else if (yawConfig == 1) waypointYaw2 = atan2(waypointY - vehicleY, waypointX - vehicleX);
    else if (yawConfig == 2) waypointYaw2 = atan2(waypointY - vehicleY, waypointX - vehicleX) + PI / 2;

    if (waypointYaw2 < -PI) waypointYaw2 += 2 * PI;
    else if (waypointYaw2 > PI) waypointYaw2 -= 2 * PI;

    float angDis = waypointYaw2 - vehicleYaw;
    if (angDis < -PI) angDis += 2 * PI;
    else if (angDis > PI) angDis -= 2 * PI;

    float projYaw = waypointYaw2;
    if (!twoWayHeading) {
      if (angDis < -PI / 3) projYaw = vehicleYaw - PI / 3;
      else if (angDis > PI / 3) projYaw = vehicleYaw + PI / 3;
    }

    waypointX2 = vehicleX + waypointProjDis * cos(projYaw);
    waypointY2 = vehicleY + waypointProjDis * sin(projYaw);
    speed2 = 0;
  } else if (waypointAdj) {
    pcl::PointXYZ point;
    point.x = vehicleX;
    point.y = vehicleY;
    point.z = 0;

    kdtreeTravArea->radiusSearch(point, searchDisThre, pointSearchInd, pointSearchSqDis);

    int minInd = -1;
    float minDis = 1000000;
    int pointSearchNum = pointSearchInd.size();
    for (int ind = 0; ind < pointSearchNum; ind++) {
      float disX2 = travArea->points[pointSearchInd[ind]].x - waypointX;
      float disY2 = travArea->points[pointSearchInd[ind]].y - waypointY;
      float dis2 = sqrt(disX2 * disX2 + disY2 * disY2);

      if (minDis > dis2) {
        minInd = ind;
        minDis = dis2;
      }
    }

    if (minInd >= 0) {
      float disX2 = travArea->points[pointSearchInd[minInd]].x - waypointX;
      float disY2 = travArea->points[pointSearchInd[minInd]].y - waypointY;
      float dis2 = sqrt(disX2 * disX2 + disY2 * disY2);

      waypointX2 = travArea->points[pointSearchInd[minInd]].x;
      waypointY2 = travArea->points[pointSearchInd[minInd]].y;

      if (extDis != 0) {
        waypointX2 += extDis * disX2 / dis2;
        waypointY2 += extDis * disY2 / dis2;
      }
    }
  }

  waypointMsgs.header.stamp = rclcpp::Time(static_cast<uint64_t>(curTime * 1e9));
  waypointMsgs.point.x = waypointX2;
  waypointMsgs.point.y = waypointY2;
  waypointMsgs.point.z = vehicleZ;
  pubWaypointPtr->publish(waypointMsgs);
}

// waypoint with heading callback function
void waypointHandler(const geometry_msgs::msg::Pose2D::ConstSharedPtr waypoint)
{
  if (checkTravArea) {
    pcl::PointXYZ point;
    point.x = waypoint->x;
    point.y = waypoint->y;
    point.z = 0;
    
    kdtreeTravArea->nearestKSearch(point, 1, pointSearchInd, pointSearchSqDis);
    
    float disThre = travDisThre;
    if (waypointTravAdj) disThre = adjDisThre;

    if (pointSearchSqDis[0] > disThre * disThre) {
      printf("\nWaypoint not in traversable area, skip.\n");
      return;
    }
  }

  waypointX = waypoint->x;
  waypointY = waypoint->y;
  waypointYaw = waypoint->theta;
  waypointX2 = waypointX;
  waypointY2 = waypointY;
  waypointYaw2 = waypointYaw;
  speed2 = speed;
  
  waypointInit = true;
  waypointReached = false;
  waypointAdj = false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("waypointConverter");

  nh->declare_parameter<std::string>("trav_area_file_dir", trav_area_file_dir);
  nh->declare_parameter<std::string>("boundary_file_dir", boundary_file_dir);
  nh->declare_parameter<double>("waypointXYRadius", waypointXYRadius);
  nh->declare_parameter<double>("waypointProjDis", waypointProjDis);
  nh->declare_parameter<double>("twoWayHeading", twoWayHeading);
  nh->declare_parameter<double>("frameRate", frameRate);
  nh->declare_parameter<bool>("checkTravArea", checkTravArea);
  nh->declare_parameter<bool>("waypointTravAdj", waypointTravAdj);
  nh->declare_parameter<double>("adjDisThre", adjDisThre);
  nh->declare_parameter<double>("searchDisThre", searchDisThre);
  nh->declare_parameter<double>("travDisThre", travDisThre);
  nh->declare_parameter<double>("extDis", extDis);
  nh->declare_parameter<int>("yawConfig", yawConfig);
  nh->declare_parameter<double>("speed", speed);
  nh->declare_parameter<bool>("sendSpeed", sendSpeed);
  nh->declare_parameter<bool>("sendBoundary", sendBoundary);

  nh->get_parameter("trav_area_file_dir", trav_area_file_dir);
  nh->get_parameter("boundary_file_dir", boundary_file_dir);
  nh->get_parameter("waypointXYRadius", waypointXYRadius);
  nh->get_parameter("waypointProjDis", waypointProjDis);
  nh->get_parameter("twoWayHeading", twoWayHeading);
  nh->get_parameter("frameRate", frameRate);
  nh->get_parameter("checkTravArea", checkTravArea);
  nh->get_parameter("waypointTravAdj", waypointTravAdj);
  nh->get_parameter("adjDisThre", adjDisThre);
  nh->get_parameter("searchDisThre", searchDisThre);
  nh->get_parameter("travDisThre", travDisThre);
  nh->get_parameter("extDis", extDis);
  nh->get_parameter("yawConfig", yawConfig);
  nh->get_parameter("speed", speed);
  nh->get_parameter("sendSpeed", sendSpeed);
  nh->get_parameter("sendBoundary", sendBoundary);

  speed2 = speed;

  auto subPose = nh->create_subscription<nav_msgs::msg::Odometry> ("/state_estimation", 5, poseHandler);

  auto subWaypoint = nh->create_subscription<geometry_msgs::msg::Pose2D> ("/way_point_with_heading", 5, waypointHandler);

  auto pubWaypoint = nh->create_publisher<geometry_msgs::msg::PointStamped> ("/way_point", 5);
  waypointMsgs.header.frame_id = "map";

  pubWaypointPtr = pubWaypoint;

  auto pubTravArea = nh->create_publisher<sensor_msgs::msg::PointCloud2> ("/traversable_area", 1);
  sensor_msgs::msg::PointCloud2 travArea2;

  auto pubSpeed = nh->create_publisher<std_msgs::msg::Float32> ("/speed", 5);
  std_msgs::msg::Float32 speedMsgs;

  auto pubBoundary = nh->create_publisher<geometry_msgs::msg::PolygonStamped> ("/navigation_boundary", 5);
  geometry_msgs::msg::PolygonStamped boundaryMsgs;
  boundaryMsgs.header.frame_id = "map";

  if (checkTravArea) {
    pcl::PLYReader ply_reader;
    if (ply_reader.read(trav_area_file_dir, *travArea) == -1) {
      checkTravArea = false;
      printf("\nCouldn't read traversable_area.ply file.\n\n");
    } else {
      int travAreaSize = travArea->points.size();
      if (travAreaSize > 0) {
        for (int i = 0; i < travAreaSize; i++) {
          travArea->points[i].z = 0;
        }

        kdtreeTravArea->setInputCloud(travArea);
        pcl::toROSMsg(*travArea, travArea2);
      } else {
        checkTravArea = false;
      }
    }
  }

  if (sendBoundary) {
    readBoundaryFile();

    int boundarySize = boundary->points.size();
    boundaryMsgs.polygon.points.resize(boundarySize);
    for (int i = 0; i < boundarySize; i++) {
      boundaryMsgs.polygon.points[i].x = boundary->points[i].x;
      boundaryMsgs.polygon.points[i].y = boundary->points[i].y;
      boundaryMsgs.polygon.points[i].z = boundary->points[i].z;
    }
  }

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (curTime - waypointTime > 1.0 / frameRate) {
      if (sendSpeed) {
        speedMsgs.data = speed2;
        pubSpeed->publish(speedMsgs);
      }

      if (sendBoundary) {
        boundaryMsgs.header.stamp = rclcpp::Time(static_cast<uint64_t>(curTime * 1e9));
        pubBoundary->publish(boundaryMsgs);
      }

      waypointTime = curTime;
    }

    if (curTime - travAreaTime > 1.0 && checkTravArea) {
      travArea2.header.frame_id = "map";
      travArea2.header.stamp = rclcpp::Time(static_cast<uint64_t>(curTime * 1e9));
      pubTravArea->publish(travArea2);
      
      travAreaTime = curTime;
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
