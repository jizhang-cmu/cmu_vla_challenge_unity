#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

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
double travDisThre = 0.1;
double speed = 1.0;
double speed2 = speed;
bool sendSpeed = true;
bool sendBoundary = true;

pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr travArea(new pcl::PointCloud<pcl::PointXYZ>());
pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeTravArea(new pcl::KdTreeFLANN<pcl::PointXYZ>());

float vehicleX = 0, vehicleY = 0, vehicleZ = 0, vehicleYaw = 0;
float waypointX = 0, waypointY = 0, waypointYaw = 0;

double curTime = 0, waypointTime = 0, travAreaTime = 0;
bool waypointInit = false;
bool waypointReached = false;

std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;

ros::Publisher *pubWaypointPtr = NULL;
geometry_msgs::PointStamped waypointMsgs;

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
void poseHandler(const nav_msgs::Odometry::ConstPtr& pose)
{
  curTime = pose->header.stamp.toSec();

  if (!waypointInit) return;

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = pose->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleYaw = yaw;
  vehicleX = pose->pose.pose.position.x;
  vehicleY = pose->pose.pose.position.y;
  vehicleZ = pose->pose.pose.position.z;
  
  float disX = vehicleX - waypointX;
  float disY = vehicleY - waypointY;
  float dis = sqrt(disX * disX + disY * disY);

  if (dis < waypointXYRadius) waypointReached = true;
  
  if (waypointReached) {
    float angDis = waypointYaw - vehicleYaw;
    if (angDis < -PI) angDis += 2 * PI;
    else if (angDis > PI) angDis -= 2 * PI;
    
    float projYaw = waypointYaw;
    if (!twoWayHeading) {
      if (angDis < -PI / 3) projYaw = vehicleYaw - PI / 3;
      else if (angDis > PI / 3) projYaw = vehicleYaw + PI / 3;
    }

    waypointX = vehicleX + waypointProjDis * cos(projYaw);
    waypointY = vehicleY + waypointProjDis * sin(projYaw);
    speed2 = 0;
  }
  
  waypointMsgs.header.stamp = ros::Time().fromSec(curTime);
  waypointMsgs.point.x = waypointX;
  waypointMsgs.point.y = waypointY;
  waypointMsgs.point.z = vehicleZ;
  pubWaypointPtr->publish(waypointMsgs);
}

// waypoint with heading callback function
void waypointHandler(const geometry_msgs::Pose2D::ConstPtr& waypoint)
{
  if (checkTravArea) {
    pcl::PointXYZ point;
    point.x = waypoint->x;
    point.y = waypoint->y;
    point.z = 0;
    
    kdtreeTravArea->nearestKSearch(point, 1, pointSearchInd, pointSearchSqDis);
    
    if (pointSearchSqDis[0] > travDisThre * travDisThre) {
      printf("\nWaypoint not in traversable area, skip.\n");
      return;
    }
  }

  waypointX = waypoint->x;
  waypointY = waypoint->y;
  waypointYaw = waypoint->theta; 
  speed2 = speed;
  
  waypointInit = true;
  waypointReached = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypointConverter");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("trav_area_file_dir", trav_area_file_dir);
  nhPrivate.getParam("boundary_file_dir", boundary_file_dir);
  nhPrivate.getParam("waypointXYRadius", waypointXYRadius);
  nhPrivate.getParam("waypointProjDis", waypointProjDis);
  nhPrivate.getParam("twoWayHeading", twoWayHeading);
  nhPrivate.getParam("frameRate", frameRate);
  nhPrivate.getParam("checkTravArea", checkTravArea);
  nhPrivate.getParam("travDisThre", travDisThre);
  nhPrivate.getParam("speed", speed);
  nhPrivate.getParam("sendSpeed", sendSpeed);
  nhPrivate.getParam("sendBoundary", sendBoundary);

  speed2 = speed;

  ros::Subscriber subPose = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, poseHandler);

  ros::Subscriber subWaypoint = nh.subscribe<geometry_msgs::Pose2D> ("/way_point_with_heading", 5, waypointHandler);

  ros::Publisher pubWaypoint = nh.advertise<geometry_msgs::PointStamped> ("/way_point", 5);
  pubWaypointPtr = &pubWaypoint;
  waypointMsgs.header.frame_id = "map";

  ros::Publisher pubTravArea = nh.advertise<sensor_msgs::PointCloud2> ("/traversable_area", 1);
  sensor_msgs::PointCloud2 travArea2;

  ros::Publisher pubSpeed = nh.advertise<std_msgs::Float32> ("/speed", 5);
  std_msgs::Float32 speedMsgs;

  ros::Publisher pubBoundary = nh.advertise<geometry_msgs::PolygonStamped> ("/navigation_boundary", 5);
  geometry_msgs::PolygonStamped boundaryMsgs;
  boundaryMsgs.header.frame_id = "map";

  if (checkTravArea) {
    pcl::PLYReader ply_reader;
    if (ply_reader.read(trav_area_file_dir, *travArea) == -1) {
      checkTravArea = false;
      printf("\nCouldn't read traversable_area.ply file.\n\n");
    } else {
      int travAreaSize = travArea->points.size();
      for (int i = 0; i < travAreaSize; i++) {
        travArea->points[i].z = 0;
      }

      kdtreeTravArea->setInputCloud(travArea);
      pcl::toROSMsg(*travArea, travArea2);
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

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (curTime - waypointTime > 1.0 / frameRate) {
      if (sendSpeed) {
        speedMsgs.data = speed2;
        pubSpeed.publish(speedMsgs);
      }

      if (sendBoundary) {
        boundaryMsgs.header.stamp = ros::Time().fromSec(curTime);
        pubBoundary.publish(boundaryMsgs);
      }

      waypointTime = curTime;
    }

    if (curTime - travAreaTime > 1.0 && checkTravArea) {
      travArea2.header.frame_id = "map";
      travArea2.header.stamp = ros::Time().fromSec(curTime);
      pubTravArea.publish(travArea2);
      
      travAreaTime = curTime;
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
