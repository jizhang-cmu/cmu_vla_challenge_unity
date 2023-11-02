#ifndef WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H
#define WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H

#include <sstream>
#include <ros/ros.h>
#include <QObject>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Pose2D.h>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"
#include "rviz/default_plugin/tools/pose_tool.h"

namespace rviz
{
class StringProperty;

class WaypointTool : public PoseTool
{
  Q_OBJECT
public:
  WaypointTool();
  virtual ~WaypointTool()
  {
  }
  virtual void onInitialize();

protected:
  virtual void onPoseSet(double x, double y, double theta);

private Q_SLOTS:
  void updateTopic();

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher pub_joy_;

  StringProperty* topic_property_;
};
}

#endif  // WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H
