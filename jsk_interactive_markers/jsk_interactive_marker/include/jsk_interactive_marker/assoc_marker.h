#ifndef ASSOC_MARKER
#define ASSOC_MARKER

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class AssocTopic
{
 public:
  std::string parent_topic_ns;
  AssocTopic();
  ~AssocTopic();
};

class AssocMarker
{
 public:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_update_;
  ros::Subscriber sub_feedback_;
  std::string parent_name_;
  std::string parent_topic_;
  std::string child_name_;
  std::string child_topic_;
  AssocMarker();
  ~AssocMarker();
  bool assoc_cb();
  bool disassoc_cb();
};

#endif
