#include <jsk_interactive_marker/assoc_marker.h>
#include <std_srvs/Empty.h>

AssocMarker::AssocMarker():
  nh_(),
  pnh_("~")
{
  pnh_.param<std::string>("parent_name", parent_name_, "");
  pnh_.param<std::string>("parent_topic", parent_topic_, "parent_topic");
  pnh_.param<std::string>("child_name", child_name_, "");
  pnh_.param<std::string>("child_topic", child_topic_, "child_topic");
}



AssocMarker::~AssocMarker(){};
int main(int argc, char **argv){
  ros::init(argc, argv, "AssocMarker");
  AssocMarker assoc_marker;
  ros::spin();
}
