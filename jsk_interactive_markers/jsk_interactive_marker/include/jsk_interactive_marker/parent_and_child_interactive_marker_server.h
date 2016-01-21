#ifndef JSK_INTERACTIVE_MARKER_PARENT_AND_CHILD_INTERACTIVE_MARKER_SERVER_H_
#define JSK_INTERACTIVE_MARKER_PARENT_AND_CHILD_INTERACTIVE_MARKER_SERVER_H_

#include <interactive_markers/interactive_marker_server.h>
#include <std_srvs/Empty.h>
#include <jsk_interactive_marker/GetTransformableMarkerPose.h>
#include <jsk_interactive_marker/SetParentMarker.h>
#include <jsk_interactive_marker/RemoveParentMarker.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

namespace jsk_interactive_marker
{
  class ParentMarkerInformation{
  public:
    ParentMarkerInformation(std::string p_t_n, std::string p_m_n, Eigen::Affine3d r_p)
    {
      parent_topic_name = p_t_n;
      parent_marker_name = p_m_n;
      relative_pose = r_p;
    }
    ParentMarkerInformation()
    {
    }
    std::string parent_topic_name;
    std::string parent_marker_name;
    Eigen::Affine3d relative_pose;
  };

  class ParentAndChildInteractiveMarkerServer: public interactive_markers::InteractiveMarkerServer
  {
  public:
    ParentAndChildInteractiveMarkerServer(const std::string &topic_ns, const std::string &server_id="", bool spin_thread = false);
    ros::ServiceServer set_parent_srv_;
    ros::ServiceServer remove_parent_srv_;
    ros::ServiceServer get_marker_pose_srv_;
    tf::TransformListener tf_listener_;
    std::string topic_server_name_;
    std::string get_marker_pose_service_name_;
    std::string set_parent_service_name_;
    std::string remove_parent_service_name_;
    // map of <map of markers with server> // including self association
    // access: child -> <map <parent_topic_name & parent_marker_name & pose>>
    std::map <std::string, ParentMarkerInformation> association_list_;
    // map of subscriber
    // access: parent server -> subscriber
    std::map <std::string, ros::Subscriber> parent_update_subscribers_; // self association is not included
    std::map <std::string, ros::Subscriber> parent_feedback_subscribers_; // self association is not included
    std::map <std::string, int> parent_subscriber_nums_;
    bool setParentService(jsk_interactive_marker::SetParentMarker::Request &req, jsk_interactive_marker::SetParentMarker::Response &res);
    void renewPoseWithParent(std::map <std::string, ParentMarkerInformation>::iterator assoc_it_, geometry_msgs::Pose parent_pose, std_msgs::Header parent_header);
    bool removeParentService(jsk_interactive_marker::RemoveParentMarker::Request &req, jsk_interactive_marker::RemoveParentMarker::Response &res);
    bool registerAssociationItself(std::string parent_marker_name, std::string parent_topic_name, std::string child_marker_name, geometry_msgs::PoseStamped child_pose_stamped);
    bool registerAssociationWithOtherNode(std::string parent_marker_name, std::string parent_topic_name, std::string child_marker_name, geometry_msgs::PoseStamped child_pose_stamped);
    bool registerAssociation(std::string parent_marker_name, std::string parent_topic_name, std::string child_marker_name, geometry_msgs::PoseStamped child_pose_stamped, geometry_msgs::PoseStamped parent_pose_stamped);
    bool getMarkerPose(std::string target_name, geometry_msgs::PoseStamped &pose_stamped);
    bool getMarkerPoseService(jsk_interactive_marker::GetTransformableMarkerPose::Request &req,jsk_interactive_marker::GetTransformableMarkerPose::Response &res);
    ros::NodeHandle n_;
    geometry_msgs::Pose getRelativePose(geometry_msgs::PoseStamped parent_pose_stamped, geometry_msgs::PoseStamped child_pose_stamped);
    void selfFeedbackCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void parentUpdateCb(const visualization_msgs::InteractiveMarkerUpdateConstPtr &update, std::string parent_topic_name);
    void parentFeedbackCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string parent_topic_name);
    void applyChanges();
    bool setCallback(const std::string &name, FeedbackCallback feedback_cb, uint8_t feedback_type=DEFAULT_FEEDBACK_CB);
  };

}

#endif
