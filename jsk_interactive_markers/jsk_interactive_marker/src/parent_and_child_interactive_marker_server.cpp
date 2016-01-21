#include <jsk_interactive_marker/parent_and_child_interactive_marker_server.h>

namespace jsk_interactive_marker
{
  ParentAndChildInteractiveMarkerServer::ParentAndChildInteractiveMarkerServer(const std::string &topic_ns, const std::string &server_id, bool spin_thread) : InteractiveMarkerServer(topic_ns, server_id, spin_thread)
  {
    topic_server_name_ = topic_ns;
    get_marker_pose_service_name_ = topic_ns + "/get_marker_pose";
    set_parent_service_name_ = topic_ns + "/set_parent_marker";
    remove_parent_service_name_ = topic_ns + "/remove_parent_marker";
    get_marker_pose_srv_ = n_.advertiseService(get_marker_pose_service_name_, &ParentAndChildInteractiveMarkerServer::getMarkerPoseService, this);
    set_parent_srv_ = n_.advertiseService(set_parent_service_name_, &ParentAndChildInteractiveMarkerServer::setParentService, this);
    remove_parent_srv_ = n_.advertiseService(remove_parent_service_name_, &ParentAndChildInteractiveMarkerServer::removeParentService, this);

  }
  bool ParentAndChildInteractiveMarkerServer::setParentService(jsk_interactive_marker::SetParentMarker::Request &req, jsk_interactive_marker::SetParentMarker::Response &res)
  {
    geometry_msgs::PoseStamped child_pose_stamped;
    // get current pose stamped
    getMarkerPose(req.child_marker_name, child_pose_stamped);
    if (get_marker_pose_service_name_ == (req.parent_topic_name + "/get_marker_pose"))
      // self association
    {
      if (!registerAssociationItself(req.parent_marker_name, req.parent_topic_name, req.child_marker_name, child_pose_stamped))
        return false;
    }
    else{ // refer to different server
      if (registerAssociationWithOtherNode(req.parent_marker_name, req.parent_topic_name, req.child_marker_name, child_pose_stamped))
      {

        if (parent_subscriber_nums_.find(req.parent_topic_name) == parent_subscriber_nums_.end() || parent_subscriber_nums_[req.parent_topic_name] == 0 )
          {
            // register subscriber
            parent_subscriber_nums_[req.parent_topic_name] = 1;
            parent_update_subscribers_[req.parent_topic_name] = n_.subscribe<visualization_msgs::InteractiveMarkerUpdate>(req.parent_topic_name + "/update", 1, boost::bind(&ParentAndChildInteractiveMarkerServer::parentUpdateCb, this, _1, req.parent_topic_name));
            parent_feedback_subscribers_[req.parent_topic_name] = n_.subscribe<visualization_msgs::InteractiveMarkerFeedback>(req.parent_topic_name + "/feedback", 1, boost::bind(&ParentAndChildInteractiveMarkerServer::parentFeedbackCb, this, _1, req.parent_topic_name));
          }
        else
          {
            parent_subscriber_nums_[req.parent_topic_name] += 1;
          }
      }
      else{
        return false;
      }
    }
    // calculate relative pose
    setCallback(req.child_marker_name, boost::bind(&ParentAndChildInteractiveMarkerServer::selfFeedbackCb, this, _1), visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP);
  }
  bool ParentAndChildInteractiveMarkerServer::registerAssociationItself(std::string parent_marker_name, std::string parent_topic_name, std::string child_marker_name, geometry_msgs::PoseStamped child_pose_stamped)
  {
    geometry_msgs::PoseStamped parent_pose_stamped;
    getMarkerPose(parent_marker_name, parent_pose_stamped);
    registerAssociation(parent_marker_name, parent_topic_name, child_marker_name, child_pose_stamped, parent_pose_stamped);
  }
  bool ParentAndChildInteractiveMarkerServer::registerAssociationWithOtherNode(std::string parent_marker_name, std::string parent_topic_name, std::string child_marker_name, geometry_msgs::PoseStamped child_pose_stamped)
  {
    geometry_msgs::PoseStamped parent_pose_stamped;
    ros::ServiceClient client = n_.serviceClient<jsk_interactive_marker::GetTransformableMarkerPose>(parent_topic_name + "/get_marker_pose");
    jsk_interactive_marker::GetTransformableMarkerPose srv;
    srv.request.target_name = parent_marker_name;
    if (!client.call(srv)){
        return false;
    }
    parent_pose_stamped = srv.response.pose_stamped;
    return registerAssociation(parent_marker_name, parent_topic_name, child_marker_name, child_pose_stamped, parent_pose_stamped);
  }
  bool ParentAndChildInteractiveMarkerServer::registerAssociation(std::string parent_marker_name, std::string parent_topic_name, std::string child_marker_name, geometry_msgs::PoseStamped child_pose_stamped, geometry_msgs::PoseStamped parent_pose_stamped)
  {
    geometry_msgs::PoseStamped parent_pose_stamped_transed;
    if (tf_listener_.waitForTransform(child_pose_stamped.header.frame_id,
                                       parent_pose_stamped.header.frame_id, parent_pose_stamped.header.stamp, ros::Duration(1.0)))
    {
      tf_listener_.transformPose(child_pose_stamped.header.frame_id, parent_pose_stamped, parent_pose_stamped_transed);
    }
    else
    {
      return false;
    }
    Eigen::Affine3d parent_pose_eigened, child_pose_eigened;
    tf::poseMsgToEigen(parent_pose_stamped_transed.pose, parent_pose_eigened);
    tf::poseMsgToEigen(child_pose_stamped.pose, child_pose_eigened);
    association_list_[child_marker_name] = jsk_interactive_marker::ParentMarkerInformation(parent_topic_name, parent_marker_name, parent_pose_eigened.inverse() * child_pose_eigened);
  }

  bool ParentAndChildInteractiveMarkerServer::removeParentService(jsk_interactive_marker::RemoveParentMarker::Request &req, jsk_interactive_marker::RemoveParentMarker::Response &res)
  {
    std::string parent_topic_name = association_list_[req.child_marker_name].parent_topic_name;
    parent_subscriber_nums_[parent_topic_name] -= 1;
    if (parent_subscriber_nums_[parent_topic_name] == 0)
    {
      parent_feedback_subscribers_[parent_topic_name].shutdown();
      parent_update_subscribers_[parent_topic_name].shutdown();
    }
    setCallback(req.child_marker_name, NULL, 200);
  }

  geometry_msgs::Pose ParentAndChildInteractiveMarkerServer::getRelativePose(geometry_msgs::PoseStamped parent_pose_stamped, geometry_msgs::PoseStamped child_pose_stamped)
  {
  }

  bool ParentAndChildInteractiveMarkerServer::getMarkerPoseService(jsk_interactive_marker::GetTransformableMarkerPose::Request &req, jsk_interactive_marker::GetTransformableMarkerPose::Response &res)
  {
    geometry_msgs::PoseStamped pose_stamped;
    if (getMarkerPose(req.target_name, pose_stamped))
      {
        res.pose_stamped = pose_stamped;
      }
    else
      return false;
  }
  bool ParentAndChildInteractiveMarkerServer::getMarkerPose(std::string target_name, geometry_msgs::PoseStamped &pose_stamped)
  {
    visualization_msgs::InteractiveMarker focus_marker;
    if (get(target_name, focus_marker))
      {
        pose_stamped.pose = focus_marker.pose;
        pose_stamped.header = focus_marker.header;
        return true;
      }
    return false;
  }
  void ParentAndChildInteractiveMarkerServer::parentUpdateCb(const visualization_msgs::InteractiveMarkerUpdateConstPtr &update, std::string parent_topic_name)
  {
    // keep relative pose
    bool need_apply_change = false;
    std::map <std::string, ParentMarkerInformation>::iterator assoc_it = association_list_.begin();
    while (assoc_it != association_list_.end()){
      for (size_t i=0; i<update->markers.size(); i++){
        if (assoc_it -> second.parent_topic_name == parent_topic_name && assoc_it -> second.parent_marker_name == update -> markers[i].name){
          renewPoseWithParent(assoc_it, update -> markers[i].pose, update -> markers[i].header);
        }
      }
      ++assoc_it;
    }
    // apply change
    if (need_apply_change)
    {
      applyChanges();
    }
  }
  void ParentAndChildInteractiveMarkerServer::parentFeedbackCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string parent_topic_name)
  {
    // keep relative pose
    bool need_apply_change = false;
    std::map <std::string, ParentMarkerInformation>::iterator assoc_it = association_list_.begin();
    while (assoc_it != association_list_.end()){
      if (assoc_it -> second.parent_topic_name == parent_topic_name && assoc_it -> second.parent_marker_name == feedback -> marker_name){
        renewPoseWithParent(assoc_it, feedback -> pose, feedback -> header);
        need_apply_change = true;
      }
      ++assoc_it;
    }
    // apply change
    if (need_apply_change)
      applyChanges();
    // apply change
  }
  void ParentAndChildInteractiveMarkerServer::renewPoseWithParent(std::map <std::string, ParentMarkerInformation>::iterator assoc_it, geometry_msgs::Pose parent_pose, std_msgs::Header parent_header)
  {
    Eigen::Affine3d parent_pose_eigened, child_new_pose_eigened;
    tf::poseMsgToEigen(parent_pose, parent_pose_eigened);
    child_new_pose_eigened = parent_pose_eigened * assoc_it -> second.relative_pose;
    geometry_msgs::Pose child_new_pose;
    tf::poseEigenToMsg(child_new_pose_eigened, child_new_pose);
    setPose(assoc_it -> first, child_new_pose, parent_header);
  }
  void ParentAndChildInteractiveMarkerServer::selfFeedbackCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) // for updating relative pose with feedback
  {
    // check self parent
    // renew all
    // only when mouse is removed because it takes much cost for service call
    if (feedback -> event_type != visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    {
      ROS_INFO("no change");
      return;
    }
    // for -> get() for reserved objects
    ROS_INFO("change relative");

    geometry_msgs::PoseStamped feedback_pose_stamped;
    feedback_pose_stamped.pose = feedback->pose;
    feedback_pose_stamped.header = feedback->header;

    std::map <std::string, ParentMarkerInformation>::iterator assoc_it = association_list_.begin();
    while (assoc_it != association_list_.end()){
      if (assoc_it->first == feedback->marker_name){
        if (topic_server_name_ == assoc_it->second.parent_topic_name)
          {
            registerAssociationItself(assoc_it->second.parent_marker_name, assoc_it->second.parent_topic_name, assoc_it->first, feedback_pose_stamped);
          }
        else
          {
            registerAssociationWithOtherNode(assoc_it->second.parent_marker_name, assoc_it->second.parent_topic_name, assoc_it->first, feedback_pose_stamped);
          }
      }
      ++assoc_it;
    }
  }
  void ParentAndChildInteractiveMarkerServer::applyChanges() // for updating relative pose with update
  {
    interactive_markers::InteractiveMarkerServer::applyChanges();
    // keep relative pose
    // apply change
    // for -> get() for reserved objects
    // check self parent change
    // renew all
  }
  bool ParentAndChildInteractiveMarkerServer::setCallback(const std::string &name, FeedbackCallback feedback_cb, uint8_t feedback_type)
  {
    if (feedback_type != visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    {
        return interactive_markers::InteractiveMarkerServer::setCallback(name, feedback_cb, feedback_type);
    }
    else
    {
      //[&](const FeedbackConstPtr& feedback_msg) -> void {feedback_cb(feedback_msg); return;};
      //std::function<int(int, int)> f2 = [](int x, int y) { return x + y; };
      // FeedbackCallback new_fb = feedback_cb;
      return true;
    }
  }
}
