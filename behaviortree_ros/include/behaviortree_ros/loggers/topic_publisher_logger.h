#ifndef BT_TOPIC_PUBLISHER_LOGGER_H
#define BT_TOPIC_PUBLISHER_LOGGER_H

#include <memory>

#include <behaviortree_cpp_v3/loggers/abstract_logger.h>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

#include <behaviortree_ros_msgs/StatusChange.h>

namespace behaviortree_ros
{
class TopicPublisherLogger : public BT::StatusChangeLogger
{
  static std::atomic<bool> ref_count;

public:
  TopicPublisherLogger(BT::TreeNode* root_node, ros::NodeHandle& nh, std::string topic_name);

  ~TopicPublisherLogger() override;

  virtual void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                        BT::NodeStatus status) override;

  virtual void flush() override;

private:
  behaviortree_ros_msgs::NodeStatus convertBtStatusToRosStatus(BT::NodeStatus status);
  void recursiveRegisterNodePublisher(const BT::TreeNode* node);

  ros::NodeHandle nh_;
  std::string topic_name_;

  std::map<std::string, std::shared_ptr<realtime_tools::RealtimePublisher<behaviortree_ros_msgs::StatusChange>>>
      map_realtime_pub_;
};

}  // namespace behaviortree_ros

#endif  // BT_TOPIC_PUBLISHER_LOGGER_H
