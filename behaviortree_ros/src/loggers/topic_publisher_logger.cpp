#include "behaviortree_ros/loggers/topic_publisher_logger.h"

namespace behaviortree_ros
{
std::atomic<bool> TopicPublisherLogger::ref_count(false);

TopicPublisherLogger::TopicPublisherLogger(BT::TreeNode* root_node, ros::NodeHandle& nh, std::string topic_name)
  : BT::StatusChangeLogger(root_node), topic_name_(topic_name), nh_(nh)
{
  bool expected = false;
  if (!ref_count.compare_exchange_strong(expected, true))
  {
    throw std::logic_error("Only a single instance of TopicPublisherLogger shall be created");
  }

  recursiveRegisterNodePublisher(root_node);
}

TopicPublisherLogger::~TopicPublisherLogger()
{
  ref_count.store(false);
}

void TopicPublisherLogger::callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                                    BT::NodeStatus status)
{
  auto realtime_pub = map_realtime_pub_[node.name()];

  if (realtime_pub->trylock())
  {
    realtime_pub->msg_.uid = node.UID();
    realtime_pub->msg_.prev_status = convertBtStatusToRosStatus(prev_status);
    realtime_pub->msg_.status = convertBtStatusToRosStatus(status);
    realtime_pub->msg_.progress = node.progress();

    //
    realtime_pub->msg_.duration.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(timestamp).count());
    realtime_pub->unlockAndPublish();
  }
}

void TopicPublisherLogger::flush()
{
}

behaviortree_ros_msgs::NodeStatus TopicPublisherLogger::convertBtStatusToRosStatus(BT::NodeStatus status)
{
  behaviortree_ros_msgs::NodeStatus converted_status;
  switch (status)
  {
    case BT::NodeStatus::IDLE:
      converted_status.value = behaviortree_ros_msgs::NodeStatus::IDLE;
      break;
    case BT::NodeStatus::RUNNING:
      converted_status.value = behaviortree_ros_msgs::NodeStatus::RUNNING;
      break;
    case BT::NodeStatus::SUCCESS:
      converted_status.value = behaviortree_ros_msgs::NodeStatus::SUCCESS;
      break;
    case BT::NodeStatus::FAILURE:
      converted_status.value = behaviortree_ros_msgs::NodeStatus::FAILURE;
      break;
  }
  return converted_status;
}

void TopicPublisherLogger::recursiveRegisterNodePublisher(const BT::TreeNode* node)
{
  map_realtime_pub_[node->name()] =
      std::make_shared<realtime_tools::RealtimePublisher<behaviortree_ros_msgs::StatusChange>>(nh_, node->name(), 1,
                                                                                               true);

  auto realtime_pub = map_realtime_pub_[node->name()];
  if (realtime_pub->trylock())
  {
    realtime_pub->msg_.uid = node->UID();
    realtime_pub->msg_.prev_status.value = behaviortree_ros_msgs::NodeStatus::IDLE;
    realtime_pub->msg_.status = convertBtStatusToRosStatus(node->status());
    realtime_pub->msg_.progress = node->progress();
    // realtime_pub->msg_.duration = ros::Duration(0.0);
    realtime_pub->unlockAndPublish();
  }

  if (auto control = dynamic_cast<const BT::ControlNode*>(node))
  {
    for (const auto& child : control->children())
    {
      recursiveRegisterNodePublisher(child);
    }
  }
  else if (auto decorator = dynamic_cast<const BT::DecoratorNode*>(node))
  {
    recursiveRegisterNodePublisher(decorator->child());
  }
}

}  // namespace behaviortree_ros
