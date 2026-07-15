#include "opencv_apps/node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace opencv_apps
{
rcl_interfaces::msg::ParameterDescriptor describeParam(const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  return d;
}

rcl_interfaces::msg::ParameterDescriptor describeIntParam(
    const std::string & description, int lo, int hi)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  rcl_interfaces::msg::IntegerRange r;
  r.from_value = lo;
  r.to_value = hi;
  r.step = 1;
  d.integer_range.push_back(r);
  return d;
}

rcl_interfaces::msg::ParameterDescriptor describeDoubleParam(
    const std::string & description, double lo, double hi)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  rcl_interfaces::msg::FloatingPointRange r;
  r.from_value = lo;
  r.to_value = hi;
  r.step = 0.0;
  d.floating_point_range.push_back(r);
  return d;
}

Node::Node(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
  , always_subscribe_(this->declare_parameter("always_subscribe", false))
  , verbose_connection_(this->declare_parameter("verbose_connection", false))
  , ever_subscribed_(false)
  , connection_status_(ConnectionStatus::NOT_INITIALIZED)
{
}

void Node::onInitPostProcess()
{
  connection_status_ = ConnectionStatus::NOT_SUBSCRIBED;
  if (always_subscribe_) {
    subscribe();
    connection_status_ = ConnectionStatus::SUBSCRIBED;
    ever_subscribed_ = true;
  } else {
    connection_timer_ = this->create_wall_timer(500ms, std::bind(&Node::checkConnections, this));
  }
  never_subscribed_timer_ =
      this->create_wall_timer(5s, std::bind(&Node::warnNeverSubscribedCallback, this));
}

void Node::checkConnections()
{
  bool any_subscribed = false;
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    for (const auto & getter : subscription_count_getters_) {
      if (getter() > 0) {
        any_subscribed = true;
        break;
      }
    }
  }

  if (any_subscribed) {
    ever_subscribed_ = true;
    if (connection_status_ != ConnectionStatus::SUBSCRIBED) {
      if (verbose_connection_) {
        RCLCPP_INFO(this->get_logger(), "Subscribing to input topics");
      }
      subscribe();
      connection_status_ = ConnectionStatus::SUBSCRIBED;
    }
  } else {
    if (connection_status_ == ConnectionStatus::SUBSCRIBED) {
      if (verbose_connection_) {
        RCLCPP_INFO(this->get_logger(), "Unsubscribing from input topics");
      }
      unsubscribe();
      connection_status_ = ConnectionStatus::NOT_SUBSCRIBED;
    }
  }
}

void Node::warnNeverSubscribedCallback()
{
  if (!ever_subscribed_) {
    RCLCPP_WARN(
        this->get_logger(),
        "Never had any subscribers on the watched output topics of this node. "
        "Did you forget to remap topics or connect a subscriber?");
  }
  never_subscribed_timer_->cancel();
}

void Node::deferApplyParams(const std::function<void()> & apply_fn)
{
  defer_apply_params_timer_ = this->create_wall_timer(0s, [this, apply_fn]() {
    defer_apply_params_timer_->cancel();
    apply_fn();
  });
}

image_transport::Publisher Node::advertiseImage(const std::string & topic, int queue_size)
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = static_cast<size_t>(queue_size);
  image_transport::Publisher pub = image_transport::create_publisher(this, topic, qos);
  std::lock_guard<std::mutex> lock(connection_mutex_);
  subscription_count_getters_.push_back([pub]() { return pub.getNumSubscribers(); });
  return pub;
}

image_transport::CameraPublisher Node::advertiseCamera(const std::string & topic, int queue_size)
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = static_cast<size_t>(queue_size);
  image_transport::CameraPublisher pub = image_transport::create_camera_publisher(this, topic, qos);
  std::lock_guard<std::mutex> lock(connection_mutex_);
  subscription_count_getters_.push_back([pub]() { return pub.getNumSubscribers(); });
  return pub;
}
}  // namespace opencv_apps
