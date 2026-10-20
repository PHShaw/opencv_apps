#include "opencv_apps/node.hpp"

#include <chrono>
#include <cmath>
#include <utility>

#include <opencv2/highgui.hpp>

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

bool Node::syncTrackbar(const std::string & name, const std::string & window, int & value, int max_value,
                        int min_value)
{
  const auto key = std::make_pair(window, name);
  int pos = cv::getTrackbarPos(name, window);
  if (pos < 0)
  {
    // Not created yet (or the window was re-created): start the slider at the current value
    cv::createTrackbar(name, window, nullptr, max_value);
    if (min_value != 0)
    {
      cv::setTrackbarMin(name, window, min_value);
    }
    cv::setTrackbarPos(name, window, value);
    trackbar_positions_[key] = value;
    return false;
  }

  auto last = trackbar_positions_.find(key);
  if (last != trackbar_positions_.end() && pos != last->second)
  {
    // Moved by the user
    last->second = pos;
    value = pos;
    return true;
  }

  if (pos != value)
  {
    // Changed elsewhere, e.g. by a parameter update
    cv::setTrackbarPos(name, window, value);
  }
  trackbar_positions_[key] = value;
  return false;
}

bool Node::syncParamTrackbar(const std::string & window, const std::string & param, int & value)
{
  int min_value = 0;
  int max_value = 255;
  const auto descriptor = this->describe_parameter(param);
  if (!descriptor.integer_range.empty())
  {
    min_value = static_cast<int>(descriptor.integer_range[0].from_value);
    max_value = static_cast<int>(descriptor.integer_range[0].to_value);
  }
  if (!syncTrackbar(param, window, value, max_value, min_value))
  {
    return false;
  }
  this->set_parameter(rclcpp::Parameter(param, value));
  return true;
}

bool Node::syncParamTrackbar(const std::string & window, const std::string & param, double & value, double step)
{
  double min_value = 0.0;
  double max_value = 255.0;
  const auto descriptor = this->describe_parameter(param);
  if (!descriptor.floating_point_range.empty())
  {
    min_value = descriptor.floating_point_range[0].from_value;
    max_value = descriptor.floating_point_range[0].to_value;
  }

  // The slider works in whole steps, so show the scale factor in its label
  std::string label = param;
  const long scale = std::lround(1.0 / step);
  if (scale > 1)
  {
    label += " x" + std::to_string(scale);
  }

  int pos = static_cast<int>(std::lround(value / step));
  const int min_pos = static_cast<int>(std::ceil(min_value / step - 1e-9));
  const int max_pos = static_cast<int>(std::floor(max_value / step + 1e-9));
  if (!syncTrackbar(label, window, pos, max_pos, min_pos))
  {
    return false;
  }
  value = pos * step;
  this->set_parameter(rclcpp::Parameter(param, value));
  return true;
}

bool Node::syncParamTrackbar(const std::string & window, const std::string & param, bool & value)
{
  int pos = value ? 1 : 0;
  if (!syncTrackbar(param, window, pos, 1))
  {
    return false;
  }
  value = (pos != 0);
  this->set_parameter(rclcpp::Parameter(param, value));
  return true;
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
