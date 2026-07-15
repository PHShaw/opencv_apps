/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Ryohei Ueda.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Kei Okada nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef OPENCV_APPS_NODE_HPP_
#define OPENCV_APPS_NODE_HPP_

#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

namespace opencv_apps
{
enum class ConnectionStatus
{
  NOT_INITIALIZED,
  NOT_SUBSCRIBED,
  SUBSCRIBED
};

/** @brief
 * Parameter descriptor helpers used when declaring the parameters that
 * replace each node's old dynamic_reconfigure cfg (1:1 translation of the
 * cfg's gen.add(name, type, level, description, default, min, max) calls).
 */
rcl_interfaces::msg::ParameterDescriptor describeParam(const std::string & description);
rcl_interfaces::msg::ParameterDescriptor describeIntParam(
    const std::string & description, int lo, int hi);
rcl_interfaces::msg::ParameterDescriptor describeDoubleParam(
    const std::string & description, double lo, double hi);

/** @brief
 * Base class for opencv_apps ROS 2 component nodes: automatically calls
 * subscribe()/unsubscribe() as advertised topics gain/lose subscribers, so
 * that no input topic is subscribed to unless something is listening to our
 * output.
 *
 * ROS 1's nodelet::Nodelet-derived equivalent used per-publisher
 * connect/disconnect callbacks (ros::SubscriberStatusCallback,
 * image_transport::SubscriberStatusCallback). As of Humble,
 * image_transport's ROS 2 publishers do not implement subscriber
 * connect/disconnect callbacks (only getNumSubscribers() polling is
 * available), so this base class polls all watched publishers on a wall
 * timer instead. This is a deliberate, documented behavior change from the
 * event-driven ROS 1 implementation, not an oversight.
 *
 * Subclasses must implement subscribe()/unsubscribe(), advertise their
 * outputs via advertise<T>()/advertiseImage()/advertiseCamera() (instead of
 * create_publisher()/image_transport::create_publisher() directly) so this
 * class can watch them, and call onInitPostProcess() at the end of their
 * constructor.
 *
 * Output topic names should be advertised as "~/<name>" (node-private),
 * matching ROS 1's pnh_->advertise("<name>", ...). Input topics are
 * subscribed to under the plain relative name "<name>" (matching ROS 1's
 * nh_->subscribe()). Advertising outputs under the plain name instead would
 * make them share the same name as same-named inputs, so a single
 * `-r <name>:=X` remap would incorrectly send both to X.
 */
class Node : public rclcpp::Node
{
public:
  Node(const std::string & node_name, const rclcpp::NodeOptions & options);

protected:
  /** @brief
   * Post-construction hook. Must be called at the end of the derived
   * class's constructor: starts the lazy-subscribe watch timer, or
   * subscribes immediately if `always_subscribe` is true.
   */
  void onInitPostProcess();

  /** @brief
   * Called when at least one watched publisher gains its first subscriber.
   * Set up subscribers in this method.
   */
  virtual void subscribe() = 0;

  /** @brief
   * Called once every watched publisher has lost all of its subscribers.
   * Shut down subscribers in this method.
   */
  virtual void unsubscribe() = 0;

  /** @brief
   * Advertise a topic and watch its subscriber count.
   */
  template <class MessageT>
  typename rclcpp::Publisher<MessageT>::SharedPtr advertise(const std::string & topic, int queue_size)
  {
    auto pub = this->create_publisher<MessageT>(topic, queue_size);
    std::lock_guard<std::mutex> lock(connection_mutex_);
    subscription_count_getters_.push_back([pub]() { return pub->get_subscription_count(); });
    return pub;
  }

  /** @brief
   * Advertise an image topic and watch its subscriber count.
   */
  image_transport::Publisher advertiseImage(const std::string & topic, int queue_size = 1);

  /** @brief
   * Advertise an image + camera_info topic pair and watch their subscriber
   * counts.
   */
  image_transport::CameraPublisher advertiseCamera(const std::string & topic, int queue_size = 1);

  /** @brief
   * A flag to disable the lazy-subscribe mechanism and always subscribe
   * input topics. Set via the `always_subscribe` parameter.
   */
  bool always_subscribe_;

  /** @brief
   * true if the `verbose_connection` parameter is true.
   */
  bool verbose_connection_;

  /** @brief
   * Call this from an add_on_set_parameters_callback instead of re-reading
   * changed values with get_parameter() synchronously.
   *
   * The callback registered with add_on_set_parameters_callback runs
   * *before* the new values are committed to parameter storage -- inside it,
   * get_parameter() still returns the previous value. A node whose callback
   * calls get_parameter() to refresh its cached config (the pattern used
   * throughout this package) therefore always applies the previous update,
   * one update behind the parameter that was just set. Deferring apply_fn to
   * a one-shot wall timer runs it on the next executor iteration, after the
   * new values have been committed, so it observes the value that was just
   * requested.
   */
  void deferApplyParams(const std::function<void()> & apply_fn);

private:
  void checkConnections();
  void warnNeverSubscribedCallback();

  std::mutex connection_mutex_;
  std::vector<std::function<size_t()>> subscription_count_getters_;
  rclcpp::TimerBase::SharedPtr connection_timer_;
  rclcpp::TimerBase::SharedPtr never_subscribed_timer_;
  rclcpp::TimerBase::SharedPtr defer_apply_params_timer_;
  bool ever_subscribed_;
  ConnectionStatus connection_status_;
};
}  // namespace opencv_apps

#endif  // OPENCV_APPS_NODE_HPP_
