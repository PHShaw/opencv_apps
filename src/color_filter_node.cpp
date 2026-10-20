// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab.
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

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv_apps/node.hpp"

namespace opencv_apps
{
// Shared base for the four color-space filters below (RGB/HLS/HSV/Lab): each
// thresholds the input image against a lower/upper cv::Scalar range computed
// from its own limit parameters, producing a binary mask. Was one templated
// class (ColorFilterNodelet<Config>) in ROS 1; the Config template parameter
// is gone since there's no dynamic_reconfigure code generation in ROS 2, but
// the shared subscribe/publish/doWork logic stays factored out here.
class ColorFilterNode : public opencv_apps::Node
{
protected:
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  int queue_size_;
  bool debug_view_;
  bool use_camera_info_;

  std::string window_name_;

  cv::Scalar lower_color_range_;
  cv::Scalar upper_color_range_;

  // Names of the derived class's limit parameters, shown as debug-view sliders
  std::vector<std::string> limit_params_;

  virtual void applyLimits() = 0;
  virtual void updateCondition() = 0;
  virtual void filter(const cv::Mat & input_image, cv::Mat & output_image) = 0;

  void imageCallbackWithInfo(
      const sensor_msgs::msg::Image::ConstSharedPtr & msg,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
  {
    doWork(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    doWork(msg, msg->header.frame_id);
  }

  void doWork(
      const sensor_msgs::msg::Image::ConstSharedPtr & image_msg, const std::string & /*input_frame_from_msg*/)
  {
    try
    {
      cv::Mat frame = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image;

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        // Slide the parameters' own values: updateCondition() may have swapped the
        // cached min/max limits
        bool changed = false;
        for (const auto & name : limit_params_)
        {
          int value = static_cast<int>(this->get_parameter(name).as_int());
          changed |= syncParamTrackbar(window_name_, name, value);
        }
        if (changed)
        {
          applyLimits();
        }
      }

      cv::Mat out_frame;
      filter(frame, out_frame);

      if (debug_view_)
      {
        cv::imshow(window_name_, out_frame);
        cv::waitKey(1);
      }

      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::MONO8, out_frame).toImageMsg();
      img_pub_.publish(*out_img);
    }
    catch (cv::Exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(),
                   e.file.c_str(), e.line);
    }
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Subscribing to image topic.");
    std::string transport = image_transport::TransportHints(this).getTransport();
    if (use_camera_info_)
      cam_sub_ = image_transport::create_camera_subscription(
          this, "image",
          std::bind(&ColorFilterNode::imageCallbackWithInfo, this, std::placeholders::_1, std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&ColorFilterNode::imageCallback, this, std::placeholders::_1), transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

  ColorFilterNode(const std::string & node_name, const rclcpp::NodeOptions & options)
    : opencv_apps::Node(node_name, options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with the filter mask."));
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    window_name_ = "ColorFilter Demo";

    use_camera_info_ = this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
  }

  // Must be called at the end of the derived class's constructor, after it has
  // declared its own limit parameters and applied their initial values.
  void finishInit()
  {
    img_pub_ = advertiseImage("~/image", 1);
    onInitPostProcess();
  }
};

class RGBColorFilterNode : public ColorFilterNode
{
  int r_min_, r_max_, g_min_, g_max_, b_min_, b_max_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyLimits() override
  {
    r_max_ = this->get_parameter("r_limit_max").as_int();
    r_min_ = this->get_parameter("r_limit_min").as_int();
    g_max_ = this->get_parameter("g_limit_max").as_int();
    g_min_ = this->get_parameter("g_limit_min").as_int();
    b_max_ = this->get_parameter("b_limit_max").as_int();
    b_min_ = this->get_parameter("b_limit_min").as_int();
    updateCondition();
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> &)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    deferApplyParams([this] { applyLimits(); });
    return result;
  }

  void updateCondition() override
  {
    if (r_max_ < r_min_)
      std::swap(r_max_, r_min_);
    if (g_max_ < g_min_)
      std::swap(g_max_, g_min_);
    if (b_max_ < b_min_)
      std::swap(b_max_, b_min_);
    lower_color_range_ = cv::Scalar(b_min_, g_min_, r_min_);
    upper_color_range_ = cv::Scalar(b_max_, g_max_, r_max_);
  }

  void filter(const cv::Mat & input_image, cv::Mat & output_image) override
  {
    cv::inRange(input_image, lower_color_range_, upper_color_range_, output_image);
  }

public:
  explicit RGBColorFilterNode(const rclcpp::NodeOptions & options) : ColorFilterNode("rgb_color_filter", options)
  {
    this->declare_parameter("r_limit_max", 255, describeIntParam("The maximum allowed field value Red", 0, 255));
    this->declare_parameter("r_limit_min", 0, describeIntParam("The minimum allowed field value Red", 0, 255));
    this->declare_parameter("g_limit_max", 255, describeIntParam("The maximum allowed field value Green", 0, 255));
    this->declare_parameter("g_limit_min", 0, describeIntParam("The minimum allowed field value Green", 0, 255));
    this->declare_parameter("b_limit_max", 255, describeIntParam("The maximum allowed field value Blue", 0, 255));
    this->declare_parameter("b_limit_min", 0, describeIntParam("The minimum allowed field value Blue", 0, 255));
    limit_params_ = { "r_limit_min", "r_limit_max", "g_limit_min", "g_limit_max", "b_limit_min", "b_limit_max" };
    applyLimits();

    finishInit();
    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&RGBColorFilterNode::onParamChange, this, std::placeholders::_1));
  }
};

class HLSColorFilterNode : public ColorFilterNode
{
  int h_min_, h_max_, s_min_, s_max_, l_min_, l_max_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyLimits() override
  {
    h_max_ = this->get_parameter("h_limit_max").as_int();
    h_min_ = this->get_parameter("h_limit_min").as_int();
    s_max_ = this->get_parameter("s_limit_max").as_int();
    s_min_ = this->get_parameter("s_limit_min").as_int();
    l_max_ = this->get_parameter("l_limit_max").as_int();
    l_min_ = this->get_parameter("l_limit_min").as_int();
    updateCondition();
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> &)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    deferApplyParams([this] { applyLimits(); });
    return result;
  }

  void updateCondition() override
  {
    if (s_max_ < s_min_)
      std::swap(s_max_, s_min_);
    if (l_max_ < l_min_)
      std::swap(l_max_, l_min_);
    lower_color_range_ = cv::Scalar(h_min_ / 2, l_min_, s_min_, 0);
    upper_color_range_ = cv::Scalar(h_max_ / 2, l_max_, s_max_, 0);
  }

  void filter(const cv::Mat & input_image, cv::Mat & output_image) override
  {
    cv::Mat hls_image;
    cv::cvtColor(input_image, hls_image, cv::COLOR_BGR2HLS);
    if (lower_color_range_[0] < upper_color_range_[0])
    {
      cv::inRange(hls_image, lower_color_range_, upper_color_range_, output_image);
    }
    else
    {
      cv::Scalar lower_color_range_0 = cv::Scalar(0, l_min_, s_min_, 0);
      cv::Scalar upper_color_range_0 = cv::Scalar(h_max_ / 2, l_max_, s_max_, 0);
      cv::Scalar lower_color_range_360 = cv::Scalar(h_min_ / 2, l_min_, s_min_, 0);
      cv::Scalar upper_color_range_360 = cv::Scalar(360 / 2, l_max_, s_max_, 0);
      cv::Mat output_image_0, output_image_360;
      cv::inRange(hls_image, lower_color_range_0, upper_color_range_0, output_image_0);
      cv::inRange(hls_image, lower_color_range_360, upper_color_range_360, output_image_360);
      output_image = output_image_0 | output_image_360;
    }
  }

public:
  explicit HLSColorFilterNode(const rclcpp::NodeOptions & options) : ColorFilterNode("hls_color_filter", options)
  {
    this->declare_parameter("h_limit_max", 360, describeIntParam("The maximum allowed field value Hue", 0, 360));
    this->declare_parameter("h_limit_min", 0, describeIntParam("The minimum allowed field value Hue", 0, 360));
    this->declare_parameter(
        "s_limit_max", 256, describeIntParam("The maximum allowed field value Saturation", 0, 256));
    this->declare_parameter(
        "s_limit_min", 0, describeIntParam("The minimum allowed field value Saturation", 0, 256));
    this->declare_parameter(
        "l_limit_max", 256, describeIntParam("The maximum allowed field value Lightness", 0, 256));
    this->declare_parameter(
        "l_limit_min", 0, describeIntParam("The minimum allowed field value Lightness", 0, 256));
    limit_params_ = { "h_limit_min", "h_limit_max", "s_limit_min", "s_limit_max", "l_limit_min", "l_limit_max" };
    applyLimits();

    finishInit();
    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&HLSColorFilterNode::onParamChange, this, std::placeholders::_1));
  }
};

class HSVColorFilterNode : public ColorFilterNode
{
  int h_min_, h_max_, s_min_, s_max_, v_min_, v_max_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyLimits() override
  {
    h_max_ = this->get_parameter("h_limit_max").as_int();
    h_min_ = this->get_parameter("h_limit_min").as_int();
    s_max_ = this->get_parameter("s_limit_max").as_int();
    s_min_ = this->get_parameter("s_limit_min").as_int();
    v_max_ = this->get_parameter("v_limit_max").as_int();
    v_min_ = this->get_parameter("v_limit_min").as_int();
    updateCondition();
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> &)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    deferApplyParams([this] { applyLimits(); });
    return result;
  }

  void updateCondition() override
  {
    if (s_max_ < s_min_)
      std::swap(s_max_, s_min_);
    if (v_max_ < v_min_)
      std::swap(v_max_, v_min_);
    lower_color_range_ = cv::Scalar(h_min_ / 2, s_min_, v_min_, 0);
    upper_color_range_ = cv::Scalar(h_max_ / 2, s_max_, v_max_, 0);
  }

  void filter(const cv::Mat & input_image, cv::Mat & output_image) override
  {
    cv::Mat hsv_image;
    cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);
    if (lower_color_range_[0] < upper_color_range_[0])
    {
      cv::inRange(hsv_image, lower_color_range_, upper_color_range_, output_image);
    }
    else
    {
      cv::Scalar lower_color_range_0 = cv::Scalar(0, s_min_, v_min_, 0);
      cv::Scalar upper_color_range_0 = cv::Scalar(h_max_ / 2, s_max_, v_max_, 0);
      cv::Scalar lower_color_range_360 = cv::Scalar(h_min_ / 2, s_min_, v_min_, 0);
      cv::Scalar upper_color_range_360 = cv::Scalar(360 / 2, s_max_, v_max_, 0);
      cv::Mat output_image_0, output_image_360;
      cv::inRange(hsv_image, lower_color_range_0, upper_color_range_0, output_image_0);
      cv::inRange(hsv_image, lower_color_range_360, upper_color_range_360, output_image_360);
      output_image = output_image_0 | output_image_360;
    }
  }

public:
  explicit HSVColorFilterNode(const rclcpp::NodeOptions & options) : ColorFilterNode("hsv_color_filter", options)
  {
    this->declare_parameter("h_limit_max", 360, describeIntParam("The maximum allowed field value Hue", 0, 360));
    this->declare_parameter("h_limit_min", 0, describeIntParam("The minimum allowed field value Hue", 0, 360));
    this->declare_parameter(
        "s_limit_max", 256, describeIntParam("The maximum allowed field value Saturation", 0, 256));
    this->declare_parameter(
        "s_limit_min", 0, describeIntParam("The minimum allowed field value Saturation", 0, 256));
    this->declare_parameter("v_limit_max", 256, describeIntParam("The maximum allowed field value Value", 0, 256));
    this->declare_parameter("v_limit_min", 0, describeIntParam("The minimum allowed field value Value", 0, 256));
    limit_params_ = { "h_limit_min", "h_limit_max", "s_limit_min", "s_limit_max", "v_limit_min", "v_limit_max" };
    applyLimits();

    finishInit();
    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&HSVColorFilterNode::onParamChange, this, std::placeholders::_1));
  }
};

class LabColorFilterNode : public ColorFilterNode
{
  int l_min_, l_max_, a_min_, a_max_, b_min_, b_max_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyLimits() override
  {
    l_max_ = this->get_parameter("l_limit_max").as_int();
    l_min_ = this->get_parameter("l_limit_min").as_int();
    a_max_ = this->get_parameter("a_limit_max").as_int();
    a_min_ = this->get_parameter("a_limit_min").as_int();
    b_max_ = this->get_parameter("b_limit_max").as_int();
    b_min_ = this->get_parameter("b_limit_min").as_int();
    updateCondition();
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> &)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    deferApplyParams([this] { applyLimits(); });
    return result;
  }

  void updateCondition() override
  {
    if (l_max_ < l_min_)
      std::swap(l_max_, l_min_);
    if (a_max_ < a_min_)
      std::swap(a_max_, a_min_);
    if (b_max_ < b_min_)
      std::swap(b_max_, b_min_);
    lower_color_range_ = cv::Scalar(l_min_, a_min_, b_min_);
    upper_color_range_ = cv::Scalar(l_max_, a_max_, b_max_);
  }

  void filter(const cv::Mat & input_image, cv::Mat & output_image) override
  {
    cv::Mat lab_image;
    cv::cvtColor(input_image, lab_image, cv::COLOR_BGR2Lab);
    cv::inRange(lab_image, lower_color_range_, upper_color_range_, output_image);
  }

public:
  explicit LabColorFilterNode(const rclcpp::NodeOptions & options) : ColorFilterNode("lab_color_filter", options)
  {
    this->declare_parameter("l_limit_max", 255, describeIntParam("The maximum allowed field value L*", 0, 255));
    this->declare_parameter("l_limit_min", 0, describeIntParam("The minimum allowed field value L*", 0, 255));
    this->declare_parameter("a_limit_max", 255, describeIntParam("The maximum allowed field value a*", 0, 255));
    this->declare_parameter("a_limit_min", 0, describeIntParam("The minimum allowed field value a*", 0, 255));
    this->declare_parameter("b_limit_max", 255, describeIntParam("The maximum allowed field value b*", 0, 255));
    this->declare_parameter("b_limit_min", 0, describeIntParam("The minimum allowed field value b*", 0, 255));
    limit_params_ = { "l_limit_min", "l_limit_max", "a_limit_min", "a_limit_max", "b_limit_min", "b_limit_max" };
    applyLimits();

    finishInit();
    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&LabColorFilterNode::onParamChange, this, std::placeholders::_1));
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::RGBColorFilterNode)
RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::HLSColorFilterNode)
RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::HSVColorFilterNode)
RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::LabColorFilterNode)
