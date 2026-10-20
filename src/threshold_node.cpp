// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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
 *   * Neither the name of the JSK Lab nor the names of its
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

// https://github.com/opencv/opencv/blob/2.4/samples/cpp/tutorial_code/ImgProc/Threshold.cpp
/**
 * This is a demo of threshold image processing,
 */

#include <functional>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "opencv_apps/node.hpp"

namespace opencv_apps
{
class ThresholdNode : public opencv_apps::Node
{
  int queue_size_;
  bool debug_view_;

  std::string window_name_;

  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  bool use_camera_info_;
  int threshold_type_;
  int max_binary_value_;
  int threshold_value_;
  bool apply_otsu_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyParams()
  {
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
    threshold_value_ = this->get_parameter("threshold").as_int();
    threshold_type_ = this->get_parameter("threshold_type").as_int();
    max_binary_value_ = this->get_parameter("max_binary").as_int();
    apply_otsu_ = this->get_parameter("apply_otsu").as_bool();
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> &)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    deferApplyParams([this] { applyParams(); });
    return result;
  }

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

  void subscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Subscribing to image topic.");
    std::string transport = image_transport::TransportHints(this).getTransport();
    if (use_camera_info_)
      cam_sub_ = image_transport::create_camera_subscription(
          this, "image",
          std::bind(&ThresholdNode::imageCallbackWithInfo, this, std::placeholders::_1, std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&ThresholdNode::imageCallback, this, std::placeholders::_1), transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg, const std::string & /*input_frame_from_msg*/)
  {
    try
    {
      cv::Mat src_image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image;

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        syncParamTrackbar(window_name_, "threshold_type", threshold_type_);
        syncParamTrackbar(window_name_, "threshold", threshold_value_);
        syncParamTrackbar(window_name_, "max_binary", max_binary_value_);
        syncParamTrackbar(window_name_, "apply_otsu", apply_otsu_);
      }

      cv::Mat gray_image;
      cv::cvtColor(src_image, gray_image, cv::COLOR_BGR2GRAY);
      cv::Mat result_image;

      int threshold_type = threshold_type_;
      if (apply_otsu_)
      {
        threshold_type |= cv::THRESH_OTSU;
      }
      cv::threshold(gray_image, result_image, threshold_value_, max_binary_value_, threshold_type);

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::imshow(window_name_, result_image);
        cv::waitKey(1);
      }
      img_pub_.publish(
          *cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::MONO8, result_image).toImageMsg());
    }
    catch (cv::Exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(),
                   e.file.c_str(), e.line);
    }
  }

public:
  explicit ThresholdNode(const rclcpp::NodeOptions & options) : opencv_apps::Node("threshold", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with the thresholded image."));
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    window_name_ = "Threshold Demo";

    this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    this->declare_parameter("apply_otsu", false, describeParam("Apply otsu threshold"));
    this->declare_parameter(
        "threshold_type", 0,
        describeIntParam(
            "Threshold Type: 0=Binary, 1=Binary_Inverted, 2=Threshold_Truncated, 3=Threshold_To_Zero, "
            "4=Threshold_To_Zero_Inverted",
            0, 4));
    this->declare_parameter("threshold", 150, describeIntParam("Threshold value", 0, 255));
    this->declare_parameter("max_binary", 255, describeIntParam("Max Binary value", 0, 255));
    applyParams();

    img_pub_ = advertiseImage("~/image", 1);

    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&ThresholdNode::onParamChange, this, std::placeholders::_1));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::ThresholdNode)
