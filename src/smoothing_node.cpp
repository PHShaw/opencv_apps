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

// https://github.com/opencv/opencv/blob/2.4/samples/cpp/tutorial_code/ImgProc/Smoothing.cpp
/**
 * file Smoothing.cpp
 * brief Sample code for simple filters
 * author OpenCV team
 */

#include <functional>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "opencv_apps/node.hpp"

namespace opencv_apps
{
namespace
{
const int kMaxKernelLength = 31;
enum FilterType
{
  FILTER_HOMOGENEOUS_BLUR = 0,
  FILTER_GAUSSIAN_BLUR = 1,
  FILTER_MEDIAN_BLUR = 2,
  FILTER_BILATERAL_FILTER = 3,
};
}  // namespace

class SmoothingNode : public opencv_apps::Node
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  int queue_size_;
  bool debug_view_;
  bool use_camera_info_;
  rclcpp::Time prev_stamp_;

  std::string window_name_;
  int filter_type_;
  int kernel_size_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyParams()
  {
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
    filter_type_ = this->get_parameter("filter_type").as_int();
    kernel_size_ = (this->get_parameter("kernel_size").as_int() / 2) * 2 + 1;
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

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const std::string & /*input_frame_from_msg*/)
  {
    try
    {
      cv::Mat in_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      if (debug_view_)
      {
        char kernel_label[] = "Kernel Size : ";
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        syncParamTrackbar(window_name_, "filter_type", filter_type_);
        if (syncTrackbar(kernel_label, window_name_, kernel_size_, kMaxKernelLength, 1))
        {
          kernel_size_ = (kernel_size_ / 2) * 2 + 1;
          this->set_parameter(rclcpp::Parameter("kernel_size", kernel_size_));
        }
      }

      cv::Mat out_image = in_image.clone();
      int i = kernel_size_;
      switch (filter_type_)
      {
        case FILTER_HOMOGENEOUS_BLUR:
          cv::blur(in_image, out_image, cv::Size(i, i), cv::Point(-1, -1));
          break;
        case FILTER_GAUSSIAN_BLUR:
          cv::GaussianBlur(in_image, out_image, cv::Size(i, i), 0, 0);
          break;
        case FILTER_MEDIAN_BLUR:
          cv::medianBlur(in_image, out_image, i);
          break;
        case FILTER_BILATERAL_FILTER:
          cv::bilateralFilter(in_image, out_image, i, i * 2, i / 2);
          break;
      }

      if (debug_view_)
      {
        cv::imshow(window_name_, out_image);
        cv::waitKey(1);
      }

      img_pub_.publish(*cv_bridge::CvImage(msg->header, "bgr8", out_image).toImageMsg());
    }
    catch (cv::Exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(),
                   e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Subscribing to image topic.");
    std::string transport = image_transport::TransportHints(this).getTransport();
    if (use_camera_info_)
      cam_sub_ = image_transport::create_camera_subscription(
          this, "image",
          std::bind(&SmoothingNode::imageCallbackWithInfo, this, std::placeholders::_1, std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&SmoothingNode::imageCallback, this, std::placeholders::_1), transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit SmoothingNode(const rclcpp::NodeOptions & options) : opencv_apps::Node("smoothing", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with the smoothed image."));
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0);
    window_name_ = "Smoothing Demo";

    this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    this->declare_parameter(
        "filter_type", static_cast<int>(FILTER_GAUSSIAN_BLUR),
        describeIntParam(
            "Smoothing filter method: 0=Homogeneous_Blur, 1=Gaussian_Blur, 2=Median_Blur, 3=Bilateral_Filter", 0, 3));
    this->declare_parameter(
        "kernel_size", 7,
        describeIntParam("Size of the kernel (only one because we use a square window). Must be odd.", 1, 31));
    applyParams();

    img_pub_ = advertiseImage("~/image", 1);

    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&SmoothingNode::onParamChange, this, std::placeholders::_1));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::SmoothingNode)
