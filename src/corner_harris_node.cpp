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

// https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/TrackingMotion/cornerHarris_Demo.cpp
/**
 * @function cornerHarris_Demo.cpp
 * @brief Demo code for detecting corners using Harris-Stephens method
 * @author OpenCV team
 */

#include <functional>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv_apps/node.hpp"

namespace opencv_apps
{
class CornerHarrisNode : public opencv_apps::Node
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  int queue_size_;
  bool debug_view_;

  std::string window_name_;
  bool need_config_update_;

  bool use_camera_info_;
  int threshold_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyParams()
  {
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
    threshold_ = this->get_parameter("threshold").as_int();
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> & /*params*/)
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

  static void trackbarCallback(int /*unused*/, void * userdata)
  {
    static_cast<CornerHarrisNode *>(userdata)->need_config_update_ = true;
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg, const std::string & /*input_frame_from_msg*/)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image;

      // Do the work
      cv::Mat dst, dst_norm, dst_norm_scaled;
      dst = cv::Mat::zeros(frame.size(), CV_32FC1);

      cv::Mat src_gray;

      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, src_gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        src_gray = frame;
        cv::cvtColor(src_gray, frame, cv::COLOR_GRAY2BGR);
      }

      /// Detector parameters
      int block_size = 2;
      int aperture_size = 3;
      double k = 0.04;

      /// Detecting corners
      cv::cornerHarris(src_gray, dst, block_size, aperture_size, k, cv::BORDER_DEFAULT);

      /// Normalizing
      cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
      cv::convertScaleAbs(dst_norm, dst_norm_scaled);

      /// Drawing a circle around corners
      for (int j = 0; j < dst_norm.rows; j++)
      {
        for (int i = 0; i < dst_norm.cols; i++)
        {
          if ((int)dst_norm.at<float>(j, i) > threshold_)
          {
            cv::circle(frame, cv::Point(i, j), 5, cv::Scalar(0), 2, 8, 0);
          }
        }
      }

      /// Create window
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        const int max_threshold = 255;
        if (need_config_update_)
        {
          this->set_parameter(rclcpp::Parameter("threshold", threshold_));
          need_config_update_ = false;
        }
        cv::createTrackbar("Threshold:", window_name_, &threshold_, max_threshold, &CornerHarrisNode::trackbarCallback,
                            this);
      }

      if (debug_view_)
      {
        cv::imshow(window_name_, frame);
        cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
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
          std::bind(&CornerHarrisNode::imageCallbackWithInfo, this, std::placeholders::_1, std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&CornerHarrisNode::imageCallback, this, std::placeholders::_1), transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit CornerHarrisNode(const rclcpp::NodeOptions & options) : opencv_apps::Node("corner_harris", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with the detected corners."));
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    need_config_update_ = false;

    window_name_ = "CornerHarris Demo";

    this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    this->declare_parameter("threshold", 200, describeIntParam("Threshold of corner.", 0, 255));
    applyParams();

    // Output uses "~/image" (node-private) rather than "image", the input topic
    // name -- otherwise a single `-r image:=X` remap would send both the input
    // subscription and the output publisher to the same topic X.
    img_pub_ = advertiseImage("~/image", 1);

    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&CornerHarrisNode::onParamChange, this, std::placeholders::_1));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::CornerHarrisNode)
