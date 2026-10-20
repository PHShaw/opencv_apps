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

// https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/ImgProc/Pyramids/Pyramids.cpp
/**
 * @file Pyramids.cpp
 * @brief Sample code of image pyramids (pyrDown and pyrUp)
 * @author OpenCV team
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
namespace
{
enum PyramidsType
{
  PYRAMIDS_UP = 0,
  PYRAMIDS_DOWN = 1,
};
}  // namespace

class PyramidsNode : public opencv_apps::Node
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  int queue_size_;
  bool debug_view_;
  bool use_camera_info_;

  int pyramids_type_;
  int num_of_pyramids_;

  std::string window_name_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyParams()
  {
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
    pyramids_type_ = this->get_parameter("pyramids_type").as_int();
    num_of_pyramids_ = this->get_parameter("num_of_pyramids").as_int();
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

  void doWork(
      const sensor_msgs::msg::Image::ConstSharedPtr & image_msg, const std::string & /*input_frame_from_msg*/)
  {
    try
    {
      // Keep the input encoding, except packed YUV and Bayer images, whose pixels
      // can't be resized directly: convert those to bgr8
      std::string encoding = image_msg->encoding;
      if (sensor_msgs::image_encodings::isBayer(encoding) || sensor_msgs::image_encodings::numChannels(encoding) == 2)
      {
        encoding = sensor_msgs::image_encodings::BGR8;
      }
      cv::Mat src_image = cv_bridge::toCvShare(image_msg, encoding)->image;

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        syncParamTrackbar(window_name_, "pyramids_type", pyramids_type_);
        syncParamTrackbar(window_name_, "num_of_pyramids", num_of_pyramids_);
      }

      int num = num_of_pyramids_;
      switch (pyramids_type_)
      {
        case PYRAMIDS_UP:
          while (num)
          {
            num--;
            cv::pyrUp(src_image, src_image, cv::Size(src_image.cols * 2, src_image.rows * 2));
          }
          break;
        case PYRAMIDS_DOWN:
          while (num)
          {
            num--;
            cv::pyrDown(src_image, src_image, cv::Size(src_image.cols / 2, src_image.rows / 2));
          }
          break;
      }

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::imshow(window_name_, src_image);
        cv::waitKey(1);
      }

      img_pub_.publish(*cv_bridge::CvImage(image_msg->header, encoding, src_image).toImageMsg());
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
          std::bind(&PyramidsNode::imageCallbackWithInfo, this, std::placeholders::_1, std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&PyramidsNode::imageCallback, this, std::placeholders::_1), transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit PyramidsNode(const rclcpp::NodeOptions & options) : opencv_apps::Node("pyramids", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with the resulting image."));
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    window_name_ = "Image Pyramids Demo";

    this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    this->declare_parameter("pyramids_type", static_cast<int>(PYRAMIDS_UP), describeIntParam("Pyramids Type: 0=Up (zoom in), 1=Down (zoom out)", 0, 1));
    this->declare_parameter("num_of_pyramids", 1, describeIntParam("Number of pyramids", 1, 10));
    applyParams();

    img_pub_ = advertiseImage("~/image", 1);

    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&PyramidsNode::onParamChange, this, std::placeholders::_1));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::PyramidsNode)
