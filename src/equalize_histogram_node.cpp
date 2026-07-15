/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Ivan Tarifa.
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
*   * Neither the name of the Ivan Tarifa nor the names of its
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

// https://github.com/opencv/opencv/blob/2.4/samples/cpp/tutorial_code/Histograms_Matching/EqualizeHist_Demo.cpp
/**
 * @function EqualizeHist_Demo.cpp
 * @brief Demo code for equalizeHist function
 * @author OpenCV team
 */

#include <functional>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "opencv2/core/ocl.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv_apps/node.hpp"

namespace opencv_apps
{
namespace
{
enum HistogramEqualizationType
{
  EQUALIZE_HIST = 0,
  CLAHE = 1,
};
}  // namespace

class EqualizeHistogramNode : public opencv_apps::Node
{
  std::string window_name_;
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  cv::Ptr<cv::CLAHE> clahe_;

  int queue_size_;
  bool debug_view_;
  bool use_camera_info_;
  bool use_opencl_;

  int histogram_equalization_type_;
  cv::Size clahe_tile_size_;
  double clahe_clip_limit_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyParams()
  {
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
    histogram_equalization_type_ = this->get_parameter("histogram_equalization_type").as_int();
    clahe_clip_limit_ = this->get_parameter("clahe_clip_limit").as_double();
    clahe_tile_size_ = cv::Size(
        this->get_parameter("clahe_tile_size_x").as_int(), this->get_parameter("clahe_tile_size_y").as_int());
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

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const std::string & input_frame_from_msg)
  {
    try
    {
      cv::UMat frame;
      if (msg->encoding == sensor_msgs::image_encodings::BGR8)
        frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.getUMat(cv::ACCESS_RW);
      else if (msg->encoding == sensor_msgs::image_encodings::MONO8)
        frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image.getUMat(cv::ACCESS_RW);

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      }

      cv::UMat gray, dst;
      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        frame.copyTo(gray);
      }

      switch (histogram_equalization_type_)
      {
        case CLAHE:
          if (clahe_ == nullptr)
            clahe_ = cv::createCLAHE();
          clahe_->setTilesGridSize(clahe_tile_size_);
          clahe_->setClipLimit(clahe_clip_limit_);
          clahe_->apply(gray, dst);
          break;
        case EQUALIZE_HIST:
          equalizeHist(gray, dst);
          break;
      }

      if (debug_view_)
      {
        cv::imshow(window_name_, dst);
        cv::waitKey(1);
      }

      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, dst.getMat(cv::ACCESS_READ))
              .toImageMsg();
      out_img->header.frame_id = input_frame_from_msg;
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
          std::bind(&EqualizeHistogramNode::imageCallbackWithInfo, this, std::placeholders::_1,
                    std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&EqualizeHistogramNode::imageCallback, this, std::placeholders::_1), transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit EqualizeHistogramNode(const rclcpp::NodeOptions & options)
    : opencv_apps::Node("equalize_histogram", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with the equalized image."));
    use_opencl_ = this->declare_parameter("use_opencl", true, describeParam("Enable OpenCV's OpenCL (T-API) path."));
    window_name_ = "Equalize Histogram Window (" + std::string(this->get_name()) + ")";

    use_camera_info_ = this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    this->declare_parameter(
        "histogram_equalization_type", static_cast<int>(EQUALIZE_HIST),
        describeIntParam("Histogram equalization method: 0=EqualizeHist, 1=Clahe", 0, 1));
    this->declare_parameter("clahe_clip_limit", 20.0, describeDoubleParam("Clip Limit for Clahe filter", 0.0, 255.0));
    this->declare_parameter("clahe_tile_size_x", 32, describeIntParam("X Tile Size for Clahe filter", 0, 300));
    this->declare_parameter("clahe_tile_size_y", 32, describeIntParam("Y Tile Size for Clahe filter", 0, 300));
    applyParams();

    img_pub_ = advertiseImage("~/image", 1);

    on_set_params_handle_ = this->add_on_set_parameters_callback(
        std::bind(&EqualizeHistogramNode::onParamChange, this, std::placeholders::_1));

    cv::ocl::setUseOpenCL(use_opencl_);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::EqualizeHistogramNode)
