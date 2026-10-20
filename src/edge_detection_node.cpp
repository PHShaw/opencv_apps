// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Kei Okada.
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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/
/**
 * @file Sobel_Demo.cpp / Laplace_Demo.cpp / CannyDetector_Demo.cpp
 * @brief Sample code using Sobel/Scharr, Laplace and Canny to make a simple edge detector
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
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv_apps/node.hpp"

namespace opencv_apps
{
// Enum values for the `edge_type` parameter (was the EdgeDetectionConfig enum
// generated from cfg/EdgeDetection.cfg's `edge_type` gen.enum(...)).
enum EdgeType
{
  EDGE_TYPE_SOBEL = 0,
  EDGE_TYPE_LAPLACE = 1,
  EDGE_TYPE_CANNY = 2,
};

class EdgeDetectionNode : public opencv_apps::Node
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  // config_-mirroring cached fields, derived on every parameter update.
  bool use_camera_info_;
  int edge_type_;
  int canny_threshold1_;
  int canny_threshold2_;
  int apertureSize_;
  bool L2gradient_;
  bool apply_blur_pre_;
  bool apply_blur_post_;
  int postBlurSize_;
  double postBlurSigma_;

  std::string window_name_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyParams()
  {
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
    edge_type_ = this->get_parameter("edge_type").as_int();
    canny_threshold1_ = this->get_parameter("canny_threshold1").as_int();
    canny_threshold2_ = this->get_parameter("canny_threshold2").as_int();
    apertureSize_ = 2 * (this->get_parameter("apertureSize").as_int() / 2) + 1;
    L2gradient_ = this->get_parameter("L2gradient").as_bool();
    apply_blur_pre_ = this->get_parameter("apply_blur_pre").as_bool();
    apply_blur_post_ = this->get_parameter("apply_blur_post").as_bool();
    postBlurSize_ = 2 * (this->get_parameter("postBlurSize").as_int() / 2) + 1;
    postBlurSigma_ = this->get_parameter("postBlurSigma").as_double();
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(
      const std::vector<rclcpp::Parameter> & /*params*/)
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
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      cv::Mat src_gray;
      cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, src_gray, cv::COLOR_RGB2GRAY);
      }
      else
      {
        src_gray = frame;
      }

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      }

      std::string new_window_name;
      cv::Mat grad;
      switch (edge_type_)
      {
        case EDGE_TYPE_SOBEL:
        {
          cv::Mat grad_x, grad_y;
          cv::Mat abs_grad_x, abs_grad_y;

          int scale = 1;
          int delta = 0;
          int ddepth = CV_16S;

          cv::Sobel(src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
          cv::convertScaleAbs(grad_x, abs_grad_x);

          cv::Sobel(src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
          cv::convertScaleAbs(grad_y, abs_grad_y);

          cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

          new_window_name = "Sobel Edge Detection Demo";
          break;
        }
        case EDGE_TYPE_LAPLACE:
        {
          cv::Mat dst;
          int kernel_size = 3;
          int scale = 1;
          int delta = 0;
          int ddepth = CV_16S;

          cv::Laplacian(src_gray, dst, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT);
          convertScaleAbs(dst, grad);

          new_window_name = "Laplace Edge Detection Demo";
          break;
        }
        case EDGE_TYPE_CANNY:
        {
          int kernel_size = 3;
          int const max_canny_threshold1 = 500;
          int const max_canny_threshold2 = 500;

          if (apply_blur_pre_)
          {
            cv::blur(src_gray, src_gray, cv::Size(apertureSize_, apertureSize_));
          }

          cv::Canny(src_gray, grad, canny_threshold1_, canny_threshold2_, kernel_size, L2gradient_);
          if (apply_blur_post_)
          {
            cv::GaussianBlur(grad, grad, cv::Size(postBlurSize_, postBlurSize_), postBlurSigma_, postBlurSigma_);
          }

          new_window_name = "Canny Edge Detection Demo";

          if (debug_view_)
          {
            if (window_name_ == new_window_name)
            {
              if (syncTrackbar("Min CannyThreshold1:", window_name_, canny_threshold1_, max_canny_threshold1))
              {
                this->set_parameter(rclcpp::Parameter("canny_threshold1", canny_threshold1_));
              }
              if (syncTrackbar("Min CannyThreshold2:", window_name_, canny_threshold2_, max_canny_threshold2))
              {
                this->set_parameter(rclcpp::Parameter("canny_threshold2", canny_threshold2_));
              }
              syncParamTrackbar(window_name_, "L2gradient", L2gradient_);
              syncParamTrackbar(window_name_, "apply_blur_pre", apply_blur_pre_);
              syncParamTrackbar(window_name_, "apertureSize", apertureSize_);
              syncParamTrackbar(window_name_, "apply_blur_post", apply_blur_post_);
              syncParamTrackbar(window_name_, "postBlurSize", postBlurSize_);
              syncParamTrackbar(window_name_, "postBlurSigma", postBlurSigma_, 0.1);
            }
          }
          break;
        }
      }

      if (debug_view_)
      {
        if (window_name_ != new_window_name)
        {
          cv::destroyWindow(window_name_);
          window_name_ = new_window_name;
        }
        cv::imshow(window_name_, grad);
        // Every method's window gets the method selector; it takes effect from the next frame
        syncParamTrackbar(window_name_, "edge_type", edge_type_);
        cv::waitKey(1);
      }

      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, grad).toImageMsg();
      img_pub_.publish(*out_img);
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
          this, "image", std::bind(&EdgeDetectionNode::imageCallbackWithInfo, this, std::placeholders::_1,
                                    std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&EdgeDetectionNode::imageCallback, this, std::placeholders::_1), transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit EdgeDetectionNode(const rclcpp::NodeOptions & options) : opencv_apps::Node("edge_detection", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with the detected edges."));

    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0);

    window_name_ = "Edge Detection Demo";

    this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    this->declare_parameter(
        "edge_type", static_cast<int>(EDGE_TYPE_SOBEL),
        describeIntParam("Edge Detection method: 0 = Sobel, 1 = Laplace, 2 = Canny", 0, 2));
    this->declare_parameter(
        "canny_threshold1", 100, describeIntParam("First threshold for the hysteresis procedure.", 0, 500));
    this->declare_parameter(
        "canny_threshold2", 200, describeIntParam("Second threshold for the hysteresis procedure.", 0, 500));
    this->declare_parameter(
        "apertureSize", 3, describeIntParam("Aperture size for the Sobel() operator.", 1, 10));
    this->declare_parameter(
        "apply_blur_pre", true, describeParam("Flag, applying Blur() to input image"));
    this->declare_parameter(
        "postBlurSize", 13, describeIntParam("Aperture size for the Blur() operator.", 3, 31));
    this->declare_parameter(
        "postBlurSigma", 3.2, describeDoubleParam("Sigma for the GaussianBlur() operator.", 0.0, 10.0));
    this->declare_parameter(
        "apply_blur_post", false, describeParam("Flag, applying GaussianBlur() to output(edge) image"));
    this->declare_parameter(
        "L2gradient", false,
        describeParam("Flag, indicating whether a more accurate L_2 norm should be used."));
    applyParams();

    // Advertise before registering our on_set_parameters_callback: image_transport
    // declares its own parameters (image.enable_pub_plugins, image.jpeg_quality, ...)
    // internally when creating the publisher, and we don't want our callback firing
    // (and calling applyParams()/get_parameter() on our own params) during that.
    //
    // Output uses "~/image" (node-private, like ROS 1's pnh_->advertise("image", ...))
    // rather than "image", which is the input topic name -- otherwise a single
    // `-r image:=X` remap would send both the input subscription and the output
    // publisher to the same topic X.
    img_pub_ = advertiseImage("~/image", 1);

    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&EdgeDetectionNode::onParamChange, this, std::placeholders::_1));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::EdgeDetectionNode)
