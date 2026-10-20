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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/HoughLines_Demo.cpp
/**
 * @file HoughLines_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

#include <cstdio>
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

#include "opencv_apps/msg/line_array_stamped.hpp"
#include "opencv_apps/node.hpp"

namespace
{
// Enum values for the `hough_type` parameter (was the HoughLinesConfig enum
// generated from cfg/HoughLines.cfg's `hough_type` gen.enum(...)).
enum HoughType
{
  HOUGH_TYPE_STANDARD = 0,
  HOUGH_TYPE_PROBABILISTIC = 1,
};
}  // namespace

namespace opencv_apps
{
class HoughLinesNode : public opencv_apps::Node
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::LineArrayStamped>::SharedPtr msg_pub_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  std::string window_name_;

  bool use_camera_info_;
  int min_threshold_;
  int max_threshold_;
  int hough_type_;
  int threshold_;
  double rho_;
  double theta_;
  double minLineLength_;
  double maxLineGap_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyParams()
  {
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
    hough_type_ = this->get_parameter("hough_type").as_int();
    threshold_ = this->get_parameter("threshold").as_int();
    rho_ = this->get_parameter("rho").as_double();
    theta_ = this->get_parameter("theta").as_double();
    minLineLength_ = this->get_parameter("minLineLength").as_double();
    maxLineGap_ = this->get_parameter("maxLineGap").as_double();
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

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const std::string & /*input_frame_from_msg*/)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat in_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::Mat src_gray;

      if (in_image.channels() > 1)
      {
        cv::cvtColor(in_image, src_gray, cv::COLOR_BGR2GRAY);
        /// Apply Canny edge detector
        cv::Canny(src_gray, in_image, 50, 200, 3);
      }
      else
      {
        /// Check whether input gray image is filtered such that canny, sobel ...etc
        bool is_filtered = true;
        for (int y = 0; y < in_image.rows; ++y)
        {
          for (int x = 0; x < in_image.cols; ++x)
          {
            if (!(in_image.at<unsigned char>(y, x) == 0 || in_image.at<unsigned char>(y, x) == 255))
            {
              is_filtered = false;
              break;
            }
            if (!is_filtered)
            {
              break;
            }
          }
        }

        if (!is_filtered)
        {
          cv::Canny(in_image, in_image, 50, 200, 3);
        }
      }

      cv::Mat out_image;
      cv::cvtColor(in_image, out_image, cv::COLOR_GRAY2BGR);

      // Messages
      opencv_apps::msg::LineArrayStamped lines_msg;
      lines_msg.header = msg->header;

      if (debug_view_)
      {
        /// Create Trackbars for Thresholds
        char thresh_label[50];
        snprintf(thresh_label, sizeof(thresh_label), "Thres: %d + input", min_threshold_);

        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        if (syncTrackbar(thresh_label, window_name_, threshold_, max_threshold_, min_threshold_))
        {
          this->set_parameter(rclcpp::Parameter("threshold", threshold_));
        }
        syncParamTrackbar(window_name_, "hough_type", hough_type_);
        syncParamTrackbar(window_name_, "rho", rho_, 1.0);
        syncParamTrackbar(window_name_, "theta", theta_, 1.0);
        syncParamTrackbar(window_name_, "minLineLength", minLineLength_, 1.0);
        syncParamTrackbar(window_name_, "maxLineGap", maxLineGap_, 1.0);
      }

      switch (hough_type_)
      {
        case HOUGH_TYPE_STANDARD:
        {
          std::vector<cv::Vec2f> s_lines;

          /// 1. Use Standard Hough Transform
          cv::HoughLines(in_image, s_lines, rho_, theta_ * CV_PI / 180, threshold_, minLineLength_, maxLineGap_);

          /// Show the result
          for (const cv::Vec2f & s_line : s_lines)
          {
            float r = s_line[0], t = s_line[1];
            double cos_t = cos(t), sin_t = sin(t);
            double x0 = r * cos_t, y0 = r * sin_t;
            double alpha = 1000;

            cv::Point pt1(cvRound(x0 + alpha * (-sin_t)), cvRound(y0 + alpha * cos_t));
            cv::Point pt2(cvRound(x0 - alpha * (-sin_t)), cvRound(y0 - alpha * cos_t));
            cv::line(out_image, pt1, pt2, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);

            opencv_apps::msg::Line line_msg;
            line_msg.pt1.x = pt1.x;
            line_msg.pt1.y = pt1.y;
            line_msg.pt2.x = pt2.x;
            line_msg.pt2.y = pt2.y;
            lines_msg.lines.push_back(line_msg);
          }

          break;
        }
        case HOUGH_TYPE_PROBABILISTIC:
        default:
        {
          std::vector<cv::Vec4i> p_lines;

          /// 2. Use Probabilistic Hough Transform
          cv::HoughLinesP(in_image, p_lines, rho_, theta_ * CV_PI / 180, threshold_, minLineLength_, maxLineGap_);

          /// Show the result
          for (const cv::Vec4i & l : p_lines)
          {
            cv::line(out_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 3, cv::LINE_AA);

            opencv_apps::msg::Line line_msg;
            line_msg.pt1.x = l[0];
            line_msg.pt1.y = l[1];
            line_msg.pt2.x = l[2];
            line_msg.pt2.y = l[3];
            lines_msg.lines.push_back(line_msg);
          }

          break;
        }
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow(window_name_, out_image);
        cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img = cv_bridge::CvImage(msg->header, "bgr8", out_image).toImageMsg();
      img_pub_.publish(*out_img);
      msg_pub_->publish(lines_msg);
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
          std::bind(&HoughLinesNode::imageCallbackWithInfo, this, std::placeholders::_1, std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&HoughLinesNode::imageCallback, this, std::placeholders::_1), transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit HoughLinesNode(const rclcpp::NodeOptions & options) : opencv_apps::Node("hough_lines", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with the detected lines."));
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0);

    window_name_ = "Hough Lines Demo";
    min_threshold_ = 50;
    max_threshold_ = 150;

    this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    this->declare_parameter(
        "hough_type", static_cast<int>(HOUGH_TYPE_STANDARD),
        describeIntParam("Hough Line Methods: 0 = Standard_Hough_Transform, 1 = Probabilistic_Hough_Transform", 0, 1));
    this->declare_parameter("threshold", 150, describeIntParam("Hough Line Threshold", 50, 150));
    this->declare_parameter(
        "rho", 1.0, describeDoubleParam("The resolution of the parameter r in pixels. We use 1 pixel.", 1.0, 100.0));
    this->declare_parameter(
        "theta", 1.0,
        describeDoubleParam("The resolution of the parameter theta in radians. We use 1 degree (CV_PI/180)", 1.0,
                             90.0));
    this->declare_parameter(
        "minLineLength", 30.0,
        describeDoubleParam("The minimum number of points that can form a line. Lines with less than this number "
                             "of points are disregarded.",
                             0.0, 500.0));
    this->declare_parameter(
        "maxLineGap", 10.0,
        describeDoubleParam("The maximum gap between two points to be considered in the same line.", 0.0, 100.0));
    applyParams();

    // Output uses "~/image"/"~/lines" (node-private) rather than plain names,
    // which would collide with the "image" input topic under a single remap.
    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::LineArrayStamped>("~/lines", 1);

    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&HoughLinesNode::onParamChange, this, std::placeholders::_1));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::HoughLinesNode)
