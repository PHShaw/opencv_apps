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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/HoughCircle_Demo.cpp
/**
 * @file HoughCircle_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

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

#include "opencv_apps/msg/circle.hpp"
#include "opencv_apps/msg/circle_array_stamped.hpp"
#include "opencv_apps/node.hpp"

namespace opencv_apps
{
// Enum values for the `debug_image_type` parameter (was the HoughCirclesConfig
// enum generated from cfg/HoughCircles.cfg's `debug_image_type` gen.enum(...)).
enum DebugImageType
{
  DEBUG_IMAGE_INPUT = 0,
  DEBUG_IMAGE_BLUR = 1,
  DEBUG_IMAGE_CANNY = 2,
};

class HoughCirclesNode : public opencv_apps::Node
{
  image_transport::Publisher img_pub_, debug_image_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::CircleArrayStamped>::SharedPtr msg_pub_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  std::string window_name_;

  bool use_camera_info_;
  double canny_threshold_;
  int canny_threshold_int_;  // for trackbar
  double accumulator_threshold_;
  int accumulator_threshold_int_;
  int gaussian_blur_size_;
  double gaussian_sigma_x_;
  int gaussian_sigma_x_int_;
  double gaussian_sigma_y_;
  int gaussian_sigma_y_int_;
  double min_distance_between_circles_;
  int min_distance_between_circles_int_;
  double dp_;
  int dp_int_;
  int min_circle_radius_;
  int max_circle_radius_;
  int debug_image_type_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyParams()
  {
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
    canny_threshold_ = this->get_parameter("canny_threshold").as_double();
    accumulator_threshold_ = this->get_parameter("accumulator_threshold").as_double();
    gaussian_blur_size_ = this->get_parameter("gaussian_blur_size").as_int();
    gaussian_sigma_x_ = this->get_parameter("gaussian_sigma_x").as_double();
    gaussian_sigma_y_ = this->get_parameter("gaussian_sigma_y").as_double();
    min_distance_between_circles_ = this->get_parameter("min_distance_between_circles").as_double();
    dp_ = this->get_parameter("dp").as_double();
    min_circle_radius_ = this->get_parameter("min_circle_radius").as_int();
    max_circle_radius_ = this->get_parameter("max_circle_radius").as_int();
    debug_image_type_ = this->get_parameter("debug_image_type").as_int();

    canny_threshold_int_ = static_cast<int>(canny_threshold_);
    accumulator_threshold_int_ = static_cast<int>(accumulator_threshold_);
    gaussian_sigma_x_int_ = static_cast<int>(gaussian_sigma_x_);
    gaussian_sigma_y_int_ = static_cast<int>(gaussian_sigma_y_);
    min_distance_between_circles_int_ = static_cast<int>(min_distance_between_circles_);
    dp_int_ = static_cast<int>(dp_);
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
    try
    {
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      opencv_apps::msg::CircleArrayStamped circles_msg;
      circles_msg.header = msg->header;

      cv::Mat src_gray, edges;

      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, src_gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        src_gray = frame;
      }

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);

        if (syncTrackbar("Canny Threshold", window_name_, canny_threshold_int_, 255, 1))
        {
          canny_threshold_ = static_cast<double>(canny_threshold_int_);
          this->set_parameter(rclcpp::Parameter("canny_threshold", canny_threshold_));
        }
        if (syncTrackbar("Accumulator Threshold", window_name_, accumulator_threshold_int_, 200, 1))
        {
          accumulator_threshold_ = static_cast<double>(accumulator_threshold_int_);
          this->set_parameter(rclcpp::Parameter("accumulator_threshold", accumulator_threshold_));
        }
        if (syncTrackbar("Gaussian Blur Size", window_name_, gaussian_blur_size_, 30, 1))
        {
          this->set_parameter(rclcpp::Parameter("gaussian_blur_size", gaussian_blur_size_));
        }
        if (syncTrackbar("Gaussian Sigam X", window_name_, gaussian_sigma_x_int_, 10, 1))
        {
          gaussian_sigma_x_ = static_cast<double>(gaussian_sigma_x_int_);
          this->set_parameter(rclcpp::Parameter("gaussian_sigma_x", gaussian_sigma_x_));
        }
        if (syncTrackbar("Gaussian Sigma Y", window_name_, gaussian_sigma_y_int_, 10, 1))
        {
          gaussian_sigma_y_ = static_cast<double>(gaussian_sigma_y_int_);
          this->set_parameter(rclcpp::Parameter("gaussian_sigma_y", gaussian_sigma_y_));
        }
        if (syncTrackbar("Min Distance between Circles", window_name_, min_distance_between_circles_int_, 100))
        {
          min_distance_between_circles_ = static_cast<double>(min_distance_between_circles_int_);
          this->set_parameter(rclcpp::Parameter("min_distance_between_circles", min_distance_between_circles_));
        }
        if (syncTrackbar("Dp", window_name_, dp_int_, 100))
        {
          dp_ = static_cast<double>(dp_int_);
          this->set_parameter(rclcpp::Parameter("dp", dp_));
        }
        if (syncTrackbar("Min Circle Radius", window_name_, min_circle_radius_, 500))
        {
          this->set_parameter(rclcpp::Parameter("min_circle_radius", min_circle_radius_));
        }
        if (syncTrackbar("Max Circle Radius", window_name_, max_circle_radius_, 2000))
        {
          this->set_parameter(rclcpp::Parameter("max_circle_radius", max_circle_radius_));
        }
        syncParamTrackbar(window_name_, "debug_image_type", debug_image_type_);
      }

      // Reduce the noise so we avoid false circle detection
      // gaussian_blur_size_ must be odd number
      if (gaussian_blur_size_ % 2 != 1)
      {
        gaussian_blur_size_ = gaussian_blur_size_ + 1;
      }
      cv::GaussianBlur(src_gray, src_gray, cv::Size(gaussian_blur_size_, gaussian_blur_size_), gaussian_sigma_x_,
                        gaussian_sigma_y_);

      // those parameters cannot be =0, so we must check here
      canny_threshold_ = std::max(canny_threshold_, 1.0);
      accumulator_threshold_ = std::max(accumulator_threshold_, 1.0);

      if (debug_view_)
      {
        // https://github.com/Itseez/opencv/blob/2.4.8/modules/imgproc/src/hough.cpp#L817
        cv::Canny(frame, edges, std::max(canny_threshold_ / 2, 1.0), canny_threshold_, 3);
      }
      if (min_distance_between_circles_ == 0)
      {  // set initial value
        min_distance_between_circles_ = src_gray.rows / 8;
        this->set_parameter(rclcpp::Parameter("min_distance_between_circles", min_distance_between_circles_));
      }

      // runs the detection, and update the display
      std::vector<cv::Vec3f> circles;
      cv::HoughCircles(src_gray, circles, cv::HOUGH_GRADIENT, dp_, min_distance_between_circles_, canny_threshold_,
                        accumulator_threshold_, min_circle_radius_, max_circle_radius_);

      cv::Mat out_image;
      if (frame.channels() == 1)
      {
        cv::cvtColor(frame, out_image, cv::COLOR_GRAY2BGR);
      }
      else
      {
        out_image = frame;
      }

      for (const cv::Vec3f & i : circles)
      {
        cv::Point center(cvRound(i[0]), cvRound(i[1]));
        int radius = cvRound(i[2]);
        // circle center
        cv::circle(out_image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // circle outline
        cv::circle(out_image, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);

        opencv_apps::msg::Circle circle_msg;
        circle_msg.center.x = center.x;
        circle_msg.center.y = center.y;
        circle_msg.radius = radius;
        circles_msg.circles.push_back(circle_msg);
      }

      // shows the results
      if (debug_view_ || debug_image_pub_.getNumSubscribers() > 0)
      {
        cv::Mat debug_image;
        switch (debug_image_type_)
        {
          case DEBUG_IMAGE_BLUR:
            debug_image = src_gray;
            break;
          case DEBUG_IMAGE_CANNY:
            debug_image = edges;
            break;
          default:
            debug_image = frame;
            break;
        }
        if (debug_view_)
        {
          cv::imshow(window_name_, debug_image);
          int c = cv::waitKey(1);
          if (c == 's')
          {
            debug_image_type_ = (debug_image_type_ + 1) % 3;
            this->set_parameter(rclcpp::Parameter("debug_image_type", debug_image_type_));
          }
        }
        if (debug_image_pub_.getNumSubscribers() > 0)
        {
          sensor_msgs::msg::Image::SharedPtr out_debug_img =
              cv_bridge::CvImage(msg->header, msg->encoding, debug_image).toImageMsg();
          debug_image_pub_.publish(*out_debug_img);
        }
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, out_image).toImageMsg();
      img_pub_.publish(*out_img);
      msg_pub_->publish(circles_msg);
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
          std::bind(&HoughCirclesNode::imageCallbackWithInfo, this, std::placeholders::_1, std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&HoughCirclesNode::imageCallback, this, std::placeholders::_1), transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit HoughCirclesNode(const rclcpp::NodeOptions & options) : opencv_apps::Node("hough_circles", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with the detected circles."));
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0);

    window_name_ = "Hough Circle Detection Demo";

    this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    this->declare_parameter("canny_threshold", 200.0, describeDoubleParam("Canny threshold", 1, 255));
    this->declare_parameter("accumulator_threshold", 50.0, describeDoubleParam("Accumulator threshold", 1, 200));
    this->declare_parameter(
        "gaussian_blur_size", 9,
        describeIntParam("the size of gaussian blur (should be odd number)", 1, 30));
    this->declare_parameter("gaussian_sigma_x", 2.0, describeDoubleParam("sigma x of gaussian kernel", 1, 10));
    this->declare_parameter("gaussian_sigma_y", 2.0, describeDoubleParam("sigma y of gaussian kernel", 1, 10));
    this->declare_parameter(
        "min_distance_between_circles", 0.0,
        describeDoubleParam("minimum distance between the centers of the detected circles", 0, 1024));
    this->declare_parameter("dp", 2.0, describeDoubleParam("dp", 0, 100));
    this->declare_parameter(
        "min_circle_radius", 0, describeIntParam("the minimum size of the circle", 0, 500));
    this->declare_parameter(
        "max_circle_radius", 0, describeIntParam("the maximum size of the circle", 0, 2000));
    this->declare_parameter(
        "debug_image_type", static_cast<int>(DEBUG_IMAGE_INPUT),
        describeIntParam("Select image type for debug output: 0 = Input image, 1 = GaussianBlur image, "
                          "2 = Canny edge image",
                          0, 2));
    applyParams();

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::CircleArrayStamped>("~/circles", 1);
    debug_image_pub_ = advertiseImage("~/debug_image", 1);

    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&HoughCirclesNode::onParamChange, this, std::placeholders::_1));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::HoughCirclesNode)
