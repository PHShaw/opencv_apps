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

// http://github.com/Itseez/opencv/blob/master/samples/cpp/tutorial_code/TrackingMotion/goodFeaturesToTrack_Demo.cpp
/**
 * @function goodFeaturesToTrack_Demo.cpp
 * @brief Demo code for detecting corners using Shi-Tomasi method
 * @author OpenCV team
 */

#include <functional>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv_apps/msg/point2_d_array_stamped.hpp"
#include "opencv_apps/node.hpp"

namespace opencv_apps
{
class GoodfeatureTrackNode : public opencv_apps::Node
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::Point2DArrayStamped>::SharedPtr msg_pub_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  std::string window_name_;
  bool need_config_update_;

  bool use_camera_info_;
  int max_corners_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyParams()
  {
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
    max_corners_ = this->get_parameter("max_corners").as_int();
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
    static_cast<GoodfeatureTrackNode *>(userdata)->need_config_update_ = true;
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const std::string & /*input_frame_from_msg*/)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Messages
      opencv_apps::msg::Point2DArrayStamped corners_msg;
      corners_msg.header = msg->header;

      // Do the work
      cv::Mat src_gray;
      int max_trackbar = 100;

      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, src_gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        src_gray = frame;
        cv::cvtColor(src_gray, frame, cv::COLOR_GRAY2BGR);
      }

      if (debug_view_)
      {
        /// Create Trackbars for Thresholds
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("Max corners", window_name_, &max_corners_, max_trackbar,
                            &GoodfeatureTrackNode::trackbarCallback, this);
        if (need_config_update_)
        {
          this->set_parameter(rclcpp::Parameter("max_corners", max_corners_));
          need_config_update_ = false;
        }
      }

      /// void goodFeaturesToTrack_Demo( int, void* )
      if (max_corners_ < 1)
      {
        max_corners_ = 1;
      }

      /// Parameters for Shi-Tomasi algorithm
      std::vector<cv::Point2f> corners;
      double quality_level = 0.01;
      double min_distance = 10;
      int block_size = 3;
      bool use_harris_detector = false;
      double k = 0.04;

      cv::RNG rng(12345);

      /// Apply corner detection
      cv::goodFeaturesToTrack(src_gray, corners, max_corners_, quality_level, min_distance, cv::Mat(), block_size,
                               use_harris_detector, k);

      /// Draw corners detected
      RCLCPP_INFO(this->get_logger(), "** Number of corners detected: %zu", corners.size());
      int r = 4;
      for (const cv::Point2f & corner : corners)
      {
        cv::circle(frame, corner, r, cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), -1, 8,
                    0);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow(window_name_, frame);
        cv::waitKey(1);
      }

      // Create msgs
      for (const cv::Point2f & i : corners)
      {
        opencv_apps::msg::Point2D corner;
        corner.x = i.x;
        corner.y = i.y;
        corners_msg.points.push_back(corner);
      }
      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
      img_pub_.publish(*out_img);
      msg_pub_->publish(corners_msg);
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
          std::bind(&GoodfeatureTrackNode::imageCallbackWithInfo, this, std::placeholders::_1,
                     std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&GoodfeatureTrackNode::imageCallback, this, std::placeholders::_1), transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit GoodfeatureTrackNode(const rclcpp::NodeOptions & options) : opencv_apps::Node("goodfeature_track", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with the detected corners."));
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0);
    need_config_update_ = false;

    window_name_ = "Image";

    this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    this->declare_parameter("max_corners", 23, describeIntParam("Max Number of Corners", 1, 100));
    applyParams();

    // Output uses "~/image"/"~/corners" (node-private) rather than plain names,
    // which would collide with the "image" input topic under a single remap.
    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::Point2DArrayStamped>("~/corners", 1);

    on_set_params_handle_ = this->add_on_set_parameters_callback(
        std::bind(&GoodfeatureTrackNode::onParamChange, this, std::placeholders::_1));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::GoodfeatureTrackNode)
