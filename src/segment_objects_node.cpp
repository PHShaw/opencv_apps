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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/segment_objects.cpp
/**
 * This program demonstrated a simple method of connected components clean up of background subtraction
 */

#include <functional>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/background_segm.hpp>

#include "opencv_apps/msg/contour.hpp"
#include "opencv_apps/msg/contour_array_stamped.hpp"
#include "opencv_apps/node.hpp"

namespace opencv_apps
{
class SegmentObjectsNode : public opencv_apps::Node
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::ContourArrayStamped>::SharedPtr msg_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr area_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr update_bg_model_service_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  std::string window_name_;

  bool use_camera_info_;

  cv::Ptr<cv::BackgroundSubtractorMOG2> bgsubtractor;
  bool update_bg_model;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyParams()
  {
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
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

      opencv_apps::msg::ContourArrayStamped contours_msg;
      contours_msg.header = msg->header;

      cv::Mat bgmask, out_frame;

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      }

      bgsubtractor->apply(frame, bgmask, update_bg_model ? -1 : 0);
      int niters = 3;

      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;

      cv::Mat temp;

      cv::dilate(bgmask, temp, cv::Mat(), cv::Point(-1, -1), niters);
      cv::erode(temp, temp, cv::Mat(), cv::Point(-1, -1), niters * 2);
      cv::dilate(temp, temp, cv::Mat(), cv::Point(-1, -1), niters);

      cv::findContours(temp, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

      out_frame = cv::Mat::zeros(frame.size(), CV_8UC3);

      if (contours.empty())
        return;

      // iterate through all the top-level contours,
      // draw each connected component with its own random color
      int idx = 0, largest_comp = 0;
      double max_area = 0;

      for (; idx >= 0; idx = hierarchy[idx][0])
      {
        const std::vector<cv::Point> & c = contours[idx];
        double area = fabs(cv::contourArea(cv::Mat(c)));
        if (area > max_area)
        {
          max_area = area;
          largest_comp = idx;
        }
      }
      cv::Scalar color(0, 0, 255);
      cv::drawContours(out_frame, contours, largest_comp, color, cv::FILLED, 8, hierarchy);

      std_msgs::msg::Float64 area_msg;
      area_msg.data = max_area;
      for (const std::vector<cv::Point> & contour : contours)
      {
        opencv_apps::msg::Contour contour_msg;
        for (const cv::Point & j : contour)
        {
          opencv_apps::msg::Point2D point_msg;
          point_msg.x = j.x;
          point_msg.y = j.y;
          contour_msg.points.push_back(point_msg);
        }
        contours_msg.contours.push_back(contour_msg);
      }

      if (debug_view_)
      {
        cv::imshow(window_name_, out_frame);
        int keycode = cv::waitKey(1);
        if (keycode == ' ')
        {
          update_bg_model = !update_bg_model;
          RCLCPP_INFO(this->get_logger(), "Learn background is in state = %d", update_bg_model);
        }
      }

      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, out_frame).toImageMsg();
      img_pub_.publish(*out_img);
      msg_pub_->publish(contours_msg);
      area_pub_->publish(area_msg);
    }
    catch (cv::Exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(),
                   e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void updateBgModelCb(
      const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    update_bg_model = !update_bg_model;
    RCLCPP_INFO(this->get_logger(), "Learn background is in state = %d", update_bg_model);
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Subscribing to image topic.");
    std::string transport = image_transport::TransportHints(this).getTransport();
    if (use_camera_info_)
      cam_sub_ = image_transport::create_camera_subscription(
          this, "image",
          std::bind(&SegmentObjectsNode::imageCallbackWithInfo, this, std::placeholders::_1, std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&SegmentObjectsNode::imageCallback, this, std::placeholders::_1), transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit SegmentObjectsNode(const rclcpp::NodeOptions & options) : opencv_apps::Node("segment_objects", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with the segmented objects."));
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0);

    window_name_ = "segmented";
    update_bg_model = true;
    bgsubtractor = cv::createBackgroundSubtractorMOG2();

    this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    applyParams();

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::ContourArrayStamped>("~/contours", 1);
    area_pub_ = advertise<std_msgs::msg::Float64>("~/area", 1);
    update_bg_model_service_ = this->create_service<std_srvs::srv::Empty>(
        "~/update_bg_model",
        std::bind(&SegmentObjectsNode::updateBgModelCb, this, std::placeholders::_1, std::placeholders::_2));

    on_set_params_handle_ = this->add_on_set_parameters_callback(
        std::bind(&SegmentObjectsNode::onParamChange, this, std::placeholders::_1));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::SegmentObjectsNode)
