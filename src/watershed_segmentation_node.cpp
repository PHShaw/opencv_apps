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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/watershed.cpp
/**
 * This program demonstrates the famous watershed segmentation algorithm in OpenCV: watershed()
 */

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

#include "opencv_apps/msg/contour.hpp"
#include "opencv_apps/msg/contour_array_stamped.hpp"
#include "opencv_apps/msg/point2_d_array.hpp"
#include "opencv_apps/node.hpp"

namespace opencv_apps
{
class WatershedSegmentationNode : public opencv_apps::Node
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::ContourArrayStamped>::SharedPtr msg_pub_;
  rclcpp::Subscription<opencv_apps::msg::Point2DArray>::SharedPtr add_seed_points_sub_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  std::string window_name_, segment_name_;

  bool use_camera_info_;

  bool on_mouse_update_;
  int on_mouse_event_, on_mouse_x_, on_mouse_y_, on_mouse_flags_;

  // Persistent state across frames.
  cv::Mat markerMask, img_gray;
  cv::Point prevPt;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  static void onMouse(int event, int x, int y, int flags, void * userdata)
  {
    auto * self = static_cast<WatershedSegmentationNode *>(userdata);
    self->on_mouse_update_ = true;
    self->on_mouse_event_ = event;
    self->on_mouse_x_ = x;
    self->on_mouse_y_ = y;
    self->on_mouse_flags_ = flags;
  }

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

      // Initialize
      if (markerMask.empty())
      {
        cv::cvtColor(frame, markerMask, cv::COLOR_BGR2GRAY);
        markerMask = cv::Scalar::all(0);
      }
      cv::Mat tmp;
      cv::cvtColor(frame, tmp, cv::COLOR_BGR2GRAY);
      cv::cvtColor(tmp, img_gray, cv::COLOR_GRAY2BGR);

      if (debug_view_)
      {
        cv::imshow(window_name_, frame);
        cv::setMouseCallback(window_name_, &WatershedSegmentationNode::onMouse, this);

        if (on_mouse_update_)
        {
          int event = on_mouse_event_;
          int x = on_mouse_x_;
          int y = on_mouse_y_;
          int flags = on_mouse_flags_;
          on_mouse_update_ = false;

          if (x < 0 || x >= frame.cols || y < 0 || y >= frame.rows)
            return;
          if (event == cv::EVENT_LBUTTONUP || !(flags & cv::EVENT_FLAG_LBUTTON))
            prevPt = cv::Point(-1, -1);
          else if (event == cv::EVENT_LBUTTONDOWN)
            prevPt = cv::Point(x, y);
          else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON))
          {
            cv::Point pt(x, y);
            if (prevPt.x < 0)
              prevPt = pt;
            cv::line(markerMask, prevPt, pt, cv::Scalar::all(255), 5, 8, 0);
            cv::line(frame, prevPt, pt, cv::Scalar::all(255), 5, 8, 0);
            prevPt = pt;
            cv::imshow(window_name_, markerMask);
          }
        }
        cv::waitKey(1);
      }

      int i, j, comp_count = 0;
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;

      cv::findContours(markerMask, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

      if (contours.empty())
      {
        RCLCPP_WARN(this->get_logger(), "contours are empty");
        return;
      }
      cv::Mat markers(markerMask.size(), CV_32S);
      markers = cv::Scalar::all(0);
      int idx = 0;
      for (; idx >= 0; idx = hierarchy[idx][0], comp_count++)
        cv::drawContours(markers, contours, idx, cv::Scalar::all(comp_count + 1), -1, 8, hierarchy, INT_MAX);

      if (comp_count == 0)
      {
        RCLCPP_WARN(this->get_logger(), "compCount is 0");
        return;
      }
      for (const std::vector<cv::Point> & contour : contours)
      {
        opencv_apps::msg::Contour contour_msg;
        for (const cv::Point & j2 : contour)
        {
          opencv_apps::msg::Point2D point_msg;
          point_msg.x = j2.x;
          point_msg.y = j2.y;
          contour_msg.points.push_back(point_msg);
        }
        contours_msg.contours.push_back(contour_msg);
      }

      std::vector<cv::Vec3b> color_tab;
      for (i = 0; i < comp_count; i++)
      {
        int b = cv::theRNG().uniform(0, 255);
        int g = cv::theRNG().uniform(0, 255);
        int r = cv::theRNG().uniform(0, 255);

        color_tab.push_back(cv::Vec3b(static_cast<uchar>(b), static_cast<uchar>(g), static_cast<uchar>(r)));
      }

      double t = static_cast<double>(cv::getTickCount());
      cv::watershed(frame, markers);
      t = static_cast<double>(cv::getTickCount()) - t;
      RCLCPP_INFO(this->get_logger(), "execution time = %gms", t * 1000. / cv::getTickFrequency());

      cv::Mat wshed(markers.size(), CV_8UC3);

      // paint the watershed image
      for (i = 0; i < markers.rows; i++)
        for (j = 0; j < markers.cols; j++)
        {
          int index = markers.at<int>(i, j);
          if (index == -1)
            wshed.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
          else if (index <= 0 || index > comp_count)
            wshed.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
          else
            wshed.at<cv::Vec3b>(i, j) = color_tab[index - 1];
        }

      wshed = wshed * 0.5 + img_gray * 0.5;

      if (debug_view_)
      {
        cv::imshow(segment_name_, wshed);

        int c = cv::waitKey(1);
        if (static_cast<char>(c) == 'r')
        {
          markerMask = cv::Scalar::all(0);
        }
      }

      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, msg->encoding, wshed).toImageMsg();
      img_pub_.publish(*out_img);
      msg_pub_->publish(contours_msg);
    }
    catch (cv::Exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(),
                   e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void addSeedPointCb(const opencv_apps::msg::Point2DArray::ConstSharedPtr & msg)
  {
    if (msg->points.empty())
    {
      markerMask = cv::Scalar::all(0);
    }
    else
    {
      for (const opencv_apps::msg::Point2D & point : msg->points)
      {
        cv::Point pt0(static_cast<int>(point.x), static_cast<int>(point.y));
        cv::Point pt1(pt0.x + 1, pt0.y + 1);
        cv::line(markerMask, pt0, pt1, cv::Scalar::all(255), 5, 8, 0);
      }
    }
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Subscribing to image topic.");
    std::string transport = image_transport::TransportHints(this).getTransport();
    if (use_camera_info_)
      cam_sub_ = image_transport::create_camera_subscription(
          this, "image",
          std::bind(&WatershedSegmentationNode::imageCallbackWithInfo, this, std::placeholders::_1,
                     std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&WatershedSegmentationNode::imageCallback, this, std::placeholders::_1),
          transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit WatershedSegmentationNode(const rclcpp::NodeOptions & options)
    : opencv_apps::Node("watershed_segmentation", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with mouse-driven seed marking."));
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0);

    window_name_ = "roughly mark the areas to segment on the image";
    segment_name_ = "watershed transform";
    on_mouse_update_ = false;
    on_mouse_event_ = 0;
    on_mouse_x_ = 0;
    on_mouse_y_ = 0;
    on_mouse_flags_ = 0;
    prevPt.x = -1;
    prevPt.y = -1;

    this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    applyParams();

    // add_seed_points is a plain external-seed input, always subscribed
    // (matching the ROS 1 nodelet, which subscribed it directly in onInit
    // rather than through subscribe()/unsubscribe()).
    add_seed_points_sub_ = this->create_subscription<opencv_apps::msg::Point2DArray>(
        "add_seed_points", 1,
        std::bind(&WatershedSegmentationNode::addSeedPointCb, this, std::placeholders::_1));
    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::ContourArrayStamped>("~/contours", 1);

    on_set_params_handle_ = this->add_on_set_parameters_callback(
        std::bind(&WatershedSegmentationNode::onParamChange, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "This program demonstrates the famous watershed segmentation algorithm in OpenCV: watershed()");
    RCLCPP_INFO(this->get_logger(), "Hot keys (with debug_view:=true): ");
    RCLCPP_INFO(this->get_logger(), "\tr - restore the original image");
    RCLCPP_INFO(this->get_logger(), "\t(before running it, *roughly* mark the areas to segment on the image)");

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::WatershedSegmentationNode)
