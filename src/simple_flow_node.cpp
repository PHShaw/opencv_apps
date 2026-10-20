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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/simpleflow_demo.cpp
/**
 * This is a demo of SimpleFlow optical flow algorithm
 */

// NOTE: this node requires OpenCV's contrib "optflow" module
// (opencv2/optflow.hpp, cv::optflow::calcOpticalFlowSF) -- same
// as the ROS 1 nodelet, which was only compiled
// `if(OPENCV_HAVE_OPTFLOW)`. See report for details on wiring this
// conditionally into CMakeLists.txt.

#include <algorithm>
#include <functional>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/video/tracking.hpp>

#include "opencv_apps/msg/flow.hpp"
#include "opencv_apps/msg/flow_array_stamped.hpp"
#include "opencv_apps/node.hpp"

namespace opencv_apps
{
class SimpleFlowNode : public opencv_apps::Node
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::FlowArrayStamped>::SharedPtr msg_pub_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  std::string window_name_;

  bool use_camera_info_;
  int scale_;

  // Persistent state across frames.
  cv::Mat color, prevColor;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyParams()
  {
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
    scale_ = this->get_parameter("scale").as_int();
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
      cv::Mat frame_src = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Convert it to 3 channel
      cv::Mat frame;
      if (frame_src.channels() > 1)
      {
        frame = frame_src;
      }
      else
      {
        cv::cvtColor(frame_src, frame, cv::COLOR_GRAY2BGR);
      }

      cv::resize(frame, color,
                 cv::Size(frame.cols / static_cast<double>(std::max(1, scale_)),
                          frame.rows / static_cast<double>(std::max(1, scale_))),
                 0, 0, cv::INTER_AREA);
      if (prevColor.empty())
        color.copyTo(prevColor);

      if (color.rows != prevColor.rows && color.cols != prevColor.cols)
      {
        RCLCPP_WARN(this->get_logger(), "Images should be of equal sizes");
        color.copyTo(prevColor);
      }

      if (frame.type() != CV_8UC3)
      {
        RCLCPP_ERROR(this->get_logger(), "Images should be of equal type CV_8UC3");
      }

      opencv_apps::msg::FlowArrayStamped flows_msg;
      flows_msg.header = msg->header;

      cv::Mat flow;

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        if (syncTrackbar("Scale", window_name_, scale_, 24, 1))
        {
          this->set_parameter(rclcpp::Parameter("scale", scale_));
        }
      }

      float start = static_cast<float>(cv::getTickCount());
      cv::optflow::calcOpticalFlowSF(color, prevColor, flow, 3, 2, 4, 4.1, 25.5, 18, 55.0, 25.5, 0.35, 18, 55.0, 25.5,
                                      10);
      RCLCPP_INFO(this->get_logger(), "calcOpticalFlowSF : %lf sec",
                  (cv::getTickCount() - start) / cv::getTickFrequency());

      int cols = flow.cols;
      int rows = flow.rows;
      double scale_col = frame.cols / static_cast<double>(flow.cols);
      double scale_row = frame.rows / static_cast<double>(flow.rows);

      for (int i = 0; i < rows; ++i)
      {
        for (int j = 0; j < cols; ++j)
        {
          cv::Vec2f flow_at_point = flow.at<cv::Vec2f>(i, j);
          cv::line(frame, cv::Point(scale_col * j, scale_row * i),
                   cv::Point(scale_col * (j + flow_at_point[0]), scale_row * (i + flow_at_point[1])),
                   cv::Scalar(0, 255, 0), 1, 8, 0);

          opencv_apps::msg::Flow flow_msg;
          opencv_apps::msg::Point2D point_msg;
          opencv_apps::msg::Point2D velocity_msg;
          point_msg.x = scale_col * j;
          point_msg.y = scale_row * i;
          velocity_msg.x = scale_col * flow_at_point[0];
          velocity_msg.y = scale_row * flow_at_point[1];
          flow_msg.point = point_msg;
          flow_msg.velocity = velocity_msg;
          flows_msg.status.push_back(true);
          flows_msg.flow.push_back(flow_msg);
        }
      }

      if (debug_view_)
      {
        cv::imshow(window_name_, frame);
        cv::waitKey(1);
      }

      cv::swap(prevColor, color);
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
      img_pub_.publish(*out_img);
      msg_pub_->publish(flows_msg);
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
          std::bind(&SimpleFlowNode::imageCallbackWithInfo, this, std::placeholders::_1, std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&SimpleFlowNode::imageCallback, this, std::placeholders::_1), transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit SimpleFlowNode(const rclcpp::NodeOptions & options) : opencv_apps::Node("simple_flow", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with the dense optical flow."));
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0);

    window_name_ = "simpleflow_demo";

    this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    this->declare_parameter("scale", 4, describeIntParam("Scale", 1, 24));
    applyParams();

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::FlowArrayStamped>("~/flows", 1);

    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&SimpleFlowNode::onParamChange, this, std::placeholders::_1));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::SimpleFlowNode)
