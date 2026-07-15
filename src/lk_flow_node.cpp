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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/lk_demo.cpp
/**
 * This is a demo of Lukas-Kanade optical flow lkdemo(),
 */

#include <functional>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_srvs/srv/empty.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "opencv_apps/msg/flow.hpp"
#include "opencv_apps/msg/flow_array_stamped.hpp"
#include "opencv_apps/node.hpp"

namespace opencv_apps
{
class LKFlowNode : public opencv_apps::Node
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::FlowArrayStamped>::SharedPtr msg_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr initialize_points_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr delete_points_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr toggle_night_mode_service_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  std::string window_name_;
  bool need_config_update_;

  bool use_camera_info_;
  double quality_level_;
  int min_distance_;
  int block_size_;
  double harris_k_;

  // Persistent tracking state across frames -- plain C++ state, carried over
  // unchanged from the ROS 1 nodelet.
  int MAX_COUNT;
  bool needToInit;
  bool nightMode;
  cv::Point2f point;
  bool addRemovePt;
  cv::Mat gray, prevGray;
  std::vector<cv::Point2f> points[2];

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyParams()
  {
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
    quality_level_ = this->get_parameter("quality_level").as_double();
    min_distance_ = this->get_parameter("min_distance").as_int();
    block_size_ = this->get_parameter("block_size").as_int();
    harris_k_ = this->get_parameter("harris_k").as_double();
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
    static_cast<LKFlowNode *>(userdata)->need_config_update_ = true;
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const std::string & /*input_frame_from_msg*/)
  {
    try
    {
      cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      opencv_apps::msg::FlowArrayStamped flows_msg;
      flows_msg.header = msg->header;

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);

        cv::createTrackbar("Min Distance", window_name_, &min_distance_, 100, &LKFlowNode::trackbarCallback, this);
        cv::createTrackbar("Block Size", window_name_, &block_size_, 100, &LKFlowNode::trackbarCallback, this);

        if (need_config_update_)
        {
          this->set_parameter(rclcpp::Parameter("min_distance", min_distance_));
          this->set_parameter(rclcpp::Parameter("block_size", block_size_));
          need_config_update_ = false;
        }
      }

      cv::TermCriteria termcrit(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.03);
      cv::Size sub_pix_win_size(10, 10), win_size(31, 31);

      if (image.channels() > 1)
      {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        image.copyTo(gray);
      }

      if (nightMode)
        image = cv::Scalar::all(0);

      if (needToInit)
      {
        // automatic initialization
        cv::goodFeaturesToTrack(gray, points[1], MAX_COUNT, quality_level_, min_distance_, cv::Mat(), block_size_,
                                 false, harris_k_);
        cv::cornerSubPix(gray, points[1], sub_pix_win_size, cv::Size(-1, -1), termcrit);
        addRemovePt = false;
      }
      else if (!points[0].empty())
      {
        std::vector<uchar> status;
        std::vector<float> err;
        if (prevGray.empty())
          gray.copyTo(prevGray);
        cv::calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, win_size, 3, termcrit, 0, 0.001);
        size_t i;
        for (i = 0; i < points[1].size(); i++)
        {
          if (addRemovePt)
          {
            if (cv::norm(point - points[1][i]) <= 5)
            {
              addRemovePt = false;
              continue;
            }
          }

          opencv_apps::msg::Flow flow_msg;
          opencv_apps::msg::Point2D point_msg;
          opencv_apps::msg::Point2D velocity_msg;
          if (status[i])
          {
            cv::circle(image, points[1][i], 3, cv::Scalar(0, 255, 0), -1, 8);
            cv::line(image, points[1][i], points[0][i], cv::Scalar(0, 255, 0), 1, 8, 0);
            point_msg.x = points[1][i].x;
            point_msg.y = points[1][i].y;
            velocity_msg.x = points[1][i].x - points[0][i].x;
            velocity_msg.y = points[1][i].y - points[0][i].y;
          }
          flow_msg.point = point_msg;
          flow_msg.velocity = velocity_msg;
          flows_msg.status.push_back(status[i]);
          flows_msg.flow.push_back(flow_msg);
        }
      }

      if (addRemovePt && points[1].size() < static_cast<size_t>(MAX_COUNT))
      {
        std::vector<cv::Point2f> tmp;
        tmp.push_back(point);
        cv::cornerSubPix(gray, tmp, win_size, cv::Size(-1, -1), termcrit);
        points[1].push_back(tmp[0]);
        addRemovePt = false;
      }

      needToInit = false;
      if (debug_view_)
      {
        cv::imshow(window_name_, image);

        char c = static_cast<char>(cv::waitKey(1));
        switch (c)
        {
          case 'r':
            needToInit = true;
            break;
          case 'c':
            points[0].clear();
            points[1].clear();
            break;
          case 'n':
            nightMode = !nightMode;
            break;
          default:;
        }
      }
      std::swap(points[1], points[0]);
      cv::swap(prevGray, gray);

      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();
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

  void initializePointsCb(
      const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    needToInit = true;
  }

  void deletePointsCb(
      const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    points[0].clear();
    points[1].clear();
  }

  void toggleNightModeCb(
      const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    nightMode = !nightMode;
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Subscribing to image topic.");
    std::string transport = image_transport::TransportHints(this).getTransport();
    if (use_camera_info_)
      cam_sub_ = image_transport::create_camera_subscription(
          this, "image",
          std::bind(&LKFlowNode::imageCallbackWithInfo, this, std::placeholders::_1, std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&LKFlowNode::imageCallback, this, std::placeholders::_1), transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit LKFlowNode(const rclcpp::NodeOptions & options) : opencv_apps::Node("lk_flow", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with tracked feature points."));
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0);

    window_name_ = "LK Demo";
    need_config_update_ = false;
    MAX_COUNT = 500;
    needToInit = true;
    nightMode = false;
    addRemovePt = false;

    this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    this->declare_parameter(
        "quality_level", 0.01,
        describeDoubleParam("Parameter characterizing the minimal acceptance quality of image corners.", 0.001, 10));
    this->declare_parameter(
        "min_distance", 10,
        describeIntParam("Minimum possible Euclidean distance between the returned corners.", 1, 100));
    this->declare_parameter(
        "block_size", 3,
        describeIntParam(
            "Size of an average block for computing a derivative covariation matrix over each pixel "
            "neighborhood.",
            1, 100));
    this->declare_parameter(
        "harris_k", 0.04, describeDoubleParam("Free parameter of the Harris detector", 0.001, 10));
    applyParams();

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::FlowArrayStamped>("~/flows", 1);
    initialize_points_service_ = this->create_service<std_srvs::srv::Empty>(
        "~/initialize_points", std::bind(&LKFlowNode::initializePointsCb, this, std::placeholders::_1,
                                          std::placeholders::_2));
    delete_points_service_ = this->create_service<std_srvs::srv::Empty>(
        "~/delete_points",
        std::bind(&LKFlowNode::deletePointsCb, this, std::placeholders::_1, std::placeholders::_2));
    toggle_night_mode_service_ = this->create_service<std_srvs::srv::Empty>(
        "~/toggle_night_mode",
        std::bind(&LKFlowNode::toggleNightModeCb, this, std::placeholders::_1, std::placeholders::_2));

    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&LKFlowNode::onParamChange, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Hot keys (with debug_view:=true): ");
    RCLCPP_INFO(this->get_logger(), "\tr - auto-initialize tracking");
    RCLCPP_INFO(this->get_logger(), "\tc - delete all the points");
    RCLCPP_INFO(this->get_logger(), "\tn - switch the \"night\" mode on/off");

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::LKFlowNode)
