// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) JSK, 2016 Lab
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

// https://github.com/opencv/opencv/tree/2.4/samples/cpp/tutorial_code/ImgProc/AddingImages.cpp
/**
 * This is a demo of adding image (linear blending).
 */

#include <algorithm>
#include <functional>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include "opencv_apps/node.hpp"

namespace opencv_apps
{
class AddingImagesNode : public opencv_apps::Node
{
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                                                     sensor_msgs::msg::CameraInfo>
      SyncPolicyWithCameraInfo;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                                                           sensor_msgs::msg::CameraInfo>
      ApproxSyncPolicyWithCameraInfo;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>
      ApproxSyncPolicy;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  std::string window_name_;

  image_transport::SubscriberFilter sub_image1_, sub_image2_;
  image_transport::Publisher img_pub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_camera_info_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicyWithCameraInfo>> sync_with_info_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicyWithCameraInfo>> async_with_info_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> async_;

  bool use_camera_info_;
  bool approximate_sync_;
  bool auto_beta_;
  double alpha_;
  double beta_;
  double gamma_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyParams()
  {
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
    alpha_ = this->get_parameter("alpha").as_double();
    auto_beta_ = this->get_parameter("auto_beta").as_bool();
    beta_ = auto_beta_ ? 1.0 - alpha_ : this->get_parameter("beta").as_double();
    gamma_ = this->get_parameter("gamma").as_double();
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> &)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    deferApplyParams([this] { applyParams(); });
    return result;
  }

  void imageCallbackWithInfo(
      const sensor_msgs::msg::Image::ConstSharedPtr & msg1, const sensor_msgs::msg::Image::ConstSharedPtr & msg2,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
  {
    doWork(msg1, msg2, cam_info->header.frame_id);
  }

  void imageCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr & msg1, const sensor_msgs::msg::Image::ConstSharedPtr & msg2)
  {
    doWork(msg1, msg2, msg1->header.frame_id);
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Subscribing to image topics.");
    std::string transport = image_transport::TransportHints(this).getTransport();
    sub_image1_.subscribe(this, "image1", transport);
    sub_image2_.subscribe(this, "image2", transport);
    sub_camera_info_.subscribe(this, "camera_info");
    if (use_camera_info_)
    {
      if (approximate_sync_)
      {
        async_with_info_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicyWithCameraInfo>>(
            ApproxSyncPolicyWithCameraInfo(queue_size_));
        async_with_info_->connectInput(sub_image1_, sub_image2_, sub_camera_info_);
        async_with_info_->registerCallback(
            std::bind(&AddingImagesNode::imageCallbackWithInfo, this, std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3));
      }
      else
      {
        sync_with_info_ = std::make_shared<message_filters::Synchronizer<SyncPolicyWithCameraInfo>>(
            SyncPolicyWithCameraInfo(queue_size_));
        sync_with_info_->connectInput(sub_image1_, sub_image2_, sub_camera_info_);
        sync_with_info_->registerCallback(
            std::bind(&AddingImagesNode::imageCallbackWithInfo, this, std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3));
      }
    }
    else
    {
      if (approximate_sync_)
      {
        async_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(ApproxSyncPolicy(queue_size_));
        async_->connectInput(sub_image1_, sub_image2_);
        async_->registerCallback(
            std::bind(&AddingImagesNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
      }
      else
      {
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(queue_size_));
        sync_->connectInput(sub_image1_, sub_image2_);
        sync_->registerCallback(
            std::bind(&AddingImagesNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
      }
    }
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topics.");
    sub_image1_.unsubscribe();
    sub_image2_.unsubscribe();
    sub_camera_info_.unsubscribe();
  }

  void doWork(
      const sensor_msgs::msg::Image::ConstSharedPtr & image_msg1,
      const sensor_msgs::msg::Image::ConstSharedPtr & image_msg2, const std::string & /*input_frame_from_msg*/)
  {
    try
    {
      cv::Mat image1 = cv_bridge::toCvCopy(image_msg1, image_msg1->encoding)->image;
      cv::Mat image2 = cv_bridge::toCvCopy(image_msg2, image_msg1->encoding)->image;
      if (cv_bridge::getCvType(image_msg1->encoding) != cv_bridge::getCvType(image_msg2->encoding))
      {
        RCLCPP_ERROR(this->get_logger(), "Encoding of input images must be same type: %s, %s",
                     image_msg1->encoding.c_str(), image_msg2->encoding.c_str());
        return;
      }

      cv::Mat result_image;
      if (image1.rows != image2.rows || image1.cols != image2.cols)
      {
        int new_rows = std::max(image1.rows, image2.rows);
        int new_cols = std::max(image1.cols, image2.cols);
        cv::Mat image1_tmp = cv::Mat(new_rows, new_cols, image1.type());
        image1.copyTo(image1_tmp(cv::Rect(0, 0, image1.cols, image1.rows)));
        image1 = image1_tmp.clone();

        cv::Mat image2_tmp = cv::Mat(new_rows, new_cols, image2.type());
        image2.copyTo(image2_tmp(cv::Rect(0, 0, image2.cols, image2.rows)));
        image2 = image2_tmp.clone();
      }
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        bool changed = syncParamTrackbar(window_name_, "alpha", alpha_, 0.01);
        changed |= syncParamTrackbar(window_name_, "auto_beta", auto_beta_);
        // beta_ holds 1 - alpha when auto_beta is set, so slide the parameter's own value
        double beta = this->get_parameter("beta").as_double();
        changed |= syncParamTrackbar(window_name_, "beta", beta, 0.01);
        changed |= syncParamTrackbar(window_name_, "gamma", gamma_, 1.0);
        if (changed)
        {
          beta_ = auto_beta_ ? 1.0 - alpha_ : beta;
        }
      }
      cv::addWeighted(image1, alpha_, image2, beta_, gamma_, result_image);

      sensor_msgs::msg::Image::SharedPtr image_msg3 =
          cv_bridge::CvImage(image_msg1->header, image_msg1->encoding, result_image).toImageMsg();
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv_bridge::CvtColorForDisplayOptions options;
        if (sensor_msgs::image_encodings::bitDepth(image_msg1->encoding) == 32 ||
            sensor_msgs::image_encodings::bitDepth(image_msg1->encoding) == 64)
        {
          options.do_dynamic_scaling = true;
        }
        cv::imshow(window_name_, cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg3), "", options)->image);
        cv::waitKey(1);
      }
      img_pub_.publish(*image_msg3);
    }
    catch (cv::Exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(),
                   e.file.c_str(), e.line);
    }

    prev_stamp_ = image_msg1->header.stamp;
  }

public:
  explicit AddingImagesNode(const rclcpp::NodeOptions & options) : opencv_apps::Node("adding_images", options)
  {
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with the blended image."));
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0);
    window_name_ = "AddingImages Demo";

    this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    this->declare_parameter(
        "alpha", 0.5, describeDoubleParam("weight of the first array elements.", 0.0, 1.0));
    this->declare_parameter(
        "auto_beta", true,
        describeParam("True: Automatically set beta weight as 1 - alpha, False: Use user defined beta weight"));
    this->declare_parameter(
        "beta", 0.5, describeDoubleParam("weight of the second array elements.", 0.0, 1.0));
    this->declare_parameter("gamma", 0.0, describeDoubleParam("scalar added to each sum.", 0.0, 255.0));
    applyParams();

    approximate_sync_ = this->declare_parameter(
        "approximate_sync", true, describeParam("Use approximate-time instead of exact-time synchronization."));
    queue_size_ = this->declare_parameter("queue_size", 100, describeParam("Synchronizer queue size."));

    img_pub_ = advertiseImage("~/image", 1);

    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&AddingImagesNode::onParamChange, this, std::placeholders::_1));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::AddingImagesNode)
