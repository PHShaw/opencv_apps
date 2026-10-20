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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/peopledetect.cpp
/**
 * Demonstrate the use of the HoG descriptor using
 * HOGDescriptor::hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
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
#include <opencv2/objdetect/objdetect.hpp>

#include "opencv_apps/msg/rect.hpp"
#include "opencv_apps/msg/rect_array_stamped.hpp"
#include "opencv_apps/node.hpp"

namespace opencv_apps
{
class PeopleDetectNode : public opencv_apps::Node
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::RectArrayStamped>::SharedPtr msg_pub_;

  int queue_size_;
  bool debug_view_;
  bool use_camera_info_;
  rclcpp::Time prev_stamp_;

  std::string window_name_;

  cv::HOGDescriptor hog_;

  double hit_threshold_;
  int win_stride_;
  int padding_;
  double scale0_;
  int group_threshold_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void applyParams()
  {
    use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
    hit_threshold_ = this->get_parameter("hit_threshold").as_double();
    win_stride_ = this->get_parameter("win_stride").as_int();
    padding_ = this->get_parameter("padding").as_int();
    scale0_ = this->get_parameter("scale0").as_double();
    group_threshold_ = this->get_parameter("group_threshold").as_int();
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

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const std::string & /*input_frame_from_msg*/)
  {
    try
    {
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      opencv_apps::msg::RectArrayStamped found_msg;
      found_msg.header = msg->header;

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        syncParamTrackbar(window_name_, "hit_threshold", hit_threshold_, 0.1);
        syncParamTrackbar(window_name_, "win_stride", win_stride_);
        syncParamTrackbar(window_name_, "padding", padding_);
        syncParamTrackbar(window_name_, "scale0", scale0_, 0.01);
        syncParamTrackbar(window_name_, "group_threshold", group_threshold_);
      }

      std::vector<cv::Rect> found, found_filtered;
      double t = (double)cv::getTickCount();
      // run the detector with default parameters. to get a higher hit-rate
      // (and more false alarms, respectively), decrease the hitThreshold and
      // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
      hog_.detectMultiScale(frame, found, hit_threshold_, cv::Size(win_stride_, win_stride_),
                            cv::Size(padding_, padding_), scale0_, group_threshold_);
      t = (double)cv::getTickCount() - t;
      RCLCPP_INFO(this->get_logger(), "tdetection time = %gms", t * 1000. / cv::getTickFrequency());
      size_t i, j;
      for (i = 0; i < found.size(); i++)
      {
        cv::Rect r = found[i];
        for (j = 0; j < found.size(); j++)
          if (j != i && (r & found[j]) == r)
            break;
        if (j == found.size())
          found_filtered.push_back(r);
      }
      for (i = 0; i < found_filtered.size(); i++)
      {
        cv::Rect r = found_filtered[i];
        // the HOG detector returns slightly larger rectangles than the real objects.
        // so we slightly shrink the rectangles to get a nicer output.
        r.x += cvRound(r.width * 0.1);
        r.width = cvRound(r.width * 0.8);
        r.y += cvRound(r.height * 0.07);
        r.height = cvRound(r.height * 0.8);
        cv::rectangle(frame, r.tl(), r.br(), cv::Scalar(0, 255, 0), 3);

        opencv_apps::msg::Rect rect_msg;
        rect_msg.x = r.x;
        rect_msg.y = r.y;
        rect_msg.width = r.width;
        rect_msg.height = r.height;
        found_msg.rects.push_back(rect_msg);
      }

      if (debug_view_)
      {
        cv::imshow(window_name_, frame);
        cv::waitKey(1);
      }

      img_pub_.publish(*cv_bridge::CvImage(msg->header, msg->encoding, frame).toImageMsg());
      msg_pub_->publish(found_msg);
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
          std::bind(&PeopleDetectNode::imageCallbackWithInfo, this, std::placeholders::_1, std::placeholders::_2),
          transport);
    else
      img_sub_ = image_transport::create_subscription(
          this, "image", std::bind(&PeopleDetectNode::imageCallback, this, std::placeholders::_1), transport);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit PeopleDetectNode(const rclcpp::NodeOptions & options) : opencv_apps::Node("people_detect", options)
  {
    queue_size_ = this->declare_parameter("queue_size", 3, describeParam("Queue size for image subscription."));
    debug_view_ = this->declare_parameter(
        "debug_view", false, describeParam("Show a cv::imshow debug window with the detected people."));
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0);
    window_name_ = "people detector";

    this->declare_parameter(
        "use_camera_info", false,
        describeParam("Indicates that the camera_info topic should be subscribed to to get the default "
                       "input_frame_id. Otherwise the frame from the image message will be used."));
    this->declare_parameter(
        "hit_threshold", 0.0,
        describeDoubleParam("Threshold for the distance between features and SVM classifying plane.", 0.0, 10.0));
    this->declare_parameter(
        "win_stride", 8, describeIntParam("Window stride. It must be a multiple of block stride.", 1, 100));
    this->declare_parameter(
        "padding", 32,
        describeIntParam("Mock parameter to keep the CPU interface compatibility. It must be (0,0).", 0, 128));
    this->declare_parameter(
        "scale0", 1.05, describeDoubleParam("Coefficient of the detection window increase.", 1.0, 100.0));
    this->declare_parameter(
        "group_threshold", 2,
        describeIntParam(
            "Coefficient to regulate the similarity threshold. When detected, some objects can be covered by "
            "many rectangles. 0 means not to perform grouping.",
            0, 100));
    applyParams();

    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::RectArrayStamped>("~/found", 1);

    on_set_params_handle_ =
        this->add_on_set_parameters_callback(std::bind(&PeopleDetectNode::onParamChange, this, std::placeholders::_1));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::PeopleDetectNode)
