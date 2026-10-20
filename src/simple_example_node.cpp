/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Tal Regev.
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
// http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
/**
 * This is a demo of Simple Example from wiki tutorial
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

namespace opencv_apps
{
namespace
{
const char kOpenCvWindow[] = "Image window";
}  // namespace

// This is a minimal always-on demo (not lazy-subscribe like the rest of the
// package's nodes), matching the ROS 1 cv_bridge tutorial it's based on.
class SimpleExampleNode : public rclcpp::Node
{
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  bool debug_view_;

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (cv_ptr->image.rows > 110 && cv_ptr->image.cols > 110)
      cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols / 2, cv_ptr->image.rows / 2), 100,
                 cv::Scalar(0, 0, 255));

    if (debug_view_)
    {
      cv::imshow(kOpenCvWindow, cv_ptr->image);
      cv::waitKey(3);
    }

    image_pub_.publish(*cv_ptr->toImageMsg());
  }

public:
  explicit SimpleExampleNode(const rclcpp::NodeOptions & options) : rclcpp::Node("simple_example", options)
  {
    int queue_size = this->declare_parameter("queue_size", 1);
    debug_view_ = this->declare_parameter("debug_view", false);
    if (debug_view_)
    {
      cv::namedWindow(kOpenCvWindow);
    }

    std::string transport = image_transport::TransportHints(this).getTransport();
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = static_cast<size_t>(queue_size);
    image_sub_ = image_transport::create_subscription(
        this, "image", std::bind(&SimpleExampleNode::imageCb, this, std::placeholders::_1), transport, qos);
    image_pub_ = image_transport::create_publisher(this, "/image_converter/output_video/raw");
  }

  ~SimpleExampleNode() override
  {
    if (debug_view_)
    {
      cv::destroyWindow(kOpenCvWindow);
    }
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::SimpleExampleNode)
