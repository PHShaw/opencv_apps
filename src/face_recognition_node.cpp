/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Yuki Furuta.
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

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <pwd.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/face.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include "opencv_apps/msg/face.hpp"
#include "opencv_apps/msg/face_array_stamped.hpp"
#include "opencv_apps/node.hpp"
#include "opencv_apps/srv/face_recognition_train.hpp"

namespace fs = std::filesystem;

namespace opencv_apps
{
namespace
{
// Replace a leading "~" or "~user" with the corresponding home directory,
// same behavior as ROS 1's boost::filesystem::user_expanded_path() helper.
fs::path userExpandedPath(const fs::path & p)
{
  if (p.empty())
    return p;
  auto it = p.begin();
  std::string first = it->string();
  if (first.empty() || first[0] != '~')
    return p;

  const char * homedir = nullptr;
  if (first.length() == 1)
  {
    homedir = getenv("HOME");
    if (homedir == nullptr)
    {
      passwd * pw = getpwuid(getuid());
      if (pw != nullptr)
        homedir = pw->pw_dir;
    }
  }
  else
  {
    std::string uname = first.substr(1);
    passwd * pw = getpwnam(uname.c_str());
    if (pw == nullptr)
      return p;
    homedir = pw->pw_dir;
  }
  if (homedir == nullptr)
    return p;

  fs::path ret(homedir);
  ++it;
  for (; it != p.end(); ++it)
  {
    ret /= *it;
  }
  return ret;
}
}  // namespace

class LabelMapper
{
  std::map<std::string, int> m_;

public:
  void add(std::vector<std::string> & l)
  {
    int id = 0;
    for (const auto & kv : m_)
    {
      if (id < kv.second)
        id = kv.second + 1;
    }
    for (const std::string & i : l)
    {
      if (m_.find(i) == m_.end())
      {
        m_[i] = id;
        id++;
      }
    }
  }
  std::vector<int> get(std::vector<std::string> & v)
  {
    std::vector<int> ret(v.size());
    for (size_t i = 0; i < v.size(); ++i)
    {
      ret[i] = m_[v[i]];
    }
    return ret;
  }
  std::string lookup(int id)
  {
    for (const auto & kv : m_)
    {
      if (kv.second == id)
        return kv.first;
    }
    return "nan";
  }
  const std::map<std::string, int> & getMap() const
  {
    return m_;
  }
};

class Storage
{
  fs::path base_dir_;
  rclcpp::Logger logger_;

public:
  Storage(const fs::path & base_dir, const rclcpp::Logger & logger) : logger_(logger)
  {
    base_dir_ = userExpandedPath(base_dir);
    if (!fs::exists(base_dir_))
    {
      init();
    }
    if (!fs::is_directory(base_dir_))
    {
      std::stringstream ss;
      ss << base_dir_ << " is not a directory";
      throw std::runtime_error(ss.str());
    }
  }

  void init()
  {
    if (!fs::create_directories(base_dir_))
    {
      std::stringstream ss;
      ss << "failed to initialize directory: " << base_dir_;
      throw std::runtime_error(ss.str());
    }
  }

  void load(std::vector<cv::Mat> & images, std::vector<std::string> & labels, bool append = true)
  {
    if (!append)
    {
      images.clear();
      labels.clear();
    }
    for (const auto & entry : fs::directory_iterator(base_dir_))
    {
      if (entry.is_directory())
      {
        std::string label = entry.path().stem().string();
        for (const auto & centry : fs::directory_iterator(entry.path()))
        {
          if (centry.is_directory())
            continue;
          fs::path file_path = centry.path();
          try
          {
            cv::Mat img = cv::imread(file_path.string(), cv::IMREAD_COLOR);
            labels.push_back(label);
            images.push_back(img);
          }
          catch (cv::Exception & e)
          {
            RCLCPP_ERROR_STREAM(logger_, "Error: load image from " << file_path << ": " << e.what());
          }
        }
      }
    }
  }

  void save(const std::vector<cv::Mat> & images, const std::vector<std::string> & labels)
  {
    if (images.size() != labels.size())
    {
      throw std::length_error("images.size() != labels.size()");
    }
    for (size_t i = 0; i < images.size(); ++i)
    {
      save(images[i], labels[i]);
    }
  }

  void save(const cv::Mat & image, const std::string & label)
  {
    fs::path img_dir = base_dir_ / fs::path(label);
    if (!fs::exists(img_dir) && !fs::create_directories(img_dir))
    {
      std::stringstream ss;
      ss << "failed to initialize directory: " << img_dir;
      throw std::runtime_error(ss.str());
    }
    int file_count = 0;
    for (const auto & entry : fs::directory_iterator(img_dir))
    {
      if (!entry.is_directory())
        file_count++;
    }

    std::stringstream ss;
    ss << label << "_" << std::setw(6) << std::setfill('0') << file_count << ".jpg";
    fs::path file_path = img_dir / ss.str();
    RCLCPP_INFO_STREAM(logger_, "saving image to :" << file_path);
    try
    {
      cv::imwrite(file_path.string(), image);
    }
    catch (cv::Exception & e)
    {
      RCLCPP_ERROR_STREAM(logger_, "Error: save image to " << file_path << ": " << e.what());
    }
  }
};

class FaceRecognitionNode : public opencv_apps::Node
{
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, opencv_apps::msg::FaceArrayStamped>
      SyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, opencv_apps::msg::FaceArrayStamped>
      ApproximateSyncPolicy;

  image_transport::SubscriberFilter img_sub_;
  message_filters::Subscriber<opencv_apps::msg::FaceArrayStamped> face_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> async_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
  rclcpp::Publisher<opencv_apps::msg::FaceArrayStamped>::SharedPtr face_pub_;
  rclcpp::Service<opencv_apps::srv::FaceRecognitionTrain>::SharedPtr train_srv_;

  bool use_async_;
  int queue_size_;

  // Cached parameter values (mirrors ROS 1's dynamic_reconfigure Config).
  std::string model_method_;
  bool use_saved_data_;
  bool save_train_data_;
  std::string data_dir_;
  int face_model_width_;
  int face_model_height_;
  double face_padding_;
  int model_num_components_;
  double model_threshold_;
  int lbph_radius_, lbph_neighbors_, lbph_grid_x_, lbph_grid_y_;

  std::mutex mutex_;

  std::shared_ptr<LabelMapper> label_mapper_;
  std::shared_ptr<Storage> storage_;
  cv::Size face_model_size_;
  cv::Ptr<cv::face::FaceRecognizer> model_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handle_;

  void drawFace(cv::Mat & img, const opencv_apps::msg::Face & face)
  {
    cv::Rect r(int(face.face.x - face.face.width / 2.0 - face.face.width * face_padding_ / 2.0),
               int(face.face.y - face.face.height / 2.0 - face.face.height * face_padding_ / 2.0),
               int(face.face.width + face.face.width * face_padding_),
               int(face.face.height + face.face.height * face_padding_));
    cv::Scalar color(0.0, 0.0, 255.0);
    int boldness = 2;
    cv::rectangle(img, r.tl(), r.br(), color, boldness, cv::LINE_AA);

    double font_scale = 1.5;
    int text_height = 20;
    cv::Point text_bl;
    if (r.br().y + text_height > img.rows)
      text_bl = r.tl() + cv::Point(0, -text_height);
    else
      text_bl = r.br() + cv::Point(-r.width, text_height);
    std::stringstream ss;
    ss << face.label << " (" << std::fixed << std::setprecision(2) << face.confidence << ")";
    cv::putText(img, ss.str(), text_bl, cv::FONT_HERSHEY_PLAIN, font_scale, color, boldness, cv::LINE_AA);
  }

  void extractImage(const cv::Mat & img, const opencv_apps::msg::Rect & rect, cv::Mat & ret, double padding = 0.0)
  {
    int x = std::max(0, int(rect.x - rect.width / 2.0 - rect.width * padding / 2.0));
    int y = std::max(0, int(rect.y - rect.height / 2.0 - rect.height * padding / 2.0));
    cv::Rect r(x, y, std::min(img.cols - x, int(rect.width + rect.width * padding)),
               std::min(img.rows - y, int(rect.height + rect.height * padding)));
    ret = img(r);
  }

  void extractImage(const cv::Mat & img, const opencv_apps::msg::Face & face, cv::Mat & ret, double padding = 0.0)
  {
    extractImage(img, face.face, ret, padding);
  }

  void createModel()
  {
    if (model_method_ == "eigen")
      model_ = cv::face::EigenFaceRecognizer::create(model_num_components_, model_threshold_);
    else if (model_method_ == "fisher")
      model_ = cv::face::FisherFaceRecognizer::create(model_num_components_, model_threshold_);
    else if (model_method_ == "LBPH")
      model_ = cv::face::LBPHFaceRecognizer::create(lbph_radius_, lbph_neighbors_, lbph_grid_x_, lbph_grid_y_);
  }

  void retrain()
  {
    RCLCPP_DEBUG(this->get_logger(), "retrain");
    std::vector<cv::Mat> images;
    std::vector<std::string> labels;
    train(images, labels);
  }

  void train(std::vector<cv::Mat> & images, std::vector<std::string> & labels)
  {
    size_t new_image_size = images.size();

    if (use_saved_data_)
    {
      storage_->load(images, labels);
    }

    if (images.empty())
      return;

    std::vector<cv::Mat> resized_images(images.size());
    for (size_t i = 0; i < images.size(); ++i)
    {
      cv::resize(images[i], resized_images[i], face_model_size_, 0, 0, cv::INTER_CUBIC);
      cv::cvtColor(resized_images[i], resized_images[i], cv::COLOR_BGR2GRAY);
    }

    label_mapper_->add(labels);
    std::vector<int> ids = label_mapper_->get(labels);
    RCLCPP_INFO_STREAM(this->get_logger(), "training with " << images.size() << " images");
    model_->train(resized_images, ids);

    if (save_train_data_ && new_image_size > 0)
    {
      std::vector<cv::Mat> new_images(images.begin(), images.begin() + new_image_size);
      std::vector<std::string> new_labels(labels.begin(), labels.begin() + new_image_size);
      storage_->save(new_images, new_labels);
    }
  }

  void predict(const cv::Mat & img, int & label, double & confidence)
  {
    cv::Mat resized_img;
    cv::resize(img, resized_img, face_model_size_, 0, 0, cv::INTER_CUBIC);
    cv::cvtColor(resized_img, resized_img, cv::COLOR_BGR2GRAY);
    model_->predict(resized_img, label, confidence);
  }

  void faceImageCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr & image,
      const opencv_apps::msg::FaceArrayStamped::ConstSharedPtr & faces)
  {
    RCLCPP_DEBUG(this->get_logger(), "faceImageCallback");
    std::lock_guard<std::mutex> lock(mutex_);

    if (label_mapper_->getMap().empty())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Face data is not trained. Please train first.");
      return;
    }

    bool publish_debug_image = debug_img_pub_->get_subscription_count() > 0;

    try
    {
      cv::Mat cv_img = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8)->image;
      opencv_apps::msg::FaceArrayStamped ret_msg = *faces;
      for (size_t i = 0; i < faces->faces.size(); ++i)
      {
        cv::Mat face_img;
        extractImage(cv_img, faces->faces[i], face_img, face_padding_);

        int label_id = -1;
        double confidence = 0.0;
        predict(face_img, label_id, confidence);
        if (label_id == -1)
          continue;
        ret_msg.faces[i].label = label_mapper_->lookup(label_id);
        ret_msg.faces[i].confidence = confidence;

        if (publish_debug_image)
          drawFace(cv_img, ret_msg.faces[i]);
      }
      face_pub_->publish(ret_msg);
      if (publish_debug_image)
      {
        debug_img_pub_->publish(*cv_bridge::CvImage(image->header, sensor_msgs::image_encodings::BGR8, cv_img).toImageMsg());
        RCLCPP_DEBUG(this->get_logger(), "Published debug image");
      }
    }
    catch (cv::Exception & e)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "error at image processing: " << e.err << " " << e.func << " "
                                                                              << e.file << " " << e.line);
    }
  }

  void trainCallback(
      const std::shared_ptr<opencv_apps::srv::FaceRecognitionTrain::Request> req,
      std::shared_ptr<opencv_apps::srv::FaceRecognitionTrain::Response> res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
      std::vector<cv::Mat> images(req->images.size());
      bool use_roi = !req->rects.empty();

      if (use_roi && req->images.size() != req->rects.size())
      {
        throw std::length_error("req->images.size() != req->rects.size()");
      }

      for (size_t i = 0; i < req->images.size(); ++i)
      {
        images[i] = cv_bridge::toCvCopy(req->images[i], sensor_msgs::image_encodings::BGR8)->image;
        if (use_roi)
        {
          cv::Mat face_img;
          extractImage(images[i], req->rects[i], face_img);
          images[i] = face_img;
        }
      }
      std::vector<std::string> labels(req->labels.begin(), req->labels.end());
      train(images, labels);
      res->ok = true;
    }
    catch (cv::Exception & e)
    {
      std::stringstream ss;
      ss << "error at training: " << e.err << " " << e.func << " " << e.file << " " << e.line;
      res->ok = false;
      res->error = ss.str();
    }
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> & params)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool need_recreate_model = false;
    bool need_retrain = false;
    bool threshold_changed = false;

    for (const auto & p : params)
    {
      const std::string & name = p.get_name();
      if (name == "model_method")
      {
        std::string v = p.as_string();
        if (v != "eigen" && v != "fisher" && v != "LBPH")
        {
          result.successful = false;
          result.reason = "model_method must be one of: eigen, fisher, LBPH";
          continue;
        }
        if (v != model_method_)
        {
          model_method_ = v;
          need_recreate_model = true;
        }
      }
      else if (name == "use_saved_data")
        use_saved_data_ = p.as_bool();
      else if (name == "save_train_data")
        save_train_data_ = p.as_bool();
      else if (name == "face_padding")
        face_padding_ = p.as_double();
      else if (name == "face_model_width")
      {
        int v = static_cast<int>(p.as_int());
        if (v != face_model_width_)
        {
          face_model_width_ = v;
          need_retrain = true;
        }
      }
      else if (name == "face_model_height")
      {
        int v = static_cast<int>(p.as_int());
        if (v != face_model_height_)
        {
          face_model_height_ = v;
          need_retrain = true;
        }
      }
      else if (name == "data_dir")
      {
        std::string v = p.as_string();
        if (v != data_dir_)
        {
          data_dir_ = v;
          need_retrain = true;
          label_mapper_ = std::make_shared<LabelMapper>();
          storage_ = std::make_shared<Storage>(fs::path(data_dir_), this->get_logger());
        }
      }
      else if (name == "model_num_components")
      {
        int v = static_cast<int>(p.as_int());
        if (v != model_num_components_)
        {
          model_num_components_ = v;
          need_recreate_model = true;
        }
      }
      else if (name == "model_threshold")
      {
        double v = p.as_double();
        if (v != model_threshold_)
        {
          model_threshold_ = v;
          threshold_changed = true;
        }
      }
      else if (name == "lbph_radius")
      {
        int v = static_cast<int>(p.as_int());
        if (v != lbph_radius_)
        {
          lbph_radius_ = v;
          need_recreate_model = true;
        }
      }
      else if (name == "lbph_neighbors")
      {
        int v = static_cast<int>(p.as_int());
        if (v != lbph_neighbors_)
        {
          lbph_neighbors_ = v;
          need_recreate_model = true;
        }
      }
      else if (name == "lbph_grid_x")
      {
        int v = static_cast<int>(p.as_int());
        if (v != lbph_grid_x_)
        {
          lbph_grid_x_ = v;
          need_recreate_model = true;
        }
      }
      else if (name == "lbph_grid_y")
      {
        int v = static_cast<int>(p.as_int());
        if (v != lbph_grid_y_)
        {
          lbph_grid_y_ = v;
          need_recreate_model = true;
        }
      }
    }

    if (!result.successful)
      return result;

    face_model_size_ = cv::Size(face_model_width_, face_model_height_);

    if (need_recreate_model)
    {
      try
      {
        createModel();
        need_retrain = true;
      }
      catch (cv::Exception & e)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error: create face recognizer: " << e.what());
      }
    }

    if (need_retrain)
    {
      try
      {
        retrain();
      }
      catch (cv::Exception & e)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error: train: " << e.what());
      }
    }
    else if (threshold_changed && model_)
    {
      try
      {
        model_->setThreshold(model_threshold_);
      }
      catch (cv::Exception & e)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error: set threshold: " << e.what());
      }
    }

    return result;
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "subscribe");
    std::string transport = image_transport::TransportHints(this).getTransport();
    img_sub_.subscribe(this, "image", transport);
    face_sub_.subscribe(this, "faces");
    if (use_async_)
    {
      async_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
          ApproximateSyncPolicy(queue_size_));
      async_->connectInput(img_sub_, face_sub_);
      async_->registerCallback(
          std::bind(&FaceRecognitionNode::faceImageCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
    else
    {
      sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(queue_size_));
      sync_->connectInput(img_sub_, face_sub_);
      sync_->registerCallback(
          std::bind(&FaceRecognitionNode::faceImageCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(this->get_logger(), "unsubscribe");
    img_sub_.unsubscribe();
    face_sub_.unsubscribe();
  }

public:
  explicit FaceRecognitionNode(const rclcpp::NodeOptions & options) : opencv_apps::Node("face_recognition", options)
  {
    face_model_size_ = cv::Size(190, 90);

    use_async_ = this->declare_parameter(
        "approximate_sync", false, describeParam("Use approximate-time instead of exact-time synchronization."));
    queue_size_ = this->declare_parameter("queue_size", 100, describeParam("Synchronizer queue size."));

    model_method_ = this->declare_parameter(
        "model_method", std::string("eigen"),
        describeParam("Method to recognize faces: one of eigen, fisher, LBPH"));
    use_saved_data_ = this->declare_parameter("use_saved_data", true, describeParam("Use saved data"));
    save_train_data_ = this->declare_parameter("save_train_data", true, describeParam("Save train data"));
    data_dir_ = this->declare_parameter(
        "data_dir", std::string("~/.ros/opencv_apps/face_data"),
        describeParam("Save directory for train data"));
    face_model_width_ = this->declare_parameter(
        "face_model_width", 190, describeIntParam("Width of training face image", 30, 500));
    face_model_height_ = this->declare_parameter(
        "face_model_height", 90, describeIntParam("Height of training face image", 30, 500));
    face_padding_ =
        this->declare_parameter("face_padding", 0.1, describeDoubleParam("Padding ratio of each face", 0.0, 2.0));
    model_num_components_ = this->declare_parameter(
        "model_num_components", 0, describeIntParam("Number of components for face recognizer model", 0, 100));
    model_threshold_ = this->declare_parameter(
        "model_threshold", 8000.0, describeDoubleParam("Threshold for face recognizer model", 0.0, 10000.0));
    lbph_radius_ = this->declare_parameter(
        "lbph_radius", 1, describeIntParam("Radius parameter used only for LBPH model", 1, 10));
    lbph_neighbors_ = this->declare_parameter(
        "lbph_neighbors", 8, describeIntParam("Neighbors parameter used only for LBPH model", 1, 30));
    lbph_grid_x_ = this->declare_parameter(
        "lbph_grid_x", 8, describeIntParam("grid_x parameter used only for LBPH model", 1, 30));
    lbph_grid_y_ = this->declare_parameter(
        "lbph_grid_y", 8, describeIntParam("grid_y parameter used only for LBPH model", 1, 30));

    face_model_size_ = cv::Size(face_model_width_, face_model_height_);
    label_mapper_ = std::make_shared<LabelMapper>();
    storage_ = std::make_shared<Storage>(fs::path(data_dir_), this->get_logger());
    createModel();
    try
    {
      retrain();
    }
    catch (cv::Exception & e)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error: train: " << e.what());
    }

    debug_img_pub_ = advertise<sensor_msgs::msg::Image>("~/debug_image", 1);
    face_pub_ = advertise<opencv_apps::msg::FaceArrayStamped>("~/output", 1);
    train_srv_ = this->create_service<opencv_apps::srv::FaceRecognitionTrain>(
        "~/train", std::bind(&FaceRecognitionNode::trainCallback, this, std::placeholders::_1, std::placeholders::_2));

    on_set_params_handle_ = this->add_on_set_parameters_callback(
        std::bind(&FaceRecognitionNode::onParamChange, this, std::placeholders::_1));

    onInitPostProcess();
  }

  ~FaceRecognitionNode() override
  {
    sync_.reset();
    async_.reset();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::FaceRecognitionNode)
