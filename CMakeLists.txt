cmake_minimum_required(VERSION 3.8)
project(opencv_apps)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ---------------------------------------------------------------- interfaces
# All .msg/.srv definitions are unchanged from the ROS 1 package (they already
# use package-qualified std_msgs/Header and only float64/string/nested-custom
# fields, so no field-level rework was needed for the ROS 2 IDL).
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point2D.msg"
  "msg/Point2DStamped.msg"
  "msg/Point2DArray.msg"
  "msg/Point2DArrayStamped.msg"
  "msg/Rect.msg"
  "msg/RectArray.msg"
  "msg/RectArrayStamped.msg"
  "msg/Flow.msg"
  "msg/FlowStamped.msg"
  "msg/FlowArray.msg"
  "msg/FlowArrayStamped.msg"
  "msg/Size.msg"
  "msg/Face.msg"
  "msg/FaceArray.msg"
  "msg/FaceArrayStamped.msg"
  "msg/Line.msg"
  "msg/LineArray.msg"
  "msg/LineArrayStamped.msg"
  "msg/RotatedRect.msg"
  "msg/RotatedRectStamped.msg"
  "msg/RotatedRectArray.msg"
  "msg/RotatedRectArrayStamped.msg"
  "msg/Circle.msg"
  "msg/CircleArray.msg"
  "msg/CircleArrayStamped.msg"
  "msg/Moment.msg"
  "msg/MomentArray.msg"
  "msg/MomentArrayStamped.msg"
  "msg/Contour.msg"
  "msg/ContourArray.msg"
  "msg/ContourArrayStamped.msg"
  "srv/FaceRecognitionTrain.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

# ------------------------------------------------------------------ library
# All opencv_apps::<X>Node component classes are compiled into this single
# shared library (mirrors the ROS 1 package's single libopencv_apps.so
# holding every nodelet plugin), each registered with
# rclcpp_components_register_node() below.

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_component SHARED
  src/node.cpp
  src/edge_detection_node.cpp
  src/color_filter_node.cpp
  src/camshift_node.cpp
  src/adding_images_node.cpp
  src/smoothing_node.cpp
  src/morphology_node.cpp
  src/pyramids_node.cpp
  src/threshold_node.cpp
  src/equalize_histogram_node.cpp
  src/discrete_fourier_transform_node.cpp
  src/simple_example_node.cpp
  src/simple_compressed_example_node.cpp
  src/people_detect_node.cpp
  src/face_detection_node.cpp
  src/face_recognition_node.cpp
  src/corner_harris_node.cpp
  src/goodfeature_track_node.cpp
  src/find_contours_node.cpp
  src/convex_hull_node.cpp
  src/general_contours_node.cpp
  src/contour_moments_node.cpp
  src/hough_lines_node.cpp
  src/hough_circles_node.cpp
  src/lk_flow_node.cpp
  src/fback_flow_node.cpp
  src/simple_flow_node.cpp
  src/phase_corr_node.cpp
  src/segment_objects_node.cpp
  src/watershed_segmentation_node.cpp
)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_component
  "${cpp_typesupport_target}"
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  message_filters
  sensor_msgs
  std_msgs
  std_srvs
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::EdgeDetectionNode"
  EXECUTABLE edge_detection
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::RGBColorFilterNode"
  EXECUTABLE rgb_color_filter
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::HLSColorFilterNode"
  EXECUTABLE hls_color_filter
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::HSVColorFilterNode"
  EXECUTABLE hsv_color_filter
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::LabColorFilterNode"
  EXECUTABLE lab_color_filter
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::CamShiftNode"
  EXECUTABLE camshift
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::AddingImagesNode"
  EXECUTABLE adding_images
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::SmoothingNode"
  EXECUTABLE smoothing
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::MorphologyNode"
  EXECUTABLE morphology
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::PyramidsNode"
  EXECUTABLE pyramids
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::ThresholdNode"
  EXECUTABLE threshold
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::EqualizeHistogramNode"
  EXECUTABLE equalize_histogram
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::DiscreteFourierTransformNode"
  EXECUTABLE discrete_fourier_transform
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::SimpleExampleNode"
  EXECUTABLE simple_example
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::SimpleCompressedExampleNode"
  EXECUTABLE simple_compressed_example
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::PeopleDetectNode"
  EXECUTABLE people_detect
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::FaceDetectionNode"
  EXECUTABLE face_detection
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::FaceRecognitionNode"
  EXECUTABLE face_recognition
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::CornerHarrisNode"
  EXECUTABLE corner_harris
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::GoodfeatureTrackNode"
  EXECUTABLE goodfeature_track
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::FindContoursNode"
  EXECUTABLE find_contours
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::ConvexHullNode"
  EXECUTABLE convex_hull
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::GeneralContoursNode"
  EXECUTABLE general_contours
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::ContourMomentsNode"
  EXECUTABLE contour_moments
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::HoughLinesNode"
  EXECUTABLE hough_lines
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::HoughCirclesNode"
  EXECUTABLE hough_circles
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::LKFlowNode"
  EXECUTABLE lk_flow
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::FBackFlowNode"
  EXECUTABLE fback_flow
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::SimpleFlowNode"
  EXECUTABLE simple_flow
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::PhaseCorrNode"
  EXECUTABLE phase_corr
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::SegmentObjectsNode"
  EXECUTABLE segment_objects
)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "opencv_apps::WatershedSegmentationNode"
  EXECUTABLE watershed_segmentation
)
# ... (one rclcpp_components_register_node call per ported node, added incrementally)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/face_recognition_trainer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_component)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime rclcpp rclcpp_components cv_bridge image_transport message_filters sensor_msgs std_msgs std_srvs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  # A handful of representative launch_testing smoke tests, not a full port of
  # the ROS 1 rostest suite under test/*.test (see plan notes: that suite
  # depends on ROS 1 bags and hztest, neither of which exist in ROS 2).
  add_launch_test(test/test_launch_edge_detection.py)
  add_launch_test(test/test_launch_find_contours.py)
  add_launch_test(test/test_launch_camshift.py)
endif()

ament_package()
