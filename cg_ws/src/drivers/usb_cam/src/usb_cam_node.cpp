#include "usb_cam/usb_cam_node.hpp"

#include <sstream>
// #include <std_srvs/srv/Empty.h>

#include <string>
#include <memory>


namespace usb_cam
{

UsbCamNode::UsbCamNode(const rclcpp::NodeOptions & node_options)
: Node("usb_cam", node_options),
  img_(new sensor_msgs::msg::Image()),
  image_pub_(std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(this, "image_raw", rclcpp::QoS{100}.get_rmw_qos_profile()))),
  service_capture_(
    this->create_service<std_srvs::srv::SetBool>(
      "set_capture",
      std::bind(
        &UsbCamNode::service_capture,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3)))
{
  // declare params
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("framerate", 60.0);
  this->declare_parameter("frame_id", "camera");
  this->declare_parameter("image_height", 720);
  this->declare_parameter("image_width", 1280);
  this->declare_parameter("io_method", "mmap");
  this->declare_parameter("pixel_format", "yuyv");
  this->declare_parameter("video_device", "/dev/video2");

  get_params();
  init();
}

UsbCamNode::~UsbCamNode()
{
  RCLCPP_WARN(this->get_logger(), "shutting down");
  cam_.shutdown();
}

void UsbCamNode::service_capture(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void) request_header;
  if (request->data) {
    cam_.start_capturing();
    response->message = "Start Capturing";
  } else {
    cam_.stop_capturing();
    response->message = "Stop Capturing";
  }
}

void UsbCamNode::init()
{
  while (frame_id_ == "") {
    RCLCPP_WARN_ONCE(
      this->get_logger(), "Required Parameters not set...waiting until they are set");
    get_params();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // load the camera info
  cinfo_.reset(new camera_info_manager::CameraInfoManager(this, camera_name_, camera_info_url_));
  // check for default camera info
  if (!cinfo_->isCalibrated()) {
    cinfo_->setCameraName(video_device_name_);
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header.frame_id = img_->header.frame_id;
    camera_info.width = image_width_;
    camera_info.height = image_height_;
    cinfo_->setCameraInfo(camera_info);
  }

  img_->header.frame_id = frame_id_;
  RCLCPP_INFO(
    this->get_logger(), "Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS",
    camera_name_.c_str(), video_device_name_.c_str(),
    image_width_, image_height_, io_method_name_.c_str(),
    pixel_format_name_.c_str(), framerate_);
  // set the IO method
  UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
  if (io_method == UsbCam::IO_METHOD_UNKNOWN) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Unknown IO method '%s'", io_method_name_.c_str());
    rclcpp::shutdown();
    return;
  }
  // set the pixel format
  UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
  if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Unknown pixel format '%s'", pixel_format_name_.c_str());
    rclcpp::shutdown();
    return;
  }
  // start the camera
  cam_.start(
    video_device_name_.c_str(), io_method, pixel_format, image_width_,
    image_height_, framerate_);
  cam_.get_formats();

  const int period_ms = 1000.0 / framerate_;
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
    std::bind(&UsbCamNode::update, this));
  RCLCPP_INFO_STREAM(this->get_logger(), "starting timer " << period_ms);

  // Do calculation beforehand
  dim.height = image_height_;
  dim.width = image_width_;
  cv::fisheye::initUndistortRectifyMap(K, D, R, K, dim, CV_16SC2, map1, map2);
  out_msg.encoding = "rgb8"; // Same type as maps
}

void UsbCamNode::get_params()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);

  for (auto & parameter : parameters_client->get_parameters(
      {"camera_name", "camera_info_url", "frame_id", "framerate",
        "image_height", "image_width", "io_method", "pixel_format", "video_device"}))
  {
    if (parameter.get_name() == "camera_name") {
      RCLCPP_INFO(this->get_logger(), "camera_name value: %s", parameter.value_to_string().c_str());
      camera_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "camera_info_url") {
      camera_info_url_ = parameter.value_to_string();
    } else if (parameter.get_name() == "frame_id") {
      frame_id_ = parameter.value_to_string();
    } else if (parameter.get_name() == "framerate") {
      RCLCPP_WARN(this->get_logger(), "framerate: %f", parameter.as_double());
      framerate_ = parameter.as_double();
    } else if (parameter.get_name() == "image_height") {
      image_height_ = parameter.as_int();
    } else if (parameter.get_name() == "image_width") {
      image_width_ = parameter.as_int();
    } else if (parameter.get_name() == "io_method") {
      io_method_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "pixel_format") {
      pixel_format_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "video_device") {
      video_device_name_ = parameter.value_to_string();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid parameter name: %s", parameter.get_name().c_str());
    }
  }
}

bool UsbCamNode::take_and_send_image()
{
  // grab the image
  if (!cam_.get_image(
      img_->header.stamp, img_->encoding, img_->height, img_->width,
      img_->step, img_->data))
  {
    RCLCPP_ERROR(this->get_logger(), "grab failed");
    return false;
  }
  // Copy the input message to a cv pointer with RGB 8-bit encoding
  cv_ptr = cv_bridge::toCvCopy(*img_, sensor_msgs::image_encodings::RGB8);

  // Undistort the image using fisheye model
  cv::remap(cv_ptr->image, out_msg.image, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

  // Create the output image
  out_msg.header = img_->header; // Same timestamp and tf frame as input image
  
  // Get output image message - dereferenced
  img_->data = out_msg.toImageMsg()->data;

  auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
  ci->header = img_->header;
  image_pub_->publish(*img_, *ci);
  return true;
}

void UsbCamNode::update()
{
  if (cam_.is_capturing()) {
    // If the camera exposure longer higher than the framerate period
    // then that caps the framerate.
    // auto t0 = now();
    if (!take_and_send_image()) {
      RCLCPP_WARN(this->get_logger(), "USB camera did not respond in time.");
    }
    // auto diff = now() - t0;
    // INFO(diff.nanoseconds() / 1e6 << " " << int(t0.nanoseconds() / 1e9));
  }
}
}  // namespace usb_cam


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_cam::UsbCamNode)
