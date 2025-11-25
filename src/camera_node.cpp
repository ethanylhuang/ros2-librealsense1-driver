#include <memory>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <librealsense/rs.hpp>

class CameraNode : public rclcpp::Node {
public:
  CameraNode()
  : Node("rgbd")
  {
    // intel realsense context object
    ctx_ = std::make_unique<rs::context>();
    if (ctx_->get_device_count() == 0) {
      throw std::runtime_error("No RealSense device detected");
    }

    dev_ = ctx_->get_device(0);
    RCLCPP_INFO(get_logger(), "Using device: %s", dev_->get_name());

    width_ = 640;
    height_ = 480;
    fps_ = 60;

    // capture rgb, depth, and ir streams
    dev_->enable_stream(
      rs::stream::color,
      width_, height_,
      rs::format::rgb8,
      fps_);

    dev_->enable_stream(
      rs::stream::depth,
      width_, height_,
      rs::format::z16,
      fps_
    );

    dev_->enable_stream(
      rs::stream::infrared,
      width_, height_,
      rs::format::y8,
      fps_
    );

    dev_->start();

    depth_intrin_ = dev_->get_stream_intrinsics(rs::stream::depth);

    // ros2 publishers 
    rgb_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "camera/color/image_raw", 10);

    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "camera/depth/image_raw", 10);

    infra_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "camera/infra1/image_raw", 10);

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "camera/depth/points", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&CameraNode::tick, this));
  }

  ~CameraNode() {
    try { dev_->stop(); } catch (...) {}
  }

private:
  void tick() {
    dev_->wait_for_frames();

    const uint8_t *rgb = reinterpret_cast<const uint8_t *>(
      dev_->get_frame_data(rs::stream::color));

    const uint16_t *depth = reinterpret_cast<const uint16_t *>(
      dev_->get_frame_data(rs::stream::depth));

    const uint8_t *infra = reinterpret_cast<const uint8_t *>(
      dev_->get_frame_data(rs::stream::infrared));
    
    auto stamp = now();

    publish_color(stamp, rgb);
    publish_depth(stamp, depth);
    publish_infra(stamp, infra);
    publish_pointcloud(stamp);
  }

  void publish_color(const rclcpp::Time & stamp, const uint8_t *rgb) {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "camera_color_optical_frame";
    msg.height = height_;
    msg.width = width_;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = width_ * 3;

    msg.data.resize(height_ * msg.step);
    std::memcpy(msg.data.data(), rgb, msg.data.size());

    rgb_pub_->publish(msg);
  }

  void publish_depth(const rclcpp::Time & stamp, const uint16_t *depth) {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "camera_depth_optical_frame";
    msg.height = height_;
    msg.width = width_;
    msg.encoding = "16UC1";
    msg.is_bigendian = false;
    msg.step = width_ * sizeof(uint16_t);

    msg.data.resize(height_ * msg.step);
    std::memcpy(msg.data.data(),
                reinterpret_cast<const uint8_t *>(depth),
                msg.data.size());

    depth_pub_->publish(msg);
  }

  void publish_infra(const rclcpp::Time & stamp, const uint8_t *infra) {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "camera_depth_optical_frame";
    msg.height = height_;
    msg.width = width_;
    msg.encoding = "mono8";
    msg.is_bigendian = false;
    msg.step = width_;

    msg.data.resize(height_ * msg.step);
    std::memcpy(msg.data.data(), infra, msg.data.size());

    infra_pub_->publish(msg);
  }

  void publish_pointcloud(const rclcpp::Time & stamp) {
    const rs::float3 *points = reinterpret_cast<const rs::float3 *>(
        dev_->get_frame_data(rs::stream::points));

    int width  = depth_intrin_.width;
    int height = depth_intrin_.height;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = "camera_depth_optical_frame";

    cloud.height = height;
    cloud.width  = width;
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    cloud.fields.resize(3);

    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;

    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;

    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    cloud.point_step = 12;
    cloud.row_step   = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);

    uint8_t *data_ptr = cloud.data.data();

    for (int i = 0; i < width * height; ++i)
    {
        const rs::float3 &p = points[i];
        float *out = reinterpret_cast<float *>(data_ptr + i * cloud.point_step);

        // if (p.z == 0.0f) {
        // out[0] = std::numeric_limits<float>::quiet_NaN();
        // out[1] = std::numeric_limits<float>::quiet_NaN();
        // out[2] = std::numeric_limits<float>::quiet_NaN();
        // } else {
        // out[0] = p.x;
        // out[1] = p.y;
        // out[2] = p.z;
        // }

        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
    }

    cloud_pub_->publish(cloud);
    }

  std::unique_ptr<rs::context> ctx_;
  rs::device * dev_;

  int width_, height_, fps_;

  rs::intrinsics depth_intrin_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr infra_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CameraNode>());
  } catch (const std::exception &e) {
    fprintf(stderr, "FATAL: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
