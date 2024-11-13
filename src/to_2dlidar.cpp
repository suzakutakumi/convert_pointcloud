#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>

#include <chrono>
#include <iostream>
#include <sstream>

std::string getTimestamp()
{
  // 現在のUnixタイム（エポック秒）を取得
  const auto &now = std::chrono::system_clock::now();
  const auto &currentTime = std::chrono::system_clock::to_time_t(now);

  // Unixタイムを文字列に変換
  std::stringstream ss;
  ss << currentTime;

  return ss.str();
}

using std::placeholders::_1;

class To2DLiDARNode : public rclcpp::Node
{
public:
  To2DLiDARNode()
      : Node("pubsub_pointcloud_node")
  {
    this->declare_parameter("split_thresholds", std::vector<double>{0});
    this->declare_parameter("robot_tilt", std::vector<double>{0, 0, 0});

    split_thresholds = get_parameter("split_thresholds").as_double_array();

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud", 10, std::bind(&To2DLiDARNode::subscriber_callback, this, _1));

    std::string publisher_basename = "lidar/data";
    publishers.push_back(create_publisher<sensor_msgs::msg::PointCloud2>(publisher_basename + std::to_string(0), 10));
    for (size_t i = 0; i < split_thresholds.size(); i++)
    {
      publishers.push_back(create_publisher<sensor_msgs::msg::PointCloud2>(publisher_basename + std::to_string(i + 1), 10));
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> publishers;

  std::vector<double> split_thresholds;

  const float ThresholdLowest = std::numeric_limits<float>::lowest();
  const float ThresholdMax = std::numeric_limits<float>::max();

  void subscriber_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "subscribe pointcloud(size: %d)", msg->data.size());

    // PCLに変換
    RCLCPP_INFO(this->get_logger(), "change to pcl");
    pcl::PointCloud<pcl::PointXYZRGB> original_cloud;
    pcl::fromROSMsg(*msg, original_cloud);

    // 傾きの補正
    RCLCPP_DEBUG(this->get_logger(), "correct the tilt");
    auto robot_tilt = get_parameter("robot_tilt").as_double_array();
    std::cout << robot_tilt[0] << "," << robot_tilt[1] << "," << robot_tilt[2] << std::endl;

    auto roll = robot_tilt[0] * M_PI / 180;
    auto pitch = robot_tilt[1] * M_PI / 180;
    auto yaw = robot_tilt[2] * M_PI / 180;

    // カメラの回転と同じにすることに注意
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(original_cloud, *transformed_cloud, rotation_y_matrix(yaw) * rotation_z_matrix(-pitch) * rotation_x_matrix(roll));

    // 分割してROSのMSGに変換
    std::vector<sensor_msgs::msg::PointCloud2> new_msgs(publishers.size());

    RCLCPP_INFO(this->get_logger(), "generate first layer by spliting(%f ~ %f)", ThresholdLowest, split_thresholds[0]);
    auto splited = split_pointcloud(*transformed_cloud, ThresholdLowest, split_thresholds[0]);
    RCLCPP_INFO(this->get_logger(), "first layer to ros msg");
    pcl::toROSMsg(*splited, new_msgs[0]);

    RCLCPP_INFO(this->get_logger(), "generate some layer by spliting");
    for (size_t i = 0; i < split_thresholds.size() - 1; i++)
    {
      const auto &min_th = split_thresholds[i];
      const auto &max_th = split_thresholds[i + 1];

      auto splited = split_pointcloud(*transformed_cloud, min_th, max_th);
      pcl::toROSMsg(*splited, new_msgs[i + 1]);
    }

    RCLCPP_INFO(this->get_logger(), "generate last layer by spliting");
    splited = split_pointcloud(*transformed_cloud, split_thresholds.back(), ThresholdMax);
    pcl::toROSMsg(*splited, new_msgs.back());

    RCLCPP_INFO(this->get_logger(), "publish");
    // headerを追加してpublish
    for (size_t i = 0; i < publishers.size(); i++)
    {
      new_msgs[i].header = msg->header;
      publishers[i]->publish(new_msgs[i]);
    }
  }

  static Eigen::Matrix4f rotation_x_matrix(const double &theta)
  {
    auto c = cos(theta), s = sin(theta);
    Eigen::Matrix4f rotation_matrix;
    rotation_matrix << 1, 0, 0, 0,
        0, c, -s, 0,
        0, s, c, 0,
        0, 0, 0, 1;

    return rotation_matrix;
  }

  static Eigen::Matrix4f rotation_y_matrix(const double &theta)
  {
    auto c = cos(theta), s = sin(theta);
    Eigen::Matrix4f rotation_matrix;
    rotation_matrix << c, 0, s, 0,
        0, 1, 0, 0,
        -s, 0, c, 0,
        0, 0, 0, 1;

    return rotation_matrix;
  }

  static Eigen::Matrix4f rotation_z_matrix(const double &theta)
  {
    auto c = cos(theta), s = sin(theta);
    Eigen::Matrix4f rotation_matrix;
    rotation_matrix << c, -s, 0, 0,
        s, c, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return rotation_matrix;
  }

  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr split_pointcloud(const pcl::PointCloud<pcl::PointXYZRGB> &origin, const float &min_value, const float &max_value)
  {
    pcl::PointCloud<pcl::PointXYZRGB> splited;
    pcl::PassThrough<pcl::PointXYZRGB> cloud_filter;
    cloud_filter.setInputCloud(origin.makeShared());
    cloud_filter.setFilterFieldName("y");
    cloud_filter.setFilterLimits(min_value, max_value);
    cloud_filter.filter(splited);

    return splited.makeShared();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<To2DLiDARNode>());
  rclcpp::shutdown();

  return 0;
}