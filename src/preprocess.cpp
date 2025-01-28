#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>

#include <Eigen/Dense>

#include <chrono>
#include <iostream>

using std::placeholders::_1;

class PreprocessNode : public rclcpp::Node
{
public:
  PreprocessNode()
      : Node("preprocess_node")
  {
    this->declare_parameter("split_thresholds", std::vector<double>{0});
    this->declare_parameter("robot_tilt", std::vector<double>{0, 0, 0});

    split_thresholds = get_parameter("split_thresholds").as_double_array();
    split_thresholds.insert(split_thresholds.begin(), ThresholdLowest);
    split_thresholds.push_back(ThresholdMax);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud", 10, std::bind(&PreprocessNode::subscriber_callback, this, _1));

    std::string publisher_basename = "converted_pointcloud/splited_data";
    for (size_t i = 0; i < split_thresholds.size() - 1; i++)
    {
      publishers.push_back(create_publisher<sensor_msgs::msg::PointCloud2>(publisher_basename + std::to_string(i), 10));
    }

    converted_pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("converted_pointcloud", 10);
  }

private:
  using PC_Type = pcl::PointCloud<pcl::PointXYZRGB>;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> publishers;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr converted_pointcloud_publisher;

  std::vector<double> split_thresholds;

  const float ThresholdLowest = std::numeric_limits<float>::lowest();
  const float ThresholdMax = std::numeric_limits<float>::max();

  void subscriber_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "subscribe pointcloud(size: %d)", msg->data.size());

    // PCLに変換
    RCLCPP_INFO(this->get_logger(), "change to pcl");
    PC_Type original_cloud;
    pcl::fromROSMsg(*msg, original_cloud);

    // 傾きの補正
    RCLCPP_INFO(this->get_logger(), "correct the tilt");
    auto robot_tilt = get_parameter("robot_tilt").as_double_array();

    auto roll = robot_tilt[0] * M_PI / 180;
    auto pitch = robot_tilt[1] * M_PI / 180;
    auto yaw = robot_tilt[2] * M_PI / 180;

    // カメラの回転と同じにすることに注意
    PC_Type::Ptr transformed_cloud(new PC_Type);
    pcl::transformPointCloud(original_cloud, *transformed_cloud, rotation_y_matrix(yaw) * rotation_z_matrix(-pitch) * rotation_x_matrix(roll));

    // 分割してROSのMSGに変換
    std::vector<sensor_msgs::msg::PointCloud2::UniquePtr> new_msgs;

    RCLCPP_INFO(this->get_logger(), "generate layers by spliting");
    for (size_t i = 0; i < split_thresholds.size() - 1; i++)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "generate layers by spliting " << i);
      const auto &min_th = split_thresholds[i];
      const auto &max_th = split_thresholds[i + 1];

      auto splited = split_pointcloud(*transformed_cloud, min_th, max_th);
      sensor_msgs::msg::PointCloud2::UniquePtr new_msg(new sensor_msgs::msg::PointCloud2());
      remove_floor(splited);
      pcl::toROSMsg(*splited, *new_msg);
      new_msgs.push_back(std::move(new_msg));
    }

    // headerを追加してpublish
    RCLCPP_INFO(this->get_logger(), "publish");
    msg->header.set__frame_id("nemui");
    for (size_t i = 0; i < publishers.size(); i++)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "publish layer " << i << " size: " << new_msgs[i]->data.size() / 32);
      new_msgs[i]->header = msg->header;
      publishers[i]->publish(std::move(new_msgs[i]));
    }

    remove_floor(transformed_cloud);

    sensor_msgs::msg::PointCloud2::UniquePtr converted_pointcloud(new sensor_msgs::msg::PointCloud2());
    pcl::toROSMsg(*transformed_cloud, *converted_pointcloud);
    RCLCPP_INFO_STREAM(this->get_logger(), "publish converted pc that is size: " << transformed_cloud->size());
    converted_pointcloud->header = msg->header;
    converted_pointcloud_publisher->publish(std::move(converted_pointcloud));
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

  static PC_Type::Ptr split_pointcloud(const PC_Type &origin, const float &min_value, const float &max_value)
  {
    PC_Type splited;
    pcl::PassThrough<PC_Type::PointType> cloud_filter;
    cloud_filter.setInputCloud(origin.makeShared());
    cloud_filter.setFilterFieldName("y");
    cloud_filter.setFilterLimits(min_value, max_value);
    cloud_filter.filter(splited);

    return splited.makeShared();
  }

  void remove_floor(PC_Type::Ptr origin)
  {
    pcl::SACSegmentation<PC_Type::PointType> seg;

    seg.setOptimizeCoefficients(true);     // モデル係数を最適化
    seg.setModelType(pcl::SACMODEL_PLANE); // 平行平面モデル
    seg.setMethodType(pcl::SAC_MLESAC);    // MLESAC または MSAC
    seg.setMaxIterations(1000);            // 最大イテレーション数
    seg.setDistanceThreshold(0.06);        // 平面距離の閾値

    // Set axis to Y direction
    seg.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));
    seg.setEpsAngle(5.0 * M_PI / 180.0); // Allow deviation within 5 degrees

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    seg.setInputCloud(origin);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty())
    {
      return;
    }

    pcl::ExtractIndices<PC_Type::PointType> extract;
    extract.setInputCloud(origin);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*origin);

    if (origin->empty())
    {
      return;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreprocessNode>());
  rclcpp::shutdown();

  return 0;
}