#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>

#include <chrono>
#include <iostream>

using std::placeholders::_1;

class PostprocessNode : public rclcpp::Node
{
public:
    PostprocessNode()
        : Node("postprocess_node")
    {
        pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("pointcloud", 10, std::bind(&PostprocessNode::pointcloud_subscriber_callback, this, _1));
        pose_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("pose_estimation_2D", 10, std::bind(&PostprocessNode::pose_subscriber_callback, this, _1));

        converted_pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("converted_pointcloud", 10);
    }

private:
    using PC_Type = pcl::PointCloud<pcl::PointXYZRGB>;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pose_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr converted_pointcloud_publisher;

    PC_Type original_pointcloud;
    void pointcloud_subscriber_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "subscribe pointcloud");

        pcl::fromROSMsg(*msg, original_pointcloud);
    }

    void pose_subscriber_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "subscribe pose");

        // 回転の変換
        auto x = msg->data[0];
        auto z = msg->data[1];
        auto angle = -msg->data[2];

        // 傾き・位置の補正
        PC_Type::Ptr transformed(new PC_Type);
        auto trans_matrix = pcl::getTransformation(x, 0, z, 0, angle, 0);
        pcl::transformPointCloud(original_pointcloud, *transformed, trans_matrix);

        RCLCPP_INFO(this->get_logger(), "publish transformed pointcloud");
        sensor_msgs::msg::PointCloud2::UniquePtr converted_pointcloud(new sensor_msgs::msg::PointCloud2());
        pcl::toROSMsg(*transformed, *converted_pointcloud);
        converted_pointcloud->header.set__frame_id("nemui");
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PostprocessNode>());
    rclcpp::shutdown();

    return 0;
}