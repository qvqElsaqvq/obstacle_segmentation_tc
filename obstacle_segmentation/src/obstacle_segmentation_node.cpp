#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>

class ObstacleSegmentationNode : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    ObstacleSegmentationNode(std::string name)
        : Node("obstacle_segmentation_node")
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        // 声明参数
        this->declare_parameter("input_cloud_topic", "input_cloud");
        this->declare_parameter("output_cloud_topic", "output_cloud");
        this->declare_parameter("leaf_size", 0.1);
        this->declare_parameter("point_num_for_normal", 50);
        this->declare_parameter("angle_threshold", 0.1);
        this->declare_parameter("obstacle_x_min", -10.0);
        this->declare_parameter("obstacle_x_max", 10.0);
        this->declare_parameter("obstacle_y_min", -10.0);
        this->declare_parameter("obstacle_y_max", 10.0);
        this->declare_parameter("obstacle_z_min", 0.0);
        this->declare_parameter("obstacle_z_max", 2.0);
        this->declare_parameter("obstacle_range_min", 0.5);
        this->declare_parameter("obstacle_range_max", 2.0);
        this->declare_parameter("point_frame", "livox");
        this->declare_parameter("use_downsample", true);

        RCLCPP_INFO(this->get_logger(), "%s节点初始化.", name.c_str());
        this->get_parameter("input_cloud_topic", input_cloud_topic_);
        this->get_parameter("output_cloud_topic", output_cloud_topic_);
        this->get_parameter("leaf_size", leaf_size_);
        this->get_parameter("point_num_for_normal", point_num_for_normal_);
        this->get_parameter("angle_threshold", angle_threshold_);
        this->get_parameter("obstacle_x_min", obstacle_x_min_);
        this->get_parameter("obstacle_x_max", obstacle_x_max_);
        this->get_parameter("obstacle_y_min", obstacle_y_min_);
        this->get_parameter("obstacle_y_max", obstacle_y_max_);
        this->get_parameter("obstacle_z_min", obstacle_z_min_);
        this->get_parameter("obstacle_z_max", obstacle_z_max_);
        this->get_parameter("obstacle_range_min", obstacle_range_min_);
        this->get_parameter("obstacle_range_max", obstacle_range_max_);
        this->get_parameter("point_frame", point_frame_);
        this->get_parameter("use_downsample", use_downsample_);

        // 初始化pub和sub
        output_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
        input_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_cloud_topic_, 10, std::bind(&ObstacleSegmentationNode::cloudCallback, this, std::placeholders::_1));

    }

private:
    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    std::string point_frame_;
    float leaf_size_;          // 体素滤波器的体素大小
    int point_num_for_normal_; // 用于计算法向量的点数
    float angle_threshold_;    // 法向量与地面的夹角阈值
    float obstacle_x_min_;     // 障碍物点云范围(livox坐标系)
    float obstacle_x_max_;
    float obstacle_y_min_;
    float obstacle_y_max_;
    float obstacle_z_min_;
    float obstacle_z_max_;
    float obstacle_range_min_; // 障碍物点云范围(livox坐标系)
    float obstacle_range_max_;
    bool use_downsample_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_cloud_pub_;
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg->data.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "接收到的点云数据为空.");
            return;
        }

        // 将点云转换为pcl格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // 创建体素滤波器主要作用是对点云进行降采样，可以在保证点云原有几何结构基本不变的前提下减少点的数量
        if (use_downsample_)
        {
            pcl::VoxelGrid<pcl::PointXYZ> voxfilter;
            voxfilter.setInputCloud(cloud);
            // 设置滤波器的体素大小
            voxfilter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            voxfilter.filter(*cloud);
        }

        // 创建法向量估计对象
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        // 创建一个空的kdtree对象，并把它传递给法向量估计对象
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        // 输出数据集
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.setKSearch(point_num_for_normal_); // 使用最近的point_num_for_normal_个点计算法向量
        ne.compute(*cloud_normals);           // 计算法向量

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
        for (long i = 0; i < cloud->points.size(); i++)
        {
            float gradient = acos(sqrt(pow(cloud_normals->points[i].normal_x, 2) + pow(cloud_normals->points[i].normal_y, 2)) / sqrt(pow(cloud_normals->points[i].normal_x, 2) + pow(cloud_normals->points[i].normal_y, 2) + pow(cloud_normals->points[i].normal_z, 2)));
            // 如果法向量与地面的夹角小于角度阈值，是障碍物
            if (gradient < angle_threshold_)
            {
                // 按点云范围进行筛选
                if (cloud->points[i].x < obstacle_x_min_ || cloud->points[i].x > obstacle_x_max_ || cloud->points[i].y < obstacle_y_min_ || cloud->points[i].y > obstacle_y_max_ || cloud->points[i].z < obstacle_z_min_ || cloud->points[i].z > obstacle_z_max_)
                {
                    continue;
                }
                // 如果该点和机器人位置的距离大于距离阈值
                if ((pow(cloud->points[i].x, 2) + pow(cloud->points[i].y, 2)) < pow(obstacle_range_min_, 2) || (pow(cloud->points[i].x, 2) + pow(cloud->points[i].y, 2)) > pow(obstacle_range_max_, 2))
                {
                    continue;
                }
                cloud_out->points.push_back(cloud->points[i]);
            } //留下的点是范围内的障碍物
        }
        cloud_out->width = cloud_out->points.size();
        cloud_out->height = 1;
        cloud_out->is_dense = true;
        sensor_msgs::msg::PointCloud2::SharedPtr output_cloud(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*cloud_out, *output_cloud);
        output_cloud->header.frame_id = point_frame_;
        output_cloud->header.stamp = msg->header.stamp;
        output_cloud_pub_->publish(*output_cloud);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    rclcpp::executors::SingleThreadedExecutor executor;
    std::shared_ptr<ObstacleSegmentationNode> node = std::make_shared<ObstacleSegmentationNode>("obstacle_segmentation_node");
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
