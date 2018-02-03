#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <unistd.h>		// clock
#include <time.h>		// clock
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "rclcpp/clock.hpp"

int main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2 cloud_filtered;

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height 
       << " data points (" << pcl::getFieldsList (cloud_filtered) << ").";

  // pcl::PCDWriter writer;
  // writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
  //        Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

    // Convert to ROS data type
  sensor_msgs::msg::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  std::cerr << "PointCloud in output: " << output.width * output.height 
       << " data points (" << pcl::getFieldsList (cloud_filtered) << ").";

  output.header.frame_id = "pointcloud";
  std::cout<<"Frame Id is"<<output.header.frame_id<<std::endl;
  //std::cout<<"Frame Id original is"<<cloud_filtered.header.frame_id<<std::endl;

  rclcpp::init(argc, argv);

  auto rviz_node = rclcpp::Node::make_shared("talker");
  //auto rviz_node = std::make_shared<rclcpp::Node>("basic_shapes");
 
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 7;
  
  auto marker_pub = rviz_node->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud",custom_qos_profile);

  rclcpp::WallRate loop_rate(1);
  marker_pub->publish(output);

  //rclcpp::Clock system_clock(RCL_ROS_TIME);

 
  // Set our initial shape type to be a cube
  //uint32_t shape = visualization_msgs::msg::Marker::CUBE;

  while (rclcpp::ok())
  {
    std::cout<<"Publishing point cloud output"<<std::endl;
   std::cout<<"Frame Id is"<<output.header.frame_id<<std::endl;
    marker_pub->publish(output);
    rclcpp::spin_some(rviz_node);
    loop_rate.sleep();
  }

  return 0;
}
