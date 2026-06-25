#include "pcd2pgm/pcd2pgm.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>

#include <filesystem>
#include <sstream>
#include <fstream>
#include <limits>

namespace pcd2pgm
{

Pcd2Pgm::Pcd2Pgm(const rclcpp::NodeOptions & options) : Node("pcd2pgm", options)
{
    // Common Parameters
    declare_parameter<std::string>("pcd_path", "");
    declare_parameter<std::string>("output_dir", "");
    declare_parameter<std::string>("save_map_name", "map");
    declare_parameter<double>("resolution", 0.10);
    declare_parameter<int>("min_points_per_cell", 1);
    declare_parameter<double>("h_min", 0.10);
    declare_parameter<double>("h_max", 2.00);
    declare_parameter<float>("downsample_leaf_size", 0.10);

    get_parameter("pcd_path", pcd_path_);
    get_parameter("output_dir", output_dir_);
    get_parameter("save_map_name", save_map_name_);
    get_parameter("resolution", resolution_);
    get_parameter("min_points_per_cell",min_points_per_cell_);
    get_parameter("h_min", h_min_);
    get_parameter("h_max", h_max_);
    get_parameter("downsample_leaf_size", downsample_leaf_size_);


    // PMF parameters
    declare_parameter<int>("max_window_size", 20);
    declare_parameter<double>("slope", 1.00);
    declare_parameter<double>("initial_distance", 0.50);
    declare_parameter<double>("max_distance", 3.00);
    get_parameter("max_window_size", max_window_size_);
    get_parameter("slope", slope_);
    get_parameter("initial_distance", initial_distance_);
    get_parameter("max_distance", max_distance_);

    // SOR parameters
    declare_parameter<int>("mean_k", 20);
    declare_parameter<double>("stddev_mul", 1.00);
    get_parameter("mean_k", mean_k_);
    get_parameter("stddev_mul", stddev_mul_);

    // Debug
    declare_parameter<bool>("save_all_cloud", false);
    get_parameter("save_all_cloud", save_all_cloud_);


    std::filesystem::create_directories(output_dir_);

    if (pcd_path_.empty()) {
        RCLCPP_ERROR(get_logger(), "Parameter 'pcd_path' is empty.");
        rclcpp::shutdown();
        return;
    }

    process();
    rclcpp::shutdown();
}

void Pcd2Pgm::process()
{
    // Load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *cloud) != 0 || cloud->empty()) {
        RCLCPP_ERROR(get_logger(), "Failed to read PCD: %s", pcd_path_.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(), "Loaded PCD: %zu points", cloud->size());


    // Downsample
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(downsample_leaf_size_, downsample_leaf_size_, downsample_leaf_size_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsample(new pcl::PointCloud<pcl::PointXYZ>());
    vg.filter(*cloud_downsample);
    RCLCPP_INFO(get_logger(), "Downsample Point Cloud: %zu points", cloud_downsample->size());
    

    // Ground Segmentation (PMF)
    pcl::PointIndicesPtr ground_idx(new pcl::PointIndices);
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(cloud_downsample);
    pmf.setMaxWindowSize(max_window_size_);
    pmf.setSlope(slope_);
    pmf.setInitialDistance(initial_distance_);
    pmf.setMaxDistance(max_distance_);
    pmf.extract(ground_idx->indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonground(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud(cloud_downsample);
    ex.setIndices(ground_idx);
    ex.setNegative(false);
    ex.filter(*cloud_ground);
    ex.setNegative(true);
    ex.filter(*cloud_nonground);

    // SOR
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonground_sor(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_nonground);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(stddev_mul_);
    sor.setNegative(false);
    sor.filter(*cloud_nonground_sor);
    RCLCPP_INFO(get_logger(), "Ground: %zu, Non ground: %zu", cloud_ground->size(), cloud_nonground_sor->size());
    if (cloud_ground->empty()) {
        RCLCPP_ERROR(get_logger(), "No ground points detected. Adjust PMF params.");
        return;
    }


    // Obstacle Cloud Extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
    kdtree->setInputCloud(cloud_ground);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_obstacles->reserve(cloud_nonground_sor->size());

    std::vector<int> nn_idx; std::vector<float> nn_dist;
    const int K = 12;

    for (const auto &p : cloud_nonground_sor->points) {
        if (kdtree->nearestKSearch(p, K, nn_idx, nn_dist) <= 0) continue;
        double gz = 0.0; int cnt = 0;
      for (int idx : nn_idx) {
        if (idx < 0 || static_cast<size_t>(idx) >= cloud_ground->size()) continue;
        gz += cloud_ground->points[idx].z; cnt++;
      }
      if (cnt == 0) continue;
      gz /= static_cast<double>(cnt);

      double dz = static_cast<double>(p.z) - gz;
      if (dz >= h_min_ && dz <= h_max_) cloud_obstacles->push_back(p);
    }
    RCLCPP_INFO(get_logger(), "Obstacle: %zu", cloud_obstacles->size());


    // Convert Obstacle Cloud to PGM and create yaml
    savePgmYaml(cloud_obstacles);
    

    // Debug
    if (save_all_cloud_) {
        std::ostringstream oss_ds;    
        oss_ds << output_dir_ << "/cloud_downsample.pcd";
        pcl::io::savePCDFileBinary(oss_ds.str(), *cloud_downsample);
        RCLCPP_DEBUG(get_logger(), "Saved %zu points to %s", cloud_downsample->size(), oss_ds.str().c_str());

        std::ostringstream oss_g;
        oss_g << output_dir_ << "/cloud_ground.pcd";
        pcl::io::savePCDFileBinary(oss_g.str(), *cloud_ground);
        RCLCPP_DEBUG(get_logger(), "Saved %zu points to %s", cloud_ground->size(), oss_g.str().c_str());

        std::ostringstream oss_ng;
        oss_ng << output_dir_ << "/cloud_nonground.pcd";
        pcl::io::savePCDFileBinary(oss_ng.str(), *cloud_nonground_sor);
        RCLCPP_DEBUG(get_logger(), "Saved %zu points to %s", cloud_nonground_sor->size(), oss_ng.str().c_str());

        std::ostringstream oss_o;    
        oss_o << output_dir_ << "/cloud_obstacles.pcd";
        pcl::io::savePCDFileBinary(oss_o.str(), *cloud_obstacles);
        RCLCPP_DEBUG(get_logger(), "Saved %zu points to %s", cloud_obstacles->size(), oss_o.str().c_str());
    }

    return;
}

void Pcd2Pgm::savePgmYaml(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_obstacles)
{
    namespace fs = std::filesystem;
    if (!cloud_obstacles || cloud_obstacles->empty()) {
        RCLCPP_WARN(get_logger(), "No obstacle points to save.");
        return;
    }

    std::error_code ec;
    fs::create_directories(output_dir_, ec);
    if (ec) {
        RCLCPP_ERROR(get_logger(), "Failed to create output_dir '%s': %s", output_dir_.c_str(), ec.message().c_str());
        return;
    }

    double min_x =  std::numeric_limits<double>::infinity();
    double min_y =  std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    for (const auto& p : cloud_obstacles->points) {
        min_x = std::min(min_x, (double)p.x);
        min_y = std::min(min_y, (double)p.y);
        max_x = std::max(max_x, (double)p.x);
        max_y = std::max(max_y, (double)p.y);
    }

    if (!std::isfinite(min_x) || !std::isfinite(min_y) ||
        !std::isfinite(max_x) || !std::isfinite(max_y)) {
        RCLCPP_ERROR(get_logger(), "Invalid bbox for obstacles.");
        return;
    }

    const int width  = std::max(1, (int)std::ceil((max_x - min_x) / resolution_));
    const int height = std::max(1, (int)std::ceil((max_y - min_y) / resolution_));

    std::vector<uint16_t> counters(width * height, 0);

    auto xy_to_index = [&](double x, double y)->int {
        int ix = (int)std::floor((x - min_x) / resolution_);
        int iy = (int)std::floor((y - min_y) / resolution_);
        if (ix < 0 || ix >= width || iy < 0 || iy >= height) return -1;
        return iy * width + ix;
    };

    for (const auto& p : cloud_obstacles->points) {
        int i = xy_to_index(p.x, p.y);
        if (i >= 0) counters[i] += 1;
    }

    const std::string pgm = (fs::path(output_dir_) / (save_map_name_ + ".pgm")).string();
    std::ofstream ofs(pgm, std::ios::binary);
    if (!ofs) {
        RCLCPP_ERROR(get_logger(), "Failed to open PGM: %s", pgm.c_str());
        return;
    }

    ofs << "P5\n" << width << " " << height << "\n255\n";
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
        int i = y * width + x;
        uint8_t pix = 254;
        if (counters[i] >= (uint16_t)min_points_per_cell_) pix = 0;
        ofs.write(reinterpret_cast<char*>(&pix), 1);
        }
    }
    ofs.close();

    const std::string yaml = (fs::path(output_dir_) / (save_map_name_ + ".yaml")).string();
    std::ofstream yfs(yaml);
    if (!yfs) {
        RCLCPP_ERROR(get_logger(), "Failed to open YAML: %s", yaml.c_str());
        return;
    }
    yfs << "image: " << (save_map_name_ + ".pgm") << "\n";
    yfs << "mode: trinary\n";
    yfs << "resolution: " << resolution_ << "\n";
    yfs << "origin: [" << min_x << ", " << min_y << ", 0.0]\n";
    yfs << "negate: 0\n";
    yfs << "occupied_thresh: 0.65\n";
    yfs << "free_thresh: 0.196\n";
    yfs.close();

    RCLCPP_INFO(get_logger(), "Saved map: %s, %s (w=%d, h=%d, res=%.3f)",
                pgm.c_str(), yaml.c_str(), width, height, resolution_);
}

}   // namespace pcd2pgm

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pcd2pgm::Pcd2Pgm);