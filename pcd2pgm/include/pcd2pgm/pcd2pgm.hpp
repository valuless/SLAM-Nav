#ifndef PCD2PGM__PCD2PGM_HPP_
#define PCD2PGM__PCD2PGM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

namespace pcd2pgm
{

class Pcd2Pgm : public rclcpp::Node
{

    public:
        explicit Pcd2Pgm(const rclcpp::NodeOptions & options);

    private:
        void process();
        void savePgmYaml(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_obstacles);

        // Common Parameters
        std::string pcd_path_{""};
        std::string output_dir_{""};
        std::string save_map_name_{""};
        double resolution_{0.10};
        int min_points_per_cell_{1};
        double h_min_{0.10};
        double h_max_{2.00};
        float downsample_leaf_size_{0.10};

        // PMF parameters
        int max_window_size_{20};
        double slope_{1.00};
        double initial_distance_{0.50};
        double max_distance_{3.00};

        // SOR parameters
        int mean_k_{20};
        double stddev_mul_{1.00};

        // Debug
        bool save_all_cloud_{false};
};

}   // namespace pcd2pgm

#endif  // PCD2PGM__PCD2PGM_HPP_