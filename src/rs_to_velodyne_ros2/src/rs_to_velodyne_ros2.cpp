#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/impl/point_types.hpp>
#include <cmath>

#include <string>
#include <cstring>
#include <memory>

// Ruby 128线映射表
static const int RING_ID_MAP_RUBY[] = {
    3, 66, 33, 96, 11, 74, 41, 104, 19, 82, 49, 112, 27, 90, 57, 120,
    35, 98, 1, 64, 43, 106, 9, 72, 51, 114, 17, 80, 59, 122, 25, 88,
    67, 34, 97, 0, 75, 42, 105, 8, 83, 50, 113, 16, 91, 58, 121, 24,
    99, 2, 65, 32, 107, 10, 73, 40, 115, 18, 81, 48, 123, 26, 89, 56,
    7, 70, 37, 100, 15, 78, 45, 108, 23, 86, 53, 116, 31, 94, 61, 124,
    39, 102, 5, 68, 47, 110, 13, 76, 55, 118, 21, 84, 63, 126, 29, 92,
    71, 38, 101, 4, 79, 46, 109, 12, 87, 54, 117, 20, 95, 62, 125, 28,
    103, 6, 69, 36, 111, 14, 77, 44, 119, 22, 85, 52, 127, 30, 93, 60
};

// 16线映射表
static const int RING_ID_MAP_16[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8
};

// 速腾点云格式
struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)
    (uint16_t, ring, ring)(double, timestamp, timestamp))

// Velodyne带时间戳格式
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint16_t, ring, ring)(float, time, time))

// Velodyne不带时间戳格式
struct VelodynePointXYZIR {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint16_t, ring, ring))

class RsToVelodyneNode : public rclcpp::Node {
public:
    RsToVelodyneNode() : Node("rs_converter") {
        declare_parameter<std::string>("input_type", "XYZIRT");
        declare_parameter<std::string>("output_type", "XYZIRT");
        declare_parameter<std::string>("input_topic", "/rslidar_points");
        declare_parameter<std::string>("output_topic", "/velodyne_points");
        declare_parameter<std::string>("output_frame_id", "velodyne");
        
        std::string input_type = get_parameter("input_type").as_string();
        output_type_ = get_parameter("output_type").as_string();
        std::string input_topic = get_parameter("input_topic").as_string();
        std::string output_topic = get_parameter("output_topic").as_string();
        output_frame_id_ = get_parameter("output_frame_id").as_string();
        
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
        
        if (input_type == "XYZI") {
            sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                input_topic, 10,
                std::bind(&RsToVelodyneNode::rsHandler_XYZI, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Subscribing to %s with XYZI type", input_topic.c_str());
        } else if (input_type == "XYZIRT") {
            sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
                input_topic, 10,
                std::bind(&RsToVelodyneNode::rsHandler_XYZIRT, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Subscribing to %s with XYZIRT type", input_topic.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Unsupported input type: %s", input_type.c_str());
            rclcpp::shutdown();
        }
        
        RCLCPP_INFO(get_logger(), "Publishing to %s with output type: %s", output_topic.c_str(), output_type_.c_str());
    }

private:
    template<typename T>
    bool has_nan(const T& point) {
        // 使用 std::isnan 替代 pcl_isnan
        return (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z));
    }
    
    template<typename T>
    void publish_points(const T& new_pc, const sensor_msgs::msg::PointCloud2::SharedPtr& old_msg) {
        new_pc->is_dense = true;
        
        sensor_msgs::msg::PointCloud2 pc_new_msg;
        pcl::toROSMsg(*new_pc, pc_new_msg);
        pc_new_msg.header = old_msg->header;
        pc_new_msg.header.stamp = this->now();
        pc_new_msg.header.frame_id = output_frame_id_;
        pub_->publish(pc_new_msg);
    }
    
    void rsHandler_XYZI(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_new(new pcl::PointCloud<VelodynePointXYZIR>());
        pcl::fromROSMsg(*pc_msg, *pc);
        
        pc_new->reserve(pc->points.size());
        
        for (size_t point_id = 0; point_id < pc->points.size(); ++point_id) {
            if (has_nan(pc->points[point_id]))
                continue;
            
            VelodynePointXYZIR new_point;
            new_point.x = pc->points[point_id].x;
            new_point.y = pc->points[point_id].y;
            new_point.z = pc->points[point_id].z;
            new_point.intensity = pc->points[point_id].intensity;
            
            if (pc->height == 16) {
                new_point.ring = RING_ID_MAP_16[point_id / pc->width];
            } else if (pc->height == 128) {
                new_point.ring = RING_ID_MAP_RUBY[point_id % pc->height];
            }
            pc_new->points.push_back(new_point);
        }
        
        publish_points(pc_new, pc_msg);
    }
    
    template<typename T_in_p, typename T_out_p>
    void handle_pc_msg(const typename pcl::PointCloud<T_in_p>::Ptr& pc_in,
                       const typename pcl::PointCloud<T_out_p>::Ptr& pc_out) {
        pc_out->reserve(pc_in->points.size());
        
        for (size_t point_id = 0; point_id < pc_in->points.size(); ++point_id) {
            if (has_nan(pc_in->points[point_id]))
                continue;
            
            T_out_p new_point;
            new_point.x = pc_in->points[point_id].x;
            new_point.y = pc_in->points[point_id].y;
            new_point.z = pc_in->points[point_id].z;
            new_point.intensity = pc_in->points[point_id].intensity;
            pc_out->points.push_back(new_point);
        }
    }
    
    template<typename T_in_p, typename T_out_p>
    void add_ring(const typename pcl::PointCloud<T_in_p>::Ptr& pc_in,
                  typename pcl::PointCloud<T_out_p>::Ptr& pc_out) {
        size_t valid_point_id = 0;
        for (size_t point_id = 0; point_id < pc_in->points.size(); ++point_id) {
            if (has_nan(pc_in->points[point_id]))
                continue;
            pc_out->points[valid_point_id++].ring = pc_in->points[point_id].ring;
        }
    }
    
    template<typename T_in_p, typename T_out_p>
    void add_time(const typename pcl::PointCloud<T_in_p>::Ptr& pc_in,
                  typename pcl::PointCloud<T_out_p>::Ptr& pc_out) {
        size_t valid_point_id = 0;
        for (size_t point_id = 0; point_id < pc_in->points.size(); ++point_id) {
            if (has_nan(pc_in->points[point_id]))
                continue;
            pc_out->points[valid_point_id++].time = static_cast<float>(
                pc_in->points[point_id].timestamp - pc_in->points[0].timestamp);
        }
    }
    
    void rsHandler_XYZIRT(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
        pcl::PointCloud<RsPointXYZIRT>::Ptr pc_in(new pcl::PointCloud<RsPointXYZIRT>());
        pcl::fromROSMsg(*pc_msg, *pc_in);
        
        

        if (output_type_ == "XYZIRT") {
            pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIRT>());
            handle_pc_msg<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
            add_ring<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
            add_time<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
            publish_points(pc_out, pc_msg);
        } else if (output_type_ == "XYZIR") {
            pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIR>());
            handle_pc_msg<RsPointXYZIRT, VelodynePointXYZIR>(pc_in, pc_out);
            add_ring<RsPointXYZIRT, VelodynePointXYZIR>(pc_in, pc_out);
            publish_points(pc_out, pc_msg);
        } else if (output_type_ == "XYZI") {
            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZI>());
            handle_pc_msg<RsPointXYZIRT, pcl::PointXYZI>(pc_in, pc_out);
            publish_points(pc_out, pc_msg);
        } else {
            RCLCPP_WARN(get_logger(), "Unsupported output type: %s", output_type_.c_str());
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::string output_type_;
    std::string output_frame_id_;
};

int main(int argc, char** argv) {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RsToVelodyneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}