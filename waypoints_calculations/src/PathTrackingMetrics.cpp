#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>
#include <fstream>
#include <chrono>
#include <iomanip>

class PathTrackingMetrics : public rclcpp::Node
{
public:
    PathTrackingMetrics() : Node("path_tracking_metrics")
    {
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&PathTrackingMetrics::odomCallback, this, std::placeholders::_1));
        
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, 
            std::bind(&PathTrackingMetrics::pathCallback, this, std::placeholders::_1));
        
        // Publishers
        cross_track_error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/metrics/cross_track_error", 10);
        heading_error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/metrics/heading_error", 10);
        closest_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/metrics/closest_point", 10);
        
        // Timer for periodic metrics calculation
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PathTrackingMetrics::calculateMetrics, this));
        
        // Open metrics file
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
            now.time_since_epoch()).count();
        
        std::string filename = "/home/armdut/waypoints/metrics_exp1_" + 
                              std::to_string(timestamp) + ".csv";
        metrics_file_.open(filename);
        metrics_file_ << "timestamp,cross_track_error,heading_error,speed,x,y,closest_x,closest_y\n";
        
        RCLCPP_INFO(this->get_logger(), "✅ Path Tracking Metrics Node Started");
        RCLCPP_INFO(this->get_logger(), "📊 Logging to: %s", filename.c_str());
    }
    
    ~PathTrackingMetrics()
    {
        if (metrics_file_.is_open()) {
            metrics_file_.close();
        }
        printSummary();
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        current_velocity_ = std::sqrt(
            std::pow(msg->twist.twist.linear.x, 2) + 
            std::pow(msg->twist.twist.linear.y, 2)
        );
        has_odom_ = true;
    }
    
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        reference_path_ = *msg;
        has_path_ = true;
        RCLCPP_INFO(this->get_logger(), "📍 Received path with %zu waypoints", 
                    reference_path_.poses.size());
    }
    
    void calculateMetrics()
    {
        if (!has_odom_ || !has_path_ || reference_path_.poses.empty()) {
            return;
        }
        
        // Find closest point on path
        double min_distance = std::numeric_limits<double>::max();
        size_t closest_idx = 0;
        
        for (size_t i = 0; i < reference_path_.poses.size(); ++i) {
            double dx = reference_path_.poses[i].pose.position.x - current_pose_.position.x;
            double dy = reference_path_.poses[i].pose.position.y - current_pose_.position.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_idx = i;
            }
        }
        
        // Calculate cross-track error (lateral distance from path)
        double cross_track_error = min_distance;
        
        // Calculate heading error
        double path_heading = 0.0;
        if (closest_idx + 1 < reference_path_.poses.size()) {
            // Use direction to next waypoint
            double dx = reference_path_.poses[closest_idx + 1].pose.position.x - 
                       reference_path_.poses[closest_idx].pose.position.x;
            double dy = reference_path_.poses[closest_idx + 1].pose.position.y - 
                       reference_path_.poses[closest_idx].pose.position.y;
            path_heading = std::atan2(dy, dx);
        }
        
        // Get vehicle heading from quaternion
        double siny_cosp = 2.0 * (current_pose_.orientation.w * current_pose_.orientation.z + 
                                  current_pose_.orientation.x * current_pose_.orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (current_pose_.orientation.y * current_pose_.orientation.y + 
                                       current_pose_.orientation.z * current_pose_.orientation.z);
        double vehicle_heading = std::atan2(siny_cosp, cosy_cosp);
        
        double heading_error = path_heading - vehicle_heading;
        // Normalize to [-pi, pi]
        while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI) heading_error += 2.0 * M_PI;
        
        // Publish metrics
        std_msgs::msg::Float64 cte_msg, he_msg;
        cte_msg.data = cross_track_error;
        he_msg.data = heading_error;
        
        cross_track_error_pub_->publish(cte_msg);
        heading_error_pub_->publish(he_msg);
        
        // Visualize closest point
        publishClosestPointMarker(reference_path_.poses[closest_idx].pose.position);
        
        // Log to file
        auto timestamp = this->now().seconds();
        metrics_file_ << std::fixed << std::setprecision(3)
                     << timestamp << "," 
                     << cross_track_error << "," 
                     << heading_error << "," 
                     << current_velocity_ << ","
                     << current_pose_.position.x << ","
                     << current_pose_.position.y << ","
                     << reference_path_.poses[closest_idx].pose.position.x << ","
                     << reference_path_.poses[closest_idx].pose.position.y << "\n";
        
        // Update statistics
        total_cte_ += cross_track_error;
        total_he_ += std::abs(heading_error);
        max_cte_ = std::max(max_cte_, cross_track_error);
        max_he_ = std::max(max_he_, std::abs(heading_error));
        sample_count_++;
        
        // Log every 2 seconds (20 samples at 100ms rate)
        if (sample_count_ % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "📊 CTE: %.3f m | HE: %.3f rad (%.1f°) | Speed: %.2f m/s", 
                cross_track_error, heading_error, heading_error * 180.0 / M_PI, current_velocity_);
        }
    }
    
    void publishClosestPointMarker(const geometry_msgs::msg::Point& point)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.ns = "closest_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position = point;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        closest_point_pub_->publish(marker);
    }
    
    void printSummary()
    {
        if (sample_count_ > 0) {
            RCLCPP_INFO(this->get_logger(), "\n");
            RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════╗");
            RCLCPP_INFO(this->get_logger(), "║     EXPERIMENT 1: PATH TRACKING RESULTS        ║");
            RCLCPP_INFO(this->get_logger(), "╠════════════════════════════════════════════════╣");
            RCLCPP_INFO(this->get_logger(), "║  Cross-Track Error (CTE):                      ║");
            RCLCPP_INFO(this->get_logger(), "║    Average: %.3f m                             ║", 
                        total_cte_ / sample_count_);
            RCLCPP_INFO(this->get_logger(), "║    Maximum: %.3f m                             ║", max_cte_);
            RCLCPP_INFO(this->get_logger(), "║                                                ║");
            RCLCPP_INFO(this->get_logger(), "║  Heading Error:                                ║");
            RCLCPP_INFO(this->get_logger(), "║    Average: %.3f rad (%.1f°)                   ║", 
                        total_he_ / sample_count_, 
                        (total_he_ / sample_count_) * 180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(), "║    Maximum: %.3f rad (%.1f°)                   ║", 
                        max_he_, max_he_ * 180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(), "║                                                ║");
            RCLCPP_INFO(this->get_logger(), "║  Total Samples: %ld                            ║", sample_count_);
            RCLCPP_INFO(this->get_logger(), "║  Duration: %.1f seconds                        ║", 
                        sample_count_ * 0.1);
            RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════╝");
            RCLCPP_INFO(this->get_logger(), "\n");
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cross_track_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_error_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr closest_point_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    geometry_msgs::msg::Pose current_pose_;
    nav_msgs::msg::Path reference_path_;
    double current_velocity_ = 0.0;
    bool has_odom_ = false;
    bool has_path_ = false;
    
    std::ofstream metrics_file_;
    double total_cte_ = 0.0;
    double total_he_ = 0.0;
    double max_cte_ = 0.0;
    double max_he_ = 0.0;
    long sample_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathTrackingMetrics>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
