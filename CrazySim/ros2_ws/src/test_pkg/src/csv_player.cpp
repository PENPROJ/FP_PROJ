#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <chrono>

class CsvTopicReplay : public rclcpp::Node
{
public:
    CsvTopicReplay(const std::string& filename)
    : Node("csv_topic_replay"), index_(0)
    {
        // === Publishers ===
        pose_pub_ = this->create_publisher<turtlesim::msg::Pose>("/turtle1/pose", 10);
        cmd_pub_  = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        cf_pub_   = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cf2/pose", 10);
        ee_pub_   = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/EE_cmd_xyzYaw", 10);

        // === Load CSV ===
        std::string filepath = std::string(std::getenv("HOME")) +
                               "/sitl_crazy/CrazySim/ros2_ws/src/test_pkg/bag/" +
                               filename;

        std::ifstream file(filepath);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to open: %s", filepath.c_str());
            rclcpp::shutdown();
            return;
        }

        std::string line;
        std::getline(file, line);  // skip header

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            std::vector<double> row;
        
            // 👇 여기! timestamp 포함해서 모든 값 파싱
            while (std::getline(ss, token, ',')) {
                try {
                    row.push_back(std::stod(token));
                } catch (...) {
                    row.push_back(0.0);  // fallback for invalid entries
                }
            }
        
            if (row.size() >= 20)  // timestamp + 19개
                parsed_data_.push_back(row);
        }

        RCLCPP_INFO(this->get_logger(), "✅ Loaded %zu rows from CSV", parsed_data_.size());

        // === 150Hz Timer ===
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(6666),
            std::bind(&CsvTopicReplay::publish_next, this));
    }

private:
    void publish_next()
    {
        if (index_ >= parsed_data_.size()) {
            RCLCPP_INFO(this->get_logger(), "✅ Finished replaying all rows.");
            rclcpp::shutdown();
            return;
        }

        const auto& row = parsed_data_[index_];

        // === 1. /turtle1/pose ===
        turtlesim::msg::Pose pose;
        pose.x = row[1]; pose.y = row[2]; pose.theta = row[3];
        pose.linear_velocity = row[4]; pose.angular_velocity = row[5];
        pose_pub_->publish(pose);

        // === 2. /turtle1/cmd_vel ===
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = row[6];
        cmd.angular.z = row[7];
        cmd_pub_->publish(cmd);

        // === 3. /cf2/pose ===
        geometry_msgs::msg::PoseStamped cf;
        cf.header.stamp = this->now();
        cf.header.frame_id = "world";
        cf.pose.position.x = row[8];
        cf.pose.position.y = row[9];
        cf.pose.position.z = row[10];
        cf.pose.orientation.x = row[11];
        cf.pose.orientation.y = row[12];
        cf.pose.orientation.z = row[13];
        cf.pose.orientation.w = row[14];
        cf_pub_->publish(cf);

        // === 4. /pen/EE_cmd_xyzYaw ===
        std_msgs::msg::Float64MultiArray ee;
        for (size_t i = 15; i <= 18; ++i)
            ee.data.push_back(row[i]);
        ee_pub_->publish(ee);

        index_++;
    }

    // Publishers
    rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_pub_;

    // Timer + CSV buffer
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::vector<double>> parsed_data_;
    size_t index_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cerr << "Usage: ros2 run test_pkg csv_topic_replay <filename.csv>" << std::endl;
        return 1;
    }

    auto node = std::make_shared<CsvTopicReplay>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
