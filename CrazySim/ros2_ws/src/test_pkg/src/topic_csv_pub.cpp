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

class AllLoggerSubscriber : public rclcpp::Node
{
public:
    AllLoggerSubscriber(const std::string& filename)
    : Node("all_logger_subscriber"), index_(0)
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
        std::getline(file, line);  // skip header if any

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            std::vector<double> row;

            while (std::getline(ss, token, ',')) {
                try {
                    row.push_back(std::stod(token));
                } catch (...) {
                    row.push_back(0.0);
                }
            }

            if (row.size() >= 19)
                parsed_data_.push_back(row);
        }

        RCLCPP_INFO(this->get_logger(), "✅ Loaded %zu rows from CSV", parsed_data_.size());

        timer_ = this->create_wall_timer(
            std::chrono::microseconds(6666),  // 150Hz
            std::bind(&AllLoggerSubscriber::publish_next, this));
    }

private:
void publish_next()
{
    if (index_ >= parsed_data_.size()) {
        RCLCPP_INFO(this->get_logger(), "✅ Finished publishing.");
        rclcpp::shutdown();
        return;
    }

    const auto& row = parsed_data_[index_];

    // ✅ [0] timestamp (초 단위)
    double timestamp_sec = row[0];
    rclcpp::Time ros_time(static_cast<uint64_t>(timestamp_sec * 1e9));

    // === /turtle1/pose ===
    turtlesim::msg::Pose pose;
    pose.x = row[1];
    pose.y = row[2];
    pose.theta = row[3];
    pose.linear_velocity = row[4];
    pose.angular_velocity = row[5];
    pose_pub_->publish(pose);

    // === /turtle1/cmd_vel ===
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = row[6];
    cmd.angular.z = row[7];
    cmd_pub_->publish(cmd);

    // === /cf2/pose ===
    geometry_msgs::msg::PoseStamped cf;
    cf.header.stamp = ros_time;               // ⬅️ 여기 중요!
    cf.header.frame_id = "world";
    cf.pose.position.x = row[8];
    cf.pose.position.y = row[9];
    cf.pose.position.z = row[10];
    cf.pose.orientation.x = row[11];
    cf.pose.orientation.y = row[12];
    cf.pose.orientation.z = row[13];
    cf.pose.orientation.w = row[14];
    cf_pub_->publish(cf);

    // === /pen/EE_cmd_xyzYaw ===
    std_msgs::msg::Float64MultiArray ee;
    for (size_t i = 15; i <= 18; ++i)
        ee.data.push_back(row[i]);
    ee_pub_->publish(ee);

    index_++;
}

    rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::vector<double>> parsed_data_;
    size_t index_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cerr << "Usage: ros2 run test_pkg all_logger_subscriber <all_MMDDHHMM.csv>" << std::endl;
        return 1;
    }

    auto node = std::make_shared<AllLoggerSubscriber>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

