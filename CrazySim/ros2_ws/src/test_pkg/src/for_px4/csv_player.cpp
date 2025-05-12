#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class CsvPublisher : public rclcpp::Node
{
public:
  CsvPublisher()
  : Node("csv_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/csv_data", 10);
    load_csv();
    publish_loop();
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

  // <timestamp (nanosec), data>
  std::vector<std::pair<uint64_t, std::vector<double>>> data_vector_;

  void load_csv()
  {
    std::string filepath = std::string(std::getenv("HOME")) +
                           "/sitl_crazy/CrazySim/ros2_ws/src/test_pkg/bag/" +
                           "rosbag2_2025_05_09-04_12_59.csv";

    std::ifstream file(filepath);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file at: %s", filepath.c_str());
      return;
    }

    std::string line;
    std::getline(file, line);  // Skip header

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string cell;
      std::vector<double> row_data;
      uint64_t timestamp = 0;
      int col_index = 0;

      while (std::getline(ss, cell, ',')) {
        if (col_index == 0) {
          timestamp = std::stoull(cell);  // nanoseconds
        } else if (col_index >= 2) {
          row_data.push_back(std::stod(cell));
        }
        ++col_index;
      }

      data_vector_.emplace_back(timestamp, row_data);
    }
  }

  void publish_loop()
  {
    std::thread([this]() {
      for (size_t i = 0; i < data_vector_.size(); ++i) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = data_vector_[i].second;
        publisher_->publish(msg);

        if (i + 1 < data_vector_.size()) {
          uint64_t t_curr = data_vector_[i].first;
          uint64_t t_next = data_vector_[i + 1].first;
          uint64_t delta_ns = t_next - t_curr;

          // Avoid sleeping for unreasonable delays
          if (delta_ns > 0 && delta_ns < 1000000000ULL) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(delta_ns));
          }
        }
      }
    }).detach();
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CsvPublisher>());
  rclcpp::shutdown();
  return 0;
}
