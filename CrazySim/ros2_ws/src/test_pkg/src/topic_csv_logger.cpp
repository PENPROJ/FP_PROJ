#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <fstream>
#include <filesystem>
#include <chrono>

namespace fs = std::filesystem;

class TurtlePoseLogger : public rclcpp::Node
{
public:
    TurtlePoseLogger()
    : Node("topic_csv_logger")
    {
        auto qos = rclcpp::QoS(1)
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        data_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/data_logging", 10);

        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", qos,
            [this](const turtlesim::msg::Pose::SharedPtr msg) {
                latest_pose_ = *msg;
            });

        cmdvel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", qos,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                latest_cmd_vel_ = *msg;
            });

        cf2_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cf2/pose", qos,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                latest_cf2_pose_ = *msg;
            });

        ee_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/pen/EE_cmd_xyzYaw", qos,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                latest_ee_cmd_ = *msg;
            });

        // 기준 시간
        start_time_ = this->now();

        // 파일 경로 생성
        auto now_sys = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now_sys);
        std::tm now_tm = *std::localtime(&now_time);
        char filename_buf[64];
        std::strftime(filename_buf, sizeof(filename_buf), "%m%d%H%M.csv", &now_tm);
        std::string base_name(filename_buf);

        std::string base_path = std::string(std::getenv("HOME")) + "/sitl_crazy/CrazySim/ros2_ws/src/test_pkg/bag/";
        fs::create_directories(fs::path(base_path));

        // misc
        csv_path_ = base_path + "misc_" + base_name;
        csv_file_.open(csv_path_, std::ios::out | std::ios::app);

        // all
        all_csv_path_ = base_path + "all_" + base_name;
        all_csv_file_.open(all_csv_path_, std::ios::out | std::ios::app);

        if (csv_file_.is_open() && all_csv_file_.is_open()) {
            RCLCPP_INFO(this->get_logger(), "📄 Logging to:\n  misc: %s\n  all : %s", csv_path_.c_str(), all_csv_path_.c_str());
            csv_file_ << "time,x,y,theta,v,w,cmd_v,cmd_w,"
                      << "cf_x,cf_y,cf_z,qx,qy,qz,qw,"
                      << "ee_x,ee_y,ee_z,ee_yaw\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to open misc or all CSV file.");
        }

        // 150Hz 타이머
        control_loop_timer_ = this->create_wall_timer(
            std::chrono::microseconds(6666),
            std::bind(&TurtlePoseLogger::publish_combined_data, this));
    }

private:
    void publish_combined_data()
    {
        std_msgs::msg::Float64MultiArray msg;
        double elapsed = (this->now() - start_time_).seconds();

        // turtle1/pose
        msg.data.push_back(latest_pose_.x);
        msg.data.push_back(latest_pose_.y);
        msg.data.push_back(latest_pose_.theta);
        msg.data.push_back(latest_pose_.linear_velocity);
        msg.data.push_back(latest_pose_.angular_velocity);

        // cmd_vel
        msg.data.push_back(latest_cmd_vel_.linear.x);
        msg.data.push_back(latest_cmd_vel_.angular.z);

        // cf2/pose
        msg.data.push_back(latest_cf2_pose_.pose.position.x);
        msg.data.push_back(latest_cf2_pose_.pose.position.y);
        msg.data.push_back(latest_cf2_pose_.pose.position.z);
        msg.data.push_back(latest_cf2_pose_.pose.orientation.x);
        msg.data.push_back(latest_cf2_pose_.pose.orientation.y);
        msg.data.push_back(latest_cf2_pose_.pose.orientation.z);
        msg.data.push_back(latest_cf2_pose_.pose.orientation.w);

        // ee_cmd (pad if needed)
        size_t ee_len = std::min(static_cast<size_t>(4), latest_ee_cmd_.data.size());
        for (size_t i = 0; i < ee_len; ++i)
            msg.data.push_back(latest_ee_cmd_.data[i]);
        while (msg.data.size() < 19)
            msg.data.push_back(0.0);

        // publish
        data_pub_->publish(msg);

        // misc: 분리된 열로 저장
        if (csv_file_.is_open()) {
            csv_file_ << elapsed
                      << "," << latest_pose_.x
                      << "," << latest_pose_.y
                      << "," << latest_pose_.theta
                      << "," << latest_pose_.linear_velocity
                      << "," << latest_pose_.angular_velocity
                      << "," << latest_cmd_vel_.linear.x
                      << "," << latest_cmd_vel_.angular.z
                      << "," << latest_cf2_pose_.pose.position.x
                      << "," << latest_cf2_pose_.pose.position.y
                      << "," << latest_cf2_pose_.pose.position.z
                      << "," << latest_cf2_pose_.pose.orientation.x
                      << "," << latest_cf2_pose_.pose.orientation.y
                      << "," << latest_cf2_pose_.pose.orientation.z
                      << "," << latest_cf2_pose_.pose.orientation.w;

            for (size_t i = 0; i < 4; ++i)
                csv_file_ << "," << (i < latest_ee_cmd_.data.size() ? latest_ee_cmd_.data[i] : 0.0);
            csv_file_ << "\n";
            csv_file_.flush();
        }

        // all: msg.data 그대로 저장
        if (all_csv_file_.is_open()) {
            all_csv_file_ << elapsed;
            for (double val : msg.data)
                all_csv_file_ << "," << val;
            all_csv_file_ << "\n";
            all_csv_file_.flush();
        }
    }

    // ROS I/O
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf2_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ee_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr data_pub_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    // 상태 저장
    turtlesim::msg::Pose latest_pose_;
    geometry_msgs::msg::Twist latest_cmd_vel_;
    geometry_msgs::msg::PoseStamped latest_cf2_pose_;
    std_msgs::msg::Float64MultiArray latest_ee_cmd_;

    // CSV 기록
    std::ofstream csv_file_, all_csv_file_;
    std::string csv_path_, all_csv_path_;
    rclcpp::Time start_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlePoseLogger>());
    rclcpp::shutdown();
    return 0;
}
