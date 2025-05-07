#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/wrench.pb.h>

using std::placeholders::_1;

class GzToRosWrenchBridge : public rclcpp::Node
{
public:
  GzToRosWrenchBridge()
  : Node("gz_to_ros_wrench_bridge")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/ee/force_wrench", 10);

    // Subscribe to Gazebo Transport topic
    node_.Subscribe("/gz/EE_forcetorque", &GzToRosWrenchBridge::gzCallback, this);

    RCLCPP_INFO(this->get_logger(), "Subscribed to /gz/EE_forcetorque, publishing to /ee/force_wrench");
  }

private:
  void gzCallback(const gz::msgs::Wrench &msg)
  {

    geometry_msgs::msg::Wrench wrench;

    wrench.force.x = msg.force().x();
    wrench.force.y = msg.force().y();
    wrench.force.z = msg.force().z();

    wrench.torque.x = msg.torque().x();
    wrench.torque.y = msg.torque().y();
    wrench.torque.z = msg.torque().z();

    if (std::abs(wrench.force.x - wrench_prev.force.x) < 0.002 &&
        std::abs(wrench.force.y - wrench_prev.force.y) < 0.002 &&
        std::abs(wrench.force.z - wrench_prev.force.z) < 0.002 &&
        std::abs(wrench.torque.x - wrench_prev.torque.x) < 0.002 &&
        std::abs(wrench.torque.y - wrench_prev.torque.y) < 0.002 &&
        std::abs(wrench.torque.z - wrench_prev.torque.z) < 0.002
        ) publisher_->publish(wrench);

    wrench_prev = wrench;
  }

  geometry_msgs::msg::Wrench wrench_prev;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
  gz::transport::Node node_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GzToRosWrenchBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}