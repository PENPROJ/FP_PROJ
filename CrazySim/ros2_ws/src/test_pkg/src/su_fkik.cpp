#include <iostream>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>  // tf2::Quaternion 추가
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

using namespace std::chrono_literals;


class su_fkik : public rclcpp::Node {
public:
su_fkik() : Node("su_fkik"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
   {
      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);




        cf_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/cf_1/pose", qos_settings,  // Topic name and QoS depth
          std::bind(&su_fkik::cf_pose_subscriber, this, std::placeholders::_1));

        cf_vel_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
          "/cf_1/velocity", qos_settings,
          std::bind(&su_fkik::cf_velocity_subscriber, this, std::placeholders::_1));



          timer_ = this->create_wall_timer(
            10ms, std::bind(&su_fkik::visual_timer_callback, this));

      }


    private:
      void visual_timer_callback()
      {
        cf_velocity_generator();
        
      }


    void cf_pose_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      global_xyz_meas[0] = msg->pose.position.x;
      global_xyz_meas[1] = msg->pose.position.y;
      global_xyz_meas[2] = msg->pose.position.z;

      // Convert quaternion to roll, pitch, yaw
      tf2::Quaternion quat(
          msg->pose.orientation.x,
          msg->pose.orientation.y,
          msg->pose.orientation.z,
          msg->pose.orientation.w);

      tf2::Matrix3x3 mat(quat);
      mat.getRPY(body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);
    }


    void cf_velocity_subscriber(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received velocity data:");
      for (const auto& val : msg->values) {
          RCLCPP_INFO(this->get_logger(), "%f", val);
      }
    }


    void cf_velocity_generator() {
    //TODO: global_xyz_meas, body_rpy_meas 를 조합해서 end effector velocity data를 만들기

    }

    
    

    rclcpp::TimerBase::SharedPtr timer_;



    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_subscriber_;
    rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_vel_subscriber_;


    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    Eigen::Vector3d global_xyz_meas;
    Eigen::Vector3d body_rpy_meas;




};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<su_fkik>());
    rclcpp::shutdown();
    return 0;
}