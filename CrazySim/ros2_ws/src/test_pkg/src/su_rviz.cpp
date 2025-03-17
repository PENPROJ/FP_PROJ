#include <iostream>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>  // tf2::Quaternion 추가
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

using namespace std::chrono_literals;


class su_rviz : public rclcpp::Node {
public:
    su_rviz() : Node("su_rviz"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
   {
      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);




        cf_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/cf_1/pose", qos_settings,  // Topic name and QoS depth
          std::bind(&su_rviz::cf_pose_subscriber, this, std::placeholders::_1));



          timer_ = this->create_wall_timer(
            10ms, std::bind(&su_rviz::visual_timer_callback, this));

      }


    private:
      void visual_timer_callback()
      {
        cf_pose_broadcaster();
        
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

    void cf_pose_broadcaster() {

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "Body_frame";

        transformStamped.transform.translation.x = global_xyz_meas[0];
        transformStamped.transform.translation.y = global_xyz_meas[1];
        transformStamped.transform.translation.z = global_xyz_meas[2];

        tf2::Quaternion quat;
        quat.setRPY(body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);
        transformStamped.transform.rotation.x = quat.x();
        transformStamped.transform.rotation.y = quat.y();
        transformStamped.transform.rotation.z = quat.z();
        transformStamped.transform.rotation.w = quat.w();

        tf_broadcaster_->sendTransform(transformStamped);
    }

    
    

    rclcpp::TimerBase::SharedPtr timer_;



    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_subscriber_;


    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    Eigen::Vector3d global_xyz_meas;
    Eigen::Vector3d body_rpy_meas;




};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<su_rviz>());
    rclcpp::shutdown();
    return 0;
}