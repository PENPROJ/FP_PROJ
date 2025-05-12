#include <iostream>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>  // tf2::Quaternion 추가
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "tf2/LinearMath/Matrix3x3.h"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;


class su_rviz : public rclcpp::Node {
public:
    su_rviz() : Node("px4_rviz"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
   {
      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    //SUBSCRIBER GROUP
    px4_data_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/csv_data", qos_settings,  // Topic name and QoS depth
      std::bind(&su_rviz::px4_data_subscriber_callback, this, std::placeholders::_1));



      Linear_velocity_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/Linear_velocity_arrow", 10);

      Desired_Linear_velocity_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/Desired_Linear_velocity_arrow", 10);


      Acceleration_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/Acceleration_arrow", 10);

      Desired_Acceleration_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/Desired_Acceleration_arrow", 10);


        timer_ = this->create_wall_timer(
          10ms, std::bind(&su_rviz::visual_timer_callback, this));

      }


    private:
      void visual_timer_callback()
      {

        Position_tf_publisher();
        Desired_Position_tf_publisher();

        Linear_velocity_arrow_publisher();
        Desired_Linear_velocity_arrow_publisher();

        accel_arrow_publsiher();
        Desired_accel_arrow_publsiher();

      }

      void Position_tf_publisher()
      {
          geometry_msgs::msg::TransformStamped transform_msg;
          transform_msg.header.stamp = this->get_clock()->now();
          transform_msg.header.frame_id = "world";  // 부모 프레임
          transform_msg.child_frame_id = "desired_position"; // 자식 프레임 (EE)

          transform_msg.transform.translation.x = position[0];
          transform_msg.transform.translation.y = position[1];
          transform_msg.transform.translation.z = position[2];

          tf2::Quaternion quat;
          quat.setRPY(attitude[0], attitude[1], attitude[2]);

          transform_msg.transform.rotation.x = quat.x();
          transform_msg.transform.rotation.y = quat.y();
          transform_msg.transform.rotation.z = quat.z();
          transform_msg.transform.rotation.w = quat.w();

          // TF Broadcast
          tf_broadcaster_->sendTransform(transform_msg);
      }

      void Desired_Position_tf_publisher()
      {
          geometry_msgs::msg::TransformStamped transform_msg;
          transform_msg.header.stamp = this->get_clock()->now();
          transform_msg.header.frame_id = "world";  // 부모 프레임
          transform_msg.child_frame_id = "desired_position"; // 자식 프레임 (EE)

          transform_msg.transform.translation.x = desired_position[0];
          transform_msg.transform.translation.y = desired_position[1];
          transform_msg.transform.translation.z = desired_position[2];

          tf2::Quaternion quat;
          quat.setRPY(desired_attitude[0], desired_attitude[1], desired_attitude[2]);

          transform_msg.transform.rotation.x = quat.x();
          transform_msg.transform.rotation.y = quat.y();
          transform_msg.transform.rotation.z = quat.z();
          transform_msg.transform.rotation.w = quat.w();

          // TF Broadcast
          tf_broadcaster_->sendTransform(transform_msg);
      }


      void Linear_velocity_arrow_publisher()
      {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "Linear_velocity";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start_point, end_point;
        start_point.x = position[0];
        start_point.y = position[1];
        start_point.z = position[2];

        end_point.x = start_point.x + linear_velocity[0];
        end_point.y = start_point.y + linear_velocity[1];
        end_point.z = start_point.z + linear_velocity[2];

        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        marker.scale.x = 0.02;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        Linear_velocity_arrow_publisher_->publish(marker);
      }


      void Desired_Linear_velocity_arrow_publisher()
      {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "Desired_Linear_velocity";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start_point, end_point;
        start_point.x = position[0];
        start_point.y = position[1];
        start_point.z = position[2];

        end_point.x = start_point.x + desired_linear_velocity[0];
        end_point.y = start_point.y + desired_linear_velocity[1];
        end_point.z = start_point.z + desired_linear_velocity[2];

        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        marker.scale.x = 0.02;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        Desired_Linear_velocity_arrow_publisher_->publish(marker);
      }


      void accel_arrow_publsiher()
      {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "Force";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start_point, end_point;
        start_point.x = position[0];
        start_point.y = position[1];
        start_point.z = position[2];

        end_point.x = start_point.x + acceleration[0];
        end_point.y = start_point.y + acceleration[1];
        end_point.z = start_point.z + acceleration[2];

        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        marker.scale.x = 0.02;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        Acceleration_arrow_publisher_->publish(marker);
      }


      void Desired_accel_arrow_publsiher()
      {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "Desired_Force";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start_point, end_point;
        start_point.x = position[0];
        start_point.y = position[1];
        start_point.z = position[2];

        end_point.x = start_point.x + desired_acceleration[0];
        end_point.y = start_point.y + desired_acceleration[1];
        end_point.z = start_point.z + desired_acceleration[2];

        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        marker.scale.x = 0.02;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        Desired_Acceleration_arrow_publisher_->publish(marker);
      }



    void px4_data_subscriber_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
    //////////////////////////
    // Data Analysis SCHEME //
    //////////////////////////
    // 시각화 시 모든 데이터의 header frame은 world frame으로 정렬할 것임
    // 따라서, body frame 기준으로 정의된 토픽들은 R_B를 곱한 뒤에 시각화 하였을 때 정상적으로 rviz에 보일 것임.
    // ex) desired_angular_velocity = R_B * desired_angular_velocity; => RVIZ에서 profit 함 => desired_angular_velocity는 body frame 기준으로 잘 정의 됨.

    // Position [Global]
    position[0] = msg->data[0];
    position[1] = msg->data[1];
    position[2] = msg->data[2];

    desired_position[0] = msg->data[3];
    desired_position[1] = msg->data[4];
    desired_position[2] = msg->data[5];

    // Linear Velocity [Global]
    linear_velocity[0] = msg->data[6];
    linear_velocity[1] = msg->data[7];
    linear_velocity[2] = msg->data[8];

    desired_linear_velocity[0] = msg->data[9];
    desired_linear_velocity[1] = msg->data[10];
    desired_linear_velocity[2] = msg->data[11];

    // Attitude [Body (Current frame rpy)]
    attitude[0] = msg->data[12];
    attitude[1] = msg->data[13];
    attitude[2] = msg->data[14];
    set_Rotation_matrix();

    desired_attitude[0] = msg->data[15];
    desired_attitude[1] = msg->data[16];
    desired_attitude[2] = msg->data[17];

    // Angular Velocity [Body]
    angular_velocity[0] = msg->data[18];
    angular_velocity[1] = msg->data[19];
    angular_velocity[2] = msg->data[20];
    angular_velocity = R_B * angular_velocity;

    desired_angular_velocity[0] = msg->data[21];
    desired_angular_velocity[1] = msg->data[22];
    desired_angular_velocity[2] = msg->data[23];
    desired_angular_velocity = R_B * desired_angular_velocity;

    // Desired Force [Body]
    desired_force[0] = msg->data[24];
    desired_force[1] = msg->data[25];
    desired_force[2] = msg->data[26];
    desired_force = R_B * desired_force;
    // Desired Torque [Body]
    desired_torque[0] = msg->data[27];
    desired_torque[1] = msg->data[28];
    desired_torque[2] = msg->data[29];
    desired_torque = R_B * desired_torque;


    // Individual Motor Thrust [Trvial]
    individual_motor_thrust[0] = msg->data[30];
    individual_motor_thrust[1] = msg->data[31];
    individual_motor_thrust[2] = msg->data[32];
    individual_motor_thrust[3] = msg->data[33];

    // Servo Angle [Trvial]
    servo_angle[0] = msg->data[34];
    servo_angle[1] = msg->data[35];
    servo_angle[2] = msg->data[36];
    servo_angle[3] = msg->data[37];

    // Desired Servo Angle [Trvial]
    desired_servo_angle[0] = msg->data[38];
    desired_servo_angle[1] = msg->data[39];
    desired_servo_angle[2] = msg->data[40];
    desired_servo_angle[3] = msg->data[41];

    // Acceleration [Body]
    acceleration[0] = msg->data[42];
    acceleration[1] = msg->data[43];
    acceleration[2] = msg->data[44];
    acceleration = R_B * acceleration;

    // Desired Acceleration [Body]
    desired_acceleration[0] = msg->data[45];
    desired_acceleration[1] = msg->data[46];
    desired_acceleration[2] = msg->data[47];
    desired_acceleration = R_B *desired_acceleration;
    }

    void set_Rotation_matrix()
    {
      Rz << cos(attitude[2]), -sin(attitude[2]), 0,
            sin(attitude[2]),  cos(attitude[2]), 0,
                 0  ,       0  , 1;

      Ry << cos(attitude[1]), 0, sin(attitude[1]),
                   0 , 1,     0,
           -sin(attitude[1]), 0, cos(attitude[1]);

      Rx << 1,      0       ,       0,
            0, cos(attitude[0]), -sin(attitude[0]),
            0, sin(attitude[0]),  cos(attitude[0]);

      R_B = Rz * Ry * Rx;    /// Body 에서 Global로 변환해주는 Matrix이다.
                             /// Ex) [global value] = R_B * [body value]
    }


    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr Linear_velocity_arrow_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr Desired_Linear_velocity_arrow_publisher_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr Acceleration_arrow_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr Desired_Acceleration_arrow_publisher_;




    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr px4_data_subscriber;


    // Position
    Eigen::Vector3d position;                // data[0] ~ data[2]
    Eigen::Vector3d desired_position;        // data[3] ~ data[5]

    // Linear Velocity
    Eigen::Vector3d linear_velocity;         // data[6] ~ data[8]
    Eigen::Vector3d desired_linear_velocity; // data[9] ~ data[11]

    // Attitude (RPY)
    Eigen::Vector3d attitude;                // data[12] ~ data[14]
    Eigen::Vector3d desired_attitude;        // data[15] ~ data[17]

    // Angular Velocity
    Eigen::Vector3d angular_velocity;        // data[18] ~ data[20]
    Eigen::Vector3d desired_angular_velocity;// data[21] ~ data[23]

    // Desired Force
    Eigen::Vector3d desired_force;           // data[24] ~ data[26]

    // Desired Torque
    Eigen::Vector3d desired_torque;          // data[27] ~ data[29]

    // Individual Motor Thrust
    Eigen::VectorXd individual_motor_thrust = Eigen::VectorXd::Zero(4); // data[30] ~ data[33]

    // Servo Angle
    Eigen::VectorXd servo_angle = Eigen::VectorXd::Zero(4);             // data[34] ~ data[37]
    Eigen::VectorXd desired_servo_angle = Eigen::VectorXd::Zero(4);     // data[38] ~ data[41]

    // Acceleration
    Eigen::Vector3d acceleration;            // data[42] ~ data[44]
    Eigen::Vector3d desired_acceleration;    // data[45] ~ data[47]


    Eigen::Matrix3d Rz, Ry, Rx, R_B;


};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<su_rviz>());
    rclcpp::shutdown();
    return 0;
}
