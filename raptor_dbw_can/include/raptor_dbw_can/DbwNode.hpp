// Copyright (c) 2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RAPTOR_DBW_CAN__DBWNODE_HPP_
#define RAPTOR_DBW_CAN__DBWNODE_HPP_

// std libraries
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

// ros
#include <rclcpp/rclcpp.hpp>

// libraries
#include <can_dbc_parser/Dbc.hpp>
#include <can_dbc_parser/DbcBuilder.hpp>
#include <can_dbc_parser/DbcMessage.hpp>
#include <can_dbc_parser/DbcSignal.hpp>
#include <raptor_dbw_can/DbwDispatch.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>

// ROS messages
#include <can_msgs/msg/frame.hpp>
#include <deep_orange_msgs/msg/ct_report.hpp>
#include <deep_orange_msgs/msg/diagnostic_report.hpp>
#include <deep_orange_msgs/msg/lap_time_report.hpp>
#include <deep_orange_msgs/msg/marelli_report.hpp>
#include <deep_orange_msgs/msg/misc_report.hpp>
#include <deep_orange_msgs/msg/my_laps_report.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>
#include <deep_orange_msgs/msg/race_control_report.hpp>
#include <deep_orange_msgs/msg/tire_report.hpp>
#include <deep_orange_msgs/msg/tire_temp_report.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_report.hpp>
#include <raptor_dbw_msgs/msg/brake2_report.hpp>
#include <raptor_dbw_msgs/msg/brake_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_extended_report.hpp>
#include <raptor_dbw_msgs/msg/steering_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

static const double GRAVITY = 9.81;

using namespace std::chrono_literals;  // NOLINT

namespace raptor_dbw_can {
class DbwNode : public rclcpp::Node {
   public:
    explicit DbwNode(const rclcpp::NodeOptions& options);
    ~DbwNode();

   private:
    void sendFrame(const can_msgs::msg::Frame& msg);
    void receive();
    void timerHighRateCallback();
    void timerLowRateCallback();
    void recvCAN(const can_msgs::msg::Frame::SharedPtr msg);
    void generateTireTemp();
    void recvBrakeCmd(const raptor_dbw_msgs::msg::BrakeCmd::SharedPtr msg);
    void recvAcceleratorPedalCmd(
        const raptor_dbw_msgs::msg::AcceleratorPedalCmd::SharedPtr msg);
    void recvSteeringCmd(
        const raptor_dbw_msgs::msg::SteeringCmd::SharedPtr msg);
    void recvGearShiftCmd(const std_msgs::msg::UInt8::SharedPtr msg);
    void recvCtReport(const deep_orange_msgs::msg::CtReport::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void recvDashSwitches(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr timer_high_rate_;
    rclcpp::TimerBase::SharedPtr timer_low_rate_;

    deep_orange_msgs::msg::PtReport pt_report_msg;
    deep_orange_msgs::msg::TireReport tire_report_msg;
    deep_orange_msgs::msg::MyLapsReport mylaps_report_msg;
    deep_orange_msgs::msg::TireTempReport tire_temp_report_msg;
    deep_orange_msgs::msg::MarelliReport marelli_report_msg;
    deep_orange_msgs::msg::RaceControlReport rc_report_msg;

    // Subscribed topics
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
    rclcpp::Subscription<raptor_dbw_msgs::msg::BrakeCmd>::SharedPtr sub_brake_;
    rclcpp::Subscription<raptor_dbw_msgs::msg::AcceleratorPedalCmd>::SharedPtr
        sub_accelerator_pedal_;
    rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringCmd>::SharedPtr
        sub_steering_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_gear_shift_cmd_;
    rclcpp::Subscription<deep_orange_msgs::msg::CtReport>::SharedPtr
        sub_ct_report_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr
        sub_dash_cmds_;

    //! Published topics
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_;
    rclcpp::Publisher<deep_orange_msgs::msg::RaceControlReport>::SharedPtr
        pub_rc_report_;
    rclcpp::Publisher<raptor_dbw_msgs::msg::AcceleratorPedalReport>::SharedPtr
        pub_accel_pedal_;
    rclcpp::Publisher<raptor_dbw_msgs::msg::SteeringReport>::SharedPtr
        pub_steering_;
    rclcpp::Publisher<raptor_dbw_msgs::msg::SteeringExtendedReport>::SharedPtr
        pub_steering_ext_;
    rclcpp::Publisher<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr
        pub_wheel_speeds_;
    //! publish wheel speeds separately
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        pub_front_left_wheel_speed_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        pub_front_right_wheel_speed_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        pub_back_left_wheel_speed_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        pub_back_right_wheel_speed_;
    rclcpp::Publisher<raptor_dbw_msgs::msg::Brake2Report>::SharedPtr
        pub_brake_2_report_;
    rclcpp::Publisher<deep_orange_msgs::msg::MiscReport>::SharedPtr
        pub_misc_do_;
    rclcpp::Publisher<deep_orange_msgs::msg::TireReport>::SharedPtr
        pub_tire_report_;
    rclcpp::Publisher<deep_orange_msgs::msg::PtReport>::SharedPtr
        pub_pt_report_;
    rclcpp::Publisher<deep_orange_msgs::msg::DiagnosticReport>::SharedPtr
        pub_diag_report_;
    rclcpp::Publisher<deep_orange_msgs::msg::LapTimeReport>::SharedPtr
        pub_timing_report_;
    rclcpp::Publisher<deep_orange_msgs::msg::MyLapsReport>::SharedPtr
        pub_mylaps_report_;
    rclcpp::Publisher<deep_orange_msgs::msg::TireTempReport>::SharedPtr
        pub_tire_temp_report_;
    rclcpp::Publisher<deep_orange_msgs::msg::MarelliReport>::SharedPtr
        pub_marelli_report_;

    NewEagle::Dbc dbwDbc_;
    std::string dbcFile_;
    double wheel_speed_cov_;
    static constexpr double kph2ms = 1.0 / 3.6;

    //! Driver Dash Switches States
    static constexpr uint8_t TRACTION_AIM_DEFAULT = 3;
    static constexpr uint8_t TRACTION_RANGE_DEFAULT = 3;
    uint8_t last_driver_traction_range_switch_ = TRACTION_RANGE_DEFAULT;
    uint8_t last_traction_aim_ = TRACTION_AIM_DEFAULT;

    //! CAN interfacing
    bool use_socketcan;
    bool use_bus_time_;
    std::string interface_;
    std::chrono::nanoseconds interval_ns_;
    std::unique_ptr<drivers::socketcan::SocketCanReceiver> can_receiver_;
    std::unique_ptr<drivers::socketcan::SocketCanSender> can_sender_;
    std::unique_ptr<std::thread> receiver_thread_;
};

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__DBWNODE_HPP_
