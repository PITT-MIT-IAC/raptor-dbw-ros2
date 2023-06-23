// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights
// reserved.
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

#include <raptor_dbw_can/DbwNode.hpp>

namespace raptor_dbw_can {

DbwNode::DbwNode(const rclcpp::NodeOptions& options)
    : Node("raptor_dbw_can_node", options) {
    dbcFile_ = this->declare_parameter<std::string>("dbw_dbc_file", "");
    wheel_speed_cov_ =
        this->declare_parameter<double>("wheel_speed_covariance", 0.0004);

    // Initializing tire report
    // set sizes for the arrays
    for (int i = 0; i < 16; i++) {
        tire_report_msg.fl_tire_temperature.push_back(-1.0);
        tire_report_msg.fr_tire_temperature.push_back(-1.0);
        tire_report_msg.rl_tire_temperature.push_back(-1.0);
        tire_report_msg.rr_tire_temperature.push_back(-1.0);
    }

    // Set up Publishers
    pub_can_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 20);
    pub_rc_report_ =
        this->create_publisher<deep_orange_msgs::msg::RaceControlReport>(
            "race_control_report", 20);
    pub_mylaps_report_ =
        this->create_publisher<deep_orange_msgs::msg::MyLapsReport>(
            "mylaps_report", 20);
    pub_accel_pedal_ =
        this->create_publisher<raptor_dbw_msgs::msg::AcceleratorPedalReport>(
            "accelerator_pedal_report", 20);
    pub_steering_ =
        this->create_publisher<raptor_dbw_msgs::msg::SteeringReport>(
            "steering_report", 20);
    pub_steering_ext_ =
        this->create_publisher<raptor_dbw_msgs::msg::SteeringExtendedReport>(
            "steering_extended_report", 20);
    pub_wheel_speeds_ =
        this->create_publisher<raptor_dbw_msgs::msg::WheelSpeedReport>(
            "wheel_speed_report", 20);

    // publish wheel speeds as separate topics
    pub_front_left_wheel_speed_ =
        this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "wheel_speed/front_left", 20);
    pub_front_right_wheel_speed_ =
        this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "wheel_speed/front_right", 20);
    pub_back_left_wheel_speed_ =
        this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "wheel_speed/rear_left", 20);
    pub_back_right_wheel_speed_ =
        this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "wheel_speed/rear_right", 20);

    pub_brake_2_report_ =
        this->create_publisher<raptor_dbw_msgs::msg::Brake2Report>(
            "brake_2_report", 20);
    pub_misc_do_ = this->create_publisher<deep_orange_msgs::msg::MiscReport>(
        "misc_report_do", 10);
    pub_tire_report_ =
        this->create_publisher<deep_orange_msgs::msg::TireReport>("tire_report",
                                                                  10);
    pub_pt_report_ = this->create_publisher<deep_orange_msgs::msg::PtReport>(
        "pt_report", 10);
    pub_diag_report_ =
        this->create_publisher<deep_orange_msgs::msg::DiagnosticReport>(
            "diag_report", 10);
    pub_timing_report_ =
        this->create_publisher<deep_orange_msgs::msg::LapTimeReport>(
            "lap_time_report", 10);
    pub_tire_temp_report_ =
        this->create_publisher<deep_orange_msgs::msg::TireTempReport>(
            "tire_temp_report", 10);
    pub_marelli_report_ =
        this->create_publisher<deep_orange_msgs::msg::MarelliReport>(
            "marelli_report", 10);

    // Set up Subscribers
    sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
        "can_rx", 500,
        std::bind(&DbwNode::recvCAN, this, std::placeholders::_1));

    sub_brake_ = this->create_subscription<raptor_dbw_msgs::msg::BrakeCmd>(
        "brake_cmd", 1,
        std::bind(&DbwNode::recvBrakeCmd, this, std::placeholders::_1));

    sub_accelerator_pedal_ =
        this->create_subscription<raptor_dbw_msgs::msg::AcceleratorPedalCmd>(
            "accelerator_pedal_cmd", 1,
            std::bind(&DbwNode::recvAcceleratorPedalCmd, this,
                      std::placeholders::_1));

    sub_steering_ =
        this->create_subscription<raptor_dbw_msgs::msg::SteeringCmd>(
            "steering_cmd", 1,
            std::bind(&DbwNode::recvSteeringCmd, this, std::placeholders::_1));

    sub_gear_shift_cmd_ = this->create_subscription<std_msgs::msg::UInt8>(
        "gear_cmd", 10,
        std::bind(&DbwNode::recvGearShiftCmd, this, std::placeholders::_1));

    sub_ct_report_ = this->create_subscription<deep_orange_msgs::msg::CtReport>(
        "ct_report", 1,
        std::bind(&DbwNode::recvCtReport, this, std::placeholders::_1));

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/novatel_top/rawimux", 1,
        std::bind(&DbwNode::imuCallback, this, std::placeholders::_1));

    sub_dash_cmds_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "dashboard_cmd", 1,
        std::bind(&DbwNode::recvDashSwitches, this, std::placeholders::_1));

    dbwDbc_ = NewEagle::DbcBuilder().NewDbc(dbcFile_);

    // Set up Timer
    timer_tire_report_ = this->create_wall_timer(
        10ms, std::bind(&DbwNode::timerTireCallback, this));
    timer_pt_report_ = this->create_wall_timer(
        10ms, std::bind(&DbwNode::timerPtCallback, this));
    timer_mylaps_report_ = this->create_wall_timer(
        200ms, std::bind(&DbwNode::timerMyLapsReportCallback, this));
}

DbwNode::~DbwNode() {}

void DbwNode::recvCAN(const can_msgs::msg::Frame::SharedPtr msg) {
    if (!msg->is_rtr && !msg->is_error) {
        switch (msg->id) {
            case ID_LAP_TIME_REPORT: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_LAP_TIME_REPORT);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);

                    deep_orange_msgs::msg::LapTimeReport out;
                    out.stamp = msg->header.stamp;
                    out.laps = message->GetSignal("laps")->GetResult();
                    out.lap_time = message->GetSignal("lap_time")->GetResult();
                    out.time_stamp =
                        message->GetSignal("time_stamp")->GetResult();
                    pub_timing_report_->publish(out);
                }
            } break;

            case ID_MYLAPS_REPORT_1: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_MYLAPS_REPORT_1);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    mylaps_report_msg.stamp = msg->header.stamp;
                    mylaps_report_msg.speed =
                        message->GetSignal("mylaps_speed")->GetResult();
                    mylaps_report_msg.heading =
                        message->GetSignal("mylaps_heading")->GetResult();
                }
            } break;

            case ID_MYLAPS_REPORT_2: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_MYLAPS_REPORT_2);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    mylaps_report_msg.stamp = msg->header.stamp;
                    mylaps_report_msg.lat =
                        message->GetSignal("Latitude")->GetResult();
                    mylaps_report_msg.lon =
                        message->GetSignal("Longitude")->GetResult();
                }
            } break;

            case ID_BASE_TO_CAR_SUMMARY: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_BASE_TO_CAR_SUMMARY);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);

                    // publish race control report
                    rc_report_msg.stamp = msg->header.stamp;
                    rc_report_msg.base_to_car_heartbeat =
                        message->GetSignal("base_to_car_heartbeat")
                            ->GetResult();
                    rc_report_msg.track_flag =
                        message->GetSignal("track_flag")->GetResult();
                    rc_report_msg.veh_flag =
                        message->GetSignal("veh_flag")->GetResult();
                    rc_report_msg.veh_rank =
                        message->GetSignal("veh_rank")->GetResult();
                    rc_report_msg.lap_count =
                        message->GetSignal("lap_count")->GetResult();
                    rc_report_msg.lap_distance =
                        message->GetSignal("lap_distance")->GetResult();
                    rc_report_msg.round_target_speed =
                        message->GetSignal("round_target_speed")->GetResult();

                    pub_rc_report_->publish(rc_report_msg);
                }
            } break;

            case ID_MARELLI_REPORT_1: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_MARELLI_REPORT_1);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);

                    auto track_flag =
                        message->GetSignal("marelli_track_flag")->GetResult();
                    auto vehicle_flag =
                        message->GetSignal("marelli_vehicle_flag")->GetResult();
                    auto sector_flag =
                        message->GetSignal("marelli_sector_flag")->GetResult();
                    auto lte_sync_ok =
                        message->GetSignal("marelli_rc_base_sync_check")
                            ->GetResult();
                    auto lte_modem_lte_rssi =
                        message->GetSignal("marelli_rc_lte_rssi")->GetResult();

                    // acknowledge flags
                    NewEagle::DbcMessage* ct_report_2_message =
                        dbwDbc_.GetMessage("ct_report_2");
                    ct_report_2_message->GetSignal("marelli_track_flag_ack")
                        ->SetResult(track_flag);
                    ct_report_2_message->GetSignal("marelli_vehicle_flag_ack")
                        ->SetResult(vehicle_flag);
                    ct_report_2_message->GetSignal("marelli_sector_flag_ack")
                        ->SetResult(sector_flag);
                    can_msgs::msg::Frame frame =
                        ct_report_2_message->GetFrame();
                    pub_can_->publish(frame);

                    // publish race control report
                    rc_report_msg.stamp = msg->header.stamp;
                    rc_report_msg.marelli_track_flag = track_flag;
                    rc_report_msg.marelli_vehicle_flag = vehicle_flag;
                    rc_report_msg.marelli_sector_flag = sector_flag;
                    rc_report_msg.lte_modem_lte_rssi = lte_modem_lte_rssi;
                    rc_report_msg.lte_sync_ok = lte_sync_ok;
                    pub_rc_report_->publish(rc_report_msg);

                    // set marelli report fields
                    marelli_report_msg.lte_sync_ok = lte_sync_ok;
                    marelli_report_msg.lte_modem_lte_rssi = lte_modem_lte_rssi;
                    marelli_report_msg.stamp = msg->header.stamp;
                }
            } break;

            case ID_MARELLI_REPORT_2: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_MARELLI_REPORT_2);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    marelli_report_msg.lat =
                        static_cast<float>(message->GetSignal("marelli_gps_lat")
                                               ->GetResult()) *
                        1e7;
                    marelli_report_msg.lon =
                        static_cast<float>(
                            message->GetSignal("marelli_gps_long")
                                ->GetResult()) *
                        1e7;
                    marelli_report_msg.stamp = msg->header.stamp;
                    pub_marelli_report_->publish(marelli_report_msg);
                }
            } break;

            case ID_WHEEL_SPEED_REPORT_DO: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_WHEEL_SPEED_REPORT_DO);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);

                    raptor_dbw_msgs::msg::WheelSpeedReport out;
                    out.header.stamp = msg->header.stamp;

                    // TODO fix this once we have an idea of how we want to do
                    // this
                    std::string frame_id = "base_link";
                    geometry_msgs::msg::TwistWithCovarianceStamped
                        front_left_speed_msg;
                    geometry_msgs::msg::TwistWithCovarianceStamped
                        front_right_speed_msg;
                    geometry_msgs::msg::TwistWithCovarianceStamped
                        back_left_speed_msg;
                    geometry_msgs::msg::TwistWithCovarianceStamped
                        back_right_speed_msg;

                    front_left_speed_msg.header.frame_id = frame_id;
                    front_left_speed_msg.header.stamp = msg->header.stamp;
                    front_right_speed_msg.header.frame_id = frame_id;
                    front_right_speed_msg.header.stamp = msg->header.stamp;
                    back_left_speed_msg.header.frame_id = frame_id;
                    back_left_speed_msg.header.stamp = msg->header.stamp;
                    back_right_speed_msg.header.frame_id = frame_id;
                    back_right_speed_msg.header.stamp = msg->header.stamp;

                    out.front_left =
                        message->GetSignal("wheel_speed_FL")->GetResult();
                    out.front_right =
                        message->GetSignal("wheel_speed_FR")->GetResult();
                    out.rear_left =
                        message->GetSignal("wheel_speed_RL")->GetResult();
                    out.rear_right =
                        message->GetSignal("wheel_speed_RR")->GetResult();

                    front_left_speed_msg.twist.twist.linear.x =
                        out.front_left * kph2ms;
                    front_left_speed_msg.twist.covariance[0] = wheel_speed_cov_;
                    front_left_speed_msg.twist.covariance[7] = wheel_speed_cov_;
                    front_left_speed_msg.twist.covariance[14] =
                        wheel_speed_cov_;

                    front_right_speed_msg.twist.twist.linear.x =
                        out.front_right * kph2ms;
                    front_right_speed_msg.twist.covariance[0] =
                        wheel_speed_cov_;
                    front_right_speed_msg.twist.covariance[7] =
                        wheel_speed_cov_;
                    front_right_speed_msg.twist.covariance[14] =
                        wheel_speed_cov_;

                    back_left_speed_msg.twist.twist.linear.x =
                        out.rear_left * kph2ms;
                    back_left_speed_msg.twist.covariance[0] = wheel_speed_cov_;
                    back_left_speed_msg.twist.covariance[7] = wheel_speed_cov_;
                    back_left_speed_msg.twist.covariance[14] = wheel_speed_cov_;

                    back_right_speed_msg.twist.twist.linear.x =
                        out.rear_right * kph2ms;
                    back_right_speed_msg.twist.covariance[0] = wheel_speed_cov_;
                    back_right_speed_msg.twist.covariance[7] = wheel_speed_cov_;
                    back_right_speed_msg.twist.covariance[14] =
                        wheel_speed_cov_;

                    pub_wheel_speeds_->publish(out);
                    pub_front_left_wheel_speed_->publish(front_left_speed_msg);
                    pub_front_right_wheel_speed_->publish(
                        front_right_speed_msg);
                    pub_back_left_wheel_speed_->publish(back_left_speed_msg);
                    pub_back_right_wheel_speed_->publish(back_right_speed_msg);
                }
            } break;

            case ID_BRAKE_PRESSURE_REPORT_DO: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_BRAKE_PRESSURE_REPORT_DO);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);

                    raptor_dbw_msgs::msg::Brake2Report out;
                    out.header.stamp = msg->header.stamp;

                    out.front_brake_pressure =
                        message->GetSignal("brake_pressure_fdbk_front")
                            ->GetResult();
                    out.rear_brake_pressure =
                        message->GetSignal("brake_pressure_fdbk_rear")
                            ->GetResult();
                    out.rolling_counter =
                        message->GetSignal("brk_pressure_fdbk_counter")
                            ->GetResult();
                    pub_brake_2_report_->publish(out);
                }
            } break;

            case ID_ACCELERATOR_REPORT_DO: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_ACCELERATOR_REPORT_DO);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);

                    raptor_dbw_msgs::msg::AcceleratorPedalReport out;
                    out.header.stamp = msg->header.stamp;
                    out.pedal_output =
                        message->GetSignal("acc_pedal_fdbk")->GetResult();
                    out.rolling_counter =
                        message->GetSignal("acc_pedal_fdbk_counter")
                            ->GetResult();
                    pub_accel_pedal_->publish(out);
                }
            } break;

            case ID_STEERING_REPORT_DO: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_STEERING_REPORT_DO);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);

                    raptor_dbw_msgs::msg::SteeringReport out;
                    out.header.stamp = msg->header.stamp;

                    out.steering_wheel_angle =
                        message->GetSignal("steering_motor_ang_avg_fdbk")
                            ->GetResult();
                    out.rolling_counter =
                        message->GetSignal("steering_motor_fdbk_counter")
                            ->GetResult();
                    pub_steering_->publish(out);
                }
            } break;

            case ID_STEERING_REPORT_EXTD: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_STEERING_REPORT_EXTD);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    raptor_dbw_msgs::msg::SteeringExtendedReport out;
                    out.header.stamp = msg->header.stamp;

                    out.steering_motor_ang_1 =
                        message->GetSignal("steering_motor_ang_1_fdbk")
                            ->GetResult();
                    out.steering_motor_ang_2 =
                        message->GetSignal("steering_motor_ang_2_fdbk")
                            ->GetResult();
                    out.steering_motor_ang_3 =
                        message->GetSignal("steering_motor_ang_3_fdbk")
                            ->GetResult();
                    pub_steering_ext_->publish(out);
                }
            } break;

            case ID_MISC_REPORT_DO: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_MISC_REPORT_DO);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);

                    deep_orange_msgs::msg::MiscReport out;
                    out.stamp = msg->header.stamp;
                    out.sys_state =
                        message->GetSignal("sys_state")->GetResult();
                    out.safety_switch_state =
                        message->GetSignal("safety_switch_state")->GetResult();
                    out.mode_switch_state =
                        message->GetSignal("mode_switch_state")->GetResult();
                    out.battery_voltage =
                        message->GetSignal("battery_voltage")->GetResult();
                    pub_misc_do_->publish(out);
                }
            } break;

            case ID_DIAG_REPORT: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_DIAG_REPORT);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);

                    deep_orange_msgs::msg::DiagnosticReport out;
                    out.stamp = msg->header.stamp;
                    out.sd_system_warning =
                        message->GetSignal("sd_system_warning")->GetResult();
                    out.sd_system_failure =
                        message->GetSignal("sd_system_failure")->GetResult();
                    out.motec_warning =
                        message->GetSignal("motec_warning")->GetResult();
                    out.sd_brake_warning1 =
                        message->GetSignal("sd_brake_warning1")->GetResult();
                    out.sd_brake_warning2 =
                        message->GetSignal("sd_brake_warning2")->GetResult();
                    out.sd_brake_warning3 =
                        message->GetSignal("sd_brake_warning3")->GetResult();
                    out.sd_steer_warning1 =
                        message->GetSignal("sd_steer_warning1")->GetResult();
                    out.sd_steer_warning2 =
                        message->GetSignal("sd_steer_warning2")->GetResult();
                    out.sd_steer_warning3 =
                        message->GetSignal("sd_steer_warning3")->GetResult();
                    out.est1_oos_front_brk =
                        message->GetSignal("est1_oos_front_brk")->GetResult();
                    out.est2_oos_rear_brk =
                        message->GetSignal("est2_oos_rear_brk")->GetResult();
                    out.est3_low_eng_speed =
                        message->GetSignal("est3_low_eng_speed")->GetResult();
                    out.est4_sd_comms_loss =
                        message->GetSignal("est4_sd_comms_loss")->GetResult();
                    out.est5_motec_comms_loss =
                        message->GetSignal("est5_motec_comms_loss")
                            ->GetResult();
                    out.est6_sd_ebrake =
                        message->GetSignal("est6_sd_ebrake")->GetResult();
                    out.adlink_hb_lost =
                        message->GetSignal("adlink_hb_lost")->GetResult();
                    out.rc_lost = message->GetSignal("rc_lost")->GetResult();

                    pub_diag_report_->publish(out);
                }
            } break;

            case ID_PT_REPORT_1: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_PT_REPORT_1);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    pt_report_msg.stamp = msg->header.stamp;
                    pt_report_msg.throttle_position =
                        message->GetSignal("throttle_position")->GetResult();
                    pt_report_msg.engine_run_switch_status =
                        message->GetSignal("engine_run_switch")->GetResult();
                    pt_report_msg.current_gear =
                        message->GetSignal("current_gear")->GetResult();
                    pt_report_msg.engine_rpm =
                        message->GetSignal("engine_speed_rpm")->GetResult();
                    pt_report_msg.vehicle_speed_kmph =
                        message->GetSignal("vehicle_speed_kmph")->GetResult();
                }
            } break;

            case ID_PT_REPORT_2: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_PT_REPORT_2);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);

                    pt_report_msg.fuel_pressure =
                        message->GetSignal("fuel_pressure_kPa")->GetResult();
                    pt_report_msg.engine_oil_pressure =
                        message->GetSignal("engine_oil_pressure_kPa")
                            ->GetResult();
                    pt_report_msg.engine_coolant_temperature =
                        message->GetSignal("coolant_temperature")->GetResult();
                    pt_report_msg.transmission_oil_temperature =
                        message->GetSignal("transmission_temperature")
                            ->GetResult();
                    pt_report_msg.transmission_oil_pressure =
                        message->GetSignal("transmission_pressure_kPa")
                            ->GetResult();
                }
            } break;

            case ID_PT_REPORT_3: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_PT_REPORT_3);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);

                    pt_report_msg.engine_oil_temperature =
                        message->GetSignal("engine_oil_temperature")
                            ->GetResult();
                    pt_report_msg.torque_wheels =
                        message->GetSignal("torque_wheels")->GetResult();
                    pt_report_msg.driver_traction_aim_switch =
                        message->GetSignal("driver_traction_aim_swicth_fbk")
                            ->GetResult();
                    pt_report_msg.driver_traction_range_switch =
                        message->GetSignal("driver_traction_range_switch_fbk")
                            ->GetResult();
                }
            } break;

            case ID_TIRE_PRESSURE_FL: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_PRESSURE_FL);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.fl_tire_pressure =
                        message->GetSignal("FL_Tire_Pressure")->GetResult();
                    tire_report_msg.fl_tire_pressure_gauge =
                        message->GetSignal("FL_Tire_Pressure_Gauge")
                            ->GetResult();
                }
            } break;

            case ID_TIRE_PRESSURE_FR: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_PRESSURE_FR);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.fr_tire_pressure =
                        message->GetSignal("FR_Tire_Pressure")->GetResult();
                    tire_report_msg.fr_tire_pressure_gauge =
                        message->GetSignal("FR_Tire_Pressure_Gauge")
                            ->GetResult();
                }
            } break;

            case ID_TIRE_PRESSURE_RL: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_PRESSURE_RL);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.rl_tire_pressure =
                        message->GetSignal("RL_Tire_Pressure")->GetResult();
                    tire_report_msg.rl_tire_pressure_gauge =
                        message->GetSignal("RL_Tire_Pressure_Gauge")
                            ->GetResult();
                }
            } break;

            case ID_TIRE_PRESSURE_RR: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_PRESSURE_RR);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.rr_tire_pressure =
                        message->GetSignal("RR_Tire_Pressure")->GetResult();
                    tire_report_msg.rr_tire_pressure_gauge =
                        message->GetSignal("RR_Tire_Pressure_Gauge")
                            ->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_FL_1: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_FL_1);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.fl_tire_temperature[0] =
                        message->GetSignal("FL_Tire_Temp_01")->GetResult();
                    tire_report_msg.fl_tire_temperature[1] =
                        message->GetSignal("FL_Tire_Temp_02")->GetResult();
                    tire_report_msg.fl_tire_temperature[2] =
                        message->GetSignal("FL_Tire_Temp_03")->GetResult();
                    tire_report_msg.fl_tire_temperature[3] =
                        message->GetSignal("FL_Tire_Temp_04")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_FL_2: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_FL_2);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.fl_tire_temperature[4] =
                        message->GetSignal("FL_Tire_Temp_05")->GetResult();
                    tire_report_msg.fl_tire_temperature[5] =
                        message->GetSignal("FL_Tire_Temp_06")->GetResult();
                    tire_report_msg.fl_tire_temperature[6] =
                        message->GetSignal("FL_Tire_Temp_07")->GetResult();
                    tire_report_msg.fl_tire_temperature[7] =
                        message->GetSignal("FL_Tire_Temp_08")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_FL_3: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_FL_3);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.fl_tire_temperature[8] =
                        message->GetSignal("FL_Tire_Temp_09")->GetResult();
                    tire_report_msg.fl_tire_temperature[9] =
                        message->GetSignal("FL_Tire_Temp_10")->GetResult();
                    tire_report_msg.fl_tire_temperature[10] =
                        message->GetSignal("FL_Tire_Temp_11")->GetResult();
                    tire_report_msg.fl_tire_temperature[11] =
                        message->GetSignal("FL_Tire_Temp_12")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_FL_4: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_FL_4);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.fl_tire_temperature[12] =
                        message->GetSignal("FL_Tire_Temp_13")->GetResult();
                    tire_report_msg.fl_tire_temperature[13] =
                        message->GetSignal("FL_Tire_Temp_14")->GetResult();
                    tire_report_msg.fl_tire_temperature[14] =
                        message->GetSignal("FL_Tire_Temp_15")->GetResult();
                    tire_report_msg.fl_tire_temperature[15] =
                        message->GetSignal("FL_Tire_Temp_16")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_FR_1: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_FR_1);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.fr_tire_temperature[0] =
                        message->GetSignal("FR_Tire_Temp_01")->GetResult();
                    tire_report_msg.fr_tire_temperature[1] =
                        message->GetSignal("FR_Tire_Temp_02")->GetResult();
                    tire_report_msg.fr_tire_temperature[2] =
                        message->GetSignal("FR_Tire_Temp_03")->GetResult();
                    tire_report_msg.fr_tire_temperature[3] =
                        message->GetSignal("FR_Tire_Temp_04")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_FR_2: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_FR_2);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.fr_tire_temperature[4] =
                        message->GetSignal("FR_Tire_Temp_05")->GetResult();
                    tire_report_msg.fr_tire_temperature[5] =
                        message->GetSignal("FR_Tire_Temp_06")->GetResult();
                    tire_report_msg.fr_tire_temperature[6] =
                        message->GetSignal("FR_Tire_Temp_07")->GetResult();
                    tire_report_msg.fr_tire_temperature[7] =
                        message->GetSignal("FR_Tire_Temp_08")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_FR_3: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_FR_3);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.fr_tire_temperature[8] =
                        message->GetSignal("FR_Tire_Temp_09")->GetResult();
                    tire_report_msg.fr_tire_temperature[9] =
                        message->GetSignal("FR_Tire_Temp_10")->GetResult();
                    tire_report_msg.fr_tire_temperature[10] =
                        message->GetSignal("FR_Tire_Temp_11")->GetResult();
                    tire_report_msg.fr_tire_temperature[11] =
                        message->GetSignal("FR_Tire_Temp_12")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_FR_4: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_FR_4);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.fr_tire_temperature[12] =
                        message->GetSignal("FR_Tire_Temp_13")->GetResult();
                    tire_report_msg.fr_tire_temperature[13] =
                        message->GetSignal("FR_Tire_Temp_14")->GetResult();
                    tire_report_msg.fr_tire_temperature[14] =
                        message->GetSignal("FR_Tire_Temp_15")->GetResult();
                    tire_report_msg.fr_tire_temperature[15] =
                        message->GetSignal("FR_Tire_Temp_16")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_RL_1: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_RL_1);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.rl_tire_temperature[0] =
                        message->GetSignal("RL_Tire_Temp_01")->GetResult();
                    tire_report_msg.rl_tire_temperature[1] =
                        message->GetSignal("RL_Tire_Temp_02")->GetResult();
                    tire_report_msg.rl_tire_temperature[2] =
                        message->GetSignal("RL_Tire_Temp_03")->GetResult();
                    tire_report_msg.rl_tire_temperature[3] =
                        message->GetSignal("RL_Tire_Temp_04")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_RL_2: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_RL_2);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.rl_tire_temperature[4] =
                        message->GetSignal("RL_Tire_Temp_05")->GetResult();
                    tire_report_msg.rl_tire_temperature[5] =
                        message->GetSignal("RL_Tire_Temp_06")->GetResult();
                    tire_report_msg.rl_tire_temperature[6] =
                        message->GetSignal("RL_Tire_Temp_07")->GetResult();
                    tire_report_msg.rl_tire_temperature[7] =
                        message->GetSignal("RL_Tire_Temp_08")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_RL_3: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_RL_3);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.rl_tire_temperature[8] =
                        message->GetSignal("RL_Tire_Temp_09")->GetResult();
                    tire_report_msg.rl_tire_temperature[9] =
                        message->GetSignal("RL_Tire_Temp_10")->GetResult();
                    tire_report_msg.rl_tire_temperature[10] =
                        message->GetSignal("RL_Tire_Temp_11")->GetResult();
                    tire_report_msg.rl_tire_temperature[11] =
                        message->GetSignal("RL_Tire_Temp_12")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_RL_4: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_RL_4);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.rl_tire_temperature[12] =
                        message->GetSignal("RL_Tire_Temp_13")->GetResult();
                    tire_report_msg.rl_tire_temperature[13] =
                        message->GetSignal("RL_Tire_Temp_14")->GetResult();
                    tire_report_msg.rl_tire_temperature[14] =
                        message->GetSignal("RL_Tire_Temp_15")->GetResult();
                    tire_report_msg.rl_tire_temperature[15] =
                        message->GetSignal("RL_Tire_Temp_16")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_RR_1: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_RR_1);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.rr_tire_temperature[0] =
                        message->GetSignal("RR_Tire_Temp_01")->GetResult();
                    tire_report_msg.rr_tire_temperature[1] =
                        message->GetSignal("RR_Tire_Temp_02")->GetResult();
                    tire_report_msg.rr_tire_temperature[2] =
                        message->GetSignal("RR_Tire_Temp_03")->GetResult();
                    tire_report_msg.rr_tire_temperature[3] =
                        message->GetSignal("RR_Tire_Temp_04")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_RR_2: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_RR_2);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.rr_tire_temperature[4] =
                        message->GetSignal("RR_Tire_Temp_05")->GetResult();
                    tire_report_msg.rr_tire_temperature[5] =
                        message->GetSignal("RR_Tire_Temp_06")->GetResult();
                    tire_report_msg.rr_tire_temperature[6] =
                        message->GetSignal("RR_Tire_Temp_07")->GetResult();
                    tire_report_msg.rr_tire_temperature[7] =
                        message->GetSignal("RR_Tire_Temp_08")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_RR_3: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_RR_3);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.rr_tire_temperature[8] =
                        message->GetSignal("RR_Tire_Temp_09")->GetResult();
                    tire_report_msg.rr_tire_temperature[9] =
                        message->GetSignal("RR_Tire_Temp_10")->GetResult();
                    tire_report_msg.rr_tire_temperature[10] =
                        message->GetSignal("RR_Tire_Temp_11")->GetResult();
                    tire_report_msg.rr_tire_temperature[11] =
                        message->GetSignal("RR_Tire_Temp_12")->GetResult();
                }
            } break;

            case ID_TIRE_TEMP_RR_4: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_TIRE_TEMP_RR_4);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.rr_tire_temperature[12] =
                        message->GetSignal("RR_Tire_Temp_13")->GetResult();
                    tire_report_msg.rr_tire_temperature[13] =
                        message->GetSignal("RR_Tire_Temp_14")->GetResult();
                    tire_report_msg.rr_tire_temperature[14] =
                        message->GetSignal("RR_Tire_Temp_15")->GetResult();
                    tire_report_msg.rr_tire_temperature[15] =
                        message->GetSignal("RR_Tire_Temp_16")->GetResult();
                }
            } break;

            case ID_WHEEL_STRAIN_GAUGE: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_WHEEL_STRAIN_GAUGE);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.fl_wheel_load =
                        message->GetSignal("wheel_strain_gauge_FL")
                            ->GetResult();
                    tire_report_msg.fr_wheel_load =
                        message->GetSignal("wheel_strain_gauge_FR")
                            ->GetResult();
                    tire_report_msg.rl_wheel_load =
                        message->GetSignal("wheel_strain_gauge_RL")
                            ->GetResult();
                    tire_report_msg.rr_wheel_load =
                        message->GetSignal("wheel_strain_gauge_RR")
                            ->GetResult();
                }
            } break;

            case ID_WHEEL_POTENTIOMETER: {
                NewEagle::DbcMessage* message =
                    dbwDbc_.GetMessageById(ID_WHEEL_POTENTIOMETER);
                if (msg->dlc >= message->GetDlc()) {
                    message->SetFrame(msg);
                    tire_report_msg.stamp = msg->header.stamp;
                    tire_report_msg.fl_damper_linear_potentiometer =
                        message->GetSignal("wheel_potentiometer_FL")
                            ->GetResult();
                    tire_report_msg.fr_damper_linear_potentiometer =
                        message->GetSignal("wheel_potentiometer_FR")
                            ->GetResult();
                    tire_report_msg.rl_damper_linear_potentiometer =
                        message->GetSignal("wheel_potentiometer_RL")
                            ->GetResult();
                    tire_report_msg.rr_damper_linear_potentiometer =
                        message->GetSignal("wheel_potentiometer_RR")
                            ->GetResult();
                }
            } break;
        }
    }
}

// computes values of TireTemp message for a given array of tire temperatures.
void DbwNode::generateTireTemp() {
    std::vector<float> fl_temps = tire_report_msg.fl_tire_temperature;
    std::vector<float> fr_temps = tire_report_msg.fr_tire_temperature;
    std::vector<float> rl_temps = tire_report_msg.rl_tire_temperature;
    std::vector<float> rr_temps = tire_report_msg.rr_tire_temperature;

    tire_temp_report_msg.front_left.mean = 0;
    tire_temp_report_msg.front_right.mean = 0;
    tire_temp_report_msg.rear_left.mean = 0;
    tire_temp_report_msg.rear_right.mean = 0;

    for (int i = 0; i < 16; i++) {
        float temp = tire_report_msg.fl_tire_temperature[i];
        tire_temp_report_msg.front_left.mean += temp;

        temp = tire_report_msg.fr_tire_temperature[i];
        tire_temp_report_msg.front_right.mean += temp;

        temp = tire_report_msg.rl_tire_temperature[i];
        tire_temp_report_msg.rear_left.mean += temp;

        temp = tire_report_msg.rr_tire_temperature[i];
        tire_temp_report_msg.rear_right.mean += temp;
    }

    std::sort(fl_temps.begin(), fl_temps.end());
    std::sort(fr_temps.begin(), fr_temps.end());
    std::sort(rl_temps.begin(), rl_temps.end());
    std::sort(rr_temps.begin(), rr_temps.end());

    tire_temp_report_msg.front_left.median =
        (fl_temps[15 / 2] + fl_temps[16 / 2]) / 2.0;
    tire_temp_report_msg.front_right.median =
        (fr_temps[15 / 2] + fr_temps[16 / 2]) / 2.0;
    tire_temp_report_msg.rear_left.median =
        (rl_temps[15 / 2] + fl_temps[16 / 2]) / 2.0;
    tire_temp_report_msg.rear_right.median =
        (rr_temps[15 / 2] + rr_temps[16 / 2]) / 2.0;

    tire_temp_report_msg.front_left.mean /= 16;
    tire_temp_report_msg.front_right.mean /= 16;
    tire_temp_report_msg.rear_left.mean /= 16;
    tire_temp_report_msg.rear_right.mean /= 16;

    tire_temp_report_msg.front_left.min = fl_temps[0];
    tire_temp_report_msg.front_right.min = fr_temps[0];
    tire_temp_report_msg.rear_left.min = rl_temps[0];
    tire_temp_report_msg.rear_right.min = rr_temps[0];

    tire_temp_report_msg.front_left.max = fl_temps[15];
    tire_temp_report_msg.front_right.max = fr_temps[15];
    tire_temp_report_msg.rear_left.max = rl_temps[15];
    tire_temp_report_msg.rear_right.max = rr_temps[15];
}

void DbwNode::recvBrakeCmd(
    const raptor_dbw_msgs::msg::BrakeCmd::SharedPtr msg) {
    NewEagle::DbcMessage* message = dbwDbc_.GetMessage("brake_pressure_cmd");
    message->GetSignal("brake_pressure_cmd")->SetResult(msg->pedal_cmd);
    message->GetSignal("brk_pressure_cmd_counter")
        ->SetResult(msg->rolling_counter);
    can_msgs::msg::Frame frame = message->GetFrame();
    pub_can_->publish(frame);
}

void DbwNode::recvAcceleratorPedalCmd(
    const raptor_dbw_msgs::msg::AcceleratorPedalCmd::SharedPtr msg) {
    NewEagle::DbcMessage* message = dbwDbc_.GetMessage("accelerator_cmd");
    message->GetSignal("acc_pedal_cmd")->SetResult(msg->pedal_cmd);
    message->GetSignal("acc_pedal_cmd_counter")
        ->SetResult(msg->rolling_counter);
    can_msgs::msg::Frame frame = message->GetFrame();
    pub_can_->publish(frame);
}

void DbwNode::recvSteeringCmd(
    const raptor_dbw_msgs::msg::SteeringCmd::SharedPtr msg) {
    NewEagle::DbcMessage* message = dbwDbc_.GetMessage("steering_cmd");
    message->GetSignal("steering_motor_ang_cmd")->SetResult(msg->angle_cmd);
    message->GetSignal("steering_motor_cmd_counter")
        ->SetResult(msg->rolling_counter);
    can_msgs::msg::Frame frame = message->GetFrame();
    pub_can_->publish(frame);
}

void DbwNode::recvCtReport(
    const deep_orange_msgs::msg::CtReport::SharedPtr msg) {
    NewEagle::DbcMessage* message = dbwDbc_.GetMessage("ct_report");
    message->GetSignal("track_cond_ack")->SetResult(msg->track_cond_ack);
    message->GetSignal("veh_sig_ack")->SetResult(msg->veh_sig_ack);
    message->GetSignal("ct_state")->SetResult(msg->ct_state);
    message->GetSignal("ct_state_rolling_counter")
        ->SetResult(msg->rolling_counter);

    can_msgs::msg::Frame frame = message->GetFrame();

    pub_can_->publish(frame);
}

void DbwNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    NewEagle::DbcMessage* message =
        dbwDbc_.GetMessage("ct_vehicle_acc_feedback");
    message->GetSignal("long_ct_vehicle_acc_fbk")
        ->SetResult(msg->linear_acceleration.x / GRAVITY);
    message->GetSignal("lat_ct_vehicle_acc_fbk")
        ->SetResult(msg->linear_acceleration.y / GRAVITY);
    message->GetSignal("vertical_ct_vehicle_acc_fbk")
        ->SetResult((msg->linear_acceleration.z - GRAVITY) / GRAVITY);
    can_msgs::msg::Frame frame = message->GetFrame();
    pub_can_->publish(frame);
}

void DbwNode::recvDashSwitches(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    NewEagle::DbcMessage* message = dbwDbc_.GetMessage("dash_switches_cmd");
    if (msg->data.size() >= 2) {
        switch (msg->data[0]) {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
                last_traction_aim_ = msg->data[0];
                break;
            default:
                last_traction_aim_ = TRACTION_AIM_DEFAULT;
                break;
        }
        switch (msg->data[1]) {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
                last_driver_traction_range_switch_ = msg->data[1];
                break;
            default:
                last_driver_traction_range_switch_ = TRACTION_RANGE_DEFAULT;
                break;
        }
    }
    can_msgs::msg::Frame frame = message->GetFrame();
    pub_can_->publish(frame);
}

void DbwNode::recvGearShiftCmd(const std_msgs::msg::UInt8::SharedPtr msg) {
    NewEagle::DbcMessage* message = dbwDbc_.GetMessage("gear_shift_cmd");
    message->GetSignal("desired_gear")->SetResult(msg->data);
    can_msgs::msg::Frame frame = message->GetFrame();
    pub_can_->publish(frame);
}

void DbwNode::timerTireCallback() {
    pub_tire_report_->publish(tire_report_msg);
    generateTireTemp();
    pub_tire_temp_report_->publish(tire_temp_report_msg);

    NewEagle::DbcMessage* message = dbwDbc_.GetMessage("dash_switches_cmd");
    message->GetSignal("driver_traction_aim_switch")
        ->SetResult(last_traction_aim_);
    message->GetSignal("driver_traction_range_switch")
        ->SetResult(last_driver_traction_range_switch_);
    can_msgs::msg::Frame frame = message->GetFrame();
    pub_can_->publish(frame);
}

void DbwNode::timerPtCallback() { pub_pt_report_->publish(pt_report_msg); }

void DbwNode::timerMyLapsReportCallback() {
    pub_mylaps_report_->publish(mylaps_report_msg);
}

}  // namespace raptor_dbw_can
