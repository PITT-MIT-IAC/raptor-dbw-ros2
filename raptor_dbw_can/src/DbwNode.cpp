// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
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

#include "raptor_dbw_can/DbwNode.hpp"
#include <iostream>

#include <algorithm>
#include <cmath>
#include <string>

namespace raptor_dbw_can
{

DbwNode::DbwNode(const rclcpp::NodeOptions & options)
: Node("raptor_dbw_can_node", options)
{
  dbcFile_ = this->declare_parameter("dbw_dbc_file", "");

  // Initializing tire report 
  // set sizes for the arrays
  for (int i=0;i<16;i++) 
  {
    tire_report_msg.fl_tire_temperature.push_back(-1.0);
    tire_report_msg.fr_tire_temperature.push_back(-1.0);
    tire_report_msg.rl_tire_temperature.push_back(-1.0);
    tire_report_msg.rr_tire_temperature.push_back(-1.0);
  }

  // Set up Publishers
  pub_can_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 20);
  pub_flags_ = this->create_publisher<deep_orange_msgs::msg::BaseToCarSummary>("flag_report", 20);
  pub_accel_pedal_ = this->create_publisher<raptor_dbw_msgs::msg::AcceleratorPedalReport>("accelerator_pedal_report", 20);
  pub_steering_ = this->create_publisher<raptor_dbw_msgs::msg::SteeringReport>("steering_report", 20);
  pub_steering_ext_ = this->create_publisher<raptor_dbw_msgs::msg::SteeringExtendedReport>("steering_extended_report", 20);
  pub_wheel_speeds_ = this->create_publisher<raptor_dbw_msgs::msg::WheelSpeedReport>("wheel_speed_report", 20);
  pub_brake_2_report_ = this->create_publisher<raptor_dbw_msgs::msg::Brake2Report>("brake_2_report", 20);
 
  pub_misc_do_ = this->create_publisher<deep_orange_msgs::msg::MiscReport>("misc_report_do", 10);
  pub_rc_to_ct_ = this->create_publisher<deep_orange_msgs::msg::RcToCt>("rc_to_ct", 10);
  pub_tire_report_ = this->create_publisher<deep_orange_msgs::msg::TireReport>("tire_report", 10);
  pub_pt_report_ = this->create_publisher<deep_orange_msgs::msg::PtReport>("pt_report", 10);

  // Set up Subscribers
  sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_rx", 500, std::bind(&DbwNode::recvCAN, this, std::placeholders::_1));

  sub_brake_ = this->create_subscription<raptor_dbw_msgs::msg::BrakeCmd>(
    "brake_cmd", 1, std::bind(&DbwNode::recvBrakeCmd, this, std::placeholders::_1));

  sub_accelerator_pedal_ = this->create_subscription<raptor_dbw_msgs::msg::AcceleratorPedalCmd>(
    "accelerator_pedal_cmd", 1, std::bind(&DbwNode::recvAcceleratorPedalCmd, this, std::placeholders::_1));

  sub_steering_ = this->create_subscription<raptor_dbw_msgs::msg::SteeringCmd>(
    "steering_cmd", 1, std::bind(&DbwNode::recvSteeringCmd, this, std::placeholders::_1));

  sub_gear_shift_cmd_ = this->create_subscription<std_msgs::msg::UInt8>(
      "gear_cmd", 10, std::bind(&DbwNode::recvGearShiftCmd, this, std::placeholders::_1));

  sub_ct_report_ = this->create_subscription<deep_orange_msgs::msg::CtReport>(
      "ct_report", 1, std::bind(&DbwNode::recvCtReport, this, std::placeholders::_1));


  dbwDbc_ = NewEagle::DbcBuilder().NewDbc(dbcFile_);

  // Set up Timer
  timer_tire_report_ = this->create_wall_timer(10ms, std::bind(&DbwNode::timerTireCallback, this));
  timer_pt_report_ = this->create_wall_timer(10ms, std::bind(&DbwNode::timerPtCallback, this));
}

DbwNode::~DbwNode()
{
}

void DbwNode::recvCAN(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (!msg->is_rtr && !msg->is_error) {
    switch (msg->id) {
      case ID_BASE_TO_CAR_SUMMARY:
        {
          NewEagle::DbcMessage * message = dbwDbc_.GetMessageById(ID_BASE_TO_CAR_SUMMARY);
          if (msg->dlc >= message->GetDlc()) {
            message->SetFrame(msg);

            deep_orange_msgs::msg::BaseToCarSummary out;
            out.stamp = msg->header.stamp;
            out.base_to_car_heartbeat = message->GetSignal("base_to_car_heartbeat")->GetResult();
            out.track_flag = message->GetSignal("track_flag")->GetResult();
            out.veh_flag = message->GetSignal("veh_flag")->GetResult();
            out.veh_rank = message->GetSignal("veh_rank")->GetResult();
            out.lap_status_whole = message->GetSignal("lap_status_whole")->GetResult();
            out.lap_status_fraction = message->GetSignal("lap_status_fraction")->GetResult();

            pub_flags_->publish(out);

            deep_orange_msgs::msg::RcToCt out2;
            out2.stamp = msg->header.stamp;
            out2.track_cond = message->GetSignal("track_flag")->GetResult(); 
            out2.rolling_counter = message->GetSignal("base_to_car_heartbeat")->GetResult();
            pub_rc_to_ct_->publish(out2);

          }
        }
        break;

      case ID_WHEEL_SPEED_REPORT_DO:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_WHEEL_SPEED_REPORT_DO);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            raptor_dbw_msgs::msg::WheelSpeedReport out;
            out.header.stamp = msg->header.stamp;

            out.front_left  = message->GetSignal("wheel_speed_FL")->GetResult();
            out.front_right = message->GetSignal("wheel_speed_FR")->GetResult();
            out.rear_left = message->GetSignal("wheel_speed_RL")->GetResult();
            out.rear_right = message->GetSignal("wheel_speed_RR")->GetResult();

            pub_wheel_speeds_->publish(out);
          }
        }
        break;

      case ID_BRAKE_PRESSURE_REPORT_DO:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_BRAKE_PRESSURE_REPORT_DO);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            raptor_dbw_msgs::msg::Brake2Report out;
            out.header.stamp = msg->header.stamp;


            out.front_brake_pressure  = message->GetSignal("brake_pressure_fdbk_front")->GetResult();
            out.rear_brake_pressure  = message->GetSignal("brake_pressure_fdbk_rear")->GetResult();
            out.rolling_counter = message->GetSignal("brk_pressure_fdbk_counter")->GetResult();
            pub_brake_2_report_->publish(out);
          }
        }
        break;

      case ID_ACCELERATOR_REPORT_DO:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_ACCELERATOR_REPORT_DO);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            raptor_dbw_msgs::msg::AcceleratorPedalReport out;
            out.header.stamp = msg->header.stamp;
            out.pedal_output  = message->GetSignal("acc_pedal_fdbk")->GetResult();
            out.rolling_counter = message->GetSignal("acc_pedal_fdbk_counter")->GetResult(); 
            pub_accel_pedal_->publish(out);
          }
        }
        break;

      case ID_STEERING_REPORT_DO:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_STEERING_REPORT_DO);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            

            raptor_dbw_msgs::msg::SteeringReport out;
            out.header.stamp = msg->header.stamp;

            out.steering_wheel_angle  = message->GetSignal("steering_motor_ang_avg_fdbk")->GetResult();
            out.rolling_counter = message->GetSignal("steering_motor_fdbk_counter")->GetResult(); 
            pub_steering_->publish(out);
          }
        }
        break;

      case ID_STEERING_REPORT_EXTD:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_STEERING_REPORT_EXTD);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            raptor_dbw_msgs::msg::SteeringExtendedReport out;
            out.header.stamp = msg->header.stamp;

            out.steering_motor_ang_1  = message->GetSignal("steering_motor_ang_1_fdbk")->GetResult();
            out.steering_motor_ang_2  = message->GetSignal("steering_motor_ang_2_fdbk")->GetResult();
            out.steering_motor_ang_3  = message->GetSignal("steering_motor_ang_3_fdbk")->GetResult();
            pub_steering_ext_->publish(out);
          }
        }
        break;

      case ID_MISC_REPORT_DO:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_MISC_REPORT_DO);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            deep_orange_msgs::msg::MiscReport out;
            out.stamp = msg->header.stamp;
            out.sys_state = message->GetSignal("sys_state")->GetResult(); 
            out.safety_switch_state = message->GetSignal("safety_switch_state")->GetResult(); 
            out.mode_switch_state = message->GetSignal("mode_switch_state")->GetResult();
            out.battery_voltage = message->GetSignal("battery_voltage")->GetResult();
            pub_misc_do_->publish(out);
          }
        }
        break;

      case ID_RC_TO_CT:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_RC_TO_CT);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            deep_orange_msgs::msg::RcToCt out;
            out.stamp = msg->header.stamp;
            // out.current_position  = message->GetSignal("DBW_CurrentPosition")->GetResult();
            out.track_cond = message->GetSignal("track_cond")->GetResult(); 
            // TODO: adding statements for arrays of black checkered purple flags trackpositions
            out.rolling_counter = message->GetSignal("rc_rolling_counter")->GetResult();
            pub_rc_to_ct_->publish(out);
          }
        }
        break;

      case ID_PT_REPORT_1:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_PT_REPORT_1);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            pt_report_msg.stamp = msg->header.stamp;
            pt_report_msg.throttle_position = message->GetSignal("throttle_position")->GetResult();
            pt_report_msg.engine_run_switch_status = message->GetSignal("engine_run_switch")->GetResult();
            pt_report_msg.current_gear = message->GetSignal("current_gear")->GetResult();
            pt_report_msg.engine_rpm = message->GetSignal("engine_speed_rpm")->GetResult();
            pt_report_msg.vehicle_speed_kmph = message->GetSignal("vehicle_speed_kmph")->GetResult();
          }
        }
        break;

      case ID_PT_REPORT_2:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_PT_REPORT_2);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);

            pt_report_msg.fuel_pressure = message->GetSignal("fuel_pressure_kPa")->GetResult();
            pt_report_msg.engine_oil_pressure = message->GetSignal("engine_oil_pressure_kPa")->GetResult();
            pt_report_msg.engine_coolant_temperature = message->GetSignal("coolant_temperature")->GetResult();
            pt_report_msg.transmission_oil_temperature = message->GetSignal("transmission_temperature")->GetResult();
            pt_report_msg.transmission_oil_pressure = message->GetSignal("transmission_pressure_kPa")->GetResult();
          }
        }
        break;

      case ID_TIRE_PRESSURE_FL:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_PRESSURE_FL);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.fl_tire_pressure = message->GetSignal("FL_Tire_Pressure")->GetResult();
            tire_report_msg.fl_tire_pressure_gauge = message->GetSignal("FL_Tire_Pressure_Gauge")->GetResult();
          }
        }
        break;

      case ID_TIRE_PRESSURE_FR:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_PRESSURE_FR);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.fr_tire_pressure = message->GetSignal("FR_Tire_Pressure")->GetResult();
            tire_report_msg.fr_tire_pressure_gauge = message->GetSignal("FR_Tire_Pressure_Gauge")->GetResult();
          }
        }
        break;

      case ID_TIRE_PRESSURE_RL:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_PRESSURE_RL);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.rl_tire_pressure = message->GetSignal("RL_Tire_Pressure")->GetResult();
            tire_report_msg.rl_tire_pressure_gauge = message->GetSignal("RL_Tire_Pressure_Gauge")->GetResult();
          }
        }
        break;
      
      case ID_TIRE_PRESSURE_RR:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_PRESSURE_RR);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.rr_tire_pressure = message->GetSignal("RR_Tire_Pressure")->GetResult();
            tire_report_msg.rr_tire_pressure_gauge = message->GetSignal("RR_Tire_Pressure_Gauge")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_FL_1:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_FL_1);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.fl_tire_temperature[0] = message->GetSignal("FL_Tire_Temp_01")->GetResult();
            tire_report_msg.fl_tire_temperature[1] = message->GetSignal("FL_Tire_Temp_02")->GetResult();
            tire_report_msg.fl_tire_temperature[2] = message->GetSignal("FL_Tire_Temp_03")->GetResult();
            tire_report_msg.fl_tire_temperature[3] = message->GetSignal("FL_Tire_Temp_04")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_FL_2:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_FL_2);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.fl_tire_temperature[4] = message->GetSignal("FL_Tire_Temp_05")->GetResult();
            tire_report_msg.fl_tire_temperature[5] = message->GetSignal("FL_Tire_Temp_06")->GetResult();
            tire_report_msg.fl_tire_temperature[6] = message->GetSignal("FL_Tire_Temp_07")->GetResult();
            tire_report_msg.fl_tire_temperature[7] = message->GetSignal("FL_Tire_Temp_08")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_FL_3:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_FL_3);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.fl_tire_temperature[8] = message->GetSignal("FL_Tire_Temp_09")->GetResult();
            tire_report_msg.fl_tire_temperature[9] = message->GetSignal("FL_Tire_Temp_10")->GetResult();
            tire_report_msg.fl_tire_temperature[10] = message->GetSignal("FL_Tire_Temp_11")->GetResult();
            tire_report_msg.fl_tire_temperature[11] = message->GetSignal("FL_Tire_Temp_12")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_FL_4:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_FL_4);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.fl_tire_temperature[12] = message->GetSignal("FL_Tire_Temp_13")->GetResult();
            tire_report_msg.fl_tire_temperature[13] = message->GetSignal("FL_Tire_Temp_14")->GetResult();
            tire_report_msg.fl_tire_temperature[14] = message->GetSignal("FL_Tire_Temp_15")->GetResult();
            tire_report_msg.fl_tire_temperature[15] = message->GetSignal("FL_Tire_Temp_16")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_FR_1:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_FR_1);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.fr_tire_temperature[0] = message->GetSignal("FR_Tire_Temp_01")->GetResult();
            tire_report_msg.fr_tire_temperature[1] = message->GetSignal("FR_Tire_Temp_02")->GetResult();
            tire_report_msg.fr_tire_temperature[2] = message->GetSignal("FR_Tire_Temp_03")->GetResult();
            tire_report_msg.fr_tire_temperature[3] = message->GetSignal("FR_Tire_Temp_04")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_FR_2:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_FR_2);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.fr_tire_temperature[4] = message->GetSignal("FR_Tire_Temp_05")->GetResult();
            tire_report_msg.fr_tire_temperature[5] = message->GetSignal("FR_Tire_Temp_06")->GetResult();
            tire_report_msg.fr_tire_temperature[6] = message->GetSignal("FR_Tire_Temp_07")->GetResult();
            tire_report_msg.fr_tire_temperature[7] = message->GetSignal("FR_Tire_Temp_08")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_FR_3:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_FR_3);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.fr_tire_temperature[8] = message->GetSignal("FR_Tire_Temp_09")->GetResult();
            tire_report_msg.fr_tire_temperature[9] = message->GetSignal("FR_Tire_Temp_10")->GetResult();
            tire_report_msg.fr_tire_temperature[10] = message->GetSignal("FR_Tire_Temp_11")->GetResult();
            tire_report_msg.fr_tire_temperature[11] = message->GetSignal("FR_Tire_Temp_12")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_FR_4:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_FR_4);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.fr_tire_temperature[12] = message->GetSignal("FR_Tire_Temp_13")->GetResult();
            tire_report_msg.fr_tire_temperature[13] = message->GetSignal("FR_Tire_Temp_14")->GetResult();
            tire_report_msg.fr_tire_temperature[14] = message->GetSignal("FR_Tire_Temp_15")->GetResult();
            tire_report_msg.fr_tire_temperature[15] = message->GetSignal("FR_Tire_Temp_16")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_RL_1:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_RL_1);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.rl_tire_temperature[0] = message->GetSignal("RL_Tire_Temp_01")->GetResult();
            tire_report_msg.rl_tire_temperature[1] = message->GetSignal("RL_Tire_Temp_02")->GetResult();
            tire_report_msg.rl_tire_temperature[2] = message->GetSignal("RL_Tire_Temp_03")->GetResult();
            tire_report_msg.rl_tire_temperature[3] = message->GetSignal("RL_Tire_Temp_04")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_RL_2:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_RL_2);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.rl_tire_temperature[4] = message->GetSignal("RL_Tire_Temp_05")->GetResult();
            tire_report_msg.rl_tire_temperature[5] = message->GetSignal("RL_Tire_Temp_06")->GetResult();
            tire_report_msg.rl_tire_temperature[6] = message->GetSignal("RL_Tire_Temp_07")->GetResult();
            tire_report_msg.rl_tire_temperature[7] = message->GetSignal("RL_Tire_Temp_08")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_RL_3:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_RL_3);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.rl_tire_temperature[8] = message->GetSignal("RL_Tire_Temp_09")->GetResult();
            tire_report_msg.rl_tire_temperature[9] = message->GetSignal("RL_Tire_Temp_10")->GetResult();
            tire_report_msg.rl_tire_temperature[10] = message->GetSignal("RL_Tire_Temp_11")->GetResult();
            tire_report_msg.rl_tire_temperature[11] = message->GetSignal("RL_Tire_Temp_12")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_RL_4:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_RL_4);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.rl_tire_temperature[12] = message->GetSignal("RL_Tire_Temp_13")->GetResult();
            tire_report_msg.rl_tire_temperature[13] = message->GetSignal("RL_Tire_Temp_14")->GetResult();
            tire_report_msg.rl_tire_temperature[14] = message->GetSignal("RL_Tire_Temp_15")->GetResult();
            tire_report_msg.rl_tire_temperature[15] = message->GetSignal("RL_Tire_Temp_16")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_RR_1:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_RR_1);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.rr_tire_temperature[0] = message->GetSignal("RR_Tire_Temp_01")->GetResult();
            tire_report_msg.rr_tire_temperature[1] = message->GetSignal("RR_Tire_Temp_02")->GetResult();
            tire_report_msg.rr_tire_temperature[2] = message->GetSignal("RR_Tire_Temp_03")->GetResult();
            tire_report_msg.rr_tire_temperature[3] = message->GetSignal("RR_Tire_Temp_04")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_RR_2:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_RR_2);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.rr_tire_temperature[4] = message->GetSignal("RR_Tire_Temp_05")->GetResult();
            tire_report_msg.rr_tire_temperature[5] = message->GetSignal("RR_Tire_Temp_06")->GetResult();
            tire_report_msg.rr_tire_temperature[6] = message->GetSignal("RR_Tire_Temp_07")->GetResult();
            tire_report_msg.rr_tire_temperature[7] = message->GetSignal("RR_Tire_Temp_08")->GetResult();
          }
        }
        break;

      case ID_TIRE_TEMP_RR_3:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_RR_3);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.rr_tire_temperature[8] = message->GetSignal("RR_Tire_Temp_09")->GetResult();
            tire_report_msg.rr_tire_temperature[9] = message->GetSignal("RR_Tire_Temp_10")->GetResult();
            tire_report_msg.rr_tire_temperature[10] = message->GetSignal("RR_Tire_Temp_11")->GetResult();
            tire_report_msg.rr_tire_temperature[11] = message->GetSignal("RR_Tire_Temp_12")->GetResult();
          }
        }
        break;

        case ID_TIRE_TEMP_RR_4:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_TIRE_TEMP_RR_4);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg);
            tire_report_msg.rr_tire_temperature[12] = message->GetSignal("RR_Tire_Temp_13")->GetResult();
            tire_report_msg.rr_tire_temperature[13] = message->GetSignal("RR_Tire_Temp_14")->GetResult();
            tire_report_msg.rr_tire_temperature[14] = message->GetSignal("RR_Tire_Temp_15")->GetResult();
            tire_report_msg.rr_tire_temperature[15] = message->GetSignal("RR_Tire_Temp_16")->GetResult();
          }
        }
        break;
      
      case ID_WHEEL_STRAIN_GAUGE:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_WHEEL_STRAIN_GAUGE);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg); 
            tire_report_msg.fl_wheel_load = message->GetSignal("wheel_strain_gauge_FL")->GetResult();
            tire_report_msg.fr_wheel_load = message->GetSignal("wheel_strain_gauge_FR")->GetResult();
            tire_report_msg.rl_wheel_load = message->GetSignal("wheel_strain_gauge_RL")->GetResult();
            tire_report_msg.rr_wheel_load = message->GetSignal("wheel_strain_gauge_RR")->GetResult();

          }
        }
        break; 

      case ID_WHEEL_POTENTIOMETER:
        {
         NewEagle::DbcMessage* message = dbwDbc_.GetMessageById(ID_WHEEL_POTENTIOMETER);
          if (msg->dlc >= message->GetDlc()) {

            message->SetFrame(msg); 
            tire_report_msg.stamp = msg->header.stamp;
            tire_report_msg.fl_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_FL")->GetResult();
            tire_report_msg.fr_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_FR")->GetResult();
            tire_report_msg.rl_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_RL")->GetResult();
            tire_report_msg.rr_damper_linear_potentiometer = message->GetSignal("wheel_potentiometer_RR")->GetResult();

          }
        }
        break;  
    }
  }
}

  void DbwNode::recvBrakeCmd(const raptor_dbw_msgs::msg::BrakeCmd::SharedPtr msg)
  {
    NewEagle::DbcMessage * message = dbwDbc_.GetMessage("brake_pressure_cmd"); 
    message->GetSignal("brake_pressure_cmd")->SetResult(msg->pedal_cmd); 
    message->GetSignal("brk_pressure_cmd_counter")->SetResult(msg->rolling_counter);
    can_msgs::msg::Frame frame = message->GetFrame();
    pub_can_->publish(frame);
  }

  void DbwNode::recvAcceleratorPedalCmd(const raptor_dbw_msgs::msg::AcceleratorPedalCmd::SharedPtr msg)
  {
    NewEagle::DbcMessage * message = dbwDbc_.GetMessage("accelerator_cmd");
    message->GetSignal("acc_pedal_cmd")->SetResult(msg->pedal_cmd);
    message->GetSignal("acc_pedal_cmd_counter")->SetResult(msg->rolling_counter);
    can_msgs::msg::Frame frame = message->GetFrame();
    pub_can_->publish(frame);
  }

  void DbwNode::recvSteeringCmd(const raptor_dbw_msgs::msg::SteeringCmd::SharedPtr msg)
  {
    NewEagle::DbcMessage * message = dbwDbc_.GetMessage("steering_cmd");
    message->GetSignal("steering_motor_ang_cmd")->SetResult(msg->angle_cmd);
    message->GetSignal("steering_motor_cmd_counter")->SetResult(msg->rolling_counter);
    can_msgs::msg::Frame frame = message->GetFrame();
    pub_can_->publish(frame);
  }

  void DbwNode::recvCtReport(const deep_orange_msgs::msg::CtReport::SharedPtr msg) 
  {

    NewEagle::DbcMessage* message = dbwDbc_.GetMessage("ct_report");
    message->GetSignal("track_cond_ack")->SetResult(msg->track_cond_ack); 
    message->GetSignal("veh_sig_ack")->SetResult(msg->veh_sig_ack);
    message->GetSignal("ct_state")->SetResult(msg->ct_state);
    message->GetSignal("ct_state_rolling_counter")->SetResult(msg->rolling_counter);

    can_msgs::msg::Frame frame = message->GetFrame();

    pub_can_->publish(frame);
  }

void DbwNode::recvGearShiftCmd(const std_msgs::msg::UInt8::SharedPtr msg) 
{
  NewEagle::DbcMessage* message = dbwDbc_.GetMessage("gear_shift_cmd");
  message->GetSignal("desired_gear")->SetResult(msg->data);
  can_msgs::msg::Frame frame = message->GetFrame();
  pub_can_->publish(frame);
}

void DbwNode::timerTireCallback() {
    pub_tire_report_->publish(tire_report_msg);
}

void DbwNode::timerPtCallback() {
    pub_pt_report_->publish(pt_report_msg);
}

}  // namespace raptor_dbw_can
