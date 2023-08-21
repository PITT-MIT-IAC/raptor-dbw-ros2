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

#ifndef ECU_CONTROL_CAN__DISPATCH_HPP_
#define ECU_CONTROL_CAN__DISPATCH_HPP_

#include <stdint.h>

namespace ecu_control_can2 {

#undef BUILD_ASSERT

enum {
    // taken from `/config/CAN2-INDY-V{x}.dbc`
    ID_M1_GENERAL_0x640 = 0x0640,
    ID_M1_GENERAL_0x641 = 0x0641,
    ID_M1_GENERAL_0X642 = 0x0642,
    ID_M1_GENERAL_0X644 = 0x0644,
    ID_M1_GENERAL_0X645 = 0x0645,
    ID_M1_GENERAL_0X648 = 0x0648,
    ID_M1_GENERAL_0X649 = 0x0649,
    ID_M1_GENERAL_0X64C = 0x064C,
    ID_M1_GENERAL_0X64D = 0x064D,
    ID_M1_GENERAL_0X64F = 0x064F,
    ID_M1_GENERAL_0X650 = 0x0650,
    ID_M1_GENERAL_0X651 = 0x0651,
    ID_M1_GENERAL_0X653 = 0x0653,
    ID_M1_GENERAL_0X659 = 0x0659,
    ID_M1_GENERAL_0X688 = 0x0688,

    ID_M1_GEAR_0X630 = 0x0630,
    ID_M1_GEAR_0X631 = 0x0631,

    ID_WHEEL_SPEED_FAULTS = 0x0018

};
}  // namespace ecu_control_can2

namespace ecu_control_can3 {

#undef BUILD_ASSERT

enum {
    // taken from `/config/CAN3-INDY-V{x}.dbc`
    ID_VECTOR__INDEPENDENT_SIG_MSG = 0xC0000000,
    ID_SHUTDOWN_SEQ = 0x0160,
    ID_REQ_BRAKE_POS = 0x0021,
    ID_REQ_STEER_POS = 0x0020,
    ID_STAT_BRK_3 = 0x0085,
    ID_STAT_BRK_2 = 0x0083,
    ID_STAT_BRK_1 = 0x0081,
    ID_STAT_STEER_3 = 0x0084,
    ID_STAT_STEER_2 = 0x0082,
    ID_STAT_STEER_1 = 0x0080,
    ID_ACT_ERR_BRAKE_3 = 0x0065,
    ID_ACT_ERR_BRAKE_2 = 0x0063,
    ID_ACT_ERR_STEER_3 = 0x0064,
    ID_ACT_ERR_STEER_2 = 0x0062,
    ID_ACT_ERR_BRAKE_1 = 0x0061,
    ID_ACT_ERR_STEER_1 = 0x0060,
    ID_CALB_STEER_POS_3 = 0x0124,
    ID_CALB_STEER_POS_2 = 0x0122,
    ID_CALB_BRAKE_POS_3 = 0x0124,
    ID_CALB_BRAKE_POS_2 = 0x0123,
    ID_CALB_BRAKE_POS_1 = 0x0121,
    ID_CALB_STEER_POS_1 = 0x0120,
    ID_SPEED_OVER_GND_NOVATEL = 0x89F80201,

    // taken from `/config/CAN3-TTPMS-INDY-V{x}.dbc
    ID_RR_TTPMS_6 = 0x061D,
    ID_RR_TTPMS_5 = 0x061C,
    ID_RR_TTPMS_4 = 0x061B,
    ID_RR_TTPMS_3 = 0x061A,
    ID_RR_TTPMS_2 = 0x0619,
    ID_RR_TTPMS_1 = 0x0618,

    ID_LR_TTPMS_6 = 0x0617,
    ID_LR_TTPMS_5 = 0x0616,
    ID_LR_TTPMS_4 = 0x0615,
    ID_LR_TTPMS_3 = 0x0614,
    ID_LR_TTPMS_2 = 0x0613,
    ID_LR_TTPMS_1 = 0x0612,

    ID_RF_TTPMS_6 = 0x0611,
    ID_RF_TTPMS_5 = 0x0610,
    ID_RF_TTPMS_4 = 0x060F,
    ID_RF_TTPMS_3 = 0x060E,
    ID_RF_TTPMS_2 = 0x060D,
    ID_RF_TTPMS_1 = 0x060C,

    ID_LF_TTPMS_6 = 0x060B,
    ID_LF_TTPMS_5 = 0x060A,
    ID_LF_TTPMS_4 = 0x0609,
    ID_LF_TTPMS_3 = 0x0608,
    ID_LF_TTPMS_2 = 0x0607,
    ID_LF_TTPMS_1 = 0x0606

};
}  // namespace ecu_control_can3

#endif  // ECU_CONTROL_CAN__DISPATCH_HPP_
