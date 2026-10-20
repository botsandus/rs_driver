/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include <array>
#include <cstdint>

namespace robosense { namespace lidar {

// Raw diagnostic telemetry, never a safety permit. Unknown codes stay raw.
struct SafetyAiryStatus
{
  bool valid = false;
  bool calibration_valid = false;
  bool imu_extrinsic_valid = false;
  double timestamp = 0;  // host receipt time, seconds
  uint8_t system_state = 0, fault_level = 0, active_group = 0xff;
  uint8_t sync_mode = 0, sync_status = 0;
  uint8_t warning_intrusion = 0, protected_intrusion = 0, occlusion = 0;
  uint16_t rpm = 0, actual_rpm = 0;
  std::array<uint8_t, 4> ossd{};
  std::array<uint8_t, 6> serial{};
  std::array<uint8_t, 24> firmware{};
  std::array<uint8_t, 20> faults{};
  std::array<float, 7> imu_extrinsic{};
  uint64_t msop_packets = 0, difop_packets = 0, imu_packets = 0;
  uint64_t crc_errors = 0, rejected_packets = 0, sequence_gaps = 0;
};

}}  // namespace robosense::lidar
