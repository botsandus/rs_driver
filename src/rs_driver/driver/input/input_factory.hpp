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

#include <rs_driver/driver/input/input.hpp>
#include <rs_driver/driver/input/input_sock.hpp>
#include <rs_driver/driver/input/input_pcap.hpp>
#include <rs_driver/driver/input/input_raw.hpp>

namespace robosense
{
namespace lidar
{

class InputFactory
{
public:
  static std::shared_ptr<Input> createInput(InputType type, 
      const RSInputParam& param, const std::function<void(const Error&)>& excb,
      double sec_to_delay);
};

inline std::shared_ptr<Input> InputFactory::createInput(InputType type, 
    const RSInputParam& param, const std::function<void(const Error&)>& excb, 
    double sec_to_delay)
{
  std::shared_ptr<Input> input;

  switch(type)
  {
    case InputType::ONLINE_LIDAR:
      {
        input = std::make_shared<InputSock>(param, excb);
      }
      break;

    case InputType::PCAP_FILE:
      {
        input = std::make_shared<InputPcap>(param, excb, sec_to_delay);
      }
      break;

    case InputType::RAW_PACKET:
      {
        input = std::make_shared<InputRaw>(param, excb);
      }
      break;

    default:
      RS_ERROR << "Wrong Input Type." << RS_REND;
      exit(-1);
  }

  return input;
}

}  // namespace lidar
}  // namespace robosense
