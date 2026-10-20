// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <rs_driver/driver/decoder/decoder_mech.hpp>
#include <array>
#include <algorithm>
#include <limits>

namespace robosense
{
namespace lidar
{
// Byte arrays avoid unaligned integer loads from network packets.
struct RSSafetyAiryChannel
{
  uint8_t range[2];
  uint8_t intensity;
  uint8_t reserved;
};
struct RSSafetyAiryBlock
{
  uint8_t id[2];
  uint8_t azimuth[2];
  RSSafetyAiryChannel channels[48];
};
struct RSSafetyAiryMsop
{
  uint8_t header[42];
  RSSafetyAiryBlock blocks[6];
  uint8_t tail[22];
};
static_assert(sizeof(RSSafetyAiryChannel) == 4, "Safety Airy channel layout");
static_assert(sizeof(RSSafetyAiryBlock) == 196, "Safety Airy block layout");
static_assert(sizeof(RSSafetyAiryMsop) == 1240, "Safety Airy MSOP layout");

template <typename T_PointCloud>
class DecoderRSSafetyAIRY : public DecoderMech<T_PointCloud>
{
public:
  explicit DecoderRSSafetyAIRY(const RSDecoderParam& param)
    : DecoderMech<T_PointCloud>(getConstParam(), param)
  {
    this->packet_duration_ = 3 * this->mech_const_param_.BLOCK_DURATION;
    this->block_az_diff_ = 40;
  }

  void decodeDifopPkt(const uint8_t* pkt, size_t size) override;
  bool decodeMsopPkt(const uint8_t* pkt, size_t size) override;
  void decodeImuPkt(const uint8_t* pkt, size_t size) override;
  bool isNewFrame(const uint8_t* pkt) override;

  // Recovered from SafeParamTool 3.6.0; verified against captured MSOP/DIFOP.
  static uint32_t crc32(const uint8_t* data, size_t size)
  {
    static const std::array<uint32_t, 256> table = [] {
      std::array<uint32_t, 256> t{};
      for (uint32_t i = 0; i < 256; ++i)
      {
        uint32_t crc = i << 24;
        for (int bit = 0; bit < 8; ++bit)
          crc = (crc << 1) ^ ((crc & 0x80000000u) ? 0xf4acfb13u : 0u);
        t[i] = crc;
      }
      return t;
    }();
    uint32_t crc = 0;
    for (size_t i = 0; i < size; ++i)
      crc = (crc << 8) ^ table[(crc >> 24) ^ data[i]];
    return crc ^ 0xffffffffu;
  }

#ifndef UNIT_TEST
private:
#endif
  static uint16_t be16(const uint8_t* p) { return (uint16_t(p[0]) << 8) | p[1]; }
  static uint32_t be32(const uint8_t* p)
  { return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3]; }
  static int32_t signedBe32(const uint8_t* p)
  {
    uint32_t bits = be32(p);
    int32_t result;
    std::memcpy(&result, &bits, sizeof(result));
    return result;
  }
  static float floatBe32(const uint8_t* p)
  {
    uint32_t bits = be32(p);
    float result;
    std::memcpy(&result, &bits, sizeof(result));
    return result;
  }
  static double sensorTime(const uint8_t* p)
  {
    uint64_t seconds = 0;
    for (size_t i = 0; i < 6; ++i) seconds = (seconds << 8) | p[i];
    return double(seconds) + double(be32(p + 6)) * 1e-6;
  }
  static bool crcMatches(const uint8_t* pkt, size_t offset)
  { return crc32(pkt, offset) == be32(pkt + offset); }

  bool checkMsopCrc(const uint8_t* pkt, size_t size) override
  {
    const bool valid = size == 1240 && crcMatches(pkt, 1230);
    if (!valid) ++status_.crc_errors;
    return valid;
  }
  void reject(ErrCode code = ERRCODE_INVALIDPACKET)
  {
    ++status_.rejected_packets;
    if (this->cb_excep_) { LIMIT_CALL(this->cb_excep_(Error(code)), 1); }
  }
  double stamp(double sensor_seconds)
  {
    if (this->param_.use_lidar_clock) return sensor_seconds;
    const double now = getTimeHost() * 1e-6;
    // Anchor the sensor's common MSOP/IMU clock to the host once. Per-packet
    // receipt jitter must not make point offsets negative within a frame.
    if (!clock_ready_ || std::abs(sensor_seconds + host_offset_ - now) > 2.0)
    {
      host_offset_ = now - sensor_seconds;
      clock_ready_ = true;
      sequence_ready_ = false;
      if (this->point_cloud_) this->point_cloud_->points.clear();
    }
    return sensor_seconds + host_offset_;
  }
  static const RSDecoderMechConstParam& getConstParam()
  {
    static const RSDecoderMechConstParam param = [] {
      RSDecoderMechConstParam p{};
      p.base.MSOP_LEN = 1240; p.base.DIFOP_LEN = 1248;
      p.base.MSOP_ID_LEN = 4; p.base.DIFOP_ID_LEN = 8;
      const uint8_t msop[] = {0x55, 0xaa, 0x05, 0x5a};
      const uint8_t difop[] = {0xa5, 0xff, 0x00, 0x5a, 0x11, 0x11, 0x55, 0x55};
      const uint8_t imu[] = {0xaa, 0x55, 0x5a, 0x05};
      std::copy(msop, msop + 4, p.base.MSOP_ID);
      std::copy(difop, difop + 8, p.base.DIFOP_ID);
      std::copy(imu, imu + 4, p.base.IMU_ID);
      p.base.BLOCK_ID[0] = 0xff; p.base.BLOCK_ID[1] = 0xee;
      p.base.LASER_NUM = 96; p.base.BLOCKS_PER_PKT = 6; p.base.CHANNELS_PER_BLOCK = 48;
      p.base.DISTANCE_MIN = 0.1f; p.base.DISTANCE_MAX = 60.0f;
      p.base.DISTANCE_RES = 0.005f; p.base.IMU_LEN = 51; p.base.IMU_ID_LEN = 4;
      p.RX = 0.00979f; p.RY = 0.00068f; p.RZ = 0.04955f;
      p.BLOCK_DURATION = 111.080e-6;
      const double groups[] = {0, 7.616, 16.184, 24.752, 33.320, 42.840,
                               52.360, 61.880, 71.400, 79.968, 88.536, 97.104};
      for (size_t i = 0; i < 96; ++i)
      {
        p.CHAN_TSS[i] = groups[i / 8] * 1e-6;
        p.CHAN_AZIS[i] = groups[i / 8] / 111.080;
      }
      return p;
    }();
    return param;
  }
  bool validProfile(const uint8_t* p) const
  {
    return p[16] == 0 && p[17] == 1 && p[32] == 2 &&
           (p[31] == 0x10 || p[31] == 0x01);
  }
  SafetyAiryStatus status_;
  bool clock_ready_ = false, sequence_ready_ = false;
  double host_offset_ = 0;
  uint16_t sequence_ = 0;
};

template <typename T_PointCloud>
void DecoderRSSafetyAIRY<T_PointCloud>::decodeDifopPkt(const uint8_t* pkt, size_t size)
{
  if (size != 1248 || std::memcmp(pkt, this->const_param_.DIFOP_ID, 8) ||
      pkt[1246] != 0x0f || pkt[1247] != 0xf0)
  { reject(); return; }
  if (!crcMatches(pkt, 1242))
  { ++status_.crc_errors; reject(ERRCODE_WRONGCRC32); return; }

  status_.valid = true;
  status_.timestamp = getTimeHost() * 1e-6;
  status_.system_state = pkt[30]; status_.fault_level = pkt[31];
  status_.active_group = pkt[69];
  status_.sync_mode = pkt[301]; status_.sync_status = pkt[302];
  status_.warning_intrusion = pkt[313]; status_.protected_intrusion = pkt[314];
  status_.occlusion = pkt[379];
  status_.rpm = be16(pkt + 8); status_.actual_rpm = be16(pkt + 373);
  std::copy(pkt + 65, pkt + 69, status_.ossd.begin());
  std::copy(pkt + 292, pkt + 298, status_.serial.begin());
  std::copy(pkt + 40, pkt + 64, status_.firmware.begin());
  std::copy(pkt + 1214, pkt + 1234, status_.faults.begin());
  float quaternion_norm = 0;
  bool finite = true;
  for (size_t i = 0; i < 7; ++i)
  {
    const float value = floatBe32(pkt + 1092 + 4 * i);
    status_.imu_extrinsic[i] = value;
    finite &= std::isfinite(value);
    if (i < 4) quaternion_norm += value * value;
  }
  status_.imu_extrinsic_valid = finite && std::abs(quaternion_norm - 1.0f) < 0.05f;

  if (!this->param_.config_from_file)
  {
    RSCalibrationAngle vertical[96], horizontal[96];
    std::memcpy(vertical, pkt + 468, sizeof(vertical));
    std::memcpy(horizontal, pkt + 756, sizeof(horizontal));
    bool valid = false, signs_valid = true;
    for (size_t i = 0; i < 96; ++i)
    {
      valid |= be16(pkt + 469 + 3 * i) != 0;
      signs_valid &= pkt[468 + 3 * i] <= 1 && pkt[756 + 3 * i] <= 1;
    }
    this->angles_ready_ = valid && signs_valid &&
                          this->chan_angles_.loadFromDifop(vertical, horizontal) == 0;
    if (!this->angles_ready_) reject();
  }
  status_.calibration_valid = this->angles_ready_;
  if (status_.rpm > 0)
  {
    this->rps_ = std::max<uint16_t>(1, status_.rpm / 60);
    this->block_az_diff_ = uint16_t(std::round(36000.0 * status_.rpm / 60.0 *
                                               this->mech_const_param_.BLOCK_DURATION));
  }
  ++status_.difop_packets;
  if (this->cb_safety_status_) this->cb_safety_status_(status_);
}

template <typename T_PointCloud>
bool DecoderRSSafetyAIRY<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, size_t size)
{
  if (size != 1240 || !this->point_cloud_) { reject(); return false; }
  if (!validProfile(pkt)) { reject(ERRCODE_UNSUPPORTEDPROFILE); return false; }
  static const uint8_t marker[] = {0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc};
  if (std::memcmp(pkt + 1234, marker, 6) || be32(pkt + 26) >= 1000000 ||
      be32(pkt + 12) > 65535)
  { reject(); return false; }
  if (!this->angles_ready_) { reject(ERRCODE_NODIFOPRECV); return false; }

  RSSafetyAiryMsop packet;
  std::memcpy(&packet, pkt, sizeof(packet));
  uint16_t azimuth[3];
  // Validate the entire packet before appending any points or splitting a frame.
  for (size_t b = 0; b < 6; ++b)
  {
    const auto& block = packet.blocks[b];
    if (block.id[0] != 0xff || block.id[1] != 0xee || be16(block.azimuth) >= 36000)
    { reject(ERRCODE_WRONGMSOPBLKID); return false; }
    if ((b % 2) == 0) azimuth[b / 2] = be16(block.azimuth);
    else if (azimuth[b / 2] != be16(block.azimuth)) { reject(); return false; }
  }
  const double packet_ts = stamp(sensorTime(pkt + 20));
  const uint16_t seq = uint16_t(be32(pkt + 12));
  if (sequence_ready_)
  {
    const uint16_t delta = uint16_t(seq - sequence_);
    if (delta == 0 || delta > 32768) { reject(); return false; }
    status_.sequence_gaps += delta - 1;
  }
  sequence_ = seq;
  sequence_ready_ = true;
  ++status_.msop_packets;
  this->prev_pkt_ts_ = packet_ts;
  bool split = false;
  for (size_t col = 0; col < 3; ++col)
  {
    int32_t step = this->block_az_diff_;
    if (col < 2)
    {
      const int32_t actual_step = (azimuth[col + 1] - azimuth[col] + 36000) % 36000;
      if (actual_step <= 100) step = actual_step;
    }
    // Use the factory column schedule. At sector boundaries, captured encoder
    // jumps do not consistently imply elapsed time in the packet timestamps.
    // Inferring a delay from those jumps can put points after the next packet.
    const double column_ts = packet_ts + col * this->mech_const_param_.BLOCK_DURATION;
    if (this->split_strategy_->newBlock(azimuth[col]))
    {
      if (this->cb_split_frame_) this->cb_split_frame_(96, this->cloudTs());
      this->first_point_ts_ = column_ts;
      split = true;
    }
    if (this->point_cloud_->points.empty()) this->first_point_ts_ = column_ts;
    for (size_t chan = 0; chan < 96; ++chan)
    {
      const auto& raw = packet.blocks[2 * col + chan / 48].channels[chan % 48];
      const uint16_t word = be16(raw.range);
      const float distance = (word & 0x3fff) * 0.005f;
      const int32_t h = (azimuth[col] + int32_t(step * this->mech_const_param_.CHAN_AZIS[chan])) % 36000;
      const int32_t a = (this->chan_angles_.horizAdjust(chan, h) + 36000) % 36000;
      const int32_t v = this->chan_angles_.vertAdjust(chan);
      const double ts = column_ts + this->mech_const_param_.CHAN_TSS[chan];
      typename T_PointCloud::PointT point{};
      const bool valid = (word & 0x3fff) != 0 && distance >= 0.1f && distance <= 60.0f &&
                         this->distance_section_.in(distance) && this->scan_section_.in(a);
      if (valid)
      {
        float x = distance * COS(v) * COS(a) + this->lidar_lens_center_Rxy_ * COS(h);
        float y = -distance * COS(v) * SIN(a) - this->lidar_lens_center_Rxy_ * SIN(h);
        float z = distance * SIN(v) + this->mech_const_param_.RZ;
        this->transformPoint(x, y, z);
        setX(point, x); setY(point, y); setZ(point, z);
        setIntensity(point, raw.intensity);
      }
      else
      {
        setX(point, NAN); setY(point, NAN); setZ(point, NAN);
        setIntensity(point, 0);
      }
      setRing(point, this->chan_angles_.toUserChan(chan));
      setTimestamp(point, ts);
      setFeature(point, uint8_t(word >> 14));  // Opaque raw bits, not a validity classification.
      if (valid || !this->param_.dense_points) this->point_cloud_->points.emplace_back(point);
      this->prev_point_ts_ = ts;
    }
  }
  // Leave recorded wire bytes and their CRC intact, even in host-clock mode.
  return split;
}

template <typename T_PointCloud>
void DecoderRSSafetyAIRY<T_PointCloud>::decodeImuPkt(const uint8_t* pkt, size_t size)
{
  if (size != 51 || std::memcmp(pkt, this->const_param_.IMU_ID, 4) ||
      pkt[49] != 0xf0 || pkt[50] != 0x0f || pkt[42] > 4 || pkt[43] > 3 ||
      pkt[44] > 3 || be32(pkt + 10) >= 1000000)
  { reject(); return; }
  if (!this->imuDataPtr_ || !this->cb_imu_data_) return;
  auto& imu = *this->imuDataPtr_;
  imu.timestamp = stamp(sensorTime(pkt + 4));
  const double accel_scale = double(2u << pkt[43]) / 32768.0 * 9.80665;
  const double gyro_scale = double(250u << pkt[44]) / 32768.0 * 0.017453292519943295;
  imu.linear_acceleration_x = signedBe32(pkt + 14) * accel_scale;
  imu.linear_acceleration_y = signedBe32(pkt + 18) * accel_scale;
  imu.linear_acceleration_z = signedBe32(pkt + 22) * accel_scale;
  imu.angular_velocity_x = signedBe32(pkt + 26) * gyro_scale;
  imu.angular_velocity_y = signedBe32(pkt + 30) * gyro_scale;
  imu.angular_velocity_z = signedBe32(pkt + 34) * gyro_scale;
  imu.state = true;
  ++status_.imu_packets;
  this->cb_imu_data_();
}

template <typename T_PointCloud>
bool DecoderRSSafetyAIRY<T_PointCloud>::isNewFrame(const uint8_t* pkt)
{
  // The input's PCAP callback supplies a complete MSOP payload.
  if (!validProfile(pkt) || !crcMatches(pkt, 1230)) return false;
  bool split = false;
  for (size_t col = 0; col < 3; ++col)
    split |= this->pre_split_strategy_->newBlock(be16(pkt + 44 + 392 * col));
  return split;
}

}  // namespace lidar
}  // namespace robosense
