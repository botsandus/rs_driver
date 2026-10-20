// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <rs_driver/driver/decoder/decoder_factory.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <fstream>
#include <iterator>

using namespace robosense::lidar;
using Cloud = PointCloudT<PointXYZIRTF>;
using SafetyDecoder = DecoderRSSafetyAIRY<Cloud>;

namespace {
std::vector<uint8_t> fixture(const char* name)
{
  std::ifstream file(std::string(SAFETY_AIRY_FIXTURES) + "/" + name, std::ios::binary);
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
void put32(std::vector<uint8_t>& p, size_t offset, uint32_t value)
{
  for (int b = 3; b >= 0; --b) { p[offset + b] = value & 0xff; value >>= 8; }
}
void fixCrc(std::vector<uint8_t>& p, size_t offset)
{ put32(p, offset, SafetyDecoder::crc32(p.data(), offset)); }

class SafetyAiry : public testing::Test
{
protected:
  std::vector<uint8_t> msop = fixture("msop.bin"), difop = fixture("difop.bin"), imu = fixture("imu.bin");
  RSDecoderParam params;
  std::unique_ptr<SafetyDecoder> decoder;
  size_t imu_calls = 0, status_calls = 0;
  SafetyAiryStatus status;
  void SetUp() override
  {
    ASSERT_EQ(msop.size(), 1240u); ASSERT_EQ(difop.size(), 1248u); ASSERT_EQ(imu.size(), 51u);
    params.use_lidar_clock = true;
    params.ts_first_point = true;
    params.min_distance = 0.1;
    params.max_distance = 60;
    params.dense_points = false;
    decoder.reset(new SafetyDecoder(params));
    decoder->point_cloud_ = std::make_shared<Cloud>();
    decoder->imuDataPtr_ = std::make_shared<ImuData>();
    decoder->regCallback([](const Error&) {}, [](uint16_t, double) {});
    decoder->regImuCallback([this] { ++imu_calls; });
    decoder->regSafetyAiryStatusCallback([this](const SafetyAiryStatus& s) { status = s; ++status_calls; });
  }
  void calibrate() { decoder->processDifopPkt(difop.data(), difop.size()); }
  void scan() { decoder->processMsopPkt(msop.data(), msop.size()); }
};
}

TEST_F(SafetyAiry, RealCaptureCrcAndMutation)
{
  EXPECT_EQ(SafetyDecoder::crc32(msop.data(), 1230), 0xd8dd3d2du);
  EXPECT_EQ(SafetyDecoder::crc32(difop.data(), 1242), 0x473ed739u);
  calibrate();
  msop[50] ^= 1;
  scan();
  EXPECT_TRUE(decoder->point_cloud_->points.empty());
  EXPECT_EQ(decoder->status_.crc_errors, 1u);
}

TEST_F(SafetyAiry, RealPacketGeometryFlagsRingsAndMicroseconds)
{
  calibrate(); scan();
  const auto& points = decoder->point_cloud_->points;
  ASSERT_EQ(points.size(), 288u);
  EXPECT_EQ(points[0].ring, 0); EXPECT_EQ(points[48].ring, 48);
  EXPECT_EQ(points[95].ring, 95); EXPECT_EQ(points[96].ring, 0);
  EXPECT_EQ(points[0].feature, 2);
  // Golden coordinates from captured range/calibration and independent trig calculation.
  EXPECT_NEAR(points[0].x, -0.149057795, 1e-5);
  EXPECT_NEAR(points[0].y, -0.014989840, 1e-5);
  EXPECT_NEAR(points[0].z, 0.049305654, 1e-5);
  EXPECT_NEAR(points[48].x, -2.501133908, 1e-5);
  EXPECT_NEAR(points[48].y, -0.284542548, 1e-5);
  EXPECT_NEAR(points[48].z, 2.541307354, 1e-5);
  EXPECT_NEAR(points[0].timestamp, 656.024710, 1e-9);
  EXPECT_NEAR(points[48].timestamp - points[0].timestamp, 52.360e-6, 1e-9);
  EXPECT_NEAR(points[96].timestamp - points[0].timestamp, 111.080e-6, 1e-9);
  EXPECT_EQ(points[0].intensity, msop[48]);
}

TEST_F(SafetyAiry, ValidDifopReportsRawStatusWithoutInventingImuTransform)
{
  calibrate();
  ASSERT_EQ(status_calls, 1u);
  EXPECT_TRUE(status.valid); EXPECT_TRUE(status.calibration_valid);
  EXPECT_FALSE(status.imu_extrinsic_valid);
  EXPECT_EQ(status.system_state, 0x33); EXPECT_EQ(status.active_group, 0xff);
  EXPECT_EQ(status.ossd[0], 0);
  EXPECT_EQ(status.firmware[8], 0x11); EXPECT_EQ(status.firmware[11], 0x13);
  EXPECT_EQ(status.rpm, 600);
}

TEST_F(SafetyAiry, BadDifopDoesNotEnableCloudOrRefreshStatus)
{
  difop[500] ^= 1;
  calibrate(); scan();
  EXPECT_EQ(status_calls, 0u);
  EXPECT_TRUE(decoder->point_cloud_->points.empty());
}

TEST_F(SafetyAiry, InvalidCalibrationIsRejectedEvenWithCorrectCrc)
{
  difop[468] = 2; fixCrc(difop, 1242);
  calibrate(); scan();
  EXPECT_FALSE(status.calibration_valid);
  EXPECT_TRUE(decoder->point_cloud_->points.empty());
}

TEST_F(SafetyAiry, TruncationAndUnsupportedModeDoNotAppendPoints)
{
  calibrate();
  for (size_t length : {size_t(0), size_t(1), size_t(42), size_t(1239)})
    decoder->processMsopPkt(msop.data(), length);
  EXPECT_TRUE(decoder->point_cloud_->points.empty());
  msop[17] = 3; fixCrc(msop, 1230); scan();
  EXPECT_TRUE(decoder->point_cloud_->points.empty());
  EXPECT_EQ(decoder->status_.rejected_packets, 1u);
}

TEST_F(SafetyAiry, LaterBrokenBlockCannotPartiallyPublishPacket)
{
  calibrate();
  msop[42 + 5 * 196] = 0; fixCrc(msop, 1230); scan();
  EXPECT_TRUE(decoder->point_cloud_->points.empty());
}

TEST_F(SafetyAiry, PairedAzimuthAndTimestampBoundsAreChecked)
{
  calibrate();
  msop[44 + 196] ^= 1; fixCrc(msop, 1230); scan();
  EXPECT_TRUE(decoder->point_cloud_->points.empty());
  msop = fixture("msop.bin");
  put32(msop, 26, 1000000); fixCrc(msop, 1230); scan();
  EXPECT_TRUE(decoder->point_cloud_->points.empty());
}

TEST_F(SafetyAiry, ZeroMaskedRangeIsInvalidEvenWithFlags)
{
  calibrate();
  msop[46] = 0x80; msop[47] = 0;
  fixCrc(msop, 1230); scan();
  ASSERT_EQ(decoder->point_cloud_->points.size(), 288u);
  EXPECT_TRUE(std::isnan(decoder->point_cloud_->points[0].x));
  EXPECT_EQ(decoder->point_cloud_->points[0].feature, 2);
}

TEST_F(SafetyAiry, EncoderSectorJumpDoesNotInventAnElapsedTimeGap)
{
  calibrate();
  const uint16_t azimuth[] = {241, 1859, 1899};
  for (size_t b = 0; b < 6; ++b)
  {
    msop[44 + 196 * b] = azimuth[b / 2] >> 8;
    msop[45 + 196 * b] = azimuth[b / 2] & 0xff;
  }
  fixCrc(msop, 1230); scan();
  const auto& points = decoder->point_cloud_->points;
  ASSERT_EQ(points.size(), 288u);
  EXPECT_NEAR(points[96].timestamp - points[0].timestamp, 111.080e-6, 1e-9);
  EXPECT_NEAR(points.back().timestamp - points[0].timestamp, 319.264e-6, 1e-9);
}

TEST_F(SafetyAiry, PcapFrameProbeRejectsShortOrWrongPacketsBeforeReadingBlocks)
{
  Decoder<Cloud>& base = *decoder;
  const uint8_t short_packet[] = {0x55};
  EXPECT_FALSE(base.isNewFrame(short_packet, sizeof(short_packet)));
  EXPECT_FALSE(base.isNewFrame(difop.data(), difop.size()));
  msop[0] = 0;
  EXPECT_FALSE(base.isNewFrame(msop.data(), msop.size()));
}

TEST_F(SafetyAiry, SequenceWrapAndLossAccounting)
{
  calibrate();
  put32(msop, 12, 65535); fixCrc(msop, 1230); scan();
  put32(msop, 12, 0); fixCrc(msop, 1230); scan();
  put32(msop, 12, 3); fixCrc(msop, 1230); scan();
  EXPECT_EQ(decoder->status_.msop_packets, 3u);
  EXPECT_EQ(decoder->status_.sequence_gaps, 2u);
  scan();  // Duplicate cannot append a second copy.
  EXPECT_EQ(decoder->status_.msop_packets, 3u);
}

TEST_F(SafetyAiry, ImuSignedValuesArePublishedInSiUnits)
{
  decoder->processImuPkt(imu.data(), imu.size());
  ASSERT_EQ(imu_calls, 1u);
  const auto& out = *decoder->imuDataPtr_;
  EXPECT_NEAR(out.linear_acceleration_x, -2097.0 * 4 / 32768 * 9.80665, 1e-6);
  EXPECT_NEAR(out.linear_acceleration_z, 7820.0 * 4 / 32768 * 9.80665, 1e-6);
  EXPECT_NEAR(out.angular_velocity_y, -24.0 * 500 / 32768 * M_PI / 180, 1e-7);
  imu[43] = 32; decoder->processImuPkt(imu.data(), imu.size());
  EXPECT_EQ(imu_calls, 1u);
  imu = fixture("imu.bin"); imu[49] = 0;
  decoder->processImuPkt(imu.data(), imu.size()); EXPECT_EQ(imu_calls, 1u);
}

TEST(SafetyAiryRegistration, FactoryAndExistingModelValues)
{
  EXPECT_EQ(int(RSAIRY), 4); EXPECT_EQ(int(RSHELIOS), 5); EXPECT_EQ(int(RSP48), 12);
  EXPECT_EQ(strToLidarType("RSSAFETYAIRY"), RSSAFETYAIRY);
  EXPECT_EQ(lidarTypeToStr(RSSAFETYAIRY), "RSSAFETYAIRY");
  auto decoder = DecoderFactory<Cloud>::createDecoder(RSSAFETYAIRY, RSDecoderParam{});
  EXPECT_NE(dynamic_cast<SafetyDecoder*>(decoder.get()), nullptr);
  auto ordinary = DecoderFactory<Cloud>::createDecoder(RSAIRY, RSDecoderParam{});
  EXPECT_EQ(ordinary->const_param_.MSOP_LEN, 1248);
}
