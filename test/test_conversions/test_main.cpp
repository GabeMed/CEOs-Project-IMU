// Host-side tests for include/icm20948.h. Run with: pio test -e native
#include <unity.h>

#include "icm20948.h"

using namespace icm20948;

void setUp() {}
void tearDown() {}

static void test_be16_reads_high_byte_first() {
  const uint8_t a[] = {0x12, 0x34};
  const uint8_t minus_two[] = {0xFF, 0xFE};
  const uint8_t most_negative[] = {0x80, 0x00};
  TEST_ASSERT_EQUAL_INT16(0x1234, be16(a));
  TEST_ASSERT_EQUAL_INT16(-2, be16(minus_two));
  TEST_ASSERT_EQUAL_INT16(-32768, be16(most_negative));
}

static void test_le16_reads_low_byte_first() {
  const uint8_t a[] = {0x34, 0x12};
  const uint8_t minus_two[] = {0xFE, 0xFF};
  TEST_ASSERT_EQUAL_INT16(0x1234, le16(a));
  TEST_ASSERT_EQUAL_INT16(-2, le16(minus_two));
}

// Sensor lying flat, face up, at rest: +1 g on Z (16384 LSB = 0x4000).
static void test_accel_flat_is_one_g_on_z() {
  const uint8_t raw[ACCEL_GYRO_BYTES] = {0x00, 0x00, 0x00, 0x00, 0x40, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const Sample s = parseAccelGyro(raw);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.accel_g.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.accel_g.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, s.accel_g.z);
}

static void test_accel_full_scale_is_two_g() {
  const uint8_t raw[ACCEL_GYRO_BYTES] = {0x80, 0x00, 0x7F, 0xFF, 0xC0, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const Sample s = parseAccelGyro(raw);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -2.0f, s.accel_g.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, s.accel_g.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, s.accel_g.z);
}

// The Mahony library takes degrees/second, so the conversion must not go to
// rad/s: 131 LSB is 1 dps, 32750 LSB is 250 dps.
static void test_gyro_is_in_degrees_per_second() {
  const uint8_t raw[ACCEL_GYRO_BYTES] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x83,   // +131
                                         0x7F, 0xEE,   // +32750
                                         0x80, 0x12};  // -32750
  const Sample s = parseAccelGyro(raw);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, s.gyro_dps.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 250.0f, s.gyro_dps.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -250.0f, s.gyro_dps.z);
}

// ST1=DRDY, HX=+100, HY=+200, HZ=-300 (little-endian), TMPS, ST2=0.
// 0.15 uT/LSB, and Y/Z are flipped into the accel/gyro frame.
static void test_mag_scale_and_axis_alignment() {
  const uint8_t raw[MAG_BLOCK_BYTES] = {MAG_ST1_DRDY, 0x64, 0x00, 0xC8, 0x00,
                                        0xD4, 0xFE, 0x00, 0x00};
  Vec3 out = {0, 0, 0};
  TEST_ASSERT_TRUE(parseMag(raw, out));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 15.0f, out.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -30.0f, out.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 45.0f, out.z);
}

static void test_mag_not_ready_keeps_previous_sample() {
  const uint8_t raw[MAG_BLOCK_BYTES] = {0x00, 0x64, 0x00, 0xC8, 0x00,
                                        0xD4, 0xFE, 0x00, 0x00};
  Vec3 out = {1.0f, 2.0f, 3.0f};
  TEST_ASSERT_FALSE(parseMag(raw, out));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, out.x);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, out.y);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, out.z);
}

static void test_mag_overflow_is_rejected() {
  const uint8_t raw[MAG_BLOCK_BYTES] = {MAG_ST1_DRDY, 0xFF, 0x7F, 0x00, 0x00,
                                        0x00, 0x00, 0x00, MAG_ST2_HOFL};
  Vec3 out = {1.0f, 2.0f, 3.0f};
  TEST_ASSERT_FALSE(parseMag(raw, out));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, out.x);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_be16_reads_high_byte_first);
  RUN_TEST(test_le16_reads_low_byte_first);
  RUN_TEST(test_accel_flat_is_one_g_on_z);
  RUN_TEST(test_accel_full_scale_is_two_g);
  RUN_TEST(test_gyro_is_in_degrees_per_second);
  RUN_TEST(test_mag_scale_and_axis_alignment);
  RUN_TEST(test_mag_not_ready_keeps_previous_sample);
  RUN_TEST(test_mag_overflow_is_rejected);
  return UNITY_END();
}
