// ICM-20948 register map (the parts this project uses) and raw-to-physical
// conversions. Header-only and free of Arduino code, so the conversions can
// be unit-tested on the host (see test/test_conversions).
//
// Register addresses and bit positions follow the TDK ICM-20948 datasheet
// (DS-000189) and the AK09916 datasheet for the magnetometer die inside it.
#pragma once

#include <stdint.h>

namespace icm20948 {

// ---- ICM-20948, User Bank 0 (the bank selected after reset) --------------
// Device address: 0x69 with AD0 high, 0x68 with AD0 low (see src/main.cpp).
constexpr uint8_t REG_WHO_AM_I = 0x00;
constexpr uint8_t WHO_AM_I_VALUE = 0xEA;

constexpr uint8_t REG_PWR_MGMT_1 = 0x06;  // resets to 0x41: SLEEP=1
constexpr uint8_t PWR_MGMT_1_WAKE = 0x01; // SLEEP=0, CLKSEL=1 (best clock)

constexpr uint8_t REG_INT_PIN_CFG = 0x0F;
constexpr uint8_t INT_PIN_CFG_BYPASS_EN = 0x02;  // expose the AK09916 on the main I2C bus

// ACCEL_XOUT_H..GYRO_ZOUT_L: 12 bytes, big-endian, accel X/Y/Z then gyro X/Y/Z.
// (The next two bytes, 0x39-0x3A, are the temperature, not the magnetometer.)
constexpr uint8_t REG_ACCEL_XOUT_H = 0x2D;
constexpr uint8_t ACCEL_GYRO_BYTES = 12;

// ---- AK09916 magnetometer (reachable at 0x0C once bypass is enabled) -----
constexpr uint8_t MAG_ADDR = 0x0C;
constexpr uint8_t MAG_REG_WIA2 = 0x01;
constexpr uint8_t MAG_WIA2_VALUE = 0x09;
constexpr uint8_t MAG_REG_ST1 = 0x10;   // bit 0: DRDY
constexpr uint8_t MAG_REG_CNTL2 = 0x31;
constexpr uint8_t MAG_MODE_CONT_100HZ = 0x08;
// ST1, HXL..HZH (little-endian), TMPS, ST2. Reading ST2 ends the read and
// lets the sensor latch the next sample.
constexpr uint8_t MAG_BLOCK_BYTES = 9;
constexpr uint8_t MAG_ST1_DRDY = 0x01;
constexpr uint8_t MAG_ST2_HOFL = 0x08;  // magnetic overflow

// ---- Scale factors for the reset configuration ---------------------------
// ACCEL_CONFIG and GYRO_CONFIG_1 (User Bank 2) are left at their reset
// values: accelerometer +-2 g, gyroscope +-250 dps.
constexpr float ACCEL_LSB_PER_G = 16384.0f;
constexpr float GYRO_LSB_PER_DPS = 131.0f;
constexpr float MAG_UT_PER_LSB = 0.15f;

struct Vec3 {
  float x, y, z;
};

struct Sample {
  Vec3 accel_g;
  Vec3 gyro_dps;
};

inline int16_t be16(const uint8_t *p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

inline int16_t le16(const uint8_t *p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[1]) << 8) | p[0]);
}

// Parses the 12 bytes read from REG_ACCEL_XOUT_H into g and degrees/second.
inline Sample parseAccelGyro(const uint8_t raw[ACCEL_GYRO_BYTES]) {
  Sample s;
  s.accel_g = {be16(raw + 0) / ACCEL_LSB_PER_G,
               be16(raw + 2) / ACCEL_LSB_PER_G,
               be16(raw + 4) / ACCEL_LSB_PER_G};
  s.gyro_dps = {be16(raw + 6) / GYRO_LSB_PER_DPS,
                be16(raw + 8) / GYRO_LSB_PER_DPS,
                be16(raw + 10) / GYRO_LSB_PER_DPS};
  return s;
}

// Parses the 9 bytes read from MAG_REG_ST1. Returns false if no new sample
// was ready or the reading overflowed; `out` is then left unchanged.
//
// The AK09916 axes are not the accelerometer/gyroscope axes: X matches,
// Y and Z point the other way (datasheet, "Orientation of Axes"). The result
// is rotated into the accel/gyro frame, in microtesla.
inline bool parseMag(const uint8_t raw[MAG_BLOCK_BYTES], Vec3 &out) {
  const uint8_t st1 = raw[0];
  const uint8_t st2 = raw[8];
  if (!(st1 & MAG_ST1_DRDY) || (st2 & MAG_ST2_HOFL)) {
    return false;
  }
  out = {le16(raw + 1) * MAG_UT_PER_LSB,
         -le16(raw + 3) * MAG_UT_PER_LSB,
         -le16(raw + 5) * MAG_UT_PER_LSB};
  return true;
}

}  // namespace icm20948
