#include <Arduino.h>
#include <Wire.h>
#include <MahonyAHRS.h>

#include "icm20948.h"

#define ICM20948_ADDR_1 0x69
#define ICM20948_ADDR_2 0x68

using namespace icm20948;

constexpr float SAMPLE_RATE_HZ = 50.0f;
constexpr unsigned long SAMPLE_PERIOD_MS = 20;  // 1000 / SAMPLE_RATE_HZ

Mahony mahonyFilterObject;
uint8_t icmAddress = ICM20948_ADDR_1;
bool magAvailable = false;
// Last good magnetometer sample in uT, in the accel/gyro frame. While it is
// all zeros, Mahony::update() falls back to the 6-axis (no yaw reference) update.
Vec3 mag = {0.0f, 0.0f, 0.0f};

static void halt(const char *message) {
  Serial.println(message);
  while (1) delay(1000);
}

static void writeRegister(uint8_t device, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Reads `length` consecutive registers starting at `reg`.
// Returns false if the device did not answer or sent fewer bytes.
static bool readRegisters(uint8_t device, uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(device);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(device, length) != length) return false;
  for (uint8_t i = 0; i < length; i++) buffer[i] = Wire.read();
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Inicializa o filtro Mahony
  mahonyFilterObject.begin(SAMPLE_RATE_HZ); // Taxa de atualização de 50 Hz

  // Verifica qual endereço o sensor está respondendo
  Wire.beginTransmission(ICM20948_ADDR_1);
  if (Wire.endTransmission() == 0) {
    icmAddress = ICM20948_ADDR_1;
  } else {
    Wire.beginTransmission(ICM20948_ADDR_2);
    if (Wire.endTransmission() == 0) {
      icmAddress = ICM20948_ADDR_2;
    } else {
      halt("ICM-20948 não encontrado!");
    }
  }

  Serial.print("ICM-20948 encontrado no endereço: 0x");
  Serial.println(icmAddress, HEX);

  // Other IMUs (MPU-6050, MPU-9250) answer on the same addresses.
  uint8_t whoAmI = 0;
  if (!readRegisters(icmAddress, REG_WHO_AM_I, &whoAmI, 1) || whoAmI != WHO_AM_I_VALUE) {
    halt("WHO_AM_I is not 0xEA: this is not an ICM-20948");
  }

  // The chip comes out of reset asleep (PWR_MGMT_1 = 0x41), and every data
  // register reads zero until SLEEP is cleared.
  writeRegister(icmAddress, REG_PWR_MGMT_1, PWR_MGMT_1_WAKE);
  delay(50);

  // The magnetometer is a separate AK09916 die. Bypass mode connects it
  // straight to this I2C bus at address 0x0C.
  writeRegister(icmAddress, REG_INT_PIN_CFG, INT_PIN_CFG_BYPASS_EN);
  delay(10);
  uint8_t magId = 0;
  if (readRegisters(MAG_ADDR, MAG_REG_WIA2, &magId, 1) && magId == MAG_WIA2_VALUE) {
    writeRegister(MAG_ADDR, MAG_REG_CNTL2, MAG_MODE_CONT_100HZ);
    magAvailable = true;
    Serial.println("AK09916 magnetometer found (continuous mode, 100 Hz)");
  } else {
    Serial.println("AK09916 magnetometer not found: 6-axis mode, yaw will drift");
  }
}

void loop() {
  static unsigned long lastTime = millis();
  if (millis() - lastTime < SAMPLE_PERIOD_MS) return; // Atualiza a 50Hz
  // Advance by exactly one period so the average rate matches the rate the
  // filter was given in begin(), even if one iteration runs late.
  lastTime += SAMPLE_PERIOD_MS;

  // Lê os dados do sensor
  uint8_t raw[ACCEL_GYRO_BYTES];
  if (!readRegisters(icmAddress, REG_ACCEL_XOUT_H, raw, ACCEL_GYRO_BYTES)) {
    return; // I2C error: skip this sample rather than feed garbage to the filter
  }
  // Converte para unidades físicas: g e graus/s (escala padrão ±2 g, ±250 dps)
  const Sample sample = parseAccelGyro(raw);

  if (magAvailable) {
    uint8_t magRaw[MAG_BLOCK_BYTES];
    if (readRegisters(MAG_ADDR, MAG_REG_ST1, magRaw, MAG_BLOCK_BYTES)) {
      parseMag(magRaw, mag); // keeps the previous sample if none is ready
    }
  }

  // Atualiza o filtro Mahony.
  // This Mahony library expects the gyroscope in degrees/second and converts
  // to rad/s itself. Accelerometer and magnetometer units do not matter:
  // both vectors are normalised.
  mahonyFilterObject.update(sample.gyro_dps.x, sample.gyro_dps.y, sample.gyro_dps.z,
                            sample.accel_g.x, sample.accel_g.y, sample.accel_g.z,
                            mag.x, mag.y, mag.z);

  // Obtém a orientação
  float pitch = mahonyFilterObject.getPitch();
  float roll = mahonyFilterObject.getRoll();
  float yaw = mahonyFilterObject.getYaw();

  // Exibe os valores
  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.print(" Yaw: "); Serial.println(yaw);
}
