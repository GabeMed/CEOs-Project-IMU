#include <Arduino.h>
#include <Wire.h>
#include <MahonyAHRS.h>

#define ICM20948_ADDR_1 0x69
#define ICM20948_ADDR_2 0x68

Mahony mahonyFilterObject;
uint8_t icmAddress = ICM20948_ADDR_1;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Inicializa o filtro Mahony
  mahonyFilterObject.begin(50); // Taxa de atualização de 50 Hz

  // Verifica qual endereço o sensor está respondendo
  Wire.beginTransmission(ICM20948_ADDR_1);
  if (Wire.endTransmission() == 0) {
    icmAddress = ICM20948_ADDR_1;
  } else {
    Wire.beginTransmission(ICM20948_ADDR_2);
    if (Wire.endTransmission() == 0) {
      icmAddress = ICM20948_ADDR_2;
    } else {
      Serial.println("ICM-20948 não encontrado!");
      while (1);
    }
  }

  Serial.print("ICM-20948 encontrado no endereço: 0x");
  Serial.println(icmAddress, HEX);
}

void loop() {
  static unsigned long lastTime = 0;
  if (millis() - lastTime < 20) return; // Atualiza a 50Hz
  lastTime = millis();

  // Lê os dados do sensor
  Wire.beginTransmission(icmAddress);
  Wire.write(0x2D);
  Wire.endTransmission(false);
  Wire.requestFrom(icmAddress, 14);

  int16_t accelX = Wire.read() << 8 | Wire.read();
  int16_t accelY = Wire.read() << 8 | Wire.read();
  int16_t accelZ = Wire.read() << 8 | Wire.read();
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();
  int16_t magX = Wire.read() << 8 | Wire.read();
  int16_t magY = Wire.read() << 8 | Wire.read();
  int16_t magZ = Wire.read() << 8 | Wire.read();

  // Converte para unidades físicas (assumindo escala padrão), isso aqui eu pedi pro GPT me dar valores pq eu n fazia ideia
  const float accelScale = 1.0 / 16384.0; // ±2g
  const float gyroScale = 1.0 / 131.0;    // ±250dps
  const float magScale = 1.0 / 0.15;      // µT (valor típico, pode variar)

  float ax = accelX * accelScale;
  float ay = accelY * accelScale;
  float az = accelZ * accelScale;
  float gx = (gyroX * gyroScale) * PI / 180.0; // Converte para rad/s
  float gy = (gyroY * gyroScale) * PI / 180.0;
  float gz = (gyroZ * gyroScale) * PI / 180.0;
  float mx = magX * magScale;
  float my = magY * magScale;
  float mz = magZ * magScale;

  // Atualiza o filtro Mahony
  mahonyFilterObject.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  // Obtém a orientação
  float pitch = mahonyFilterObject.getPitch();
  float roll = mahonyFilterObject.getRoll();
  float yaw = mahonyFilterObject.getYaw();

  // Exibe os valores
  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" Roll: "); Serial.print(roll);
  Serial.print(" Yaw: "); Serial.println(yaw);
}
