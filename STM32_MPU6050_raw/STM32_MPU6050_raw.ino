/*
   Lectura raw del sensor MPU6050
   Versión para STM32
*/

#include <Wire.h>
TwoWire I2C2_Wire (2, I2C_FAST_MODE);  // SDA: PB11, SCL: PB10

// Declaración de variables
#define MPU6050_address 0x68
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw, acc_x_raw, acc_y_raw, acc_z_raw, temperature;
float gyro_x_raw_DPS, gyro_y_raw_DPS, gyro_z_raw_DPS, acc_x_raw_G, acc_y_raw_G, acc_z_raw_G;

void setup() {
  Serial.begin(115200);

  I2C2_Wire.setClock(400000);                                      // I2C fast mode (400 kbit/s)
  I2C2_Wire.begin();

  I2C2_Wire.beginTransmission(MPU6050_address);                    //Comenzar comunicación con sensor
  I2C2_Wire.write(0x6B);                                           //Registro a modificar
  I2C2_Wire.write(0x00);                                           //Valor asignado a ese registro (00000000)
  I2C2_Wire.endTransmission();                                     //Finalizar comunicación con sensor

  //Configurar giroscopio a 500º/s (por defecto 250º/s)
  I2C2_Wire.beginTransmission(MPU6050_address);                    //Comenzar comunicación con sensor
  I2C2_Wire.write(0x1B);                                           //Registro a modificar
  I2C2_Wire.write(0x08);                                           //Valor asignado a ese registro (00001000)
  I2C2_Wire.endTransmission();                                     //Finalizar comunicación con sensor

  //Configurar acelerómetro a +/-8g (por defecto +/-2g)
  I2C2_Wire.beginTransmission(MPU6050_address);                    //Comenzar comunicación con sensor
  I2C2_Wire.write(0x1C);                                           //Registro a modificar
  I2C2_Wire.write(0x10);                                           //Valor asignado a ese registro (00010000)
  I2C2_Wire.endTransmission();                                     //Finalizar comunicación con sensor
}

void loop() {

  I2C2_Wire.beginTransmission(MPU6050_address);                    // Comenzar comunicación con sensor
  I2C2_Wire.write(0x3B);                                           // Enviar petición del primer registro
  I2C2_Wire.endTransmission();                                     // Finalizar comunicación con sensor
  I2C2_Wire.requestFrom(MPU6050_address, 14);                      // Solicitar un total de 14 registros
  while (I2C2_Wire.available() < 14);                              // Esperamos hasta recibir los 14 bytes
  acc_x_raw = I2C2_Wire.read() << 8 | I2C2_Wire.read();            // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  acc_y_raw = I2C2_Wire.read() << 8 | I2C2_Wire.read();            // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acc_z_raw = I2C2_Wire.read() << 8 | I2C2_Wire.read();            // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temperature = I2C2_Wire.read() << 8 | I2C2_Wire.read();          // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
  gyro_x_raw = I2C2_Wire.read() << 8 | I2C2_Wire.read();           // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  gyro_y_raw = I2C2_Wire.read() << 8 | I2C2_Wire.read();           // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  gyro_z_raw = I2C2_Wire.read() << 8 | I2C2_Wire.read();           // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)

  // Conversión a G y DPS (degrees per second)
  acc_x_raw_G = acc_x_raw / 4096;
  acc_y_raw_G = acc_y_raw / 4096;
  acc_z_raw_G = acc_z_raw / 4096;
  gyro_x_raw_DPS = gyro_x_raw / 65.5;
  gyro_y_raw_DPS = gyro_y_raw / 65.5;
  gyro_z_raw_DPS = gyro_z_raw / 65.5;

  // Visualizar variables por canal serie
  Serial.print(acc_x_raw_G);
  Serial.print("\t");
  Serial.print(acc_y_raw_G);
  Serial.print("\t");
  Serial.print(acc_z_raw_G);
  Serial.print("\t");
  Serial.print(gyro_x_raw_DPS);
  Serial.print("\t");
  Serial.print(gyro_y_raw_DPS);
  Serial.print("\t");
  Serial.println(gyro_z_raw_DPS);

  delay(20);
}
