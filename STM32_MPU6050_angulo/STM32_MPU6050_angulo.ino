/*
   Lectura y procesamiento del sensor MPU6050 para obtención de los ángulos de inclinación
   Versión para STM32
*/

#include <Wire.h>
TwoWire I2C2_Wire (2, I2C_FAST_MODE); // SDA: PB11, SCL: PB10

//Declaración de variables
#define MPU6050_address 0x68
#define tiempo_ciclo_us 5000
#define RAD_A_DEG 57.295779 
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw, acc_x_raw, acc_y_raw, acc_z_raw, temperature;
int32_t gyro_x_cal, gyro_y_cal, gyro_z_cal, acc_z_cal, acc_y_cal, acc_x_cal;
float angulo_x_gyro, angulo_y_gyro, angulo_x_acc, angulo_y_acc;
float gyro_z_DPS, gyro_x_DPS, gyro_y_DPS;
float acc_total_vector;
bool set_gyro_angles, MPU6050_calibrado  = false;
uint32_t loop_timer;

void setup() {
  Serial.begin(115200);

  I2C2_Wire.setClock(400000);                                      //I2C fast mode (400 kbit/s)
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

  // Calibrar giroscopio y acelerómetro: con el sensor estático, se hacen 3000 muetras a intervalos de 1ms tanto e los 3 ejes
  // del giroscopio como del acelerómetro, y se calcula la media.
  for (uint16_t cal_int = 0; cal_int < 3000 ; cal_int ++) {
    leer_MPU6050();
    gyro_x_cal += gyro_x_raw;
    gyro_y_cal += gyro_y_raw;
    gyro_z_cal += gyro_z_raw;
    acc_x_cal += acc_x_raw;
    acc_y_cal += acc_y_raw;
    acc_z_cal += acc_z_raw;
    delayMicroseconds(1000);
  }

  // División entre el número de muetras para calcular el valor medio, que será el offset de cada eje
  gyro_x_cal = gyro_x_cal / 3000;
  gyro_y_cal = gyro_y_cal / 3000;
  gyro_z_cal = gyro_z_cal / 3000;
  acc_x_cal  = acc_x_cal / 3000;
  acc_y_cal  = acc_y_cal / 3000;
  acc_z_cal  = acc_z_cal / 3000;
  MPU6050_calibrado = true;

  loop_timer = micros();
}

void loop() {
  // Hacer que el loop() se ejecuta de forma periódica a intervalos de 'tiempo_ciclo_us'
  while (micros() - loop_timer < tiempo_ciclo_us);
  loop_timer = micros();

  leer_MPU6050();     // Leer MPU6050
  procesar_MPU6050(); // Procesar lecturas de MPU6050

  // Visualizar variables por canal serie
  Serial.print(angulo_x_gyro);
  Serial.print("\t");
  Serial.println(angulo_y_gyro);
}

void leer_MPU6050() {
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
}

void procesar_MPU6050() {
  // Restar offset
  gyro_x_raw -= gyro_x_cal;
  gyro_y_raw -= gyro_y_cal;
  gyro_z_raw -= gyro_z_cal;
  acc_x_raw  -= acc_x_cal;
  acc_y_raw  -= acc_y_cal;
  acc_z_raw  -= acc_z_cal - 4096;

  // Conversión a grados por segundo (DPS: degree per secord)
  // 65.5 en raw, significa que gira a 1º/s
  gyro_x_DPS = gyro_x_raw / 65.5;
  gyro_y_DPS = gyro_y_raw / 65.5;
  gyro_z_DPS = gyro_z_raw / 65.5;

  // Calcular ángulo con lecturas de giroscopio (º/s * s = º)
  // ang(º) = ang_ant(º) + vel_ang(º/s)*dt(s)
  // División entre 1000000 para pasar de us a s
  angulo_x_gyro = angulo_x_gyro + gyro_x_DPS * tiempo_ciclo_us / 1000000;
  angulo_y_gyro = angulo_y_gyro + gyro_y_DPS * tiempo_ciclo_us / 1000000;

  // Calcular ángulo con lecturas del acelerómetro
  // Más información sobre las ecuaciones en https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
  angulo_y_acc = atan(-acc_x_raw / sqrt(pow(acc_y_raw, 2) + pow(acc_z_raw, 2))) * RAD_TO_DEG;
  angulo_x_acc = atan(acc_y_raw  / sqrt(pow(acc_x_raw, 2) + pow(acc_z_raw, 2))) * RAD_TO_DEG;

  // Filtro complementario. Corregir drift con los ángulos obtenido del acelerómetro.
  angulo_x_gyro = angulo_x_gyro * 0.95 + angulo_x_acc * 0.05;
  angulo_y_gyro = angulo_y_gyro * 0.95 + angulo_y_acc * 0.05;
}
