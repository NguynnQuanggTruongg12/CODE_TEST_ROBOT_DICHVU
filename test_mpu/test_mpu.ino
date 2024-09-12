#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Khai báo đối tượng cảm biến
MPU6050 mpu6050(Wire);
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified();

// Khai báo biến để lưu trữ dữ liệu cảm biến
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float yaw;
float qw, qx, qy, qz;
uint8_t dataPacket[28] = { '$', 0x03, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// Khai báo sai số con quay hồi chuyển
float GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;

void setupIMU() {
   Wire.begin();                   // Initialize communication
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  Wire.setWireTimeout(3000, true);

  Wire.beginTransmission(0x68);   // This is the I2C address of the MPU 0x68
  Wire.write(0x6B);               // Accessing the register 6B 
  Wire.write(0x00);               // Setting SLEEP register to 0
  Wire.endTransmission(true); 
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);                  // Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x08);                  // Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission(true);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                   // Set the register bits as 00001000 (500deg/s full scale) / 65.5
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);           // Start I2C communication and send I2C slave address 0x68 to MPU 
  Wire.write(0x1A);                       // Send address to register 0x1A
  Wire.write(0x03);                       // Write 0x03 to register 0x1A to select DLPF bandwidth 44Hz
  Wire.endTransmission(true);
  
  delay(20);
}

bool readDataFromIMU6050(float *acc_x, float *acc_y, float *acc_z, float *gy_x, float *gy_y, float *gy_z) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  byte len = Wire.requestFrom(0x68, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  float temp_acc_x = (Wire.read() << 8 | Wire.read()) / 8192.0; // X-axis value
  float temp_acc_y = (Wire.read() << 8 | Wire.read()) / 8192.0; // Y-axis value
  float temp_acc_z = (Wire.read() << 8 | Wire.read()) / 8192.0; // Z-axis value

  if((len == 0)||(temp_acc_x == 0 && temp_acc_y == 0 && temp_acc_z == 0)){
    Serial.print(" Setup MPU6050 again:");
    setupIMU();
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    
    temp_acc_x = (Wire.read() << 8 | Wire.read()) / 8192.0; // X-axis value
    temp_acc_y = (Wire.read() << 8 | Wire.read()) / 8192.0; // Y-axis value
    temp_acc_z = (Wire.read() << 8 | Wire.read()) / 8192.0; // Z-axis value
  }
  
  // === Read gyroscope data === //
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  byte len2 = Wire.requestFrom(0x68, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  float temp_gy_x = (Wire.read() << 8 | Wire.read()) / 65.5 - GyroErrorX; // For a 500deg/s range, divide by 65.5
  float temp_gy_y = (Wire.read() << 8 | Wire.read()) / 65.5 - GyroErrorY;
  float temp_gy_z = (Wire.read() << 8 | Wire.read()) / 65.5 - GyroErrorZ;
  
  if(isnan(temp_acc_x) || isnan(temp_acc_y) || isnan(temp_acc_z)
     || isnan(temp_gy_x) || isnan(temp_gy_y) || isnan(temp_gy_z)) return false;
  
  *acc_x = temp_acc_x; *acc_y = temp_acc_y; *acc_z = temp_acc_z;
  *gy_x = temp_gy_x; *gy_y = temp_gy_y; *gy_z = temp_gy_z;   

  return true;
}

void setup() {
  // Khởi tạo giao tiếp Serial
  Serial.begin(115200);
  
  // Khởi tạo giao tiếp I2C
  Wire.begin();
  
  // Khởi tạo MPU6050
  setupIMU();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  // Khởi tạo HMC5883L
  if (!compass.begin()) {
    Serial.println("Couldn't find HMC5883L sensor");
    while (1);
  }
  
  Serial.println("Setup complete.");
}

void loop() {
  // Đọc dữ liệu từ MPU6050
  float temp_ax, temp_ay, temp_az;
  float temp_gx, temp_gy, temp_gz;
  if (readDataFromIMU6050(&temp_ax, &temp_ay, &temp_az, &temp_gx, &temp_gy, &temp_gz)) {
    ax = temp_ax;
    ay = temp_ay;
    az = temp_az;
    gx = temp_gx;
    gy = temp_gy;
    gz = temp_gz;
  }

  // Đọc dữ liệu từ HMC5883L
  sensors_event_t event;
  compass.getEvent(&event);
  mx = event.magnetic.x;
  my = event.magnetic.y;
  mz = event.magnetic.z;
  
  // Tính góc yaw từ dữ liệu từ trường từ tính
  yaw = atan2(my, mx) * 180 / M_PI; // Chuyển đổi từ radian sang độ
  
  // Chuyển đổi góc yaw từ độ sang radian
  float yawRad = yaw * M_PI / 180.0;
  
  // Tính toán quaternion từ góc yaw
  qw = cos(yawRad / 2.0);
  qx = 0;
  qy = 0;
  qz = sin(yawRad / 2.0);
  
  // Gửi dữ liệu qua Serial
  Serial.print(ax); Serial.print(" ");
  Serial.print(ay); Serial.print(" ");
  Serial.print(az); Serial.print(" ");
  Serial.print(gx); Serial.print(" ");
  Serial.print(gy); Serial.print(" ");
  Serial.print(gz); Serial.print(" ");
  Serial.print(yaw); Serial.print(" ");

  Serial.print(qw); Serial.print(" ");
  Serial.print(qx); Serial.print(" ");
  Serial.print(qy); Serial.print(" ");
  Serial.println(qz);
  
  delay(500); // Đợi một chút trước khi đọc dữ liệu tiếp theo
}

//
//
//#include <Wire.h>
//#include <MPU6050_tockn.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_HMC5883_U.h>
//
//// Khai báo đối tượng cảm biến
//MPU6050 mpu6050(Wire);
//Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified();
//
//// Khai báo biến để lưu trữ dữ liệu cảm biến
//float ax, ay, az;
//float gx, gy, gz;
//float mx, my, mz;
//float yaw;
//float qw, qx, qy, qz;
//uint8_t dataPacket[28] = { '$', 0x03, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
//
//
//// Khai báo sai số con quay hồi chuyển
//float GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;
//
//void setupIMU() {
//   Wire.begin();                   // Initialize communication
//#if ARDUINO >= 157
//  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
//#else
//  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
//#endif
//
//  Wire.setWireTimeout(3000, true);
//
//  Wire.beginTransmission(0x68);   // This is the I2C address of the MPU 0x68
//  Wire.write(0x6B);               // Accessing the register 6B 
//  Wire.write(0x00);               // Setting SLEEP register to 0
//  Wire.endTransmission(true); 
//  
//  Wire.beginTransmission(0x68);
//  Wire.write(0x1C);                  // Talk to the ACCEL_CONFIG register (1C hex)
//  Wire.write(0x08);                  // Set the register bits as 00001000 (+/- 4g full scale range)
//  Wire.endTransmission(true);
//  
//  Wire.beginTransmission(0x68);
//  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
//  Wire.write(0x08);                   // Set the register bits as 00001000 (500deg/s full scale) / 65.5
//  Wire.endTransmission(true);
//
//  Wire.beginTransmission(0x68);           // Start I2C communication and send I2C slave address 0x68 to MPU 
//  Wire.write(0x1A);                       // Send address to register 0x1A
//  Wire.write(0x03);                       // Write 0x03 to register 0x1A to select DLPF bandwidth 44Hz
//  Wire.endTransmission(true);
//  
//  delay(20);
//}
//
//bool readDataFromIMU6050(float *acc_x, float *acc_y, float *acc_z, float *gy_x, float *gy_y, float *gy_z) {
//  Wire.beginTransmission(0x68);
//  Wire.write(0x3B);
//  Wire.endTransmission(false);
//  byte len = Wire.requestFrom(0x68, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
//  float temp_acc_x = (Wire.read() << 8 | Wire.read()) / 8192.0; // X-axis value
//  float temp_acc_y = (Wire.read() << 8 | Wire.read()) / 8192.0; // Y-axis value
//  float temp_acc_z = (Wire.read() << 8 | Wire.read()) / 8192.0; // Z-axis value
//
//  if((len == 0)||(temp_acc_x == 0 && temp_acc_y == 0 && temp_acc_z == 0)){
//    Serial.print(" Setup MPU6050 again:");
//    setupIMU();
//    Wire.beginTransmission(0x68);
//    Wire.write(0x3B);
//    Wire.endTransmission(false);
//    Wire.requestFrom(0x68, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
//    
//    temp_acc_x = (Wire.read() << 8 | Wire.read()) / 8192.0; // X-axis value
//    temp_acc_y = (Wire.read() << 8 | Wire.read()) / 8192.0; // Y-axis value
//    temp_acc_z = (Wire.read() << 8 | Wire.read()) / 8192.0; // Z-axis value
//  }
//  
//  // === Read gyroscope data === //
//  Wire.beginTransmission(0x68);
//  Wire.write(0x43); // Gyro data first register address 0x43
//  Wire.endTransmission(false);
//  byte len2 = Wire.requestFrom(0x68, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
//  float temp_gy_x = (Wire.read() << 8 | Wire.read()) / 65.5 - GyroErrorX; // For a 500deg/s range, divide by 65.5
//  float temp_gy_y = (Wire.read() << 8 | Wire.read()) / 65.5 - GyroErrorY;
//  float temp_gy_z = (Wire.read() << 8 | Wire.read()) / 65.5 - GyroErrorZ;
//  
//  if(isnan(temp_acc_x) || isnan(temp_acc_y) || isnan(temp_acc_z)
//     || isnan(temp_gy_x) || isnan(temp_gy_y) || isnan(temp_gy_z)) return false;
//  
//  *acc_x = temp_acc_x; *acc_y = temp_acc_y; *acc_z = temp_acc_z;
//  *gy_x = temp_gy_x; *gy_y = temp_gy_y; *gy_z = temp_gy_z;   
//
//  return true;
//}
//
//void setup() {
//  Serial.begin(115200);
//  setupIMU();
//  mpu6050.begin();
//  mpu6050.calcGyroOffsets(true);
//  
//  if (!compass.begin()) {
//    Serial.println("Couldn't find HMC5883L sensor");
//    while (1);
//  }
//  
//  Serial.println("Setup complete.");
//}
//
//void loop() {
//  float temp_ax, temp_ay, temp_az;
//  float temp_gx, temp_gy, temp_gz;
//  
//  if (readDataFromIMU6050(&temp_ax, &temp_ay, &temp_az, &temp_gx, &temp_gy, &temp_gz)) {
//    ax = temp_ax;
//    ay = temp_ay;
//    az = temp_az;
//    gx = temp_gx;
//    gy = temp_gy;
//    gz = temp_gz;
//  }
//
//  sensors_event_t event;
//  compass.getEvent(&event);
//  mx = event.magnetic.x;
//  my = event.magnetic.y;
//  mz = event.magnetic.z;
//  
//  yaw = atan2(my, mx) * 180 / M_PI;
//  
//  float yawRad = yaw * M_PI / 180.0;
//  qw = cos(yawRad / 2.0);
//  qx = 0;
//  qy = 0;
//  qz = sin(yawRad / 2.0);
//  
//  // Cập nhật gói dữ liệu
//  dataPacket[2] = (uint8_t)(ax * 100);
//  dataPacket[3] = (uint8_t)(ay * 100);
//  dataPacket[4] = (uint8_t)(az * 100);
//  dataPacket[5] = (uint8_t)(gx * 100);
//  dataPacket[6] = (uint8_t)(gy * 100);
//  dataPacket[7] = (uint8_t)(gz * 100);
//  dataPacket[8] = (uint8_t)(mx * 100);
//  dataPacket[9] = (uint8_t)(my * 100);
//  dataPacket[10] = (uint8_t)(mz * 100);
//  dataPacket[11] = (uint8_t)yaw;
//  dataPacket[12] = (uint8_t)(qw * 100);
//  dataPacket[13] = (uint8_t)(qx * 100);
//  dataPacket[14] = (uint8_t)(qy * 100);
//  dataPacket[15] = (uint8_t)(qz * 100);
//
//  // Gửi dữ liệu qua Serial
//  Serial.write(dataPacket, sizeof(dataPacket));
//  delay(10);
//}
