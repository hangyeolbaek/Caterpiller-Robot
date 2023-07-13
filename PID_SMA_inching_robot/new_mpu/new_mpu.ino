#include <Wire.h>
#define mpu_add 0x68 //mpu6050 address
 
long ac_x, ac_y, ac_z, gy_x, gy_y, gy_z ; //acc, gyro data 
 
double angle = 0, deg ; // angle, deg data
double dgy_x ; //double type acc data
 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600) ;  //set serial baud
  Wire.begin() ;  //set I2C
  Wire.beginTransmission(mpu_add) ;
  Wire.write(0x6B) ;
  Wire.write(0) ;
  Wire.endTransmission(true) ;
}
 
void loop() {
  // put your main code here, to run repeatedly:
 
  Wire.beginTransmission(mpu_add) ; //get acc data
  Wire.write(0x3B) ;
  Wire.endTransmission(false) ;
  Wire.requestFrom(mpu_add, 6, true) ;
  ac_x = Wire.read() << 8 | Wire.read() ;
  ac_y = Wire.read() << 8 | Wire.read() ;
  ac_z = Wire.read() << 8 | Wire.read() ;
 
  Wire.beginTransmission(mpu_add) ; //get gyro data
  Wire.write(0x43) ;
  Wire.endTransmission(false) ;
  Wire.requestFrom(mpu_add, 6, true) ;
  gy_x = Wire.read() << 8 | Wire.read() ;
  gy_y = Wire.read() << 8 | Wire.read() ;
  gy_z = Wire.read() << 8 | Wire.read() ;
 
  deg = atan2(ac_x, ac_z) * 180 / PI ;  //rad to deg
 
  dgy_x = gy_y / 131. ;  //16-bit data to 250 deg/sec
  angle = (0.95 * (angle + (dgy_x * 0.001))) + (0.05 * deg) ; //complementary filter
  Serial.println(angle) ;
}
