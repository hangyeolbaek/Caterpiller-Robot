

#include <PID_v1.h>

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <SPI.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf


Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

Kalman kalmanX2;
Kalman kalmanY2;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

const int MPU2 = 0x69, MPU1 = 0x68;

/* IMU Data2 */
double accX2, accY2, accZ2;
double gyroX2, gyroY2, gyroZ2;
int16_t tempRaw2;

double gyroXangle2, gyroYangle2; // Angle calculate using the gyro only
double compAngleX2, compAngleY2; // Calculated angle using a complementary filter
double kalAngleX2, kalAngleY2; // Calculated angle using a Kalman filter

uint32_t timer2;
uint8_t i2cData2[14]; // Buffer for I2C data

int distance = 0;
int DIR = 0;
//unsigned long time;





#define MAX6675_CS   22  //2//16
#define MAX6675_SO   23  //3//15
#define MAX6675_SCK  24  //4//14

#define MAX6675_CS1   25  //2//16
#define MAX6675_SO1   26  //3//15
#define MAX6675_SCK1  27  //4//14

#define MAX6675_CS2   28  //2//16
#define MAX6675_SO2   29  //3//15
#define MAX6675_SCK2  30  //4//14

#define MAX6675_CS3   31  //2//16
#define MAX6675_SO3   32  //3//15
#define MAX6675_SCK3  33  //4//14
//Pins
int PWM_pin = 3;

//Variables
float temperature_read = 0.0;
double temperature_read1 = 0.0;
double temperature_read2 = 0.0;
double temperature_read3 = 0.0;
float set_temperature = 100;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;



//PID constants
int kp = 9.1;   int ki = 0.3;   int kd = 1.8;
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=0.3, Ki=4, Kd=0.1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Define Variables we'll be connecting to
double Setpoint2, Input2, Output2;

//Specify the links and initial tuning parameters
double Kp2=0.3, Ki2=4, Kd2=0.1;
PID myPID2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

//Define Variables we'll be connecting to
double Setpoint3, Input3, Output3;

//Specify the links and initial tuning parameters
double Kp3=0.3, Ki3=4, Kd3=0.1;
PID myPID3(&Input3, &Output3, &Setpoint3, Kp3, Ki3, Kd3, DIRECT);

//Define Variables we'll be connecting to
double Setpoint4, Input4, Output4;

//Specify the links and initial tuning parameters
double Kp4=0.3, Ki4=4, Kd4=0.1;
PID myPID4(&Input4, &Output4, &Setpoint4, Kp4, Ki4, Kd4, DIRECT);

void setup()
{
  pinMode(PWM_pin,OUTPUT);
  TCCR2B = TCCR2B & B11111000 | 0x03;    // pin 3 and 11 PWM frequency of 980.39 Hz
  Time = millis();
  // pinMode(Relaypin,OUTPUT);         // 릴레이를 출력으로 설정
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);


  Serial.begin(9600);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor 1"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

//---------------------------------------------------
   i2cData2[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData2[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData2[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData2[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite2(0x19, i2cData2, 4, false)); // Write to all four registers at once
  while (i2cWrite2(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
/*
  while (i2cRead2(0x75, i2cData2, 1));
  if (i2cData2[0] != 0x69) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor 2"));
    while (1);
  }
*/
  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead2(0x3B, i2cData2, 6));
  accX2 = (int16_t)((i2cData2[0] << 8) | i2cData2[1]);
  accY2 = (int16_t)((i2cData2[2] << 8) | i2cData2[3]);
  accZ2 = (int16_t)((i2cData2[4] << 8) | i2cData2[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH2 // Eq. 25 and 26
  double roll2  = atan2(accY2, accZ2) * RAD_TO_DEG;
  double pitch2 = atan(-accX2 / sqrt(accY2 * accY2 + accZ2 * accZ2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll2  = atan(accY2 / sqrt(accX2 * accX2 + accZ2 * accZ2)) * RAD_TO_DEG;
  double pitch2 = atan2(-accX2, accZ2) * RAD_TO_DEG;
#endif

  kalmanX2.setAngle(roll2); // Set starting angle
  kalmanY2.setAngle(pitch2);
  gyroXangle2 = roll2;
  gyroYangle2 = pitch2;
  compAngleX2 = roll2;
  compAngleY2 = pitch2;

  timer = micros();
Setpoint =30;
Setpoint2=30;
Setpoint3 =20;
Setpoint4 =30;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
   //turn the PID on
  myPID2.SetMode(AUTOMATIC);
  //turn the PID on
  myPID3.SetMode(AUTOMATIC);
  //turn the PID on
  myPID4.SetMode(AUTOMATIC);

}

void loop() {


/*
Input = -1*kalAngleX2; // Anterior
  myPID.Compute();
       Input2 = kalAngleX2;
  myPID2.Compute();
  Input3 = kalAngleX; // Anterior
  myPID3.Compute();
  Input4 = -1*kalAngleX; // Anterior
  myPID4.Compute();*/
   // Input4 = -1*kalAngleX; // Anterior
 // myPID4.Compute();
  analogWrite(5,Output4);     //앞 밑
  digitalWrite(6, LOW);    //앞 위
  digitalWrite(10, LOW);    //뒤 밑
  digitalWrite(11, LOW);   //뒤 위
  A();
 /* 
Input = -1*kalAngleX2; // Anterior
  myPID.Compute();
       Input2 = kalAngleX2;
  myPID2.Compute();
  Input3 = kalAngleX; // Anterior
  myPID3.Compute();
  Input4 = -1*kalAngleX; // Anterior
  myPID4.Compute();*/
 // Input2 = kalAngleX2;
 // myPID2.Compute();
  digitalWrite(5,LOW);     //앞 밑
  digitalWrite(6, LOW);    //앞 위
  analogWrite(10,Output2);    //뒤 밑
  digitalWrite(11, LOW);   //뒤 위
    B();
  /*  
Input = -1*kalAngleX2; // Anterior
  myPID.Compute();
       Input2 = kalAngleX2;
  myPID2.Compute();
  Input3 = kalAngleX; // Anterior
  myPID3.Compute();
  Input4 = -1*kalAngleX; // Anterior
  myPID4.Compute();*/
 // Input3 = kalAngleX; // Anterior
 // myPID3.Compute();
  digitalWrite(5, LOW);     //앞 밑
  analogWrite(6, Output3);    //앞 위
  digitalWrite(10, LOW);    //뒤 밑
  digitalWrite(11, LOW);   //뒤 위
  C();
/*
Input = -1*kalAngleX2; // Anterior
  myPID.Compute();
       Input2 = kalAngleX2;
  myPID2.Compute();
  Input3 = kalAngleX; // Anterior
  myPID3.Compute();
  Input4 = -1*kalAngleX; // Anterior
  myPID4.Compute();*/
 // Input = -1*kalAngleX2; // Anterior
 // myPID.Compute();
  digitalWrite(5, LOW);     //앞 밑
  digitalWrite(6, LOW);    //앞 위
  digitalWrite(10, LOW);    //뒤 밑
  analogWrite(11, Output);   //뒤 위
  D();
  /*
Input = -1*kalAngleX2; // Anterior
  myPID.Compute();
       Input2 = kalAngleX2;
  myPID2.Compute();
  Input3 = kalAngleX; // Anterior
  myPID3.Compute();
  Input4 = -1*kalAngleX; // Anterior
  myPID4.Compute();*/
 // Input4 = -1*kalAngleX; // Anterior
 // myPID4.Compute();
  analogWrite(5,Output4);     //앞 밑
  digitalWrite(6, LOW);    //앞 위
  digitalWrite(10, LOW);    //뒤 밑
  digitalWrite(11, LOW);   //뒤 위
E();
}





void A() {
  while (true) {
    if (kalAngleX < -30.00) {
      break;
    }
    while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  //Serial.print(roll); Serial.print("\t");
  // Serial.print(gyroXangle); Serial.print("\t");
  // Serial.print(compAngleX); Serial.print("\t");
  Serial.print("     forward Angle : ");
  Serial.print(kalAngleX);






  while (i2cRead2(0x3B, i2cData2, 14));
  accX2 = (int16_t)((i2cData2[0] << 8) | i2cData2[1]);
  accY2 = (int16_t)((i2cData2[2] << 8) | i2cData2[3]);
  accZ2 = (int16_t)((i2cData2[4] << 8) | i2cData2[5]);
  tempRaw2 = (int16_t)((i2cData2[6] << 8) | i2cData2[7]);
  gyroX2 = (int16_t)((i2cData2[8] << 8) | i2cData2[9]);
  gyroY2 = (int16_t)((i2cData2[10] << 8) | i2cData2[11]);
  gyroZ2 = (int16_t)((i2cData2[12] << 8) | i2cData2[13]);;
  double dt2 = (double)(micros() - timer2) / 1000000; // Calculate delta time
  timer2 = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH_2 // Eq. 25 and 26
  double roll2  = atan2(accY2, accZ2) * RAD_TO_DEG;
  double pitch2 = atan(-accX2 / sqrt(accY2 * accY2 + accZ2 * accZ2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll2  = atan(accY2 / sqrt(accX2 * accX2 + accZ2 * accZ2)) * RAD_TO_DEG;
  double pitch2 = atan2(-accX2, accZ2) * RAD_TO_DEG;
#endif

  double gyroXrate2 = gyroX2 / 131.0; // Convert to deg/s
  double gyroYrate2 = gyroY2 / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll2 < -90 && kalAngleX2 > 90) || (roll2 > 90 && kalAngleX2 < -90)) {
    kalmanX2.setAngle(roll2);
    compAngleX2 = roll2;
    kalAngleX2 = roll2;
    gyroXangle2 = roll2;
  } else
    kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt2); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX2) > 90)
    gyroYrate2 = -gyroYrate2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt2);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch2 < -90 && kalAngleY2 > 90) || (pitch2 > 90 && kalAngleY2 < -90)) {
    kalmanY2.setAngle(pitch2);
    compAngleY2 = pitch2;
    kalAngleY2 = pitch2;
    gyroYangle2 = pitch2;
  } else
    kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt2); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY2) > 90)
    gyroXrate2 = -gyroXrate2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt2); // Calculate the angle using a Kalman filter
#endif

  gyroXangle2 += gyroXrate2 * dt2; // Calculate gyro angle without any filter
  gyroYangle2 += gyroYrate2 * dt2;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX2 = 0.93 * (compAngleX2 + gyroXrate2 * dt2) + 0.07 * roll2; // Calculate the angle using a Complimentary filter
  compAngleY2 = 0.93 * (compAngleY2 + gyroYrate2 * dt2) + 0.07 * pitch2;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle2 < -180 || gyroXangle2 > 180)
    gyroXangle2 = kalAngleX2;
  if (gyroYangle2 < -180 || gyroYangle2 > 180)
    gyroYangle2 = kalAngleY2;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX2); Serial.print("\t");
  Serial.print(accY2); Serial.print("\t");
  Serial.print(accZ2); Serial.print("\t");

  Serial.print(gyroX2); Serial.print("\t");
  Serial.print(gyroY2); Serial.print("\t");
  Serial.print(gyroZ2); Serial.print("\t");

  Serial.print("\t");
#endif


  Serial.print("      rear Angle : ");
  Serial.print(kalAngleX2);


  Input4 = -1*kalAngleX; // Anterior
  myPID4.Compute();
  analogWrite(5,Output4);     //앞 밑
  Serial.print("     control_1 : ");  Serial.print(Output); 
Serial.print("     control_2 : ");  Serial.print(Output2);
  Serial.print("     control_3 : ");  Serial.print(Output3); 
Serial.print("     control_4 : ");  Serial.print(Output4);
/*
// First we read the real value of temperature
  temperature_read = readThermocouple();
  //Next we calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read;
  //Calculate the P value
  PID_p = kp * PID_error;
  //Calculate the I value in a range on +-3
  if(-3 < PID_error <3)
  {
    PID_i = PID_i + (ki * PID_error);
  }

  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

  //We define PWM range between 0 and 255
  if(PID_value < 0)
  {    PID_value = 0;    }
  if(PID_value > 255)  
  {    PID_value = 255;  }
  //Now we can write the PWM signal to the mosfet on digital pin D3
  analogWrite(PWM_pin,255-PID_value);
  previous_error = PID_error;     //Remember to store the previous error for next loop.*/

  delay(300);

  

    temperature_read = readThermocouple();
    Serial.print("  T_SMA(A):  "); Serial.print(temperature_read); Serial.print(" oC  ");
    temperature_read1 = readThermocouple1();
    Serial.print("  T_SMA(B):  "); Serial.print(temperature_read1); Serial.print(" oC  ");
    temperature_read2 = readThermocouple2();
    Serial.print("  T_SMA(C):  "); Serial.print(temperature_read2); Serial.print(" oC  ");
    temperature_read3 = readThermocouple3();
    Serial.print("  T_SMA(D):  "); Serial.print(temperature_read3); Serial.println(" oC  ");

/*
 Input = kalAngleX;
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
Serial.print("     roll : ");  Serial.print(Input); Serial.print("\t"); Serial.print("     control : ");  Serial.print(Output); Serial.print("\t");
*/

  }
}






void B() {
  while (true) {
    if (kalAngleX2 > 30.00) {
      break;
    }



   while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  //Serial.print(roll); Serial.print("\t");
  // Serial.print(gyroXangle); Serial.print("\t");
  // Serial.print(compAngleX); Serial.print("\t");
  Serial.print("     forward Angle : ");
  Serial.print(kalAngleX);






  while (i2cRead2(0x3B, i2cData2, 14));
  accX2 = (int16_t)((i2cData2[0] << 8) | i2cData2[1]);
  accY2 = (int16_t)((i2cData2[2] << 8) | i2cData2[3]);
  accZ2 = (int16_t)((i2cData2[4] << 8) | i2cData2[5]);
  tempRaw2 = (int16_t)((i2cData2[6] << 8) | i2cData2[7]);
  gyroX2 = (int16_t)((i2cData2[8] << 8) | i2cData2[9]);
  gyroY2 = (int16_t)((i2cData2[10] << 8) | i2cData2[11]);
  gyroZ2 = (int16_t)((i2cData2[12] << 8) | i2cData2[13]);;
  double dt2 = (double)(micros() - timer2) / 1000000; // Calculate delta time
  timer2 = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH_2 // Eq. 25 and 26
  double roll2  = atan2(accY2, accZ2) * RAD_TO_DEG;
  double pitch2 = atan(-accX2 / sqrt(accY2 * accY2 + accZ2 * accZ2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll2  = atan(accY2 / sqrt(accX2 * accX2 + accZ2 * accZ2)) * RAD_TO_DEG;
  double pitch2 = atan2(-accX2, accZ2) * RAD_TO_DEG;
#endif

  double gyroXrate2 = gyroX2 / 131.0; // Convert to deg/s
  double gyroYrate2 = gyroY2 / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll2 < -90 && kalAngleX2 > 90) || (roll2 > 90 && kalAngleX2 < -90)) {
    kalmanX2.setAngle(roll2);
    compAngleX2 = roll2;
    kalAngleX2 = roll2;
    gyroXangle2 = roll2;
  } else
    kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt2); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX2) > 90)
    gyroYrate2 = -gyroYrate2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt2);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch2 < -90 && kalAngleY2 > 90) || (pitch2 > 90 && kalAngleY2 < -90)) {
    kalmanY2.setAngle(pitch2);
    compAngleY2 = pitch2;
    kalAngleY2 = pitch2;
    gyroYangle2 = pitch2;
  } else
    kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt2); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY2) > 90)
    gyroXrate2 = -gyroXrate2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt2); // Calculate the angle using a Kalman filter
#endif

  gyroXangle2 += gyroXrate2 * dt2; // Calculate gyro angle without any filter
  gyroYangle2 += gyroYrate2 * dt2;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX2 = 0.93 * (compAngleX2 + gyroXrate2 * dt2) + 0.07 * roll2; // Calculate the angle using a Complimentary filter
  compAngleY2 = 0.93 * (compAngleY2 + gyroYrate2 * dt2) + 0.07 * pitch2;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle2 < -180 || gyroXangle2 > 180)
    gyroXangle2 = kalAngleX2;
  if (gyroYangle2 < -180 || gyroYangle2 > 180)
    gyroYangle2 = kalAngleY2;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX2); Serial.print("\t");
  Serial.print(accY2); Serial.print("\t");
  Serial.print(accZ2); Serial.print("\t");

  Serial.print(gyroX2); Serial.print("\t");
  Serial.print(gyroY2); Serial.print("\t");
  Serial.print(gyroZ2); Serial.print("\t");

  Serial.print("\t");
#endif


  Serial.print("      rear Angle : ");
  Serial.print(kalAngleX2);



       Input2 = kalAngleX2;
  myPID2.Compute();

  analogWrite(10,Output2);    //뒤 밑
  Serial.print("     control_1 : ");  Serial.print(Output); 
Serial.print("     control_2 : ");  Serial.print(Output2);
  Serial.print("     control_3 : ");  Serial.print(Output3); 
Serial.print("     control_4 : ");  Serial.print(Output4);
 /* // First we read the real value of temperature
  temperature_read = readThermocouple();
  //Next we calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read;
  //Calculate the P value
  PID_p = kp * PID_error;
  //Calculate the I value in a range on +-3
  if(-3 < PID_error <3)
  {
    PID_i = PID_i + (ki * PID_error);
  }

  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

  //We define PWM range between 0 and 255
  if(PID_value < 0)
  {    PID_value = 0;    }
  if(PID_value > 255)  
  {    PID_value = 255;  }
  //Now we can write the PWM signal to the mosfet on digital pin D3
  analogWrite(PWM_pin,255-PID_value);
  previous_error = PID_error;     //Remember to store the previous error for next loop.*/

  delay(300);

  
    temperature_read = readThermocouple();
    Serial.print("  T_SMA(A):  "); Serial.print(temperature_read); Serial.print(" oC  ");
    temperature_read1 = readThermocouple1();
    Serial.print("  T_SMA(B):  "); Serial.print(temperature_read1); Serial.print(" oC  ");
    temperature_read2 = readThermocouple2();
    Serial.print("  T_SMA(C):  "); Serial.print(temperature_read2); Serial.print(" oC  ");
    temperature_read3 = readThermocouple3();
    Serial.print("  T_SMA(D):  "); Serial.print(temperature_read3); Serial.println(" oC  ");

  }
}















void C() {

  while (true) {
    if (kalAngleX > 20.00) {
      break;
    }


    while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  //Serial.print(roll); Serial.print("\t");
  // Serial.print(gyroXangle); Serial.print("\t");
  // Serial.print(compAngleX); Serial.print("\t");
  Serial.print("     forward Angle : ");
  Serial.print(kalAngleX);






  while (i2cRead2(0x3B, i2cData2, 14));
  accX2 = (int16_t)((i2cData2[0] << 8) | i2cData2[1]);
  accY2 = (int16_t)((i2cData2[2] << 8) | i2cData2[3]);
  accZ2 = (int16_t)((i2cData2[4] << 8) | i2cData2[5]);
  tempRaw2 = (int16_t)((i2cData2[6] << 8) | i2cData2[7]);
  gyroX2 = (int16_t)((i2cData2[8] << 8) | i2cData2[9]);
  gyroY2 = (int16_t)((i2cData2[10] << 8) | i2cData2[11]);
  gyroZ2 = (int16_t)((i2cData2[12] << 8) | i2cData2[13]);;
  double dt2 = (double)(micros() - timer2) / 1000000; // Calculate delta time
  timer2 = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH_2 // Eq. 25 and 26
  double roll2  = atan2(accY2, accZ2) * RAD_TO_DEG;
  double pitch2 = atan(-accX2 / sqrt(accY2 * accY2 + accZ2 * accZ2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll2  = atan(accY2 / sqrt(accX2 * accX2 + accZ2 * accZ2)) * RAD_TO_DEG;
  double pitch2 = atan2(-accX2, accZ2) * RAD_TO_DEG;
#endif

  double gyroXrate2 = gyroX2 / 131.0; // Convert to deg/s
  double gyroYrate2 = gyroY2 / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll2 < -90 && kalAngleX2 > 90) || (roll2 > 90 && kalAngleX2 < -90)) {
    kalmanX2.setAngle(roll2);
    compAngleX2 = roll2;
    kalAngleX2 = roll2;
    gyroXangle2 = roll2;
  } else
    kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt2); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX2) > 90)
    gyroYrate2 = -gyroYrate2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt2);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch2 < -90 && kalAngleY2 > 90) || (pitch2 > 90 && kalAngleY2 < -90)) {
    kalmanY2.setAngle(pitch2);
    compAngleY2 = pitch2;
    kalAngleY2 = pitch2;
    gyroYangle2 = pitch2;
  } else
    kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt2); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY2) > 90)
    gyroXrate2 = -gyroXrate2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt2); // Calculate the angle using a Kalman filter
#endif

  gyroXangle2 += gyroXrate2 * dt2; // Calculate gyro angle without any filter
  gyroYangle2 += gyroYrate2 * dt2;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX2 = 0.93 * (compAngleX2 + gyroXrate2 * dt2) + 0.07 * roll2; // Calculate the angle using a Complimentary filter
  compAngleY2 = 0.93 * (compAngleY2 + gyroYrate2 * dt2) + 0.07 * pitch2;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle2 < -180 || gyroXangle2 > 180)
    gyroXangle2 = kalAngleX2;
  if (gyroYangle2 < -180 || gyroYangle2 > 180)
    gyroYangle2 = kalAngleY2;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX2); Serial.print("\t");
  Serial.print(accY2); Serial.print("\t");
  Serial.print(accZ2); Serial.print("\t");

  Serial.print(gyroX2); Serial.print("\t");
  Serial.print(gyroY2); Serial.print("\t");
  Serial.print(gyroZ2); Serial.print("\t");

  Serial.print("\t");
#endif


  Serial.print("      rear Angle : ");
  Serial.print(kalAngleX2);



  Input3 = kalAngleX; // Anterior
  myPID3.Compute();

  analogWrite(6, Output3);    //앞 위
  Serial.print("     control_1 : ");  Serial.print(Output); 
Serial.print("     control_2 : ");  Serial.print(Output2);
  Serial.print("     control_3 : ");  Serial.print(Output3); 
Serial.print("     control_4 : ");  Serial.print(Output4);

  /*// First we read the real value of temperature
  temperature_read = readThermocouple();
  //Next we calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read;
  //Calculate the P value
  PID_p = kp * PID_error;
  //Calculate the I value in a range on +-3
  if(-3 < PID_error <3)
  {
    PID_i = PID_i + (ki * PID_error);
  }

  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

  //We define PWM range between 0 and 255
  if(PID_value < 0)
  {    PID_value = 0;    }
  if(PID_value > 255)  
  {    PID_value = 255;  }
  //Now we can write the PWM signal to the mosfet on digital pin D3
  analogWrite(PWM_pin,255-PID_value);
  previous_error = PID_error;     //Remember to store the previous error for next loop.*/

  delay(300);

  
    temperature_read = readThermocouple();
    Serial.print("  T_SMA(A):  "); Serial.print(temperature_read); Serial.print(" oC  ");
    temperature_read1 = readThermocouple1();
    Serial.print("  T_SMA(B):  "); Serial.print(temperature_read1); Serial.print(" oC  ");
    temperature_read2 = readThermocouple2();
    Serial.print("  T_SMA(C):  "); Serial.print(temperature_read2); Serial.print(" oC  ");
    temperature_read3 = readThermocouple3();
    Serial.print("  T_SMA(D):  "); Serial.print(temperature_read3); Serial.println(" oC  ");
  }
}



void D() {

  while (true) {
    if (kalAngleX2 < -30.00) {
      break;
    }


    while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  //Serial.print(roll); Serial.print("\t");
  // Serial.print(gyroXangle); Serial.print("\t");
  // Serial.print(compAngleX); Serial.print("\t");
  Serial.print("     forward Angle : ");
  Serial.print(kalAngleX);






  while (i2cRead2(0x3B, i2cData2, 14));
  accX2 = (int16_t)((i2cData2[0] << 8) | i2cData2[1]);
  accY2 = (int16_t)((i2cData2[2] << 8) | i2cData2[3]);
  accZ2 = (int16_t)((i2cData2[4] << 8) | i2cData2[5]);
  tempRaw2 = (int16_t)((i2cData2[6] << 8) | i2cData2[7]);
  gyroX2 = (int16_t)((i2cData2[8] << 8) | i2cData2[9]);
  gyroY2 = (int16_t)((i2cData2[10] << 8) | i2cData2[11]);
  gyroZ2 = (int16_t)((i2cData2[12] << 8) | i2cData2[13]);;
  double dt2 = (double)(micros() - timer2) / 1000000; // Calculate delta time
  timer2 = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH_2 // Eq. 25 and 26
  double roll2  = atan2(accY2, accZ2) * RAD_TO_DEG;
  double pitch2 = atan(-accX2 / sqrt(accY2 * accY2 + accZ2 * accZ2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll2  = atan(accY2 / sqrt(accX2 * accX2 + accZ2 * accZ2)) * RAD_TO_DEG;
  double pitch2 = atan2(-accX2, accZ2) * RAD_TO_DEG;
#endif

  double gyroXrate2 = gyroX2 / 131.0; // Convert to deg/s
  double gyroYrate2 = gyroY2 / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll2 < -90 && kalAngleX2 > 90) || (roll2 > 90 && kalAngleX2 < -90)) {
    kalmanX2.setAngle(roll2);
    compAngleX2 = roll2;
    kalAngleX2 = roll2;
    gyroXangle2 = roll2;
  } else
    kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt2); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX2) > 90)
    gyroYrate2 = -gyroYrate2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt2);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch2 < -90 && kalAngleY2 > 90) || (pitch2 > 90 && kalAngleY2 < -90)) {
    kalmanY2.setAngle(pitch2);
    compAngleY2 = pitch2;
    kalAngleY2 = pitch2;
    gyroYangle2 = pitch2;
  } else
    kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt2); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY2) > 90)
    gyroXrate2 = -gyroXrate2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt2); // Calculate the angle using a Kalman filter
#endif

  gyroXangle2 += gyroXrate2 * dt2; // Calculate gyro angle without any filter
  gyroYangle2 += gyroYrate2 * dt2;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX2 = 0.93 * (compAngleX2 + gyroXrate2 * dt2) + 0.07 * roll2; // Calculate the angle using a Complimentary filter
  compAngleY2 = 0.93 * (compAngleY2 + gyroYrate2 * dt2) + 0.07 * pitch2;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle2 < -180 || gyroXangle2 > 180)
    gyroXangle2 = kalAngleX2;
  if (gyroYangle2 < -180 || gyroYangle2 > 180)
    gyroYangle2 = kalAngleY2;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX2); Serial.print("\t");
  Serial.print(accY2); Serial.print("\t");
  Serial.print(accZ2); Serial.print("\t");

  Serial.print(gyroX2); Serial.print("\t");
  Serial.print(gyroY2); Serial.print("\t");
  Serial.print(gyroZ2); Serial.print("\t");

  Serial.print("\t");
#endif


  Serial.print("      rear Angle : ");
  Serial.print(kalAngleX2);



Input = -1*kalAngleX2; // Anterior
  myPID.Compute();

  analogWrite(11, Output);   //뒤 위
  Serial.print("     control_1 : ");  Serial.print(Output); 
Serial.print("     control_2 : ");  Serial.print(Output2);
  Serial.print("     control_3 : ");  Serial.print(Output3); 
Serial.print("     control_4 : ");  Serial.print(Output4);
  /*// First we read the real value of temperature
  temperature_read = readThermocouple();
  //Next we calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read;
  //Calculate the P value
  PID_p = kp * PID_error;
  //Calculate the I value in a range on +-3
  if(-3 < PID_error <3)
  {
    PID_i = PID_i + (ki * PID_error);
  }

  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

  //We define PWM range between 0 and 255
  if(PID_value < 0)
  {    PID_value = 0;    }
  if(PID_value > 255)  
  {    PID_value = 255;  }
  //Now we can write the PWM signal to the mosfet on digital pin D3
  analogWrite(PWM_pin,255-PID_value);
  previous_error = PID_error;     //Remember to store the previous error for next loop.*/

  delay(300);

  
    temperature_read = readThermocouple();
    Serial.print("  T_SMA(A):  "); Serial.print(temperature_read); Serial.print(" oC  ");
    temperature_read1 = readThermocouple1();
    Serial.print("  T_SMA(B):  "); Serial.print(temperature_read1); Serial.print(" oC  ");
    temperature_read2 = readThermocouple2();
    Serial.print("  T_SMA(C):  "); Serial.print(temperature_read2); Serial.print(" oC  ");
    temperature_read3 = readThermocouple3();
    Serial.print("  T_SMA(D):  "); Serial.print(temperature_read3); Serial.println(" oC  ");
  }
}


void E() {
  while (true) {
    if (kalAngleX < -30.00) {
      break;
    }
    while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  //Serial.print(roll); Serial.print("\t");
  // Serial.print(gyroXangle); Serial.print("\t");
  // Serial.print(compAngleX); Serial.print("\t");
  Serial.print("     forward Angle : ");
  Serial.print(kalAngleX);






  while (i2cRead2(0x3B, i2cData2, 14));
  accX2 = (int16_t)((i2cData2[0] << 8) | i2cData2[1]);
  accY2 = (int16_t)((i2cData2[2] << 8) | i2cData2[3]);
  accZ2 = (int16_t)((i2cData2[4] << 8) | i2cData2[5]);
  tempRaw2 = (int16_t)((i2cData2[6] << 8) | i2cData2[7]);
  gyroX2 = (int16_t)((i2cData2[8] << 8) | i2cData2[9]);
  gyroY2 = (int16_t)((i2cData2[10] << 8) | i2cData2[11]);
  gyroZ2 = (int16_t)((i2cData2[12] << 8) | i2cData2[13]);;
  double dt2 = (double)(micros() - timer2) / 1000000; // Calculate delta time
  timer2 = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH_2 // Eq. 25 and 26
  double roll2  = atan2(accY2, accZ2) * RAD_TO_DEG;
  double pitch2 = atan(-accX2 / sqrt(accY2 * accY2 + accZ2 * accZ2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll2  = atan(accY2 / sqrt(accX2 * accX2 + accZ2 * accZ2)) * RAD_TO_DEG;
  double pitch2 = atan2(-accX2, accZ2) * RAD_TO_DEG;
#endif

  double gyroXrate2 = gyroX2 / 131.0; // Convert to deg/s
  double gyroYrate2 = gyroY2 / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll2 < -90 && kalAngleX2 > 90) || (roll2 > 90 && kalAngleX2 < -90)) {
    kalmanX2.setAngle(roll2);
    compAngleX2 = roll2;
    kalAngleX2 = roll2;
    gyroXangle2 = roll2;
  } else
    kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt2); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX2) > 90)
    gyroYrate2 = -gyroYrate2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt2);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch2 < -90 && kalAngleY2 > 90) || (pitch2 > 90 && kalAngleY2 < -90)) {
    kalmanY2.setAngle(pitch2);
    compAngleY2 = pitch2;
    kalAngleY2 = pitch2;
    gyroYangle2 = pitch2;
  } else
    kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt2); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY2) > 90)
    gyroXrate2 = -gyroXrate2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt2); // Calculate the angle using a Kalman filter
#endif

  gyroXangle2 += gyroXrate2 * dt2; // Calculate gyro angle without any filter
  gyroYangle2 += gyroYrate2 * dt2;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX2 = 0.93 * (compAngleX2 + gyroXrate2 * dt2) + 0.07 * roll2; // Calculate the angle using a Complimentary filter
  compAngleY2 = 0.93 * (compAngleY2 + gyroYrate2 * dt2) + 0.07 * pitch2;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle2 < -180 || gyroXangle2 > 180)
    gyroXangle2 = kalAngleX2;
  if (gyroYangle2 < -180 || gyroYangle2 > 180)
    gyroYangle2 = kalAngleY2;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX2); Serial.print("\t");
  Serial.print(accY2); Serial.print("\t");
  Serial.print(accZ2); Serial.print("\t");

  Serial.print(gyroX2); Serial.print("\t");
  Serial.print(gyroY2); Serial.print("\t");
  Serial.print(gyroZ2); Serial.print("\t");

  Serial.print("\t");
#endif


  Serial.print("      rear Angle : ");
  Serial.print(kalAngleX2);


  Input4 = -1*kalAngleX; // Anterior
  myPID4.Compute();
  analogWrite(5,Output4);     //앞 밑
  Serial.print("     control_1 : ");  Serial.print(Output); 
Serial.print("     control_2 : ");  Serial.print(Output2);
  Serial.print("     control_3 : ");  Serial.print(Output3); 
Serial.print("     control_4 : ");  Serial.print(Output4);
/*
// First we read the real value of temperature
  temperature_read = readThermocouple();
  //Next we calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read;
  //Calculate the P value
  PID_p = kp * PID_error;
  //Calculate the I value in a range on +-3
  if(-3 < PID_error <3)
  {
    PID_i = PID_i + (ki * PID_error);
  }

  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

  //We define PWM range between 0 and 255
  if(PID_value < 0)
  {    PID_value = 0;    }
  if(PID_value > 255)  
  {    PID_value = 255;  }
  //Now we can write the PWM signal to the mosfet on digital pin D3
  analogWrite(PWM_pin,255-PID_value);
  previous_error = PID_error;     //Remember to store the previous error for next loop.*/

  delay(300);

  

    temperature_read = readThermocouple();
    Serial.print("  T_SMA(A):  "); Serial.print(temperature_read); Serial.print(" oC  ");
    temperature_read1 = readThermocouple1();
    Serial.print("  T_SMA(B):  "); Serial.print(temperature_read1); Serial.print(" oC  ");
    temperature_read2 = readThermocouple2();
    Serial.print("  T_SMA(C):  "); Serial.print(temperature_read2); Serial.print(" oC  ");
    temperature_read3 = readThermocouple3();
    Serial.print("  T_SMA(D):  "); Serial.print(temperature_read3); Serial.println(" oC  ");

/*
 Input = kalAngleX;
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
Serial.print("     roll : ");  Serial.print(Input); Serial.print("\t"); Serial.print("     control : ");  Serial.print(Output); Serial.print("\t");
*/

  }
}





double readThermocouple() {

  uint16_t v;
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);
  pinMode(MAX6675_SCK, OUTPUT);
  
  digitalWrite(MAX6675_CS, LOW);
  delay(1);

  // Read in 16 bits,
  //  15    = 0 always
  //  14..2 = 0.25 degree counts MSB First
  //  2     = 1 if thermocouple is open circuit  
  //  1..0  = uninteresting status
  
  v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  
  digitalWrite(MAX6675_CS, HIGH);
  if (v & 0x4) 
  {    
    // Bit 2 indicates if the thermocouple is disconnected
    return NAN;     
  }

  // The lower three bits (0,1,2) are discarded status bits
  v >>= 3;

  // The remaining bits are the number of 0.25 degree (C) counts
  return v*0.25;
}


double readThermocouple1() {

  uint16_t v1;
  pinMode(MAX6675_CS1, OUTPUT);
  pinMode(MAX6675_SO1, INPUT);
  pinMode(MAX6675_SCK1, OUTPUT);
  
  digitalWrite(MAX6675_CS1, LOW);
  delay(1);

  // Read in 16 bits,
  //  15    = 0 always
  //  14..2 = 0.25 degree counts MSB First
  //  2     = 1 if thermocouple is open circuit  
  //  1..0  = uninteresting status
  
  v1 = shiftIn(MAX6675_SO1, MAX6675_SCK1, MSBFIRST);
  v1 <<= 8;
  v1 |= shiftIn(MAX6675_SO1, MAX6675_SCK1, MSBFIRST);
  
  digitalWrite(MAX6675_CS1, HIGH);
  if (v1 & 0x4) 
  {    
    // Bit 2 indicates if the thermocouple is disconnected
    return NAN;     
  }

  // The lower three bits (0,1,2) are discarded status bits
  v1 >>= 3;

  // The remaining bits are the number of 0.25 degree (C) counts
  return v1*0.25;
}

double readThermocouple2() {

  uint16_t v2;
  pinMode(MAX6675_CS2, OUTPUT);
  pinMode(MAX6675_SO2, INPUT);
  pinMode(MAX6675_SCK2, OUTPUT);
  
  digitalWrite(MAX6675_CS2, LOW);
  delay(1);

  // Read in 16 bits,
  //  15    = 0 always
  //  14..2 = 0.25 degree counts MSB First
  //  2     = 1 if thermocouple is open circuit  
  //  1..0  = uninteresting status
  
  v2 = shiftIn(MAX6675_SO2, MAX6675_SCK2, MSBFIRST);
  v2 <<= 8;
  v2 |= shiftIn(MAX6675_SO2, MAX6675_SCK2, MSBFIRST);
  
  digitalWrite(MAX6675_CS2, HIGH);
  if (v2 & 0x4) 
  {    
    // Bit 2 indicates if the thermocouple is disconnected
    return NAN;     
  }

  // The lower three bits (0,1,2) are discarded status bits
  v2 >>= 3;

  // The remaining bits are the number of 0.25 degree (C) counts
  return v2*0.25;
}

double readThermocouple3() {

  uint16_t v3;
  pinMode(MAX6675_CS3, OUTPUT);
  pinMode(MAX6675_SO3, INPUT);
  pinMode(MAX6675_SCK3, OUTPUT);
  
  digitalWrite(MAX6675_CS3, LOW);
  delay(1);

  // Read in 16 bits,
  //  15    = 0 always
  //  14..2 = 0.25 degree counts MSB First
  //  2     = 1 if thermocouple is open circuit  
  //  1..0  = uninteresting status
  
  v3 = shiftIn(MAX6675_SO3, MAX6675_SCK3, MSBFIRST);
  v3 <<= 8;
  v3 |= shiftIn(MAX6675_SO3, MAX6675_SCK3, MSBFIRST);
  
  digitalWrite(MAX6675_CS3, HIGH);
  if (v3 & 0x4) 
  {    
    // Bit 2 indicates if the thermocouple is disconnected
    return NAN;     
  }

  // The lower three bits (0,1,2) are discarded status bits
  v3 >>= 3;

  // The remaining bits are the number of 0.25 degree (C) counts
  return v3*0.25;
}
