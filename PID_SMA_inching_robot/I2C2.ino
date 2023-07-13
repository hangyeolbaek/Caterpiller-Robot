/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

const uint8_t IMUAddress2 = 0x69; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT2 = 1000; // Used to check for errors in I2C communication

uint8_t i2cWrite2(uint8_t registerAddress2, uint8_t data2, bool sendStop2) {
  return i2cWrite2(registerAddress2, &data2, 1, sendStop2); // Returns 0 on success
}

uint8_t i2cWrite2(uint8_t registerAddress2, uint8_t *data2, uint8_t length, bool sendStop2) {
  Wire.beginTransmission(IMUAddress2);
  Wire.write(registerAddress2);
  Wire.write(data2, length);
  uint8_t rcode2 = Wire.endTransmission(sendStop2); // Returns 0 on success
  if (rcode2) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode2);
  }
  return rcode2; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead2(uint8_t registerAddress2, uint8_t *data2, uint8_t nbytes2) {
  uint32_t timeOutTimer2;
  Wire.beginTransmission(IMUAddress2);
  Wire.write(registerAddress2);
  uint8_t rcode2 = Wire.endTransmission(false); // Don't release the bus
  if (rcode2) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode2);
    return rcode2; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress2, nbytes2, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes2; i++) {
    if (Wire.available())
      data2[i] = Wire.read();
    else {
      timeOutTimer2 = micros();
      while (((micros() - timeOutTimer2) < I2C_TIMEOUT2) && !Wire.available());
      if (Wire.available())
        data2[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}
