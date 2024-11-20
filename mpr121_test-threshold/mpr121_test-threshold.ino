#include <Wire.h>
#include "mpr121.h"

// Define the I2C address for MPR121
#define MPR121_ADDR 0x5A

// Define new threshold values
#define NEW_TOU_THRESH 0x38  // Touch threshold
#define NEW_REL_THRESH 0x75  // Release threshold

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPR121
  configureMPR121();

  Serial.println("MPR121 capacitive touch sensor test with custom thresholds...");
}

void loop() {
  // Read touch status from MPR121
  uint16_t touchStatus = readTouchStatus();

  // Iterate over each electrode (0-11)
  for (int i = 0; i < 12; i++) {
    if (touchStatus & (1 << i)) {
      Serial.print("Electrode ");
      Serial.print(i);
      Serial.println(" touched");
      Serial.print("Touch strength (hex): ");
      Serial.println(readTouchStrength(i), HEX);
    } else {
      Serial.print("Electrode ");
      Serial.print(i);
      Serial.println(" released");
      Serial.print("Release strength (hex): ");
      Serial.println(readReleaseStrength(i), HEX);
    }
  }

  delay(1000);  // Delay for readability in the serial output
}

void configureMPR121() {
  // Stop MPR121 to set registers
  setRegister(MPR121_ADDR, ELE_CFG, 0x00);

  // Set thresholds for all electrodes
  for (int i = 0; i < 12; i++) {
    setRegister(MPR121_ADDR, ELE0_T + i * 2, NEW_TOU_THRESH);  // Touch threshold
    setRegister(MPR121_ADDR, ELE0_R + i * 2, NEW_REL_THRESH);  // Release threshold
  }

  // Configure other necessary registers
  setRegister(MPR121_ADDR, MHD_R, 0x01);
  setRegister(MPR121_ADDR, NHD_R, 0x01);
  setRegister(MPR121_ADDR, NCL_R, 0x00);
  setRegister(MPR121_ADDR, FDL_R, 0x00);

  setRegister(MPR121_ADDR, MHD_F, 0x01);
  setRegister(MPR121_ADDR, NHD_F, 0x01);
  setRegister(MPR121_ADDR, NCL_F, 0xFF);
  setRegister(MPR121_ADDR, FDL_F, 0x02);

  setRegister(MPR121_ADDR, FIL_CFG, 0x04);
  setRegister(MPR121_ADDR, ELE_CFG, 0x0C);  // Enable all 12 electrodes
}

uint16_t readTouchStatus() {
  Wire.beginTransmission(MPR121_ADDR);
  Wire.write(0x00);  // Touch status register
  Wire.endTransmission(false);
  Wire.requestFrom(MPR121_ADDR, 2);

  uint16_t touchStatus = Wire.read() | (Wire.read() << 8);
  return touchStatus;
}

uint8_t readTouchStrength(int electrode) {
  // Returns the touch strength for a given electrode
  uint8_t strength = 0;
  Wire.beginTransmission(MPR121_ADDR);
  Wire.write(ELE0_T + electrode * 2);
  Wire.endTransmission(false);
  Wire.requestFrom(MPR121_ADDR, 1);
  if (Wire.available()) {
    strength = Wire.read();
  }
  return strength;
}

uint8_t readReleaseStrength(int electrode) {
  // Returns the release strength for a given electrode
  uint8_t strength = 0;
  Wire.beginTransmission(MPR121_ADDR);
  Wire.write(ELE0_R + electrode * 2);
  Wire.endTransmission(false);
  Wire.requestFrom(MPR121_ADDR, 1);
  if (Wire.available()) {
    strength = Wire.read();
  }
  return strength;
}

void setRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
