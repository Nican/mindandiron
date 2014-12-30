#include <SPI.h>
#include "DW1000.h"
#include "MMRanging.h"

const int chipSelectPin = 7;

void setup() {
  Serial.begin(9600);
  Serial.println("Serial up and running!");
  delay(2000);
  
  // From DW1000.cpp ---------------------------------------------------------
  SPI.begin(); // start the SPI library:
  pinMode(chipSelectPin, OUTPUT); // initalize the  data ready and chip select pins:
  deselect(); // Chip must be deselected first
  
  Serial.println("SPI up and running!");
  delay(2000);
  
  resetAll();
  loadLDE();
  Serial.println("Reset and loading!");
  delay(2000);
  
  writeRegister8(DW1000_SYS_CFG, 3, 0x20); // enable auto reenabling receiver after error
  writeRegister8(DW1000_SYS_CFG, 2, 0x03); // enable 1024 byte frames
  
  message[0] = '\0';
//  messageRX[0] = '\0';  // This is causing serial communication to go poof?
  event_i = 0;
  counter = 0;
  startRX();
}

void loop() {
	Serial.println("Here!");
	delay(1000);
}














void startRX() {
  writeRegister8(DW1000_SYS_CTRL, 0x01, 0x01);                    // start listening for preamble by setting the RXENAB bit
}

void stopTRX() {
  writeRegister8(DW1000_SYS_CTRL, 0, 0x40);                       // disable tranceiver go back to idle mode
}

void resetAll() {
  writeRegister8(DW1000_PMSC, 0, 0x01);   // set clock to XTAL
  writeRegister8(DW1000_PMSC, 3, 0x00);   // set All reset
  delay(1);                               // wait for PLL to lock
  writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear All reset
}

void loadLDE() {                                            // initialise LDE algorithm LDELOAD User Manual p22
  writeRegister16(DW1000_PMSC, 0, 0x0301);                        // set clock to XTAL so OTP is reliable
  writeRegister16(DW1000_OTP_IF, 0x06, 0x8000);                   // set LDELOAD bit in OTP
  delay(1);
  writeRegister16(DW1000_PMSC, 0, 0x0200);                        // recover to PLL clock
}



// SPI Interface --------------------------------------------------------------
//////// Integrated
uint8_t readRegister8(uint8_t reg, uint16_t subaddress) {
  uint8_t result;
  readRegister(reg, subaddress, &result, 1);
  return result;
}

//////// Integrated
uint16_t readRegister16(uint8_t reg, uint16_t subaddress) {
  uint16_t result;
  readRegister(reg, subaddress, (uint8_t*)&result, 2);
  return result;
}

//////// Integrated
uint64_t readRegister40(uint8_t reg, uint16_t subaddress) {
  uint64_t result;
  readRegister(reg, subaddress, (uint8_t*)&result, 5);
  // only 40-Bit
  result &= 0xFFFFFFFFFFLL;
  return result;
}

//////// Integrated
void writeRegister8(uint8_t reg, uint16_t subaddress, uint8_t buffer) {
  writeRegister(reg, subaddress, &buffer, 1);
}

//////// Integrated
void writeRegister16(uint8_t reg, uint16_t subaddress, uint16_t buffer) {
  writeRegister(reg, subaddress, (uint8_t*)&buffer, 2);
}

//////// Integrated
void readRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length) {
  setupTransaction(reg, subaddress, false);
  for(int i=0; i<length; i++) {
    // get data
    buffer[i] = SPI.transfer(0x00);
  }
  deselect();
}

//////// Integrated
void writeRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length) {
  setupTransaction(reg, subaddress, true);
  for(int i=0; i<length; i++)                             // put data
    SPI.transfer(buffer[i]);
  deselect();
}

//////// Integrated
void setupTransaction(uint8_t reg, uint16_t subaddress, bool write) {
  // set read/write flag
  reg |=  (write * DW1000_WRITE_FLAG);
  select();
  if (subaddress > 0) {
    // there's a subadress, we need to set flag and send second header byte
    SPI.transfer(reg | DW1000_SUBADDRESS_FLAG);
    if (subaddress > 0x7F) {
      // sub address too long, we need to set flag and send third header byte
      // say which register address we want to access
      SPI.transfer((uint8_t)(subaddress & 0x7F) | DW1000_2_SUBADDRESS_FLAG);
      SPI.transfer((uint8_t)(subaddress >> 7));
    } else {
      SPI.transfer((uint8_t)subaddress);
    }
  } else {
    SPI.transfer(reg);
  }
}

// set CS low to start transmission
void select() {
//////// Appears to be just for the Teensy  
  // SPI.beginTransaction(DW1000Settings);
  digitalWrite(chipSelectPin, LOW);
}

// set CS high to stop transmission
void deselect() { 
  digitalWrite(chipSelectPin, HIGH);
//////// Appears to be just for the Teensy
  // SPI.endTransaction();
}
