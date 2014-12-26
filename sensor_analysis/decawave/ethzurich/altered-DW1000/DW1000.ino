#include "DW1000.h"

const int chipSelectPin = 7;

void setup() {
  Serial.begin(9600);

  // start the SPI library:
  SPI.begin();

  // initalize the  data ready and chip select pins:
  pinMode(chipSelectPin, OUTPUT);

  // Chip must be deselected first
  deselect();

  // give the sensor time to set up:
  delay(100);
}
 
DW1000(PinName MOSI, PinName MISO, PinName SCLK, PinName CS, PinName IRQ) : irq(IRQ), spi(MOSI, MISO, SCLK), cs(CS) {
  setCallbacks(NULL, NULL);
  
  deselect();                         // Chip must be deselected first
  spi.format(8,0);                    // Setup the spi for standard 8 bit data and SPI-Mode 0 (GPIO5, GPIO6 open circuit or ground on DW1000)
  spi.frequency(1000000);             // with a 1MHz clock rate (worked up to 49MHz in our Test)
  
  resetAll();                         // we can do a soft reset if we want to (only needed for debugging)
  loadLDE();                          // important everytime DW1000 initialises/awakes otherwise the LDE algorithm must be turned of or there's receiving malfunction see User Manual LDELOAD on p22 & p158
  
  // Configuration TODO: make method for that
  writeRegister8(DW1000_SYS_CFG, 3, 0x20); // enable auto reenabling receiver after error
  writeRegister8(DW1000_SYS_CFG, 2, 0x03); // enable 1024 byte frames
  
  irq.rise(this, &ISR);       // attach Interrupt handler to rising edge
}
 
void setCallbacks(void (*callbackRX)(void), void (*callbackTX)(void)) {
  bool RX = false;
  bool TX = false;
  if (callbackRX) {
    callbackRX.attach(callbackRX);
    RX = true;
  }
  if (callbackTX) {
    callbackTX.attach(callbackTX);
    TX = true;
  }
  setInterrupt(RX,TX);
}
 
uint32_t getDeviceID() {
  uint32_t result;
  readRegister(DW1000_DEV_ID, 0, (uint8_t*)&result, 4);
  return result;
}
 
uint64_t getEUI() {
  uint64_t result;
  readRegister(DW1000_EUI, 0, (uint8_t*)&result, 8);
  return result;
}
 
void setEUI(uint64_t EUI) {
  writeRegister(DW1000_EUI, 0, (uint8_t*)&EUI, 8);
}
 
float getVoltage() {
  uint8_t buffer[7] = {0x80, 0x0A, 0x0F, 0x01, 0x00};             // algorithm form User Manual p57
  writeRegister(DW1000_RF_CONF, 0x11, buffer, 2);
  writeRegister(DW1000_RF_CONF, 0x12, &buffer[2], 1);
  writeRegister(DW1000_TX_CAL, 0x00, &buffer[3], 1);
  writeRegister(DW1000_TX_CAL, 0x00, &buffer[4], 1);
  readRegister(DW1000_TX_CAL, 0x03, &buffer[5], 2);               // get the 8-Bit readings for Voltage and Temperature
  float Voltage = buffer[5] * 0.0057 + 2.3;
  //float Temperature = buffer[6] * 1.13 - 113.0;                 // TODO: getTemperature was always ~35 degree with better formula/calibration see instance_common.c row 391
  return Voltage;
}
 
uint64_t getStatus() {
  return readRegister40(DW1000_SYS_STATUS, 0);
}
 
uint64_t getRXTimestamp() {
  return readRegister40(DW1000_RX_TIME, 0);
}
 
uint64_t getTXTimestamp() {
  return readRegister40(DW1000_TX_TIME, 0);
}
 
void sendString(char* message) {
  sendFrame((uint8_t*)message, strlen(message)+1);
}
 
void receiveString(char* message) {
  uint16_t framelength = getFramelength();
  readRegister(DW1000_RX_BUFFER, 0, (uint8_t*)message, framelength);  // get data from buffer
}
 
void sendFrame(uint8_t* message, uint16_t length) {
  if (length >= 1021) length = 1021;                              // check for maximim length a frame can have            TODO: 127 Byte mode?
  writeRegister(DW1000_TX_BUFFER, 0, message, length);            // fill buffer
  
  uint8_t backup = readRegister8(DW1000_TX_FCTRL, 1);             // put length of frame
  length += 2;                                                    // including 2 CRC Bytes
  length = ((backup & 0xFC) << 8) | (length & 0x03FF);
  writeRegister16(DW1000_TX_FCTRL, 0, length);
  
  stopTRX();                                                      // stop receiving
  writeRegister8(DW1000_SYS_CTRL, 0, 0x02);                       // trigger sending process by setting the TXSTRT bit
  startRX();                                                      // enable receiver again
}
 
void startRX() {
  writeRegister8(DW1000_SYS_CTRL, 0x01, 0x01);                    // start listening for preamble by setting the RXENAB bit
}
 
void stopTRX() {
  writeRegister8(DW1000_SYS_CTRL, 0, 0x40);                       // disable tranceiver go back to idle mode
}
 
// PRIVATE Methods ------------------------------------------------------------------------------------
void loadLDE() {                                            // initialise LDE algorithm LDELOAD User Manual p22
  writeRegister16(DW1000_PMSC, 0, 0x0301);                        // set clock to XTAL so OTP is reliable
  writeRegister16(DW1000_OTP_IF, 0x06, 0x8000);                   // set LDELOAD bit in OTP
  wait_us(150);
  writeRegister16(DW1000_PMSC, 0, 0x0200);                        // recover to PLL clock
}
 
void resetRX() {    
  writeRegister8(DW1000_PMSC, 3, 0xE0);   // set RX reset
  writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear RX reset
}
 
void resetAll() {
  writeRegister8(DW1000_PMSC, 0, 0x01);   // set clock to XTAL
  writeRegister8(DW1000_PMSC, 3, 0x00);   // set All reset
  wait_us(10);                            // wait for PLL to lock
  writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear All reset
}
 
 
void setInterrupt(bool RX, bool TX) {
  writeRegister16(DW1000_SYS_MASK, 0, RX*0x4000 | TX*0x0080);  // RX good frame 0x4000, TX done 0x0080
}
 
void ISR() {
  uint64_t status = getStatus();
  if (status & 0x4000) {                                          // a frame was received
    callbackRX.call();
    writeRegister16(DW1000_SYS_STATUS, 0, 0x6F00);              // clearing of receiving status bits
  }
  if (status & 0x80) {                                            // sending complete
    callbackTX.call();
    writeRegister8(DW1000_SYS_STATUS, 0, 0xF8);                 // clearing of sending status bits
  }
}
 
uint16_t getFramelength() {
  uint16_t framelength = readRegister16(DW1000_RX_FINFO, 0);      // get framelength
  framelength = (framelength & 0x03FF) - 2;                       // take only the right bits and subtract the 2 CRC Bytes
  return framelength;
}
 
// SPI Interface ------------------------------------------------------------------------------------
uint8_t readRegister8(uint8_t reg, uint16_t subaddress) {
  uint8_t result;
  readRegister(reg, subaddress, &result, 1);
  return result;
}
 
uint16_t readRegister16(uint8_t reg, uint16_t subaddress) {
  uint16_t result;
  readRegister(reg, subaddress, (uint8_t*)&result, 2);
  return result;
}
 
uint64_t readRegister40(uint8_t reg, uint16_t subaddress) {
  uint64_t result;
  readRegister(reg, subaddress, (uint8_t*)&result, 5);
  result &= 0xFFFFFFFFFF;                                 // only 40-Bit
  return result;
}
 
void writeRegister8(uint8_t reg, uint16_t subaddress, uint8_t buffer) {
  writeRegister(reg, subaddress, &buffer, 1);
}
 
void writeRegister16(uint8_t reg, uint16_t subaddress, uint16_t buffer) {
  writeRegister(reg, subaddress, (uint8_t*)&buffer, 2);
}
 
void readRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length) {
  setupTransaction(reg, subaddress, false);
  for(int i=0; i<length; i++)                             // get data
    buffer[i] = spi.write(0x00);
  deselect();
}
 
void writeRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length) {
  setupTransaction(reg, subaddress, true);
  for(int i=0; i<length; i++)                             // put data
    spi.write(buffer[i]);
  deselect();
}
 
void setupTransaction(uint8_t reg, uint16_t subaddress, bool write) {
  reg |=  (write * DW1000_WRITE_FLAG);                                        // set read/write flag
  select();
  if (subaddress > 0) {                                                       // there's a subadress, we need to set flag and send second header byte
    spi.write(reg | DW1000_SUBADDRESS_FLAG);
    if (subaddress > 0x7F) {                                                // sub address too long, we need to set flag and send third header byte
      spi.write((uint8_t)(subaddress & 0x7F) | DW1000_2_SUBADDRESS_FLAG); // and 
      spi.write((uint8_t)(subaddress >> 7));
    } else {
      spi.write((uint8_t)subaddress);
    }
  } else {
    spi.write(reg);                                                         // say which register address we want to access
  }
}

// set CS low to start transmission
void select() {
  digitalWrite(chipSelectPin, LOW);
}

// set CS high to stop transmission
void deselect() { 
  digitalWrite(chipSelectPin, HIGH);
}