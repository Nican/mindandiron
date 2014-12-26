#include <SPI.h>
//#include <Wire.h>
//#include "ADXL345.h"
//#include "L3G4200D.h"
//#include "MAG3110.h"


//Gyro Registers
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define REFERENCE 0x25
#define OUT_TEMP 0x26
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_TSH_XH 0x32
#define INT1_TSH_XL 0x33
#define INT1_TSH_YH 0x34
#define INT1_TSH_YL 0x35
#define INT1_TSH_ZH 0x36
#define INT1_TSH_ZL 0x37
#define INT1_DURATION 0x38

//ADXL345 Register Addresses
#define	DEVID		0x00	//Device ID Register
#define THRESH_TAP	0x1D	//Tap Threshold
#define	OFSX		0x1E	//X-axis offset
#define	OFSY		0x1F	//Y-axis offset
#define	OFSZ		0x20	//Z-axis offset
#define	DURATION	0x21	//Tap Duration
#define	LATENT		0x22	//Tap latency
#define	WINDOW		0x23	//Tap window
#define	THRESH_ACT	0x24	//Activity Threshold
#define	THRESH_INACT	0x25	//Inactivity Threshold
#define	TIME_INACT	0x26	//Inactivity Time
#define	ACT_INACT_CTL	0x27	//Axis enable control for activity and inactivity detection
#define	THRESH_FF	0x28	//free-fall threshold
#define	TIME_FF		0x29	//Free-Fall Time
#define	TAP_AXES	0x2A	//Axis control for tap/double tap
#define ACT_TAP_STATUS	0x2B	//Source of tap/double tap
#define	BW_RATE		0x2C	//Data rate and power mode control
#define POWER_CTL	0x2D	//Power Control Register
#define	INT_ENABLE	0x2E	//Interrupt Enable Control
#define	INT_MAP		0x2F	//Interrupt Mapping Control
#define	INT_SOURCE	0x30	//Source of interrupts
#define	DATA_FORMAT	0x31	//Data format control
#define DATAX0		0x32	//X-Axis Data 0
#define DATAX1		0x33	//X-Axis Data 1
#define DATAY0		0x34	//Y-Axis Data 0
#define DATAY1		0x35	//Y-Axis Data 1
#define DATAZ0		0x36	//Z-Axis Data 0
#define DATAZ1		0x37	//Z-Axis Data 1
#define	FIFO_CTL	0x38	//FIFO control
#define	FIFO_STATUS	0x39	//FIFO status



//Set channels for the three differetn sensors.

int Acc_chan = 10;
int Gyr_chan = 9;
int Mag_chan = 8;

//Assign the Chip Select signal to pin 10 for accelerometer, pin 9 for gyro.
int Acc_CS=10;
int Gyr_CS=9;

const int int2pin = 6;
const int int1pin = 7;

//Buffer to hold variables.

//char acc_values[10], gyr_values[10], mag_values[10];

char acc_values[10], gyr_values[10];

//Variables used to hold values.

//int x_acc, y_acc, z_acc, t_acc, x_gyr, y_gyr, z_gyr, t_gyr, x_mag, y_mag, z_mag, t_mag;

int x_acc, y_acc, z_acc, t_acc, x_gyr, y_gyr, z_gyr, t_gyr, d_t, t_write_start, t_write_end;

void setup(){
  //Start SPI communication.
  SPI.begin();
  
  //Configure SPI connection.
  //SPI_MODE0 = Clock polarity 0, Clock phase 0
  //SPI_MODE1 = Clock polarity 0, Clock phase 1
  //SPI_MODE2 = Clock polarity 1, Clock phase 0
  //SPI_MODE3 = Clock polarity 1, Clock phase 1
  SPI.setDataMode(SPI_MODE3);
  
  //Create serial connection. BAUD rate determined by device.
  Serial.begin(38400);
  
  //Set up pins being used.
  pinMode(int1pin, INPUT);
  pinMode(int2pin, INPUT);
  
  pinMode(Acc_chan, OUTPUT);
  pinMode(Gyr_chan, OUTPUT);
  //pinMode(Mag_chan, OUTPUT);
  
  digitalWrite(Acc_chan, HIGH);
  digitalWrite(Gyr_chan, HIGH);
  
  //-----------------------------------------
  
  //-----------------------------------------
  
  
  //Format Accelerometer
  //Format range +-2g, 4, 8, 16: 11, 10, 01, 00
  writeADXL(DATA_FORMAT, 0x02); //Format of write equation: writeADXL(register ID, value to write (in hex))
  
  //Configure I/O
  
  //Set data rate to 100 Hz
  writeADXL(BW_RATE, 0x0A);
  
  //Sample at 8Hz when asleep
  writeADXL(POWER_CTL, 0x38);
  
  //Format activity and inactivity
  writeADXL(INT_ENABLE, 0x18);
  
  //Map interupts for activity and inactivity
  writeADXL(INT_MAP, 0x08);
  
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeADXL(POWER_CTL, 0x08);  //Measurement mode
  readADXL(INT_SOURCE, 1, acc_values); //Clear the interrupts from the INT_SOURCE register.
  
  
  
  //---------------------------------
  
  //---------------------------------
  
  //Format Gyro
  
  setupL3G4200D(2);  // Configure L3G4200 with selectabe full scale range
  // 0: 250 dps
  // 1: 500 dps
  // 2: 2000 dps
  
}


void loop(){
  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readADXL(DATAX0, 6, acc_values);

  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
  x_acc = ((int)acc_values[1]<<8)|(int)acc_values[0];
  //The Y value is stored in values[2] and values[3].
  y_acc = ((int)acc_values[3]<<8)|(int)acc_values[2];
  //The Z value is stored in values[4] and values[5].
  z_acc = ((int)acc_values[5]<<8)|(int)acc_values[4];
  
  t_acc = millis();
  
  //x_gyr = (readL3G(0x29)&0xFF)<<8;
  //x_gyr |= (readL3G(0x28)&0xFF);
  
  //y_gyr = (readL3G(0x2B)&0xFF)<<8;
  //y_gyr |= (readL3G(0x2A)&0xFF);
  
  //z_gyr = (readL3G(0x2D)&0xFF)<<8;
  //z_gyr |= (readL3G(0x2C)&0xFF);
  
  t_gyr = millis();
  
  t_write_start = millis();
  
  //Convert the accelerometer value to G's. 
  //With 10 bits measuring over a +/-4g range we can find how to convert by using the equation:
  // Gs = Measurement Value * (G-range/(2^10)) or Gs = Measurement Value * (8/1024)
  //xg = x * 0.0078;
  //yg = y * 0.0078;
  //zg = z * 0.0078;
  
  Serial.print(x_acc);
  Serial.print(',');
  Serial.print(y_acc);
  Serial.print(',');
  Serial.print(z_acc);
  Serial.print(',');
  //Serial.print(t_acc);
  //Serial.print(',');
  //Serial.print(x_gyr);
  //Serial.print(',');
  //Serial.print(y_gyr);
  //Serial.print(',');
  //Serial.println(z_gyr);
  //Serial.print(',');
  //Serial.println(t_gyr);
  
   // t_write = millis();
  //Serial.println(t_write);
  t_write_end = micros();
  d_t = t_write_end-t_write_start;
  
  while(d_t<6667){
  t_write_end = micros();
  d_t = t_write_end-t_write_start;
  }
  
  //delay(1);
}

void writeADXL(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(Acc_CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(Acc_CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readADXL(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(Acc_CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(Acc_CS, HIGH);
}




int readL3G(byte address)
{
  int toRead;
  
  address |= 0x80;  // This tells the L3G4200D we're reading;
  
  digitalWrite(Gyr_CS, LOW);
  SPI.transfer(address);
  toRead = SPI.transfer(0x00);
  digitalWrite(Gyr_CS, HIGH);
  
  return toRead;
}

void writeL3G(byte address, byte data)
{
  address &= 0x7F;  // This to tell the L3G4200D we're writing
  
  digitalWrite(Gyr_CS, LOW);
  SPI.transfer(address);
  SPI.transfer(data);
  digitalWrite(Gyr_CS, HIGH);
}

int setupL3G4200D(byte fullScale)
{
  // Let's first check that we're communicating properly
  // The WHO_AM_I register should read 0xD3
  if(readL3G(WHO_AM_I)!=0xD3)
    return -1;
    
  // Enable x, y, z and turn off power down:
  writeL3G(CTRL_REG1, 0b00001111);
  
  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeL3G(CTRL_REG2, 0b00000000);
  
  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeL3G(CTRL_REG3, 0b00001000);
  
  // CTRL_REG4 controls the full-scale range, among other things:
  fullScale &= 0x03;
  writeL3G(CTRL_REG4, fullScale<<4);
  
  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeL3G(CTRL_REG5, 0b00000000);
}
