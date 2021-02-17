#include "xSI02.h"
#include <Wire.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define LIS3MDL_SA1_HIGH_ADDRESS  0b0011110
#define LIS3MDL_SA1_LOW_ADDRESS   0b0011100

#define TEST_REG_ERROR -1

#define LIS3MDL_WHO_ID  0x3D

// Constructors ////////////////////////////////////////////////////////////////

xSI02::xSI02(void)
{
  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in read() since the last call to timeoutOccurred()?
bool xSI02::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

void xSI02::setTimeout(uint16_t timeout)
{
  io_timeout = timeout;
}

uint16_t xSI02::getTimeout()
{
  return io_timeout;
}

bool xSI02::init(deviceType device, sa1State sa1)
{
  // perform auto-detection unless device type and SA1 state were both specified
  if (device == device_auto || sa1 == sa1_auto)
  {
    // check for LIS3MDL if device is unidentified or was specified to be this type
    if (device == device_auto || device == device_LIS3MDL)
    {
      // check SA1 high address unless SA1 was specified to be low
      if (sa1 != sa1_low && testReg(LIS3MDL_SA1_HIGH_ADDRESS, WHO_AM_I) == LIS3MDL_WHO_ID)
      {
        sa1 = sa1_high;
        if (device == device_auto) { device = device_LIS3MDL; }
      }
      // check SA1 low address unless SA1 was specified to be high
      else if (sa1 != sa1_high && testReg(LIS3MDL_SA1_LOW_ADDRESS, WHO_AM_I) == LIS3MDL_WHO_ID)
      {
        sa1 = sa1_low;
        if (device == device_auto) { device = device_LIS3MDL; }
      }
    }

    // make sure device and SA1 were successfully detected; otherwise, indicate failure
    if (device == device_auto || sa1 == sa1_auto)
    {
      return false;
    }
  }

  _device = device;

  switch (device)
  {
    case device_LIS3MDL:
      address = (sa1 == sa1_high) ? LIS3MDL_SA1_HIGH_ADDRESS : LIS3MDL_SA1_LOW_ADDRESS;
      break;
  }

  return true;
}

/*
Enables the LIS3MDL's magnetometer. Also:
- Selects ultra-high-performance mode for all axes
- Sets ODR (output data rate) to default power-on value of 10 Hz
- Sets magnetometer full scale (gain) to default power-on value of +/- 4 gauss
- Enables continuous conversion mode
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void xSI02::enableDefault(void)
{
  if (_device == device_LIS3MDL)
  {
    // 0x70 = 0b01110000
    // OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
    writeReg(CTRL_REG1, 0x70);

    // 0x00 = 0b00000000
    // FS = 00 (+/- 4 gauss full scale)
    writeReg(CTRL_REG2, 0x00);

    // 0x00 = 0b00000000
    // MD = 00 (continuous-conversion mode)
    writeReg(CTRL_REG3, 0x00);

    // 0x0C = 0b00001100
    // OMZ = 11 (ultra-high-performance mode for Z)
    writeReg(CTRL_REG4, 0x0C);
  }
}

// Writes a mag register
void xSI02::writeReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  last_status = Wire.endTransmission();
}

// Reads a mag register
uint8_t xSI02::readReg(uint8_t reg)
{
  uint8_t value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  last_status = Wire.endTransmission();
  Wire.requestFrom(address, (uint8_t)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

// Reads the 3 mag channels and stores them in vector m
void xSI02::read()
{
  Wire.beginTransmission(address);
  // assert MSB to enable subaddress updating
  Wire.write(OUT_X_L | 0x80);
  Wire.endTransmission();
  Wire.requestFrom(address, (uint8_t)6);

  uint16_t millis_start = millis();
  while (Wire.available() < 6)
  {
    if (io_timeout > 0 && ((uint16_t)millis() - millis_start) > io_timeout)
    {
      did_timeout = true;
      return;
    }
  }

  uint8_t xlm = Wire.read();
  uint8_t xhm = Wire.read();
  uint8_t ylm = Wire.read();
  uint8_t yhm = Wire.read();
  uint8_t zlm = Wire.read();
  uint8_t zhm = Wire.read();

  // combine high and low bytes
  m.x = (int16_t)(xhm << 8 | xlm);
  m.y = (int16_t)(yhm << 8 | ylm);
  m.z = (int16_t)(zhm << 8 | zlm);
}

void xSI02::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

int16_t xSI02::testReg(uint8_t address, regAddr reg)
{
  Wire.beginTransmission(address);
  Wire.write((uint8_t)reg);
  if (Wire.endTransmission() != 0)
  {
    return TEST_REG_ERROR;
  }

  Wire.requestFrom(address, (uint8_t)1);
  if (Wire.available())
  {
    return Wire.read();
  }
  else
  {
    return TEST_REG_ERROR;
  }
}

void xSI02::ACC_INIT()
{
    I2C_SEND(ctrl_reg1 ,0X00); // standby to be able to configure
    delay(10);

    I2C_SEND(xyz_data_cfg ,B00000000); // 2G full range mode
    delay(1);
//   I2C_SEND(xyz_data_cfg ,B00000001); // 4G full range mode
//   delay(1);
//   I2C_SEND(xyz_data_cfg ,B00000010); // 8G full range mode
//   delay(1);

    I2C_SEND(ctrl_reg1 ,B00000001); // Output data rate at 800Hz, no auto wake, no auto scale adjust, no fast read mode
    delay(1);
//   I2C_SEND(ctrl_reg1 ,B00100001); // Output data rate at 200Hz, no auto wake, no auto scale adjust, no fast read mode
//   delay(1);
//   I2C_SEND(ctrl_reg1 ,B01000001); // Output data rate at 50Hz, no auto wake, no auto scale adjust, no fast read mode
//   delay(1);
//   I2C_SEND(ctrl_reg1 ,B01110001); // Output data rate at 1.5Hz, no auto wake, no auto scale adjust, no fast read mode
//   delay(1);
}

//------------------------------------------------------------------

void xSI02::I2C_SEND(unsigned char REG_ADDRESS, unsigned  char DATA)  //SEND data to MMA7660
{

  Wire.beginTransmission(adress_acc);
  Wire.write(REG_ADDRESS);
  Wire.write(DATA);
  Wire.endTransmission();
}

//------------------------------------------------------------------

void xSI02::I2C_READ_ACC(int ctrlreg_address) //READ number data from i2c slave ctrl-reg register and return the result in a vector
{
  byte REG_ADDRESS[7];
  int accel[4];
  int i=0;
  Wire.beginTransmission(adress_acc); //=ST + (Device Adress+W(0)) + wait for ACK
  Wire.write(ctrlreg_address);  // store the register to read in the buffer of the wire library
  Wire.endTransmission(); // actually send the data on the bus -note: returns 0 if transmission OK-
  Wire.requestFrom(adress_acc,7); // read a number of byte and store them in wire.read (note: by nature, this is called an "auto-increment register adress")

  for(i=0; i<7; i++) // 7 because on datasheet p.19 if FREAD=0, on auto-increment, the adress is shifted
  // according to the datasheet, because it's shifted, outZlsb are in adress 0x00
  // so we start reading from 0x00, forget the 0x01 which is now "status" and make the adapation by ourselves
  //this gives:
  //0 = status
  //1= X_MSB
  //2= X_LSB
  //3= Y_MSB
  //4= Y_LSB
  //5= Z_MSB
  // 6= Z_LSB
  {
  REG_ADDRESS[i]=Wire.read(); //each time you read the write.read it gives you the next byte stored. The couter is reset on requestForm
}


// MMA8653FC gives the answer on 10bits. 8bits are on _MSB, and 2 are on _LSB
// this part is used to concatenate both, and then put a sign on it (the most significant bit is giving the sign)
// the explanations are on p.14 of the 'application notes' given by freescale.
      for (i=1;i<7;i=i+2)
      {
      accel[0] = (REG_ADDRESS[i+1]|((int)REG_ADDRESS[i]<<8))>>6; // X
        if (accel[0]>0x01FF) {accel[1]=(((~accel[0])+1)-0xFC00);} // note: with signed int, this code is optional
        else {accel[1]=accel[0];} // note: with signed int, this code is optional
            switch(i){
            case 1: axeXnow=accel[1];
                              break;
            case 3: axeYnow=accel[1];
                              break;
             case 5: axeZnow=accel[1];
                              break;
                }
       }

}

//------------------------------------------------------------------

void xSI02::I2C_READ_REG(int ctrlreg_address) //READ number data from i2c slave ctrl-reg register and return the result in a vector
{
  unsigned char REG_ADDRESS;
  int i=0;
  Wire.beginTransmission(adress_acc); //=ST + (Device Adress+W(0)) + wait for ACK
  Wire.write(ctrlreg_address);  // register to read
  Wire.endTransmission();
  Wire.requestFrom(adress_acc,1); // read a number of byte and store them in write received
}

bool xSI02::begin()
{
	ACC_INIT();
	init();
	enableDefault();
}

int xSI02::getAX()
{
	I2C_READ_ACC(0x00);
	return axeXnow;
}

int xSI02::getAY()
{
	I2C_READ_ACC(0x00);
	return axeYnow;
}

int xSI02::getAZ()
{
	I2C_READ_ACC(0x00);
	return axeZnow;
}

int xSI02::getMX()
{
	read();
	return m.x;
}

int xSI02::getMY()
{
	read();
	return m.y;
}

int xSI02::getMZ()
{
	read();
	return m.z;
}