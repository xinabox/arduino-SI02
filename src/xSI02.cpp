#include "xSI02.h"
#include <Wire.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define LIS3MDL_SA1_HIGH_ADDRESS 0b0011110
#define LIS3MDL_SA1_LOW_ADDRESS 0b0011100

#define TEST_REG_ERROR -1

#define LIS3MDL_WHO_ID 0x3D

// Constructors ////////////////////////////////////////////////////////////////

xSI02::xSI02(void)
{
  _device = device_auto;

  io_timeout = 0; // 0 = no timeout
  did_timeout = false;

  _addr = 0x1D;
  _rad2deg = 180.0 / M_PI;
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
        if (device == device_auto)
        {
          device = device_LIS3MDL;
        }
      }
      // check SA1 low address unless SA1 was specified to be high
      else if (sa1 != sa1_high && testReg(LIS3MDL_SA1_LOW_ADDRESS, WHO_AM_I) == LIS3MDL_WHO_ID)
      {
        sa1 = sa1_low;
        if (device == device_auto)
        {
          device = device_LIS3MDL;
        }
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
  update();
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

bool xSI02::begin()
{
  init();
  enableDefault();
  _begin(false, 2);
  return true;
}

int xSI02::getMX()
{
  //read();
  return m.x;
}

int xSI02::getMY()
{
  //read();
  return m.y;
}

int xSI02::getMZ()
{
  //read();
  return m.z;
}

// xSI02::MMA8653(uint8_t addr)
// {
//   _addr = addr;
//   _rad2deg = 180.0 / M_PI;
// }

//begin private methods

#define MMA_8653_CTRL_REG1 0x2A
#define MMA_8653_CTRL_REG1_VALUE_ACTIVE 0x01
#define MMA_8653_CTRL_REG1_VALUE_F_READ 0x02

#define MMA_8653_CTRL_REG2 0x2B
#define MMA_8653_CTRL_REG2_RESET 0x40

#define MMA_8653_CTRL_REG3 0x2C
#define MMA_8653_CTRL_REG3_VALUE_OD 0x01

#define MMA_8653_CTRL_REG4 0x2D
#define MMA_8653_CTRL_REG4_VALUE_INT_ASLP 0x80
#define MMA_8653_CTRL_REG4_VALUE_INT_ENLP 0x10
#define MMA_8653_CTRL_REG4_VALUE_INT_FFMT 0x04
#define MMA_8653_CTRL_REG4_VALUE_INT_DRDY 0x01

#define MMA_8653_CTRL_REG5 0x2E // 1: routed to INT1

#define MMA_8653_PL_STATUS 0x10
#define MMA_8653_PL_CFG 0x11
#define MMA_8653_PL_EN 0x40

#define MMA_8653_XYZ_DATA_CFG 0x0E
#define MMA_8653_2G_MODE 0x00 //Set Sensitivity to 2g
#define MMA_8653_4G_MODE 0x01 //Set Sensitivity to 4g
#define MMA_8653_8G_MODE 0x02 //Set Sensitivity to 8g

#define MMA_8653_FF_MT_CFG 0x15
#define MMA_8653_FF_MT_CFG_ELE 0x80
#define MMA_8653_FF_MT_CFG_OAE 0x40
#define MMA_8653_FF_MT_CFG_XYZ 0x38

#define MMA_8653_FF_MT_SRC 0x16
#define MMA_8653_FF_MT_SRC_EA 0x80

#define MMA_8653_FF_MT_THS 0x17

#define MMA_8653_FF_MT_COUNT 0x18

#define MMA_8653_PULSE_CFG 0x21
#define MMA_8653_PULSE_CFG_ELE 0x80

#define MMA_8653_PULSE_SRC 0x22
#define MMA_8653_PULSE_SRC_EA 0x80

// Sample rate
#define MMA_8653_ODR_800 0x00
#define MMA_8653_ODR_400 0x08
#define MMA_8653_ODR_200 0x10
#define MMA_8653_ODR_100 0x18 // default ratio 100 samples per second
#define MMA_8653_ODR_50 0x20
#define MMA_8653_ODR_12_5 0x28
#define MMA_8653_ODR_6_25 0x30
#define MMA_8653_ODR_1_56 0x38

uint8_t xSI02::_read_register(uint8_t offset)
{
  Wire.beginTransmission(_addr);
  Wire.write(offset);
  Wire.endTransmission(false);

  Wire.requestFrom(_addr, (uint8_t)1);

  if (Wire.available())
    return Wire.read();
  return 0;
}

void xSI02::_write_register(uint8_t b, uint8_t offset)
{
  Wire.beginTransmission(_addr);
  Wire.write(offset);
  Wire.write(b);
  Wire.endTransmission();
}

void xSI02::initMotion()
{
  standby();
  _write_register(MMA_8653_FF_MT_CFG, MMA_8653_FF_MT_CFG_XYZ);
  _write_register(MMA_8653_FF_MT_THS, 0x04);
  _write_register(MMA_8653_FF_MT_COUNT, 0x00);
  _write_register(MMA_8653_CTRL_REG3, MMA_8653_CTRL_REG3_VALUE_OD);
  _write_register(MMA_8653_CTRL_REG4, MMA_8653_CTRL_REG4_VALUE_INT_FFMT);
  _write_register(MMA_8653_CTRL_REG5, MMA_8653_CTRL_REG4_VALUE_INT_FFMT);
  active();
  // return (_read_register(MMA_8653_PULSE_SRC) & MMA_8653_PULSE_SRC_EA);
}

void xSI02::standby()
{
  uint8_t reg1 = 0x00;
  Wire.beginTransmission(_addr); // Set to status reg
  Wire.write((uint8_t)MMA_8653_CTRL_REG1);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)_addr, (uint8_t)1);
  if (Wire.available())
  {
    reg1 = Wire.read();
  }
  Wire.beginTransmission(_addr); // Reset
  Wire.write((uint8_t)MMA_8653_CTRL_REG1);
  Wire.write(reg1 & ~MMA_8653_CTRL_REG1_VALUE_ACTIVE);
  Wire.endTransmission();
}

void xSI02::active()
{
  uint8_t reg1 = 0x00;
  Wire.beginTransmission(_addr); // Set to status reg
  Wire.write((uint8_t)MMA_8653_CTRL_REG1);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)_addr, (uint8_t)1);
  if (Wire.available())
  {
    reg1 = Wire.read();
  }
  Wire.beginTransmission(_addr); // Reset
  Wire.write(MMA_8653_CTRL_REG2);
  Wire.write(0x09);
  Wire.endTransmission();

  Wire.beginTransmission(_addr); // Reset
  Wire.write((uint8_t)MMA_8653_CTRL_REG1);
  Wire.write(reg1 | MMA_8653_CTRL_REG1_VALUE_ACTIVE | (_highres ? 0 : MMA_8653_CTRL_REG1_VALUE_F_READ) | MMA_8653_ODR_6_25);
  Wire.endTransmission();
}

float xSI02::geta2d(float gx, float gy)
{
  float a;

  a = gx * gx;
  a = fma(gy, gy, a);

  return sqrt(a);
}

//gets the magnitude of the 3d vector
//the formula is a^2 = x^2 + y^2 + z^2
float xSI02::geta3d(float gx, float gy, float gz)
{
  float a;

  //use floating point multiply-add cpu func
  //sometimes we get better precision
  a = gx * gx;
  a = fma(gy, gy, a);
  a = fma(gz, gz, a);

  return sqrt(a);
}

float xSI02::_getRho(float ax, float ay, float az)
{
  return geta3d(_xg, _yg, _zg);
}

float xSI02::_getPhi(float ax, float ay, float az)
{
  return atan2(ay, ax) * _rad2deg;
}

float xSI02::_getTheta(float ax, float ay, float az)
{
  float rho = _getRho(ax, ay, az);

  if (rho == 0.0)
    return NAN;
  else
    return acos(az / rho) * _rad2deg;
}

//end private methods

//begin public methods
void xSI02::_begin(bool highres, uint8_t scale)
{
  Wire.begin();

  _highres = highres;

  _scale = scale;
#ifdef _MMA_8653_FACTOR
  _step_factor = (_highres ? 0.0039 : 0.0156); // Base value at 2g setting
  if (_scale == 4)
    _step_factor *= 2;
  else if (_scale == 8)
    _step_factor *= 4;
#endif

  uint8_t wai = _read_register(0x0D); // Get Who Am I from the device.
  // return value for MMA8543Q is 0x3A

  Wire.beginTransmission(_addr); // Reset
  Wire.write(MMA_8653_CTRL_REG2);
  Wire.write(MMA_8653_CTRL_REG2_RESET);
  Wire.endTransmission();
  delay(10); // Give it time to do the reset
  standby();

#ifdef _MMA_8653_PORTRAIT_LANDSCAPE
  Wire.beginTransmission(_addr); // Set Portrait/Landscape mode
  Wire.write(MMA_8653_PL_CFG);
  Wire.write(0x80 | MMA_8653_PL_EN);
  Wire.endTransmission();
#endif

  Wire.beginTransmission(_addr);
  Wire.write(MMA_8653_XYZ_DATA_CFG);
  if (_scale == 4 || _scale == 8)
    Wire.write((_scale == 4) ? MMA_8653_4G_MODE : MMA_8653_8G_MODE);
  else // Default to 2g mode
    Wire.write((uint8_t)MMA_8653_2G_MODE);
  Wire.endTransmission();
  active();
}

uint8_t xSI02::getPLStatus()
{
  return _read_register(MMA_8653_PL_STATUS);
}

uint8_t xSI02::getPulse()
{
  _write_register(MMA_8653_PULSE_CFG, MMA_8653_PULSE_CFG_ELE);
  return (_read_register(MMA_8653_PULSE_SRC) & MMA_8653_PULSE_SRC_EA);
}

float xSI02::getXG()
{
  return _xg;
}

float xSI02::getYG()
{
  return _yg;
}

float xSI02::getZG()
{
  return _zg;
}

int8_t xSI02::getAX()
{
  return _x;
}

int8_t xSI02::getAY()
{
  return _y;
}

int8_t xSI02::getAZ()
{
  return _z;
}

float xSI02::getRho()
{
  return _getRho(_xg, _yg, _zg);
}

float xSI02::getPhi()
{
  return _getPhi(_xg, _yg, _zg);
}

float xSI02::getTheta()
{
  return _getTheta(_xg, _yg, _zg);
}

int16_t rx, ry, rz;

byte xSI02::update()
{
  Wire.beginTransmission(_addr); // Set to status reg
  Wire.write((uint8_t)0x00);

  byte error = Wire.endTransmission(false);

  Wire.requestFrom((uint8_t)_addr, (uint8_t)(_highres ? 7 : 4));
  if (Wire.available())
  {
    _stat = Wire.read();
    if (_highres)
    {
      return error;
      // _x = (int16_t)((Wire.read() << 8) || Wire.read() >> 6);
      // _y = (int16_t)((Wire.read() << 8) || Wire.read() >> 6);
      // _z = (int16_t)((Wire.read() << 8) || Wire.read() >> 6);
#ifdef _MMA_8653_FACTOR
      _xg = (_x / 64) * _step_factor;
      _yg = (_y / 64) * _step_factor;
      _zg = (_z / 64) * _step_factor;
#endif
    }
    else
    {
      _x = Wire.read();
      _y = Wire.read();
      _z = Wire.read();
#ifdef _MMA_8653_FACTOR
      _xg = _x * _step_factor;
      _yg = _y * _step_factor;
      _zg = _z * _step_factor;
#endif
    }
  }
  return error;
}

bool xSI02::setInterrupt(uint8_t type, uint8_t pin, bool on)
{
  uint8_t current_value = _read_register(0x2D);

  if (on)
    current_value |= type;
  else
    current_value &= ~(type);

  _write_register(0x2D, current_value);

  uint8_t current_routing_value = _read_register(0x2E);

  if (pin == 1)
  {
    current_routing_value &= ~(type);
  }
  else if (pin == 2)
  {
    current_routing_value |= type;
  }

  _write_register(0x2E, current_routing_value);
}

bool xSI02::disableAllInterrupts()
{
  _write_register(0x2D, 0);
}

int16_t xSI02::getRoll()
{
  return 180 * atan2(getAY() * 0.0156, sqrt(getAX() * 0.0156 * getAX() * 0.0156 + getAZ() * 0.0156 * getAZ() * 0.0156)) / PI;
}

int16_t xSI02::getPitch()
{
  return 180 * atan2(getAX() * 0.0156, sqrt(getAY() * 0.0156 * getAY() * 0.0156 + getAZ() * 0.0156 * getAZ() * 0.0156)) / PI;
}

float xSI02::getGForce()
{
  float gforce = sqrt((getAX() * 0.0156 * getAX() * 0.0156) + (getAY() * 0.0156 * getAY() * 0.0156) + (getAZ() * 0.0156 * getAZ() * 0.0156));
  return gforce;
}