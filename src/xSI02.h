#ifndef xSI02_h
#define xSI02_h

#include <Arduino.h>

#define MMA_8653_DEFAULT_ADDRESS 0x1D

// Auto SLEEP/WAKE interrupt
#define INT_ASLP   (1<<7)
// Transient interrupt
#define INT_TRANS  (1<<5)
// Orientation (landscape/portrait) interrupt
#define INT_LNDPRT (1<<4)
// Pulse detection interrupt
#define INT_PULSE  (1<<3)
// Freefall/Motion interrupt
#define INT_FF_MT  (1<<2)
// Data ready interrupt
#define INT_DRDY   (1<<0)

class xSI02
{
public:
	xSI02();

	template <typename T>
	struct vector
	{
		T x, y, z;
	};

	enum deviceType
	{
		device_LIS3MDL,
		device_auto
	};
	enum sa1State
	{
		sa1_low,
		sa1_high,
		sa1_auto
	};

	// register addresses
	enum regAddr
	{
		WHO_AM_I = 0x0F,

		CTRL_REG1 = 0x20,
		CTRL_REG2 = 0x21,
		CTRL_REG3 = 0x22,
		CTRL_REG4 = 0x23,
		CTRL_REG5 = 0x24,

		STATUS_REG = 0x27,
		OUT_X_L = 0x28,
		OUT_X_H = 0x29,
		OUT_Y_L = 0x2A,
		OUT_Y_H = 0x2B,
		OUT_Z_L = 0x2C,
		OUT_Z_H = 0x2D,
		TEMP_OUT_L = 0x2E,
		TEMP_OUT_H = 0x2F,
		INT_CFG = 0x30,
		INT_SRC = 0x31,
		INT_THS_L = 0x32,
		INT_THS_H = 0x33,
	};

	vector<int16_t> m; // magnetometer readings

	uint8_t last_status; // status of last I2C transmission

	bool init(deviceType device = device_auto, sa1State sa1 = sa1_auto);
	deviceType getDeviceType(void) { return _device; }

	void enableDefault(void);

	void writeReg(uint8_t reg, uint8_t value);
	uint8_t readReg(uint8_t reg);

	void read(void);

	void setTimeout(uint16_t timeout);
	uint16_t getTimeout(void);
	bool timeoutOccurred(void);

	// vector functions
	template <typename Ta, typename Tb, typename To>
	static void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
	template <typename Ta, typename Tb>
	static float vector_dot(const vector<Ta> *a, const vector<Tb> *b);
	static void vector_normalize(vector<float> *a);

	bool begin();

	int getMX();
	int getMY();
	int getMZ();


    //MMA8653(uint8_t addr = MMA_8653_DEFAULT_ADDRESS);
    void _begin(bool highres = true, uint8_t scale = 2);
    float getXG();
    float getYG();
    float getZG();
    int8_t getAX();
    int8_t getAY();
    int8_t getAZ();
    float getRho();
    float getPhi();
    float getTheta();
	float getGForce();
	int16_t getRoll();
	int16_t getPitch();
    byte update();

    uint8_t getPLStatus();
    uint8_t getPulse();

    // Interrupts
    bool setInterrupt(uint8_t type, uint8_t pin, bool on);
    bool disableAllInterrupts();
    void initMotion();
    void standby();
    void active();

private:
	deviceType _device; // chip type
	uint8_t address;

	uint16_t io_timeout;
	bool did_timeout;

	int16_t testReg(uint8_t address, regAddr reg);

    uint8_t _read_register(uint8_t offset);
    void _write_register(uint8_t b, uint8_t offset);

    float geta2d(float gx, float gy);
    float geta3d(float gx, float gy, float gz);
    float _getRho(float ax, float ay, float az);
    float _getPhi(float ax, float ay, float az);
    float _getTheta(float ax, float ay, float az);

    uint8_t _addr;
    uint8_t _stat;
    uint8_t _scale;
    int16_t _x;
    int16_t _y;
    int16_t _z;
    float _step_factor;
    bool _highres;
    float _xg;
    float _yg;
    float _zg;
    float _rad2deg;

};

template <typename Ta, typename Tb, typename To> void xSI02::vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float xSI02::vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

#endif
