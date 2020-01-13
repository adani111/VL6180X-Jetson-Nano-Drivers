// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

#include <VL6180X.h>
#include <sys/time.h>
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <cstddef>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <errno.h>
#include <iostream>
// for delay function.
#include <chrono>
#include <thread>
// for signal handling
#include <signal.h>
#include <JetsonGPIO.h>

extern "C" {
#include <i2c/smbus.h>
}

using namespace std;

// Pin Definitions
int output_pin = 6; // BOARD pin 31, BCM pin 6

inline void delay(int s){
        this_thread::sleep_for(chrono::seconds(s));
}

void VL6180X::gpio(void)
{
        // Pin Setup. 
        GPIO::setmode(GPIO::BCM);
        // set pin as an output pin with optional initial state of HIGH
        GPIO::setup(output_pin, GPIO::OUT, GPIO::HIGH);
	
	uint16_t value;	
	bool check = true;
	while(check){
        	GPIO::output(output_pin, GPIO::HIGH);
		delay(0.001);
        	value = readReg16Bit(SYSTEM_FRESH_OUT_OF_RESET);
		printf("%d \n", value);
		if(value != 0) { 
			writeReg16Bit(SYSTEM_FRESH_OUT_OF_RESET, 0);
			check = false;
		}
		else {
			//printf("GPIO0 not configured correctly");
			GPIO::output(output_pin, GPIO::LOW);
			delay(0.001);
		}
	}
}

// Defines /////////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
// #define ADDRESS_DEFAULT 0b0101001
#define ADDRESS_DEFAULT 0x29

// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = millis())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)millis() - timeout_start_ms) > io_timeout)

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// RANGE_SCALER values for 1x, 2x, 3x scaling - see STSW-IMG003 core/src/vl6180x_api.c (ScalerLookUP[])
static uint16_t const ScalerValues[] = {0, 253, 127, 84};

// Constructors ////////////////////////////////////////////////////////////////

VL6180X::VL6180X(void)
    : address(ADDRESS_DEFAULT)
    , scaling(0)
    , ptp_offset(0)
    , io_timeout(0) // no timeout
    , did_timeout(false)
    , kI2CBus(0)
{
}

// Returns true if device file descriptor opens correctly, false otherwise
//
bool VL6180X::openVL6180X()
{
    char fileNameBuffer[32];
    sprintf(fileNameBuffer,"/dev/i2c-%d", kI2CBus);
    kI2CFileDescriptor = open(fileNameBuffer, O_RDWR);
    if (kI2CFileDescriptor < 0) {
        // Could not open the file
        error = errno ;
        return false ;
    }
    if (ioctl(kI2CFileDescriptor, I2C_SLAVE, address) < 0) {
        // Could not open the device on the bus
        error = errno ;
        return false ;
    }
    return true ;
}

void VL6180X::closeVL6180X()
{
    if (kI2CFileDescriptor > 0) {
        close(kI2CFileDescriptor);
        // WARNING - This is not quite right, need to check for error first
        kI2CFileDescriptor = -1 ;
    }
}


// Public Methods //////////////////////////////////////////////////////////////

int VL6180X::millis(void){
    struct timeval myTime;
    gettimeofday(&myTime, NULL);
    return (myTime.tv_sec ) * 1000 + (myTime.tv_usec ) / 1000;
}

void VL6180X::setAddress(uint8_t new_addr)
{
    writeReg(I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    address = new_addr;
}

// Initialize sensor using sequence based on VL6180X datasheet appendix,
void VL6180X::init()
{
  // Store part-to-part range offset so it can be adjusted if scaling is changed
  ptp_offset = readReg(SYSRANGE_PART_TO_PART_RANGE_OFFSET);

  if (readReg(SYSTEM_FRESH_OUT_OF_RESET) == 1)
  {
    scaling = 1;

    writeReg(0x207, 0x01);
    writeReg(0x208, 0x01);
    writeReg(0x096, 0x00);
    writeReg(0x097, 0xFD); // RANGE_SCALER = 253
    writeReg(0x0E3, 0x00);
    writeReg(0x0E4, 0x04);
    writeReg(0x0E5, 0x02);
    writeReg(0x0E6, 0x01);
    writeReg(0x0E7, 0x03);
    writeReg(0x0F5, 0x02);
    writeReg(0x0D9, 0x05);
    writeReg(0x0DB, 0xCE);
    writeReg(0x0DC, 0x03);
    writeReg(0x0DD, 0xF8);
    writeReg(0x09F, 0x00);
    writeReg(0x0A3, 0x3C);
    writeReg(0x0B7, 0x00);
    writeReg(0x0BB, 0x3C);
    writeReg(0x0B2, 0x09);
    writeReg(0x0CA, 0x09);
    writeReg(0x198, 0x01);
    writeReg(0x1B0, 0x17);
    writeReg(0x1AD, 0x00);
    writeReg(0x0FF, 0x05);
    writeReg(0x100, 0x05);
    writeReg(0x199, 0x05);
    writeReg(0x1A6, 0x1B);
    writeReg(0x1AC, 0x3E);
    writeReg(0x1A7, 0x1F);
    writeReg(0x030, 0x00);

  }
  else
  {
    // Sensor has already been initialized, so try to get scaling settings by
    // reading registers.

    uint16_t s = readReg16Bit(RANGE_SCALER);

    if      (s == ScalerValues[3]) { scaling = 3; }
    else if (s == ScalerValues[2]) { scaling = 2; }
    else                           { scaling = 1; }

    // Adjust the part-to-part range offset value read earlier to account for
    // existing scaling. If the sensor was already in 2x or 3x scaling mode,
    // precision will be lost calculating the original (1x) offset, but this can
    // be resolved by resetting the sensor awriteReg16Bit(SYSTEM_FRESH_OUT_OF_RESET, 0);nd Arduino again.
    ptp_offset *= scaling;
  }
}

// Configure some settings for the sensor's default behavior from AN4545 -
// "Recommended : Public registers" and "Optional: Public registers"
//
// Note that this function does not set up GPIO1 as an interrupt output as
// suggested, though you can do so by calling:
// writeReg(SYSTEM__MODE_GPIO1, 0x10);
void VL6180X::configureDefault(void)
{
  // "Recommended : Public registers"

  // readout__averaging_sample_period = 48
  writeReg(READOUT_AVERAGING_SAMPLE_PERIOD, 0x30);

  // sysals__analogue_gain_light = 6 (ALS gain = 1 nominal, actually 1.01 according to Table 14 in datasheet)
  writeReg(SYSALS_ANALOGUE_GAIN, 0x46);

  // sysrange__vhv_repeat_rate = 255 (auto Very High Voltage temperature recalibration after every 255 range measurements)
  writeReg(SYSRANGE_VHV_REPEAT_RATE, 0xFF);

  // sysals__integration_period = 99 (100 ms)
  // AN4545 incorrectly recommends writing to register 0x040; 0x63 should go in the lower byte, which is register 0x041.
  writeReg16Bit(SYSALS_INTEGRATION_PERIOD, 0x0063);

  // sysrange__vhv_recalibrate = 1 (manually trigger a VHV recalibration)
  writeReg(SYSRANGE_VHV_RECALIBRATE, 0x01);


  // "Optional: Public registers"

  // sysrange__intermeasurement_period = 9 (100 ms)
  writeReg(SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09);

  // sysals__intermeasurement_period = 49 (500 ms)
  writeReg(SYSALS_INTERMEASUREMENT_PERIOD, 0x31);

  // als_int_mode = 4 (ALS new sample ready interrupt); range_int_mode = 4 (range new sample ready interrupt)
  writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24);


  // Reset other settings to power-on defaults

  // sysrange__max_convergence_time = 49 (49 ms)
  writeReg(VL6180X::SYSRANGE_MAX_CONVERGENCE_TIME, 0x31);

  // disable interleaved mode
  writeReg(INTERLEAVED_MODE_ENABLE, 0);

  // reset range scaling factor to 1x
  setScaling(1);
}

// Write an 8-bit register
void VL6180X::writeReg(uint16_t reg, uint8_t value)
{
    int toReturn = i2c_smbus_write_byte_data(kI2CFileDescriptor, reg, value);
    // Wait a little bit to make sure it settles
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    last_status = toReturn ;
}

// Write a 16-bit register
void VL6180X::writeReg16Bit(uint16_t reg, uint16_t value)
{
    int toReturn = i2c_smbus_write_word_data(kI2CFileDescriptor, reg, value);
    // Wait a little bit to make sure it settles
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    last_status = toReturn ;
}

// Write a 32-bit register
void VL6180X::writeReg32Bit(uint16_t reg, uint32_t value)
{
    uint8_t buffer[4];
writeReg16Bit(SYSTEM_FRESH_OUT_OF_RESET, 0);    buffer[0] = (value >> 24) & 0xFF ;
    buffer[1] = (value >> 16) & 0xFF ;
    buffer[2] = (value >>  8) & 0xFF ;
    buffer[3] = (value      ) & 0xFF ;

    int toReturn = i2c_smbus_write_block_data(kI2CFileDescriptor, reg, 4, buffer);
    // Wait a little bit to make sure it settles
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    last_status = toReturn ;
}

// Read an 8-bit register
uint8_t VL6180X::readReg(uint16_t reg)
{
    uint8_t value;
    value = i2c_smbus_read_byte_data(kI2CFileDescriptor, reg) ;
    if (value < 0) {
        error = errno ;
        last_status = -1 ;
    }
    return value ;
}

// Read a 16-bit register
uint16_t VL6180X::readReg16Bit(uint16_t reg)
{
    uint16_t value;

    value = i2c_smbus_read_word_data(kI2CFileDescriptor, reg) ;
    if (value < 0) {
        error = errno ;
        last_status = -1 ;
    }
    return value ;
}

// Read a 32-bit register
uint32_t VL6180X::readReg32Bit(uint16_t reg)
{
    uint8_t value[4];
    uint32_t buffer ;
    int toReturn ;

    toReturn = i2c_smbus_read_block_data(kI2CFileDescriptor,reg,value) ;
    if (toReturn < 0) {
        error = errno ;
    }
    last_status = toReturn ;
    buffer  = (uint32_t)value[0] << 24;
    buffer |= (uint32_t)value[1] << 16;
    buffer |= (uint32_t)value[2] <<  8;
    buffer |= (uint32_t)value[3]      ;
    return buffer ;
}

// Set range scaling factor. The sensor uses 1x scaling by default, giving range
// measurements in units of mm. Increasing the scaling to 2x or 3x makes it give
// raw values in units of 2 mm or 3 mm instead. In other words, a bigger scaling
// factor increases the sensor's potential maximum range but reduces its
// resolution.

// Implemented using ST's VL6180X API as a reference (STSW-IMG003); see
// VL6180x_UpscaleSetScaling() in vl6180x_api.c.
void VL6180X::setScaling(uint8_t new_scaling)
{
  uint8_t const DefaultCrosstalkValidHeight = 20; // default value of SYSRANGE__CROSSTALK_VALID_HEIGHT

  // do nothing if scaling value is invalid
  if (new_scaling < 1 || new_scaling > 3) { return; }

  scaling = new_scaling;
  writeReg16Bit(RANGE_SCALER, ScalerValues[scaling]);

  // apply scaling on part-to-part offset
  writeReg(VL6180X::SYSRANGE_PART_TO_PART_RANGE_OFFSET, ptp_offset / scaling);

  // apply scaling on CrossTalkValidHeight
  writeReg(VL6180X::SYSRANGE_CROSSTALK_VALID_HEIGHT, DefaultCrosstalkValidHeight / scaling);

  // This function does not apply scaling to RANGE_IGNORE_VALID_HEIGHT.

  // enable early convergence estimate only at 1x scaling
  uint8_t rce = readReg(VL6180X::SYSRANGE_RANGE_CHECK_ENABLES);
  writeReg(VL6180X::SYSRANGE_RANGE_CHECK_ENABLES, (rce & 0xFE) | (scaling == 1));
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
uint8_t VL6180X::readRangeSingle()
{
  writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  writeReg(SYSRANGE_START, 0x01);
  return readRangeContinuous();
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint8_t VL6180X::readRangeContinuous()
{
    startTimeout();
    uint8_t value = readReg(RESULT_INTERRUPT_STATUS_GPIO);
    printf("%d",value);
    while (readReg(RESULT_INTERRUPT_STATUS_GPIO) != 0x04)
    {
        if (checkTimeoutExpired())
        {
            did_timeout = true;
            return 255;
        }
    }

    uint8_t range = readReg(RESULT_RANGE_VAL);
    writeReg(SYSTEM_INTERRUPT_CLEAR, 0x07);

    return range;
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool VL6180X::timeoutOccurred()
{
    bool tmp = did_timeout;
    did_timeout = false;
    return tmp;
}
