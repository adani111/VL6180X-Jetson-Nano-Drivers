#ifndef VL6180X_h
#define VL6180X_h

#include <stdint.h>

typedef bool boolean;

class VL6180X
{
  public:
    // register addresses from API vl53l0x_device.h (ordered as listed there)
    enum regAddr
    {
      IDENTIFICATION_MODEL_ID              = 0x000,
      IDENTIFICATION_MODEL_REV_MAJOR       = 0x001,
      IDENTIFICATION_MODEL_REV_MINOR       = 0x002,
      IDENTIFICATION_MODULE_REV_MAJOR      = 0x003,
      IDENTIFICATION_MODULE_REV_MINOR      = 0x004,
      IDENTIFICATION_DATE_HI               = 0x006,
      IDENTIFICATION_DATE_LO               = 0x007,
      IDENTIFICATION_TIME                  = 0x008, // 16-bit

      SYSTEM_MODE_GPIO0                    = 0x010,
      SYSTEM_MODE_GPIO1                    = 0x011,
      SYSTEM_HISTORY_CTRL                  = 0x012,
      SYSTEM_INTERRUPT_CONFIG_GPIO         = 0x014,
      SYSTEM_INTERRUPT_CLEAR               = 0x015,
      SYSTEM_FRESH_OUT_OF_RESET            = 0x016,
      SYSTEM_GROUPED_PARAMETER_HOLD        = 0x017,

      SYSRANGE_START                       = 0x018,
      SYSRANGE_THRESH_HIGH                 = 0x019,
      SYSRANGE_THRESH_LOW                  = 0x01A,
      SYSRANGE_INTERMEASUREMENT_PERIOD     = 0x01B,
      SYSRANGE_MAX_CONVERGENCE_TIME        = 0x01C,
      SYSRANGE_CROSSTALK_COMPENSATION_RATE = 0x01E, // 16-bit
      SYSRANGE_CROSSTALK_VALID_HEIGHT      = 0x021,
      SYSRANGE_EARLY_CONVERGENCE_ESTIMATE  = 0x022, // 16-bit
      SYSRANGE_PART_TO_PART_RANGE_OFFSET   = 0x024,
      SYSRANGE_RANGE_IGNORE_VALID_HEIGHT   = 0x025,
      SYSRANGE_RANGE_IGNORE_THRESHOLD      = 0x026, // 16-bit
      SYSRANGE_MAX_AMBIENT_LEVEL_MULT      = 0x02C,
      SYSRANGE_RANGE_CHECK_ENABLES         = 0x02D,
      SYSRANGE_VHV_RECALIBRATE             = 0x02E,
      SYSRANGE_VHV_REPEAT_RATE             = 0x031,

      SYSALS_START                         = 0x038,
      SYSALS_THRESH_HIGH                   = 0x03A,
      SYSALS_THRESH_LOW                    = 0x03C,
      SYSALS_INTERMEASUREMENT_PERIOD       = 0x03E,
      SYSALS_ANALOGUE_GAIN                 = 0x03F,
      SYSALS_INTEGRATION_PERIOD            = 0x040,

      RESULT_RANGE_STATUS                  = 0x04D,
      RESULT_ALS_STATUS                    = 0x04E,
      RESULT_INTERRUPT_STATUS_GPIO         = 0x04F,
      RESULT_ALS_VAL                       = 0x050, // 16-bit
      RESULT_HISTORY_BUFFER_0              = 0x052, // 16-bit
      RESULT_HISTORY_BUFFER_1              = 0x054, // 16-bit
      RESULT_HISTORY_BUFFER_2              = 0x056, // 16-bit
      RESULT_HISTORY_BUFFER_3              = 0x058, // 16-bit
      RESULT_HISTORY_BUFFER_4              = 0x05A, // 16-bit
      RESULT_HISTORY_BUFFER_5              = 0x05C, // 16-bit
      RESULT_HISTORY_BUFFER_6              = 0x05E, // 16-bit
      RESULT_HISTORY_BUFFER_7              = 0x060, // 16-bit
      RESULT_RANGE_VAL                     = 0x062,
      RESULT_RANGE_RAW                     = 0x064,
      RESULT_RANGE_RETURN_RATE             = 0x066, // 16-bit
      RESULT_RANGE_REFERENCE_RATE          = 0x068, // 16-bit
      RESULT_RANGE_RETURN_SIGNAL_COUNT     = 0x06C, // 32-bit
      RESULT_RANGE_REFERENCE_SIGNAL_COUNT  = 0x070, // 32-bit
      RESULT_RANGE_RETURN_AMB_COUNT        = 0x074, // 32-bit
      RESULT_RANGE_REFERENCE_AMB_COUNT     = 0x078, // 32-bit
      RESULT_RANGE_RETURN_CONV_TIME        = 0x07C, // 32-bit
      RESULT_RANGE_REFERENCE_CONV_TIME     = 0x080, // 32-bit

      RANGE_SCALER = 0x096, // 16-bit - see STSW-IMG003 core/inc/vl6180x_def.h

      READOUT_AVERAGING_SAMPLE_PERIOD      = 0x10A,
      FIRMWARE_BOOTUP                      = 0x119,
      FIRMWARE_RESULT_SCALER               = 0x120,
      I2C_SLAVE_DEVICE_ADDRESS             = 0x212,
      INTERLEAVED_MODE_ENABLE              = 0x2A3,
    };

    uint8_t last_status; // status of last I2C transmission
    unsigned char kI2CBus ;         // I2C bus of the VL53L0X
    int kI2CFileDescriptor ;        // File Descriptor to the VL53L0X
    int error ;

    VL6180X(void);

    int millis(void);

    void setAddress(uint8_t new_addr);
    inline uint8_t getAddress(void) { return address; }

    void init(void);
    bool openVL6180X();		// Open the I2C bus to the VL6180X
    void closeVL6180X();	// Close the I2C bus to the VL6180X

    void configureDefault(void);

    void writeReg(uint16_t reg, uint8_t value);
    void writeReg16Bit(uint16_t reg, uint16_t value);
    void writeReg32Bit(uint16_t reg, uint32_t value);
    uint8_t readReg(uint16_t reg);
    uint16_t readReg16Bit(uint16_t reg);
    uint32_t readReg32Bit(uint16_t reg);

    void setScaling(uint8_t new_scaling);
    inline uint8_t getScaling(void) { return scaling; }

    uint8_t readRangeSingle(void);
    inline uint16_t readRangeSingleMillimeters(void) { return (uint16_t)scaling * readRangeSingle(); }

    void startRangeContinuous(uint16_t period = 100);

    uint8_t readRangeContinuous(void);
    inline uint16_t readRangeContinuousMillimeters(void) { return (uint16_t)scaling * readRangeContinuous(); }

    inline void setTimeout(uint16_t timeout) { io_timeout = timeout; }
    inline uint16_t getTimeout(void) { return io_timeout; }
    bool timeoutOccurred(void);

  private:
    uint8_t address;
    uint8_t scaling;
    uint8_t ptp_offset;    
    uint16_t io_timeout;
    bool did_timeout;
    uint16_t timeout_start_ms;
};

#endif



