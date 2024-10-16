#ifndef __ADS112U04_H__
#define __ADS112U04_H__

#define ADS112U04_ADDRESS_PROTOCOL 0x55

// ads112u04 Command Definitions in Table 15
#define ADS112U04_RESET_CMD             0x06     //0000 011x      Reset
#define ADS112U04_START_CMD             0x08     //0000 100x      Start/Sync
#define ADS112U04_POWERDOWN_CMD         0x02     //0000 001x      PowerDown
#define ADS112U04_RDATA_CMD             0x10     //0001 xxxx      RDATA
#define ADS112U04_RREG_CMD              0x20     //0010 rrxx      Read REG rr= register address 00 to 11
#define ADS112U04_WREG_CMD              0x40     //0100 rrxx      Write REG rr= register address 00 to 11

#define ADS112U04_WRITE_CMD(reg)     (ADS112U04_WREG_CMD | (reg << 1))    //Shift is 1-bit in ADS112U04
#define ADS112U04_READ_CMD(reg)      (ADS112U04_RREG_CMD | (reg << 1))    //Shift is 1-bit in ADS112U04

// ads112u04 Command Definitions in Table 16
#define ADS112U04_CONFIG_0_REG          0 // Configuration Register 0
#define ADS112U04_CONFIG_1_REG          1 // Configuration Register 1
#define ADS112U04_CONFIG_2_REG          2 // Configuration Register 2
#define ADS112U04_CONFIG_3_REG          3 // Configuration Register 3

/* Configuration Register 0 Field Descriptions in Table 18 */
// Input Multiplexer Configuration
#define ADS112U04_MUX_AIN0_AIN1         0x0
#define ADS112U04_MUX_AIN0_AIN2         0x1
#define ADS112U04_MUX_AIN0_AIN3         0x2
#define ADS112U04_MUX_AIN1_AIN0         0x3
#define ADS112U04_MUX_AIN1_AIN2         0x4
#define ADS112U04_MUX_AIN1_AIN3         0x5
#define ADS112U04_MUX_AIN2_AIN3         0x6
#define ADS112U04_MUX_AIN3_AIN2         0x7
#define ADS112U04_MUX_AIN0_AVSS         0x8
#define ADS112U04_MUX_AIN1_AVSS         0x9
#define ADS112U04_MUX_AIN2_AVSS         0xa
#define ADS112U04_MUX_AIN3_AVSS         0xb
#define ADS112U04_MUX_REFPmREFN         0xc
#define ADS112U04_MUX_AVDDmAVSS         0xd
#define ADS112U04_MUX_SHORTED           0xe

// Gain Configuration
#define ADS112U04_GAIN_1                0x0
#define ADS112U04_GAIN_2                0x1
#define ADS112U04_GAIN_4                0x2
#define ADS112U04_GAIN_8                0x3
#define ADS112U04_GAIN_16               0x4
#define ADS112U04_GAIN_32               0x5
#define ADS112U04_GAIN_64               0x6
#define ADS112U04_GAIN_128              0x7

// PGA Bypass (PGA is disabled when the PGA_BYPASS bit is set)
#define ADS112U04_PGA_DISABLED          0x1
#define ADS112U04_PGA_ENABLED           0x0

/* Configuration Register 1 Field Descriptions in Table 19 */
// Data Rate
#define ADS112U04_DATA_RATE_20SPS       0x0
#define ADS112U04_DATA_RATE_45SPS       0x1
#define ADS112U04_DATA_RATE_90SPS       0x2
#define ADS112U04_DATA_RATE_175SPS      0x3
#define ADS112U04_DATA_RATE_330SPS      0x4
#define ADS112U04_DATA_RATE_600SPS      0x5
#define ADS112U04_DATA_RATE_1000SPS     0x6

// Operating Mode
#define ADS112U04_OP_MODE_NORMAL        0x0
#define ADS112U04_OP_MODE_TURBO         0x1

// Conversion Mode
#define ADS112U04_CONVERSION_MODE_SINGLE_SHOT   0x0
#define ADS112U04_CONVERSION_MODE_CONTINUOUS    0x1

// Voltage Reference Selection
#define ADS112U04_VREF_INTERNAL         0x0 //2.048V internal
#define ADS112U04_VREF_EXT_REF_PINS     0x1 //REFp and REFn external
#define ADS112U04_VREF_AVDD             0x2 //Analog Supply AVDD and AVSS

// Temperature Sensor Mode
#define ADS112U04_TEMP_SENSOR_OFF       0x0
#define ADS112U04_TEMP_SENSOR_ON        0x1

/* Configuration Register 2 Field Descriptions in Table 21 */
// Conversion result ready flag
#define ADS112U04_DRDY_DISABLE          0x0
#define ADS112U04_DRDY_ENABLE           0x1

// Data Counter Enable
#define ADS112U04_DCNT_DISABLE          0x0
#define ADS112U04_DCNT_ENABLE           0x1

// Data Integrity Check Enable
#define ADS112U04_CRC_DISABLED          0x0
#define ADS112U04_CRC_INVERTED          0x1
#define ADS112U04_CRC_CRC16_ENABLED     0x2

// Burn-Out Current Source
#define ADS112U04_BURN_OUT_CURRENT_OFF  0x0
#define ADS112U04_BURN_OUT_CURRENT_ON   0x1

// IDAC Current Setting
#define ADS112U04_IDAC_CURRENT_OFF      0x0
#define ADS112U04_IDAC_CURRENT_10_UA    0x1
#define ADS112U04_IDAC_CURRENT_50_UA    0x2
#define ADS112U04_IDAC_CURRENT_100_UA   0x3
#define ADS112U04_IDAC_CURRENT_250_UA   0x4
#define ADS112U04_IDAC_CURRENT_500_UA   0x5
#define ADS112U04_IDAC_CURRENT_1000_UA  0x6
#define ADS112U04_IDAC_CURRENT_1500_UA  0x7

/* Configuration Register 3 Field Descriptions in Table 22 */
// IDAC1 Routing Configuration
#define ADS112U04_IDAC1_DISABLED        0x0
#define ADS112U04_IDAC1_AIN0            0x1
#define ADS112U04_IDAC1_AIN1            0x2
#define ADS112U04_IDAC1_AIN2            0x3
#define ADS112U04_IDAC1_AIN3            0x4
#define ADS112U04_IDAC1_REFP            0x5
#define ADS112U04_IDAC1_REFN            0x6

// IDAC2 Routing Configuration
#define ADS112U04_IDAC2_DISABLED        0x0
#define ADS112U04_IDAC2_AIN0            0x1
#define ADS112U04_IDAC2_AIN1            0x2
#define ADS112U04_IDAC2_AIN2            0x3
#define ADS112U04_IDAC2_AIN3            0x4
#define ADS112U04_IDAC2_REFP            0x5
#define ADS112U04_IDAC2_REFN            0x6

// ADC data output mode.
#define ADS112U04_ADC_MANUA_READ        0x0
#define ADS112U04_ADC_AUTO_READ         0x1

/* Configuration Register 4 Field Descriptions in Table 23 */
// GPIO2 direction control
#define ADS112U04_GPIO2_INPUT           0x0
#define ADS112U04_GPIO2_OUTPUT          0x1

// GPIO1 direction control
#define ADS112U04_GPIO1_INPUT           0x0
#define ADS112U04_GPIO1_OUTPUT          0x1

// GPIO0 direction control
#define ADS112U04_GPIO0_INPUT           0x0
#define ADS112U04_GPIO0_OUTPUT          0x1

// GPIO2/DRDY control.
#define ADS112U04_GPIO2_DAT             0x0
#define ADS112U04_GPIO2_DRDY            0x1

// GPIO2 input/output level.
#define ADS112U04_GPIO2_LOGIC_LOW       0x0
#define ADS112U04_GPIO2_LOGIC_HIGH      0x1

// GPIO1 input/output level.
#define ADS112U04_GPIO1_LOGIC_LOW       0x0
#define ADS112U04_GPIO1_LOGIC_HIGH      0x1

// GPIO0 input/output level.
#define ADS112U04_GPIO0_LOGIC_LOW       0x0
#define ADS112U04_GPIO0_LOGIC_HIGH      0x1

// Define 2/3/4-Wire, Temperature and Raw modes
typedef enum {
    ADS112U04_4WIRE_MODE,
    ADS112U04_3WIRE_MODE,
    ADS112U04_2WIRE_MODE,
    ADS112U04_TEMPERATURE_MODE,
    ADS112U04_RAW_MODE,
    ADS112U04_4WIRE_HI_TEMP,
    ADS112U04_3WIRE_HI_TEMP,
    ADS112U04_2WIRE_HI_TEMP,
} ads112u04_config_mode_e;

// struct to hold the initialisation parameters
typedef struct{
  uint8_t inputMux;
  uint8_t gainLevel;
  uint8_t pgaBypass;
  uint8_t dataRate;
  uint8_t opMode;
  uint8_t convMode;
  uint8_t selectVref;
  uint8_t tempSensorEn;
  uint8_t dataCounterEn;
  uint8_t dataCRCen;
  uint8_t burnOutEn;
  uint8_t idacCurrent;
  uint8_t routeIDAC1;
  uint8_t routeIDAC2;
} ads112u04_init_param_t;

// Bit field type register configuration
// Configuration Map register ads112u04
struct CONFIG_REG_0 {
  uint8_t PGA_BYPASS:1;                     // 0
  uint8_t GAIN:3;                           // 1-3
  uint8_t MUX:4;                            // 4-7
};
union CONFIG_REG_0_U {
  uint8_t all;
  struct CONFIG_REG_0 bit;
};

struct CONFIG_REG_1 {
  uint8_t TS:1;                             // 0
  uint8_t VREF:2;                           // 1-2
  uint8_t CMBIT:1;                          // 3
  uint8_t MODE:1;                           // 4
  uint8_t DR:3;                             // 5-7
};
union CONFIG_REG_1_U {
  uint8_t all;
  struct CONFIG_REG_1 bit;
};

struct CONFIG_REG_2 {
  uint8_t IDAC:3;                           // 0-2
  uint8_t BCS:1;                            // 3
  uint8_t CRCbits:2;                        // 4-5
  uint8_t DCNT:1;                           // 6
  uint8_t DRDY:1;                           // 7
};
union CONFIG_REG_2_U {
  uint8_t all;
  struct CONFIG_REG_2 bit;
};

struct CONFIG_REG_3 {
  uint8_t RESERVED:2;                       // 0-1
  uint8_t I2MUX:3;                          // 2-4
  uint8_t I1MUX:3;                          // 5-7
};
union CONFIG_REG_3_U {
  uint8_t all;
  struct CONFIG_REG_3 bit;
};

struct CONFIG_REG_4 {
  uint8_t GPIO0DAT:1;                       // 0            
  uint8_t GPIO1DAT:1;                       // 1     
  uint8_t GPIO2DAT:1;                       // 2      
  uint8_t GPIO2SEL:1;                       // 3        
  uint8_t GPIO0DIR:1;                       // 4
  uint8_t GPIO1DIR:1;                       // 5
  uint8_t GPIO2DIR:1;                       // 6
  uint8_t RESERVED:1;                       // 7
};
union CONFIG_REG_4_U {
  uint8_t all;
  struct CONFIG_REG_4 bit;
};

// All four registers
typedef struct {
  union CONFIG_REG_0_U reg0;
  union CONFIG_REG_1_U reg1;
  union CONFIG_REG_2_U reg2;
  union CONFIG_REG_3_U reg3;
} ads112u04_reg_t;

void ads112u04_config_mode(ads112u04_config_mode_e mode);
void ads112u04_powerdown(void);
void ads112u04_start(void);
void ads112u04_reset(void);
void ads112u04_read_reg(uint8_t reg, uint8_t *value);
void ads112u04_write_reg(uint8_t reg, uint8_t value);
void ads112u04_raw_voltage(uint8_t mux, uint32_t *conversion_data);
void ads112u04_voltage(uint32_t *s1, uint32_t *s2, uint32_t *s3, uint32_t *s4, uint32_t *s5, uint32_t *s6);

#endif