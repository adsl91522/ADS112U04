#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "main.h"

LOG_MODULE_REGISTER(ads112u04, LOG_LEVEL_INF);

static ads112u04_reg_t reg;
ads112u04_init_param_t init_params;

static void read_ads112u04_config(void)
{   
    ads112u04_read_reg(ADS112U04_CONFIG_0_REG, &reg.reg0.all);
    ads112u04_read_reg(ADS112U04_CONFIG_1_REG, &reg.reg1.all);
    ads112u04_read_reg(ADS112U04_CONFIG_2_REG, &reg.reg2.all);
    ads112u04_read_reg(ADS112U04_CONFIG_3_REG, &reg.reg3.all);

    LOG_DBG("reg.reg0.bit.MUX = %02x", reg.reg0.bit.MUX);
    LOG_DBG("reg.reg0.bit.GAIN = %02x", reg.reg0.bit.GAIN);
    LOG_DBG("reg.reg0.bit.PGA_BYPASS = %02x", reg.reg0.bit.PGA_BYPASS);
    LOG_DBG("reg.reg1.bit.DR = %02x", reg.reg1.bit.DR);
    LOG_DBG("reg.reg1.bit.MODE = %02x", reg.reg1.bit.MODE);
    LOG_DBG("reg.reg1.bit.CMBIT = %02x", reg.reg1.bit.CMBIT);
    LOG_DBG("reg.reg1.bit.VREF = %02x", reg.reg1.bit.VREF);
    LOG_DBG("reg.reg1.bit.TS = %02x", reg.reg1.bit.TS);
    LOG_DBG("reg.reg2.bit.DCNT = %02x", reg.reg2.bit.DCNT);
    LOG_DBG("reg.reg2.bit.CRCbits = %02x", reg.reg2.bit.CRCbits);
    LOG_DBG("reg.reg2.bit.BCS = %02x", reg.reg2.bit.BCS);
    LOG_DBG("reg.reg2.bit.IDAC = %02x",  reg.reg2.bit.IDAC);
    LOG_DBG("reg.reg3.bit.I1MUX = %02x", reg.reg3.bit.I1MUX);
    LOG_DBG("reg.reg3.bit.I2MUX = %02x", reg.reg3.bit.I2MUX);
}

static void ads112u04_init(ads112u04_init_param_t *init_params)
{
    reg.reg0.all = 0;
    reg.reg1.all = 0;
    reg.reg2.all = 0;
    reg.reg3.all = 0;

    reg.reg0.bit.MUX = init_params->inputMux;
    reg.reg0.bit.GAIN = init_params->gainLevel;
    reg.reg0.bit.PGA_BYPASS = init_params->pgaBypass;

    reg.reg1.bit.DR = init_params->dataRate;
    reg.reg1.bit.MODE = init_params->opMode;
    reg.reg1.bit.CMBIT = init_params->convMode;
    reg.reg1.bit.VREF = init_params->selectVref;
    reg.reg1.bit.TS = init_params->tempSensorEn;

    reg.reg2.bit.DCNT = init_params->dataCounterEn;
    reg.reg2.bit.CRCbits = init_params->dataCRCen;
    reg.reg2.bit.BCS = init_params->burnOutEn;
    reg.reg2.bit.IDAC = init_params->idacCurrent;

    reg.reg3.bit.I1MUX = init_params->routeIDAC1;
    reg.reg3.bit.I2MUX = init_params->routeIDAC2;

    ads112u04_write_reg(ADS112U04_CONFIG_0_REG, reg.reg0.all);
    ads112u04_write_reg(ADS112U04_CONFIG_1_REG, reg.reg1.all);
    ads112u04_write_reg(ADS112U04_CONFIG_2_REG, reg.reg2.all);
    ads112u04_write_reg(ADS112U04_CONFIG_3_REG, reg.reg3.all);
    // Read and print the new configuration (if enableDebugging has been called)
    read_ads112u04_config();
}

void ads112u04_config_mode(ads112u04_config_mode_e mode)
{
    switch (mode)
    {
    case ADS112U04_4WIRE_MODE:
        init_params.inputMux = ADS112U04_MUX_AIN0_AVSS; // Route AIN1 to AINP and AIN0 to AINN
        init_params.gainLevel = ADS112U04_GAIN_8; // Set the gain to 8
        init_params.pgaBypass = ADS112U04_PGA_ENABLED; // The PGA must be enabled for gains >= 8
        init_params.dataRate = ADS112U04_DATA_RATE_20SPS; // Set the data rate (samples per second). Defaults to 20
        init_params.opMode = ADS112U04_OP_MODE_NORMAL; // Disable turbo mode
        init_params.convMode = ADS112U04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        init_params.selectVref = ADS112U04_VREF_EXT_REF_PINS; // Use the external REF pins
        init_params.tempSensorEn = ADS112U04_TEMP_SENSOR_OFF; // Disable the temperature sensor
        init_params.dataCounterEn = ADS112U04_DCNT_DISABLE; // Disable the data counter
        init_params.dataCRCen = ADS112U04_CRC_DISABLED; // Disable CRC checking
        init_params.burnOutEn = ADS112U04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
        init_params.idacCurrent = ADS112U04_IDAC_CURRENT_1000_UA; // Set the IDAC current to 1mA
        init_params.routeIDAC1 = ADS112U04_IDAC1_AIN3; // Route IDAC1 to AIN3
        init_params.routeIDAC2 = ADS112U04_IDAC2_DISABLED; // Disable IDAC2
        break;
    case ADS112U04_3WIRE_MODE:
        init_params.inputMux = ADS112U04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
        init_params.gainLevel = ADS112U04_GAIN_8; // Set the gain to 8
        init_params.pgaBypass = ADS112U04_PGA_ENABLED; // The PGA must be enabled for gains >= 8
        init_params.dataRate = ADS112U04_DATA_RATE_20SPS; // Set the data rate (samples per second). Defaults to 20
        init_params.opMode = ADS112U04_OP_MODE_NORMAL; // Disable turbo mode
        init_params.convMode = ADS112U04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        init_params.selectVref = ADS112U04_VREF_EXT_REF_PINS; // Use the external REF pins
        init_params.tempSensorEn = ADS112U04_TEMP_SENSOR_OFF; // Disable the temperature sensor
        init_params.dataCounterEn = ADS112U04_DCNT_DISABLE; // Disable the data counter
        init_params.dataCRCen = ADS112U04_CRC_DISABLED; // Disable CRC checking
        init_params.burnOutEn = ADS112U04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
        init_params.idacCurrent = ADS112U04_IDAC_CURRENT_500_UA; // Set the IDAC current to 0.5mA
        init_params.routeIDAC1 = ADS112U04_IDAC1_AIN2; // Route IDAC1 to AIN2
        init_params.routeIDAC2 = ADS112U04_IDAC2_AIN3; // Route IDAC2 to AIN3
        break;
    case ADS112U04_2WIRE_MODE:
        init_params.inputMux = ADS112U04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
        init_params.gainLevel = ADS112U04_GAIN_8; // Set the gain to 8
        init_params.pgaBypass = ADS112U04_PGA_ENABLED; // The PGA must be enabled for gains >= 8
        init_params.dataRate = ADS112U04_DATA_RATE_20SPS; // Set the data rate (samples per second). Defaults to 20
        init_params.opMode = ADS112U04_OP_MODE_NORMAL; // Disable turbo mode
        init_params.convMode = ADS112U04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        init_params.selectVref = ADS112U04_VREF_EXT_REF_PINS; // Use the external REF pins
        init_params.tempSensorEn = ADS112U04_TEMP_SENSOR_OFF; // Disable the temperature sensor
        init_params.dataCounterEn = ADS112U04_DCNT_DISABLE; // Disable the data counter
        init_params.dataCRCen = ADS112U04_CRC_DISABLED; // Disable CRC checking
        init_params.burnOutEn = ADS112U04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
        init_params.idacCurrent = ADS112U04_IDAC_CURRENT_1000_UA; // Set the IDAC current to 1mA
        init_params.routeIDAC1 = ADS112U04_IDAC1_AIN3; // Route IDAC1 to AIN3
        init_params.routeIDAC2 = ADS112U04_IDAC2_DISABLED; // Disable IDAC2
        break;
    case ADS112U04_TEMPERATURE_MODE:
        init_params.inputMux = ADS112U04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
        init_params.gainLevel = ADS112U04_GAIN_1; // Set the gain to 1
        init_params.pgaBypass = ADS112U04_PGA_DISABLED;
        init_params.dataRate = ADS112U04_DATA_RATE_20SPS; // Set the data rate (samples per second). Defaults to 20
        init_params.opMode = ADS112U04_OP_MODE_NORMAL; // Disable turbo mode
        init_params.convMode = ADS112U04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        init_params.selectVref = ADS112U04_VREF_INTERNAL; // Use the internal 2.048V reference
        init_params.tempSensorEn = ADS112U04_TEMP_SENSOR_ON; // Enable the temperature sensor
        init_params.dataCounterEn = ADS112U04_DCNT_DISABLE; // Disable the data counter
        init_params.dataCRCen = ADS112U04_CRC_DISABLED; // Disable CRC checking
        init_params.burnOutEn = ADS112U04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
        init_params.idacCurrent = ADS112U04_IDAC_CURRENT_OFF; // Disable the IDAC current
        init_params.routeIDAC1 = ADS112U04_IDAC1_DISABLED; // Disable IDAC1
        init_params.routeIDAC2 = ADS112U04_IDAC2_DISABLED; // Disable IDAC2
        break;
    case ADS112U04_RAW_MODE:
        init_params.inputMux = ADS112U04_MUX_AIN0_AVSS; // Route AIN1 to AINP and AIN0 to AINN
        init_params.gainLevel = ADS112U04_GAIN_1; // Set the gain to 1
        init_params.pgaBypass = ADS112U04_PGA_DISABLED;
        init_params.dataRate = ADS112U04_DATA_RATE_20SPS; // Set the data rate (samples per second). Defaults to 20
        init_params.opMode = ADS112U04_OP_MODE_NORMAL; // Disable turbo mode
        init_params.convMode = ADS112U04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        init_params.selectVref = ADS112U04_VREF_INTERNAL; // Use the internal 2.048V reference
        init_params.tempSensorEn = ADS112U04_TEMP_SENSOR_OFF; // Disable the temperature sensor
        init_params.dataCounterEn = ADS112U04_DCNT_DISABLE; // Disable the data counter
        init_params.dataCRCen = ADS112U04_CRC_DISABLED; // Disable CRC checking
        init_params.burnOutEn = ADS112U04_IDAC1_DISABLED; // Disable the burn-out current
        init_params.idacCurrent = ADS112U04_IDAC_CURRENT_OFF; // Disable the IDAC current
        init_params.routeIDAC1 = ADS112U04_IDAC2_DISABLED; // Disable IDAC1
        init_params.routeIDAC2 = ADS112U04_IDAC2_DISABLED; // Disable IDAC2
        break;
    case ADS112U04_4WIRE_HI_TEMP:
        init_params.inputMux = ADS112U04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
        init_params.gainLevel = ADS112U04_GAIN_4; // Set the gain to 4
        init_params.pgaBypass = ADS112U04_PGA_ENABLED; // Enable the PGA
        init_params.dataRate = ADS112U04_DATA_RATE_20SPS; // Set the data rate (samples per second). Defaults to 20
        init_params.opMode = ADS112U04_OP_MODE_NORMAL; // Disable turbo mode
        init_params.convMode = ADS112U04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        init_params.selectVref = ADS112U04_VREF_EXT_REF_PINS; // Use the external REF pins
        init_params.tempSensorEn = ADS112U04_TEMP_SENSOR_OFF; // Disable the temperature sensor
        init_params.dataCounterEn = ADS112U04_DCNT_DISABLE; // Disable the data counter
        init_params.dataCRCen = ADS112U04_CRC_DISABLED; // Disable CRC checking
        init_params.burnOutEn = ADS112U04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
        init_params.idacCurrent = ADS112U04_IDAC_CURRENT_1000_UA; // Set the IDAC current to 1mA
        init_params.routeIDAC1 = ADS112U04_IDAC1_AIN3; // Route IDAC1 to AIN3
        init_params.routeIDAC2 = ADS112U04_IDAC2_DISABLED; // Disable IDAC2
        break;
    case ADS112U04_3WIRE_HI_TEMP:
        init_params.inputMux = ADS112U04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
        init_params.gainLevel = ADS112U04_GAIN_8; // Set the gain to 8
        init_params.pgaBypass = ADS112U04_PGA_ENABLED; // The PGA must be enabled for gains >= 8
        init_params.dataRate = ADS112U04_DATA_RATE_20SPS; // Set the data rate (samples per second). Defaults to 20
        init_params.opMode = ADS112U04_OP_MODE_NORMAL; // Disable turbo mode
        init_params.convMode = ADS112U04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        init_params.selectVref = ADS112U04_VREF_EXT_REF_PINS; // Use the external REF pins
        init_params.tempSensorEn = ADS112U04_TEMP_SENSOR_OFF; // Disable the temperature sensor
        init_params.dataCounterEn = ADS112U04_DCNT_DISABLE; // Disable the data counter
        init_params.dataCRCen = ADS112U04_CRC_DISABLED; // Disable CRC checking
        init_params.burnOutEn = ADS112U04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
        init_params.idacCurrent = ADS112U04_IDAC_CURRENT_500_UA; // Set the IDAC current to 0.5mA
        init_params.routeIDAC1 = ADS112U04_IDAC1_AIN2; // Route IDAC1 to AIN2
        init_params.routeIDAC2 = ADS112U04_IDAC2_AIN3; // Route IDAC2 to AIN3
        break;
    case ADS112U04_2WIRE_HI_TEMP:
        init_params.inputMux = ADS112U04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
        init_params.gainLevel = ADS112U04_GAIN_4; // Set the gain to 4
        init_params.pgaBypass = ADS112U04_PGA_ENABLED; // Enable the PGA
        init_params.dataRate = ADS112U04_DATA_RATE_20SPS; // Set the data rate (samples per second). Defaults to 20
        init_params.opMode = ADS112U04_OP_MODE_NORMAL; // Disable turbo mode
        init_params.convMode = ADS112U04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        init_params.selectVref = ADS112U04_VREF_EXT_REF_PINS; // Use the external REF pins
        init_params.tempSensorEn = ADS112U04_TEMP_SENSOR_OFF; // Disable the temperature sensor
        init_params.dataCounterEn = ADS112U04_DCNT_DISABLE; // Disable the data counter
        init_params.dataCRCen = ADS112U04_CRC_DISABLED; // Disable CRC checking
        init_params.burnOutEn = ADS112U04_BURN_OUT_CURRENT_OFF; // Disable the burn-out current
        init_params.idacCurrent = ADS112U04_IDAC_CURRENT_1000_UA; // Set the IDAC current to 1mA
        init_params.routeIDAC1 = ADS112U04_IDAC1_AIN3; // Route IDAC1 to AIN3
        init_params.routeIDAC2 = ADS112U04_IDAC2_DISABLED; // Disable IDAC2
        break;
    default:
        break;
    }

    ads112u04_init(&init_params);
}

void ads112u04_powerdown(void)
{
    uint8_t buf[2] = {ADS112U04_ADDRESS_PROTOCOL, ADS112U04_POWERDOWN_CMD};

    uart_clear_rx_buf(DEV_UART0);
    uart_tx_data(DEV_UART0, buf, sizeof(buf));
}

void ads112u04_start(void)
{
    uint8_t buf[2] = {ADS112U04_ADDRESS_PROTOCOL, ADS112U04_START_CMD};

    uart_clear_rx_buf(DEV_UART0);
    uart_tx_data(DEV_UART0, buf, sizeof(buf));
}

void ads112u04_reset(void)
{
    uint8_t buf[2] = {ADS112U04_ADDRESS_PROTOCOL, ADS112U04_RESET_CMD};

    uart_clear_rx_buf(DEV_UART0);
    uart_tx_data(DEV_UART0, buf, sizeof(buf));
}

void ads112u04_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t buf[2] = {ADS112U04_ADDRESS_PROTOCOL, ADS112U04_READ_CMD(reg)};
    
    uart_clear_rx_buf(DEV_UART0);
    uart_tx_data(DEV_UART0, buf, sizeof(buf));
    uart_rx_get_data_blocked(DEV_UART0, value, 2, 10);

    LOG_DBG("ads112u04_read_reg data[0] %02x, data[1] %02x", value[0] , value[1]);
}

void ads112u04_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[3] = {ADS112U04_ADDRESS_PROTOCOL, ADS112U04_WRITE_CMD(reg), value};

    uart_clear_rx_buf(DEV_UART0);
    uart_tx_data(DEV_UART0, buf, sizeof(buf));

    LOG_DBG("ads112u04_write_reg buf[0] %02x, buf[1] %02x, buf[2] %02x", buf[0], buf[1], buf[2]);
}

void ads112u04_raw_voltage(uint8_t mux, uint32_t *conversion_data)
{
    uint8_t buf[2] = {ADS112U04_ADDRESS_PROTOCOL, ADS112U04_RDATA_CMD};
    uint8_t rx_data[6];

    reg.reg0.bit.MUX = mux;
    reg.reg0.bit.GAIN = ADS112U04_GAIN_1;
    reg.reg0.bit.PGA_BYPASS = ADS112U04_PGA_DISABLED;
    ads112u04_write_reg(ADS112U04_CONFIG_0_REG, reg.reg0.all);

    read_ads112u04_config();

    uart_clear_rx_buf(DEV_UART0);
    uart_tx_data(DEV_UART0, buf, sizeof(buf));

    uart_rx_get_data_blocked(DEV_UART0, rx_data, 2, 10);

    LOG_DBG("ads112u04_raw_voltage raw = %d, %d", rx_data[0], rx_data[1]);
    *conversion_data = (rx_data[1] << 8) | rx_data[0];
    *conversion_data = *conversion_data * (2 * 2048 / 1) / 65536;
    /* converted data increased 2 times */
    *conversion_data = ((*conversion_data - 1500) * 2) + 1500;
    LOG_DBG("conversion_data %d", *conversion_data);
}

void ads112u04_voltage(uint32_t *s1, uint32_t *s2, uint32_t *s3, uint32_t *s4, uint32_t *s5, uint32_t *s6)
{
    gpio_set(DB_CS, 1);

   	ads112u04_reset();
   	ads112u04_start();
   	ads112u04_raw_voltage(ADS112U04_MUX_AIN3_AVSS, s1);

    ads112u04_reset();
    ads112u04_start();
    ads112u04_raw_voltage(ADS112U04_MUX_AIN2_AVSS, s2);

    gpio_set(DB_CS, 0);
    
    ads112u04_reset();
    ads112u04_start();
    ads112u04_raw_voltage(ADS112U04_MUX_AIN3_AVSS, s3);

    ads112u04_reset();
    ads112u04_start();
    ads112u04_raw_voltage(ADS112U04_MUX_AIN2_AVSS, s4);

    ads112u04_reset();
    ads112u04_start();
    ads112u04_raw_voltage(ADS112U04_MUX_AIN1_AVSS, s5);

   	ads112u04_reset();
   	ads112u04_start();
   	ads112u04_raw_voltage(ADS112U04_MUX_AIN0_AVSS, s6);
}
