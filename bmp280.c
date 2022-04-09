/*  BMP280 library (v2) for Pi Pico:
    Slight rewrite based upon RasPi official example: https://github.com/raspberrypi/pico-examples/blob/master/i2c/bmp280_i2c/bmp280_i2c.c
    Datasheet here: https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
*/


#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"


// hardware registers
#define REG_CONFIG _u(0xF5)
#define REG_CTRL_MEAS _u(0xF4)
#define REG_RESET _u(0xE0)

#define REG_TEMP_XLSB _u(0xFC)
#define REG_TEMP_LSB _u(0xFB)
#define REG_TEMP_MSB _u(0xFA)

#define REG_PRESSURE_XLSB _u(0xF9)
#define REG_PRESSURE_LSB _u(0xF8)
#define REG_PRESSURE_MSB _u(0xF7)

// calibration registers
#define REG_DIG_T1_LSB _u(0x88)
#define REG_DIG_T1_MSB _u(0x89)
#define REG_DIG_T2_LSB _u(0x8A)
#define REG_DIG_T2_MSB _u(0x8B)
#define REG_DIG_T3_LSB _u(0x8C)
#define REG_DIG_T3_MSB _u(0x8D)
#define REG_DIG_P1_LSB _u(0x8E)
#define REG_DIG_P1_MSB _u(0x8F)
#define REG_DIG_P2_LSB _u(0x90)
#define REG_DIG_P2_MSB _u(0x91)
#define REG_DIG_P3_LSB _u(0x92)
#define REG_DIG_P3_MSB _u(0x93)
#define REG_DIG_P4_LSB _u(0x94)
#define REG_DIG_P4_MSB _u(0x95)
#define REG_DIG_P5_LSB _u(0x96)
#define REG_DIG_P5_MSB _u(0x97)
#define REG_DIG_P6_LSB _u(0x98)
#define REG_DIG_P6_MSB _u(0x99)
#define REG_DIG_P7_LSB _u(0x9A)
#define REG_DIG_P7_MSB _u(0x9B)
#define REG_DIG_P8_LSB _u(0x9C)
#define REG_DIG_P8_MSB _u(0x9D)
#define REG_DIG_P9_LSB _u(0x9E)
#define REG_DIG_P9_MSB _u(0x9F)

// number of calibration registers to be read
#define NUM_CALIB_PARAMS 24

#define RW_TIMEOUT  100000 // 0.1 second

struct bmp280_calib_param {
    // temperature params
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;

    // pressure params
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
};


// Local functions
bool bmp280_read_raw(int32_t* temp, int32_t* pressure);
int32_t bmp280_convert_temp(int32_t temp, struct bmp280_calib_param* params);
int32_t bmp280_convert_pressure(int32_t pressure, int32_t temp, struct bmp280_calib_param* params);
bool bmp280_get_calib_params(struct bmp280_calib_param* params);


// Global variables 
uint8_t addr;
i2c_inst_t* i2c_bus;
struct bmp280_calib_param params;

// Setup BMP. Returns HIGH flag if error in setup.
bool BMP_Setup(i2c_inst_t* device_i2c_bus, uint8_t device_address) {
    // Assign local (global across functions) variables 
    i2c_bus = device_i2c_bus;
    addr = device_address;

    // use the "handheld device dynamic" optimal setting (see datasheet)
    uint8_t buf[2];

    // 500ms sampling time, x16 filter
    const uint8_t reg_config_val = ((0x04 << 5) | (0x05 << 2)) & 0xFC;

    // send register number followed by its corresponding value
    buf[0] = REG_CONFIG;
    buf[1] = reg_config_val;
    if(i2c_write_timeout_us(i2c_default, addr, buf, 2, false, RW_TIMEOUT) < 0)
        return(true);

    // osrs_t x1, osrs_p x4, normal mode operation
    const uint8_t reg_ctrl_meas_val = (0x01 << 5) | (0x03 << 2) | (0x03);
    buf[0] = REG_CTRL_MEAS;
    buf[1] = reg_ctrl_meas_val;
    if(i2c_write_timeout_us(i2c_default, addr, buf, 2, false, RW_TIMEOUT) < 0)
        return(true);

    // retrieve fixed compensation params
    if(bmp280_get_calib_params(&params) == true)
        return(true); // failed to get calibration parameters.
    else { return(false); } // all good with setup.
}


// Reads values from BMP280, converts these to temperature & pressure.
// Returns HIGH flag if error.
// Pressure given in Pa
// Temperature in (1/100) of 1°C
bool BMP_Values(uint32_t *pressure, int16_t *temperature) {
    int32_t raw_temp, raw_pz; 
    // Get raw data (or flag if error)
    if(bmp280_read_raw(&raw_temp, &raw_pz) == true)
        return(true);

    *temperature = bmp280_convert_temp(raw_temp, &params);
    *pressure = bmp280_convert_pressure(raw_pz, raw_temp, &params);
    return(false);
}


// Reset the device with the power-on-reset procedure
void BMP280_Reset() {
    uint8_t buf[2] = { REG_RESET, 0xB6 };
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

// Reads both temperature and pressure in raw form (requires conversion using registers)
// Returns HIGH if error in read.
bool bmp280_read_raw(int32_t* temp, int32_t* pressure) {
    // BMP280 data registers are auto-incrementing and we have 3 temperature and
    // pressure registers each, so we start at 0xF7 and read 6 bytes to 0xFC
    // note: normal mode does not require further ctrl_meas and config register writes

    uint8_t buf[6];
    uint8_t reg = REG_PRESSURE_MSB;
    if(i2c_write_timeout_us(i2c_bus, addr, &reg, 1, true, RW_TIMEOUT) < 0)  // true to keep master control of bus
        return(true);
    if(i2c_read_timeout_us(i2c_bus, addr, buf, 6, false, RW_TIMEOUT) < 0)  // false - finished with bus
        return(true);

    // store the 20 bit read in a 32 bit signed integer for conversion
    *pressure = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    *temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);

    return(false);
}



int32_t bmp280_convert_temp(int32_t temp, struct bmp280_calib_param* params) {
    // Uses the BMP280 calibration parameters to compensate the temperature value read from its registers.

    int32_t var1, var2;
    // use the 32-bit fixed point compensation implementation given in the datasheet.
    var1 = ((((temp >> 3) - ((int32_t)params->dig_t1 << 1))) * ((int32_t)params->dig_t2)) >> 11;
    var2 = (((((temp >> 4) - ((int32_t)params->dig_t1)) * ((temp >> 4) - ((int32_t)params->dig_t1))) >> 12) * ((int32_t)params->dig_t3)) >> 14;

    int32_t t_fine = var1 + var2;
    return( (t_fine * 5 + 128) >> 8 );
}


int32_t bmp280_convert_pressure(int32_t pressure, int32_t temp, struct bmp280_calib_param* params) {
    // To be ran after the temperature is taken, as this is used to correct pressure reading.

    int32_t var1, var2;
    uint32_t converted = 0.0;
    var1 = (((int32_t)temp) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)params->dig_p6);
    var2 += ((var1 * ((int32_t)params->dig_p5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)params->dig_p4) << 16);
    var1 = (((params->dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)params->dig_p2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)params->dig_p1)) >> 15);
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    converted = (((uint32_t)(((int32_t)1048576) - pressure) - (var2 >> 12))) * 3125;
    if (converted < 0x80000000) {
        converted = (converted << 1) / ((uint32_t)var1);
    } else {
        converted = (converted / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)params->dig_p9) * ((int32_t)(((converted >> 3) * (converted >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(converted >> 2)) * ((int32_t)params->dig_p8)) >> 13;
    converted = (uint32_t)((int32_t)converted + ((var1 + var2 + params->dig_p7) >> 4));
    return converted;
}


bool bmp280_get_calib_params(struct bmp280_calib_param* params) {
    // raw temp and pressure values need to be calibrated according to
    // parameters generated during the manufacturing of the sensor
    // there are 3 temperature params, and 9 pressure params, each with a LSB
    // and MSB register, so we read from 24 registers

    uint8_t buf[NUM_CALIB_PARAMS] = { 0 };
    uint8_t reg = REG_DIG_T1_LSB;
    if(i2c_write_timeout_us(i2c_bus, addr, &reg, 1, true, RW_TIMEOUT) < 0);  // true to keep master control of bus
        return(true);

    // read in one go as register addresses auto-increment
    if(i2c_read_timeout_us(i2c_bus, addr, buf, NUM_CALIB_PARAMS, false, RW_TIMEOUT) < 0);  // false, we're done reading
        return(true);

    // store these in a struct for later use
    params->dig_t1 = (uint16_t)(buf[1] << 8) | buf[0];
    params->dig_t2 = (int16_t)(buf[3] << 8) | buf[2];
    params->dig_t3 = (int16_t)(buf[5] << 8) | buf[4];

    params->dig_p1 = (uint16_t)(buf[7] << 8) | buf[6];
    params->dig_p2 = (int16_t)(buf[9] << 8) | buf[8];
    params->dig_p3 = (int16_t)(buf[11] << 8) | buf[10];
    params->dig_p4 = (int16_t)(buf[13] << 8) | buf[12];
    params->dig_p5 = (int16_t)(buf[15] << 8) | buf[14];
    params->dig_p6 = (int16_t)(buf[17] << 8) | buf[16];
    params->dig_p7 = (int16_t)(buf[19] << 8) | buf[18];
    params->dig_p8 = (int16_t)(buf[21] << 8) | buf[20];
    params->dig_p9 = (int16_t)(buf[23] << 8) | buf[22];

    return(false);
}


