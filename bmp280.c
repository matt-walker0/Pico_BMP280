/*  BMP280 library for Pi Pico
    Based upon https://github.com/daveake/pico-tracker/blob/main/bme280.c
    Datasheet used here: https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
    Modified by Matt Walker 10th May 2021
*/

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "math.h"

// 101325 Pa = standard pressure at msl
#define seaLevelPa 101325

// Local functions
int32_t compensate_temp(int32_t adc_T);
uint32_t compensate_pressure(int32_t adc_P);
bool read_registers(uint8_t reg, uint8_t *buffer, uint16_t len);
bool read_compensation_parameters();


// Global variables
uint8_t device_address; // BMP280 typically on the 0x76
i2c_inst_t* i2c_port; // i2c

// Calibration variables. I'll leave these be.
int32_t t_fine;
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t dig_H1, dig_H3;
int8_t dig_H6;
int16_t dig_H2, dig_H4, dig_H5;


// Setup BMP280, returns HIGH flag if this process fails.
// Expects I2C bus to be pre-setup as can have multiple devices on bus.
bool BMP_Setup(i2c_inst_t* _i2c_port_, uint8_t _device_address_) {
    printf("BMP280 -> ");
	
    uint8_t id;
    if(read_registers(0xD0, &id, 1) == false) { return(false);} // Get I2C ID byte from register
    if(id == 0x58)
    {
        if(read_compensation_parameters() != true) {  // no errors getting compensation numbers
            uint8_t buffer[2] = {0xF4, 0x57};
            if(i2c_write_timeout_us(_i2c_port_, _device_address_, buffer, sizeof(buffer), false, 10000) < 0) { return(false); }
            printf("OK\n");
            return(false);
        }
    }
    printf("ERR\n");
    return(true);
}


// Reads ADC values from BMP280, converts these to temperature and pressure.
// Returns HIGH flag if error.
// Pressure given in Pa
// Temperature in (1/100) of 1°C
bool BMP_Values(uint32_t *pressure, int16_t *temperature) {
    uint8_t buffer[6];
    int32_t adc_P, adc_T;

    if(read_registers(0xF7, buffer, 6) == true) { return true; }
    adc_T = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | (buffer[2] >> 4);
    adc_P = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4 | buffer[5] >> 4);

    *pressure = compensate_pressure(adc_P);
    *temperature = compensate_temp(adc_T);
    return(false);
}


/* The following compensation functions are required to convert from the raw ADC
data from the chip to something usable. Each chip has a different set of
compensation parameters stored on the chip at point of manufacture, which are
read from the chip at startup and used inthese routines.
*/
int32_t compensate_temp(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T =(t_fine*5+128)>>8;
    return T;
}

uint32_t compensate_pressure(int32_t adc_P) {
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
    var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
    var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18; var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
    if (var1 == 0)
        return 0;

    p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
    if (p < 0x80000000)
        p = (p << 1) / ((uint32_t)var1);
    else
        p = (p / (uint32_t)var1) * 2;

    var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12; var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));

    return p;
}


// Helper function to write single byte to register and read reply of len.
// Returns HIGH flag if error.
bool read_registers(uint8_t reg, uint8_t *buffer, uint16_t len) {
    if(i2c_write_timeout_us(i2c_port, device_address, &reg, 1, true, 10000) < 0) {return true; } // true to keep master control of bus
    if(i2c_read_timeout_us(i2c_port, device_address, buffer, len, false, 10000) < 0) {return true; }
    return(false);
}

// This function reads the manufacturing assigned compensation parameters from the device
// Returns HIGH if error.
bool read_compensation_parameters()
{
    uint8_t buffer[26];
    uint8_t address;

    address = 0x88;
    if(read_registers(address,buffer,24) == true) { return(true); }

    dig_T1 = buffer[0] | (buffer[1] << 8);
    dig_T2 = buffer[2] | (buffer[3] << 8);
    dig_T3 = buffer[4] | (buffer[5] << 8);

    dig_P1 = buffer[6] | (buffer[7] << 8);
    dig_P2 = buffer[8] | (buffer[9] << 8);
    dig_P3 = buffer[10] | (buffer[11] << 8);
    dig_P4 = buffer[12] | (buffer[13] << 8);
    dig_P5 = buffer[14] | (buffer[15] << 8);
    dig_P6 = buffer[16] | (buffer[17] << 8);
    dig_P7 = buffer[18] | (buffer[19] << 8);
    dig_P8 = buffer[20] | (buffer[21] << 8);
    dig_P9 = buffer[22] | (buffer[23] << 8);

    dig_H1 = buffer[25];

    address = 0xE1;
    if(read_registers(address,buffer,8) == true) { return(true); }

    dig_H2 = buffer[0] | (buffer[1] << 8);
    dig_H3 = (int8_t)buffer[2];
    dig_H4 = buffer[3] << 4 | (buffer[4] & 0xf);
    dig_H5 = (buffer[5] >> 4) | (buffer[6] << 4);
    dig_H6 = (int8_t)buffer[7];

    return(false);
}


