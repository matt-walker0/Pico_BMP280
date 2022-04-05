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
#include "../../FLYC.h"
#include "../../pins.h"

// Local functions
int32_t compensate_temp(int32_t adc_T);
uint32_t compensate_pressure(int32_t adc_P);
bool read_registers(uint8_t reg, uint8_t *buffer, uint16_t len);
bool read_compensation_parameters();
bool bme280_read_raw(int32_t *pressure, int32_t *temperature);

// BMP280 on the 0x76
const int DeviceAddress = 0x76; 

// Variables
int32_t temperature;
uint32_t pressure;

// 101325 Pa = standard pressure at msl
#define seaLevelPa 101325

// Calibration variables. I'll leave these be.
int32_t t_fine;
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t dig_H1, dig_H3;
int8_t dig_H6;
int16_t dig_H2, dig_H4, dig_H5;


bool bmp_setup() {
    printf("BMP280 -> ");
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_SENSORS_PORT, 400*1000);
    gpio_set_function(I2C_SENSORS_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SENSORS_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SENSORS_SDA);
    gpio_pull_up(I2C_SENSORS_SCL);
	
    uint8_t id;
    if(read_registers(0xD0, &id, 1) == false) { return(false);} // Get I2C ID byte from register
    if(id == 0x58)
    {
        read_compensation_parameters();

        uint8_t buffer[2] = {0xF4, 0x57};
        if(i2c_write_timeout_us(I2C_SENSORS_PORT, DeviceAddress, buffer, sizeof(buffer), false, 10000) < 0) { return(false); }
        printf("OK\n");
        return(true);
    }
    else
    {
        printf("ERR\n");
        return(false);
    }
}

// Returns true if read successfully
bool BMPReadValues() {
    return(bme280_read_raw(&pressure, &temperature));  
}

uint32_t BMPPressure() {
    return(compensate_pressure(pressure));
}

int32_t BMPTemperature() {
    return(compensate_temp(temperature));
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


// helper function to write single byte to register and read reply of len
bool read_registers(uint8_t reg, uint8_t *buffer, uint16_t len)
{
    if(i2c_write_timeout_us(I2C_SENSORS_PORT, DeviceAddress, &reg, 1, true, 10000) < 0) {return false; } // true to keep master control of bus
    if(i2c_read_timeout_us(I2C_SENSORS_PORT, DeviceAddress, buffer, len, false, 10000) < 0) {return false; }
    return(true);
}

/* This function reads the manufacturing assigned compensation parameters from the device */
// returns true if read correctly.
bool read_compensation_parameters()
{
    uint8_t buffer[26];
    uint8_t address;

    address = 0x88;
    if(read_registers(address,buffer,24) == false) { return(false); }

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
    if(read_registers(address,buffer,8) == false) { return(false); }

    dig_H2 = buffer[0] | (buffer[1] << 8);
    dig_H3 = (int8_t)buffer[2];
    dig_H4 = buffer[3] << 4 | (buffer[4] & 0xf);
    dig_H5 = (buffer[5] >> 4) | (buffer[6] << 4);
    dig_H6 = (int8_t)buffer[7];

    return(true);
}

bool bme280_read_raw(int32_t *pressure, int32_t *temperature) {
    uint8_t buffer[6];

    if(read_registers(0xF7, buffer, 6) == false) { return false; }
    *pressure = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | (buffer[2] >> 4);
    *temperature = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4 | buffer[5] >> 4);
    return(true);
}

