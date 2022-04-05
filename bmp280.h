#ifndef bmp_included
#define bmp_included


// Setup BMP280, returns HIGH flag if this process fails.
bool BMP_Setup(i2c_inst_t* _i2c_port_, uint8_t _device_address_);


// Reads ADC values from BMP280, converts these to temperature and pressure.
// Returns HIGH flag if error.
// Pressure given in Pa
// Temperature in (1/100) of 1°C
bool BMP_Values(uint32_t *pressure, int16_t *temperature);


#endif