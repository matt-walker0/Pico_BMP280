#ifndef bmp_included
#define bmp_included


bool BMP_Setup(i2c_inst_t* _i2c_port_, uint8_t _device_address_);

bool BMP_Values(uint32_t *pressure, int16_t *temperature);

void BMP280_Reset();

#endif
