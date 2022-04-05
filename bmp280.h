#ifndef bmp_included
#define bmp_included

bool bmp_setup(void);

bool BMPReadValues();
uint32_t BMPPressure();
int32_t BMPTemperature();
//void bmp_value(struct AFCS *aircraft);

#endif