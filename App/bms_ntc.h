

#ifndef _BMS_NTC_H_
#define _BMS_NTC_H_

#define NTC_SERIAL_RESISTOR 2.4f /* NTC串联电阻 2.4k */
#define NTC_VCC 3.3f /* NTC上拉电源电压 */
#define TEMP_SCALE 100
#define ZERO_CEL_KELVIN 273.15f

#define B 3950
#define T2 (273.15f + 25)
#define R_25 10 /* 25℃时电阻 10K */

#include <stdint.h>

uint16_t NTC_res_calc_temp(float res);


#endif


