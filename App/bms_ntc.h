

#ifndef _BMS_NTC_H_
#define _BMS_NTC_H_

#define QUERY_TABLE_LEN 156
#define TEMP_SCALE 100.f
#define R_VALUE 2.4f // NTC之外的电阻值 2.4k   硬件版本V0.3
#define NO_NTC_VALUE 0xffff // 当没有接NTC时，传输此数据
#include <stdint.h>

uint16_t NTC_res_calc_temp(float res);


#endif


