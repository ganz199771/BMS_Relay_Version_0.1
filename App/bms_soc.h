
/** 
 * 头文件默认对应电芯：国轩高科 INP24.8256106A-115.8Ah
 * 
*/


#ifndef _INP24P8256106A_115P8AH_SOC_H_
#define _INP24P8256106A_115P8AH_SOC_H_

#include <stdint.h>


#define SOC_TABLE_ROW_COUNT 20
#define SOC_TABLE_COLUMN_COUNT 6
#define SOC_CALC_PERIOD_MS 20
#define MS_BY_HOUR (0.000000277f)

/// @brief 上电时，对SOC初始值进行估算
/// @param  
void soc_init(void);

#endif



