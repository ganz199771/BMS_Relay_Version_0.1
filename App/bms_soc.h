
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

#define SOC_PRECISION (0.01f) /* 单位0.01，例如5208表示SOC=52.08% */
#define SOC_MIN 500 /* 对应SOC = 5% */
#define SOC_MAX 10000 /* 对应SOC = 100% */
#define CAP_PRECISION (0.1f) /* 容量精度，单位0.1Ah */
#define HUANDRED_PERCENT 100 /* %放大倍数 */

/// @brief 上电时，对SOC初始值进行估算
/// @param  
void soc_init(void);

#endif



