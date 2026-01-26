
#include "bms_ntc.h"
#include <math.h>

/// @brief 根据电阻计算NTC温度，返回实际温度（K）* 100（放大系数）
/// @param res 电阻值，单位K
/// @return 温度K，放大系数100
uint16_t NTC_res_calc_temp(float res)
{
    float Temp = 1 / (1.0f / B * log(res / R_25) + 1 / T2);
    return Temp * 100;
}

