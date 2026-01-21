

#ifndef BMS_CODE_EVAL_H
#define BMS_CODE_EVAL_H

#include <stdint.h>

/// @brief 在main函数开头调用此函数
void fill_stack_with_pattern();

/// @brief BMS代码运行时所占用RAM空间大小，单位%
/// @return 0~100
uint8_t bms_ram_usage();

#endif



