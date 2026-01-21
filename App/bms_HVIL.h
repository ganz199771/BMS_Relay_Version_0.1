

#ifndef BMS_HVIL_H
#define BMS_HVIL_H



#include <stdint.h>


typedef struct pwm_cap_info
{
    uint16_t rising_edge_cnt; /* rising edge count, used for calculate period and duty cycle */
    uint16_t falling_edge_cnt; /* falling edge count, used for calculate period and duty cycle */
    uint16_t period_us; /* period in us */
    uint8_t duty_cycle; /* duty_cycle is 80 means duty cycle is 80% */
}pwm_cap_info_t;



/* Every time that a capture trigger 0 occurs, CCcapt0, the actual value of the 
 * timer is captured into the capture register 1 and the previous value stored in 
 * this register is transferred into capture register 0 */
#define CAPTURE_0_VALUE_RISING_EDGE  1

/* Every time that a capture trigger 1 occurs, CCcapt1, the actual value of the 
 * timer is captured into the capture register 3 and the previous value stored in 
 * this register is transferred into capture register 2 */
#define CAPTURE_0_VALUE_FALLING_EDGE 3


/// @brief 获取HVIL PWM输入捕获信息
/// @return 
pwm_cap_info_t* get_HVIL_pwm_cap();

/// @brief 获取主继电器 PWM输入捕获信息
/// @return 
pwm_cap_info_t* get_main_coil_pwm_cap();

#endif
