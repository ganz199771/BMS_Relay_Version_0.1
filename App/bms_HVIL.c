

#include "bms_HVIL.h"
#include "cycfg_peripherals.h"


static pwm_cap_info_t main_coil_pwm_cap; /* 主继电器状态 */
static pwm_cap_info_t HVIL_pwm_cap; /* HVIL状态 */


pwm_cap_info_t* get_HVIL_pwm_cap()
{
    return &HVIL_pwm_cap;
}

pwm_cap_info_t* get_main_coil_pwm_cap()
{
    return &main_coil_pwm_cap;
}

/// @brief 捕获HVIL环路PWM信号上升沿
/// @param  
void ccu4_0_SR0_INTERRUPT_HANDLER(void)
{
    if(XMC_CCU4_SLICE_GetEvent(HVIL_PWM_Receive_HW, XMC_CCU4_SLICE_IRQ_ID_EVENT0))
    {
        HVIL_pwm_cap.rising_edge_cnt = XMC_CCU4_SLICE_GetCaptureRegisterValue(HVIL_PWM_Receive_HW, CAPTURE_0_VALUE_RISING_EDGE);
    }
}

/// @brief 捕获HVIL环路PWM信号下降沿
/// @param  
void ccu4_0_SR1_INTERRUPT_HANDLER(void)
{
    if(XMC_CCU4_SLICE_GetEvent(HVIL_PWM_Receive_HW, XMC_CCU4_SLICE_IRQ_ID_EVENT1))
    {
        HVIL_pwm_cap.falling_edge_cnt = XMC_CCU4_SLICE_GetCaptureRegisterValue(HVIL_PWM_Receive_HW, CAPTURE_0_VALUE_FALLING_EDGE);
        HVIL_pwm_cap.period_us = (uint16_t)(HVIL_pwm_cap.falling_edge_cnt * 10.667f);
        HVIL_pwm_cap.duty_cycle = 1.0f * (HVIL_pwm_cap.falling_edge_cnt - HVIL_pwm_cap.rising_edge_cnt) / HVIL_pwm_cap.falling_edge_cnt * 100;
    }

    if(HVIL_pwm_cap.duty_cycle > 51 || HVIL_pwm_cap.duty_cycle < 49)
    {
        HVIL_pwm_cap.duty_cycle = 0;
    }
}

/// @brief 捕获主接触器辅助线圈环路PWM信号上升沿
/// @param  
void ccu8_0_SR0_INTERRUPT_HANDLER(void)
{
    if(XMC_CCU8_SLICE_GetEvent(MainCoil_PWM_Receive_HW, XMC_CCU8_SLICE_IRQ_ID_EVENT0))
    {
        main_coil_pwm_cap.rising_edge_cnt = XMC_CCU8_SLICE_GetCaptureRegisterValue(MainCoil_PWM_Receive_HW, CAPTURE_0_VALUE_RISING_EDGE);
    }
}

/// @brief 捕获主接触器辅助线圈环路PWM信号下降沿
/// @param  
void ccu8_0_SR1_INTERRUPT_HANDLER(void)
{
    if(XMC_CCU8_SLICE_GetEvent(MainCoil_PWM_Receive_HW, XMC_CCU8_SLICE_IRQ_ID_EVENT1))
    {
        main_coil_pwm_cap.falling_edge_cnt = XMC_CCU8_SLICE_GetCaptureRegisterValue(MainCoil_PWM_Receive_HW, CAPTURE_0_VALUE_FALLING_EDGE);
        main_coil_pwm_cap.period_us = (uint16_t)(main_coil_pwm_cap.falling_edge_cnt * 10.667f);
        main_coil_pwm_cap.duty_cycle = 1.0f * (main_coil_pwm_cap.falling_edge_cnt - main_coil_pwm_cap.rising_edge_cnt) / main_coil_pwm_cap.falling_edge_cnt * 100;
    }

    if(main_coil_pwm_cap.duty_cycle > 51 || main_coil_pwm_cap.duty_cycle < 49)
    {
        main_coil_pwm_cap.duty_cycle = 0;
    }
}




