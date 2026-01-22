
#include "bms_soc.h"
#include "cybsp.h"
#include "bms.h"
#include "bms_config.h"
#include "bms_rs485_uart.h"

static float bms_f_soc = 0.f;

/* 三元锂SOC-OCV表格 */
static const uint16_t LiPo_SOC_OCV_table[20][6] = {
//   -25℃  -10℃  0℃   10℃  25℃  40℃
    {3435, 3433, 3431, 3429, 3424, 3424}, // 5%
    {3482, 3480, 3479, 3478, 3475, 3474}, // 10%
    {3521, 3519, 3518, 3518, 3515, 3515}, // 15%
    {3557, 3556, 3556, 3556, 3554, 3555}, // 20%
    {3586, 3586, 3587, 3587, 3586, 3588}, // 25%
    {3611, 3612, 3613, 3614, 3613, 3616}, // 30%
    {3636, 3638, 3639, 3640, 3640, 3643}, // 35%
    {3666, 3668, 3669, 3671, 3672, 3675}, // 40%
    {3708, 3710, 3712, 3713, 3714, 3717}, // 45%
    {3760, 3762, 3764, 3765, 3766, 3769}, // 50%
    {3813, 3815, 3816, 3817, 3818, 3820}, // 55%
    {3860, 3862, 3863, 3864, 3865, 3867}, // 60%
    {3905, 3906, 3908, 3909, 3909, 3911}, // 65%
    {3951, 3953, 3953, 3954, 3955, 3957}, // 70%
    {4002, 4003, 4003, 4004, 4004, 4006}, // 75%
    {4052, 4053, 4054, 4054, 4054, 4056}, // 80%
    {4088, 4089, 4090, 4090, 4090, 4092}, // 85%
    {4111, 4112, 4113, 4114, 4115, 4116}, // 90%
    {4137, 4138, 4139, 4140, 4141, 4143}, // 95%
    {4200, 4201, 4203, 4204, 4213, 4207}  // 100% 
};

/* 磷酸铁锂SOC-OSC表格 */
static const uint16_t LiFeO4_SOC_OCV_table[20][6] = {
//  -10℃  0℃   10℃   25℃  35℃  45℃
    {3210, 3200, 3199, 3163, 3147, 3147}, // 5%
    {3225, 3210, 3206, 3204, 3204, 3204}, // 10%
    {3243, 3233, 3232, 3223, 3219, 3219}, // 15%
    {3256, 3252, 3250, 3245, 3241, 3241}, // 20%
    {3265, 3266, 3265, 3259, 3257, 3256}, // 25%
    {3270, 3274, 3278, 3277, 3272, 3272}, // 30%
    {3272, 3278, 3283, 3288, 3290, 3292}, // 35%
    {3274, 3279, 3284, 3289, 3292, 3295}, // 40%
    {3275, 3280, 3284, 3290, 3293, 3295}, // 45%
    {3276, 3280, 3285, 3290, 3294, 3296}, // 50%
    {3278, 3282, 3286, 3291, 3295, 3297}, // 55%
    {3282, 3286, 3290, 3294, 3296, 3299}, // 60%
    {3288, 3296, 3302, 3315, 3308, 3309}, // 65%
    {3296, 3311, 3320, 3328, 3330, 3330}, // 70%
    {3304, 3318, 3324, 3328, 3330, 3331}, // 75%
    {3309, 3319, 3324, 3329, 3330, 3331}, // 80%
    {3311, 3320, 3324, 3329, 3331, 3332}, // 85%
    {3310, 3319, 3324, 3329, 3331, 3332}, // 90%
    {3311, 3319, 3324, 3330, 3332, 3333}, // 95%
    {3418, 3412, 3435, 3445, 3404, 3438}  // 100%
};

/// @brief 根据三元锂 SOC-OCV表格、电芯电压、温度，得到SOC值
/// @param table SOC-OCV表格
/// @param cell_voltage 电芯电压，单位mV
/// @param temp_k 温度，单位0.01K
/// @return SOC值
static float LiPo_soc_from_table(const uint16_t(*table)[6] , uint16_t cell_voltage, uint16_t temp_k)
{
    int8_t temp_cel = (int8_t)(temp_k * 0.01f - 273.15f);
    uint8_t col_index = 0;
    if(temp_cel <= -17.5f)
        col_index = 0;
    else if(temp_cel > -17.5f && temp_cel <= -5)
        col_index = 1;
    else if(temp_cel > -5 && temp_cel <= 5)
        col_index = 2;
    else if(temp_cel > 5 && temp_cel <= 17.5f)
        col_index = 3;
    else if(temp_cel > 17.5f && temp_cel <= 32.5f)
        col_index = 4;
    else if(temp_cel > 32.5f)
        col_index = 5;

    if(cell_voltage <= table[0][col_index])
        return SOC_MIN; /* 5% */
    if(cell_voltage >= table[19][col_index])
        return SOC_MAX; /* 100% */

    for (uint8_t i = 0; i < 19; i++)
    {
        if(table[i][col_index] <= cell_voltage && table[i + 1][col_index] >= cell_voltage)
        {
            if(table[i + 1][col_index] == table[i][col_index])
                return (i + 1) * 5 / SOC_PRECISION;

            float k = 1.0f * (table[i + 1][col_index] - cell_voltage) / (table[i + 1][col_index] - table[i][col_index]); /* 系数 */
            return (i + 1 + k) * 5 / SOC_PRECISION;
        }
    }
    return SOC_MAX;
}

/// @brief 根据磷酸铁锂 SOC-OCV表格、电芯电压、温度，得到SOC值
/// @param table SOC-OCV表格
/// @param cell_voltage 电芯电压，单位mV
/// @param temp_k 温度，单位0.01K
/// @return SOC值
static float LiFeO4_soc_from_table(const uint16_t(*table)[6] , uint16_t cell_voltage, uint16_t temp_k)
{
    int8_t temp_cel = (int8_t)(temp_k * 0.01f - 273.15f);
    uint8_t col_index = 0;
    if(temp_cel <= -5)
        col_index = 0;
    else if(temp_cel > -5 && temp_cel <= 5)
        col_index = 1;
    else if(temp_cel > 5 && temp_cel <= 17.5f)
        col_index = 2;
    else if(temp_cel > 17.5f && temp_cel <= 30)
        col_index = 3;
    else if(temp_cel > 30 && temp_cel <= 40)
        col_index = 4;
    else if(temp_cel > 40)
        col_index = 5;

    if(cell_voltage <= table[0][col_index])
        return SOC_MIN; /* 5% */
    if(cell_voltage >= table[19][col_index])
        return SOC_MAX; /* 100% */

    for (uint8_t i = 0; i < 19; i++)
    {
        if(table[i][col_index] <= cell_voltage && table[i + 1][col_index] >= cell_voltage)
        {
            float k = 1.0f * (table[i + 1][col_index] - cell_voltage) / (table[i + 1][col_index] - table[i][col_index]); /* 系数 */
            return (i + 1 + k) * 5 / SOC_PRECISION;
        }
    }
    return SOC_MAX;
}


/// @brief 在放电时，需要以单体电芯最低的SOC作为BMS系统的SOC
/// @return BMS系统SOC
static float init_SOC_under_OCV()
{
    slave_node_t* slave_node_head_ptr = get_slave_list_head();
    slave_node_t* ptr = NULL;
    slave_status_t* slave_st_ptr = NULL;
    slave_config_t* slave_cfg_ptr = get_slave_cfg();
    bms_status_t* bms_st_ptr = read_bms_status();

    list_for_each_entry(ptr, &(slave_node_head_ptr->entry), entry)
    {
        slave_st_ptr = read_slave_status(ptr->slave_id);
        uint16_t ntc_temp = 0;
        for (uint8_t i = 0; i < slave_cfg_ptr->ntc_count; i++)
        {
            ntc_temp += (0.01f * slave_st_ptr->cmu_board_ntc_temp_result[i]); /* cmu_board_ntc_temp_result单位100K，转到K */
        }
        ntc_temp /= slave_cfg_ptr->ntc_count; /* 平均值为电芯温度 */
        ntc_temp -= ZERO_CELDIUS_KELVIN;

        /* 找到最低电压的电芯，以此计算SOC */
        uint16_t cell_volt = 4500; /* 4.5V */
        for (uint8_t i = 0; i < slave_cfg_ptr->cell_serial_count; i++)
        {
            if(cell_volt > slave_st_ptr->cmu_board_cell_voltage[i] && slave_st_ptr->cmu_board_cell_voltage[i] > 2200)
                cell_volt = slave_st_ptr->cmu_board_cell_voltage[i];
        }

        /* 根据放电的SOC-OCV表格，确定单个电池包初始SOC值 */
        if(slave_cfg_ptr->cell_type == Ternary_lithium)
            bms_f_soc = LiPo_soc_from_table(LiPo_SOC_OCV_table, cell_volt, ntc_temp); 
        else if(slave_cfg_ptr->cell_type == Lithium_Iron_Phosphate)
            bms_f_soc = LiFeO4_soc_from_table(LiFeO4_SOC_OCV_table, cell_volt, ntc_temp);

        if(bms_st_ptr->SOC > bms_f_soc)
            bms_st_ptr->SOC = bms_f_soc; /* 总的SOC采用单个电池SOC最低值 */
    }

    return bms_f_soc;
}

void soc_init(void)
{   
    init_SOC_under_OCV(); /* 上电时刻，认为是准备让电池放电，更新BMS的SOC */

    NVIC_SetPriority(CCU40_2_IRQn, 4U);
    NVIC_EnableIRQ(CCU40_2_IRQn);
}


/// @brief 定时器中断，每20ms触发一次
/// @param  
void CCU40_2_IRQHandler(void)
{
    bms_status_t* bms_st_ptr = read_bms_status();
    bms_config_t* bms_cfg_ptr = get_bms_config();
    float bsp_cap = bms_cfg_ptr->bat_total_cap * CAP_PRECISION * bms_f_soc * SOC_PRECISION / HUANDRED_PERCENT; /* 上一次计算得到的容量，单位Ah */
    float delta_cap = bms_st_ptr->power_current_A * SOC_CALC_PERIOD_MS * MS_BY_HOUR; /* 积分步长的容量变化 */
    bsp_cap -= delta_cap; /* 积分更新容量 */
    bms_f_soc = 1.0f * bsp_cap / (bms_cfg_ptr->bat_total_cap * CAP_PRECISION) * HUANDRED_PERCENT / SOC_PRECISION; /* 根据容量，更新SOC值 */

    if(bms_f_soc < SOC_MIN)
    {
        bms_f_soc = SOC_MIN;
        bms_charge_discharge(stop); /* 停止大电流放电、停止大电流充电 */
        bms_precharge(start); /* 开始小电流充电 */
        NVIC_DisableIRQ(CCU40_2_IRQn);
    }
    if(bms_f_soc > SOC_MAX)
    {
        bms_f_soc = SOC_MAX;
        bms_charge_discharge(stop); /* 停止放电 */
        NVIC_DisableIRQ(CCU40_2_IRQn);
    }  

    bms_st_ptr->SOC = bms_f_soc;

    if(bms_st_ptr->power_current_A < 0) /* 充电 */
        bms_cfg_ptr->charge_cap += (-1 * delta_cap * 10); /* 因为 charge_cap 单位是0.1Ah */
    else /* 放电 */
        bms_cfg_ptr->discharge_cap += (delta_cap * 10); /* 因为 discharge_cap 单位是0.1Ah */

    if(bms_cfg_ptr->bat_total_cap != 0)
        bms_cfg_ptr->cycle_times = (uint16_t)(1.0f * bms_cfg_ptr->discharge_cap / bms_cfg_ptr->bat_total_cap); /* 更新循环次数 */
}

