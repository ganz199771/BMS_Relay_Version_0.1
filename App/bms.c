
#include "bms.h"
#include "config.h"
#include "mempool.h"
#include "cycfg_peripherals.h"
#include "cycfg_pins.h"
#include "bms_rs485_uart.h"
#include "bms_ntc.h"
#include "bms_wifi.h"
#include "bms_config.h"
#include "bms_soc.h"
#include "bms_HVIL.h"
#include "bms_code_eval.h"

#include <stdint.h>
#include <stdlib.h>

#include <xmc_gpio.h>

static bms_status_t _bms_st; // BMS状态
static slave_config_t slave_cfg; /* 从机配置 */
static uint32_t bms_know_slave_flag = BMS_KNOW_NOTHING;

static bms_preapare_state_t bms_prepare_st = wake; /* 准备状态 */

static volatile uint16_t prepare_ticks = 0; // 准备定时器运行时间，单位ms
static volatile uint16_t bms_background_ticks = 0; // BMS后台定时器运行时间，单位ms
static volatile uint16_t bms_polling_slave_ticks = 0; // BMS轮询从机定时器运行时间，单位ms
static volatile uint16_t ram_usage_ticks = 0; // 评估RAM使用的定时器运行时间，单位ms
static volatile uint32_t bms_run_ticks = 0; /* 统计bms运行时间，单位ms */
static uint32_t bms_idle_ticks = 0; /* 统计bms主机调用xmc_delay的时间，单位ms */

/// @brief 延时函数，主要目的是统计空闲时间
/// @param ms 
static inline void bms_delay_ms(uint32_t ms)
{
    XMC_Delay(ms);
    bms_idle_ticks += ms;
}


void bms_init(void)
{
    bms_rs485_uart_tx_init();

    /* 重置BMS状态 */
    _bms_st.error_state = 0;
    _bms_st.iso_RN = _bms_st.iso_RP = _bms_st.NTC1_temp = _bms_st.NTC2_temp = _bms_st.power_current_A = _bms_st.voltage = 0;
    _bms_st.cpu_usage = _bms_st.ram_usage = 0;
    _bms_st.timestamp = 0; /* 运行时长 */
    _bms_st.state = Idle;
    _bms_st.SOC = 100;

    bms_config_init(); /* 上电后初始化BMS配置 */
    bms_rs485_uart_init(); /* 初始化RS485(从机)、CAN(从机)、UART(上位机) */
    bms_wifi_init(); /* 初始化Wifi(上位机)、蓝牙(上位机) */
}

/// @brief 检查电芯电压是否满足2个条件，如果满足则开启均衡，具体做法就是向从机发送命令
/// 1：电芯电压 > 均衡开启电压
/// 2：电芯电压压差 > 均衡精度
/// @param p_slave_st 
void bms_prepare_balance(uint8_t slave_id, slave_status_t* p_slave_st)
{
    if(!p_slave_st)
        return;

    bms_config_t* p_bms_cfg = get_bms_config();

    uint16_t v_cell1;
    uint16_t v_cell2;
    uint32_t open_balance_channel_bit_OR_value = 0;
    uint32_t shift = 0x02; /* 从第2个电芯开始，当第2个电芯电压 > 第一个电芯电压，打开电芯2均衡开关，向电芯1放电（均衡） */

    uint8_t tx_buf[3] = {0}; 

    for (uint8_t index = 0; index < slave_cfg.cell_serial_count - 1; index++)
    {
        v_cell1 = p_slave_st->cmu_board_cell_voltage[index];
        v_cell2 = p_slave_st->cmu_board_cell_voltage[index + 1];
        if(v_cell1 > p_bms_cfg->balance_start_volt && v_cell2 > p_bms_cfg->balance_start_volt)
        {
            if(v_cell2 > v_cell1 && (v_cell2 - v_cell1) > p_bms_cfg->balance_precision)
                open_balance_channel_bit_OR_value |= shift; /* 添加标志位 */
            else
                open_balance_channel_bit_OR_value &= (~shift); /* 去除标志位 */
        }
        shift <<= 1;
    }

    tx_buf[0] = open_balance_channel_bit_OR_value & 0xff; /* 电芯1~8的均衡通道 */
    open_balance_channel_bit_OR_value >>= 8;
    tx_buf[1] = open_balance_channel_bit_OR_value & 0xff; /* 电芯9~16的均衡通道 */
    open_balance_channel_bit_OR_value >>= 8;
    tx_buf[2] = open_balance_channel_bit_OR_value & 0xff; /* 电芯17~24的均衡通道 */

    uint8_t* tx_frame = bms_prepare_rs485_tx_frame(slave_id, Control_Balance, 3, tx_buf);
    if(tx_frame)
        rs485_transmit(tx_frame, BMS_FRAME_NO_DATA_LEN + 3);
}


/// @brief BMS获取PACK的配置信息，这里假设系统中所有的PACK配置是一样的
static void bms_call_slave_config()
{    
    if(bms_know_slave_flag == BMS_KNOW_ALL) /* 如果BMS主机知道了从机配置信息，则不重复问询 */
        return;
    
    uint8_t* tx_frame = NULL;
    slave_node_t* ptr;
    slave_node_t* slave_head_ptr = get_slave_list_head();

    list_for_each_entry(ptr, &(slave_head_ptr->entry), entry)
    {
        if(ptr->slave_id >= SLAVE_ID_MIN && ptr->slave_id <= SLAVE_ID_MAX)
        {
            tx_frame = bms_prepare_rs485_tx_frame(ptr->slave_id, Read_Pack_Config, 0, NULL);
            if(tx_frame)
                rs485_transmit(tx_frame, BMS_FRAME_NO_DATA_LEN);
            bms_delay_ms(10);
            break;
        }
    }
}

/* BMS主机在RS485总线上回复从机，在UART上回复上位机 */
static void rs485_uart_response(void)
{
    bms_rx_node_t* rx_node = get_wire_oldest_rx_node();
	if(!rx_node || (rx_node->buffer_ptr == NULL))
		return;

    uint8_t transmitter_id = rx_node->buffer_ptr[2]; // 发送方ID
    if(transmitter_id != HOST_ID)
        bms_response_slave(rx_node);
    else
        bms_response_host(rx_node);
}

/// @brief BMS对上位机发来的命令进行响应，对从机发来的数据帧进行处理并记录/更新
void bms_response()
{
    /* WiFi回复上位机命令帧 */
    ESP12F_response();

    /* 回复上位机命令帧，处理从机数据帧 */
    rs485_uart_response();
}

/// @brief BMS主板开始运行任务循环
void bms_run()
{
    if(bms_idle_ticks == 0)
        bms_run_ticks = 0; /* 第一次调用bms_run，清零 */

    bms_response(); /* BMS对于接收到的帧进行处理，如果是命令帧则进行回复，如果是数据帧则保存数据 */
        
    if(bms_background_ticks >= 20) /* 每20ms进行一次后台工作 */
    {
        bms_background_ticks = 0;
        bms_background_work(); /* BMS进行后台工作，例如测量电压、电流、温度 */
    }

    if(bms_polling_slave_ticks >= 200) /* 每200ms轮询从机 */
    {
        bms_polling_slave_ticks = 0;
        bms_poll_slave_status(); /* BMS轮询当前所有从机状态 */
    }

    if(prepare_ticks >= 20)
    {
        prepare_ticks = 0;
        bms_prepare(); /* 做一些准备工作，如果需要的话 */
    }
}

/// @brief BMS对特定从机发出Active命令，当特定从机响应时，说明此从机在线
/// @param slave_id 
static void bms_call_slave(uint8_t slave_id)
{
    uint8_t* tx_frame = bms_prepare_rs485_tx_frame(slave_id, Active, 0, NULL);
    if(tx_frame)
        rs485_transmit(tx_frame, BMS_FRAME_NO_DATA_LEN);
    bms_delay_ms(10);
}

/// @brief 检查BMS是否获取当前所有从机初始状态
/// @return 
static int check_if_all_slave_initial_state_fetched()
{
    /* 当没有在线从机时，返回-1 */
    if(get_slave_node_count() == 0)
        return -1;

    slave_node_t* ptr = NULL;
    slave_node_t* slave_head_ptr = get_slave_list_head();
    list_for_each_entry(ptr, &(slave_head_ptr->entry), entry)
    {
        if((ptr->slave_st.chip_temp == BMS_SLAVE_STATE_NOT_FETCH_FLAG))
            return -2; /* 当BMS还没有获取所有的从机的状态数据，返回-2 */
    }

    return 0;
}


void bms_prepare(void)
{
    static uint8_t call_slave_id = SLAVE_ID_MIN;

    switch (bms_prepare_st)
    {
    case wake: /* 当处于初始状态时，获得当前所有连接从机ID */
        if(call_slave_id == SLAVE_ID_MAX)
        {
            if(get_slave_node_count() != 0)
                bms_prepare_st = know_all_slave_id;
            else
            {
                bms_call_slave(SLAVE_ID_MAX);
                call_slave_id = SLAVE_ID_MIN;
            }  
        }
        else
        {
            bms_call_slave(call_slave_id++); /* 查询有哪些从机 */
        }
        break;
    
    case know_all_slave_id: /* 获得了所有的从机ID，下一步就是获得从机配置 */
        if(bms_know_slave_flag == BMS_KNOW_ALL)
            bms_prepare_st = know_slave_config; /* 获得了从机配置 */
        else
            bms_call_slave_config(); /* 轮询每个PACK的配置信息，如果知道了全部的配置信息，则停止询问 */
        break;

    case know_slave_config: /* 获得了从机配置，下一步就是获得从机初始状态 */
        if(check_if_all_slave_initial_state_fetched() == 0)
            bms_prepare_st = know_slave_inital_state;
        else
            bms_poll_slave_status(); /* BMS轮询当前所有从机状态，准备计算SOC初始值 */
        break;

    case know_slave_inital_state: /* 获得了所有从机初始状态，下一步就是进行SOC初始化 */
        soc_init(); /* SOC初始化，当BMS上电时一般情况下假设电池既不在充电，也不在放电 */
        bms_prepare_st = init_soc_done;
        break;

    case init_soc_done:
        bms_prepare_st = prepared; /* 准备完毕 */
        break;

    default:
        break;
    }
}



void heat_slave_cell(uint8_t slave_id, uint8_t heat_until_temp)
{
    /* BMS主机向从机发出命令，开始自加热 */
    uint8_t* tx_frame = bms_prepare_rs485_tx_frame(slave_id, Heat_Pack, 1, &heat_until_temp);
    if(tx_frame)
        rs485_transmit(tx_frame, BMS_FRAME_NO_DATA_LEN + 1);
}

/// @brief 在RS485总线上轮询从机，如果收到某个从机回复，则确认此从机在线
void bms_poll_active_slave()
{
    /* BMS主机轮询从机，ID范围是0x0A~0x15 */
    for (uint8_t id = SLAVE_ID_MIN; id <= SLAVE_ID_MAX; id++)
    {
        /* 每个ID轮询4次 */
        for (uint8_t times = 0; times < 1; times++)
        {
            uint8_t* tx_frame = bms_prepare_rs485_tx_frame(id, Active, 0, NULL);
            if(tx_frame)
            {
                rs485_transmit(tx_frame, BMS_FRAME_NO_DATA_LEN);
            }
            XMC_Delay(10); // 延时10ms
        }
    }
    XMC_Delay(50); /* 延时50ms等待BMS主机接收到从机回复，主机因此识别到所有从机 */
}

/// @brief 在RS485总线上轮询在线从机的状态
void bms_poll_slave_status()
{
    /* 主要是轮询所有在线从机的状态：电芯电压、NTC温度、电池电压、过压/欠压状态 */
    slave_node_t* ptr;
    slave_node_t* slave_head_ptr = get_slave_list_head();
    uint8_t* tx_frame;
    XMC_GPIO_SetOutputLow(LED_Ctrl_Pin_PORT, LED_Ctrl_Pin_PIN);

    list_for_each_entry(ptr, &(slave_head_ptr->entry), entry)
    {
        /* BMS主机轮询从机状态，包括电芯电压、NTC温度、均衡状态 */
        tx_frame = bms_prepare_rs485_tx_frame(ptr->slave_id, Read_Pack_State, 0, NULL);
        if(tx_frame)
        {
            rs485_transmit(tx_frame, BMS_FRAME_NO_DATA_LEN);
            bms_delay_ms(15);
        }
        rs485_uart_response();
    }
    XMC_GPIO_SetOutputHigh(LED_Ctrl_Pin_PORT, LED_Ctrl_Pin_PIN);
}

/// @brief 计算BMS绝缘电阻
/// @param iso_rp 
/// @param iso_rn 
/// @return 
static int iso_resistor_detection(float* iso_rp, float* iso_rn)
{
    if(iso_rp == 0 || iso_rn == 0)
    {
        return -1;
    }

    /* step 1: open Main switch 1, close Main switch 2 */
    XMC_GPIO_SetOutputLow(ISO_Switch2_Pin_PORT, ISO_Switch2_Pin_PIN);
    bms_delay_ms(5);
    XMC_GPIO_SetOutputHigh(ISO_Switch1_Pin_PORT, ISO_Switch1_Pin_PIN);

    /* step 2: wait 10ms then read isolate input voltage for AMC3330DWER */
    uint16_t vin_amc = XMC_VADC_GROUP_GetResult(VADC_G1, 6);
    float v_inP = (4.97f * vin_amc / 4096 - 2.492f) / 2; // 2.5V是电平移位电压
    if(v_inP < 0)
    {
        *iso_rp = BMS_ISO_RP_MAX;
        return -2;
    }  
    float v_P = v_inP * (R_ST + RIN_AMC) / RIN_AMC;

    /* step 3: open Main switch 2, close Main switch 1 */
    XMC_GPIO_SetOutputLow(ISO_Switch1_Pin_PORT, ISO_Switch1_Pin_PIN);
    bms_delay_ms(5);
    XMC_GPIO_SetOutputHigh(ISO_Switch2_Pin_PORT, ISO_Switch2_Pin_PIN);

    /* step 4: wait 10ms then read isolate input voltage for AMC3330DWER */
    bms_delay_ms(5);
    vin_amc = XMC_VADC_GROUP_GetResult(VADC_G1, 6);
    float v_inN = (4.97f * vin_amc / 4096 - 2.492f) / 2;
    if(v_inN > 0)
    {
        *iso_rn = BMS_ISO_RN_MAX;
        return -3;
    }
    float v_N = v_inN * (R_ST + RIN_AMC) / RIN_AMC;

    /* step 5: caculate isolate resistor */
    *iso_rp = -1.0f * (RIN_AMC + R_ST) * (_bms_st.voltage + v_N - v_P) / v_N;
    *iso_rn = (RIN_AMC + R_ST) * (_bms_st.voltage + v_N - v_P) / v_P;
    return 0;
}

/// @brief BMS检查当前HVIL和主接触器的异常
/// @param  
static void bms_background_check_error(void)
{
    /* 主继电器上有2根线用来监测状态，当继电器吸合时这两根线导通，因此发送一个周期信号（PWM）也能接收到
        当继电器断开时，这两根线不导通，因此无法接收到发送的周期信号
     */

    /* 如果处于充电/放电状态，那么主接触器应当是闭合的，则辅助线圈上的PWM占空比应该是50%，否则说明
        1：接触器闭合失败
        2：辅助线圈断开
     */
    pwm_cap_info_t* main_coil_pwm_info = get_main_coil_pwm_cap();
    pwm_cap_info_t* hvil_pwm_info = get_HVIL_pwm_cap();

    if(main_coil_pwm_info->duty_cycle == 0 && _bms_st.state != Idle)
        _bms_st.error_state |= BMS_MainCoil_NotIdle_But_Disconnect;            
    else
        _bms_st.error_state &= (~BMS_MainCoil_NotIdle_But_Disconnect);

    /* 上强电之后，那么HVIL应当是闭合的，则PWM占空比应该是50%，否则说明
        1：高压存在开路隐患
     */
    if(hvil_pwm_info->duty_cycle == 0)
        _bms_st.error_state |= BMS_HVIL_Disconnect;
    else
        _bms_st.error_state &= (~BMS_HVIL_Disconnect);
}

static void bms_background_measure(void)
{
    uint16_t vadc_rslt = 0;

    /* 测量板载5V电压 */
    vadc_rslt = XMC_VADC_GROUP_GetResult(VADC_G1, 5);
    float vcc_5v = (5.0f * vadc_rslt / 4096 * BMS_5V_RES_SPLIT);

    /* 测量BMS总电压 */
    vadc_rslt = XMC_VADC_GROUP_GetResult(VADC_G0, 1);
    float bat_volt = (5.0f * vadc_rslt / 4096 / AMC3330_GAIN / BMS_VOLT_AMP_ZOOM * BMS_VOLT_SPLIT);
    _bms_st.voltage = bat_volt;

    /* 测量BMS动力电流 */
    vadc_rslt = XMC_VADC_GROUP_GetResult(VADC_G0, 3);
    _bms_st.power_current_A = (5.0f * vadc_rslt / 4096 - vcc_5v / 2 ) / 2.0f * BMS_HALL_CURRENT_MAX;

    /* 测量NTC1温度 */
    vadc_rslt = XMC_VADC_GROUP_GetResult(VADC_G1, 7);
    float resistor = 2.4f / (3.3f / (5.0f * vadc_rslt / 4096) - 1);
    _bms_st.NTC1_temp = 1.0f * NTC_res_calc_temp(resistor) / 100 - 273.15f;

    /* 测量NTC2温度 */
    vadc_rslt = XMC_VADC_GROUP_GetResult(VADC_G0, 0);
    resistor = 2.4f / (3.3f / (5.0f * vadc_rslt / 4096) - 1);
    _bms_st.NTC2_temp = 1.0f * NTC_res_calc_temp(resistor) / 100 - 273.15f;

    /* 测量绝缘电阻 */
    float iso_rp, iso_rn;
    if(iso_resistor_detection(&iso_rp, &iso_rn) == 0)
    {
        _bms_st.iso_RP = (uint16_t)iso_rp;
        _bms_st.iso_RN = (uint16_t)iso_rp;
    }

    /* 当低压5V网络正常时，低压指示灯亮起，否则熄灭 */
    if(vcc_5v > 4.5f && vcc_5v < 5.5f)
        XMC_GPIO_SetOutputHigh(LV_Light_Ctrl_Pin_PORT, LV_Light_Ctrl_Pin_PIN);
    else
        XMC_GPIO_SetOutputLow(LV_Light_Ctrl_Pin_PORT, LV_Light_Ctrl_Pin_PIN);
    
    /* 当BAT+电压大于HV_VOLTAGE_LIMIT时，高压指示灯亮起，否则熄灭 */
    if(_bms_st.voltage > HV_VOLTAGE_LIMIT)
        XMC_GPIO_SetOutputHigh(HV_Light_Ctrl_Pin_PORT, HV_Light_Ctrl_Pin_PIN);
    else
        XMC_GPIO_SetOutputLow(HV_Light_Ctrl_Pin_PORT, HV_Light_Ctrl_Pin_PIN);
}

/// @brief BMS根据当前状态，判断是否触发了保护动作
/// @param  
static void bms_background_check_protection(void)
{
    slave_node_t* ptr = NULL;
    slave_node_t* slave_head_ptr = get_slave_list_head();
    bms_config_t* bms_cfg_ptr = get_bms_config();
    
    uint32_t pack_volt = 0; /* pack电压 */
    uint8_t cell_flag = 0; /* 电芯过压和欠压 */
    uint8_t temp_flag = 0; /* 电芯高温和低温 */
    uint8_t current_flag = 0; /* 充电与放电过流 */

    /* 单体过压与欠压 */
    list_for_each_entry(ptr, &(slave_head_ptr->entry), entry)
    {
        for (uint8_t i = 0; i < slave_cfg.cell_serial_count; i++)
        {
            /* 单体过压 */
            if(ptr->slave_st.cmu_board_cell_voltage[i] > bms_cfg_ptr->cell_ov_limit)
            {
                cell_flag |= CELL_OV_FLAG;
                bms_charge_discharge(stop); /* 停止充电 */
                _bms_st.error_state |= BMS_CELL_OV_PROTECT; /* 添加错误标志 */
            }

            /* 单体欠压 */
            else if(ptr->slave_st.cmu_board_cell_voltage[i] < bms_cfg_ptr->cell_uv_limit)
            {
                cell_flag |= CELL_UV_FLAG;
                bms_charge_discharge(stop); /* 停止放电 */
                _bms_st.error_state |= BMS_CELL_UV_PROTECT; /* 添加错误标志 */
            }

            pack_volt += ptr->slave_st.cmu_board_cell_voltage[i];
        }
    }

    if((cell_flag & CELL_OV_FLAG ) == 0)
        _bms_st.error_state &= (~BMS_CELL_OV_PROTECT); /* 如果正常，去除错误标志 */

    if((cell_flag & CELL_UV_FLAG) == 0)
        _bms_st.error_state &= (~BMS_CELL_UV_PROTECT); /* 如果正常，去除错误标志 */

    /* 充电高温与充电低温 */
    if(_bms_st.state == ChargeDischarge && _bms_st.power_current_A < 0)
    {
        list_for_each_entry(ptr, &(slave_head_ptr->entry), entry)
        {
            for (uint8_t i = 0; i < slave_cfg.ntc_count; i++)
            {
                float temp = 1.0f * ptr->slave_st.cmu_board_ntc_temp_result[i] / SLAVE_TEMP_SCALE - ZERO_CELDIUS_KELVIN;
                if(temp > bms_cfg_ptr->charge_high_temp_limit)
                {
                    temp_flag |= CELL_OVER_TEMP_FLAG;
                    bms_charge_discharge(stop); /* 停止充电 */
                    _bms_st.error_state |= BMS_CHARGE_HIGH_TEMP_PROTECT; /* 添加错误标志 */
                }
                else if(temp < bms_cfg_ptr->charge_low_temp_limit)
                {
                    temp_flag |= CELL_UNDER_TEMP_FLAG;
                    bms_charge_discharge(stop); /* 停止充电 */
                    heat_slave_cell(ptr->slave_id, bms_cfg_ptr->charge_low_temp_limit + 2); /* 加热从机pack */
                    _bms_st.error_state |= BMS_CHARGE_LOW_TEMP_PROTECT; /* 添加错误标志 */
                }
            }
        }
    }

    if((temp_flag & CELL_OVER_TEMP_FLAG) == 0)
        _bms_st.error_state &= (~BMS_CHARGE_HIGH_TEMP_PROTECT); /* 如果正常，去除错误标志 */

    if((temp_flag & CELL_UNDER_TEMP_FLAG) == 0)
        _bms_st.error_state &= (~BMS_CHARGE_LOW_TEMP_PROTECT); /* 如果正常，去除错误标志 */

    temp_flag = 0;

    /* 放电高温与放电低温 */
    if(_bms_st.state == ChargeDischarge && _bms_st.power_current_A > 0)
    {
        list_for_each_entry(ptr, &(slave_head_ptr->entry), entry)
        {
            for (uint8_t i = 0; i < slave_cfg.ntc_count; i++)
            {
                float temp = 1.0f * ptr->slave_st.cmu_board_ntc_temp_result[i] / SLAVE_TEMP_SCALE - ZERO_CELDIUS_KELVIN;
                if(temp > bms_cfg_ptr->discharge_high_temp_limit)
                {
                    temp_flag |= CELL_OVER_TEMP_FLAG;
                    bms_charge_discharge(stop); /* 停止放电 */
                    _bms_st.error_state |= BMS_DISCHARGE_HIGH_TEMP_PROTECT; /* 添加错误标志 */
                }
                else if(temp < bms_cfg_ptr->discharge_low_temp_limit)
                {
                    temp_flag |= CELL_UNDER_TEMP_FLAG;
                    bms_charge_discharge(stop); /* 停止放电 */
                    heat_slave_cell(ptr->slave_id, bms_cfg_ptr->discharge_low_temp_limit + 2); /* 加热从机pack */
                    _bms_st.error_state |= BMS_DISCHARGE_LOW_TEMP_PROTECT; /* 添加错误标志 */
                }
            }
        }
    }

    if((temp_flag & CELL_OVER_TEMP_FLAG) == 0)
        _bms_st.error_state &= (~BMS_DISCHARGE_HIGH_TEMP_PROTECT); /* 如果正常，去除错误标志 */

    if((temp_flag & CELL_UNDER_TEMP_FLAG) == 0)
        _bms_st.error_state &= (~BMS_DISCHARGE_LOW_TEMP_PROTECT); /* 如果正常，去除错误标志 */

    temp_flag = 0;

    /* 充电与放电过流 */
    if(_bms_st.power_current_A > bms_cfg_ptr->discharge_current_limit) /* 放电过流 */
    {
        current_flag |= DISCHARGE_OVER_CURRENT_FLAG;
        bms_charge_discharge(stop); /* 停止放电 */
        _bms_st.error_state |= BMS_DISCHARGE_OVER_CURRENT_PROTECT; /* 添加错误标志 */
    }

    if(_bms_st.power_current_A < 0 && (-1 * _bms_st.power_current_A ) > bms_cfg_ptr->charge_current_limit) /* 充电过流 */
    {
        current_flag |= CHARGE_OVER_CURRENT_FLAG;
        bms_charge_discharge(stop); /* 停止充电 */
        _bms_st.error_state |= BMS_CHARGE_OVER_CURRENT_PROTECT; /* 添加错误标志 */
    }
        
    if((current_flag & DISCHARGE_OVER_CURRENT_FLAG) == 0)
        _bms_st.error_state &= (~BMS_DISCHARGE_OVER_CURRENT_PROTECT); /* 如果正常，去除错误标志 */

    if((current_flag & CHARGE_OVER_CURRENT_FLAG) == 0)
        _bms_st.error_state &= (~BMS_CHARGE_OVER_CURRENT_PROTECT); /* 如果正常，去除错误标志 */
}

/// @brief 获得RAM占用率，获得CPU使用率
static void bms_background_check_code_state()
{
    if(ram_usage_ticks >= 1000)
    {
        ram_usage_ticks = 0;
        _bms_st.ram_usage = bms_ram_usage(); /* 获取代码的RAM占用情况 */
        _bms_st.timestamp = bms_run_ticks / 1000; /* 从ms转到s */
    }

    _bms_st.cpu_usage = (uint8_t)(1.0f * (bms_run_ticks - bms_idle_ticks) / bms_run_ticks * 100); /* 获取CPU运行时负载率情况 */
}

/// @brief BMS后台工作
void bms_background_work()
{
    bms_background_measure(); /* BMS后台测量 */

    bms_background_check_error(); /* 检查HVIL高压回路完整性，以及主接触器的状态 */

    bms_background_check_protection(); /* 检查是否因为过压/欠压/高温/低温触发了保护 */

    bms_background_check_code_state(); /* 检查BMS主机代码的RAM占用率，CPU使用率 */

    bms_background_update_config_into_flash(); /* 检查是否需要将修改后的BMS配置更新到Flash */
}

/// @brief BMS执行充电/放电命令
/// @param st 
void bms_charge_discharge(coil_status_t st)
{
    if(st == start)
    {
        /* 打开主继电器 */
        XMC_GPIO_SetOutputHigh(Coil1_Pin_PORT, Coil1_Pin_PIN); 
        _bms_st.state = ChargeDischarge;
        bms_record_charge_discharge_cap(); /* 有放电动作 */
    }
    else
    {
        /* 关闭主继电器 */
        XMC_GPIO_SetOutputLow(Coil1_Pin_PORT, Coil1_Pin_PIN); 
        _bms_st.state = Idle;
    }

}

/// @brief BMS执行预充命令
/// @param st 
void bms_precharge(coil_status_t st)
{
    if(st == start)
    {
        /* 打开预充继电器 */
        XMC_GPIO_SetOutputHigh(Coil2_Pin_PORT, Coil2_Pin_PIN); 
        _bms_st.state = PreCharge;
        bms_record_charge_discharge_cap(); /* 有预充动作 */
    }
    else
    {
        /* 关闭预充继电器 */
        XMC_GPIO_SetOutputLow(Coil2_Pin_PORT, Coil2_Pin_PIN); 
        _bms_st.state = Idle;
    }
}


bms_status_t* read_bms_status()
{
    return &_bms_st;
}


slave_status_t* read_slave_status(uint8_t slave_id)
{
    slave_node_t* ptr = NULL;
    slave_node_t* slave_head_ptr = get_slave_list_head();
    list_for_each_entry(ptr, &(slave_head_ptr->entry), entry)
    {
        if(slave_id == ptr->slave_id)
        {
            return &(ptr->slave_st);
        }
    }

    return NULL;
}

const uint8_t get_slave_node_count()
{
    uint8_t slave_cnt = 0;
    slave_node_t* ptr = NULL;
    slave_node_t* slave_head_ptr = get_slave_list_head();
    list_for_each_entry(ptr, &(slave_head_ptr->entry), entry)
    {
        slave_cnt++;
    }

    return slave_cnt;
}

uint32_t* get_bms_know_flag()
{
    return &bms_know_slave_flag;
}

slave_config_t* get_slave_cfg()
{
    return &slave_cfg;
}

/* 系统1ms定时器 */
void CCU40_3_IRQHandler(void)
{
    prepare_ticks++;
    bms_background_ticks++;
    bms_polling_slave_ticks++;
    ram_usage_ticks++;
    bms_run_ticks++;
}




