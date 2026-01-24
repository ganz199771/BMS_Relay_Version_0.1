
#ifndef _BMS_VERSION_0P1_H_
#define _BMS_VERSION_0P1_H_

#include <stdint.h>
#include "list.h"
#include "config.h"

#define ZERO_CELDIUS_KELVIN 273.15f /* 0℃对应热力学温度K */
#define BMS_SLAVE_STATE_NOT_FETCH_FLAG 0xff


#define CELL_OV_FLAG 0x01
#define CELL_UV_FLAG 0x02
#define PACK_OV_FLAG 0x01
#define PACK_UV_FLAG 0x02
#define CELL_OVER_TEMP_FLAG 0x01
#define CELL_UNDER_TEMP_FLAG 0x02
#define CHARGE_OVER_CURRENT_FLAG 0x01
#define DISCHARGE_OVER_CURRENT_FLAG 0x01

/// @brief 存储接收数据指针
typedef struct bms_rx_node
{
    struct list_head entry;
    uint8_t* buffer_ptr; // 存储数据指针
}bms_rx_node_t;

typedef struct slave_status
{
    uint32_t LiPo_OV_Status; // 每个电芯的OV状态集合，前24bit有效
    uint32_t LiPo_UV_Status; // 每个电芯的UV状态集合，前24bit有效
    uint32_t MT9805_DCC; // 均衡通道，每一个bit=1表示对于通道均衡开启，前24bit有效

    uint32_t open_wire; // 断路状态，每个bit=1表示对应电芯连接到CMU板的线断了，前24bit有效
    uint16_t cmu_board_ntc_temp_result[SLAVE_NTC_NUM]; // NTC温度值，= 实际温度(K) * 100
    uint16_t cmu_board_cell_voltage[SLAVE_CELL_SERIAL_COUNT]; // 电芯电压值，单位mV
    uint16_t pack_volt; // 单位mV 
    uint8_t chip_temp; // 单位摄氏度
} slave_status_t;

typedef struct slave_config
{
    uint8_t cell_type; // 电芯类型
    uint8_t cell_serial_count; // 电芯数量
    uint8_t pack_capacity; // 电池容量Ah
    uint8_t cmu_hardware_version: 4; // CMU硬件版本
    uint8_t cmu_software_version: 4; // CMU软件版本
    uint8_t ntc_count; // NTC数量

    char ntc_code[0x10]; // NTC型号
    char cell_code_string[0x20]; // 电芯型号，例如"GXGK202511086"
    char cell_source[0x10]; // 电芯来源，例如"国轩高科"
    char pack_code_string[0x10]; // 电池代号
    char cmu_afe_code_string[0x10]; // AFE型号
} slave_config_t;

typedef struct slave_node
{
    struct list_head entry;
    uint8_t slave_id; // 从机ID
    slave_status_t slave_st; // 从机状态
}slave_node_t;


typedef struct bms_cmu_rs485_frame
{
    uint8_t magic_tag[2]; // 帧头
    uint8_t transmitter_id; // 发送方ID
    uint8_t receiver_id; // 接收方ID
    uint8_t type; // 最高位为0表示请求，最高位为1表示响应
    uint8_t data_len;
    uint8_t* data;
    uint16_t crc;
} bms_cmu_rs485_frame_t;


typedef enum bms_cmd_type
{
    Active, // 确认是否在线 

    Read_Pack_Config, // 读取电池包配置
    Read_Cell_Type, // 读取电芯类型
    Read_Cell_Code_String, // 读取电芯型号
    Read_Pack_Code_String, // 读取电池代号
    Read_Cell_Serial_Count, // 读取电芯串数
    Read_Pack_Capacity, // 读取电芯容量
    Read_CMU_Version, // 读取CMU版本
    Read_CMU_AFE_Code_String, // 读取AFE型号
    Read_Cell_OV_UV_Limit, // 读取过压欠压阈值
    Read_NTC_Info, // 读取NTC数量和NTC型号
    Read_Balance_Config, // 读取均衡配置

    Read_Pack_State, // 上位机询问BMS主板，某个从机的状态，包括电压、NTC温度、过压欠压状态、均衡开关
    Read_Balance_State, // 均衡通道状态
    Read_all_cell_Voltage, // 读取电芯电压
    Read_all_gpio_Voltage, // 读取GPIO电压
    Read_all_NTC_Temp, // 读取所有温度
    Read_cell_OV_UV_status, // 读取过压欠压状态
    Read_Pack_Voltage, // 读取PACK总压
    Read_ChipTEMP, // 读取AFE芯片温度

    Open_circuit_inspection, // 电芯断路监测
    Control_Balance, // 强制启动/关闭均衡
    Heat_Pack, // 对pack进行加热

    Read_BMS_Config, // 读取BMS配置
    Read_BMS_Slave_Count, // 读取BMS从机个数
    Read_BMS_State, // 上位机询问BMS主板关于其状态，包含了：总压、总电流、板载NTC温度、充电/放电/空闲、BMS错误、绝缘电阻
    Read_All_Bat_Volt, // 主机/上位机读取BMS主板监测到的电池总压（串联之后）
    Read_Power_Current, // 主机/上位机读取BMS主板监测到的动力线电流
    Read_BMS_NTC_Temp, // 主机/上位机读取BMS主板监测到的NTC温度值
    Report_BMS_Error, // 报告BMS错误

    Control_PreCharge, // 控制预充
    Control_ChargeDischarge, // 控制充电/放电
    Write_BMS_Config, // 更新BMS配置
    Restore_BMS_Config // 将BMS配置恢复到出厂值
}bms_cmd_type_t;


typedef enum cell_type_enum
{
    Zinc_manganese, // 锌锰电池
    Ternary_lithium, // 三元锂离子电池
    Lithium_Iron_Phosphate, // 磷酸铁锂电池
    Lead_acid, // 铅酸蓄电池
    Nickel_metal_hydride, // 镍氢电池
    Solar, // 太阳能电池
    Thermoelectric, // 温差电池
    Nuclear, // 核电池
    Cell_Kinds
}cell_type_t;

typedef union u32_union
{
    uint32_t u32_data;
    uint16_t u16_data[2];
    uint8_t u8_data[4];
}u32_union_t;

typedef enum bms_charge_state
{
    Idle, // 空闲
    PreCharge, // 预充
    ChargeDischarge // 充电/放电
} bms_charge_state_t;

typedef enum bms_prepare_state
{
    wake, /* 初始状态 */
    know_all_slave_id, /* 获得所有的从机ID */
    know_slave_config, /* 获得了从机的配置 */
    know_slave_inital_state, /* 获得了从机初始状态 */
    init_soc_done, /* 完成了SOC初值的初始化 */
    prepared /* 所有都准备就绪 */
}bms_preapare_state_t;

/* 使用状态机管理BMS状态 */
typedef enum bms_FSM_state
{
    bms_start, /* 初始状态 */
    bms_prepared, /* 做玩前期准备工作 */
    bms_work_background, /* BMS执行后台任务，例如测量电流、电压 */
    bms_poll_slave, /* BMS轮询从机状态 */
}bms_FSM_state_t;


typedef struct bms_status
{
    uint16_t timestamp; // 时间戳，表明BMS主机启动后运行时间，单位s
    uint16_t voltage; // 电池总电压，单位V
    int16_t power_current_A; // 动力线电流，单位A，大于0表示放电，小于0表示充电
    uint16_t iso_RP; // BAT+和PE之间绝缘电阻，单位KΩ
    uint16_t iso_RN; // BAT-和PE之间绝缘电阻，单位KΩ
    uint16_t error_state; // 错误状态标志集合

    int8_t NTC1_temp; // NTC1温度
    int8_t NTC2_temp; // NTC2温度
    uint8_t state; // BMS状态
    uint16_t SOC; // SOC值，单位0.01，例如5208表示SOC=52.08%
    uint8_t ram_usage; // RAM使用率，范围在0~100
    uint8_t cpu_usage; // CPU使用率，范围在0~100
} bms_status_t;

typedef enum coil_status
{
    start,
    stop
} coil_status_t;


/// @brief 初始化BMS配置
/// @param  
void bms_init(void);

/// @brief BMS主机响应命令/数据帧
/// @param pframe 帧
void bms_response();

/// @brief 启动BMS运行任务，内部由状态机管理
void bms_run();

/// @brief BMS对正常运行需要进行一些准备，例如获得所有的从机ID，从机配置，从机Pack的初始状态，SOC初始值等
/// @param  
void bms_prepare(void);

/// @brief BMS主机轮询哪些从机在线
void bms_poll_active_slave();

/// @brief BMS主机轮询从机状态
void bms_poll_slave_status();

/// @brief BMS主机后台任务
void bms_background_work();

/// @brief BMS充电/放电
void bms_charge_discharge(coil_status_t st);

/// @brief BMS预充（对于负载来说是预充，对于电池来说是小电流预放电）
void bms_precharge(coil_status_t st);

/// @brief BMS命令从机开启加热功能
/// @param slave_id 从机ID
/// @param slave_id 加热到电芯温度为 heat_until_temp ℃时停止
void heat_slave_cell(uint8_t slave_id, uint8_t heat_until_temp);

/// @brief 获取当前BMS状态
/// @return 
bms_status_t* read_bms_status();

/// @brief 获取某个特定的从机的信息
/// @param slave_id 
/// @return 
slave_status_t* read_slave_status(uint8_t slave_id);

/// @brief 获取BMS从机节点数量
/// @return 
const uint8_t get_slave_node_count();

/// @brief 获取BMS对从机配置参数的了解状况
/// @return 
uint32_t* get_bms_know_flag();

/// @brief 获取从机配置信息
/// @return 
slave_config_t* get_slave_cfg();

/// @brief 检查电芯电压是否满足2个条件，如果满足则开启均衡
/// 1：电芯电压 > 均衡开启电压
/// 2：电芯电压压差 > 均衡精度
/// @param p_slave_st 
void bms_prepare_balance(uint8_t slave_id, slave_status_t* p_slave_st);


#endif