
#ifndef CMU_BSP_CONFIG_H
#define CMU_BSP_CONFIG_H

#include <stdio.h>

// #define __DEBUG 

#ifdef __DEBUG
#define debug_print(format, ...) printf(format, ##__VA_ARGS__)
#else
#define debug_print(format, ...)
#endif

#define MAGIC_BYTE1 0x49
#define MAGIC_BYTE2 0x47
#define RESPONSE_FLAG 0x80

#define RS485_FRAME_MAGIC_BYTE1 0x49 /* "I" */
#define RS485_FRAME_MAGIC_BYTE2 0x47 /* "G" */

#define BMS_SOFT_REV 0x01
#define BMS_HARDWARE_REV 0x01
#define BMS_ID 0x02

#define BMS_HOST_RX_BUFFER_MAX 0x50 // 接收上位机帧最长80字节
#define BMS_HOST_MEMBLOCK_COUNT_MAX 0x05 // 接收上位机所用内存块数量
#define BMS_RS485_RX_BUFFER_MAX 0x80         // 接收RS485帧最长为128字节
#define BMS_RS485_TX_BUFFER_MAX 0x80        // 发送RS485帧最长为128字节
#define BMS_RS485_MEMBLOCK_COUNT_MAX 0x0A   // 接收RS485帧buffer的内存块数量

#define BMS_SLAVE_COUNT_MAX 0x10 // BMS主机能识别的最大从机数目
#define BMS_FRAME_NO_DATA_LEN 8 // 数据帧中无有效数据的长度，包括帧头、发送ID、接收ID、命令、数据长度、CRC校验

#define BMS_CHARGE_OVER_CURRENT_LIMIT 100 // 充电过流保护100A
#define BMS_DISCHARGE_CURRENT_LIMIT 480 // 放电过流保护480A
#define BMS_POWER_LINE_TEMP_LIMIT 120 // 动力线温度120℃

#define BMS_OVER_CURRENT_ERROR 0x01 // BMS过流错误
#define BMS_OVER_TEMP_ERROR 0x02 // BMS过温错误

#define AMC3330_GAIN 2 // AMC3300DWER作为增强型隔离式放大器，固定增益是2
#define BMS_VOLT_AMP_ZOOM 2 // 差分运放放大系数2
#define BMS_VOLT_SPLIT 251 // 电阻分压系数 251

#define BMS_5V_RES_SPLIT 2 // 电阻分压系数2
#define BMS_HALL_CURRENT_MAX 500 // 量程500A
#define BMS_ISO_RP_MAX 1000 // isoRp最大值1MΩ
#define BMS_ISO_RN_MAX 1000 // isoRp最大值1MΩ

#define SLAVE_NTC_NUM 4
#define SLAVE_CELL_SERIAL_COUNT 18
#define SLAVE_AFE_GPIO_COUNT 9

#define SLAVE_SOC_SCALE 30
#define SLAVE_ITEMP_SCALE 75
#define SLAVE_VOLTAGE_SCALE 0.0001f // 从100uV为单位转到V为单位
#define SLAVE_VOLT_TO_MVOLT_SCALE 1000 // 单位V转到mV为单位
#define SLAVE_TEMP_SCALE 100.f // 传输的温度值(K)与实际的温度值之间的比例系数

#define SLAVE_ID_MIN 0x0A
#define SLAVE_ID_MAX 0x15

#define BMS_KNOW_CELL_TYPE 0x01 /* BMS知道电芯类型 */
#define BMS_KNOW_CELL_CODE 0x02 /* BMS知道电芯代号 */
#define BMS_KNOW_PACK_CODE 0x04 /* BMS知道电池代号 */
#define BMS_KNOW_CELL_S 0x08 /* BMS知道单个电池包串数 */
#define BMS_KNOW_PACK_CAP 0x10 /* BMS知道电池包容量 */
#define BMS_KNOW_CMU_VERSION 0x20 /* BMS知道从机版本 */
#define BMS_KNOW_AFE_CODE 0x40 /* BMS知道AFE代号 */
#define BMS_KNOW_OV_UV_LIMIT 0x80 /* BMS知道电芯过压/欠压阈值 */
#define BMS_KNOW_NTC_INFO 0x100 /* BMS知道NTC信息 */
#define BMS_KNOW_BALANCE_CONFIG 0x200 /* BMS知道均衡配置 */

#define BMS_KNOW_NOTHING 0x00 /* BMS不了解从机 */
#define BMS_KNOW_ALL (BMS_KNOW_CELL_TYPE | BMS_KNOW_CELL_CODE | BMS_KNOW_PACK_CODE | BMS_KNOW_CELL_S | BMS_KNOW_PACK_CAP | BMS_KNOW_CMU_VERSION | BMS_KNOW_AFE_CODE | BMS_KNOW_OV_UV_LIMIT | BMS_KNOW_NTC_INFO | BMS_KNOW_BALANCE_CONFIG)

#define BMS_MainCoil_NotIdle_But_Disconnect 0x01 // 主接触器本应该处于充电/放电，但是辅助线圈无法闭合
#define BMS_HVIL_Disconnect 0x02 // 高压存在开路隐患
#define BMS_5V_Abnormal 0x04 // BMS板上低压部分电压异常
#define BMS_HV_OverVoltage 0x08 // 高压部分过压
#define BMS_HV_NotConnect 0x10// 没有接高压
#define BMS_CELL_OV_PROTECT 0x20 // 触发电芯过压保护
#define BMS_CELL_UV_PROTECT 0x40 // 触发电芯欠压保护
#define BMS_PACK_OV_PROTECT 0x80 // 触发pack过压保护
#define BMS_PACK_UV_PROTECT 0x0100 // 触发pack欠压保护
#define BMS_CHARGE_HIGH_TEMP_PROTECT 0x0200 // 触发充电过温保护
#define BMS_CHARGE_LOW_TEMP_PROTECT 0x0400 // 触发充电低温保护
#define BMS_DISCHARGE_HIGH_TEMP_PROTECT 0x0200 // 触发放电过温保护
#define BMS_DISCHARGE_LOW_TEMP_PROTECT 0x0400 // 触发放电低温保护
#define BMS_CHARGE_OVER_CURRENT_PROTECT 0x0800 // 触发充电过流保护
#define BMS_DISCHARGE_OVER_CURRENT_PROTECT 0x1000 // 触发放电过流保护


//// 配置1
// #define CELL_TYPE Lithium_Iron_Phosphate  /* 磷酸铁锂 */
// #define CELL_CODE "INP24.8256106A-115.8Ah"
// #define CELL_SOURCE "国轩高科"
// #define PACK_CODE "IG_PACK 5.2kWh"
// #define CELL_SERIAL_COUNT 13
// #define PACK_CAPCITY 118
// #define HARDWARE_VERSION 1
// #define SOFTWARE_VERSION 1
// #define AFE_NAME "MT9805"
// #define OV_LIMIT 3.8f /* 过压阈值3.8V */
// #define UV_LIMIT 2.5f /* 欠压阈值2.5f */
// #define NTC_COUNT 4
// #define NTC_CODE "10K 3950"
// #define CMU_ID 0x0A
// #define CMU_BALANCE_DEFAULT_START_VOLT 3.6f // 默认均衡开启电压3.6V

//配置2
#define CELL_TYPE Ternary_lithium  /* 三元锂 */
#define CELL_CODE "xxxx-0.3Ah"
#define CELL_SOURCE "xxxx"
#define PACK_CODE "IG_PACK 14.4Wh"
#define CELL_SERIAL_COUNT 13
#define PACK_CAPCITY 5
#define HARDWARE_VERSION 1
#define SOFTWARE_VERSION 1
#define AFE_NAME "MT9805"
#define OV_LIMIT 4.2f /* 过压阈值4.2V */
#define UV_LIMIT 3.0f /* 欠压阈值3.0f */
#define NTC_COUNT 4
#define NTC_CODE "10K 3950"
#define BALANCE_DEFAULT_START_VOLT 4.0f // 默认均衡开启电压4.0V
#define MAX_OV_LIMIT 4.2f // 过压阈值的最大值
#define DEFAULT_OV_LIMIT 4.15f // 过压阈值默认值
#define MIN_OV_LIMIT 4.0f // 过压阈值的最小值
#define MAX_UV_LIMIT 3.4f // 欠压阈值的最大值
#define DEFAULT_UV_LIMIT 3.0f // 过压阈值默认值
#define MIN_UV_LIMIT 2.8f // 欠压阈值的最小值



#define RIN_AMC 0.12f // 绝缘电阻监测中，AMC3300所测电阻120Ω
#define R_ST 70 // 绝缘电阻监测中，上下桥外接电阻70K

#define HV_VOLTAGE_LIMIT 36 // 当BAT+电压超过36V，即认为有高压，亮起指示灯

#define HOST_ID 0x01 // 主机/上位机ID

// 如果定义了 RX_WITH_LIST ，表示使用list链表存储接收数据，这样可以尽可能少的丢失接收命令
// 如果没有定义 RX_WITH_LIST， 表示使用阻塞模式来接收数据，每次接收一个命令后，关闭接收中断，等待处理完-发送完响应后再开启接收中断
#define RX_WITH_LIST

#endif