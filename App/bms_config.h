

#ifndef BMS_FLASH_CONFIG_H
#define BMS_FLASH_CONFIG_H

#include <stdint.h>

#define XMC_SECTOR_ADDR   (uint32_t *)0x10032000U // 存储BMS状态信息结构体的位置
#define BMS_CONFIG_FIXHEAD  0x49474940 /* BMS配置信息固定头部 */


#define CELL_OV_LIMIT_DEFAULT 4250 /* 单体过压4.25V */
#define CELL_UV_LIMIT_DEFAULT 2850 /* 单体欠压2.85V */
#define PACK_OV_LIMIT_DEFAULT 55 /* 电池包过压55V */
#define PACK_UV_LIMIT_DEFAULT 30 /* 电池包欠压30V */
#define CHARGE_HIGH_TEMP_DEFAULT 55 /* 充电过温55℃ */
#define CHARGE_LOW_TEMP_DEFAULT -20 /* 充电低温-20℃ */
#define DISCHARGE_HIGH_TEMP_DEFAULT 65 /* 放电高温65℃ */
#define DISCHARGE_LOW_TEMP_DEFAULT -30 /* 放电低温-30℃ */
#define CHARGE_CURRENT_DEFAULT 50 /* 充电过流50A */
#define DISCHARGE_CURRENT_DEFAULT 450 /* 放电过流450A */
#define BALANCE_START_mVOLT_DEFAULT 3850 /* 均衡开启电压3.85V */
#define BALANCE_PRECISION_mVOLT_DEFAULT 10 /* 均衡精度10mV */
#define BAT_TOTAL_CAPCITY_DEFAULT 1158 /* 电池系统总容量115.8Ah */
#define CYCLE_TIMES_DEFAULT 0 /* 循环次数 */
#define CHARGE_TOTAL_CAP_DEFAULT 0 /* 总的充电容量Ah */
#define DISCHARGE_TOTAL_CAP_DEFAULT 0 /* 总的放电容量Ah */

/* 如何定义一个锂电池电芯的循环？
    循环次数的科学定义是指电池完成一次100%深度充放电的过程，但这并不意味着必须一次性完成。
    比如，今天使用电池40%的电量然后充满，明天使用60%的电量后再充满，这两次部分放电累计达到100%，即计为一次完整的充放电循环 。
 */


typedef struct bms_config
{
    uint16_t cell_ov_limit; /* 电芯过压阈值，单位mV，当电芯电压 > 此值时，需要停止充电 */
    uint16_t cell_uv_limit; /* 电芯欠压阈值，单位mV，当电芯电压 < 此值时，需要停止放电 */
    uint16_t pack_ov_limit; /* pack过压阈值，单位V，当PACK电压 > 此值时，需要停止充电 */
    uint16_t pack_uv_limit; /* pack欠压阈值，单位V，当PACK电压 < 此值时，需要停止放电 */
    int8_t charge_high_temp_limit; /* 充电时电芯高温阈值，单位℃，当电芯温度 > 此值时需要停止充电 */
    int8_t charge_low_temp_limit; /* 充电时电芯低温阈值，单位℃，当电芯温度 < 此值时需要停止充电，先打开加热膜升温，直到电芯温度 > 此值 */
    int8_t discharge_high_temp_limit; /* 放电时电芯高温阈值，单位℃，当电芯温度 > 此值时需要停止放电 */
    int8_t discharge_low_temp_limit; /* 放电时电芯低温阈值，单位℃，当电芯温度 < 此值时需要先打开加热膜升温，直到电芯温度 > 此值 */
    uint16_t charge_current_limit; /* 充电过流阈值，单位A，当充电电流 > 此值时，需要停止充电 */
    uint16_t discharge_current_limit; /* 放电过流阈值，单位A，当放电电流 > 此值时，需要停止放电 */

    uint16_t balance_start_volt; // 均衡开启电压，单位mV
    uint16_t balance_precision; // 均衡精度，单位mV
    uint16_t bat_total_cap; // 容量，单位0.1Ah

    uint16_t cycle_times; /* 循环次数 */
    uint32_t charge_cap; /* 从全新电池开始使用来，总的充电容量，单位0.1Ah */
    uint32_t discharge_cap; /* 从全新电池开始使用来，总的放电容量，单位0.1Ah */
}bms_config_t;

/// @brief 初始化BMS配置
/// @param  
void bms_config_init(void);

/// @brief 获取BMS配置结构体
/// @return 
bms_config_t* get_bms_config(void);

/// @brief 将BMS初始配置写入到Flash
/// @param  
void bms_init_flash_config(void);

/// @brief 当上位机修改了BMS主机的配置参数时，调用此函数，让BMS主机在掉电之前将配置写回到flash，
/// 假如在使用上位机时，没有修改BMS配置参数，那么BMS主机在掉电之前，不需要修改Flash的配置参数
/// @param  
void app_changed_bms_config(void);

/// @brief 在BMS经历了放电与充电时，需要记录此处放电/充电的电量
/// @param  
void bms_record_charge_discharge_cap(void);

/// @brief 如果SOC配置、BMS配置发生了更改，此时更新到Flash，如果没有则不做任何动作
/// @param  
void bms_background_update_config_into_flash(void);

#endif









