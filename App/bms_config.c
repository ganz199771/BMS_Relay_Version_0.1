
#include "bms_config.h"

#include "xmc_flash.h"
#include <stdlib.h>

#include "config.h"
#include "cycfg_peripherals.h"

static bms_config_t bms_config_data;

typedef enum bms_config_modified
{
    yes,
    no
}bms_config_modified_t;


typedef union u32_union
{
    uint32_t u32_data;
    uint16_t u16_data[2];
    uint8_t u8_data[4];
}u32_union_t;

static bms_config_modified_t app_modify_bms_cfg = no; /* App修改BMS一些配置，例如高温、低温保护、过流保护、过压保护等 */
static bms_config_modified_t bms_modify_soc_cfg = no; /* BMS在充电、放电过程中应当记录充电与放电容量 */

static void app_update_bms_config_into_flash(void);
static void bms_update_SOC_related_data_into_flash(void);

void bms_init_flash_config(void)
{
    bms_config_data.cell_ov_limit = CELL_OV_LIMIT_DEFAULT;
    bms_config_data.cell_uv_limit = CELL_UV_LIMIT_DEFAULT;
    bms_config_data.pack_ov_limit = PACK_OV_LIMIT_DEFAULT;
    bms_config_data.pack_uv_limit = PACK_UV_LIMIT_DEFAULT;
    bms_config_data.charge_high_temp_limit = CHARGE_HIGH_TEMP_DEFAULT;
    bms_config_data.charge_low_temp_limit = CHARGE_LOW_TEMP_DEFAULT;
    bms_config_data.discharge_high_temp_limit = DISCHARGE_HIGH_TEMP_DEFAULT;
    bms_config_data.discharge_low_temp_limit = DISCHARGE_LOW_TEMP_DEFAULT;
    bms_config_data.charge_current_limit = CHARGE_CURRENT_DEFAULT;
    bms_config_data.discharge_current_limit = DISCHARGE_CURRENT_DEFAULT;
    bms_config_data.balance_start_volt = BALANCE_START_mVOLT_DEFAULT;
    bms_config_data.balance_precision = BALANCE_PRECISION_mVOLT_DEFAULT;
    bms_config_data.bat_total_cap = BAT_TOTAL_CAPCITY_DEFAULT;
    bms_config_data.cycle_times = CYCLE_TIMES_DEFAULT;
    bms_config_data.charge_cap = CHARGE_TOTAL_CAP_DEFAULT;
    bms_config_data.discharge_cap = DISCHARGE_TOTAL_CAP_DEFAULT;

    app_update_bms_config_into_flash();
    bms_update_SOC_related_data_into_flash();
}

/// @brief APP修改了BMS配置，发送给BMS后，BMS在掉电之前将配置写入到Flash
/// @param  
static void app_update_bms_config_into_flash(void)
{
    // 一个page=64个word=256字节
    XMC_FLASH_ErasePage(XMC_SECTOR_ADDR); // 先擦除
    uint32_t config_data[XMC_FLASH_WORDS_PER_BLOCK] =
    {
        BMS_CONFIG_FIXHEAD, 0xffffffffU, 0xffffffffU, 0xffffffffU
    };

    u32_union_t cfg_data;
    cfg_data.u16_data[0] = bms_config_data.cell_ov_limit;
    cfg_data.u16_data[1] = bms_config_data.cell_uv_limit;
    config_data[1] = cfg_data.u32_data;

    cfg_data.u16_data[0] = bms_config_data.pack_ov_limit;
    cfg_data.u16_data[1] = bms_config_data.pack_uv_limit;
    config_data[2] = cfg_data.u32_data;

    cfg_data.u8_data[0] = bms_config_data.charge_high_temp_limit;
    cfg_data.u8_data[1] = bms_config_data.charge_low_temp_limit;
    cfg_data.u8_data[2] = bms_config_data.discharge_high_temp_limit;
    cfg_data.u8_data[3] = bms_config_data.discharge_low_temp_limit;
    config_data[3] = cfg_data.u32_data;

    XMC_FLASH_WriteBlocks(XMC_SECTOR_ADDR, config_data, 1, true); // 写配置数据

    cfg_data.u16_data[0] = bms_config_data.charge_current_limit;
    cfg_data.u16_data[1] = bms_config_data.discharge_current_limit;
    config_data[0] = cfg_data.u32_data;

    cfg_data.u16_data[0] = bms_config_data.balance_start_volt;
    cfg_data.u16_data[1] = bms_config_data.balance_precision;
    config_data[1] = cfg_data.u32_data;

    cfg_data.u16_data[0] = bms_config_data.bat_total_cap;
    cfg_data.u16_data[1] = 0xffff;
    config_data[2] = cfg_data.u32_data;
    XMC_FLASH_WriteBlocks(XMC_SECTOR_ADDR + sizeof(uint32_t) * XMC_FLASH_WORDS_PER_BLOCK, config_data, 1, true); // 写配置数据

    app_modify_bms_cfg = no; /* 上位机修改BMS配置已完成，置标志位no */
}

static void bms_update_SOC_related_data_into_flash(void)
{
    u32_union_t cfg_data;
    uint32_t config_data[XMC_FLASH_WORDS_PER_BLOCK];

    /* 关于循环次数、总充电容量、总放电容量，放在另一块page */
    cfg_data.u16_data[0] = bms_config_data.cycle_times; /* 循环次数 */
    cfg_data.u16_data[1] = 0xffff;
    config_data[0] = cfg_data.u32_data;

    cfg_data.u32_data = bms_config_data.charge_cap;
    config_data[1] = cfg_data.u32_data;

    cfg_data.u32_data = bms_config_data.discharge_cap;
    config_data[2] = cfg_data.u32_data;

    config_data[3] = 0xffffffff;
    XMC_FLASH_ErasePage(XMC_SECTOR_ADDR + XMC_FLASH_BYTES_PER_PAGE); /* 擦除 */
    XMC_FLASH_WriteBlocks(XMC_SECTOR_ADDR + XMC_FLASH_BYTES_PER_PAGE, config_data, 1, true); // 写配置数据

    bms_modify_soc_cfg = no; /* 修改充放电信息已完成，置标志位no */
}

/// @brief 从Flash读取配置信息并填充到结构体
/// @param  
static void read_config_file_from_flash(void)
{
    uint32_t bms_config_buf[4] = {0};
    XMC_FLASH_ReadBlocks(XMC_SECTOR_ADDR, bms_config_buf, 1);

    u32_union_t cfg_data;
    cfg_data.u32_data = bms_config_buf[1];
    bms_config_data.cell_ov_limit = cfg_data.u16_data[0];
    bms_config_data.cell_uv_limit = cfg_data.u16_data[1];

    cfg_data.u32_data = bms_config_buf[2];
    bms_config_data.pack_ov_limit = cfg_data.u16_data[0];
    bms_config_data.pack_uv_limit = cfg_data.u16_data[1];

    cfg_data.u32_data = bms_config_buf[3];
    bms_config_data.charge_high_temp_limit = cfg_data.u8_data[0];
    bms_config_data.charge_low_temp_limit = cfg_data.u8_data[1];
    bms_config_data.discharge_high_temp_limit = cfg_data.u8_data[2];
    bms_config_data.discharge_low_temp_limit = cfg_data.u8_data[3];

    XMC_FLASH_ReadBlocks(XMC_SECTOR_ADDR + sizeof(uint32_t) * XMC_FLASH_WORDS_PER_BLOCK, bms_config_buf, 1);

    cfg_data.u32_data = bms_config_buf[0];
    bms_config_data.charge_current_limit = cfg_data.u16_data[0];
    bms_config_data.discharge_current_limit = cfg_data.u16_data[1];

    cfg_data.u32_data = bms_config_buf[1];
    bms_config_data.balance_start_volt = cfg_data.u16_data[0];
    bms_config_data.balance_precision = cfg_data.u16_data[1];

    cfg_data.u32_data = bms_config_buf[2];
    bms_config_data.bat_total_cap = cfg_data.u16_data[0]; /* 电池总容量 */

    /* 循环次数、充电容量、放电容量 */
    XMC_FLASH_ReadBlocks(XMC_SECTOR_ADDR + XMC_FLASH_BYTES_PER_PAGE, bms_config_buf, 1);
    cfg_data.u32_data = bms_config_buf[0];
    bms_config_data.cycle_times = cfg_data.u16_data[0]; /* 循环次数 */

    cfg_data.u32_data = bms_config_buf[1];
    bms_config_data.charge_cap = cfg_data.u32_data; /* 总充电容量 */

    cfg_data.u32_data = bms_config_buf[2];
    bms_config_data.discharge_cap = cfg_data.u32_data; /* 总放电容量 */
}


void bms_config_init(void)
{
    uint32_t bms_config_buf[4] = {0};
    // 对于XMC1404来说，一个word是4个字节，1个block是4个word，对应于16个字节
    XMC_FLASH_ReadBlocks(XMC_SECTOR_ADDR, bms_config_buf, 1);

    if(bms_config_buf[0] != BMS_CONFIG_FIXHEAD) // 没有配置信息，写一个默认的BMS状态信息结构体
    {
        bms_init_flash_config();
        debug_print("first build BMS config file in flash.\r\n");
    }

    else // 有配置信息，从对应位置读取CMU状态信息
    {
        read_config_file_from_flash();
        debug_print("read BMS config file from flash done.\r\n");
    }
}

bms_config_t* get_bms_config()
{
    return &bms_config_data;
}

// /* 中断例程中将修改后的配置写入到Flash */
// void eru_1_ogu_0_INTERRUPT_HANDLER(void)
// {
//     __disable_irq(); /* 关闭所有中断 */

//     if(app_modify_bms_cfg == yes)
//         app_update_bms_config_into_flash(); /* 写回到flash */
//     if(bms_modify_soc_cfg)
//         bms_update_SOC_related_data_into_flash();

//     __enable_irq(); /* 开启中断 */
// }

void app_changed_bms_config(void)
{
    app_modify_bms_cfg = yes;
}

void bms_record_charge_discharge_cap(void)
{
    bms_modify_soc_cfg = yes;
}

void bms_background_update_config_into_flash(void)
{
    if(app_modify_bms_cfg == yes)
        app_update_bms_config_into_flash();

    if(bms_modify_soc_cfg == yes)
        bms_update_SOC_related_data_into_flash();
}
