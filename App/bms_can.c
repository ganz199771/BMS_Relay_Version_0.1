

#include "bms_can.h"
#include "bms.h"
#include "bms_config.h"
#include "bms_soc.h"
#include "bms_rs485_uart.h"

#include <xmc_can.h>
#include <cycfg.h>

#include "string.h"

static bms_can_identify_t bms_can_identify_union;
static bms_can_ID_info_t bms_can_id_info;

/// @brief 获得BMS上传数据包的状态域的值
/// @return 
static uint32_t bms_can_update_status()
{
    bms_status_t* bms_st_ptr = read_bms_status();
    uint32_t bat_status = 0;

    if((bms_st_ptr->error_state & BMS_CHARGE_OVER_CURRENT_PROTECT) || 
    (bms_st_ptr->error_state & BMS_DISCHARGE_OVER_CURRENT_PROTECT)) /* 过流 */
    {
        bat_status |= BMS_FK_OVER_CURRENT;
    }

    if((bms_st_ptr->error_state & BMS_CELL_OV_PROTECT) || 
    (bms_st_ptr->error_state & BMS_PACK_OV_PROTECT)) /* 过压 */
    {
        bat_status |= BMS_FK_OVER_VOLTAGE;
    }

    if((bms_st_ptr->error_state & BMS_CELL_UV_PROTECT) || 
    (bms_st_ptr->error_state & BMS_PACK_UV_PROTECT)) /* 欠压 */
    {
        bat_status |= BMS_FK_UNDER_VOLTAGE;
    }

    if((bms_st_ptr->error_state & BMS_CHARGE_HIGH_TEMP_PROTECT) || 
    (bms_st_ptr->error_state & BMS_DISCHARGE_HIGH_TEMP_PROTECT)) /* 高温 */
    {
        bat_status |= BMS_FK_OVER_TEMP;
    }

    if((bms_st_ptr->error_state & BMS_CHARGE_LOW_TEMP_PROTECT) || 
    (bms_st_ptr->error_state & BMS_DISCHARGE_LOW_TEMP_PROTECT)) /* 低温 */
    {
        bat_status |= BMS_FK_UNDER_TEMP;
    }

    slave_node_t* ptr = NULL;
    slave_node_t* slave_node_head_ptr = get_slave_list_head();
    slave_config_t* slave_cfg_ptr = get_slave_cfg();
    slave_status_t* slave_st_ptr = NULL;

    uint16_t max_cell_volt = 0;
    uint16_t min_cell_volt = 4500;

    list_for_each_entry(ptr, &(slave_node_head_ptr->entry), entry)
    {
        slave_st_ptr = read_slave_status(ptr->slave_id);
        for (uint8_t i = 0; i < slave_cfg_ptr->cell_serial_count; i++)
        {
            if(slave_st_ptr->cmu_board_cell_voltage[i] > max_cell_volt)
                max_cell_volt = slave_st_ptr->cmu_board_cell_voltage[i]; /* 取最大单体电压 */

            if(slave_st_ptr->cmu_board_cell_voltage[i] < min_cell_volt)
                min_cell_volt = slave_st_ptr->cmu_board_cell_voltage[i]; /* 取最小单体电压 */
        }

        if(max_cell_volt - min_cell_volt > BMS_FK_DIFF_LIMIT)
            bat_status |= BMS_FK_VOLT_DIF;
    }

    bms_config_t* bms_cfg_ptr = read_bms_config();
    if(bms_st_ptr->SOC * SOC_PRECISION < bms_cfg_ptr->discharge_soc_limit)
    {
        bat_status |= BMS_FK_SOC_WARN;
    }

    if(bms_st_ptr->power_current_A < 0 && (-1 * bms_st_ptr->power_current_A > 2 * bms_cfg_ptr->charge_current_limit))
    {
        bat_status |= BMS_FK_CHARGE_SHORT;
    }

    return bat_status;
}


/// @brief BMS主机所能监测到电芯的电压
/// @param index 电芯的序号，对于BMS主机，假设其在初始化时将4个从机添加到从机链表，且每个从机有13个电芯，
/// 那么电芯的序号就是：0 ~ 51
/// @return 获取对应序号的电芯电压，单位mV
static uint16_t cell_volt_at_index(uint8_t index)
{
    slave_node_t* ptr = NULL;
    slave_node_t* slave_node_head_ptr = get_slave_list_head();
    slave_config_t* slave_cfg_ptr = get_slave_cfg();
    slave_status_t* slave_st_ptr = NULL;

    /* 如果index是12，slave_cfg_ptr->cell_serial_count是13，那么pack_index是0，
        如果index是15，slave_cfg_ptr->cell_serial_count是13，那么pack_index是1
     */
    uint8_t pack_index = index / slave_cfg_ptr->cell_serial_count; 
    uint8_t cell_index = index % slave_cfg_ptr->cell_serial_count; 
    uint8_t i = 0;

    list_for_each_entry(ptr, &(slave_node_head_ptr->entry), entry)
    {
        if(i == pack_index)
        {
            slave_st_ptr = read_slave_status(ptr->slave_id);
            return slave_st_ptr->cmu_board_cell_voltage[cell_index];
        }
        i++;
    }

    return 0;
}


void bms_can_init(void)
{
    bms_can_identify_union.num = 1; /* 第一帧 */
    bms_can_identify_union.eof = 1; /* 默认单帧传输 */
    bms_can_identify_union.sof = 1; /* 默认单帧传输 */
    bms_can_identify_union.src_node_id = BMS_CAN_SLAVE_ID; /* BMS主机向飞控发送CAN报文，源ID是0x02 */
    bms_can_identify_union.dest_node_id = BMS_CAN_MASTER_ID; /* BMS主机向飞控发送CAN报文，目标ID是0x01 */
    bms_can_identify_union.flag = 7; /* 固定值0'b111 */
    bms_can_identify_union.msg_id = BMS_CAN_MSG_TYPE_1; /* BMS实时信息上传 */
    bms_can_identify_union.type = 0; /* 固定值0'b0000 */

    /* 对于接收messeage object，其过滤器只针对目标ID，当目标ID是自己（BMS_CAN_SLAVE_ID）时，接收消息 */

    strcpy(bms_can_id_info.manufacturer, "Intellectual Gull"); /* 智鸥 */
    strcpy(bms_can_id_info.bat_code, "IG-BAT-3C-137Ah"); /* 电池型号 */
    sprintf(bms_can_id_info.bat_ID, "%02x", BMS_CAN_SLAVE_ID); /* ID */
    sprintf(bms_can_id_info.bat_hardware_version, "%02x", BMS_HARDWARE_VERSION); /* 硬件版本 */
    sprintf(bms_can_id_info.bat_software_version, "%02x", BMS_SOFTWARE_VERSION); /* 软件版本 */
}


void bms_upload_state_info(void)
{
    /* 第一包数据 */
    bms_can_identify_union.num = 1; /* 第一帧 */
    bms_can_identify_union.eof = 0; /* 第一帧，没有结束 */
    bms_can_identify_union.sof = 1; /* 第一帧，开始 */
    bms_can_identify_union.src_node_id = BMS_CAN_SLAVE_ID; /* BMS主机向飞控发送CAN报文，源ID是0x02 */
    bms_can_identify_union.dest_node_id = BMS_CAN_MASTER_ID; /* BMS主机向飞控发送CAN报文，目标ID是0x01 */
    bms_can_identify_union.msg_id = BMS_CAN_MSG_TYPE_1; /* BMS实时信息上传 */

    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));

    bms_status_t* bms_st_ptr = read_bms_status();

    CAN_Node_LMO_0.can_data_byte[0] = ((bms_st_ptr->voltage * BMS_FK_VOLT_SCALE) & 0xff);
    CAN_Node_LMO_0.can_data_byte[1] = ((bms_st_ptr->voltage * BMS_FK_VOLT_SCALE) >> 8);

    uint32_t bms_frame_current = bms_st_ptr->power_current_A * BMS_FK_CURRENT_SCALE;
    CAN_Node_LMO_0.can_data_byte[2] = (bms_frame_current & 0xff);
    bms_frame_current >>= 8;
    CAN_Node_LMO_0.can_data_byte[3] = (bms_frame_current & 0xff);
    bms_frame_current >>= 8;
    CAN_Node_LMO_0.can_data_byte[4] = (bms_frame_current & 0xff);
    bms_frame_current >>= 8;
    CAN_Node_LMO_0.can_data_byte[5] = (bms_frame_current & 0xff);

    slave_node_t* slave_node_head_ptr = get_slave_list_head();
    slave_node_t* ptr = NULL;
    slave_status_t* slave_st_ptr = NULL;
    uint16_t max_cell_temp = 0;

    list_for_each_entry(ptr, &(slave_node_head_ptr->entry), entry)
    {
        slave_st_ptr = read_slave_status(ptr->slave_id);
        for (size_t i = 0; i < 4; i++)
        {
            if(slave_st_ptr->cmu_board_ntc_temp_result[i] > max_cell_temp)
                max_cell_temp = slave_st_ptr->cmu_board_ntc_temp_result[i]; /* 取最大温度 */
        }
    }

    int16_t cell_temp = (max_cell_temp / SLAVE_TEMP_SCALE - ZERO_CELDIUS_KELVIN) * BMS_FK_TEMP_SCALE;
    CAN_Node_LMO_0.can_data_byte[6] = (cell_temp & 0xff);
    CAN_Node_LMO_0.can_data_byte[7] = (cell_temp >> 8);

    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */

    /* 第二包数据 */
    bms_can_identify_union.num = 2; /* 第二帧 */
    bms_can_identify_union.eof = 0; /* 中间帧，没有结束 */
    bms_can_identify_union.sof = 0; /* 中间帧 */

    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));

    int16_t mosfet_temp = bms_st_ptr->NTC1_temp * BMS_FK_TEMP_SCALE;
    CAN_Node_LMO_0.can_data_byte[0] = (mosfet_temp & 0xff);
    CAN_Node_LMO_0.can_data_byte[1] = (mosfet_temp >> 8);

    uint16_t soc_val = bms_st_ptr->SOC * BMS_FK_SOC_SCALE;
    CAN_Node_LMO_0.can_data_byte[2] = (soc_val & 0xff); /* 从0.01%到0.1% */
    CAN_Node_LMO_0.can_data_byte[3] = (soc_val >> 8);
    
    uint32_t bat_status = bms_can_update_status();

    CAN_Node_LMO_0.can_data_byte[4] = (bat_status & 0xff);
    bat_status >>= 8;
    CAN_Node_LMO_0.can_data_byte[5] = (bat_status & 0xff);
    bat_status >>= 8;
    CAN_Node_LMO_0.can_data_byte[6] = (bat_status & 0xff);
    bat_status >>= 8;
    CAN_Node_LMO_0.can_data_byte[7] = (bat_status & 0xff);

    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */

    /* 第三包数据 */
    bms_can_identify_union.num = 3; /* 第三帧 */
    bms_can_identify_union.eof = 0; /* 中间帧，没有结束 */
    bms_can_identify_union.sof = 0; /* 中间帧 */

    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));

    slave_config_t* slave_cfg_ptr = get_slave_cfg();
    uint16_t cell_count = slave_cfg_ptr->cell_serial_count; /* 总的从机节点数量 */
    cell_count *= get_slave_node_count(); /* 总的从机节点数量 * 单从机电芯数量 = 总电芯数量 */

    CAN_Node_LMO_0.can_data_byte[0] = (cell_count & 0xff);
    CAN_Node_LMO_0.can_data_byte[1] = (cell_count >> 8);

    list_for_each_entry(ptr, &(slave_node_head_ptr->entry), entry)
    {
        slave_st_ptr = read_slave_status(ptr->slave_id);
        break;
    }

    /* 当总的电芯数量小于3，在第3包就发送完成 */
    if(cell_count < 3)
    {
        for (uint8_t i = 1; i <= cell_count; i++)
        {
            CAN_Node_LMO_0.can_data_byte[2 * i] = (slave_st_ptr->cmu_board_cell_voltage[i] & 0xff);
            CAN_Node_LMO_0.can_data_byte[2 * i + 1] = (slave_st_ptr->cmu_board_cell_voltage[i] >> 8);
        }
        bms_can_identify_union.eof = 1; /* 结束帧 */
        CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));

        XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
        XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */
        return;
    }

    /* 总的电芯数量>3 */

    uint16_t cell_index = 0;
    for (uint8_t i = 1; i <= 3; i++)
    {
        CAN_Node_LMO_0.can_data_byte[2 * i] = (slave_st_ptr->cmu_board_cell_voltage[i] & 0xff);
        CAN_Node_LMO_0.can_data_byte[2 * i + 1] = (slave_st_ptr->cmu_board_cell_voltage[i] >> 8);
    }
    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */

    cell_index += 3; /* 已发送电芯电压 */

    /* 第三包发送结束，开始发送后续报文 */
    while (cell_count > (cell_index + 4)) /* 只要剩余电芯数量大于4，就还可以发送一个完整的报文（8字节） */
    {
        bms_can_identify_union.num++; 
        bms_can_identify_union.eof = 0; /* 中间帧，没有结束 */
        bms_can_identify_union.sof = 0; /* 中间帧 */

        CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));

        for (uint8_t i = 0; i < 4; i++)
        {
            CAN_Node_LMO_0.can_data_byte[2 * i] = (cell_volt_at_index(cell_index + i) & 0xff);
            CAN_Node_LMO_0.can_data_byte[2 * i + 1] = (cell_volt_at_index(cell_index + i) >> 8);
        }

        cell_index += 4;
    }
    
    /* 最后一帧报文，剩余序号 cell_index ~ (cell_count - 1)，如果cell_count是13，那么就是11~12 */
    bms_can_identify_union.num++; 
    bms_can_identify_union.eof = 1; /* 结束帧 */
    bms_can_identify_union.sof = 0; /* 结束帧 */

    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    for (uint8_t i = cell_index; i < cell_count; i++)
    {
        CAN_Node_LMO_0.can_data_byte[2 * (i - cell_index)] = (cell_volt_at_index(i) & 0xff);
        CAN_Node_LMO_0.can_data_byte[2 * (i - cell_index) + 1] = (cell_volt_at_index(i) >> 8);
    }

    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */
}

/// @brief BMS向飞控回复身份信息
static void bms_response_ID_CAN()
{
    XMC_CAN_NODE_EnableConfigurationChange(CAN_Node_HW);
    bms_can_identify_union.num = 1; /* 第一帧 */
    bms_can_identify_union.eof = 0; /* 第一帧，没有结束 */
    bms_can_identify_union.sof = 1; /* 第一帧，开始 */
    bms_can_identify_union.src_node_id = BMS_CAN_SLAVE_ID; /* BMS主机向飞控发送CAN报文，源ID是0x02 */
    bms_can_identify_union.dest_node_id = BMS_CAN_MASTER_ID; /* BMS主机向飞控发送CAN报文，目标ID是0x01 */
    bms_can_identify_union.msg_id = BMS_CAN_MSG_TYPE_2; /* BMS回复ID */

    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    XMC_CAN_MO_Config(&CAN_Node_LMO_0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_Node_HW);

    memcpy(&CAN_Node_LMO_0.can_data_byte[0], bms_can_id_info.manufacturer, 8); /* 8字节 */
    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */

    XMC_CAN_NODE_EnableConfigurationChange(CAN_Node_HW);
    bms_can_identify_union.num++; /* 第二帧 */
    bms_can_identify_union.eof = 0; /* 第一帧，没有结束 */
    bms_can_identify_union.sof = 0; /* 第一帧，开始 */

    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    XMC_CAN_MO_Config(&CAN_Node_LMO_0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_Node_HW);

    memcpy(&CAN_Node_LMO_0.can_data_byte[0], &(bms_can_id_info.manufacturer[8]), 8); /* 8字节 */
    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */

    XMC_CAN_NODE_EnableConfigurationChange(CAN_Node_HW);
    bms_can_identify_union.num++; /* 第三帧 */
    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    XMC_CAN_MO_Config(&CAN_Node_LMO_0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_Node_HW);

    memcpy(&CAN_Node_LMO_0.can_data_byte[0], &(bms_can_id_info.manufacturer[16]), 4); /* 4字节 */
    memcpy(&CAN_Node_LMO_0.can_data_byte[4], &(bms_can_id_info.bat_code[0]), 4); /* 4字节 */
    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */

    XMC_CAN_NODE_EnableConfigurationChange(CAN_Node_HW);
    bms_can_identify_union.num++; /* 第四帧 */
    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    XMC_CAN_MO_Config(&CAN_Node_LMO_0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_Node_HW);

    memcpy(&CAN_Node_LMO_0.can_data_byte[0], &(bms_can_id_info.bat_code[4]), 8); /* 8字节 */
    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */

    XMC_CAN_NODE_EnableConfigurationChange(CAN_Node_HW);
    bms_can_identify_union.num++; /* 第五帧 */
    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    XMC_CAN_MO_Config(&CAN_Node_LMO_0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_Node_HW);

    memcpy(&CAN_Node_LMO_0.can_data_byte[0], &(bms_can_id_info.bat_code[12]), 8); /* 8字节 */
    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */

    XMC_CAN_NODE_EnableConfigurationChange(CAN_Node_HW);
    bms_can_identify_union.num++; /* 第六帧 */
    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    XMC_CAN_MO_Config(&CAN_Node_LMO_0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_Node_HW);

    memcpy(&CAN_Node_LMO_0.can_data_byte[0], &(bms_can_id_info.bat_ID[0]), 8); /* 8字节 */
    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */

    XMC_CAN_NODE_EnableConfigurationChange(CAN_Node_HW);
    bms_can_identify_union.num++; /* 第七帧 */
    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    XMC_CAN_MO_Config(&CAN_Node_LMO_0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_Node_HW);

    memcpy(&CAN_Node_LMO_0.can_data_byte[0], &(bms_can_id_info.bat_ID[8]), 8); /* 8字节 */
    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */

    XMC_CAN_NODE_EnableConfigurationChange(CAN_Node_HW);
    bms_can_identify_union.num++; /* 第八帧 */
    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    XMC_CAN_MO_Config(&CAN_Node_LMO_0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_Node_HW);

    memcpy(&CAN_Node_LMO_0.can_data_byte[0], &(bms_can_id_info.bat_ID[16]), 4); /* 4字节 */
    memcpy(&CAN_Node_LMO_0.can_data_byte[0], &(bms_can_id_info.bat_hardware_version[0]), 4); /* 4字节 */
    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */

    XMC_CAN_NODE_EnableConfigurationChange(CAN_Node_HW);
    bms_can_identify_union.num++; /* 第九帧 */
    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    XMC_CAN_MO_Config(&CAN_Node_LMO_0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_Node_HW);

    memcpy(&CAN_Node_LMO_0.can_data_byte[0], &(bms_can_id_info.bat_hardware_version[4]), 6); /* 6字节 */
    memcpy(&CAN_Node_LMO_0.can_data_byte[0], &(bms_can_id_info.bat_software_version[0]), 2); /* 2字节 */
    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */

    XMC_CAN_NODE_EnableConfigurationChange(CAN_Node_HW);
    bms_can_identify_union.num++; /* 第十帧 */
    bms_can_identify_union.eof = 1; /* 结束 */
    bms_can_identify_union.sof = 0; 
    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    XMC_CAN_MO_Config(&CAN_Node_LMO_0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_Node_HW);

    memcpy(&CAN_Node_LMO_0.can_data_byte[0], &(bms_can_id_info.bat_software_version[2]), 8); /* 8字节 */
    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */
}

/// @brief BMS向飞控回复基本信息
static void bms_response_basic_CAN()
{
    XMC_CAN_NODE_EnableConfigurationChange(CAN_Node_HW);
    bms_can_identify_union.num = 1; /* 第一帧 */
    bms_can_identify_union.eof = 0; /* 第一帧，没有结束 */
    bms_can_identify_union.sof = 1; /* 第一帧，开始 */
    bms_can_identify_union.src_node_id = BMS_CAN_SLAVE_ID; /* BMS主机向飞控发送CAN报文，源ID是0x02 */
    bms_can_identify_union.dest_node_id = BMS_CAN_MASTER_ID; /* BMS主机向飞控发送CAN报文，目标ID是0x01 */
    bms_can_identify_union.msg_id = BMS_CAN_MSG_TYPE_3; /* BMS回复基本信息 */

    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    XMC_CAN_MO_Config(&CAN_Node_LMO_0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_Node_HW);

    slave_config_t* slave_cfg_ptr = get_slave_cfg();

    uint16_t cap = slave_cfg_ptr->pack_capacity; /* 容量Ah */
    cap *= BMS_FK_CAP_SCALE; /* 容量0.1Ah */
    CAN_Node_LMO_0.can_data_byte[0] = cap & 0xff;
    CAN_Node_LMO_0.can_data_byte[1] = cap >> 8;

    bms_status_t* bms_st_ptr = read_bms_status();
    uint16_t discharge_rate = (uint16_t)(1.0f * bms_st_ptr->power_current_A / slave_cfg_ptr->pack_capacity); /* 放电电流A / 容量Ah 得到放电倍率 */
    CAN_Node_LMO_0.can_data_byte[2] = discharge_rate & 0xff; /* 放电倍率 */
    CAN_Node_LMO_0.can_data_byte[3] = discharge_rate >> 8;

    float cell_volt = 3.7f;
    if(slave_cfg_ptr->cell_type == Lithium_Iron_Phosphate)
    {
        cell_volt = 3.2f;
    }
    uint16_t volt = cell_volt * BMS_FK_VOLT_SCALE;
    CAN_Node_LMO_0.can_data_byte[4] = volt & 0xff; /* 电芯额定电压 */
    CAN_Node_LMO_0.can_data_byte[5] = volt >> 8;

    uint16_t cell_count = get_slave_node_count();
    cell_count *= slave_cfg_ptr->cell_serial_count;
    CAN_Node_LMO_0.can_data_byte[6] = cell_count & 0xff; /* 电芯串数 */
    CAN_Node_LMO_0.can_data_byte[7] = cell_count >> 8;

    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */
}

/// @brief BMS向飞控回复状态信息
static void bms_response_status_CAN()
{
    XMC_CAN_NODE_EnableConfigurationChange(CAN_Node_HW);
    bms_can_identify_union.num = 1; /* 第一帧 */
    bms_can_identify_union.eof = 0; /* 第一帧，没有结束 */
    bms_can_identify_union.sof = 1; /* 第一帧，开始 */
    bms_can_identify_union.src_node_id = BMS_CAN_SLAVE_ID; /* BMS主机向飞控发送CAN报文，源ID是0x02 */
    bms_can_identify_union.dest_node_id = BMS_CAN_MASTER_ID; /* BMS主机向飞控发送CAN报文，目标ID是0x01 */
    bms_can_identify_union.msg_id = BMS_CAN_MSG_TYPE_4; /* BMS回复基本信息 */

    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    XMC_CAN_MO_Config(&CAN_Node_LMO_0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_Node_HW);

    bms_status_t* bms_st_ptr = read_bms_status(); /* 获取BMS状态 */
    bms_config_t* bms_cfg_ptr = read_bms_config(); /* 获取BMS配置 */

    uint16_t val = (uint16_t)(bms_cfg_ptr->soh * SOH_PRECISION); /* SOH */
    CAN_Node_LMO_0.can_data_byte[0] = val & 0xff;
    CAN_Node_LMO_0.can_data_byte[1] = val >> 8; 

    val = bms_cfg_ptr->cycle_times; /* 循环次数 */
    CAN_Node_LMO_0.can_data_byte[2] = val & 0xff;
    CAN_Node_LMO_0.can_data_byte[3] = val >> 8; 

    val = bms_cfg_ptr->over_volt_times; /* 过充次数 */
    CAN_Node_LMO_0.can_data_byte[4] = val & 0xff;
    CAN_Node_LMO_0.can_data_byte[5] = val >> 8;
    
    val = bms_cfg_ptr->under_volt_times; /* 过放次数 */
    CAN_Node_LMO_0.can_data_byte[6] = val & 0xff;
    CAN_Node_LMO_0.can_data_byte[7] = val >> 8;

    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */

    XMC_CAN_NODE_EnableConfigurationChange(CAN_Node_HW);
    bms_can_identify_union.num++; /* 第二帧 */
    bms_can_identify_union.eof = 0; 
    bms_can_identify_union.sof = 0; 

    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    XMC_CAN_MO_Config(&CAN_Node_LMO_0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_Node_HW);

    val = bms_cfg_ptr->over_current_times; /* 过流次数 */
    CAN_Node_LMO_0.can_data_byte[0] = val & 0xff;
    CAN_Node_LMO_0.can_data_byte[1] = val >> 8; 

    val = bms_cfg_ptr->over_temp_times; /* 过温次数 */
    CAN_Node_LMO_0.can_data_byte[2] = val & 0xff;
    CAN_Node_LMO_0.can_data_byte[3] = val >> 8; 

    val = bms_st_ptr->SOC * SOC_PRECISION; /* SOC */
    CAN_Node_LMO_0.can_data_byte[4] = val & 0xff;
    CAN_Node_LMO_0.can_data_byte[5] = val >> 8; 

    val = bms_cfg_ptr->soh * SOH_PRECISION; /* SOH */
    CAN_Node_LMO_0.can_data_byte[6] = val & 0xff;
    CAN_Node_LMO_0.can_data_byte[7] = val >> 8; 

    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */

    XMC_CAN_NODE_EnableConfigurationChange(CAN_Node_HW);
    bms_can_identify_union.num++; /* 第二帧 */
    bms_can_identify_union.eof = 1; /* 第3帧，结束 */
    bms_can_identify_union.sof = 0; 

    CAN_Node_LMO_0.can_identifier = *((uint32_t*)(&bms_can_identify_union));
    XMC_CAN_MO_Config(&CAN_Node_LMO_0);
    XMC_CAN_NODE_DisableConfigurationChange(CAN_Node_HW);

    val = bms_cfg_ptr->charge_cap; /* 从使用以来所有的充电容量 */
    CAN_Node_LMO_0.can_data_byte[0] = val & 0xff;
    CAN_Node_LMO_0.can_data_byte[1] = val >> 8; 

    val = bms_st_ptr->SOC * SOC_PRECISION / HUANDRED_PERCENT * bms_cfg_ptr->soh * SOH_PRECISION / HUANDRED_PERCENT * bms_cfg_ptr->bat_total_cap; /* 剩余容量0.1Ah */
    CAN_Node_LMO_0.can_data_byte[2] = val & 0xff;
    CAN_Node_LMO_0.can_data_byte[3] = val >> 8; 

    val = bms_cfg_ptr->internal_res; /* 内阻 */
    CAN_Node_LMO_0.can_data_byte[4] = val & 0xff;
    CAN_Node_LMO_0.can_data_byte[5] = val >> 8; 

    XMC_CAN_MO_UpdateData(&CAN_Node_LMO_0); /* 更新数据 */
    XMC_CAN_MO_Transmit(&CAN_Node_LMO_0); /* 发送数据 */
}

void CAN0_0_IRQHandler(void)
{
    XMC_CAN_MO_Receive(&CAN_Node_LMO_1); /* 读取消息 */

    uint8_t target_id = ((CAN_Node_LMO_1.can_identifier & BMS_DST_NODE_ID_MASK) >> BMS_DST_NODE_ID_SHIFT);

    /* 对于BMS主机，其从CAN总线接收消息，应该首先判断是不是发给自己的 */
    if(target_id != BMS_CAN_SLAVE_ID)
        return;

    uint8_t source_id = ((CAN_Node_LMO_1.can_identifier & BMS_SRC_NODE_ID_MASK) >> BMS_SRC_NODE_ID_SHIFT);

    /* 对于BMS主机，其从CAN总线接收消息，如果消息不是从飞控发出的，也忽略掉 */
    if(source_id != BMS_CAN_MASTER_ID)
        return;

    uint8_t msg_id = ((CAN_Node_LMO_1.can_identifier & BMS_MSG_ID_MASK) >> BMS_MSG_ID_SHIFT);
    switch (msg_id)
    {
    case BMS_CAN_MSG_TYPE_1:
        break;

    case BMS_CAN_MSG_TYPE_2:
        bms_response_ID_CAN();
        break;

    case BMS_CAN_MSG_TYPE_3:
        bms_response_basic_CAN();
        break;

    case BMS_CAN_MSG_TYPE_4:
        bms_response_status_CAN();
        break;

    default:
        break;
    }
}



