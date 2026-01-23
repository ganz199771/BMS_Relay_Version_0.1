

#include "bms_rs485_uart.h"

#include <xmc_uart.h>
#include <cycfg.h>
#include "bms.h"

#include "mempool.h"
#include "bms_config.h"


static MemoryPool* _bms_rs485_mempool = NULL; // 存储BMS主机接收从机数据帧
static MemoryPool* _bms_host_mempool = NULL; // 存储BMS主机接收上位机数据帧
static MemoryPool* _bms_slave_mempool = NULL; // 存储BMS从机信息

static bms_cmu_rs485_frame_t rs485_rx_frame; /* 在RS485总线上接收从机数据帧 */
static bms_cmu_rs485_frame_t host_uart_rx_frame; /* 接收上位机机数据帧 */

static uint8_t rs485_rx_pos = 0;
static uint8_t rs485_rx_buffer[BMS_RS485_RX_BUFFER_MAX];
static uint8_t rs485_tx_buffer[BMS_RS485_TX_BUFFER_MAX];

static uint8_t host_uart_rx_pos = 0;
static uint8_t host_uart_rx_buffer[BMS_RS485_RX_BUFFER_MAX];
static uint8_t host_uart_tx_buffer[BMS_RS485_TX_BUFFER_MAX];

static bms_rx_node_t _rx_node_head;
static slave_node_t _slave_node_head;

void InvertUint8(uint8_t *DesBuf, uint8_t *SrcBuf)
{
    uint8_t temp = 0;
 
    for (int i = 0; i < 8; i++) {
        if (SrcBuf[0] & (1 << i)) {
            temp |= 1 << (7 - i);
        }
    }
    DesBuf[0] = temp;
}

void InvertUint16(uint16_t *DesBuf, uint16_t *SrcBuf)  
{
    uint16_t temp = 0;
 
    for (int i = 0; i < 16; i++) {
        if (SrcBuf[0] & (1 << i)) {
            temp |= 1 << (15 - i);
        }
    }
    DesBuf[0] = temp;
}

uint16_t CRC16_MODBUS(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint16_t wCRCin = 0xFFFF;
    uint16_t wCPoly = 0x8005;
    uint8_t wChar = 0;
 
    while (usDataLen--) {
        wChar = *(puchMsg++);
        InvertUint8(&wChar, &wChar);
        wCRCin ^= (wChar << 8);
 
        for (int i = 0; i < 8; i++)
        {
            if (wCRCin & 0x8000) {
                wCRCin = (wCRCin << 1) ^ wCPoly;
            } else {
                wCRCin = wCRCin << 1;
            }
        }
    }
    InvertUint16(&wCRCin, &wCRCin);
    return (wCRCin);
}

/// @brief 重新启动485接收中断
/// @param  
void restart_rs485(void)
{
    XMC_UART_CH_EnableEvent(RS485_Node_HW, XMC_UART_CH_EVENT_STANDARD_RECEIVE);
}


void rs485_transmit(uint8_t* frame_data, uint16_t frame_len)
{
    XMC_GPIO_SetOutputHigh(RS485_Mode_Pin_PORT, RS485_Mode_Pin_PIN);
    XMC_Delay(1);
    for (uint16_t i = 0; i < frame_len; i++)
    {
        XMC_UART_CH_Transmit(RS485_Node_HW, frame_data[i]);
        uint16_t tx_check_times_left = 0xffff;
        while (!(XMC_UART_CH_GetStatusFlag(RS485_Node_HW) & XMC_UART_CH_STATUS_FLAG_TRANSMITTER_FRAME_FINISHED))
        {
            tx_check_times_left--;
            if(tx_check_times_left == 0)
                return;
        }
    }
    XMC_Delay(1);
    XMC_GPIO_SetOutputLow(RS485_Mode_Pin_PORT, RS485_Mode_Pin_PIN);
}

void host_uart_transmit(uint8_t* frame_data, uint16_t frame_len)
{
    for (uint16_t i = 0; i < frame_len; i++)
    {
        XMC_UART_CH_Transmit(HOST_UART_HW, frame_data[i]);
        uint16_t tx_check_times_left = 0xff;
        while (!(XMC_UART_CH_GetStatusFlag(HOST_UART_HW) & XMC_UART_CH_STATUS_FLAG_TRANSMITTER_FRAME_FINISHED))
        {
            tx_check_times_left--;
            if(tx_check_times_left == 0)
                return;
        }
    }
}

void bms_rs485_uart_tx_init(void)
{
    rs485_tx_buffer[0] = RS485_FRAME_MAGIC_BYTE1; /* I */
    rs485_tx_buffer[1] = RS485_FRAME_MAGIC_BYTE2; /* G */

    host_uart_tx_buffer[0] = RS485_FRAME_MAGIC_BYTE1;
    host_uart_tx_buffer[1] = RS485_FRAME_MAGIC_BYTE2;
}


/// @brief 初始化RS485,UART相关资源
void bms_rs485_uart_init()
{
    _bms_rs485_mempool = InitMemoryPool(BMS_RS485_RX_BUFFER_MAX + sizeof(bms_rx_node_t), BMS_RS485_MEMBLOCK_COUNT_MAX);
    if(!_bms_rs485_mempool)
        return;

    _bms_host_mempool = InitMemoryPool(BMS_HOST_RX_BUFFER_MAX + sizeof(bms_rx_node_t), BMS_HOST_MEMBLOCK_COUNT_MAX);
    if(!_bms_host_mempool)
        return;

    INIT_LIST_HEAD(&_rx_node_head.entry); // 初始化接收链表
    _rx_node_head.buffer_ptr = NULL;

    _bms_slave_mempool = InitMemoryPool(sizeof(slave_node_t), BMS_SLAVE_COUNT_MAX);
    if(!_bms_slave_mempool)
        return;

    INIT_LIST_HEAD(&(_slave_node_head.entry)); // 初始化从机节点链表
}

/// @brief 将RS485总线上的接收帧插入到总接收链表
/// @param frame 
static void bms_store_rs485_frame(bms_cmu_rs485_frame_t* frame)
{
    uint8_t* rx_buffer = AllocateBlock(_bms_rs485_mempool);
    if(!rx_buffer)
        return;
    
    XMC_UART_CH_DisableEvent(RS485_Node_HW, XMC_UART_CH_EVENT_STANDARD_RECEIVE);

    bms_rx_node_t* new_rx_node = (bms_rx_node_t*)rx_buffer; // 分配空间：12字节给 bms_rx_node_t 结构体，剩余32字节给接收buffer
    new_rx_node->buffer_ptr = &rx_buffer[sizeof(bms_rx_node_t)];

    memcpy(new_rx_node->buffer_ptr, rs485_rx_buffer, BMS_FRAME_NO_DATA_LEN + rs485_rx_frame.data_len); 
    list_add(&new_rx_node->entry , &_rx_node_head.entry);

    XMC_UART_CH_EnableEvent(RS485_Node_HW, XMC_UART_CH_EVENT_STANDARD_RECEIVE);
}

/// @brief 将UART上的接收帧插入到总接收链表
/// @param frame 
static void bms_store_host_uart_frame(bms_cmu_rs485_frame_t* frame)
{
    uint8_t* rx_buffer = AllocateBlock(_bms_host_mempool);
    if(!rx_buffer)
        return;
    
    XMC_UART_CH_DisableEvent(HOST_UART_HW, XMC_UART_CH_EVENT_STANDARD_RECEIVE);
    bms_rx_node_t* new_rx_node = (bms_rx_node_t*)rx_buffer; // 分配空间：12字节给 bms_rx_node_t 结构体，剩余32字节给接收buffer
    new_rx_node->buffer_ptr = &rx_buffer[sizeof(bms_rx_node_t)];

    memcpy(new_rx_node->buffer_ptr, host_uart_rx_buffer, BMS_FRAME_NO_DATA_LEN + host_uart_rx_frame.data_len); 
    list_add(&new_rx_node->entry , &_rx_node_head.entry);

    XMC_UART_CH_EnableEvent(HOST_UART_HW, XMC_UART_CH_EVENT_STANDARD_RECEIVE);
}

/// @brief 使用UART与上位机通信，接收字节
/// @param  __attribute__ ((section (".ram_code")))
void  HOST_UART_RECEIVE_EVENT_HANDLER(void)
{
    uint8_t u8_data = XMC_UART_CH_GetReceivedData(HOST_UART_HW);
    
    if(host_uart_rx_pos == 0 && u8_data != RS485_FRAME_MAGIC_BYTE1) // 头字节1
    {
        return;
    }

    else if(host_uart_rx_pos == 1 && u8_data != RS485_FRAME_MAGIC_BYTE2) // 头字节2
    {
        host_uart_rx_pos = 0;
        return;
    }

    else if(host_uart_rx_pos == 2) // 发送方id
    {
        host_uart_rx_frame.transmitter_id = u8_data;
    }

    else if(host_uart_rx_pos == 3) // 接收方id
    {
        if(BMS_ID != u8_data)
        {
            host_uart_rx_pos = 0;
            return;
        }
        host_uart_rx_frame.receiver_id = u8_data;
    }

    else if(host_uart_rx_pos == 4) // 命令
    {
        host_uart_rx_frame.type = u8_data;
    }

    else if(host_uart_rx_pos == 5) // 数据长度
    {
        host_uart_rx_frame.data_len = u8_data; 
    }

    else
    {
        if(host_uart_rx_pos == host_uart_rx_frame.data_len + 6) // CRC16低字节
        {
            host_uart_rx_frame.crc = u8_data;
        }
        else if(host_uart_rx_pos == host_uart_rx_frame.data_len + 7) // CRC16高字节
        {
            host_uart_rx_frame.crc <<= 8;
            host_uart_rx_frame.crc |= u8_data;

            if(host_uart_rx_frame.crc != CRC16_MODBUS(host_uart_rx_buffer, 6 + host_uart_rx_frame.data_len)) // CRC校验不通过
            {
                host_uart_rx_pos = 0;
                return;
            }

            // CRC校验通过, rx_frame合法
            host_uart_rx_pos = 0;
            host_uart_rx_frame.data = &host_uart_rx_buffer[6];
            bms_store_host_uart_frame(&host_uart_rx_frame);
            return;
        }
    }

    host_uart_rx_buffer[host_uart_rx_pos++] = u8_data; // 前面2个字节是固定的"I" "G"，因此没有修改，除此之外，帧内的所有字节被记录在对应的位置上
    if(host_uart_rx_pos == BMS_RS485_RX_BUFFER_MAX - 1)
        host_uart_rx_pos = 0;
}

/// @brief 使用RS485与从机节点通信
/// @param  __attribute__ ((section (".ram_code")))
void  RS485_Node_RECEIVE_EVENT_HANDLER(void)
{
    uint8_t u8_data = XMC_UART_CH_GetReceivedData(RS485_Node_HW);

    if(rs485_rx_pos == 0 && u8_data != RS485_FRAME_MAGIC_BYTE1) // 头字节1
    {
        return;
    }

    else if(rs485_rx_pos == 1 && u8_data != RS485_FRAME_MAGIC_BYTE2) // 头字节2
    {
        rs485_rx_pos = 0;
        return;
    }

    else if(rs485_rx_pos == 2) // 发送方id
    {
        rs485_rx_frame.transmitter_id = u8_data;
    }

    else if(rs485_rx_pos == 3) // 接收方id
    {
        if(BMS_ID != u8_data)
        {
            rs485_rx_pos = 0;
            return;
        }
        rs485_rx_frame.receiver_id = u8_data;
    }

    else if(rs485_rx_pos == 4) // 命令
    {
        rs485_rx_frame.type = u8_data;
    }

    else if(rs485_rx_pos == 5) // 数据长度
    {
        rs485_rx_frame.data_len = u8_data; 
    }

    else
    {
        if(rs485_rx_pos == rs485_rx_frame.data_len + 6) // CRC16低字节
        {
            rs485_rx_frame.crc = u8_data;
        }
        else if(rs485_rx_pos == rs485_rx_frame.data_len + 7) // CRC16高字节
        {
            rs485_rx_frame.crc <<= 8;
            rs485_rx_frame.crc |= u8_data;

            if(rs485_rx_frame.crc != CRC16_MODBUS(rs485_rx_buffer, 6 + rs485_rx_frame.data_len)) // CRC校验不通过
            {
                rs485_rx_pos = 0;
                return;
            }

            // CRC校验通过, rx_frame合法
            rs485_rx_pos = 0;
            rs485_rx_frame.data = &rs485_rx_buffer[6];
            bms_store_rs485_frame(&rs485_rx_frame);
            return;
        }
    }

    rs485_rx_buffer[rs485_rx_pos++] = u8_data; // 前面2个字节是固定的"I" "G"，因此没有修改，除此之外，帧内的所有字节被记录在对应的位置上
    if(rs485_rx_pos == BMS_RS485_RX_BUFFER_MAX - 1)
        rs485_rx_pos = 0;
}


/// @brief 获取头部第一个接收节点
/// @return 
bms_rx_node_t* get_wire_oldest_rx_node()
{
    return list_entry(_rx_node_head.entry.next, bms_rx_node_t, entry);
}

/// @brief BMS接收从机的响应帧
/// @param rx_node 帧节点
void bms_response_slave(bms_rx_node_t* rx_node)
{
    uint8_t transmitter_id = rx_node->buffer_ptr[2]; // 发送方ID
	uint8_t cmd_type = rx_node->buffer_ptr[4] & (~0x80); 
    uint8_t data_len = rx_node->buffer_ptr[5]; 
	uint8_t* data = &rx_node->buffer_ptr[6];

    rs485_tx_buffer[0] = RS485_FRAME_MAGIC_BYTE1;
    rs485_tx_buffer[1] = RS485_FRAME_MAGIC_BYTE2;
    rs485_tx_buffer[2] = BMS_ID; // 发送方ID，BMS主板
    rs485_tx_buffer[3] = transmitter_id; // 发送方作为接收方
    rs485_tx_buffer[4] = (uint8_t)cmd_type | 0x80; // 最高位为1表示响应

    slave_node_t* ptr;
    uint8_t* allocate_slave_id_buffer;
    uint8_t new_node = 1;
    slave_config_t* slave_cfg_ptr = get_slave_cfg();
    uint32_t* bms_know_slave_cfg_ptr = get_bms_know_flag();

    switch (cmd_type)
    {
    case Active: 
        /* BMS主机会在启动后轮询从机是否存在，具体做法是向ID为0x0A~0x20的从机发送 Active 命令，当BMS收到此命令响应后，即认为对应的从机在线 */
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if((ptr->slave_id == transmitter_id) || (ptr->slave_id == 0)) // 添加到BMS主机的从机链表
                new_node = 0;
        }

        if(new_node == 1)
        {
            allocate_slave_id_buffer = AllocateBlock(_bms_slave_mempool);
            if(!allocate_slave_id_buffer)
                break;

            slave_node_t* new_slave_node = (slave_node_t*)allocate_slave_id_buffer; // 分配空间存储节点
            new_slave_node->slave_id = transmitter_id; // 记录节点ID
            new_slave_node->slave_st.chip_temp = BMS_SLAVE_STATE_NOT_FETCH_FLAG; /* 将0xff作为一个标志，表明当前的从机状态还没有更新 */

            list_add(&(new_slave_node->entry) , &_slave_node_head.entry); // 添加节点
        }
        break;

    case Read_Pack_Config: // 从机回复BMS主机 Read_Pack_Config 命令，这样BMS主机就能向上位机返回从机配置
        memcpy(slave_cfg_ptr, data, sizeof(slave_config_t)); // BMS主机存储从机配置信息
        *bms_know_slave_cfg_ptr = BMS_KNOW_ALL; // BMS主机知道全部信息
        break;

    case Read_Cell_Type: // 读取电芯类型
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                if(((*bms_know_slave_cfg_ptr) & BMS_KNOW_CELL_TYPE) == BMS_KNOW_NOTHING)
                {
                    slave_cfg_ptr->cell_type = data[0]; // 记录节点电芯类型
                    (*bms_know_slave_cfg_ptr) |= BMS_KNOW_CELL_TYPE; /* BMS了解了电芯类型 */
                }
            }
        }
        break;

    case Read_Cell_Code_String: // 读取电芯型号
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                if(((*bms_know_slave_cfg_ptr) & BMS_KNOW_CELL_CODE) == BMS_KNOW_NOTHING)
                {
                    memcpy(slave_cfg_ptr->cell_code_string, &data[0], data_len); // 记录电芯型号
                    (*bms_know_slave_cfg_ptr) |= BMS_KNOW_CELL_CODE; /* BMS了解了电芯代号 */
                }
            }
        }
        break;

    case Read_Pack_Code_String: // 读取电池代号
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                if(((*bms_know_slave_cfg_ptr) & BMS_KNOW_PACK_CODE) == BMS_KNOW_NOTHING)
                {
                    memcpy(slave_cfg_ptr->pack_code_string, &data[0], data_len); // 记录电池型号
                    (*bms_know_slave_cfg_ptr) |= BMS_KNOW_PACK_CODE; /* BMS了解了电池代号 */
                }
            }
        }
        break;

    case Read_Cell_Serial_Count: // 读取电芯数量串数
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                if(((*bms_know_slave_cfg_ptr) & BMS_KNOW_CELL_S) == BMS_KNOW_NOTHING)
                {
                    slave_cfg_ptr->cell_serial_count = data[0];
                    (*bms_know_slave_cfg_ptr) |= BMS_KNOW_CELL_S; /* BMS了解了电池包串数 */
                }
            }
        }
        break;

    case Read_Pack_Capacity: // 读取电池容量
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                if(((*bms_know_slave_cfg_ptr) & BMS_KNOW_PACK_CAP) == BMS_KNOW_NOTHING)
                {
                    slave_cfg_ptr->pack_capacity = data[0]; 
                    (*bms_know_slave_cfg_ptr) |= BMS_KNOW_PACK_CAP; /* BMS了解了电池包容量 */
                }
            }
        }
        break;

    case Read_CMU_Version: // 读取 硬件版本[7:4]，软件版本[3:0]
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                if(((*bms_know_slave_cfg_ptr) & BMS_KNOW_CMU_VERSION) == BMS_KNOW_NOTHING)
                {
                    slave_cfg_ptr->cmu_hardware_version = data[0] >> 4;
                    slave_cfg_ptr->cmu_software_version = data[0] & 0x0f;
                    (*bms_know_slave_cfg_ptr) |= BMS_KNOW_CMU_VERSION; /* BMS了解了从机版本 */
                }
            }
        }
        break;

    case Read_CMU_AFE_Code_String: // 读取AFE型号
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                if(((*bms_know_slave_cfg_ptr) & BMS_KNOW_AFE_CODE) == BMS_KNOW_NOTHING)
                {
                    memcpy(slave_cfg_ptr->cmu_afe_code_string, &data[0], data_len); // 记录AFE型号
                    (*bms_know_slave_cfg_ptr) |= BMS_KNOW_AFE_CODE; /* BMS了解了AFE型号 */
                }
            }
        }
        break;
    
    case Read_NTC_Info: // 读取NTC个数和NTC型号
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                if(((*bms_know_slave_cfg_ptr) & BMS_KNOW_NTC_INFO) == BMS_KNOW_NOTHING)
                {
                    slave_cfg_ptr->ntc_count = data[0];
                    memcpy(slave_cfg_ptr->ntc_code, &data[1], data_len - 1);
                    (*bms_know_slave_cfg_ptr) |= BMS_KNOW_NTC_INFO; /* BMS了解了NTC信息 */
                }
            }
        }
        break;

    case Read_Pack_State: /* 收到从机对当前状态的报告 */
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                /* 电芯电压 */
                for (uint8_t i = 0; i < slave_cfg_ptr->cell_serial_count; i++)
                {
                    ptr->slave_st.cmu_board_cell_voltage[i] = data[2 * i];
                    ptr->slave_st.cmu_board_cell_voltage[i] <<= 8;
                    ptr->slave_st.cmu_board_cell_voltage[i] |= data[2 * i + 1];
                }

                /* NTC温度 */
                for (uint8_t i = 0; i < slave_cfg_ptr->ntc_count; i++)
                {
                    ptr->slave_st.cmu_board_ntc_temp_result[i] = data[2 * slave_cfg_ptr->cell_serial_count + 2 * i];
                    ptr->slave_st.cmu_board_ntc_temp_result[i] <<= 8;
                    ptr->slave_st.cmu_board_ntc_temp_result[i] |= data[2 * slave_cfg_ptr->cell_serial_count + 2 * i + 1];
                }
                
                /* 均衡状态 */
                ptr->slave_st.MT9805_DCC = data[2 * slave_cfg_ptr->cell_serial_count + 2 * slave_cfg_ptr->ntc_count];
                ptr->slave_st.MT9805_DCC <<= 8;
                ptr->slave_st.MT9805_DCC |= data[2 * slave_cfg_ptr->cell_serial_count + 2 * slave_cfg_ptr->ntc_count + 1];
                ptr->slave_st.MT9805_DCC <<= 8;
                ptr->slave_st.MT9805_DCC |= data[2 * slave_cfg_ptr->cell_serial_count + 2 * slave_cfg_ptr->ntc_count + 2];

                ptr->slave_st.chip_temp = 0x18;
            }
        }
        break;

    case Read_Balance_State: // 读取均衡状态
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                ptr->slave_st.MT9805_DCC = data[0];
                ptr->slave_st.MT9805_DCC <<= 8;
                ptr->slave_st.MT9805_DCC |= data[1];
                ptr->slave_st.MT9805_DCC <<= 8;
                ptr->slave_st.MT9805_DCC |= data[2];
            }
        }
        break;

    case Read_all_cell_Voltage: // 读取PACK所有电芯电压
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                for (uint8_t i = 0; i < slave_cfg_ptr->cell_serial_count; i++)
                {
                    ptr->slave_st.cmu_board_cell_voltage[i] = data[2 * i];
                    ptr->slave_st.cmu_board_cell_voltage[i] <<= 8;
                    ptr->slave_st.cmu_board_cell_voltage[i] |= data[2 * i + 1];
                }
                bms_prepare_balance(transmitter_id, &(ptr->slave_st)); /* 如果达到均衡条件，向从机发出指令进行均衡 */
            }
        }
        break;
    
    case Read_all_NTC_Temp: // 读取PACK所有NTC温度值
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                for (uint8_t i = 0; i < slave_cfg_ptr->ntc_count; i++)
                {
                    ptr->slave_st.cmu_board_ntc_temp_result[i] = data[2 * i];
                    ptr->slave_st.cmu_board_ntc_temp_result[i] <<= 8;
                    ptr->slave_st.cmu_board_ntc_temp_result[i] |= data[2 * i + 1];
                }
            }
        }
        break;

    case Read_cell_OV_UV_status: // 读取PACK中所有电芯的过压欠压状态
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                ptr->slave_st.LiPo_OV_Status = data[0];
                ptr->slave_st.LiPo_OV_Status <<= 8;
                ptr->slave_st.LiPo_OV_Status |= data[1];
                ptr->slave_st.LiPo_OV_Status <<= 8;
                ptr->slave_st.LiPo_OV_Status |= data[2];

                ptr->slave_st.LiPo_UV_Status = data[3];
                ptr->slave_st.LiPo_UV_Status <<= 8;
                ptr->slave_st.LiPo_UV_Status |= data[4];
                ptr->slave_st.LiPo_UV_Status <<= 8;
                ptr->slave_st.LiPo_UV_Status |= data[5];
            }
        }
        break;

    case Read_Pack_Voltage: // 读取PACK总压
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                ptr->slave_st.pack_volt = data[0];
                ptr->slave_st.pack_volt <<= 8;
                ptr->slave_st.pack_volt |= data[1];
            }
        }
        break;

    case Read_ChipTEMP: // 读取AFE芯片温度
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                ptr->slave_st.chip_temp = data[0];
            }
        }
        break;

    case Open_circuit_inspection: // 读取PACK电压检测线开路状态
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == transmitter_id)
            {
                ptr->slave_st.open_wire = data[0];
                ptr->slave_st.open_wire <<= 8;
                ptr->slave_st.open_wire |= data[1];
                ptr->slave_st.open_wire <<= 8;
                ptr->slave_st.open_wire |= data[2];
            }
        }
        break;
    
    default:
        break;
    }

	list_del(&rx_node->entry); // 删除节点
	rx_node->buffer_ptr = NULL;
	FreeBlock(_bms_rs485_mempool, rx_node);
}


/// @brief BMS响应上位机的命令，向其发送数据帧
/// @param rx_node 来自上位机的命令帧节点
void bms_response_host(bms_rx_node_t* rx_node)
{
    uint8_t transmitter_id = rx_node->buffer_ptr[2]; // 发送方ID
	uint8_t cmd_type = rx_node->buffer_ptr[4]; // 命令 
	uint8_t* data = &rx_node->buffer_ptr[6];
    slave_node_t* ptr = NULL;
    bms_config_t* bms_cfg = get_bms_config();

    host_uart_tx_buffer[0] = RS485_FRAME_MAGIC_BYTE1;
    host_uart_tx_buffer[1] = RS485_FRAME_MAGIC_BYTE2;
    host_uart_tx_buffer[2] = BMS_ID; // 发送方ID，BMS主板
    host_uart_tx_buffer[3] = HOST_ID; // 发送方作为接收方
    host_uart_tx_buffer[4] = (uint8_t)cmd_type | 0x80; // 最高位为1表示响应

    uint8_t* tx_frame;
    uint16_t crc16_rslt;
    uint8_t index;

    slave_config_t* slave_cfg_ptr = get_slave_cfg();
    bms_status_t* bms_st_ptr = read_bms_status();

    switch (cmd_type)
    {
    case Active: // 上位机向BMS发送 Active 命令，BMS回复表示自己在线
        tx_frame = bms_prepare_host_uart_tx_frame(transmitter_id, Active, 0, NULL); /* 准备向上位机回复 */
        if(tx_frame)
            host_uart_transmit(tx_frame, BMS_FRAME_NO_DATA_LEN);
        break;

    case Read_BMS_Slave_Count: // 上位机向BMS主机发送 Read_BMS_Slave_Count 命令，主机回复当前有多少个从机，以及每个从机的ID
        index = 0;
        list_for_each_entry(ptr, &(_slave_node_head.entry), entry)
        {
            host_uart_tx_buffer[7 + index] = ptr->slave_id; /* 从机ID */
            index++;
        }
        host_uart_tx_buffer[6] = index; /* 从机数量 */
        host_uart_tx_buffer[5] = 9 + index - BMS_FRAME_NO_DATA_LEN; /* 回复字节长度 */
        
        crc16_rslt = CRC16_MODBUS(host_uart_tx_buffer, 7 + index);
        host_uart_tx_buffer[7 + index] = crc16_rslt >> 8;
        host_uart_tx_buffer[8 + index] = crc16_rslt & 0xff;

        host_uart_transmit(host_uart_tx_buffer, 9 + index);
        break; 

    case Read_BMS_Config: /* 读取BMS配置 */
        host_uart_tx_buffer[5] = sizeof(bms_config_t); /* 载荷长度 */

        /* BMS主机配置 */
        memcpy(&host_uart_tx_buffer[6], bms_cfg, sizeof(bms_config_t));
        crc16_rslt = CRC16_MODBUS(host_uart_tx_buffer, sizeof(bms_config_t) + 6);

        host_uart_tx_buffer[sizeof(bms_config_t) + 6] = crc16_rslt >> 8;
        host_uart_tx_buffer[sizeof(bms_config_t) + 7] = crc16_rslt & 0xff;

        host_uart_transmit(host_uart_tx_buffer, sizeof(bms_config_t) + BMS_FRAME_NO_DATA_LEN);
        break;

    case Write_BMS_Config:
        if(rx_node->buffer_ptr[5] != sizeof(bms_config_t)) /* 需要数据长度正确 */
            break;

        memcpy(bms_cfg, data, sizeof(bms_config_t)); /* 更新BMS配置 */
        app_changed_bms_config(); /* 上位机修改BMS主板配置参数 */

        host_uart_tx_buffer[5] = 0; /* 载荷长度 */

        /* BMS主机向上位机回复配置已修改，目前是在RAM中修改，等到下一次后台任务再修改Flash */
        crc16_rslt = CRC16_MODBUS(host_uart_tx_buffer, 6);

        host_uart_tx_buffer[6] = crc16_rslt >> 8;
        host_uart_tx_buffer[7] = crc16_rslt & 0xff;

        host_uart_transmit(host_uart_tx_buffer, BMS_FRAME_NO_DATA_LEN);
        break;

    case Restore_BMS_Config:
        bms_init_flash_config();
        break;

    case Read_Pack_Config:
        /* BMS从机配置 */
        host_uart_tx_buffer[5] = 5;
        host_uart_tx_buffer[6] = slave_cfg_ptr->cell_type;
        host_uart_tx_buffer[7] = slave_cfg_ptr->cell_serial_count;
        host_uart_tx_buffer[8] = slave_cfg_ptr->pack_capacity;
        host_uart_tx_buffer[9] = slave_cfg_ptr->cmu_hardware_version;
        host_uart_tx_buffer[9] <<= 4;
        host_uart_tx_buffer[9] |= slave_cfg_ptr->cmu_software_version;
        host_uart_tx_buffer[10] = slave_cfg_ptr->ntc_count;

        // memcpy(&host_uart_tx_buffer[11], &slave_cfg.ntc_code, sizeof(slave_cfg.ntc_code)); // 将从机配置信息发送给上位机
        // memcpy(&host_uart_tx_buffer[27], &slave_cfg.cell_code_string, sizeof(slave_cfg.cell_code_string));
        // memcpy(&host_uart_tx_buffer[59], &slave_cfg.cell_source, sizeof(slave_cfg.cell_source));
        // memcpy(&host_uart_tx_buffer[75], &slave_cfg.pack_code_string, sizeof(slave_cfg.pack_code_string));
        // memcpy(&host_uart_tx_buffer[91], &slave_cfg.cmu_afe_code_string, sizeof(slave_cfg.cmu_afe_code_string));

        crc16_rslt = CRC16_MODBUS(host_uart_tx_buffer, 11);
        host_uart_tx_buffer[11] = crc16_rslt >> 8;
        host_uart_tx_buffer[12] = crc16_rslt & 0xff;
        host_uart_transmit(host_uart_tx_buffer, 13);

        break;

    case Read_Pack_State:
        /* 如果上位机询问当前BMS的从机ID，返回对应从机状态 */
        list_for_each_entry(ptr, &_slave_node_head.entry, entry)
        {
            if(ptr->slave_id == data[0]) // 载荷字段第一个字节是上位机感兴趣的从机ID
            {
                host_uart_tx_buffer[5] = 48; // 有效载荷48字节，包括1字节ID，36字节电压，8字节温度，3字节均衡
                host_uart_tx_buffer[6] = data[0]; // 感兴趣从机ID

                /* 填充电芯电压，单位mV */
                for (uint8_t i = 0; i < SLAVE_CELL_SERIAL_COUNT; i++)
                {
                    host_uart_tx_buffer[7 + 2 * i] = ptr->slave_st.cmu_board_cell_voltage[i] >> 8;
                    host_uart_tx_buffer[7 + 2 * i + 1] = ptr->slave_st.cmu_board_cell_voltage[i] & 0xff;
                }
                /* 填充NTC温度 */
                for (uint8_t i = 0; i < SLAVE_NTC_NUM; i++)
                {
                    host_uart_tx_buffer[43 + 2 * i] = ptr->slave_st.cmu_board_ntc_temp_result[i] >> 8;
                    host_uart_tx_buffer[43 + 2 * i + 1] = ptr->slave_st.cmu_board_ntc_temp_result[i] & 0xff;
                }
                /* 填充均衡状态 */
                host_uart_tx_buffer[51] = ptr->slave_st.MT9805_DCC >> 16;
                host_uart_tx_buffer[52] = ptr->slave_st.MT9805_DCC >> 8;
                host_uart_tx_buffer[53] = ptr->slave_st.MT9805_DCC & 0xff;

                crc16_rslt = CRC16_MODBUS(host_uart_tx_buffer, 54);
                
                host_uart_tx_buffer[54] = crc16_rslt >> 8;
                host_uart_tx_buffer[55] = crc16_rslt & 0xff;

                host_uart_transmit(host_uart_tx_buffer, 56);
            }
        }
        break;

    case Read_BMS_State: /* 上位机向BMS主机询问其状态，BMS主机返回 bms_status_t 结构体 */
        host_uart_tx_buffer[5] = sizeof(bms_status_t); // 发送电池电压使用2字节，单位V
        memcpy(&host_uart_tx_buffer[6], bms_st_ptr, sizeof(bms_status_t));

        crc16_rslt = CRC16_MODBUS(host_uart_tx_buffer, 6 + sizeof(bms_status_t));

        host_uart_tx_buffer[6 + sizeof(bms_status_t)] = crc16_rslt >> 8;
        host_uart_tx_buffer[7 + sizeof(bms_status_t)] = crc16_rslt & 0xff;
        host_uart_transmit(host_uart_tx_buffer, BMS_FRAME_NO_DATA_LEN + sizeof(bms_status_t));

        break;

    case Read_All_Bat_Volt: // 上位机向BMS发送 Read_All_Bat_Volt 命令，BMS回复当前电池包串联起来后的总电压
        host_uart_tx_buffer[5] = 2; // 发送电池电压使用2字节，单位V
        host_uart_tx_buffer[6] = bms_st_ptr->voltage >> 8;
        host_uart_tx_buffer[7] = bms_st_ptr->voltage & 0xff;
        crc16_rslt = CRC16_MODBUS(host_uart_tx_buffer, 8);
        host_uart_tx_buffer[8] = crc16_rslt >> 8;
        host_uart_tx_buffer[9] = crc16_rslt & 0xff;

        host_uart_transmit(host_uart_tx_buffer, BMS_FRAME_NO_DATA_LEN + 2);
        break;

    case Read_Power_Current: // 上位机向BMS发送 Read_Power_Current 命令，BMS回复当前电池包充电/放电电流
        host_uart_tx_buffer[5] = 2; // 发送电池动力电流使用2字节，单位A
        host_uart_tx_buffer[6] = bms_st_ptr->power_current_A >> 8;
        host_uart_tx_buffer[7] = bms_st_ptr->power_current_A & 0xff;
        crc16_rslt = CRC16_MODBUS(host_uart_tx_buffer, 8);
        host_uart_tx_buffer[8] = crc16_rslt >> 8;
        host_uart_tx_buffer[9] = crc16_rslt & 0xff;

        host_uart_transmit(host_uart_tx_buffer, BMS_FRAME_NO_DATA_LEN + 2);
        break;

    case Read_BMS_NTC_Temp: // 上位机向BMS发送 Read_BMS_NTC_Temp 命令，BMS回复当前BMS主板上的两个NTC温度
        host_uart_tx_buffer[5] = 2; // 发送BMS NTC温度使用2字节，单位℃
        host_uart_tx_buffer[6] = bms_st_ptr->NTC1_temp;
        host_uart_tx_buffer[7] = bms_st_ptr->NTC2_temp;
        crc16_rslt = CRC16_MODBUS(host_uart_tx_buffer, 8);
        host_uart_tx_buffer[8] = crc16_rslt >> 8;
        host_uart_tx_buffer[9] = crc16_rslt & 0xff;

        host_uart_transmit(host_uart_tx_buffer, BMS_FRAME_NO_DATA_LEN + 2);
        break;

    case Control_PreCharge: // 上位机向BMS发送 Control_PreCharge 命令，BMS控制预充开关
        if(data[0] == start)
            bms_precharge(start);
        else if(data[0] == stop)
            bms_precharge(stop);

        host_uart_tx_buffer[5] = 1; 
        host_uart_tx_buffer[6] = data[0]; /* 回复的状态 */
        crc16_rslt = CRC16_MODBUS(host_uart_tx_buffer, 7);
        host_uart_tx_buffer[7] = crc16_rslt >> 8;
        host_uart_tx_buffer[8] = crc16_rslt & 0xff;

        host_uart_transmit(host_uart_tx_buffer, BMS_FRAME_NO_DATA_LEN + 1); /* BMS向上位机回复，表示接收到控制预充开关的命令 */
        break;

    case Control_ChargeDischarge: // 上位机向BMS发送 Control_ChargeDischarge 命令，BMS控制充电/放电开关
        if(data[0] == start)
            bms_charge_discharge(start);
        else if(data[0] == stop)
            bms_charge_discharge(stop);

        host_uart_tx_buffer[5] = 0; 
        host_uart_tx_buffer[6] = data[0]; /* 回复的状态 */
        crc16_rslt = CRC16_MODBUS(host_uart_tx_buffer, 7);
        host_uart_tx_buffer[7] = crc16_rslt >> 8;
        host_uart_tx_buffer[8] = crc16_rslt & 0xff;

        host_uart_transmit(host_uart_tx_buffer, BMS_FRAME_NO_DATA_LEN + 1); /* BMS向上位机回复，表示接收到控制充电/放电开关的命令 */
        break;
    
    case Report_BMS_Error: // 上位机向BMS发送 Report_BMS_Error 命令，BMS回复当前的异常
        host_uart_tx_buffer[5] = 2; // 发送异常状态使用2字节
        host_uart_tx_buffer[6] = bms_st_ptr->error_state >> 8;
        host_uart_tx_buffer[7] = bms_st_ptr->error_state & 0xff;

        crc16_rslt = CRC16_MODBUS(host_uart_tx_buffer, 8);
        host_uart_tx_buffer[8] = crc16_rslt >> 8;
        host_uart_tx_buffer[9] = crc16_rslt & 0xff;

        host_uart_transmit(host_uart_tx_buffer, BMS_FRAME_NO_DATA_LEN + 2);
        break;
    
    default:
        break;
    }

	list_del(&rx_node->entry); // 删除节点
	rx_node->buffer_ptr = NULL;
	FreeBlock(_bms_host_mempool, rx_node);
}

/// @brief 准备RS485数据帧，发送给从机
/// @param target_id 
/// @param cmd_type 
/// @param len 
/// @param payload 
/// @return 
uint8_t* bms_prepare_rs485_tx_frame(uint8_t target_id, bms_cmd_type_t cmd_type, uint8_t len, uint8_t* payload)
{
    rs485_tx_buffer[2] = BMS_ID;
    rs485_tx_buffer[3] = target_id;
    rs485_tx_buffer[4] = cmd_type;
    rs485_tx_buffer[5] = len;

    if(payload && len > 0)
        memcpy(&rs485_tx_buffer[6], payload, len); // 有效数据

    uint16_t crc_rslt = CRC16_MODBUS(rs485_tx_buffer, 6 + len); // 计算校验

    rs485_tx_buffer[6 + len] = crc_rslt >> 8;
    rs485_tx_buffer[6 + len + 1] = crc_rslt & 0xff;

    return rs485_tx_buffer;
}

/// @brief 准备UART数据帧，发送给上位机
/// @param target_id 
/// @param cmd_type 
/// @param len 
/// @param payload 
/// @return 
uint8_t* bms_prepare_host_uart_tx_frame(uint8_t target_id, bms_cmd_type_t cmd_type, uint8_t len, uint8_t* payload)
{
    host_uart_tx_buffer[2] = BMS_ID;
    host_uart_tx_buffer[3] = target_id;
    host_uart_tx_buffer[4] = cmd_type;
    host_uart_tx_buffer[5] = len;

    if(payload && len > 0)
        memcpy(&host_uart_tx_buffer[6], payload, len); // 有效数据

    uint16_t crc_rslt = CRC16_MODBUS(host_uart_tx_buffer, 6 + len); // 计算校验

    host_uart_tx_buffer[6 + len] = crc_rslt >> 8;
    host_uart_tx_buffer[6 + len + 1] = crc_rslt & 0xff;

    return host_uart_tx_buffer;
}

slave_node_t* get_slave_list_head()
{
    return &_slave_node_head;
}

/// @brief 用于RS485的堆内存，已使用空间
/// @return 
uint16_t rs485_heap_used()
{
    return _bms_rs485_mempool->usedCount * (BMS_RS485_RX_BUFFER_MAX + sizeof(bms_rx_node_t));
}

/// @brief 用于上位机通信的堆内存已使用空间
/// @return 
uint16_t host_heap_used()
{
    return _bms_host_mempool->usedCount * (BMS_HOST_RX_BUFFER_MAX + sizeof(bms_rx_node_t));
}

/// @brief 用于存储从机信息的堆内存使用空间
/// @return 
uint16_t slave_heap_used()
{
    return _bms_slave_mempool->usedCount * sizeof(slave_node_t);
}
