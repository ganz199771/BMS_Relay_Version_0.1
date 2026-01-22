
#include "bms_wifi.h"
#include "string.h"
#include "stdint.h"
#include "bms.h"
#include "cycfg_peripherals.h"
#include "bms_rs485_uart.h"
#include "list.h"
#include "mempool.h"
#include "bms_config.h"

#include <xmc_usic.h>

static const char* cwmode_cmd = "AT+CWMODE=2\r\n";
static const char* cwsap_cmd = "AT+CWSAP=\"IG_BMS\",\"ig12345678\",1,3,4,0\r\n";
static const char* cipmux_cmd = "AT+CIPMUX=1\r\n";
static const char* cipserver_cmd = "AT+CIPSERVER=1,333\r\n"; // 使用端口号333，可修改
// static const char* cipap_cmd = "AT+CIPAP?\r\n";
// static const char* cipmode_cmd = "AT+CIPMODE=1\r\n"; // 进入透传模式
// static const char* esp12f_TCP_addr = "192.168.4.1";

static volatile uint8_t get_wifi_link_id = 0; // 获取到连接ID，置1表示已获取
static volatile AT_rx_state_t AT_parse_state = rx_idle;
static volatile uint8_t data_pos = 0;
static volatile uint16_t frame_crc = 0;

static uint8_t wifi_tx_buf[0x100];
static wifi_data_t wifi_rx_data;
static MemoryPool* wifi_mempool = NULL; // 存储上位机发来的命令帧
static bms_rx_node_t wifi_rx_node_head; // 来自上位机的命令帧头节点

static uint8_t ch_2_u8(char ascii)
{
    return (uint8_t)(ascii - '0');
}

static void store_wifi_frame(wifi_data_t* data)
{
    uint8_t* rx_buffer = AllocateBlock(wifi_mempool);
    if(!rx_buffer)
    {
        return;
    }
    
    XMC_UART_CH_DisableEvent(WIFI_UART_HW, XMC_UART_CH_EVENT_STANDARD_RECEIVE);

    bms_rx_node_t* new_rx_node = (bms_rx_node_t*)rx_buffer; // 分配空间：12字节给 bms_rx_node_t 结构体，剩余字节给接收buffer
    new_rx_node->buffer_ptr = &rx_buffer[sizeof(bms_rx_node_t)]; // 设置指针指向的起始地址

    memcpy(new_rx_node->buffer_ptr, data->data_buf, BMS_FRAME_NO_DATA_LEN + data->data_buf[5]); /* 将 wifi_frame_t 数据拷贝到节点buffer */
    list_add(&new_rx_node->entry , &wifi_rx_node_head.entry);

    XMC_UART_CH_EnableEvent(WIFI_UART_HW, XMC_UART_CH_EVENT_STANDARD_RECEIVE);
}

/// @brief wifi接收中断
/// @param  __attribute__ ((section (".ram_code")))
void WIFI_UART_RECEIVE_EVENT_HANDLER(void)
{
    // TCP client 发送 Hello,ESP12F
    // ESP12F作为TCP服务器收到：+IPD,0,14:Hello,ESP12F      针对此种接收帧格式进行解析
    /* 在TCP client以16进制发送数据01 02 03 04 05 06时，TCP server也就是ESP12F的串口接收引脚收到：
        0D 0A 2B 49 50 44 2C 31 2C 36 3A 01 02 03 04 05 06 0D 0A
        0D是\r
        0A是\n
        2B是+
        49是I
        50是P
        44是D
        2C是,
        31是1
        2C是,
        36是6
        3A是:
        01, 02, 03, 04, 05, 06是传输数据
     */

    uint8_t u8_data = XMC_UART_CH_GetReceivedData(WIFI_UART_HW);

    /* 如果不知道连接ID，获取连接ID */
    if(get_wifi_link_id == 0)
    {
        switch (AT_parse_state)
        {
        case rx_idle:
            if(u8_data == '+')
                AT_parse_state = rx_pos;
            break;

        case rx_pos:
            if(u8_data == 'I')
                AT_parse_state = rx_I;
            else
                AT_parse_state = rx_idle;
            break;

        case rx_I:
            if(u8_data == 'P')
                AT_parse_state = rx_P;
            else
                AT_parse_state = rx_idle;
            break;

        case rx_P:
            if(u8_data == 'D')
                AT_parse_state = rx_D;
            else
                AT_parse_state = rx_idle;
            break;

        case rx_D:
            if(u8_data == ',')
                AT_parse_state = rx_first_delim;
            else
                AT_parse_state = rx_idle;
            break;

        case rx_first_delim:
            wifi_rx_data.id = ch_2_u8(u8_data);
            AT_parse_state = rx_idle;
            get_wifi_link_id = 1;

            break;
        default:
            break;
        }
    }

    /* 知道了连接ID，直接抽取帧 */
    if(data_pos == 0 && u8_data != RS485_FRAME_MAGIC_BYTE1) // 头字节1
    {
        return;
    }

    else if(data_pos == 1 && u8_data != RS485_FRAME_MAGIC_BYTE2) // 头字节2
    {
        data_pos = 0;
        return;
    }

    else if(data_pos == 2) // 发送方id
    {
        if(HOST_ID != u8_data)
        {
            data_pos = 0;
            return;
        }
        wifi_rx_data.data_buf[2] = u8_data;
    }

    else if(data_pos == 3) // 接收方id
    {
        if(BMS_ID != u8_data)
        {
            data_pos = 0;
            return;
        }
        wifi_rx_data.data_buf[3] = u8_data;
    }

    else if(data_pos == 4) // 命令
    {
        wifi_rx_data.data_buf[4] = u8_data;
    }

    else if(data_pos == 5) // 数据长度
    {
        wifi_rx_data.data_buf[5] = u8_data; 
    }

    else
    {
        if(data_pos == wifi_rx_data.data_buf[5] + 6) // CRC16低字节
        {
            frame_crc = u8_data;
        }
        else if(data_pos == wifi_rx_data.data_buf[5] + 7) // CRC16高字节
        {
            frame_crc <<= 8;
            frame_crc |= u8_data;

            if(frame_crc != CRC16_MODBUS(wifi_rx_data.data_buf, 6 + wifi_rx_data.data_buf[5])) // CRC校验不通过
            {
                data_pos = 0;
                return;
            }

            // CRC校验通过, rx_frame合法
            data_pos = 0;
            store_wifi_frame(&wifi_rx_data);
            return;
        }
    }

    wifi_rx_data.data_buf[data_pos++] = u8_data; // 前面2个字节是固定的"I" "G"，因此没有修改，除此之外，帧内的所有字节被记录在对应的位置上
    if(data_pos == (WIFI_RX_BUFFER_SIZE - 1))
        data_pos = 0;
}

void wifi_send_AT_cmd(const char* AT_cmd)
{
    if(!AT_cmd)
        return;

    for (uint8_t i = 0; i < strlen(AT_cmd); i++)
    {
        XMC_UART_CH_Transmit(WIFI_UART_HW, AT_cmd[i]);
    }
}

uint16_t wifi_heap_used()
{
    return wifi_mempool->usedCount * (sizeof(bms_rx_node_t) + WIFI_RX_BUFFER_SIZE);
}

void ESP12F_send_data(uint8_t* data, uint8_t len)
{
    /* 第一步，先发送 AT+CIPSEND=<link ID>,<length> */   
    if(!data || len == 0)
        return;

    static char buf[0x20] = {0};
    sprintf(buf, "AT+CIPSEND=%d,%d\r\n", wifi_rx_data.id, len);

    for (uint8_t i = 0; i < strlen(buf); i++)
    {
        XMC_UART_CH_Transmit(WIFI_UART_HW, buf[i]);
    }
    XMC_Delay(5); /* 延时 */

    /* 第二步，发送数据 */
    for (uint8_t i = 0; i < len; i++)
    {
        XMC_UART_CH_Transmit(WIFI_UART_HW, data[i]);
    }
}

void bms_wifi_init(void)
{
    AT_parse_state = rx_idle;

    INIT_LIST_HEAD(&wifi_rx_node_head.entry); // 初始化接收链表
    wifi_rx_node_head.buffer_ptr = NULL;

    wifi_mempool = InitMemoryPool(sizeof(bms_rx_node_t) + WIFI_RX_BUFFER_SIZE, WIFI_RX_NODE_MAX);
    if(!wifi_mempool)
        return;

    wifi_tx_buf[0] = MAGIC_BYTE1;
    wifi_tx_buf[1] = MAGIC_BYTE2;
    wifi_tx_buf[2] = BMS_ID;

    XMC_Delay(500);
    
    /* 第一步，发送 AT+CWMODE=2\r\n 进入softAP模式 */
    wifi_send_AT_cmd(cwmode_cmd);
    XMC_Delay(10);

    /* 第二步，发送 AT+CWSAP="IG_BMS","ig12345678",1,3,4,0设置 AP 的参数，这样上位机可以使用WiFi连接 */
    wifi_send_AT_cmd(cwsap_cmd);
    XMC_Delay(10);

    /* 第三步，发送 AT+CIPMUX=1 使能多链接 */
    wifi_send_AT_cmd(cipmux_cmd); 
    XMC_Delay(10);

    /* 第四步，发送 AT+CIPSERVER=1 创建 TCP 服务器，默认端口为 333 */
    wifi_send_AT_cmd(cipserver_cmd); 
    XMC_Delay(10);

    debug_print("ESP12F init done\r\n");

    /* 第五步，等待数据传输 */
    XMC_Delay(5);

    NVIC_EnableIRQ(WIFI_UART_RECEIVE_EVENT_IRQN);
}

/// @brief 获取WiFi的较早的接收帧节点
/// @return 
static bms_rx_node_t* wifi_get_oldest_rx_node()
{
    return list_entry(wifi_rx_node_head.entry.next, bms_rx_node_t, entry);
}

void free_wifi_rx_node(bms_rx_node_t* rx_node)
{
    if(!rx_node)
        return;

    FreeBlock(wifi_mempool, rx_node);
}

void ESP12F_response()
{
    bms_rx_node_t* rx_node = wifi_get_oldest_rx_node();
	if(!rx_node || (rx_node->buffer_ptr == NULL))
    {
        return;
    }

    uint8_t transmitter_id = rx_node->buffer_ptr[2]; // 发送方ID
	uint8_t cmd_type = rx_node->buffer_ptr[4] ; // 命令类型
	uint8_t* data = &rx_node->buffer_ptr[6]; // 有效载荷起始位置

    wifi_tx_buf[3] = transmitter_id;
    wifi_tx_buf[4] = cmd_type | RESPONSE_FLAG;

    bms_status_t* bms_st = read_bms_status();
    bms_config_t* bms_cfg = get_bms_config();

    slave_node_t* slave_head = get_slave_list_head();
    slave_node_t* ptr;
    slave_status_t* slave_st;
    slave_config_t* slave_cfg = NULL;

    uint16_t crc_rslt;
    uint8_t slave_id;
    uint8_t index = 0;
    uint32_t* bms_know_flag_ptr = get_bms_know_flag();

    switch (cmd_type)
    {
    case Active:
        wifi_tx_buf[5] = 0;
        crc_rslt = CRC16_MODBUS(wifi_tx_buf, 6);

        wifi_tx_buf[6] = crc_rslt >> 8;
        wifi_tx_buf[7] = crc_rslt & 0xff;
        ESP12F_send_data(wifi_tx_buf, WIFI_FRAME_NO_DATA_LEN);
        break;

    case Read_BMS_Slave_Count: /* 读取BMS从机数量和每个从机的ID */
        index = 0;
        list_for_each_entry(ptr, &(slave_head->entry), entry)
        {
            wifi_tx_buf[7 + index] = ptr->slave_id; /* 从机ID */
            index++;
        }
        wifi_tx_buf[6] = index; /* 从机数量 */
        wifi_tx_buf[5] = 9 + index - BMS_FRAME_NO_DATA_LEN; /* 回复字节长度 */

        crc_rslt = CRC16_MODBUS(wifi_tx_buf, 7 + index);
        wifi_tx_buf[7 + index] = crc_rslt >> 8;
        wifi_tx_buf[8 + index] = crc_rslt & 0xff;

        ESP12F_send_data(wifi_tx_buf, 9 + index);
        break;

    case Read_BMS_Config: /* 读取BMS配置 */

        wifi_tx_buf[5] = sizeof(bms_config_t); /* 载荷长度 */
        memcpy(&wifi_tx_buf[6], bms_cfg, sizeof(bms_config_t));
        crc_rslt = CRC16_MODBUS(wifi_tx_buf, sizeof(bms_config_t) + 6);

        wifi_tx_buf[sizeof(bms_config_t) + 6] = crc_rslt >> 8;
        wifi_tx_buf[sizeof(bms_config_t) + 7] = crc_rslt & 0xff;

        ESP12F_send_data(wifi_tx_buf, sizeof(bms_config_t) + 8);
        break;

    case Write_BMS_Config:
        if(rx_node->buffer_ptr[5] != sizeof(bms_config_t)) /* 需要数据长度正确 */
            break;
        
        memcpy(bms_cfg, data, sizeof(bms_config_t)); /* 更新BMS配置 */
        app_changed_bms_config();
        break;

    case Restore_BMS_Config:
        bms_init_flash_config();
        break;

    case Read_Pack_Config:
        if((*bms_know_flag_ptr) != BMS_KNOW_ALL) /* 当BMS主机没有了解从机配置时，不向上位机返回从机配置 */
            break;

        wifi_tx_buf[5] = sizeof(slave_config_t);
        slave_cfg = get_slave_cfg();
        memcpy(&wifi_tx_buf[6], slave_cfg, sizeof(slave_config_t));
        crc_rslt = CRC16_MODBUS(wifi_tx_buf, sizeof(slave_config_t) + 6);

        wifi_tx_buf[sizeof(slave_config_t) + 6] = crc_rslt >> 8;
        wifi_tx_buf[sizeof(slave_config_t) + 7] = crc_rslt & 0xff;

        ESP12F_send_data(wifi_tx_buf, sizeof(slave_config_t) + 8);
        break;

    case Read_Pack_State: /* 上位机读取特定PACK状态 */
        slave_id = data[0]; /* 第一个字节表示从机ID */
        slave_st = read_slave_status(slave_id);

        wifi_tx_buf[5] = 48; // 有效载荷48字节，包括1字节ID，36字节电压，8字节温度，3字节均衡
        wifi_tx_buf[6] = data[0]; // 感兴趣从机ID

        /* 填充电芯电压，单位mV */
        for (index = 0; index < SLAVE_CELL_SERIAL_COUNT; index++)
        {
            wifi_tx_buf[7 + 2 * index] = slave_st->cmu_board_cell_voltage[index] >> 8;
            wifi_tx_buf[7 + 2 * index + 1] = slave_st->cmu_board_cell_voltage[index] & 0xff;
        }
        /* 填充NTC温度 */
        for (index = 0; index < SLAVE_NTC_NUM; index++)
        {
            wifi_tx_buf[43 + 2 * index] = slave_st->cmu_board_ntc_temp_result[index] >> 8;
            wifi_tx_buf[43 + 2 * index + 1] = slave_st->cmu_board_ntc_temp_result[index] & 0xff;
        }
        /* 填充均衡状态 */
        wifi_tx_buf[51] = slave_st->MT9805_DCC >> 16;
        wifi_tx_buf[52] = slave_st->MT9805_DCC >> 8;
        wifi_tx_buf[53] = slave_st->MT9805_DCC & 0xff;

        crc_rslt = CRC16_MODBUS(wifi_tx_buf, 54);
                
        wifi_tx_buf[54] = crc_rslt >> 8;
        wifi_tx_buf[55] = crc_rslt & 0xff;

        ESP12F_send_data(wifi_tx_buf, 56);
        break;

    case Read_BMS_State: /* 上位机向BMS主机询问其状态，BMS主机返回 bms_status_t 结构体 */
        wifi_tx_buf[5] = sizeof(bms_status_t); // 发送电池电压使用2字节，单位V
        memcpy(&wifi_tx_buf[6], bms_st, sizeof(bms_status_t));

        crc_rslt = CRC16_MODBUS(wifi_tx_buf, 6 + sizeof(bms_status_t));

        wifi_tx_buf[6 + sizeof(bms_status_t)] = crc_rslt >> 8;
        wifi_tx_buf[7 + sizeof(bms_status_t)] = crc_rslt & 0xff;
        ESP12F_send_data(wifi_tx_buf, BMS_FRAME_NO_DATA_LEN + sizeof(bms_status_t));

        break;

    case Read_All_Bat_Volt: /* 上位机读取电池总压 */
        wifi_tx_buf[5] = 2;
        wifi_tx_buf[6] = bms_st->voltage >> 8;
        wifi_tx_buf[7] = bms_st->voltage & 0xff;

        crc_rslt = CRC16_MODBUS(wifi_tx_buf, 8);

        wifi_tx_buf[8] = crc_rslt >> 8;
        wifi_tx_buf[9] = crc_rslt & 0xff;
        ESP12F_send_data(wifi_tx_buf, WIFI_FRAME_NO_DATA_LEN + 2);
        break;

    case Read_Power_Current: /* 上位机读取电池输出/输入电流 */
        wifi_tx_buf[5] = 2;
        wifi_tx_buf[6] = bms_st->power_current_A >> 8;
        wifi_tx_buf[7] = bms_st->power_current_A & 0xff;

        crc_rslt = CRC16_MODBUS(wifi_tx_buf, 8);

        wifi_tx_buf[8] = crc_rslt >> 8;
        wifi_tx_buf[9] = crc_rslt & 0xff;
        ESP12F_send_data(wifi_tx_buf, WIFI_FRAME_NO_DATA_LEN + 2);
        break;

    case Read_BMS_NTC_Temp: /* 上位机读取BMS的板载NTC温度 */
        wifi_tx_buf[5] = 2;
        wifi_tx_buf[6] = bms_st->NTC1_temp;
        wifi_tx_buf[7] = bms_st->NTC2_temp;

        crc_rslt = CRC16_MODBUS(wifi_tx_buf, 8);

        wifi_tx_buf[8] = crc_rslt >> 8;
        wifi_tx_buf[9] = crc_rslt & 0xff;
        ESP12F_send_data(wifi_tx_buf, WIFI_FRAME_NO_DATA_LEN + 2);
        break;

    case Control_PreCharge: /* 上位机控制BMS预充 */
        if(data[0] == start)
            bms_precharge(start);
        else if(data[0] == stop)
            bms_precharge(stop);
        
        wifi_tx_buf[5] = 0;
        crc_rslt = CRC16_MODBUS(wifi_tx_buf, 6);
        wifi_tx_buf[6] = crc_rslt >> 8;
        wifi_tx_buf[7] = crc_rslt & 0xff;
        ESP12F_send_data(wifi_tx_buf, WIFI_FRAME_NO_DATA_LEN);
        break;

    case Control_ChargeDischarge: /* 上位机命令BMS充电/放电 */
        if(data[0] == start)
            bms_charge_discharge(start);
        else if(data[0] == stop)
            bms_charge_discharge(stop);

        wifi_tx_buf[5] = 0;
        crc_rslt = CRC16_MODBUS(wifi_tx_buf, 6);
        wifi_tx_buf[6] = crc_rslt >> 8;
        wifi_tx_buf[7] = crc_rslt & 0xff;
        ESP12F_send_data(wifi_tx_buf, WIFI_FRAME_NO_DATA_LEN);
        break;

    case Report_BMS_Error: /* 报告BMS错误 */
        wifi_tx_buf[5] = 2;
        wifi_tx_buf[6] = bms_st->error_state >> 8;
        wifi_tx_buf[7] = bms_st->error_state & 0xff;

        crc_rslt = CRC16_MODBUS(wifi_tx_buf, 8);

        wifi_tx_buf[8] = crc_rslt >> 8;
        wifi_tx_buf[9] = crc_rslt & 0xff;
        ESP12F_send_data(wifi_tx_buf, WIFI_FRAME_NO_DATA_LEN + 2);
        break;
    
    default:
        break;
    }

    list_del(&rx_node->entry); // 删除节点
	rx_node->buffer_ptr = NULL;
	free_wifi_rx_node(rx_node);
}



