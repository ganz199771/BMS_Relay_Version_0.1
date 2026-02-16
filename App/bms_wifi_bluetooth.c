
#include "bms_wifi_bluetooth.h"
#include "string.h"
#include "stdint.h"
#include "bms.h"
#include "cycfg_peripherals.h"
#include "bms_rs485_uart.h"
#include "list.h"
#include "mempool.h"
#include "bms_config.h"

#include <xmc_usic.h>

/* ESP32-C6 设备作为 TCP 服务器，实现 Network 透传 */
/* wifi相关AT命令 */

// static const char* cwmode_cmd = "AT+CWMODE=1\r\n"; /* 设置 Wi-Fi 模式为 station */ 
// static const char* cwsap_cmd = "AT+CWSAP=\"ZOKJ\",\"ig2589088\"r\n"; /* 连接路由器，2.4GHz频段 */
// static const char* cipmux_cmd = "AT+CIPMUX=1\r\n"; /* 设置多链接模式 */
// static const char* cipserver_maxcon_cmd = "AT+CIPSERVERMAXCONN=1\r\n"; /* 设置 TCP 服务器最大连接数为 1 */
// static const char* cipserver_cmd = "AT+CIPSERVER=1,8080\r\n"; /* 使用端口号8080，可修改 */
// static const char* cipsta_cmd = "AT+CIPSTA?\r\n"; /* 获取当前模块的IP地址 */

/* MQTT相关命令 */
// static const char* mqtt_cfg_cmd = "AT+MQTTUSERCFG=0,1,\"IG-BMS-202602040001\",\"jX13D7213g\",\"WHl5RVFIRllhdEd4blpoem1RV3NqNDlEQmxXNU1JWEI=\",0,0,""\r\n";
// static const char* mqtt_connect_cmd = "AT+MQTTCONN=0,\"mqtt.heclouds.com\",1883,1\r\n";


/**
 * 1、ESP32-C6模块具备蓝牙、WiFi功能，WiFi 只支持2.4GHz
 * 2、本工程令ESP32-C6模块初始化之后，令其通过蓝牙广播，app可以通过连接蓝牙进入 AT_ble_linked 状态，此时选择蓝牙作为通信方式，并且进入透传模式
 * 3、本工程令ESP32-C6模块初始化之后，等待上位机通过WiFi连接模块的softAP网络，连接之后进入 AT_wifi_linked 状态，此时选择WiFi/UDP作为通信方式，并且进入透传模式
 */



/* ESP32-C6 设备作为 softAP 在 UDP 传输中实现 Network 透传 */
static const char* cwmode_stop_cmd = "AT+CWMODE=0\r\n"; /* 关闭WiFi */
static const char* cwmode_cmd = "AT+CWMODE=2\r\n"; /* 设置 Wi-Fi 模式为 softAP */ 
static const char* softAP_cmd = "AT+CWSAP=\"IG_BMS\",\"ig12345678\",1,3,4,0\r\n"; /* 设置 softAP */
static const char* cipstart_cmd = "AT+CIPSTART=\"UDP\",\"192.168.4.2\",1001,2233,0\r\n"; /* 连接UDP端口 */
static const char* cipmode_cmd = "AT+CIPMODE=1\r\n"; /* 进入透传模式 */
static const char* cipmode_start_send_cmd = "AT+CIPSEND\r\n"; /* 开始透传 */

/* BLE相关AT命令 */
static const char* bluetooth_stop_cmd = "AT+BLEINIT=0\r\n"; /* 关闭Bluetooth LE */
static const char* bluetooth_init_cmd = "AT+BLEINIT=2\r\n"; /* Bluetooth LE 初始化，作为server */
static const char* bluetooth_name_cmd = "AT+BLENAME=\"IG_BMS_BLE\"\r\n"; /* Bluetooth LE 设置server的名称 */
static const char* bluetooth_broadcast_params_cmd = "AT+BLEADVPARAM=50,50,0,0,7,0,,\r\n"; /* 设置广播参数，广播间隔31ms，广播类型7，公共地址，广播信道所有，对方公共地址 */
static const char* bluetooth_broadcast_data_cmd = "AT+BLEADVDATA=\"0201060A0949472d424d532d424c45030302A0\"\r\n"; /* 设置广播数据：IG-BMS-BLE */
static const char* bluetooth_broadcast_start_cmd = "AT+BLEADVSTART\"\r\n"; /* 开始广播 */
static const char* bluetooth_SPP_config_cmd = "AT+BLESPPCFG=1,1,7,1,5\r\n"; /* Bluetooth LE 服务端配置 Bluetooth LE SPP：使用服务序号1、服务特征序号7来发送数据包（手机接收）；使用服务序号1、服务特征序号5来接收数据包（手机发送） */
static const char* bluetooth_SPP_enable_cmd = "AT+BLESPP\r\n"; /* 开启蓝牙串口透传 */

static volatile bluetooth_link_state_t ble_link_st = AT_ble_idle; /* BLE连接状态 */
static volatile wifi_link_state_t wifi_link_st = AT_wifi_idle; /* wifi station连接状态 */
static volatile uint8_t data_pos = 0;
static volatile uint16_t frame_crc = 0;

/* 存储 */
static uint8_t wifi_bluetooth_tx_buf[BLE_WIFI_TX_BUFFER_SIZE]; /* 用于WiFi和蓝牙发送，向上位机发送 */
static uint8_t wifi_bluetooth_rx_buf[BLE_WIFI_RX_BUFFER_SIZE]; /* 用于WiFi和蓝牙接收，接收上位机数据 */
static MemoryPool* wifi_bluetooth_mempool = NULL; /* 存储上位机发来的命令帧 */
static bms_rx_node_t wifi_bluetooth_rx_node_head; /* 来自上位机的命令帧头节点 */ 

/// @brief 将WiFi和蓝牙接收到的数据帧添加到链表
static void store_wifi_bluetooth_frame()
{
    uint8_t* rx_buffer = AllocateBlock(wifi_bluetooth_mempool);
    if(!rx_buffer)
        return;
    
    XMC_UART_CH_DisableEvent(WIFI_UART_HW, XMC_UART_CH_EVENT_STANDARD_RECEIVE);

    bms_rx_node_t* new_rx_node = (bms_rx_node_t*)rx_buffer; // 分配空间：12字节给 bms_rx_node_t 结构体，剩余字节给接收buffer
    new_rx_node->buffer_ptr = &rx_buffer[sizeof(bms_rx_node_t)]; // 设置指针指向的起始地址

    memcpy(new_rx_node->buffer_ptr, wifi_bluetooth_rx_buf, BMS_FRAME_NO_DATA_LEN + wifi_bluetooth_rx_buf[5]); /* 将 wifi_frame_t 数据拷贝到节点buffer */
    list_add(&new_rx_node->entry , &wifi_bluetooth_rx_node_head.entry);

    XMC_UART_CH_EnableEvent(WIFI_UART_HW, XMC_UART_CH_EVENT_STANDARD_RECEIVE);
}


/// @brief 检查AT响应
/// @return 
static uint8_t check_AT_response()
{
    /* 如果接收到以 "+BLECONN:" 开头的字符串，认为蓝牙已连接 */
    if(strncmp(BLUETOOTH_LINK_STRING, (const char*)(wifi_bluetooth_rx_buf), strlen(BLUETOOTH_LINK_STRING)) == 0)
        return BLE_LINKED;
    
    /* 如果接收到以 "0,CONNECT" 开头的字符串，认为热点已连接 */
    else if(strncmp(STATION_WIFI_LINK_STRING, (const char*)(&wifi_bluetooth_rx_buf[2]), strlen(STATION_WIFI_LINK_STRING)) == 0)
        return WIFI_LINKED;

    /* 如果接收到以 "+STA_CONNECTED:" 开头的字符串，认为设备主动连接模块AP网络 */
    else if(strncmp(SOFTAP_WIFI_LINK_STRING, (const char*)(wifi_bluetooth_rx_buf), strlen(SOFTAP_WIFI_LINK_STRING)) == 0)
        return WIFI_LINKED;

    return AT_NULL;
}


/// @brief wifi、蓝牙字节接收中断
/// @param  __attribute__ ((section (".ram_code")))
void WIFI_UART_RECEIVE_EVENT_HANDLER(void)
{
    /* 当收到AT响应类似于：0,CONNECT 意味着模块主动接入到路由器所在的WiFi网络
       当收到AT响应类似于：+BLECONN:0,"60:51:42:fe:98:aa"时，说明蓝牙服务端接收到上位机连接，且连接序号为0
     */
    uint8_t u8_data = XMC_UART_CH_GetReceivedData(WIFI_UART_HW);

    /* 如果蓝牙和WiFi没有连接，则监听连接时的AT响应 */
    if((ble_link_st != AT_ble_linked && ble_link_st != AT_ble_SPP) && (wifi_link_st != AT_wifi_linked && wifi_link_st != AT_wifi_SPP))
    {
        wifi_bluetooth_rx_buf[data_pos++] = u8_data; /* 存储字节 */
        if(u8_data == '\n')
        {
            uint8_t at_res = check_AT_response(); /* 对AT响应字符串进行判断 */

            if(at_res == BLE_LINKED)
                ble_link_st = AT_ble_linked;
            else if(at_res == WIFI_LINKED)
                wifi_link_st = AT_wifi_linked;

            data_pos = 0;
        }
        return; /* 直接返回 */
    }

    /* 当处于蓝牙透传模式，或者处于WiFi透传模式时，对接收字节进行判别处理 */
    if(ble_link_st == AT_ble_SPP || wifi_link_st == AT_wifi_SPP)
    {
        if(u8_data == RS485_FRAME_MAGIC_BYTE1) // 头字节1
        {
            data_pos = 0;
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
            wifi_bluetooth_rx_buf[2] = u8_data;
        }

        else if(data_pos == 3) // 接收方id
        {
            if(BMS_ID != u8_data)
            {
                data_pos = 0;
                return;
            }
            wifi_bluetooth_rx_buf[3] = u8_data;
        }

        else if(data_pos == 4) // 命令
        {
            wifi_bluetooth_rx_buf[4] = u8_data;
        }

        else if(data_pos == 5) // 数据长度
        {
            wifi_bluetooth_rx_buf[5] = u8_data; 
        }

        else
        {
            if(data_pos == wifi_bluetooth_rx_buf[5] + 6) // CRC16低字节
            {
                frame_crc = u8_data;
            }
            else if(data_pos == wifi_bluetooth_rx_buf[5] + 7) // CRC16高字节
            {
                frame_crc <<= 8;
                frame_crc |= u8_data;

                if(frame_crc != CRC16_MODBUS(wifi_bluetooth_rx_buf, 6 + wifi_bluetooth_rx_buf[5])) // CRC校验不通过
                {
                    data_pos = 0;
                    return;
                }

                // CRC校验通过, rx_frame合法
                data_pos = 0;
                store_wifi_bluetooth_frame();
                return;
            }
        }
    }

    wifi_bluetooth_rx_buf[data_pos++] = u8_data; /* 存储数据 */
    if(data_pos == (BLE_WIFI_RX_BUFFER_SIZE - 1))
        data_pos = 0;
}

void bms_bluetooth_wifi_send_AT_cmd(const char* AT_cmd)
{
    if(!AT_cmd)
        return;

    for (uint8_t i = 0; i < strlen(AT_cmd); i++)
    {
        XMC_UART_CH_Transmit(WIFI_UART_HW, AT_cmd[i]);
    }
}

uint16_t bms_bluetooth_wifi_heap_used()
{
    return wifi_bluetooth_mempool->usedCount * (sizeof(bms_rx_node_t) + BLE_WIFI_RX_BUFFER_SIZE);
}

bluetooth_link_state_t bms_ble_link_state()
{
    return ble_link_st;
}

void bms_bluetooth_wifi_send_data(uint8_t* data, uint8_t len)
{
    if(!data || len == 0)
        return;

    /* 发送数据 */
    for (uint8_t i = 0; i < len; i++)
    {
        XMC_UART_CH_Transmit(WIFI_UART_HW, data[i]);
    }
}

wifi_link_state_t bms_wifi_link_state()
{
    return wifi_link_st;
}

void bms_wifi_enter_SPP_mode()
{
    wifi_link_st = AT_wifi_SPP;

    bms_bluetooth_wifi_send_AT_cmd(bluetooth_stop_cmd); /* 关闭蓝牙 */
    XMC_Delay(10);

    bms_bluetooth_wifi_send_AT_cmd(cipstart_cmd); /* UDP连接 */
    XMC_Delay(10);

    bms_bluetooth_wifi_send_AT_cmd(cipmode_cmd); /* 进入透传模式 */
    XMC_Delay(10);

    bms_bluetooth_wifi_send_AT_cmd(cipmode_start_send_cmd); /* 开始透传 */
    XMC_Delay(10);
}

void bms_bluetooth_wifi_init(void)
{
    INIT_LIST_HEAD(&wifi_bluetooth_rx_node_head.entry); // 初始化接收链表
    wifi_bluetooth_rx_node_head.buffer_ptr = NULL;

    wifi_bluetooth_mempool = InitMemoryPool(sizeof(bms_rx_node_t) + BLE_WIFI_RX_BUFFER_SIZE, BLE_WIFI_RX_NODE_MAX);
    if(!wifi_bluetooth_mempool)
        return;

    wifi_bluetooth_tx_buf[0] = MAGIC_BYTE1;
    wifi_bluetooth_tx_buf[1] = MAGIC_BYTE2;
    wifi_bluetooth_tx_buf[2] = BMS_ID;

    XMC_Delay(500);
    
    /* 对WiFi初始化，等待客户端连接 */
    bms_bluetooth_wifi_send_AT_cmd(cwmode_cmd); /* 设置softAP模式 */
    XMC_Delay(10);

    bms_bluetooth_wifi_send_AT_cmd(softAP_cmd); /* 配置softAP */
    XMC_Delay(10);

    /* 此时WiFi初始化结束，在客户端发起连接之后，再准备进入SPP透传模式 */

    /* 对蓝牙初始化，等待客户端连接 */
    bms_bluetooth_wifi_send_AT_cmd(bluetooth_init_cmd); /* 初始化 */
    XMC_Delay(10);

    bms_bluetooth_wifi_send_AT_cmd(bluetooth_name_cmd); /* 设置名称 */
    XMC_Delay(10);

    bms_bluetooth_wifi_send_AT_cmd(bluetooth_broadcast_params_cmd); /* 设置广播参数 */
    XMC_Delay(10);

    bms_bluetooth_wifi_send_AT_cmd(bluetooth_broadcast_data_cmd); /* 设置广播数据 */
    XMC_Delay(10);

    bms_bluetooth_wifi_send_AT_cmd(bluetooth_broadcast_start_cmd); /* 开始广播，等待客户端连接 */
    XMC_Delay(10);

    /* 第五步，等待数据传输 */
    XMC_Delay(5);

    NVIC_EnableIRQ(WIFI_UART_RECEIVE_EVENT_IRQN);
}

/// @brief 获取无线模块较早的接收帧节点
/// @return 
static bms_rx_node_t* bluetooth_wifi_get_oldest_rx_node()
{
    return list_entry(wifi_bluetooth_rx_node_head.entry.next, bms_rx_node_t, entry);
}

/// @brief 释放无线模块接收帧节点
/// @param rx_node 
static void free_bluetooth_wifi_rx_node(bms_rx_node_t* rx_node)
{
    if(!rx_node)
        return;

    FreeBlock(wifi_bluetooth_mempool, rx_node);
}

void bms_bluetooth_wifi_response()
{
    bms_rx_node_t* rx_node = bluetooth_wifi_get_oldest_rx_node();
	if(!rx_node || (rx_node->buffer_ptr == NULL))
    {
        return;
    }

    uint8_t transmitter_id = rx_node->buffer_ptr[2]; // 发送方ID
	uint8_t cmd_type = rx_node->buffer_ptr[4] ; // 命令类型
	uint8_t* data = &rx_node->buffer_ptr[6]; // 有效载荷起始位置

    wifi_bluetooth_tx_buf[3] = transmitter_id;
    wifi_bluetooth_tx_buf[4] = cmd_type | RESPONSE_FLAG;

    bms_status_t* bms_st = read_bms_status();
    bms_config_t* bms_cfg = read_bms_config();

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
        wifi_bluetooth_tx_buf[5] = 0;
        crc_rslt = CRC16_MODBUS(wifi_bluetooth_tx_buf, 6);

        wifi_bluetooth_tx_buf[6] = crc_rslt >> 8;
        wifi_bluetooth_tx_buf[7] = crc_rslt & 0xff;
        bms_bluetooth_wifi_send_data(wifi_bluetooth_tx_buf, BLE_WIFI_FRAME_NO_DATA_LEN);
        break;

    case Read_BMS_Slave_Count: /* 读取BMS从机数量和每个从机的ID */
        index = 0;
        list_for_each_entry(ptr, &(slave_head->entry), entry)
        {
            wifi_bluetooth_tx_buf[7 + index] = ptr->slave_id; /* 从机ID */
            index++;
        }
        wifi_bluetooth_tx_buf[6] = index; /* 从机数量 */
        wifi_bluetooth_tx_buf[5] = 9 + index - BMS_FRAME_NO_DATA_LEN; /* 回复字节长度 */

        crc_rslt = CRC16_MODBUS(wifi_bluetooth_tx_buf, 7 + index);
        wifi_bluetooth_tx_buf[7 + index] = crc_rslt >> 8;
        wifi_bluetooth_tx_buf[8 + index] = crc_rslt & 0xff;

        bms_bluetooth_wifi_send_data(wifi_bluetooth_tx_buf, 9 + index);
        break;

    case Read_BMS_Config: /* 读取BMS配置 */

        wifi_bluetooth_tx_buf[5] = sizeof(bms_config_t); /* 载荷长度 */
        memcpy(&wifi_bluetooth_tx_buf[6], bms_cfg, sizeof(bms_config_t));
        crc_rslt = CRC16_MODBUS(wifi_bluetooth_tx_buf, sizeof(bms_config_t) + 6);

        wifi_bluetooth_tx_buf[sizeof(bms_config_t) + 6] = crc_rslt >> 8;
        wifi_bluetooth_tx_buf[sizeof(bms_config_t) + 7] = crc_rslt & 0xff;

        bms_bluetooth_wifi_send_data(wifi_bluetooth_tx_buf, sizeof(bms_config_t) + 8);
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

        wifi_bluetooth_tx_buf[5] = sizeof(slave_config_t);
        slave_cfg = get_slave_cfg();
        memcpy(&wifi_bluetooth_tx_buf[6], slave_cfg, sizeof(slave_config_t));
        crc_rslt = CRC16_MODBUS(wifi_bluetooth_tx_buf, sizeof(slave_config_t) + 6);

        wifi_bluetooth_tx_buf[sizeof(slave_config_t) + 6] = crc_rslt >> 8;
        wifi_bluetooth_tx_buf[sizeof(slave_config_t) + 7] = crc_rslt & 0xff;

        bms_bluetooth_wifi_send_data(wifi_bluetooth_tx_buf, sizeof(slave_config_t) + 8);
        break;

    case Read_Pack_State: /* 上位机读取特定PACK状态 */
        slave_id = data[0]; /* 第一个字节表示从机ID */
        slave_st = read_slave_status(slave_id);

        wifi_bluetooth_tx_buf[5] = 48; // 有效载荷48字节，包括1字节ID，36字节电压，8字节温度，3字节均衡
        wifi_bluetooth_tx_buf[6] = data[0]; // 感兴趣从机ID

        /* 填充电芯电压，单位mV */
        for (index = 0; index < SLAVE_CELL_SERIAL_COUNT; index++)
        {
            wifi_bluetooth_tx_buf[7 + 2 * index] = slave_st->cmu_board_cell_voltage[index] >> 8;
            wifi_bluetooth_tx_buf[7 + 2 * index + 1] = slave_st->cmu_board_cell_voltage[index] & 0xff;
        }
        /* 填充NTC温度 */
        for (index = 0; index < SLAVE_NTC_NUM; index++)
        {
            wifi_bluetooth_tx_buf[43 + 2 * index] = slave_st->cmu_board_ntc_temp_result[index] >> 8;
            wifi_bluetooth_tx_buf[43 + 2 * index + 1] = slave_st->cmu_board_ntc_temp_result[index] & 0xff;
        }
        /* 填充均衡状态 */
        wifi_bluetooth_tx_buf[51] = slave_st->MT9805_DCC >> 16;
        wifi_bluetooth_tx_buf[52] = slave_st->MT9805_DCC >> 8;
        wifi_bluetooth_tx_buf[53] = slave_st->MT9805_DCC & 0xff;

        crc_rslt = CRC16_MODBUS(wifi_bluetooth_tx_buf, 54);
                
        wifi_bluetooth_tx_buf[54] = crc_rslt >> 8;
        wifi_bluetooth_tx_buf[55] = crc_rslt & 0xff;

        bms_bluetooth_wifi_send_data(wifi_bluetooth_tx_buf, 56);
        break;

    case Read_BMS_State: /* 上位机向BMS主机询问其状态，BMS主机返回 bms_status_t 结构体 */
        wifi_bluetooth_tx_buf[5] = sizeof(bms_status_t); // 发送电池电压使用2字节，单位V
        memcpy(&wifi_bluetooth_tx_buf[6], bms_st, sizeof(bms_status_t));

        crc_rslt = CRC16_MODBUS(wifi_bluetooth_tx_buf, 6 + sizeof(bms_status_t));

        wifi_bluetooth_tx_buf[6 + sizeof(bms_status_t)] = crc_rslt >> 8;
        wifi_bluetooth_tx_buf[7 + sizeof(bms_status_t)] = crc_rslt & 0xff;
        bms_bluetooth_wifi_send_data(wifi_bluetooth_tx_buf, BMS_FRAME_NO_DATA_LEN + sizeof(bms_status_t));

        break;

    case Read_All_Bat_Volt: /* 上位机读取电池总压 */
        wifi_bluetooth_tx_buf[5] = 2;
        wifi_bluetooth_tx_buf[6] = bms_st->voltage >> 8;
        wifi_bluetooth_tx_buf[7] = bms_st->voltage & 0xff;

        crc_rslt = CRC16_MODBUS(wifi_bluetooth_tx_buf, 8);

        wifi_bluetooth_tx_buf[8] = crc_rslt >> 8;
        wifi_bluetooth_tx_buf[9] = crc_rslt & 0xff;
        bms_bluetooth_wifi_send_data(wifi_bluetooth_tx_buf, BLE_WIFI_FRAME_NO_DATA_LEN + 2);
        break;

    case Read_Power_Current: /* 上位机读取电池输出/输入电流 */
        wifi_bluetooth_tx_buf[5] = 2;
        wifi_bluetooth_tx_buf[6] = bms_st->power_current_A >> 8;
        wifi_bluetooth_tx_buf[7] = bms_st->power_current_A & 0xff;

        crc_rslt = CRC16_MODBUS(wifi_bluetooth_tx_buf, 8);

        wifi_bluetooth_tx_buf[8] = crc_rslt >> 8;
        wifi_bluetooth_tx_buf[9] = crc_rslt & 0xff;
        bms_bluetooth_wifi_send_data(wifi_bluetooth_tx_buf, BLE_WIFI_FRAME_NO_DATA_LEN + 2);
        break;

    case Read_BMS_NTC_Temp: /* 上位机读取BMS的板载NTC温度 */
        wifi_bluetooth_tx_buf[5] = 2;
        wifi_bluetooth_tx_buf[6] = bms_st->NTC1_temp;
        wifi_bluetooth_tx_buf[7] = bms_st->NTC2_temp;

        crc_rslt = CRC16_MODBUS(wifi_bluetooth_tx_buf, 8);

        wifi_bluetooth_tx_buf[8] = crc_rslt >> 8;
        wifi_bluetooth_tx_buf[9] = crc_rslt & 0xff;
        bms_bluetooth_wifi_send_data(wifi_bluetooth_tx_buf, BLE_WIFI_FRAME_NO_DATA_LEN + 2);
        break;

    case Control_PreCharge: /* 上位机控制BMS预充 */
        if(data[0] == start)
            bms_precharge(start);
        else if(data[0] == stop)
            bms_precharge(stop);
        
        wifi_bluetooth_tx_buf[5] = 0;
        wifi_bluetooth_tx_buf[6] = data[0];
        crc_rslt = CRC16_MODBUS(wifi_bluetooth_tx_buf, 7);
        wifi_bluetooth_tx_buf[7] = crc_rslt >> 8;
        wifi_bluetooth_tx_buf[8] = crc_rslt & 0xff;
        bms_bluetooth_wifi_send_data(wifi_bluetooth_tx_buf, BLE_WIFI_FRAME_NO_DATA_LEN + 1);
        break;

    case Control_ChargeDischarge: /* 上位机命令BMS充电/放电 */
        if(data[0] == start)
            bms_charge_discharge(start);
        else if(data[0] == stop)
            bms_charge_discharge(stop);

        wifi_bluetooth_tx_buf[5] = 0;
        wifi_bluetooth_tx_buf[6] = data[0];
        crc_rslt = CRC16_MODBUS(wifi_bluetooth_tx_buf, 7);
        wifi_bluetooth_tx_buf[7] = crc_rslt >> 8;
        wifi_bluetooth_tx_buf[8] = crc_rslt & 0xff;
        bms_bluetooth_wifi_send_data(wifi_bluetooth_tx_buf, BLE_WIFI_FRAME_NO_DATA_LEN + 1);
        break;

    case Report_BMS_Error: /* 报告BMS错误 */
        wifi_bluetooth_tx_buf[5] = 2;
        wifi_bluetooth_tx_buf[6] = bms_st->error_state >> 8;
        wifi_bluetooth_tx_buf[7] = bms_st->error_state & 0xff;

        crc_rslt = CRC16_MODBUS(wifi_bluetooth_tx_buf, 8);

        wifi_bluetooth_tx_buf[8] = crc_rslt >> 8;
        wifi_bluetooth_tx_buf[9] = crc_rslt & 0xff;
        bms_bluetooth_wifi_send_data(wifi_bluetooth_tx_buf, BLE_WIFI_FRAME_NO_DATA_LEN + 2);
        break;
    
    default:
        break;
    }

    list_del(&rx_node->entry); // 删除节点
	rx_node->buffer_ptr = NULL;
	free_bluetooth_wifi_rx_node(rx_node);
}

void bms_bluetooth_enter_SPP_mode()
{
    ble_link_st = AT_ble_SPP;

    bms_bluetooth_wifi_send_AT_cmd(cwmode_stop_cmd); /* 停止WiFi */
    XMC_Delay(10);

    bms_bluetooth_wifi_send_AT_cmd(bluetooth_SPP_config_cmd); /* 配置SPP */
    XMC_Delay(10);

    bms_bluetooth_wifi_send_AT_cmd(bluetooth_SPP_enable_cmd); /* 使能SPP，进入透传模式 */
    XMC_Delay(10);
}

