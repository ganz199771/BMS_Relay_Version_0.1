#ifndef _APP_H
#define _APP_H

#include <stdint.h>
#include "config.h"

#define HOST_APP_WIFI_ID 0x01 // 规定上位机/APP的ID(在wifi_frame_t中)是1
#define BMS_WIFI_ID 0x02 // 规定BMS主板的ID是2
#define WIFI_RX_BUFFER_SIZE 0x50 // WIFI接收缓冲区大小
#define WIFI_RX_NODE_MAX 0x05 // WIFI接收节点最大数量

#define WIFI_FRAME_NO_DATA_LEN 0x08 /* 没有有效载荷时，wifi帧长度 */

typedef enum AT_rx_flag
{
    None,
    CIPAP
}AT_rx_flag_t;


typedef enum AT_rx_state
{
    rx_idle,
    rx_pos, // 接收到+
    rx_I, // 接收到I
    rx_P, // 接收到P
    rx_D, // 接收到D
    rx_first_delim, // 接收到第一个,
    rx_id, // 发送方ID
    rx_second_delim, // 接收到第二个,
    rx_data_len, // 接收到数据的长度
    rx_dot, // 接收到: 后续就是数据
    rx_ascii_ch, // 接收到ASCII字符
    rx_r, // 接收到\r
    rx_n // 接收到\n
} AT_rx_state_t;

/// @brief 是否有设备主动连接BMS主板的无线热点或者蓝牙
typedef enum wireless_link_state
{
    no_link, /* 没有连接 */
    link /* 有连接 */
} wireless_link_state_t;


typedef struct wifi_data
{
    uint8_t id;
    uint8_t data_len;
    uint8_t data_buf[WIFI_RX_BUFFER_SIZE]; /* 存储 wifi_frame_t */
}wifi_data_t;

typedef struct wifi_frame
{
    uint8_t magic_tag[2]; // 帧头，固定值0x49 0x47
    uint8_t transmitter_id; // 发送方ID
    uint8_t receiver_id; // 接收方ID
    uint8_t type; // 最高位为0表示请求，最高位为1表示响应
    uint8_t data_len;
    uint8_t* data;
    uint16_t crc;
}wifi_frame_t;


/// @brief 使用的WIFI模块是ESP12F，对其进行初始化
/// @param  
void bms_wifi_init(void);

/// @brief ESP12F响应接收的数据
void ESP12F_response();

/// @brief BMS主板通过ESP12F向上位机发送数据
/// @param str 
void ESP12F_send_data(uint8_t* data, uint8_t len);

/// @brief wifi模块发送AT命令
/// @param AT_cmd 
void wifi_send_AT_cmd(const char* AT_cmd);

/// @brief wifi接收堆内存使用空间
/// @return 
uint16_t wifi_heap_used();

/// @brief 获取当前wifi连接状态
/// @return 
wireless_link_state_t is_wifi_linked();

#endif
