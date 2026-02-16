#ifndef _APP_H
#define _APP_H

#include <stdint.h>
#include "config.h"

#define HOST_APP_WIFI_ID 0x01 // 规定上位机/APP的ID(在wifi_frame_t中)是1
#define BMS_BLE_WIFI_ID 0x02 // 规定BMS主板的ID是2
#define BLE_WIFI_TX_BUFFER_SIZE 0x80 // WIFI接收缓冲区大小
#define BLE_WIFI_RX_BUFFER_SIZE 0x50 // WIFI接收缓冲区大小
#define BLE_WIFI_RX_NODE_MAX 0x05 // WIFI接收节点最大数量
#define BLE_WIFI_FRAME_NO_DATA_LEN 0x08 /* 没有有效载荷时，wifi帧长度 */

#define AT_NULL 0x00 /* AT响应字符串未解析含义 */
#define BLE_LINKED 0x01 /* AT响应字符串表示有设备连接了模块的蓝牙 */
#define WIFI_GET_IP 0x02 /* AT响应字符串表示获取了WiFi的IP地址 */
#define WIFI_LINKED 0x03 /* AT响应字符串表示有设备连接了模块的WiFi */

#define BLUETOOTH_LINK_STRING "+BLECONN:" /* 蓝牙连接成功显示字符串头部 */
#define STATION_WIFI_LINK_STRING "CONNECT" /* 0,CONNECT 意味着模块主动接入到路由器所在的WiFi网络 */
#define SOFTAP_WIFI_LINK_STRING "+STA_CONNECTED:" /* 意味着有设备主动连接模块AP网络 */
#define WIFI_IP_STRING "+CIPSTA:ip:" /* 获取到IP地址信息 */

typedef enum AT_rx_flag
{
    None,
    CIPAP
}AT_rx_flag_t;


typedef enum bluetooth_link_state
{
    AT_ble_idle, /* 空闲状态 */
    AT_ble_start, /* 接收到字符+ */
    AT_ble_ongoing, /* 接收字符 */
    AT_ble_linked, /* 接收到\r\n */
    AT_ble_SPP /* 进入SPP透传模式 */
} bluetooth_link_state_t;

typedef enum wifi_link_state
{
    AT_wifi_idle, /* 空闲状态 */
    AT_wifi_start, /* 接收到字符0 */
    AT_wifi_ongoing, /* 接收字符 */
    AT_wifi_linked, /* 接收到\r\n */
    AT_wifi_SPP /* 进入WiFi透传模式 */
} wifi_link_state_t;

/// @brief BMS主板通过wifi向上位机发送数据
/// @param str 
void bms_bluetooth_wifi_send_data(uint8_t* data, uint8_t len);

/// @brief 模块发送AT命令
/// @param AT_cmd 
void bms_bluetooth_wifi_send_AT_cmd(const char* AT_cmd);

/// @brief 使用蓝牙、WIFI 对其进行初始化
void bms_bluetooth_wifi_init();

/// @brief 无线模块响应接收的数据
void bms_bluetooth_wifi_response();

/// @brief BMS主机无线模块接收堆内存使用空间
/// @return 
uint16_t bms_bluetooth_wifi_heap_used();

/* ------------------------------------------------------------------------------------------------------- */

/// @brief BMS主板的wifi模块，作为station其连接状态如何
/// @return 
wifi_link_state_t bms_wifi_link_state();

/// @brief BMS通过发送AT命令，使得wifi进入SPP透传状态
void bms_wifi_enter_SPP_mode();

/* --------------------------------------------------------------------------------------------------------- */

/// @brief BMS主板的蓝牙模块，作为服务器其连接状态如何
/// @return bluetooth_link_state_t枚举变量
bluetooth_link_state_t bms_ble_link_state();

/// @brief BMS通过发送AT命令，使得蓝牙进入SPP透传状态
void bms_bluetooth_enter_SPP_mode();

#endif
