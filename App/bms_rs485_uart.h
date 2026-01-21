
#ifndef _CMU_VERSION0_1_RS485_H_
#define _CMU_VERSION0_1_RS485_H_

#include <stdint.h>
#include "bms.h"

/// @brief 计算CRC16校验值
/// @param puchMsg 
/// @param usDataLen 
/// @return 
uint16_t CRC16_MODBUS(uint8_t *puchMsg, uint16_t usDataLen);

/// @brief 重新启动RS485，当使用环形接收缓冲区或者内存池时，此函数不使用
/// @param  
void restart_rs485(void);

/// @brief RS485总线上发送数据给从机
/// @param frame_data 
/// @param frame_len 
void rs485_transmit(uint8_t* frame_data, uint16_t frame_len);

/// @brief UART发送给上位机
/// @param frame_data 
/// @param frame_len 
void host_uart_transmit(uint8_t* frame_data, uint16_t frame_len);

/// @brief BMS主机向BMS从机发送的buffer进行初始化，BMS主机向上位机发送的buffer初始化
/// @param  
void bms_rs485_uart_tx_init(void);

/// @brief 初始化RS485，UART资源
void bms_rs485_uart_init();

/// @brief 获得较早的接收节点
/// @return 
bms_rx_node_t* get_wire_oldest_rx_node();

/// @brief BMS主机回复BMS从机
/// @param rx_node 
void bms_response_slave(bms_rx_node_t* rx_node);

/// @brief BMS主机回复上位机
/// @param rx_node 
void bms_response_host(bms_rx_node_t* rx_node);


/// @brief BMS准备发送帧，发送到RS485总线上的从机
/// @param target_id 目标ID
/// @param cmd_type 命令类型
/// @param len 数据长度
/// @param payload 数据
uint8_t* bms_prepare_rs485_tx_frame(uint8_t target_id, bms_cmd_type_t cmd_type, uint8_t len, uint8_t* payload);

/// @brief BMS准备发送帧，发送到上位机
/// @param target_id 目标ID
/// @param cmd_type 命令类型
/// @param len 数据长度
/// @param payload 数据
uint8_t* bms_prepare_host_uart_tx_frame(uint8_t target_id, bms_cmd_type_t cmd_type, uint8_t len, uint8_t* payload);

/// @brief 获取从机链表头节点
/// @return 
slave_node_t* get_slave_list_head();

/// @brief 用于RS485的堆内存，已使用空间
/// @return 
uint16_t rs485_heap_used();

/// @brief 用于上位机通信的堆内存已使用空间
/// @return 
uint16_t host_heap_used();

/// @brief 用于存储从机信息的堆内存使用空间
/// @return 
uint16_t slave_heap_used();

#endif


