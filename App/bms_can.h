
#ifndef BMS_CAN_H
#define BMS_CAN_H

#include <stdint.h>

/**
 * BMS CAN适配于博瑞飞控（v1.0.0 2025-9-1）
 * 博瑞飞控：基于标准 CANBus 2.0B 协议，基于 29bit 的扩展帧，通信速率500Kbps，不添加终端匹配电阻
 * 对于29bit CAN ID，分配如下：
 * bit:  28  27  26  25  24  23  22  21  20  19  18  17  16  15  14  13  12  11  10  9  8  7  6  5   4  3   2  1  0
 * filed:|  Type  |  | Msg ID             |  |  Flag  |  |  Dest Node  ID     |  |  Src Node ID  | SOF EOF  | Num  |
 * Type: 消息类型，固定值 0'b000
 * Msg ID : 消息ID，越小优先级越高
 * Flag: 固定值 0'b111
 * Dest Node  ID : 目标节点ID
 * Src Node ID : 源节点ID
 * SOF : 多帧传输时发送的第一帧此位置1，其它帧此位置0。单帧传输时此位置1
 * EOF : 多帧传输时发送的最后一帧此位置1，其它帧此位置0。单帧传输时此位置1
 * Num : 帧序号，多帧传输时发送第一帧此3位为001，此后每发一帧加1，溢出则翻转。单帧传输时，此3位为001
 * 
 * 备注：飞控的 ID 为1，BMS ID在0x02~0x22
 * 本BMS为主从模式，因此只有一个ID设为 2
 * 
 * 协议内容：
 * Msg ID       |    方向     |    长度   |                  描述              |  备注
 *        0x01    BMS->飞控	        N            BMS发送实时信息包                连续帧5hz
 *        0x02    飞控->BMS         0            获取电池身份信息                 飞控主动请求
 *        0x02    BMS->飞控         N            BMS回复电池身份信息              BMS应答
 *        0x03    飞控->BMS         0            获取电池基本信息                 飞控主动请求
 *        0x03    BMS->飞控         N            BMS回复电池基本信息              BMS应答
 *        0x04    飞控->BMS         0            获取电池监控信息                 飞控主动请求
 *        0x04    BMS->飞控         N            BMS回复电池监控信息              BMS应答
 * 
 * 备注：小端模式，数值低字节优先发送；小于等于8字节的一包数据，单帧发送，不带校验；大于8字节的一包数据，分多帧发送，带CRC16校验，数据包前两个是校验位
 * 
 * Msg ID = 1，BMS发送实时信息
 * 序号	字段	字节	长度	    单位	备注
 * 1    总电压	[0,1]	2 ushort	10mv	
 * 2	总电流	[2,5]	4 int	    ma	    充电为 正，放电为 负
 * 3	电芯温度		2 short	    0.1℃	
 * 4	MOSFET温度	    2 short	    0.1℃	
 * 5	电量百分比		2 ushort	0.1%	
 * 6	电池组状态		4 uint		        bit 0：过流；bit 1：电压过高；bit 2：电压过低；bit 3：温度过高；bit 4：温度过低；bit 5：压差过大；bit 6：SOC低报警；bit 7：SOH报警；bit 8：充电短路
 * 7	电池串数		2 ushort		   电池串联电芯数
 * 7+1	Cell 1电压	   2 ushort	    mv	电芯1的电压
 * ...
 * 7+N  Cell N电压	   2 ushort	    mv	电芯N的电压
 * 
 * BMS 身份信息：
 * 序号	    字段	       	长度		备注
 * 1	    电池厂商信息	20	    	空字节用0填充
 * 2	    电池型号信息	20	    	必须要有，不同电池型号不同。空字节用0填充
 * 3	    电池ID信息		20	    	必须有唯一ID，空字节用0填充
 * 4	    电池硬件版本	10	
 * 5	    电池软件版本	10	
 * 
 * BMS 基本信息：
 * 序号	    字段	     	长度	单位
 * 1	    电池设计容量	2	    100maH
 * 2	    电池放电倍率	2	    C
 * 3	    单片标称电压	2	    10mv
 * 4	    电池串数	    2	    串
 * 
 * BMS 监测信息：
 * 序号	    字段	        长度	单位
 * 1	    电池健康度		2	    %
 * 2	    循环次数		2	    次
 * 3	    过充次数		2	    次
 * 4	    过放次数		2	    次
 * 5	    过流次数		2	    次
 * 6	    过温次数		2	    次
 * 7	    Soc值		    2	
 * 8	    Soh值		    2	
 * 9	    充电容量		2	    100maH
 * 10	    剩余容量		2	    100maH
 * 11	    电池内阻值		2	    mΩ
 */

#define BMS_FK_VOLT_SCALE 100 /* 单位10mV */
#define BMS_FK_CURRENT_SCALE 1000 /* 单位mA */
#define BMS_FK_TEMP_SCALE 10 /* 单位0.1℃ */
#define BMS_FK_SOC_SCALE 10 /* 单位0.1% */
#define BMS_FK_CAP_SCALE 10 /* 单位0.1Ah */
#define BMS_FK_DIFF_LIMIT 300 /* 压差大于300mV认为压差过大 */

#define BMS_FK_OVER_CURRENT 0x01 /* 过流 */
#define BMS_FK_OVER_VOLTAGE 0x02 /* 电压过高 */
#define BMS_FK_UNDER_VOLTAGE 0x04 /* 电压过低 */
#define BMS_FK_OVER_TEMP 0x08 /* 过温 */
#define BMS_FK_UNDER_TEMP 0x10 /* 低温 */
#define BMS_FK_VOLT_DIF 0x20 /* 压差过大 */
#define BMS_FK_SOC_WARN 0x40 /* SOC过低 */
#define BMS_FK_SOH_WARN 0x80 /* SOH报警 */
#define BMS_FK_CHARGE_SHORT 0x100 /* 充电短路 */

#define BMS_SRC_NODE_ID_SHIFT 5
#define BMS_DST_NODE_ID_SHIFT 11
#define BMS_MSG_ID_SHIFT 20

#define BMS_SRC_NODE_ID_MASK (0x3f << BMS_SRC_NODE_ID_SHIFT)
#define BMS_DST_NODE_ID_MASK (0x3f << BMS_DST_NODE_ID_SHIFT)
#define BMS_MSG_ID_MASK (0x3f << BMS_MSG_ID_SHIFT)

#define BMS_CAN_MASTER_ID 0x01  /* 飞控 */
#define BMS_CAN_SLAVE_ID 0x02 /* BMS */

#define BMS_CAN_MSG_TYPE_1 0x01  /* BMS实时信息上传 */
#define BMS_CAN_MSG_TYPE_2 0x02  /* 身份信息 */
#define BMS_CAN_MSG_TYPE_3 0x03 /* 基本信息 */
#define BMS_CAN_MSG_TYPE_4 0x04 /* 监控信息 */

#define BMS_CAN_UPLOAD_PERIOD 200 /* BMS主动通过CAN向飞控上传状态报文的间隔200ms */

/// @brief IDE域
typedef struct bms_can_identify
{
    uint32_t num : 3; /* 帧序号，多帧传输时发送第一帧此3位为001，此后每发一帧加1，溢出则翻转。单帧传输时，此3位为001 */
    uint32_t eof: 1; /* 多帧传输时发送的最后一帧此位置1，其它帧此位置0。单帧传输时此位置1 */
    uint32_t sof : 1; /* 多帧传输时发送的第一帧此位置1，其它帧此位置0。单帧传输时此位置1 */
    uint32_t src_node_id : 6; /* 源ID */
    uint32_t dest_node_id : 6; /* 目标ID */
    uint32_t flag : 3; /* 固定值0'b111 */
    uint32_t msg_id : 6; /* 消息ID，值越小优先级越高 */
    uint32_t type : 4; /* 固定值0'b000 */
    uint32_t unuesd: 2; /* 占位 */
}bms_can_identify_t;


/// @brief 身份信息
typedef struct bms_can_ID_info
{
    char manufacturer[20]; /* 电池厂商 */
    char bat_code[20]; /* 电池型号 */
    char bat_ID[20]; /* 电池ID */
    char bat_hardware_version[10]; /* 硬件版本 */
    char bat_software_version[10]; /* 软件版本 */
}bms_can_ID_info_t;

/// @brief 基本信息
typedef struct bms_can_basic_info
{
    uint16_t cap; /* 电池容量，单位0.1Ah */
    uint16_t discharge_rate; /* 放电倍率，单位C */
    uint16_t cell_rated_volt; /* 电芯额定电压，单位10mV */
    uint16_t cell_serial_count; /* 电芯串联数量 */
}bms_can_basic_info_t;

/// @brief 监控信息
typedef struct bms_can_monitor_info
{
    uint16_t soh; /* 电池健康度，单位%，90意味着SOH=90% */
    uint16_t cycle_times; /* 电池循环次数 */
    uint16_t OV_times; /* 过充次数 */
    uint16_t UV_times; /* 过放次数 */
    uint16_t over_current_times; /* 过流次数 */
    uint16_t over_temp_times; /* 过温次数 */
    uint16_t soc; /* SOC值，单位% */
    uint16_t soh_2; /* SOH值，单位%，与soh一致 */
    uint16_t charge_cap; /* 充电容量，单位0.1Ah */
    uint16_t current_cap; /* 剩余容量 */
    uint16_t internal_res; /* 电池内阻，单位mΩ */
}bms_can_monitor_info_t;

/// @brief BMS CAN初始化
/// @param  
void bms_can_init(void);

/// @brief BMS向飞控上传状态信息
/// @param  
void bms_upload_state_info(void);


#endif







