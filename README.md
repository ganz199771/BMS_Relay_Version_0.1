根据BMS设计方案，BMS为主从模式，每个PACK内部一个BMS从机，所有PACK被一个BMS主机控制，从BMS主机功能需求出发，对主机软件完成详细设计。

## 功能要求与分析
根据《电池PACK总体方案》，BMS主机应具备如下功能：
- 测量电池系统总电压、测量动力线上的充电/放电电流；
- 测量绝缘电阻；
- 测量两路NTC，可用于监测动力线/接触器温度；
- 可接收外部命令开关继电器，在触发保护时能自动关闭继电器；
- 支持电芯过压保护、欠压保护；
- 支持充电高温保护、低温保护；
- 支持放电高温保护、低温保护；
- 支持充电/放电过流保护；
- 支持PACK过压保护、欠压保护；
- 支持WiFi、蓝牙、UART通信；
- 与从机通过RS485/CAN通信；
- 配置参数存储在Flash，可通过上位机修改；
- 监控特定回路的完整性，例如通过接触器的辅助线圈是否闭合判断接触器主回路是否闭合


## 分析所需外设

根据BMS主机硬件设计，BMS主机主控采用XMC1404-F064X0200 Cortex-M0单片机，与代码关系密切的基本参数如下：
- SRAM为16Kbyte，Flash为200Kbyte；
- 2个CCU4定时器，共4个channel，2个CCU8定时器共4个channel；
- 2个USIC通用串行控制器共4个channel，每个channel可配置一路串口；
- 2个ADC独立内核，每个独立内核支持12路采样；
- 支持一路CAN2.0B收发器；

根据BMS主机软件功能需求列出的12点，对BMS软件所需外设进行说明
- 测量电池系统电压、测量充放电电流、测量绝缘内阻、测量两路NTC需要ADC外设，此外为丰富BMS主机功能，还测量BMS主机输入电源电压、BMS主机消耗电源电流，因此需要多路的ADC channel；
- 控制继电器，结合BMS主机硬件设计来看，需要主控通过一个GPIO控制；
- 过压保护、欠压保护等这些功能，没有直接使用到外设，其逻辑是根据ADC测量得到的值，与预设值进行比较，然后控制继电器；
- 根据BMS主机硬件设计，BMS主机使用ESP32-C6-WROOM-1-N8模块完成和上位机的蓝牙+WiFi通信，此模块与MCU通信物理接口是USIC，此外还有一个单独的USIC接口与上位机/飞控等设备通信;
- 与从机通过一路RS485以及一路CAN总线通信，需要一个USIC外设和一个CAN外设；
- MCU需要控制内部Flash存储BMS主机参数，并提供修改配置接口；
- MCU通过定时器的比较输出和输入捕获功能，监控特定回路的完整性；
- 通过系统定时器或者CCU4/8定时器，产生系统标准节拍，根据节拍安排任务或者处理任务。

## 分析工作流程

- BMS对使用到的外设进行初始化，设置外设中断优先级并启动，典型的就是启用USIC接收字节中断或者FIFO中断，触发中断后在服务函数中处理接收数据；
- 读取Flash存储的BMS配置参数结构体，初始化BMS配置；
- 初始化BMS状态，例如不处于充电/放电状态，将充电/放电继电器全部断开，将BMS错误状态置0，将发送buffer或者frame预填充固定帧头等；
- 申请接收WiFi、蓝牙、UART所需的buffer，为降低丢包率可以使用内存池预先申请若干个内存块，用于存储多个接收frame；
- 对于WiFi和蓝牙模块，还需要配置网络模式、SSID和password等操作；
- 完成以上步骤后，BMS主机开始与从机、上位机交互；
- BMS主机需要了解当前有多少从机在线，并且需要知道各自的ID号，需要通过轮询实现；
- BMS主机了解到所有在线从机之后，需要知道从机所在的电池PACK的状态，关键是电芯电压和温度；
- BMS主机知道各个PACK的状态后，根据SOC-OCV表格，计算得到SOC初值并更新BMS状态，BMS主机此时进入prepare状态；
- BMS主机进入loop状态，主机根据时间调度，主动轮询所有在线从机，更新从机状态信息，并及时处理来自从机的回复数据帧，以及来自上位机的询问/控制帧，此外BMS主机会检查从机状态、BMS状态，如果触发保护条件及时处理。


## BMS消息类型

- Active用于主机确认哪些从机在线，使用方式如下：
上位机---------Active------------>BMS主机
BMS主机-----------Active------>上位机
以及：
BMS主机---------Active------------>BMS从机
BMS从机---------Active------------>BMS主机
- Read_Pack_Config ~ Read_Balance_Config用于BMS主机读取BMS从机配置，另外，Read_BMS_Config用于上位机读取BMS主机配置；
- Read_Pack_State ~ Read_ChipTEMP用于BMS主机读取BMS从机状态；
- Read_BMS_Slave_Count ~ Report_BMS_Error用于上位机读取BMS信息；
- Control_Charge、Control_Discharge用在上位机控制BMS的继电器开关，Control_Balance用于BMS主机开启BMS从机被动均衡，Heat_Pack用于BMS主机加热BMS从机；
- Write_BMS_Config用于上位机修改BMS配置参数，Restore_BMS_Config 用于上位机强制将BMS配置参数恢复到出厂模式。


## BMS初始化函数bms_init

bms_init完成如下动作：
- 对RS485、UART等发送buffer预填充帧头；
- 重置BMS状态信息结构体；
- 初始化BMS配置，bms_config_init读取Flash存储的BMS配置信息并拷贝到RAM的BMS配置结构体；
- 初始化RS485（和从机通信）、UART（和上位机通信）、UART（和WiFi/蓝牙模块通信）所需要的资源，例如接收内存池、接收节点链表等



## BMS主机准备函数bms_prepare

根据《BMS总体设计方案》，BMS主机在轮询从机以及接收来自上位机的数据帧之前，需要做准备工作进入ready状态。
- 获取当前所有在线从机；
- 获取从机配置；
- 获取所有从机状态；
- 根据从机状态，结合电芯的SOC-OCV表格，计算SOC初始值；
- 进入ready状态



## BMS主机代码运行流程

- 初始化外设；
- 初始化内存资源；
- 使能中断；
- bms初始化；
- 进入while循环调用bms_run，依次执行bms_response回复数据帧；执行bms_background_work执行后台任务；
执行bms_poll_slave_status轮询从机状态；执行bms_prepare进行准备工作。


## 版本
2026/1/27     version 1.0         完成BMS主机基本功能，补充ReadMe文件
