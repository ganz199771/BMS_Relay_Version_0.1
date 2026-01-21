
#include "bms_code_eval.h"
#include "bms_rs485_uart.h"
#include "bms_wifi.h"

#include <cmsis_gcc.h>

#define RAM_TOTAL_SIZE (16 * 1024)
#define STACK_FILL_PATTERN 0xAAAAAAAA
#define STACK_SIZE 0x400  // 1KB栈，根据链接脚本定义

// 在链接脚本中定义这些符号（如XMC1000.ld）
extern uint32_t __data_start;
extern uint32_t __data_end;
extern uint32_t __bss_start;
extern uint32_t __bss_end;
extern uint32_t __HeapBase;
extern uint32_t __HeapLimit;
extern uint32_t __StackTop;
extern uint32_t __StackLimit;
extern uint32_t VeneerEnd;
extern uint32_t VeneerStart;

void fill_stack_with_pattern()
{
    uint32_t *stack_start = &__StackLimit;
    uint32_t *stack_end = &__StackTop;
    
    for (uint32_t *p = stack_start; p < stack_end; p++) {
        *p = STACK_FILL_PATTERN;
    }
}

/// @brief 获取堆内存使用情况
/// @return 
static uint16_t get_current_heap_usage()
{
    /* BMS主机在三处地方使用了malloc，分别是：
    1、用于和BMS从机通信的 _bms_rs485_mempool，一次性申请了 1400字节，每次释放1个block则返回140字节
    2、用于和上位机通信的 BMS_HOST_RX_BUFFER_MAX，一次性申请了 460字节，每次释放1个block则返回92字节
    3、用于WiFi通信的 wifi_mempool，一次性申请了 460字节，每次释放1个block则返回92字节
    4、用于存储从机节点信息的 _bms_slave_mempool，一次性申请了1216字节，每次释放一个block返回76字节
     */

    uint16_t heap1_used = rs485_heap_used(); // 2240
    uint16_t heap2_used = host_heap_used(); // 33972
    uint16_t heap3_used = slave_heap_used(); // 1384
    uint16_t heap4_used = wifi_heap_used(); // 2564

    uint16_t heap_used_byte = heap1_used + heap2_used + heap3_used + heap4_used;
    return heap_used_byte;
}

/// @brief 获取栈内存使用情况
/// @return 
static uint16_t get_current_stack_usage()
{
    uint32_t *stack_start = &__StackLimit;
    uint32_t *stack_end = &__StackTop;
    uint32_t unused_count = 0;
    
    // 从栈底向栈顶检查，找到第一个被修改的值
    for (uint32_t *p = stack_start; p < stack_end; p++) {
        if (*p == STACK_FILL_PATTERN) {
            unused_count++;
        } else {
            break;  // 找到已使用区域
        }
    }
    
    uint32_t total_stack = (uint32_t)stack_end - (uint32_t)stack_start;
    uint16_t used_stack = total_stack - (unused_count * sizeof(uint32_t));
    
    return used_stack;
}

/// @brief 获取RAM使用情况
/// @return 
uint8_t bms_ram_usage()
{
    // 静态数据使用
    uint16_t data_size = (uint32_t)&__data_end - (uint32_t)&__data_start;
    uint16_t bss_size = (uint32_t)&__bss_end - (uint32_t)&__bss_start;
    uint16_t venner_size = (uint32_t)&VeneerEnd - (uint32_t)&VeneerStart;

    // 动态堆使用（需要配合堆监控）
    uint16_t heap_used = get_current_heap_usage();  // 实现此函数
    
    // 栈使用（需要配合栈监控）
    uint16_t stack_used = get_current_stack_usage();  // 实现此函数
    
    uint16_t total_used = venner_size + data_size + bss_size + heap_used + stack_used;
    uint16_t total_ram = RAM_TOTAL_SIZE;  // 从数据手册获取
    
    return (uint8_t)(1.0f * total_used / total_ram * 100);
}

