#include "AIHMI.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// 重定义printf函数 
void AIHMI_printf(const char *format, ...)
{
    char buf[64] = {0};  // 输出缓冲区
    va_list args;
    
    // 1. 格式化字符串
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);  // 安全格式化，防止溢出
    va_end(args);

    // 2. 获取字符串长度
    size_t len = strlen(buf);
    if(len == 0) return;  // 空字符串直接返回

    // 3. HAL库发送 (带超时保护)
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
}

