#include <rtthread.h>
#include "stm32f1xx_hal.h"

#ifdef RT_USING_FINSH
char rt_hw_console_getchar(void)
{
    char ch = 0;

    // 轮询方式读取串口1（F103 标准写法）
    while((USART1->SR & USART_SR_RXNE) == 0);
    ch = (char)USART1->DR;

    return ch;
}
#endif