#ifndef HOST_TEST_STM32_HAL_H
#define HOST_TEST_STM32_HAL_H
#include <stdint.h>
/* 仅用于电脑模拟，不加入Keil编译，也不操作真实硬件。 */
uint32_t HAL_GetTick(void);
#endif
