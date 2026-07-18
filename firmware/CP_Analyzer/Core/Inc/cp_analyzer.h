#ifndef CP_ANALYZER_H
#define CP_ANALYZER_H

#include "stm32g4xx_hal.h"

void CP_Init(void);   /* Call once in main.c after all MX_xxx_Init()  */
void CP_Task(void);   /* Call every loop in while(1)                  */
/* HAL_TIM_IC_CaptureCallback is defined inside cp_analyzer.c — do NOT
   define it again in stm32g4xx_it.c, it will cause a duplicate error */

#endif
