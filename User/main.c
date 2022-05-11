/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : zhu ben chao
 * Version            : V1.0.0
 * Date               : 2022/05/07
 * Description        : Main program body.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Copyright (c) 2021 MEDS, BJTU
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/*
 *@Note
 * SVPWM function test.
 *
 * System core clock is 144MHz.
 *
 * Use CH1(PA8,67), CH1N(PB13,52), CH2(PA9,68), CH2N(PB14,53),
 * CH3(PA10,69), CH3N(PB15,54) of advanced timer TIM1 as PWM output pins.
 *
 */

#include "debug.h"
#include "EEMU_PWM.h"
#include <math.h>

u8 Led = 0;
SV_MODULE Sv_module;
u32 Tim1_int_count = 0;
u16 Fre_m = 200; // 调制波频率
float Coefficient_angle = 0.000314159274f;

void Interrupt_Init(void);
void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    // USART_Printf_Init(115200);
    // printf("SystemClk:%d\r\n", SystemCoreClock);
    svpwm_init(MAX_COUNT_TIM1, 5);
    sv_module_init(&Sv_module);
    Sv_module.m_ref = 0.8f;
    Fre_m = 200;
    Coefficient_angle = MAX_COUNT_TIM1 * PERIOD_TIM1 * PI_DOUBLE;

    // 使能中断
    Tim1_int_count = 0;
    Sv_module.angle_ref = (2 * Tim1_int_count + 1) * Coefficient_angle * Fre_m;
    calc_SV_Uabc(&Sv_module);
    sv_module_calc(&Sv_module, TIM1);
    // u16 compare = get_compare(Tim1_int_count, MAX_COUNT_TIM1);
    // TIM_SetCompare1(TIM1, compare);

    // 使能中断
    Interrupt_Init();
    while (1)
    {
    }
}

void Interrupt_Init(void)
{
    NVIC_EnableIRQ(TIM1_UP_IRQn);
    // 组优先级0，子优先级1，总优先级较高
    NVIC_SetPriority(TIM1_UP_IRQn, (0 << 5) | (0x01 << 4));
}

void TIM1_UP_IRQHandler(void)
{
    // 计数器更新事件中断
    if (TIM_GetFlagStatus(TIM1, TIM_FLAG_Update) == SET)
    {
        // 计数器下溢
        if (TIM1->CNT == 0)
        {
            Tim1_int_count += 1;
            // 计算下次中断的角度
            Sv_module.angle_ref = (2 * Tim1_int_count + 1) * Coefficient_angle * Fre_m;
            if (Sv_module.angle_ref > PI_DOUBLE)
            {
                Sv_module.angle_ref = Coefficient_angle * Fre_m;
                Tim1_int_count = 1;
            }
            calc_SV_Uabc(&Sv_module);
            sv_module_calc(&Sv_module, TIM1);
            // u16 compare = get_compare(Tim1_int_count, MAX_COUNT_TIM1);
            // TIM_SetCompare1(TIM1, compare);
        }

        TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    }
}
