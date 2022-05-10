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
    svpwm_init(100, 5);
    sv_module_init(&Sv_module, 100);
    Sv_module.m_ref = 0.8f;
    Fre_m = 200;

    // 设置LED
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 使能中断
    Tim1_int_count = 0;
    Sv_module.angle_ref = (Tim1_int_count + 1) * TIM1_PERIOD * Fre_m * M_PI_2;
    calc_SV_Uabc(&Sv_module);
    sv_module_calc(&Sv_module, TIM1);

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
            Sv_module.angle_ref = (Tim1_int_count + 1) * TIM1_PERIOD * Fre_m * M_PI_2;
            if (Sv_module.angle_ref > M_PI_2)
            {
                Tim1_int_count = 1;
            }

            calc_SV_Uabc(&Sv_module);
            sv_module_calc(&Sv_module, TIM1);
        }

        TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    }
}
