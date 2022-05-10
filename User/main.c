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
u16 Fre_m = 200; // ���Ʋ�Ƶ��

void Interrupt_Init(void);
void TIM1_CC_IRQHandler(void)   __attribute__((interrupt("WCH-Interrupt-fast")));


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    sv_module_init(&Sv_module);
    svpwm_init(100, 5);

    // ����LED
    /*
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    */

    // ʹ���ж�
    Tim1_int_count = 0;
    Interrupt_Init();

    while(1)
    {

    }

}


void Interrupt_Init(void)
{
    NVIC_EnableIRQ(TIM1_CC_IRQn);
    // �����ȼ�0�������ȼ�1�������ȼ��ϸ�
    NVIC_SetPriority(TIM1_CC_IRQn, (0<<5) | (0x01<<4));
}


void TIM1_CC_IRQHandler(void)
{
    // ʹLED��˸����תPA0�����
    // GPIO_WriteBit(GPIOA, GPIO_Pin_0, (Led == 0) ? (Led = Bit_SET) : (Led = Bit_RESET));

    Tim1_int_count += 1;
    // ����Ƕ�
    Sv_module->angle_ref = Tim1_int_count * TIM1->CNT * TIM1_PERIOD * Fre_m * M_PI_2;
    if (Sv_module->angle_ref > M_PI_2)
    {
        Tim1_int_count = 1;
    }

    // CH1 A���ж�
    if (TIM_GetFlagStatus(TIM1, TIM_FLAG_CC1) == SET)
    {
        calc_SV_Ua(Sv_module);
        TIM_ClearFlag(TIM1, TIM_FLAG_CC1);
    }
    if (TIM_GetFlagStatus(TIM1, TIM_FLAG_CC2) == SET)
    {
        calc_SV_Ub(Sv_module);
        TIM_ClearFlag(TIM1, TIM_FLAG_CC2);
    }
    if (TIM_GetFlagStatus(TIM1, TIM_FLAG_CC3) == SET)
    {
        calc_SV_Uc(Sv_module);
        TIM_ClearFlag(TIM1, TIM_FLAG_CC3);
    }
    if (TIM_GetFlagStatus(TIM1, TIM_FLAG_CC4) == SET)
    {
        TIM_ClearFlag(TIM1, TIM_FLAG_CC4);
    }

    sv_module_calc(&Sv_module, TIM1);
}
