

#include "ch32v30x.h"
#include "EEMU_PWM.h"
#include <string.h>
#include <math.h>

#define PIDIV2 1.570796
#define THREEPIDIV2 4.71238898
#define PIDIV6 0.52359878
#define SEVENPIDIV6 3.6651914
#define FIVEPIDIV6 2.617994
#define ELEVPIDIV6 5.7595865


u16 flag_outa_0 = 0, flag_into_0 = 0;
u16 flag_outa_1 = 0, flag_into_1 = 0;
u16 flag_outa_2 = 0, flag_into_2 = 0; //�ȽϼĴ�����־λ
u16 flag_outa_3 = 0, flag_into_3 = 0; //�ȽϼĴ�����־λ

u16 CMPA_TEMP1, CMPA_TEMP2, CMPA_TEMP3, CMPA_TEMP4;


void svpwm_init(u16 sw_fre, u16 dead_time);
void sv_module_calc(SV_MODULE_handle v, TIM_TypeDef *TIMx);
void sv_module_init(SV_MODULE_handle v);
void update_compare(TIM_TypeDef *TIMx);


/*********************************************************************
 * @fn      SVPWM_Init
 *
 * @brief   SVPWM_module_Init based on Tim1. ��ʱ��ʱ��Ƶ��Ϊ1e6Hz.
  *          ��ʼ��TIM1��4·PWM
  *          ��4·ֻ��1���źţ�������·����1�ԡ�
  *          todo   �ر���������ֹCH2~4�ж�
 *
 * @param   sw_fre - sw_fre = ��ʱ��ʱ��Ƶ�� / ����Ƶ��
 *          dead_time - dead_time = ʵ������ʱ�� * 144e6 / k - m,
 *          k = 16, ʵ��ʱ��ʱ���ѡ4~7us������kֵ��Ҫ�޸ĺ�����120�С�
 * @return  none
 */
void svpwm_init(u16 sw_fre, u16 dead_time)
{
    u16 psc = 144 - 1;
    if (TIM1_PERIOD == 1e-6)
    {
        psc = 144 - 1; // ������ʱ�ӷ�Ƶ��ʹ������ʱ��Ϊ1us
    }
    u16 ccp = sw_fre >> 1;  // ��ʼռ�ձ���Ϊ50%
    // 0xE0 => k = 16, m = 32; 0xC0 => k = 8, m = 32;
    // 0x80 => k = 2, m = 64; 0x00 => k = 1, m = 0;
    u16 k_mask = 0xE0;

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_TIM1, ENABLE);

    /* TIM1_CH1 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* TIM1_CH1N */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* TIM1_CH2 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* TIM1_CH2N */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* TIM1_CH3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* TIM1_CH3N */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* TIM1_CH4 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period = sw_fre;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    // ��������������Ϻ����¼���, �Ƚ��жϱ�־λ�ڼ��������¼���ʱ������
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    // TIM_OCMode_PWM1
    // ���ϼ���ʱ���������ڱȽϲ���Ĵ�����ֵʱ��ͨ�� x Ϊ��Ч��ƽ
    // ���¼���ʱ���������ڱȽϲ���Ĵ�����ֵʱ��ͨ�� X Ϊ��Ч��ƽ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    memset(&TIM_OCInitStructure, 0, sizeof(TIM_OCInitTypeDef));
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);

    memset(&TIM_OCInitStructure, 0, sizeof(TIM_OCInitTypeDef));
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);

    memset(&TIM_OCInitStructure, 0, sizeof(TIM_OCInitTypeDef));
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

//    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;
//    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;
//    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
//
//    u16 dt = k_mask & dead_time;
//    TIM_BDTRInitStructure.TIM_DeadTime = dt;
//    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
//    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
//    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
//    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    // BDTR ���� OCx �� OCxN ��Ϊ���
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // �����Ƚϲ���Ĵ�����Ԥװ�ع���
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // ��ARPEλʹ�ܣ�ʹ���Զ���װֵ�Ĵ�����ATRLR���������ֵ���ᱻװ�������
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    // ʹ��CH1~CH4�ж�
    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
//    TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
//    TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);
//    TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);

    // ��ʼ����֮ǰ,�� UGλ����ʼ�����мĴ���
    TIM1->SWEVGR |= TIM_UG;

    TIM_Cmd(TIM1, ENABLE);
}


void sv_module_calc(SV_MODULE_handle v, TIM_TypeDef *TIMx)
{
	float Tas, Tbs, Tcs, Tmax, Tmin, Tmid, Teff, Teff1;
	float Toffset, Tga, Tgb, Tgc;
	float t_min_k1;

	t_min_k1 = 1 - v->T_MIN_DELTA_K;
	/*******************************/
	Tas = v->Ua;
	Tbs = v->Ub;
	Tcs = v->Uc;
	// �Լ����ʱ��ֵ����������
	if (Tas > Tbs)
	{
		Tmax = Tas;
		Tmin = Tbs;
	}
	else
	{
		Tmax = Tbs;
		Tmin = Tas;
	}
	if (Tmax > Tcs)
	{
		if (Tmin > Tcs)
		{
			Tmid = Tmin;
			Tmin = Tcs;
		}
		else
			Tmid = Tcs;
	}
	else
	{
		Tmid = Tmax;
		Tmax = Tcs;
	}
	// over
	Teff = Tmax - Tmin;
	if (Teff > 1.0f)
	{
		Teff1 = 1.0f / Teff;
		Tas = Tas * Teff1;
		Tbs = Tbs * Teff1;
		Tcs = Tcs * Teff1;

		Tmax = Tmax * Teff1;
		Tmin = Tmin * Teff1;
		Tmid = Tmid * Teff1;
	}
	//
	Toffset = 0.5f * (1.0f - Tmax - Tmin);
	Tga = Tas + Toffset;
	Tgb = Tbs + Toffset;
	Tgc = Tcs + Toffset;

	Tmax += Toffset;
	Tmin += Toffset;
	Tmid += Toffset;
	//�õ����ع�����ʱ��
	v->Ta = Tga;
	v->Tb = Tgb;
	v->Tc = Tgc;
	/****/
	//������������, �����϶��������Լ����ӵĵ�ͨѹ����С���Բ���
    //if(v->ia<0)  v->tba=-v->tbconst;
    //else if(v->ia>0) v->tba=v->tbconst;
    //
    //if(v->ib<0)  v->tbb=-v->tbconst;
    //else if(v->ib>0) v->tbb=v->tbconst;
    //
    //if(v->ic<0)  v->tbc=-v->tbconst;
    //else if(v->ic>0) v->tbc=v->tbconst;
	if ((v->angle_ref <= THREEPIDIV2) && (v->angle_ref > PIDIV2))
		v->tba = -v->tbconst;
	else
		v->tba = v->tbconst;

	if ((v->angle_ref <= SEVENPIDIV6) && (v->angle_ref > PIDIV6))
		v->tbb = v->tbconst;
	else
		v->tbb = -v->tbconst;

	if ((v->angle_ref <= ELEVPIDIV6) && (v->angle_ref > FIVEPIDIV6))
		v->tbc = v->tbconst;
	else
		v->tbc = -v->tbconst;

	TIM_SetCompare4(TIMx, v->Tc);

	v->Ta += v->tba;
	v->Tb += v->tbb;
	v->Tc += v->tbc;
	//�������Ƚ�������,�������ĵ���ϵ��
	if (v->Ta < v->T_MIN_DELTA_K)
		v->Ta = v->T_MIN_DELTA_K;
	if (v->Ta > t_min_k1)
		v->Ta = t_min_k1;

	if (v->Tb < v->T_MIN_DELTA_K)
		v->Tb = v->T_MIN_DELTA_K;
	if (v->Tb > t_min_k1)
		v->Tb = t_min_k1;

	if (v->Tc < v->T_MIN_DELTA_K)
		v->Tc = v->T_MIN_DELTA_K;
	if (v->Tc > t_min_k1)
		v->Tc = t_min_k1;

	//������ȽϼĴ��������ֵ
	u16 compare1 = (1 - v->Ta) * (v->Tc);
	u16 compare2 = (1 - v->Tb) * (v->Tc);
	u16 compare3 = (1 - v->Tc) * (v->Tc);
	// u16 compare1 = FocCtrll.Syn_Deta;
	// u16 compare2 = FocCtrll.Flux_P;
	// u16 compare3 = FocCtrll.Flux_U;
	TIM_SetCompare1(TIMx, compare1);
	TIM_SetCompare2(TIMx, compare2);
	TIM_SetCompare3(TIMx, compare3);

	/**************/
	update_compare(TIMx);
	/*end*/
}


void update_compare(TIM_TypeDef *TIMx)
{
    /*--------------------pwm ch1--------------------*/
    if ((flag_outa_0 == 1) || (flag_into_0 == 1))
    {
        flag_outa_0 = 0;
        flag_into_0 = 0;
        // �ȽϼĴ���װ�ط�ʽ����Ϊ=0ʱװ�أ�ʹ��Ԥװ��
        // EPwm1Regs.CMPCTL.all = CMPCTL_INIT_STATE;
        TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

        // ���ϼ�������ʱ������������CMPAʱpwm�����
        // ���¼�������ʱ������������CMPAʱpwm�����
        // EPwm1Regs.AQCTLA.all = AQCTLA_INIT_STATE;
        u16 tmpccmrx = TIMx->CHCTLR1;
        tmpccmrx |= TIM_OCMode_PWM1;
        TIMx->CHCTLR1 = tmpccmrx;
    }
    if ((CMPA_TEMP1 == 0) && (TIMx->CH1CVR != 0))
    {
        // ��ǰ���жϺ�����ߵ�ƽ���Ƚϲ�������������ֵʱ��ֵ�ȽϼĴ�����ֵ
        //// ZRO = 1: ʱ������������0ʱʹepwma�����
        // EPwm1Regs.AQCTLA.bit.ZRO = 1;
        //// CAD = 0: ���¼���ʱ��ʱ������������CMPAʱ������
        // EPwm1Regs.AQCTLA.bit.CAD = 0; //�´ν��ж�ʱ�ָֻ�����������CAU���ֲ������Բ�Ӱ��
        u16 tmpccmrx = TIMx->CHCTLR1;
        tmpccmrx |= TIM_OCMode_Timing; // �Ƚϲ���Ĵ�����ֵ����ļ�������ıȽ�ֵ�� OC1REF��������
        TIMx->CHCTLR1 = tmpccmrx;

        // EPwm1Regs.CMPCTL.bit.LOADAMODE = 2;

        flag_outa_0 = 1;
    }
    if ((CMPA_TEMP1 != 0) && (TIMx->CH1CVR == 0))
    {
        //// ZRO = 2: ʱ������������0ʱʹepwma�����
        // EPwm1Regs.AQCTLA.bit.ZRO = 2;
        u16 tmpccmrx = TIMx->CHCTLR1;
        tmpccmrx |= TIM_OCMode_PWM1;
        TIMx->CHCTLR1 = tmpccmrx;

        flag_into_0 = 1;
    }
    // CMPA_TEMP1 = EPwm1Regs.CMPA.half.CMPA;
    CMPA_TEMP1 = TIMx->CH1CVR;
    /*-----------------------------------------------*/
    /*--------------------pwm ch3--------------------*/
    if ((flag_outa_2 == 1) || (flag_into_2 == 1))
    {
        flag_outa_2 = 0;
        flag_into_2 = 0;

        TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
        u16 tmpccmrx = TIMx->CHCTLR2;
        tmpccmrx |= TIM_OCMode_PWM1;
        TIMx->CHCTLR2 = tmpccmrx;
    }
    if ((CMPA_TEMP3 == 0) && (TIMx->CH3CVR != 0))
    {
        u16 tmpccmrx = TIMx->CHCTLR2;
        tmpccmrx |= TIM_OCMode_Timing;
        TIMx->CHCTLR2 = tmpccmrx;

        flag_outa_2 = 1;
    }
    if ((CMPA_TEMP3 != 0) && (TIMx->CH3CVR == 0))
    {
        u16 tmpccmrx = TIMx->CHCTLR2;
        tmpccmrx |= TIM_OCMode_PWM1;
        TIMx->CHCTLR2 = tmpccmrx;

        flag_into_2 = 1;
    }
    CMPA_TEMP3 = TIMx->CH3CVR;
    /*-----------------------------------------------*/
    /*--------------------pwm ch2--------------------*/
    if ((flag_outa_1 == 1) || (flag_into_1 == 1))
    {
        flag_outa_1 = 0;
        flag_into_1 = 0;

        TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
        u16 tmpccmrx = TIMx->CHCTLR1;
        tmpccmrx |= TIM_OCMode_PWM1 << 8;
        TIMx->CHCTLR1 = tmpccmrx;
    }
    if ((CMPA_TEMP2 == 0) && (TIMx->CH2CVR != 0))
    {
        u16 tmpccmrx = TIMx->CHCTLR1;
        tmpccmrx |= TIM_OCMode_Timing << 8;
        TIMx->CHCTLR1 = tmpccmrx;

        flag_outa_1 = 1;
    }
    if ((CMPA_TEMP2 != 0) && (TIMx->CH2CVR == 0))
    {
        u16 tmpccmrx = TIMx->CHCTLR1;
        tmpccmrx |= TIM_OCMode_PWM1 << 8;
        TIMx->CHCTLR1 = tmpccmrx;

        flag_into_1 = 1;
    }
    CMPA_TEMP2 = TIMx->CH2CVR;
    /*-----------------------------------------------*/
    /*--------------------pwm ch4--------------------*/
    if ((flag_outa_3 == 1) || (flag_into_3 == 1))
    {
        flag_outa_3 = 0;
        flag_into_3 = 0;

        TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
        u16 tmpccmrx = TIMx->CHCTLR2;
        tmpccmrx |= TIM_OCMode_PWM1 << 8;
        TIMx->CHCTLR2 = tmpccmrx;
    }
    if ((CMPA_TEMP4 == 0) && (TIMx->CH4CVR != 0))
    {
        u16 tmpccmrx = TIMx->CHCTLR2;
        tmpccmrx |= TIM_OCMode_Timing << 8;
        TIMx->CHCTLR2 = tmpccmrx;
        flag_outa_3 = 1;
    }
    if ((CMPA_TEMP4 != 0) && (TIMx->CH4CVR == 0))
    {
        u16 tmpccmrx = TIMx->CHCTLR2;
        tmpccmrx |= TIM_OCMode_PWM1 << 8;
        TIMx->CHCTLR2 = tmpccmrx;
        flag_into_3 = 1;
    }
    CMPA_TEMP4 = TIMx->CH4CVR;
}


void sv_module_init(SV_MODULE_handle v, u16 sw_fre)
{
    v->m_ref = 0;
    v->angle_ref = 0;
    v->Ua = 0;
    v->Ub = 0;
    v->Uc = 0;
    v->Fc = 1e6 / sw_fre;
    v->Tc = sw_fre * 1e-6;
    v->deltak = 0;
    v->DELTAKCONST = 0;
    v->vdc = 0;
    v->vdc1 = 0;
    v->ia = 0;
    v->ib = 0;
    v->ic = 0;
    v->C_CONST = 0;
    v->f_k = 0;
    v->f_k_p = 0;
    v->I_ZERO_CONST = 0;
    v->T_MIN_DELTA_K = 0;
    v->tbconst = 0;
    v->tba = 0;
    v->tbb = 0;
    v->tbc = 0;
    v->Ta = 0;
    v->Tb = 0;
    v->Tc = 0;
    v->calc = sv_module_calc;
}


void calc_angle(SV_MODULE_handle v)
{

}
