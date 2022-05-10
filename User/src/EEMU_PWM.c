

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
u16 flag_outa_2 = 0, flag_into_2 = 0; //比较寄存器标志位
u16 flag_outa_3 = 0, flag_into_3 = 0; //比较寄存器标志位

u16 CMPA_TEMP1, CMPA_TEMP2, CMPA_TEMP3, CMPA_TEMP4;

void svpwm_init(u16 sw_fre, u16 dead_time);
void sv_module_calc(SV_MODULE_handle v, TIM_TypeDef *TIMx);
void sv_module_init(SV_MODULE_handle v, u16 sw_fre);
void update_compare(TIM_TypeDef *TIMx);
void calc_SV_Uabc(SV_MODULE_handle v);

/*********************************************************************
 * @fn      SVPWM_Init
 *
 * @brief   SVPWM_module_Init based on Tim1. 定时器时钟频率为1e6Hz.
 *          初始化TIM1的4路PWM
 *          第4路只有1个信号，其他三路各有1对。
 *
 * @param   sw_fre - sw_fre = 定时器时钟频率 / 开关频率
 *          dead_time - dead_time = 实际死区时间 * 144e6 / k - m,
 *          k = 16, 实际时区时间可选4~7us，其他k值需要修改函数第120行。
 * @return  none
 */
void svpwm_init(u16 sw_fre, u16 dead_time)
{
    u16 psc = 144 - 1;
    if (TIM1_PERIOD == 1e-6)
    {
        psc = 144 - 1; // 计数器时钟分频，使计数器时钟为1us
    }
    u16 arr = sw_fre >> 1; // 中央对齐时开关频率是定时器频率的2倍
    u16 ccp = sw_fre >> 2; // 初始占空比设为50%
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

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    // 计数器交替地向上和向下计数, 比较中断标志位在计数器向下计数时被设置
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    // TIM_OCMode_PWM1
    // 向上计数时计数器大于比较捕获寄存器的值时，通道 x 为有效电平
    // 向下计数时计数器大于比较捕获寄存器的值时，通道 X 为无效电平
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

    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;

    u16 dt = k_mask & dead_time;
    TIM_BDTRInitStructure.TIM_DeadTime = dt;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    // BDTR 允许 OCx 和 OCxN 设为输出
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // 开启比较捕获寄存器的预装载功能
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // 置ARPE位使能，使能自动重装值寄存器（ATRLR），此域的值将会被装入计数器
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    // 更新事件只允许来自计数器上溢或下溢
    TIM1->CTLR1 |= TIM_URS;
    // 开始计数之前,置UG位装载比较寄存器
    TIM1->SWEVGR |= TIM_UG;
    // 使能更新事件中断
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
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
    // 对计算的时间值进行排序处理
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
    //得到开关管作用时间
    v->Ta = Tga;
    v->Tb = Tgb;
    v->Tc = Tgc;
    /****/
    //考虑死区补偿, 初步认定二极管以及管子的导通压降很小忽略不计
    // if(v->ia<0)  v->tba=-v->tbconst;
    // else if(v->ia>0) v->tba=v->tbconst;
    //
    // if(v->ib<0)  v->tbb=-v->tbconst;
    // else if(v->ib>0) v->tbb=v->tbconst;
    //
    // if(v->ic<0)  v->tbc=-v->tbconst;
    // else if(v->ic>0) v->tbc=v->tbconst;
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

    TIM_SetCompare4(TIMx, v->Ts);

    v->Ta += v->tba;
    v->Tb += v->tbb;
    v->Tc += v->tbc;
    //对脉冲宽度进行限制,牺牲最大的调制系数
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

    //计算各比较寄存器的输出值
    u16 compare1 = (1 - v->Ta) * (v->Ts);
    u16 compare2 = (1 - v->Tb) * (v->Ts);
    u16 compare3 = (1 - v->Tc) * (v->Ts);
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
        // 比较寄存器装载方式设置为=0时装载，使能预装载
        // EPwm1Regs.CMPCTL.all = CMPCTL_INIT_STATE;
        TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

        // 向上计数，且时基计数器等于CMPA时pwm输出高
        // 向下计数，且时基计数器等于CMPA时pwm输出低
        // EPwm1Regs.AQCTLA.all = AQCTLA_INIT_STATE;
        u16 tmpccmrx = TIMx->CHCTLR1;
        tmpccmrx |= TIM_OCMode_PWM1;
        TIMx->CHCTLR1 = tmpccmrx;
    }
    if ((CMPA_TEMP1 == 0) && (TIMx->CH1CVR != 0))
    {
        // 当前进中断后，输出高电平，比较不动作，在周期值时赋值比较寄存器的值
        // ZRO = 1: 时基计数器等于0时使epwma输出低
        // EPwm1Regs.AQCTLA.bit.ZRO = 1;
        // CAD = 0: 向下计数时，时基计数器等于CMPA时不动作
        // EPwm1Regs.AQCTLA.bit.CAD = 0; //下次进中断时又恢复正常，由于CAU保持不变所以不影响
        u16 tmpccmrx = TIMx->CHCTLR1;
        tmpccmrx |= TIM_OCMode_Timing; // 比较捕获寄存器的值与核心计数器间的比较值对 OC1REF不起作用
        TIMx->CHCTLR1 = tmpccmrx;

        // EPwm1Regs.CMPCTL.bit.LOADAMODE = 2;

        flag_outa_0 = 1;
    }
    if ((CMPA_TEMP1 != 0) && (TIMx->CH1CVR == 0))
    {
        // ZRO = 2: 时基计数器等于0时使epwma输出高
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

        TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
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

        TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
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
    v->Ts = (sw_fre >> 1);
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

void calc_SV_Uabc(SV_MODULE_handle v)
{
    v->Ua = 0.5f * v->m_ref * (sinf(v->angle_ref) + sinf(3 * v->angle_ref) * 0.16666667f);
    v->Ub = 0.5f * v->m_ref * (sinf(v->angle_ref - 2.0943952f) + sinf(3 * v->angle_ref) * 0.16666667f);
    v->Uc = 0.5f * v->m_ref * (sinf(v->angle_ref + 2.0943952f) + sinf(3 * v->angle_ref) * 0.16666667f);
}

void calc_angle(SV_MODULE_handle v)
{
}
