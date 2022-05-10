#ifndef _EEMU_PWM_H_
#define _EEMU_PWM_H_


#define TIM1_PERIOD 1e-6f
/*定义死区时间，直接转矩与直接方式的时钟定标要一致才行*/
#define DBCNT_INIT_STATE 600 // 8us


/********************SPWM调制*********************/
typedef struct
{
    float m_ref;     // Input: reference modulation factor
    float angle_ref; // Input: reference angle 0_2pi
    float Ua;
    float Ub;
    float Uc;
    float Fc;            //载波频率
    float Tc;            //载波周期
    float deltak;        //电压波动系数，变量
    float DELTAKCONST;   //电压允许波动系数，常量参数
    float vdc;           //直流母线电压V
    float vdc1;          //中点电压V
    float ia;            // a相输出电流值A
    float ib;            // b相输出电流值A
    float ic;            // c相输出电流值A
    float C_CONST;       //电容值，常量参数UF
    float f_k;           //平衡因子
    float f_k_p;         //平衡因子比例控制系数，参数设置，1/10000
    float I_ZERO_CONST;  //当电流比较小的时候认为其为0，参数1/1000
    float T_MIN_DELTA_K; //对输出的最小脉宽进行必要的限制，参数1/32768:最小脉宽占整个周期的比例
    float tbconst;       //固定的死区时间参数
    float tba;           // a相死区补偿时间，参数设置参数1/32768
    float tbb;           // b相死区补偿时间，参数设置参数1/32768
    float tbc;           // c相死区补偿时间，参数设置参数1/32768
    float Ta;            // Output: reference phase-a switching function
    float Tb;            // Output: reference phase-b switching function
    float Tc;            // Output: reference phase-c switching function
    // Pointer to calculation function
    void (*calc)();
} SV_MODULE;

typedef SV_MODULE *SV_MODULE_handle;


inline void calc_SV_Ua(SV_MODULE_handle v)
{
    v->Ua = 0.5f * v->m_ref * (sinf(v->angle_ref) + sinf(3 * v->angle_ref) * 0.16666667f);
}


inline void calc_SV_Ub(SV_MODULE_handle v)
{
    v->Ub = 0.5f * v->m_ref * (sinf(v->angle_ref - 2.0943952f) + sinf(3 * v->angle_ref) * 0.16666667f);
}


inline void calc_SV_Uc(SV_MODULE_handle v)
{
    v->Uc = 0.5f * v->m_ref * (sinf(v->angle_ref + 2.0943952f) + sinf(3 * v->angle_ref) * 0.16666667f);
}


void svpwm_init(u16 sw_fre, u16 dead_time);
void sv_module_calc(SV_MODULE_handle v, TIM_TypeDef *TIMx);
void sv_module_init(SV_MODULE_handle v, u16 sw_fre);

#endif

