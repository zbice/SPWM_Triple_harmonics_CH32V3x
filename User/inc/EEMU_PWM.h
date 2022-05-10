#ifndef _EEMU_PWM_H_
#define _EEMU_PWM_H_


#define TIM1_PERIOD 1e-6f
/*��������ʱ�䣬ֱ��ת����ֱ�ӷ�ʽ��ʱ�Ӷ���Ҫһ�²���*/
#define DBCNT_INIT_STATE 600 // 8us


/********************SPWM����*********************/
typedef struct
{
    float m_ref;     // Input: reference modulation factor
    float angle_ref; // Input: reference angle 0_2pi
    float Ua;
    float Ub;
    float Uc;
    float Fc;            //�ز�Ƶ��
    float Tc;            //�ز�����
    float deltak;        //��ѹ����ϵ��������
    float DELTAKCONST;   //��ѹ������ϵ������������
    float vdc;           //ֱ��ĸ�ߵ�ѹV
    float vdc1;          //�е��ѹV
    float ia;            // a���������ֵA
    float ib;            // b���������ֵA
    float ic;            // c���������ֵA
    float C_CONST;       //����ֵ����������UF
    float f_k;           //ƽ������
    float f_k_p;         //ƽ�����ӱ�������ϵ�����������ã�1/10000
    float I_ZERO_CONST;  //�������Ƚ�С��ʱ����Ϊ��Ϊ0������1/1000
    float T_MIN_DELTA_K; //���������С������б�Ҫ�����ƣ�����1/32768:��С����ռ�������ڵı���
    float tbconst;       //�̶�������ʱ�����
    float tba;           // a����������ʱ�䣬�������ò���1/32768
    float tbb;           // b����������ʱ�䣬�������ò���1/32768
    float tbc;           // c����������ʱ�䣬�������ò���1/32768
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

