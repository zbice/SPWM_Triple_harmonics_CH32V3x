# 目录结构

- SVPWM_test
  - User
    - inc
      - EEMU_PWM.h: SVPWM相关函数的头文件
    - src
      - EEMU_PWM.c: SVPWM相关函数的源文件
    - main.c

# 函数说明

`void SVPWM_Init(u16 sw_fre, u16 dead_time)`

配置TIM1定时器时钟频率为1e6Hz，将TIM1的4个通道配置为PWM输出。

- sw_fre: sw_fre = 定时器时钟频率 / 开关频率
- dead_time: dead_time = 实际死区时间 \* 144e6 / k - m
  k和m取决于函数中的k_mask变量，可选0xE0，0xC0，0x80，0x00。
  0xE0 => k = 16, m = 32; 0xC0 => k = 8, m = 32;
  0x80 => k = 2, m = 64;   0x00 => k = 1, m = 0;



`void TIM1_PWMOut_Init_debug(u16 arr, u16 psc, u16 ccp)`

调试用，测试PWM周期。



`void sv_module_calc(SV_MODULE_handle v, TIM_TypeDef *TIMx)`

实现SVPWM，计算定时器中的比较器时间。

- v: SVPWM相关参数。
- TIMx: 使用的定时器。可选TIM1, TIM8, TIM9, TIM10。