/*
 * LKS071 TIMER0 基础实验，替换工程的 main.c 使用。
 * 已按官方外设库 V3.16 的接口核对；需在 Keil 中编译、下载验证。
 *
 * 工程需加入官方 source/lks32mc07x_tim.c，保留原来的 GPIO、SYS、NVR。
 * TIMER0_IRQHandler 全工程只能有一个定义：若 interrupt.c 已有同名函数，
 * 删除原空函数或合并处理，不能同时保留两个函数体。
 * 沿用已经验证的 Hardware_init()、LED 和按键引脚。
 */
#include "lks32mc07x_lib.h"
#include "hardware_init.h"

/* 改为 96U 可复做实验；时钟和计数门限会一起改变。 */
#define LAB_SYS_MHZ          48U
#define LAB_TIMER_TH         ((LAB_SYS_MHZ * 1000000UL / 8UL / 1000UL) - 1UL)

/* 中断写入，主循环读取，因此使用 volatile；单位：ms。 */
volatile uint32_t g_ms = 0U;

static void Lab_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin  = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIO0, &gpio);
    GPIO_ResetBits(GPIO0, GPIO_Pin_6 | GPIO_Pin_7);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin  = GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    /* P2.11/P2.12 不支持内部上拉，按键释放电平由板上外部电路保证。 */
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIO2, &gpio);
}

static void Lab_TIMER0_Init(void)
{
    TIM_TimerInitTypeDef timer;

    NVIC_DisableIRQ(TIMER0_IRQn);
    SYS_ModuleClockCmd(SYS_Module_TIMER0, ENABLE);
    TIM_TimerCmd(UTIMER0, DISABLE);

    /* 注意官方函数拼写是 StrutInit，不是 StructInit。 */
    TIM_TimerStrutInit(&timer);
    /* V3.16 的 StrutInit 没有填写 Timer_EN，这里必须显式设置。 */
    timer.Timer_EN       = DISABLE;
    timer.Timer_CENTER   = 0U;               /* 只向一个方向计数 */
    timer.Timer_DIR      = 0U;               /* 从 0 递增到 TH */
    timer.Timer_XCLK_EN  = 0U;               /* 使用内部系统时钟 */
    timer.Timer_ClockDiv = TIMER_CLK_DIV8;    /* 输入时钟除以 8 */
    timer.Timer_TH       = LAB_TIMER_TH;     /* 48MHz:5999；96MHz:11999 */
    timer.Timer_IRQEna   = Timer_IRQEna_ZC;   /* 只开启回零中断 */
    TIM_TimerInit(UTIMER0, &timer);

    /* 初始化函数未写 CNT，单独把计数器清零。 */
    UTIMER0->CNT = 0U;
    TIM_ClearIRQFlag(UTIMER0, TIMER_IF_CH0 | TIMER_IF_CH1 |
                             TIMER_IF_ZERO | TIMER_IF_FAIL);
    NVIC_ClearPendingIRQ(TIMER0_IRQn);
    NVIC_SetPriority(TIMER0_IRQn, 2U);
    NVIC_EnableIRQ(TIMER0_IRQn);
    TIM_TimerCmd(UTIMER0, ENABLE);
}

/* 名称与启动文件的中断向量表一致，由 CPU 自动调用。 */
void TIMER0_IRQHandler(void)
{
    if (TIM_GetIRQFlag(UTIMER0, TIMER_IF_ZERO) != 0U)
    {
        /* 先清标志，再处理。该寄存器通过“写 1 清零”。 */
        TIM_ClearIRQFlag(UTIMER0, TIMER_IF_ZERO);
        g_ms++;
    }
}

int main(void)
{
    uint32_t now;
    uint32_t last_led_ms = 0U;
    uint8_t led1_on = 0U;

    Hardware_init();

#if LAB_SYS_MHZ == 48U
    SYS_MclkChoice(SYS_MCLK_48M_RC);
#elif LAB_SYS_MHZ == 96U
    SYS_MclkChoice(SYS_MCLK_96M_RC);
#else
#error "LAB_SYS_MHZ must be 48U or 96U"
#endif

    Lab_GPIO_Init();
    Lab_TIMER0_Init();
    __enable_irq();

    while (1)
    {
        now = g_ms;

        /* 看时间是否到了，不在这里等待时间流逝。 */
        if ((uint32_t)(now - last_led_ms) >= 500U)
        {
            last_led_ms += 500U;
            led1_on = !led1_on;
            if (led1_on)
                GPIO_SetBits(GPIO0, GPIO_Pin_6);
            else
                GPIO_ResetBits(GPIO0, GPIO_Pin_6);
        }

        /* STOP 优先：按下熄灭 LED2，START 按下点亮 LED2。 */
        /* 松开保持状态；采用设定亮灭，暂不做按一次翻转及软件消抖。 */
        if (GPIO_ReadInputDataBit(GPIO2, GPIO_Pin_12) == 0U)
            GPIO_ResetBits(GPIO0, GPIO_Pin_7);
        else if (GPIO_ReadInputDataBit(GPIO2, GPIO_Pin_11) == 0U)
            GPIO_SetBits(GPIO0, GPIO_Pin_7);
    }
}
