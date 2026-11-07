/*
 * LKS071 SYS实验：相同循环延时，比较48MHz和96MHz的LED闪烁速度。
 *
 * 上电默认48MHz：LED1(P0.6)闪烁，LED2熄灭。
 * START(P2.11，低有效)：选择48MHz，观察LED1。
 * STOP (P2.12，低有效)：选择96MHz，观察LED2。
 * 按住约1~2秒再释放；主循环使用阻塞延时，短按可能漏检。
 *
 * 使用原工程的Hardware_init()、GPIO驱动、SYS驱动和NVR库。
 * 本实验使用固定循环次数延时，不使用可能经过时钟补偿的SoftDelay()。
 */
#include "lks32mc07x_lib.h"
#include "hardware_init.h"

/* 两种主频始终使用同一个计数值。可整体调整它改变闪烁速度。 */
#define BLINK_DELAY_COUNT  2000000UL
#define KEY_SETTLE_COUNT     20000UL

/* 仅用于保存当前实验模式，不需要打开Watch观察。 */
static uint32_t clock_mhz = 48U;

/*
 * 固定循环次数延时，没有毫秒/微秒单位。
 * volatile计数变量和__NOP()保留循环中的必要操作。
 * 在相同编译设置下，主频越高，完成这段循环所需时间通常越短。
 */
static void BusyLoopDelay(uint32_t count)
{
    volatile uint32_t i;

    for (i = 0U; i < count; i++)
    {
        __NOP();
    }
}

/* 使用官方时钟切换函数；不直接改写时钟寄存器。 */
static void SelectClock(uint32_t mhz)
{
    GPIO_ResetBits(GPIO0, GPIO_Pin_6 | GPIO_Pin_7);

    if (mhz == 48U)
    {
        SYS_MclkChoice(SYS_MCLK_48M_RC);
        clock_mhz = 48U;
    }
    else
    {
        SYS_MclkChoice(SYS_MCLK_96M_RC);
        clock_mhz = 96U;
    }
}

int main(void)
{
    GPIO_InitTypeDef gpio;
    uint32_t active_led;
    unsigned char led_on = 0U;

    Hardware_init();

    /* 两个LED输出，沿用已验证的引脚。 */
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
	  gpio.GPIO_PODEna = DISABLE;           // 不启用开漏，普通推挽输出
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;     // 不启用内部上拉
    GPIO_Init(GPIO0, &gpio);

    /* START和STOP输入，沿用板上已有的按键电路。 */
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_IN;
	  gpio.GPIO_PuPd = GPIO_PuPd_UP; //启用内部上拉，默认是GPIO_PuPd_NOPULL，内部不上拉

    GPIO_Init(GPIO2, &gpio);

    /* 进入48MHz模式，不设置断点，直接运行。 */
    SelectClock(48U);

    while (1)
    {
        if (GPIO_ReadInputDataBit(GPIO2, GPIO_Pin_11) == 0)
        {
            /* 按住时等待，松开后切换，观察时应释放按键。 */
            while (GPIO_ReadInputDataBit(GPIO2, GPIO_Pin_11) == 0)
            {
            }
           // BusyLoopDelay(KEY_SETTLE_COUNT);
            SelectClock(48U);
            led_on = 0U;
        }
        else if (GPIO_ReadInputDataBit(GPIO2, GPIO_Pin_12) == 0)
        {
            while (GPIO_ReadInputDataBit(GPIO2, GPIO_Pin_12) == 0)
            {
            }
            //BusyLoopDelay(KEY_SETTLE_COUNT);
            SelectClock(96U);
            led_on = 0U;
        }

        /* 48MHz只闪LED1；96MHz只闪LED2。 */
        active_led = (clock_mhz == 48U) ? GPIO_Pin_6 : GPIO_Pin_7;

        led_on = !led_on;
        if (led_on)
        {
            GPIO_SetBits(GPIO0, active_led);
        }
        else
        {
            GPIO_ResetBits(GPIO0, active_led);
        }

        /* 每次亮灭切换后，执行完全相同次数的循环。 */
        BusyLoopDelay(BLINK_DELAY_COUNT);
    }
}
