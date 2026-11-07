/*
 * LKS071_V1.0 + MVPOWER_V5.0：UART实验2诊断版RXDBG V1，115200、8N1、轮询发送+接收中断。
 * 已按官方DevDriver V3.16核对接口；尚未在Keil或真实开发板上编译/运行。
 *
 * 用本文件内容替换原main.c，不要同时把两个含main()的文件加入工程。
 * 保留原Hardware_init()，工程加入官方：
 * lks32mc07x_gpio.c、lks32mc07x_sys.c、lks32mc07x_tim.c、
 * lks32mc07x_uart.c，以及原有NVR库和启动文件。
 * TIMER0_IRQHandler全工程只能定义一次：与interrupt.c中的同名函数合并。
 *
 * 官方核心板网络：UART0 TXD0=P1.0，RXD0=P0.15。
 * 官方MVPOWER_V5.0底板串口P4：1=GND，2=TXD0，3=RXD0。
 * 接线：P4-2 -> USB-TTL RXD；P4-3 <- USB-TTL TXD；P4-1 -> GND。
 * RXD电平必须兼容开发板TX高电平，不连接USB-TTL的VCC给板子重复供电。
 * 电脑以文本方式发送字符'1'点亮LED2，'0'熄灭LED2；忽略其他字符。
 * LED1心跳和每秒发送保留；本课不在主循环处理START/STOP，避免覆盖串口命令。
 * UART0_IRQHandler也只能定义一次，请检查interrupt.c中的空函数。
 */
#include "lks32mc07x_lib.h"
#include "hardware_init.h"

/* 改成96U后，定时器门限和UART波特率均随新的系统时钟重新计算。 */
#define LAB_SYS_MHZ       48U
#define LAB_TIMER_TH      ((LAB_SYS_MHZ * 1000000UL / 8UL / 1000UL) - 1UL)
#define LAB_UART_BAUD     115200UL

/* TIMER0中断写入，主循环读取，单位ms。 */
volatile uint32_t g_ms = 0U;

/* 接收诊断：由UART中断更新，在主循环输出。 */
volatile uint32_t g_uart_irq_count = 0U;
volatile uint32_t g_uart_rx_count  = 0U;
volatile uint32_t g_uart_error_count = 0U;
volatile uint8_t  g_uart_last_rx   = 0U;

static void Lab_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    /* LED1=P0.6、LED2=P0.7，沿用已验证的高电平点亮方式。 */
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin  = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIO0, &gpio);
    GPIO_ResetBits(GPIO0, GPIO_Pin_6 | GPIO_Pin_7);

    /* START=P2.11、STOP=P2.12，低电平表示按下。 */
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin  = GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL; /* 这两个引脚无内部上拉 */
    GPIO_Init(GPIO2, &gpio);
}

static void Lab_UART0_Init(void)
{
    GPIO_InitTypeDef gpio;
    UART_InitTypeDef uart;

    /* 发送仍轮询，接收改用中断。初始化时先关闭NVIC通道。 */
    NVIC_DisableIRQ(UART0_IRQn);

    UART_StructInit(&uart);
    uart.BaudRate     = LAB_UART_BAUD;
    uart.WordLength   = UART_WORDLENGTH_8b;
    uart.StopBits     = UART_STOPBITS_1b;
    uart.FirstSend    = UART_FIRSTSEND_LSB;
    uart.ParityMode   = UART_Parity_NO;
    uart.MultiDropEna = DISABLE;
    uart.Bit9Value    = 0U;
    uart.DUPLEX       = 0U;       /* 全双工，本课仅使用发送方向 */
    uart.Match485Addr = 0U;
    uart.IRQEna       = UART_IRQEna_RcvOver; /* 只开启接收完成中断 */
    uart.DMARE        = 0U;       /* 必须显式填写，V3.16的StructInit未赋值 */
    uart.RXD_INV      = DISABLE;
    uart.TXD_INV      = DISABLE;

    /* 库函数开启UART0模块时钟，并按SYS_ReadMcuClk()计算波特率分频。
     * 所以必须先调用SYS_MclkChoice()，再执行这里的UART_Init()。
     */
    UART_Init(UART0, &uart);
    UART_ClearIRQFlag(UART0, 0x01FFU);

    /* TX=P1.0：先把GPIO输出锁存值设高，再配置输出和UART复用。 */
    GPIO_SetBits(GPIO1, GPIO_Pin_0);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin    = GPIO_Pin_0;
    gpio.GPIO_Mode   = GPIO_Mode_OUT;
    gpio.GPIO_PODEna = DISABLE;  /* 推挽输出 */
    GPIO_Init(GPIO1, &gpio);
    GPIO_PinAFConfig(GPIO1, GPIO_PinSource_0, AF4_UART);

    /* RX=P0.15：本课需要连接USB-TTL的TXD，电平必须匹配。 */
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin  = GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP; /* P0.15支持内部上拉，悬空时保持空闲高 */
    GPIO_Init(GPIO0, &gpio);
    GPIO_PinAFConfig(GPIO0, GPIO_PinSource_15, AF4_UART);

    /* 外设接收中断已使能，再开放CPU的UART0中断通道。 */
    UART_ClearIRQFlag(UART0, UART_IF_RcvOver | UART_IF_RX_OV |
                            UART_IF_StopError | UART_IF_CheckError);
    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_SetPriority(UART0_IRQn, 1U);
    NVIC_EnableIRQ(UART0_IRQn);
}

static void Lab_UART0_SendByte(uint8_t data)
{
    /* 清上一个字节的发送完成标志，避免误认为新字节已发送完。 */
    UART_ClearIRQFlag(UART0, UART_IF_SendOver);
    UART_SendData(UART0, data);

    /* 读取原始发送标志。发送完成中断仍未使能，不能用UART_GetIRQFlag()
     * 此处短暂等待当前字节发送完成；TIMER0中断仍然可以执行。
     * 115200、8N1每字节约86.8us；不是DMA或中断式非阻塞发送。
     */
    while ((UART0->IF & UART_IF_SendOver) == 0U)
    {
    }
}

static void Lab_UART0_SendString(const char *text)
{
    while (*text != '\0')
    {
        Lab_UART0_SendByte((uint8_t)*text);
        text++;
    }
}

/* 以下输出函数只在主循环调用，不在接收中断里等待发送。 */
static void Lab_UART0_SendU32(uint32_t value)
{
    char digits[10];
    uint32_t length = 0U;

    do
    {
        digits[length++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);

    while (length != 0U)
        Lab_UART0_SendByte((uint8_t)digits[--length]);
}

static void Lab_UART0_SendHexByte(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    Lab_UART0_SendByte((uint8_t)hex[(value >> 4) & 0x0FU]);
    Lab_UART0_SendByte((uint8_t)hex[value & 0x0FU]);
}

static void Lab_TIMER0_Init(void)
{
    TIM_TimerInitTypeDef timer;

    NVIC_DisableIRQ(TIMER0_IRQn);
    SYS_ModuleClockCmd(SYS_Module_TIMER0, ENABLE);
    TIM_TimerCmd(UTIMER0, DISABLE);

    TIM_TimerStrutInit(&timer); /* 官方函数确实拼写为StrutInit */
    timer.Timer_EN       = DISABLE;
    timer.Timer_CENTER   = 0U;
    timer.Timer_DIR      = 0U;
    timer.Timer_XCLK_EN  = 0U;
    timer.Timer_ClockDiv = TIMER_CLK_DIV8;
    timer.Timer_TH       = LAB_TIMER_TH;
    timer.Timer_IRQEna   = Timer_IRQEna_ZC;
    TIM_TimerInit(UTIMER0, &timer);

    UTIMER0->CNT = 0U;
    TIM_ClearIRQFlag(UTIMER0, TIMER_IF_CH0 | TIMER_IF_CH1 |
                             TIMER_IF_ZERO | TIMER_IF_FAIL);
    NVIC_ClearPendingIRQ(TIMER0_IRQn);
    NVIC_SetPriority(TIMER0_IRQn, 2U);
    NVIC_EnableIRQ(TIMER0_IRQn);
    TIM_TimerCmd(UTIMER0, ENABLE);
}
/*
void TIMER0_IRQHandler(void)
{
    if (TIM_GetIRQFlag(UTIMER0, TIMER_IF_ZERO) != 0U)
    {
        TIM_ClearIRQFlag(UTIMER0, TIMER_IF_ZERO);
        g_ms++;
    }
}
*/

/* 接收中断入口：名称与启动文件一致。
 * 这里只读取、清标志、执行很短的LED动作，不在中断中等待或发送字符串。
 */
/*
void UART0_IRQHandler(void)
{
    uint8_t data;

    g_uart_irq_count++;

    if (UART_GetIRQFlag(UART0, UART_IF_RcvOver) != 0U)
    {
        data = (uint8_t)UART_ReadData(UART0);
        UART_ClearIRQFlag(UART0, UART_IF_RcvOver);
        g_uart_last_rx = data;
        g_uart_rx_count++;

       //这里比较的是ASCII字符：'1'=0x31，'0'=0x30。
        if (data == (uint8_t)'1')
            GPIO_SetBits(GPIO0, GPIO_Pin_7);
        else if (data == (uint8_t)'0')
            GPIO_ResetBits(GPIO0, GPIO_Pin_7);
        // 其他字符（包括回车、换行）忽略。
    }

    // 本课未使能错误中断；若接收时出现这些标志，清理后继续接收。
    // 此入门例用于单字符手动控制，不做连续数据流的完整帧解析。
    if ((UART0->IF & (UART_IF_RX_OV | UART_IF_StopError |
                     UART_IF_CheckError)) != 0U)
    {
        g_uart_error_count++;
        UART_ClearIRQFlag(UART0, UART_IF_RX_OV | UART_IF_StopError |
                                UART_IF_CheckError);
    }
}
*/

int main(void)
{
    uint32_t now;
    uint32_t last_led_ms = 0U;
    uint32_t last_uart_ms;
    uint8_t led1_on = 0U;
    uint32_t last_rx_report = 0U;
    uint32_t rx_count;
    uint32_t irq_count;
    uint32_t primask;
    uint8_t rx_byte;

    Hardware_init();

#if LAB_SYS_MHZ == 48U
    SYS_MclkChoice(SYS_MCLK_48M_RC);
#elif LAB_SYS_MHZ == 96U
    SYS_MclkChoice(SYS_MCLK_96M_RC);
#else
#error "LAB_SYS_MHZ must be 48U or 96U"
#endif

    Lab_GPIO_Init();
    Lab_UART0_Init();
    Lab_TIMER0_Init();
    __enable_irq();

    /* 上电发送一次；之后每秒发送UART OK，晚打开串口也能看到。 */
    Lab_UART0_SendString("RXDBG V1 READY: UART0 TX=P1.0 RX=P0.15, 115200 8N1\r\n");
    last_uart_ms = g_ms;

    while (1)
    {
        now = g_ms;

        /* LED1：亮500ms、灭500ms。 */
        if ((uint32_t)(now - last_led_ms) >= 500U)
        {
            last_led_ms += 500U;
            led1_on = !led1_on;
            if (led1_on)
                GPIO_SetBits(GPIO0, GPIO_Pin_6);
            else
                GPIO_ResetBits(GPIO0, GPIO_Pin_6);
        }

        /* 短暂复制接收快照，立即恢复原中断状态，再输出文字。
         * 只报告最新字节，用于手动逐个发送字符的诊断，不是接收队列。
         */
        primask = __get_PRIMASK();
        __disable_irq();
        rx_count  = g_uart_rx_count;
        irq_count = g_uart_irq_count;
        rx_byte   = g_uart_last_rx;
        __set_PRIMASK(primask);

        if (rx_count != last_rx_report)
        {
            last_rx_report = rx_count;
            Lab_UART0_SendString("RX=0x");
            Lab_UART0_SendHexByte(rx_byte);
            if (rx_byte == (uint8_t)'1')
                Lab_UART0_SendString(" LED2=ON\r\n");
            else if (rx_byte == (uint8_t)'0')
                Lab_UART0_SendString(" LED2=OFF\r\n");
            else
                Lab_UART0_SendString(" IGNORED\r\n");
        }

        if ((uint32_t)(now - last_uart_ms) >= 1000U)
        {
            last_uart_ms += 1000U;
            Lab_UART0_SendString("RXDBG V1 irq=");
            Lab_UART0_SendU32(irq_count);
            Lab_UART0_SendString(" bytes=");
            Lab_UART0_SendU32(rx_count);
            Lab_UART0_SendString(" err=");
            Lab_UART0_SendU32(g_uart_error_count);
            Lab_UART0_SendString(" pin=");
            Lab_UART0_SendU32(GPIO_ReadInputDataBit(GPIO0, GPIO_Pin_15));
            Lab_UART0_SendString(" IE=0x");
            Lab_UART0_SendHexByte((uint8_t)UART0->IE);
            Lab_UART0_SendString(" IF=0x");
            Lab_UART0_SendHexByte((uint8_t)UART0->IF);
            Lab_UART0_SendString("\r\n");
        }

        /* 本实验LED2由UART0接收中断控制，主循环不再覆盖它的状态。 */
    }
}