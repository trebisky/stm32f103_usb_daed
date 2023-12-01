/*
 */
#include "stm32f103_md.h"

#include "clock.h"
#include "gpio2.h"
#include "printf.h"
#include "usart.h"
#include "usb.h"

/*
    STM32F103CB (LQFP48/LQFP48) Pin Assignments:

    Pin   Function     DIR   Electrical     Connected to
    ---   ---------    ---   -----------    ---------------------------------------
    PA9   USART1 TX    out   AF_PP 10MHz
    PA10  USART1 RX    in    PullUp
    PA11  USB D-       automatic
    PA12  USB D+       automatic
    PA13  SWDIO        in/out               ST-Link programmer
    PA14  SWCLK        in/out               ST-Link programmer

    PC13  LED0         out   OUT_OD 2MHz    On-board yellow LED

*/

/* TJT -- sorry, but the compiler doesn't like this
 */
#ifdef notdef
enum {
    USART1_TX_PIN = PA9,
    USART1_RX_PIN = PA10,
    LED0_PIN      = PC13,
};
#endif

#define USART1_TX_PIN PA9
#define USART1_RX_PIN PA10

void printf ( char *, ... );

#define MAPLE

#ifdef MAPLE
// Change LED pin for Maple
#define LED0_PIN      PA5
// for maple
#define USB_PIN      PC12

#else
#define LED0_PIN      PC13
#endif

/* clang-format off */
static struct gpio_config_t {
    enum GPIO_Pin  pins;
    enum GPIO_Conf mode;
} pin_cfgs[] = {
    {PAAll, Mode_IN}, // reset
    {PBAll, Mode_IN}, // reset
    {PCAll, Mode_IN}, // reset
    {USART1_TX_PIN, Mode_AF_PP_50MHz},
    {USART1_RX_PIN, Mode_IPU},
    // For Maple
    // {LED0_PIN, Mode_Out_OD_2MHz},
    {LED0_PIN, Mode_Out_PP_2MHz},
#ifdef MAPLE
    {USB_PIN, Mode_Out_PP_2MHz},
#endif
    {0, 0}, // sentinel
};
/* clang-format on */

static inline void led0_on(void) { digitalLo(LED0_PIN); }
static inline void led0_off(void) { digitalHi(LED0_PIN); }
static inline void led0_toggle(void) { digitalToggle(LED0_PIN); }

/* clang-format off */
enum { IRQ_PRIORITY_GROUPING = 5 }; // prio[7:6] : 4 groups,  prio[5:4] : 4 subgroups
struct {
    enum IRQn_Type irq;
    uint8_t        group, sub;
} irqprios[] = {
    {SysTick_IRQn,          0, 0},
    {USB_LP_CAN1_RX0_IRQn,  1, 0},
    {USART1_IRQn,           2, 0},
    {TIM3_IRQn,             3, 0},
    {None_IRQn, 0xff, 0xff},
};
/* clang-format on */

static struct Ringbuffer usart1tx;

void          USART1_IRQ_Handler(void) { usart_irq_handler(&USART1, &usart1tx); }
static size_t u1puts(const char* buf, size_t len) { return usart_puts(&USART1, &usart1tx, buf, len); }
static size_t usb_puts(const char* buf, size_t len) { return usb_send(buf, len); }

void USB_LP_CAN1_RX0_IRQ_Handler(void) {
    uint64_t now = cycleCount();
    led0_toggle();
    static int i = 0;
    cbprintf(u1puts, "%lld IRQ %i: %s\r\n", now / 72, i++, usb_state_str(usb_state()));
    uint8_t buf[64];
    size_t  len = usb_recv(buf, sizeof buf);
    if (len > 0) {
        cbprintf(u1puts, "received %i: %*s\r\n", len, len, buf);
    }
}

void TIM3_IRQ_Handler(void) {
    if ((TIM3.SR & TIM_SR_UIF) == 0)
        return;
    TIM3.SR &= ~TIM_SR_UIF;
    static int i = 0;
    cbprintf(u1puts, "USB %i: %s\r\n", i, usb_state_str(usb_state()));
    cbprintf(usb_puts, "bingo %i\r\n", i++);
}

int main(void) {

    uint8_t rf = (RCC.CSR >> 24) & 0xfc;
    RCC.CSR |= RCC_CSR_RMVF; // Set RMVF bit to clear the reset flags

    SysTick_Config(1U << 24); // tick at 72Mhz/2^24 = 4.2915 HZ

    NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING);
    for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
        NVIC_SetPriority(irqprios[i].irq, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, irqprios[i].group, irqprios[i].sub));
    }

    RCC.APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
    RCC.APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_USBEN;
    delay(10); // let all clocks and peripherals start up

    for (const struct gpio_config_t* p = pin_cfgs; p->pins; ++p) {
        gpioConfig(p->pins, p->mode);
    }

    gpioLock(PAAll);
    gpioLock(PBAll);
    gpioLock(PCAll);

    led0_off();

#ifdef MAPLE
    // tjt for maple
    digitalLo(USB_PIN);
#endif

    // usart_init(&USART1, 921600);
    usart_init ( &USART1, 115200 );

    cbprintf(u1puts, "SWREV:%s\r\n", __REVISION__);
    cbprintf(u1puts, "CPUID:%08lx\r\n", SCB.CPUID);
    cbprintf(u1puts, "DEVID:%08lx:%08lx:%08lx\r\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
    cbprintf(u1puts, "RESET:%02x%s%s%s%s%s%s\r\n", rf, rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "", rf & 0x20 ? " IWDG" : "",
             rf & 0x10 ? " SFT" : "", rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");
    usart_wait(&USART1);

    // enable 1Hz TIM3
    TIM3.DIER |= TIM_DIER_UIE;
    TIM3.PSC = 7200 - 1;  // 72MHz/7200   = 10KHz
    TIM3.ARR = 10000 - 1; // 10KHz/10000  = 1Hz
    TIM3.CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM3_IRQn);

    usb_init();

    cbprintf(u1puts, "USB after init: %s\r\n", usb_state_str(usb_state()));

    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    for (;;) {
        __enable_irq();

        usart_wait(&USART1);

        __WFI(); // wait for interrupt to change the state of any of the subsystems

        __disable_irq();

    } // forever

    return 0;
}

#ifdef notyet
/* TJT - give me a printf that takes care of the \r\n thing for me.
 *
 * What a mess!
 * I finally gave up when the compiler gave me inscrutable errors
 * about the use of rb_putcb.
 * It isn't worth it.
 *
 * Why didn't the original author deal with this \r\n business?
 */

#define STB_SPRINTF_MIN 512

typedef char * STBSP_SPRINTFCB( char const * buf, void * user, int len );
void stbsp_set_separators( char comma, char period );
int stbsp_vsprintfcb( STBSP_SPRINTFCB * callback, void * user, char * buf, char const * fmt, va_list va );


static void
fix_fmt ( char *old, char *new )
{
    while ( *old ) {
	if ( *old == '\n' )
	    *new++ = '\r';
	*new++ = *old++;
    }
    *new++ = '\0';
}

// copied from printf.c rather than making global
// a little signature adapter
static char *
rb_putcb(char *buf, void *user, int len) {
    puts_t *callback = (puts_t *)user;
    size_t  ln       = len;  // explicit cast
    if (callback(buf, len) < ln) {
        return NULL;
    }
    return buf;
}

void
printf ( char *fmt, ... )
{
    char fixed[80];
    va_list args;

    fix_fmt ( fmt, fixed );
    va_start(args, fmt);

    char b[STB_SPRINTF_MIN];
    stbsp_set_separators('\'', '.');
    (void) stbsp_vsprintfcb ( rb_putcb, u1puts, b, fixed, args);
    // char * (*)(char *, void *, int)

    va_end(args);
}

#ifdef notdef
/* From printf.c */
int cbprintf(puts_t *callback, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);

    char b[STB_SPRINTF_MIN];
    stbsp_set_separators('\'', '.');
    int rv = stbsp_vsprintfcb(rb_putcb, callback, b, fmt, ap);

    va_end(ap);
    return rv;
}
#endif
#endif

// THE END
