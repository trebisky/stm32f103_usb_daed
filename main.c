/*
 */
#include "stm32f103_md.h"

#include "clock.h"
#include "gpio2.h"
// #include "printf.h"
// #include "usart.h"
#include "usb.h"

void printf ( char *, ... );
void serial_init ( void );
void serial_flush ( void );
int serial_pending ( void );
int serial_basic ( void );

void enum_handler ( void );
void enum_log_init ( void );
void enum_log_show ( void );

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

// static struct Ringbuffer usart1tx;

// void          USART1_IRQ_Handler(void) { usart_irq_handler(&USART1, &usart1tx); }
// static size_t u1puts(const char* buf, size_t len) { return usart_puts(&USART1, &usart1tx, buf, len); }
static size_t usb_puts(const char* buf, size_t len) { return usb_send(buf, len); }

void USB_LP_CAN1_RX0_IRQ_Handler(void) {
    // uint64_t now = cycleCount();
    uint32_t now = cycleCount() / 72;
    static int i = 0;

    led0_toggle ();

    enum_handler ();

    // cbprintf(u1puts, "%lld IRQ %i: %s\r\n", now / 72, i++, usb_state_str(usb_state()));

    // This output competes with the timer output (race)
    // printf ( "%d IRQ %d: %s\n", now, i++, usb_state_str(usb_state()) );

    uint8_t buf[64];
    size_t  len = usb_recv(buf, sizeof buf);

    if (len > 0) {
        // cbprintf(u1puts, "received %i: %*s\r\n", len, len, buf);
        printf ( "received %d: %s\n", len, buf);
    }
}

void TIM3_IRQ_Handler(void) {
    static int i = 0;
    static int ccount = 0;
    int s;

    if ((TIM3.SR & TIM_SR_UIF) == 0)
        return;

    TIM3.SR &= ~TIM_SR_UIF;

    led0_toggle ();

    i++;
    // cbprintf(u1puts, "USB %i: %s\r\n", i, usb_state_str(usb_state()));
    s = usb_state();
    if ( s == USB_CONFIGURED )
	ccount++;

    if ( ccount < 1 )
	printf ( "USB %d: %s\n", i, usb_state_str(usb_state()) );

    // cbprintf(usb_puts, "bingo %i\r\n", i++);
    // usb_puts ( "bingo\n" );
}

int main ( void )
{

    uint8_t enum_wait;

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
    // usart_init ( &USART1, 115200 );

    serial_init ();
    NVIC_EnableIRQ ( USART1_IRQn );

    printf ( " ----  Booting  ---\n" );

    // cbprintf(u1puts, "SWREV:%s\r\n", __REVISION__);
    // cbprintf(u1puts, "CPUID:%08lx\r\n", SCB.CPUID);
    // cbprintf(u1puts, "DEVID:%08lx:%08lx:%08lx\r\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
    // cbprintf(u1puts, "RESET:%02x%s%s%s%s%s%s\r\n", rf, rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "", rf & 0x20 ? " IWDG" : "",
    //          rf & 0x10 ? " SFT" : "", rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");

    printf("SWREV:%s\n", __REVISION__);
    printf("CPUID:%08x\n", SCB.CPUID);
    printf("DEVID:%08x:%08x:%08x\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
    printf("RESET:%02x%s%s%s%s%s%s\n", rf, rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "", rf & 0x20 ? " IWDG" : "",
             rf & 0x10 ? " SFT" : "", rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");

    // usart_wait(&USART1);

    // enable 1Hz TIM3
    TIM3.DIER |= TIM_DIER_UIE;
    TIM3.PSC = 7200 - 1;  // 72MHz/7200   = 10KHz
    TIM3.ARR = 10000 - 1; // 10KHz/10000  = 1Hz
    TIM3.CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM3_IRQn);

    enum_log_init ();

    usb_init();

    // cbprintf(u1puts, "USB after init: %s\r\n", usb_state_str(usb_state()));
    // printf("USB after init: %s\n", usb_state_str(usb_state()));
    // We see: USB after init: UNATTACHED

    /* Make sure all output above is done */
    // while ( serial_pending() )
// 	__WFI ();

    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    __enable_irq();
    printf ( "Wait\n" );

    for ( ;; ) {
	__enable_irq();
	if ( usb_state () == USB_CONFIGURED )
	    break;
        __WFI();
        __disable_irq();
    }

    printf ( "Wait done\n" );

    serial_basic ();
    enum_log_show ();
    enum_log_show ();

#ifdef notdef
    /* What was this crazy loop intended to do ?? */
    enum_wait = 1;

    for (;;) {
        __enable_irq();

        // usart_wait(&USART1);

        __WFI(); // wait for interrupt to change the state of any of the subsystems

	if ( enum_wait && usb_state () == USB_CONFIGURED ) {
	    enum_log_show ();
	    enum_wait = 0;
	}

        __disable_irq();

    } // forever
#endif

    return 0;
}

// THE END
