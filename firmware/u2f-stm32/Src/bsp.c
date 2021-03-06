#include "bsp.h"
#include "stm32f0xx_hal.h"

extern TSC_HandleTypeDef htsc;
extern UART_HandleTypeDef huart2;
extern RTC_HandleTypeDef hrtc;

void u2f_delay(uint32_t ms) {
    HAL_Delay(ms);
}

uint8_t U2F_BUTTON_IS_PRESSED(void)
{
    uint32_t val;

    HAL_TSC_IODischarge(&htsc, ENABLE);
    HAL_Delay(1);
    // HAL_TSC_IODischarge(&htsc, DISABLE); //dup in HAL_TSC_Start
    HAL_TSC_Start(&htsc);
    HAL_TSC_PollForAcquisition(&htsc);
    __HAL_TSC_CLEAR_FLAG(&htsc, (TSC_FLAG_EOA | TSC_FLAG_MCE));

//    val = HAL_TSC_GroupGetStatus(&htsc, TSC_GROUP2_IDX);
//    u2f_printlx("stat:",1,val);

    val = HAL_TSC_GroupGetValue(&htsc, TSC_GROUP2_IDX);
    // printf("btn:%d\r\n",val);

    return val < TOUCH_BTN_THRESHOLD;
}

void reboot_to_bootloader(void)
{
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0,0x32f0);
    HAL_PWR_DisableBkUpAccess();

    HAL_NVIC_SystemReset();
}

// Painfully lightweight printing routines
#ifdef U2F_PRINT

void putf(char c)
{
    HAL_UART_Transmit(&huart2,(uint8_t*)&c,1,100);
    watchdog();
}

void dump_hex(uint8_t* hex, uint8_t len)
{
    uint8_t i;
    for (i=0 ; i < len ; i++)
    {
        if (hex[i]<0x10)
        {
            putf('0');
        }
        u2f_putb(hex[i]);
    }
    u2f_prints("\r\n");
}


void u2f_prints(const char* d)
{
    while(*d)
    {
        // UART0 output queue
        putf(*d++);
    }
}

static void int2str_reduce_n(char ** snum, uint32_t copy, uint8_t n)
{
    do
    {
        copy /= n;
    }while(copy);
}


static const char * __digits = "0123456789abcdef";
static char __int2str_buf[9];

static void int2str_map_n(char ** snum, uint32_t i, uint8_t n)
{
    do
    {
        *--*snum = __digits[i % n];
        i /= n;
    }while(i);
}

#define dint2str(i)     __int2strn(i,10)
#define xint2str(i)     __int2strn(i,16)

char * __int2strn(int32_t i, uint8_t n)
{
    char * snum = __int2str_buf;
    if (i<0) *snum++ = '-';
    int2str_reduce_n(&snum, i, n);
    *snum = '\0';
    int2str_map_n(&snum, i, n);
    return snum;
}

void u2f_putd(int32_t i)
{
    u2f_prints(dint2str((int32_t)i));
}

void u2f_putx(int32_t i)
{
    u2f_prints(xint2str(i));
}

static void put_space()
{
    u2f_prints(" ");
}
static void put_line()
{
    u2f_prints("\r\n");
}

void u2f_printd(const char * tag, uint8_t c, ...)
{
    va_list args;
    u2f_prints(tag);
    va_start(args,c);
    while(c--)
    {
        u2f_putd((int32_t)va_arg(args, int));

    }
    put_line();
    va_end(args);
}

void u2f_printl(const char * tag, uint8_t c, ...)
{
    va_list args;
    u2f_prints(tag);
    va_start(args,c);
    while(c--)
    {
        u2f_putl(va_arg(args, int32_t));
        u2f_prints(" ");
    }
    put_line();
    va_end(args);
}

void u2f_printx(const char * tag, uint8_t c, ...)
{
    va_list args;
    u2f_prints(tag);
    va_start(args,c);
    while(c--)
    {
        u2f_putx((int32_t)va_arg(args, int));
        u2f_prints(" ");
    }
    put_line();
    va_end(args);
}

void u2f_printb(const char * tag, uint8_t c, ...)
{
    va_list args;
    u2f_prints(tag);
    va_start(args,c);
    while(c--)
    {
        u2f_putb(va_arg(args, int));
        put_space();
    }
    put_line();
    va_end(args);
}

void u2f_printlx(const char * tag, uint8_t c, ...)
{
    va_list args;
    u2f_prints(tag);
    va_start(args,c);
    while(c--)
    {
        u2f_putlx(va_arg(args, int32_t));
        put_space();
    }
    put_line();
    va_end(args);
}

#else




#endif

