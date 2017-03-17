#ifndef BSP_H__
#define BSP_H__ 

#include <SI_EFM8UB1_Register_Enums.h>
#include <stdint.h>
#include "descriptors.h"
#include "app.h"

#define TOUCH_BTN_THRESHOLD 0x1500

#define get_ms() HAL_GetTick()

uint8_t U2F_BUTTON_IS_PRESSED(void);

#define watchdog()

void u2f_delay(uint32_t ms);
void usb_write(uint8_t* buf, uint8_t len);

void reboot_to_bootloader(void);

#ifdef U2F_PRINT

    void dump_hex(uint8_t* hex, uint8_t len);

    void u2f_putd(int32_t i);
    void u2f_putx(int32_t i);

#define u2f_putb(x) u2f_putx((uint8_t) (x))
#define u2f_putl(x) u2f_putd((uint32_t) (x))
#define u2f_putlx(x)    u2f_putx((uint32_t) (x))

    void u2f_prints(const char * str);
    void u2f_printb(const char * tag, uint8_t c, ...);
    void u2f_printd(const char * tag, uint8_t c, ...);
    void u2f_printx(const char * tag, uint8_t c, ...);
    void u2f_printl(const char * tag, uint8_t c, ...);
    void u2f_printlx(const char * tag, uint8_t c, ...);

#else

    #define u2f_printx(...)
    #define u2f_printb(...)
    #define u2f_printlx(...)
    #define u2f_printl(...)
    #define u2f_printd(...)
    #define u2f_prints(...)

    #define u2f_putx(...)
    #define u2f_putb(...)
    #define u2f_putl(...)
    #define u2f_putlx(...)

    #define putf(...)
    #define dump_hex(...)

#endif

#endif
