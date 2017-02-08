
#include <SI_EFM8UB1_Register_Enums.h>
#include <string.h>
#include <stdint.h>
#include "stm32f0xx_hal.h"

#include "i2c.h"

#include "bsp.h"
#include "app.h"

extern I2C_HandleTypeDef hi2c1;

uint16_t  SMB_crc                   = 0;
data volatile uint8_t SMB_FLAGS     = 0;
uint8_t SMB_preflags                = 0;
uint8_t * SMB_write_ext_buf         = NULL;
data uint8_t  SMB_write_ext_len     = 0;

uint8_t smb_read (uint8_t addr, uint8_t* dest, uint8_t count)
{
    SMB_crc = 0;
    SMB_FLAGS = SMB_READ | SMB_BUSY | SMB_preflags;
    SMB_preflags = 0;

    HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(&hi2c1,
            addr,
            dest,
            count,
            100
        );
    if(ret == HAL_ERROR || ret == HAL_BUSY){
        SMB_FLAGS |= SMB_RECV_NACK;
        SMB_BUSY_CLEAR();
        return 0;
    }

    // update with length from packet
    // warning this is device specific to atecc508a
    if (dest[0] <= count){
        count = dest[0];
    }else{
        // truncated read
        SMB_FLAGS |= SMB_READ_TRUNC;
    }

    for (int i = 0; i < count-2; ++i)
    {
        SMB_crc = feed_crc(SMB_crc, dest[i]);
    }
    SMB_crc = reverse_bits(SMB_crc);
    SMB_BUSY_CLEAR();

    return count;
}


void smb_write (uint8_t addr, uint8_t* buf, uint8_t len)
{
    uint16_t count = len;
    static uint8_t writeBuf[256];
    SMB_crc = 0;
    SMB_FLAGS = SMB_WRITE | SMB_BUSY | SMB_preflags;
    SMB_preflags = 0;
    memcpy(writeBuf, buf, len);
    if(SMB_WRITING_EXT()){
        count += SMB_write_ext_len;
        memcpy(writeBuf+len, SMB_write_ext_buf, SMB_write_ext_len);
    }
    // dont crc first byte for atecc508a
    for (int i = 1; i < count; ++i)
    {
        SMB_crc = feed_crc(SMB_crc, writeBuf[i]);
    }
    SMB_crc = reverse_bits(SMB_crc);
    writeBuf[count++] = SMB_crc;
    writeBuf[count++] = SMB_crc>>8;

    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1,
            addr,
            writeBuf,
            count,
            100
        );
    if(ret == HAL_ERROR || ret == HAL_BUSY){
        SMB_FLAGS |= SMB_RECV_NACK;
    }
    SMB_BUSY_CLEAR();
}

void smb_set_ext_write( uint8_t* extbuf, uint8_t extlen)
{
    SMB_write_ext_len = extlen;
    SMB_write_ext_buf = extbuf;
    SMB_preflags |= SMB_WRITE_EXT;
}

// CRC-16 appropriate for a byte model interrupt routine.
uint16_t feed_crc(uint16_t crc, uint8_t b)
{
    crc ^= b;
    crc = crc & 1 ? (crc >> 1) ^ 0xa001 : crc >> 1;
    crc = crc & 1 ? (crc >> 1) ^ 0xa001 : crc >> 1;
    crc = crc & 1 ? (crc >> 1) ^ 0xa001 : crc >> 1;
    crc = crc & 1 ? (crc >> 1) ^ 0xa001 : crc >> 1;
    crc = crc & 1 ? (crc >> 1) ^ 0xa001 : crc >> 1;
    crc = crc & 1 ? (crc >> 1) ^ 0xa001 : crc >> 1;
    crc = crc & 1 ? (crc >> 1) ^ 0xa001 : crc >> 1;
    return crc & 1 ? (crc >> 1) ^ 0xa001 : crc >> 1;
}

// Resulting CRC should be reversed to be correct CRC-16
uint16_t reverse_bits(uint16_t crc)
{
    // efficient bit reversal for 16 bit int
    crc = (((crc & 0xaaaa) >> 1) | ((crc & 0x5555) << 1));
    crc = (((crc & 0xcccc) >> 2) | ((crc & 0x3333) << 2));
    crc = (((crc & 0xf0f0) >> 4) | ((crc & 0x0f0f) << 4));
    return (((crc & 0xff00) >> 8) | ((crc & 0x00ff) << 8));
}

void smb_init()
{
    SMB_FLAGS = 0;
}