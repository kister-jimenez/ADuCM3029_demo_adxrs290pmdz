/*
 **
 ** Source file generated on September 7, 2020 at 18:29:35.	
 **
 ** Copyright (C) 2011-2020 Analog Devices Inc., All Rights Reserved.
 **
 ** This file is generated automatically based upon the options selected in 
 ** the Pin Multiplexing configuration editor. Changes to the Pin Multiplexing
 ** configuration should be made by changing the appropriate options rather
 ** than editing this file.
 **
 ** Selected Peripherals
 ** --------------------
 ** SPI1 (SCLK, MISO, MOSI, CS_0, CS_1, CS_2, CS_3, RDY)
 ** UART0 (Tx, Rx)
 **
 ** GPIO (unavailable)
 ** ------------------
 ** P0_10, P0_11, P0_14, P1_06, P1_07, P1_08, P1_09, P1_10, P2_02, P2_11
 */

#include <sys/platform.h>
#include <stdint.h>

#define SPI1_SCLK_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<12))
#define SPI1_MISO_PORTP1_MUX  ((uint16_t) ((uint16_t) 1<<14))
#define SPI1_MOSI_PORTP1_MUX  ((uint32_t) ((uint32_t) 1<<16))
#define SPI1_CS_0_PORTP1_MUX  ((uint32_t) ((uint32_t) 1<<18))
#define SPI1_CS_1_PORTP2_MUX  ((uint32_t) ((uint32_t) 1<<22))
#define SPI1_CS_2_PORTP2_MUX  ((uint16_t) ((uint16_t) 2<<4))
#define SPI1_CS_3_PORTP1_MUX  ((uint32_t) ((uint32_t) 3<<20))
#define SPI1_RDY_PORTP0_MUX  ((uint32_t) ((uint32_t) 2<<28))
#define UART0_TX_PORTP0_MUX  ((uint32_t) ((uint32_t) 1<<20))
#define UART0_RX_PORTP0_MUX  ((uint32_t) ((uint32_t) 1<<22))

int32_t adi_initpinmux(void);

/*
 * Initialize the Port Control MUX Registers
 */
int32_t adi_initpinmux(void) {
    /* PORTx_MUX registers */
    *((volatile uint32_t *)REG_GPIO0_CFG) = SPI1_RDY_PORTP0_MUX | UART0_TX_PORTP0_MUX
     | UART0_RX_PORTP0_MUX;
    *((volatile uint32_t *)REG_GPIO1_CFG) = SPI1_SCLK_PORTP1_MUX | SPI1_MISO_PORTP1_MUX
     | SPI1_MOSI_PORTP1_MUX | SPI1_CS_0_PORTP1_MUX | SPI1_CS_3_PORTP1_MUX;
    *((volatile uint32_t *)REG_GPIO2_CFG) = SPI1_CS_1_PORTP2_MUX | SPI1_CS_2_PORTP2_MUX;

    return 0;
}

