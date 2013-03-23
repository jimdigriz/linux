/*
 * Copyright 2013 Alexander Clouter <alex@digriz.org.uk>
 *
 * arch/arm/mach-orion5x/board-ts7800.c
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mv643xx_eth.h>
#include "common.h"
#include "mpp.h"

static struct mv643xx_eth_platform_data ts7800_eth_data = {
	.phy_addr	= MV643XX_ETH_PHY_ADDR(0),
};

static unsigned int ts7800_mpp_config[] __initdata = {
	MPP0_UNUSED,
	MPP1_GPIO,		/* JTAG Clock */
	MPP2_GPIO,		/* JTAG Data In */
	MPP3_GPIO,		/* Lat ECP2 256 FPGA - PB2B */
	MPP4_GPIO,		/* JTAG Data Out */
	MPP5_GPIO,		/* JTAG TMS */
	MPP6_GPIO,		/* Lat ECP2 256 FPGA - PB31A_CLK4+ */
	MPP7_GPIO,		/* Lat ECP2 256 FPGA - PB22B */
	MPP8_UNUSED,
	MPP9_UNUSED,
	MPP10_UNUSED,
	MPP11_UNUSED,
	MPP12_UNUSED,
	MPP13_UNUSED,
	MPP14_UNUSED,
	MPP15_UNUSED,
	MPP16_UART,		/* UART1 RxD */
	MPP17_UART,		/* UART1 TxD */
	MPP18_UART,		/* UART1 CTS */
	MPP19_UART,		/* UART1 RTS */
	/*
	 * MPP[20] PCI Clock Out 1
	 * MPP[21] PCI Clock Out 0
	 * MPP[22] Unused
	 * MPP[23] Unused
	 * MPP[24] Unused
	 * MPP[25] Unused
	 */
	0,
};

void __init ts7800_init(void)
{
	/*
	 * Basic setup. Needs to be called early.
	 */
	orion5x_mpp_conf(ts7800_mpp_config);
	orion5x_eth_init(&ts7800_eth_data);
}
