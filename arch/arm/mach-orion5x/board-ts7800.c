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
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/mtd/nand.h>
#include <asm/io.h>
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

/*
 * FPGA NAND Flash
 */
void __iomem *ts7800_nand_ctrl;

static int ts7800_nand_probe(struct platform_device *pdev)
{
	struct resource *res;
	int err;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "nand_ctrl");
	if (!res)
		return -ENXIO;

	if (!request_mem_region(res->start, resource_size(res),
				dev_name(&pdev->dev))) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		err = -EBUSY;
		goto out;
	}

	ts7800_nand_ctrl = ioremap(res->start, resource_size(res));
	if (ts7800_nand_ctrl == NULL) {
		dev_err(&pdev->dev, "ioremap failed\n");
		err = -EIO;
		goto out_release_io;
	}

	return 0;

out_release_io:
	release_mem_region(res->start, resource_size(res));
out:
	return err;
}

static void ts7800_nand_remove(struct platform_device *pdev)
{
	struct resource *res;

	iounmap(ts7800_nand_ctrl);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "nand_ctrl");
	release_mem_region(res->start, resource_size(res));
}

static void ts7800_nand_cmd_ctrl(struct mtd_info *mtd, int cmd,
			unsigned int ctrl)
{
	struct nand_chip *this = mtd->priv;

	if (ctrl & NAND_CTRL_CHANGE) {
		unsigned char bits;

		bits = (ctrl & NAND_NCE) << 2;
		bits |= ctrl & NAND_CLE;
		bits |= (ctrl & NAND_ALE) >> 2;

		writeb((readb(ts7800_nand_ctrl) & ~0x7) | bits, ts7800_nand_ctrl);
	}

	if (cmd != NAND_CMD_NONE)
		writeb(cmd, this->IO_ADDR_W);
}

static int ts7800_nand_dev_ready(struct mtd_info *mtd)
{
	return readb(ts7800_nand_ctrl) & 0x20;
}

static void ts7800_nand_write_buf(struct mtd_info *mtd,
			const uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	void __iomem *io_base = chip->IO_ADDR_W;
	unsigned long off = ((unsigned long)buf & 3);
	int sz;

	if (off) {
		sz = min_t(int, 4 - off, len);
		writesb(io_base, buf, sz);
		buf += sz;
		len -= sz;
	}

	sz = len >> 2;
	if (sz) {
		u32 *buf32 = (u32 *)buf;
		writesl(io_base, buf32, sz);
		buf += sz << 2;
		len -= sz << 2;
	}

	if (len)
		writesb(io_base, buf, len);
}

static void ts7800_nand_read_buf(struct mtd_info *mtd,
			uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	void __iomem *io_base = chip->IO_ADDR_R;
	unsigned long off = ((unsigned long)buf & 3);
	int sz;

	if (off) {
		sz = min_t(int, 4 - off, len);
		readsb(io_base, buf, sz);
		buf += sz;
		len -= sz;
	}

	sz = len >> 2;
	if (sz) {
		u32 *buf32 = (u32 *)buf;
		readsl(io_base, buf32, sz);
		buf += sz << 2;
		len -= sz << 2;
	}

	if (len)
		readsb(io_base, buf, len);
}

static struct platform_nand_data ts7800_nand_data = {
	/*
	 * The HW ECC offloading functions, used to give about a 9%
	 * performance increase for 'dd if=/dev/mtdblockX' and 5% for
	 * nanddump.  This all however was changed by git commit
	 * e6cf5df1838c28bb060ac45b5585e48e71bbc740 so now there is
	 * no performance advantage to be had so we no longer bother
	 */
	.ctrl	= {
		.probe			= ts7800_nand_probe,
		.remove			= ts7800_nand_remove,
		.cmd_ctrl		= ts7800_nand_cmd_ctrl,
		.dev_ready		= ts7800_nand_dev_ready,
		.write_buf		= ts7800_nand_write_buf,
		.read_buf		= ts7800_nand_read_buf,
	},
};

static int ts7800_platform_notifier(struct notifier_block *nb,
				  unsigned long event, void *__dev)
{
	struct device *dev = __dev;

	if (event != BUS_NOTIFY_ADD_DEVICE)
		return NOTIFY_DONE;

	if (of_device_is_compatible(dev->of_node, "ts,nand"))
		dev->platform_data = &ts7800_nand_data;

	return NOTIFY_OK;
}

static struct notifier_block ts7800_platform_nb = {
	.notifier_call = ts7800_platform_notifier,
};

void __init ts7800_init(void)
{
	/*
	 * Basic setup. Needs to be called early.
	 */
	orion5x_mpp_conf(ts7800_mpp_config);
	orion5x_eth_init(&ts7800_eth_data);

	bus_register_notifier(&platform_bus_type, &ts7800_platform_nb);
}
