/*
 * ST M48T86 / Dallas DS12887 RTC driver
 * Copyright (c) 2006 Tower Technologies
 *
 * Author: Alessandro Zummo <a.zummo@towertech.it>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This drivers only supports the clock running in BCD and 24H mode.
 * If it will be ever adapted to binary and 12H mode, care must be taken
 * to not introduce bugs.
 */

#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/platform_data/rtc-m48t86.h>
#include <linux/bcd.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>

#define M48T86_REG_SEC		0x00
#define M48T86_REG_SECALRM	0x01
#define M48T86_REG_MIN		0x02
#define M48T86_REG_MINALRM	0x03
#define M48T86_REG_HOUR		0x04
#define M48T86_REG_HOURALRM	0x05
#define M48T86_REG_DOW		0x06 /* 1 = sunday */
#define M48T86_REG_DOM		0x07
#define M48T86_REG_MONTH	0x08 /* 1 - 12 */
#define M48T86_REG_YEAR		0x09 /* 0 - 99 */
#define M48T86_REG_A		0x0A
#define M48T86_REG_B		0x0B
#define M48T86_REG_C		0x0C
#define M48T86_REG_D		0x0D

#define M48T86_REG_B_H24	(1 << 1)
#define M48T86_REG_B_DM		(1 << 2)
#define M48T86_REG_B_SET	(1 << 7)
#define M48T86_REG_D_VRT	(1 << 7)

#define DRV_VERSION "0.1"

struct m48t86_rtc_private_data {
	void __iomem		*io_index;
	void __iomem		*io_data;

	struct rtc_device	*rtc;
	struct m48t86_ops	*ops;
};

static void m48t86_rtc_writebyte(struct device *dev, u8 value, u8 reg)
{
	struct m48t86_rtc_private_data *priv = dev_get_drvdata(dev);

	if (priv->ops) {
		priv->ops->writebyte((unsigned char)value,
					(unsigned long)reg);
		return;
	}

	writeb(reg, priv->io_index);
	writeb(value, priv->io_data);
}

static u8 m48t86_rtc_readbyte(struct device *dev, u8 reg)
{
	struct m48t86_rtc_private_data *priv = dev_get_drvdata(dev);

	if (priv->ops)
		return priv->ops->readbyte((unsigned long)reg);

	writeb(reg, priv->io_index);
	return readb(priv->io_data);
}

static int m48t86_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned char reg;

	reg = m48t86_rtc_readbyte(dev, M48T86_REG_B);

	if (reg & M48T86_REG_B_DM) {
		/* data (binary) mode */
		tm->tm_sec	= m48t86_rtc_readbyte(dev, M48T86_REG_SEC);
		tm->tm_min	= m48t86_rtc_readbyte(dev, M48T86_REG_MIN);
		tm->tm_hour	= m48t86_rtc_readbyte(dev, M48T86_REG_HOUR) & 0x3F;
		tm->tm_mday	= m48t86_rtc_readbyte(dev, M48T86_REG_DOM);
		/* tm_mon is 0-11 */
		tm->tm_mon	= m48t86_rtc_readbyte(dev, M48T86_REG_MONTH) - 1;
		tm->tm_year	= m48t86_rtc_readbyte(dev, M48T86_REG_YEAR) + 100;
		tm->tm_wday	= m48t86_rtc_readbyte(dev, M48T86_REG_DOW);
	} else {
		/* bcd mode */
		tm->tm_sec	= bcd2bin(m48t86_rtc_readbyte(dev, M48T86_REG_SEC));
		tm->tm_min	= bcd2bin(m48t86_rtc_readbyte(dev, M48T86_REG_MIN));
		tm->tm_hour	= bcd2bin(m48t86_rtc_readbyte(dev, M48T86_REG_HOUR) & 0x3F);
		tm->tm_mday	= bcd2bin(m48t86_rtc_readbyte(dev, M48T86_REG_DOM));
		/* tm_mon is 0-11 */
		tm->tm_mon	= bcd2bin(m48t86_rtc_readbyte(dev, M48T86_REG_MONTH)) - 1;
		tm->tm_year	= bcd2bin(m48t86_rtc_readbyte(dev, M48T86_REG_YEAR)) + 100;
		tm->tm_wday	= bcd2bin(m48t86_rtc_readbyte(dev, M48T86_REG_DOW));
	}

	/* correct the hour if the clock is in 12h mode */
	if (!(reg & M48T86_REG_B_H24))
		if (m48t86_rtc_readbyte(dev, M48T86_REG_HOUR) & 0x80)
			tm->tm_hour += 12;

	return rtc_valid_tm(tm);
}

static int m48t86_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned char reg;

	reg = m48t86_rtc_readbyte(dev, M48T86_REG_B);

	/* update flag and 24h mode */
	reg |= M48T86_REG_B_SET | M48T86_REG_B_H24;
	m48t86_rtc_writebyte(dev, reg, M48T86_REG_B);

	if (reg & M48T86_REG_B_DM) {
		/* data (binary) mode */
		m48t86_rtc_writebyte(dev, tm->tm_sec, M48T86_REG_SEC);
		m48t86_rtc_writebyte(dev, tm->tm_min, M48T86_REG_MIN);
		m48t86_rtc_writebyte(dev, tm->tm_hour, M48T86_REG_HOUR);
		m48t86_rtc_writebyte(dev, tm->tm_mday, M48T86_REG_DOM);
		m48t86_rtc_writebyte(dev, tm->tm_mon + 1, M48T86_REG_MONTH);
		m48t86_rtc_writebyte(dev, tm->tm_year % 100, M48T86_REG_YEAR);
		m48t86_rtc_writebyte(dev, tm->tm_wday, M48T86_REG_DOW);
	} else {
		/* bcd mode */
		m48t86_rtc_writebyte(dev, bin2bcd(tm->tm_sec), M48T86_REG_SEC);
		m48t86_rtc_writebyte(dev, bin2bcd(tm->tm_min), M48T86_REG_MIN);
		m48t86_rtc_writebyte(dev, bin2bcd(tm->tm_hour), M48T86_REG_HOUR);
		m48t86_rtc_writebyte(dev, bin2bcd(tm->tm_mday), M48T86_REG_DOM);
		m48t86_rtc_writebyte(dev, bin2bcd(tm->tm_mon + 1), M48T86_REG_MONTH);
		m48t86_rtc_writebyte(dev, bin2bcd(tm->tm_year % 100), M48T86_REG_YEAR);
		m48t86_rtc_writebyte(dev, bin2bcd(tm->tm_wday), M48T86_REG_DOW);
	}

	/* update ended */
	reg &= ~M48T86_REG_B_SET;
	m48t86_rtc_writebyte(dev, reg, M48T86_REG_B);

	return 0;
}

static int m48t86_rtc_proc(struct device *dev, struct seq_file *seq)
{
	unsigned char reg;

	reg = m48t86_rtc_readbyte(dev, M48T86_REG_B);

	seq_printf(seq, "mode\t\t: %s\n",
		 (reg & M48T86_REG_B_DM) ? "binary" : "bcd");

	reg = m48t86_rtc_readbyte(dev, M48T86_REG_D);

	seq_printf(seq, "battery\t\t: %s\n",
		 (reg & M48T86_REG_D_VRT) ? "ok" : "exhausted");

	return 0;
}

static const struct rtc_class_ops m48t86_rtc_ops = {
	.read_time	= m48t86_rtc_read_time,
	.set_time	= m48t86_rtc_set_time,
	.proc		= m48t86_rtc_proc,
};

/*
 * The RTC chip has 114 bytes upper bytes that can be used as user storage
 * space which we can use to test if the chip is present; for example it is
 * an optional feature and not all boards will have it present.
 *
 * I've used the method Technologic Systems use in their rtc7800.c example
 * for the detection.
 *
 * TODO: track down a guinea pig without an RTC to see if we can work out a
 *	better RTC detection routine
 */
static int m48t86_rtc_detect(struct device *dev)
{
	unsigned char tmp_rtc0, tmp_rtc1;

	tmp_rtc0 = m48t86_rtc_readbyte(dev, 126);
	tmp_rtc1 = m48t86_rtc_readbyte(dev, 127);

	m48t86_rtc_writebyte(dev, 0x00, 126);
	m48t86_rtc_writebyte(dev, 0x55, 127);
	if (m48t86_rtc_readbyte(dev, 127) == 0x55) {
		m48t86_rtc_writebyte(dev, 0xaa, 127);
		if (m48t86_rtc_readbyte(dev, 127) == 0xaa
				&& m48t86_rtc_readbyte(dev, 126) == 0x00) {
			m48t86_rtc_writebyte(dev, tmp_rtc0, 126);
			m48t86_rtc_writebyte(dev, tmp_rtc1, 127);

			return 0;
		}
	}

	dev_info(dev, "RTC not found\n");
	return -ENODEV;
}

static int m48t86_rtc_probe(struct platform_device *pdev)
{
	unsigned char reg;
	int err;
	struct m48t86_rtc_private_data *priv;
	struct resource *res_index = NULL;	/* Avoid GCC warning */
	struct resource *res_data = NULL;	/* Avoid GCC warning */

	/* Allocate memory for the device structure (and zero it) */
	priv = kzalloc(sizeof(struct m48t86_rtc_private_data), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	if (!pdev->dev.platform_data) {
		res_index = platform_get_resource_byname(
					pdev, IORESOURCE_MEM, "rtc_index");
		if (!res_index) {
			err = -ENXIO;
			goto out_free;
		}

		res_data = platform_get_resource_byname(
					pdev, IORESOURCE_MEM, "rtc_data");
		if (!res_data) {
			err = -ENXIO;
			goto out_free;
		}

		if (!request_mem_region(res_index->start,
					resource_size(res_index),
					dev_name(&pdev->dev))) {
			err = -EBUSY;
			goto out_free;
		}

		if (!request_mem_region(res_data->start,
					resource_size(res_data),
					dev_name(&pdev->dev))) {
			err = -EBUSY;
			goto out_release_index;
		}

		priv->io_index = ioremap(res_index->start,
					resource_size(res_index));
		if (!priv->io_index) {
			err = -EIO;
			goto out_release_data;
		}

		priv->io_data = ioremap(res_data->start,
					resource_size(res_data));
		if (!priv->io_data) {
			err = -EIO;
			goto out_io_index;
		}
	} else
		priv->ops = pdev->dev.platform_data;

	err = m48t86_rtc_detect(&pdev->dev);
	if (err) {
		if (!pdev->dev.platform_data)
			goto out_io_data;
		else
			goto out_free;
	}

	priv->rtc = rtc_device_register("m48t86",
				&pdev->dev, &m48t86_rtc_ops, THIS_MODULE);
	if (IS_ERR(priv->rtc)) {
		err = PTR_ERR(priv->rtc);
		if (!pdev->dev.platform_data)
			goto out_io_data;
		else
			goto out_free;
	}

	/* read battery status */
	reg = m48t86_rtc_readbyte(&pdev->dev, M48T86_REG_D);
	dev_info(&pdev->dev, "battery %s\n",
		(reg & M48T86_REG_D_VRT) ? "ok" : "exhausted");

	return 0;

out_io_data:
	iounmap(priv->io_data);
out_io_index:
	iounmap(priv->io_index);
out_release_data:
	release_mem_region(res_data->start, resource_size(res_data));
out_release_index:
	release_mem_region(res_index->start, resource_size(res_index));
out_free:
	platform_set_drvdata(pdev, NULL);
	kfree(priv);
	return err;
}

static int m48t86_rtc_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct m48t86_rtc_private_data *priv = platform_get_drvdata(pdev);

	if (priv->rtc)
		rtc_device_unregister(priv->rtc);

	if (priv->io_data) {
		iounmap(priv->io_data);
		res = platform_get_resource_byname(
					pdev, IORESOURCE_MEM, "rtc_data");
		release_mem_region(res->start, resource_size(res));
	}

	if (priv->io_index) {
		iounmap(priv->io_index);
		res = platform_get_resource_byname(
					pdev, IORESOURCE_MEM, "rtc_index");
		release_mem_region(res->start, resource_size(res));
	}

	platform_set_drvdata(pdev, NULL);

	kfree(priv);

	return 0;
}

static const struct of_device_id m48t86_rtc_match[] = {
	{ .compatible = "rtc-m48t86" },
	{},
};
MODULE_DEVICE_TABLE(of, m48t86_rtc_match);

static struct platform_driver m48t86_rtc_platform_driver = {
	.driver		= {
		.name		= "rtc-m48t86",
		.owner		= THIS_MODULE,
		.of_match_table	= m48t86_rtc_match,
	},
	.probe		= m48t86_rtc_probe,
	.remove		= m48t86_rtc_remove,
};

module_platform_driver(m48t86_rtc_platform_driver);

MODULE_AUTHOR("Alessandro Zummo <a.zummo@towertech.it>");
MODULE_DESCRIPTION("M48T86 RTC driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_ALIAS("platform:rtc-m48t86");
