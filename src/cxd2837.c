// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * cxd2837er.c
 *
 * Sony digital demodulator driver for
 *	CXD2837ER - DVB-T/T2/C
 *
 * Copyright 2012 Sony Corporation
 * Copyright (C) 2014 NetUP Inc.
 * Copyright (C) 2014 Sergey Kozlov <serjk@netup.ru>
 * Copyright (C) 2014 Abylay Ospan <aospan@netup.ru>
 */

#include <stdint.h>
#include <stdio.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <sys/time.h>
#endif

#include "rtlsdr_i2c.h"
#include "rtl-sdr.h"
#include "cxd2837.h"

#define dev_err(dev, fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)
#define dev_warn(dev, fmt, ...) printf(fmt, ##__VA_ARGS__)
#define dev_dbg(dev, fmt, ...) {}//printf(fmt, ##__VA_ARGS__)

#define MAX_WRITE_REGSIZE	16
#define	EINVAL		22	/* Invalid argument */

#define u8  uint8_t
#define u32 uint32_t
#define u16 uint16_t
#define u64 uint64_t

enum fe_delivery_system {
	SYS_UNDEFINED,
	SYS_DVBC_ANNEX_A,
	SYS_DVBC_ANNEX_B,
	SYS_DVBT,
	SYS_DSS,
	SYS_DVBS,
	SYS_DVBS2,
	SYS_DVBH,
	SYS_ISDBT,
	SYS_ISDBS,
	SYS_ISDBC,
	SYS_ATSC,
	SYS_ATSCMH,
	SYS_DTMB,
	SYS_CMMB,
	SYS_DAB,
	SYS_DVBT2,
	SYS_TURBO,
	SYS_DVBC_ANNEX_C,
};

static int cxd2841er_write_regs(struct cxd2841er_priv *priv,
				uint8_t addr, uint8_t reg, const uint8_t *data, int len)
{
	uint8_t buf[MAX_WRITE_REGSIZE];
	int rc;
	uint8_t i2c_addr = (addr == I2C_SLVX ?
		priv->i2c_addr_slvx : priv->i2c_addr_slvt);
	memcpy(buf, data, len);
	rc = rtlsdr_i2c_write_fn(priv->rtl_dev, i2c_addr, reg, buf, len);
	if (rc != len) {
		fprintf(stderr, "%s: i2c wr failed=%d reg=%02x len=%d\n",
			   __FUNCTION__, rc, reg, len);
		if (rc < 0)
			return rc;
		return -1;
	}

	return 0;
}

static int cxd2841er_write_reg(struct cxd2841er_priv *priv,
			       uint8_t addr, uint8_t reg, uint8_t val)
{
	return cxd2841er_write_regs(priv, addr, reg, &val, 1);
}

static int cxd2841er_read_regs(struct cxd2841er_priv *priv,
			       uint8_t addr, uint8_t reg, uint8_t *buf, int len)
{
	uint8_t i2c_addr = (addr == I2C_SLVX ?
		priv->i2c_addr_slvx : priv->i2c_addr_slvt);
	int rc = rtlsdr_i2c_read_fn(priv->rtl_dev, i2c_addr, reg, buf, len);
	if (rc != len) {
		fprintf(stderr, "%s: i2c rd failed=%d reg=%02x len=%d\n",
			   __FUNCTION__, rc, reg, len);
		if (rc < 0)
			return rc;
		return -1;
	}

	return 0;
}

static int cxd2841er_read_reg(struct cxd2841er_priv *priv,
			      u8 addr, u8 reg, u8 *val)
{
	return cxd2841er_read_regs(priv, addr, reg, val, 1);
}

static int cxd2841er_set_reg_bits(struct cxd2841er_priv *priv,
				  u8 addr, u8 reg, u8 data, u8 mask)
{
	int res;
	u8 rdata;

	if (mask != 0xff) {
		res = cxd2841er_read_reg(priv, addr, reg, &rdata);
		if (res)
			return res;
		data = ((data & mask) | (rdata & (mask ^ 0xFF)));
	}
	return cxd2841er_write_reg(priv, addr, reg, data);
}

static int cxd2841er_shutdown_to_sleep_tc(struct cxd2841er_priv *priv)
{
	u8 data = 0;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_SHUTDOWN) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid demod state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-X Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* Clear all demodulator registers */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x02, 0x00);
	usleep(3000);
	/* Set SLV-X Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* Set demod SW reset */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x10, 0x01);
  /* Select ADC clock mode */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x13, 0x00);

	cxd2841er_write_reg(priv, I2C_SLVX, 0x14, data);
	/* Clear demod SW reset */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x10, 0x00);
	usleep(1000);
	/* Set SLV-T Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* TADC Bias On */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x43, 0x0a);
	cxd2841er_write_reg(priv, I2C_SLVT, 0x41, 0x0a);
	/* SADC Bias On */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x63, 0x16);
	cxd2841er_write_reg(priv, I2C_SLVT, 0x65, 0x27);
	cxd2841er_write_reg(priv, I2C_SLVT, 0x69, 0x06);
	priv->state = STATE_SLEEP_TC;
	return 0;
}

static int cxd2841er_sleep_tc_to_active_t(struct cxd2841er_priv *priv)
{
	u8 data[2] = { 0x09, 0x54 };

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	//cxd2841er_set_ts_clock_mode(priv, SYS_DVBT);
	/* Set SLV-X Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* Set demod mode */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x17, 0x01);
	/* Set SLV-T Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Enable demod clock */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x2c, 0x01);
	/* Disable RF level monitor */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x2f, 0x00);
	/* Enable ADC clock */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x30, 0x00);
	/* Enable ADC 1 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x41, 0x1a);
	/* Enable ADC 2 & 3 */
	cxd2841er_write_regs(priv, I2C_SLVT, 0x43, data, 2);
	/* Enable ADC 4 */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x18, 0x00);
	/* Set SLV-T Bank : 0x10 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	/* IFAGC gain settings */
	cxd2841er_set_reg_bits(priv, I2C_SLVT, 0xd2, 0x0f, 0x1f); //0x0c
	/* Set SLV-T Bank : 0x11 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x11);
	/* BBAGC TARGET level setting */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x6a, 0x50);
	/* Set SLV-T Bank : 0x10 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	/* ASCOT setting */
	cxd2841er_set_reg_bits(priv, I2C_SLVT, 0xa5,
		((priv->flags & CXD2841ER_ASCOT) ? 0x01 : 0x00), 0x01);
	/* Set SLV-T Bank : 0x18 */
	//cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x18);
	/* Pre-RS BER monitor setting */
	//cxd2841er_set_reg_bits(priv, I2C_SLVT, 0x36, 0x40, 0x07);
	/* FEC Auto Recovery setting */
	//cxd2841er_set_reg_bits(priv, I2C_SLVT, 0x30, 0x01, 0x01);
	//cxd2841er_set_reg_bits(priv, I2C_SLVT, 0x31, 0x01, 0x01);
	/* Set SLV-T Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* TSIF setting */
	cxd2841er_set_reg_bits(priv, I2C_SLVT, 0xce, 0x01, 0x01);
	cxd2841er_set_reg_bits(priv, I2C_SLVT, 0xcf, 0x01, 0x01);

	//cxd2841er_sleep_tc_to_active_t_band(priv, bandwidth);
	/* Set SLV-T Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Disable HiZ Setting 1 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x80, 0x28);
	/* Disable HiZ Setting 2 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x81, 0x00);
	priv->state = STATE_ACTIVE_TC;
	return 0;
}

static int cxd2841er_sleep_tc_to_active_c(struct cxd2841er_priv *priv)
{
	u8 data[2] = { 0x09, 0x54 };

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	//cxd2841er_set_ts_clock_mode(priv, SYS_DVBC_ANNEX_A);
	/* Set SLV-X Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* Set demod mode */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x17, 0x04);
	/* Set SLV-T Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Enable demod clock */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x2c, 0x01);
	/* Disable RF level monitor */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x59, 0x00);
	cxd2841er_write_reg(priv, I2C_SLVT, 0x2f, 0x00);
	/* Enable ADC clock */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x30, 0x00);
	/* Enable ADC 1 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x41, 0x1a);
	/* xtal freq 20.5MHz */
	cxd2841er_write_regs(priv, I2C_SLVT, 0x43, data, 2);
	/* Enable ADC 4 */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x18, 0x00);
	/* Set SLV-T Bank : 0x10 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	/* IFAGC gain settings */
	cxd2841er_set_reg_bits(priv, I2C_SLVT, 0xd2, 0x0f, 0x1f);//0x09
	/* Set SLV-T Bank : 0x11 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x11);
	/* BBAGC TARGET level setting */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x6a, 0x50);//0x48
	/* Set SLV-T Bank : 0x10 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	/* ASCOT setting */
	cxd2841er_set_reg_bits(priv, I2C_SLVT, 0xa5,
		((priv->flags & CXD2841ER_ASCOT) ? 0x01 : 0x00), 0x01);
	/* Set SLV-T Bank : 0x40 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x40);
	/* Demod setting */
	cxd2841er_set_reg_bits(priv, I2C_SLVT, 0xc3, 0x00, 0x04);
	/* Set SLV-T Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* TSIF setting */
	cxd2841er_set_reg_bits(priv, I2C_SLVT, 0xce, 0x01, 0x01);
	cxd2841er_set_reg_bits(priv, I2C_SLVT, 0xcf, 0x01, 0x01);

	//cxd2841er_sleep_tc_to_active_c_band(priv, bandwidth);
	/* Set SLV-T Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Disable HiZ Setting 1 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x80, 0x28);
	/* Disable HiZ Setting 2 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x81, 0x00);
	priv->state = STATE_ACTIVE_TC;
	return 0;
}

static int cxd2841er_tune_done(struct cxd2841er_priv *priv)
{
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	/* Set SLV-T Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0, 0);
	/* SW Reset */
	cxd2841er_write_reg(priv, I2C_SLVT, 0xfe, 0x01);
	/* Enable TS output */
	//cxd2841er_write_reg(priv, I2C_SLVT, 0xc3, 0x00);
	return 0;
}

static int cxd2841er_active_t_to_sleep_tc(struct cxd2841er_priv *priv)
{
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* disable TS output */
	cxd2841er_write_reg(priv, I2C_SLVT, 0xc3, 0x01);
	/* enable Hi-Z setting 1 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x80, 0x3f);
	/* enable Hi-Z setting 2 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x81, 0xff);
	/* Set SLV-X Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* disable ADC 1 */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x18, 0x01);
	/* Set SLV-T Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Disable ADC 2 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x43, 0x0a);
	/* Disable ADC 3 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x41, 0x0a);
	/* Disable ADC clock */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x30, 0x00);
	/* Disable RF level monitor */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x2f, 0x00);
	/* Disable demod clock */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x2c, 0x00);
	priv->state = STATE_SLEEP_TC;
	return 0;
}

static int cxd2841er_active_c_to_sleep_tc(struct cxd2841er_priv *priv)
{
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* disable TS output */
	cxd2841er_write_reg(priv, I2C_SLVT, 0xc3, 0x01);
	/* enable Hi-Z setting 1 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x80, 0x3f);
	/* enable Hi-Z setting 2 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x81, 0xff);
	/* Cancel DVB-C setting */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x11);
	cxd2841er_set_reg_bits(priv, I2C_SLVT, 0xa3, 0x00, 0x1f);
	/* Set SLV-X Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* disable ADC 1 */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x18, 0x01);
	/* Set SLV-T Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	/* Disable ADC 2 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x43, 0x0a);
	/* Disable ADC 3 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x41, 0x0a);
	/* Disable ADC clock */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x30, 0x00);
	/* Disable RF level monitor */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x2f, 0x00);
	/* Disable demod clock */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x2c, 0x00);
	priv->state = STATE_SLEEP_TC;
	return 0;
}

static int cxd2841er_sleep_tc(struct cxd2841er_priv *priv)
{
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state == STATE_ACTIVE_TC) {
		switch (priv->system) {
		case SYS_DVBT:
			cxd2841er_active_t_to_sleep_tc(priv);
			break;
		case SYS_DVBC_ANNEX_A:
			cxd2841er_active_c_to_sleep_tc(priv);
			break;
		default:
			dev_warn(&priv->i2c->dev,
				"%s(): unknown delivery system %d\n",
				__func__, priv->system)
			;
		}
	}
	if (priv->state != STATE_SLEEP_TC) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	return 0;
}

static int cxd2841er_sleep_tc_to_shutdown(struct cxd2841er_priv *priv)
{
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_SLEEP_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid demod state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-X Bank : 0x00 */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x00, 0x00);
	/* Disable oscillator */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x15, 0x01);
	/* Set demod mode */
	cxd2841er_write_reg(priv, I2C_SLVX, 0x17, 0x01);
	priv->state = STATE_SHUTDOWN;
	return 0;
}

static int cxd2841er_shutdown_tc(struct cxd2841er_priv *priv)
{
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);

	if (!cxd2841er_sleep_tc(priv))
		cxd2841er_sleep_tc_to_shutdown(priv);
	return 0;
}

int cxd2837_read_signal_strength(struct cxd2841er_priv *priv)
{
	u8 data[2];
	u16 agc;

	if (priv->system == SYS_DVBT)
	{
		cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
		cxd2841er_read_regs(priv, I2C_SLVT, 0x26, data, 2);
	}
	else
	{
		cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x40);
		cxd2841er_read_regs(priv, I2C_SLVT, 0x49, data, 2);
	}
	agc = (((u16)data[0] & 0x0F) << 8) | (u16)(data[1] & 0xFF);
	dev_dbg(&priv->i2c->dev, "%s(): AGC value=%u\n", __func__, agc);
	return agc;
}

int cxd2837_init(struct cxd2841er_priv *priv)
{
	priv->state = STATE_SHUTDOWN;
	priv->system = SYS_DVBC_ANNEX_A;

	cxd2841er_shutdown_to_sleep_tc(priv);
	/* SONY_DEMOD_CONFIG_IFAGCNEG = 1 (0 for NO_AGCNEG */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x10);
	cxd2841er_set_reg_bits(priv, I2C_SLVT, 0xcb,
		((priv->flags & CXD2841ER_NO_AGCNEG) ? 0x00 : 0x40), 0x40);
	/* SONY_DEMOD_CONFIG_IFAGC_ADC_FS = 0 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0xcd, 0x50);
	/* SONY_DEMOD_CONFIG_PARALLEL_SEL = 1 */
	cxd2841er_write_reg(priv, I2C_SLVT, 0x00, 0x00);
	cxd2841er_set_reg_bits(priv, I2C_SLVT, 0xc4,
		((priv->flags & CXD2841ER_TS_SERIAL) ? 0x80 : 0x00), 0x80);

	/* clear TSCFG bits 3+4 */
	if (priv->flags & CXD2841ER_TSBITS)
		cxd2841er_set_reg_bits(priv, I2C_SLVT, 0xc4, 0x00, 0x18);

	if (priv->system == SYS_DVBT)
		cxd2841er_sleep_tc_to_active_t(priv);
	else
		cxd2841er_sleep_tc_to_active_c(priv);
	cxd2841er_tune_done(priv);
	return 0;
}

int cxd2837_exit(struct cxd2841er_priv *priv)
{
	return cxd2841er_shutdown_tc(priv);
}
