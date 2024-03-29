/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012-2013 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Dimitri Stolnikov <horiz0n@gmx.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __RTL_TCP_H
#define __RTL_TCP_H

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * This enum defines the possible commands in rtl_tcp
 * commands 0x01..0x0E are compatible to osmocom's rtlsdr
 * see https://github.com/osmocom/rtl-sdr/blob/master/src/rtl_tcp.c
 * commands >= 0x40 are extensions
 */
enum RTL_TCP_COMMANDS {
    SET_FREQUENCY             = 0x01,
    SET_SAMPLE_RATE           = 0x02,
    SET_GAIN_MODE             = 0x03,
    SET_GAIN                  = 0x04,
    SET_FREQUENCY_CORRECTION  = 0x05,
    SET_IF_STAGE              = 0x06,
    SET_TEST_MODE             = 0x07,
    SET_AGC_MODE              = 0x08,
    SET_DIRECT_SAMPLING       = 0x09,
    SET_OFFSET_TUNING         = 0x0A,
    SET_RTL_CRYSTAL           = 0x0B,
    SET_TUNER_CRYSTAL         = 0x0C,
    SET_TUNER_GAIN_BY_INDEX   = 0x0D,
#if 1
    /* development branch since 2018-10-03 */
    SET_BIAS_TEE              = 0x0E,
    SET_TUNER_BANDWIDTH       = 0x40,
#else
    /* prev code - used in ExtIO - to build compatible rtl_tcp.exe */
    SET_TUNER_BANDWIDTH       = 0x0E,
    SET_BIAS_TEE              = 0x0F
#endif
    UDP_ESTABLISH             = 0x41,
    UDP_TERMINATE             = 0x42,
    SET_I2C_TUNER_REGISTER    = 0x43,   /* for experiments: 32 bit data word:
                                         * 31 .. 20: register (12 bits)
                                         * 19 .. 12: mask (8 bits)
                                         * 11 ..  0: data (12 bits) */
    SET_I2C_TUNER_OVERRIDE    = 0x44,   /* encoding as with SET_I2C_TUNER_REGISTER
                                         * data (bits 11 .. 0) > 255 removes override */
    SET_TUNER_BW_IF_CENTER    = 0x45,   /* freq from SET_FREQUENCY stays in center;
                                         * the bandwidth (from SET_TUNER_BANDWIDTH)
                                         * is set to be centered at given IF frequency */
    SET_SIDEBAND              = 0x46,   /* Mixer Sideband for R820T */
    REPORT_I2C_REGS           = 0x48,   /* perodically report I2C registers
                                         * - if reverse channel is enabled */
    SET_DITHERING			  = 0x49,   /* Enable or disable frequency dithering for R820T */
	SET_FREQUENCY_CORRECTION_001_PPM = 0x4A,	/* Set frequency correction in multiples of 0.01 ppm */
    CMD_SET_LNA_STATE     	  = 0x4B,   // 0: most sensitive, 8: least sensitive
    CMD_SET_REQUEST_ALL_SERIALS = 0x80, // request for all serials to be transmitted via back channel
    CMD_SET_SELECT_SERIAL 	  = 0x81,    // value is four bytes CRC-32 of the requested serial number
	SET_FREQUENCY_CORRECTION_PPB = 0x83	/* Set frequency correction in parts per billion */
};

enum eCommState
{
	ST_IDLE = 0
	, ST_SERIALS_REQUESTED
	, ST_DEVICE_CREATED
	, ST_WELCOME_SENT
	, ST_DEVICE_RELEASED
};

#define MAX_DEVICES 6

#ifdef __cplusplus
}
#endif

#endif
