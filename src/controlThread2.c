/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012-2013 by Hoernchen <la@tfc-server.de>
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

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <fcntl.h>
#else
#include <winsock2.h>
#endif

#ifdef NEED_PTHREADS_WORKARROUND
#define HAVE_STRUCT_TIMESPEC
#endif

#include <pthread.h>

#include "rtl-sdr.h"
#include "rtl_tcp.h"
#include "controlThread.h"

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")

typedef int socklen_t;

#else
#define closesocket close
#define SOCKADDR struct sockaddr
#define SOCKET int
#define SOCKET_ERROR -1
#endif

enum eIndications
{
	  IND_GAIN              = 0
	, IND_GAIN_COUNT        = 1
	, IND_LNA_STATE         = 0x4b			  // 0: most sensitive, 8: least sensitive
	, IND_SERIAL			= 0x80            // A list with all available serial numbers, as a response on CMD_SET_RSP_REQUEST_ALL_SERIALS
	, IND_WELCOME			= 0x81            // Termination of the initial parameters
	, IND_MAGIC_STRING		= 0x82            // "RTL0"
	, IND_RX_STRING			= 0x83            // "RSPx"
	, IND_RX_TYPE			= 0x84            // 1 byte
	, IND_BIT_WIDTH			= 0x85            // 1 byte
	, IND_OVERLOAD_A		= 0x86            // 1 byte, 1 == overload
	, IND_OVERLOAD_B		= 0x87            // 1 byte
	, IND_DEVICE_RELEASED	= 0x88            // 1 byte bool
	, IND_RSPDUO_HiZ    	= 0x89            // 1 byte bool
	, IND_BIAST_STATE       = 0x8A			  // 0: off, 1: on
	, IND_RF_CHANGED        = 0x8B			  // 4 Byte current frequency
	, IND_AM_NOTCH          = 0x8C			  // 1 byte ->0: off, 1: on
	, IND_DAB_NOTCH         = 0x8D			  // 1 byte ->0: off, 1: on
	, IND_RF_NOTCH          = 0x8E			  // 1 byte ->0: off, 1: on
	, IND_ANTENNA_SELECTED  = 0x8F			  // 1 byte -> 5,6: RSPII or RSPduo TunerSelect
											  //           0,1,2: RSPdx A, B, C
											  // 7 && 0-60MHz : HiZ
};

#define MAX_I2C_REGISTERS  256
#define TX_BUF_LEN (1024)

extern rtlsdr_dev_t *dev;
extern int CommState;
extern int hardware_model;
extern int bitwidth;
extern int overload;
extern int lna_state;
extern int numDevices;
extern uint32_t serialCRCs[];
extern int sendBuffer(SOCKET socket, char *txbuf, int len, volatile int *do_exit);
extern pthread_mutex_t mut;
extern pthread_cond_t cond;

static unsigned char txbuf[TX_BUF_LEN];
static uint32_t crcTable[256];

ctrl_thread_data_t ctrl_thread_data;

/// <summary>
/// Preapres the buffer "ready to send" for an simple indication
/// </summary>
/// <param name="indic">Indication</param>
/// <param name="value">one item</param>
/// <param name="length">Length of value in bytes</param>
/// <remark>
/// One byte indication
/// Two bytes length, the length of the following value(s)
int prepareIntCommand(uint8_t* tx, int startIx, int indic, int value, int length)
{
	//Big Endian / Network Byte Order
	int ix = startIx;

	tx[ix++] = (uint8_t)(indic & 0xff);
	txbuf[ix++] = (length >> 8) & 0xff;
	txbuf[ix++] = length & 0xff;
	switch (length)
	{
	case 4:
		tx[ix++] = (value >> 24) & 0xff;
		tx[ix++] = (value >> 16) & 0xff;
		//fall through
	case 2:
		tx[ix++] = (value >> 8) & 0xff;
		//fall through
	case 1:
		tx[ix++] = (value >> 0) & 0xff;
		break;
	default:
		break;
	}
	return ix;
}

int prepareStringCommand(uint8_t* tx, int startIx, int indic, uint8_t* value, int length )
{
	//Big Endian / Network Byte Order
	int ix = startIx;

	tx[ix++] = (uint8_t)(indic & 0xff);
	txbuf[ix++] = (length >> 8) & 0xff;
	txbuf[ix++] = length & 0xff;

	for (int i = 0; i < length; i++)
		txbuf[ix++] = value[i];
	return ix;
}

void createCrcTable(uint32_t polynomial)
{
    for (int i = 0; i < 256; i++)
    {
        uint32_t crc = (uint32_t)i;
        for (int k = 8; k > 0; k--)
        {
            if ((crc & 1) != 0)
                crc = (crc >> 1) ^ polynomial;
            else
                crc >>= 1;
        }
        crcTable[i] = crc;
    }
}

uint32_t calcCrcVal(uint8_t inp[], int length, uint32_t initialValue, int finalInvert)
{
    uint32_t crc = initialValue;
    for (int i = 0; i < length; i++)
    {
        crc = (crc >> 8) ^ crcTable[(crc & 0xff) ^ inp[i]];
    }
    if (finalInvert)
        crc ^= 0xffffffff;
    return crc;
}

int prepareSerialsList(uint8_t* buf)
{
	#define SERLEN 64
	char vendor[256], product[256], serial[256];
	uint8_t* p = buf;
	char SerNo[SERLEN];

	createCrcTable(0xedb88320);
	fprintf(stderr, "Preparing device serials list.\n");
	numDevices = rtlsdr_get_device_count();
	for (int i = 0; i < numDevices; i++)
	{
		if (rtlsdr_get_device_usb_strings(i, vendor, product, serial) < 0)
			continue;
		memset(SerNo,0,SERLEN);
		sprintf(SerNo, "%d: ", i);
		strcat(SerNo, vendor);
		strcat(SerNo, " ");
		strcat(SerNo, product);
		if(strlen(serial) > 0)
		{
			strcat(SerNo, " SN: ");
			strcat(SerNo, serial);
		}
		for (int k = 0; k < SERLEN; k++)
			*p++ = SerNo[k];
		*p++ = ',';
		*p++ = 255; //hwVer;
		*p++ = ';';
		serialCRCs[i] = calcCrcVal((uint8_t*)SerNo, 64, 0xffffffff, 1);
	}
	return (int)(p - buf);
}

int old_gain = 0;
int tuner_gain = 0;

void *ctrl_thread_fn(void *arg)
{
	unsigned char reg_values[MAX_I2C_REGISTERS];
	int r = 1;
	struct timeval tv = { 1,0 };
	struct linger ling = { 1,0 };
	SOCKET listensocket;
	SOCKET controlSocket;
	int haveControlSocket = 0;
	struct sockaddr_in local, remote;
	fd_set connfds;
	ctrl_thread_data_t *data = (ctrl_thread_data_t *)arg;
	int wait = data->wait;
	volatile int* do_exit = data->pDoExit;
	u_long blockmode = 1;
	int retval;

	memset(reg_values, 0, MAX_I2C_REGISTERS);

	memset(&local, 0, sizeof(local));
	local.sin_family = AF_INET;
	local.sin_port = htons(data->port);
	local.sin_addr.s_addr = inet_addr(data->addr);

	listensocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	setsockopt(listensocket, SOL_SOCKET, SO_REUSEADDR, (char *)&r, sizeof(int));
	setsockopt(listensocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
	retval = bind(listensocket, (struct sockaddr *)&local, sizeof(local));
	if (retval == SOCKET_ERROR)
		goto close;
#ifdef _WIN32
	ioctlsocket(listensocket, FIONBIO, &blockmode);
#else
	r = fcntl(listensocket, F_GETFL, 0);
	r = fcntl(listensocket, F_SETFL, r | O_NONBLOCK);
#endif

	fprintf(stderr, "listening on port %d...\n", data->port);
	retval = listen(listensocket, 1);
	if (retval == SOCKET_ERROR)
		goto close;
	while (1)
	{
		FD_ZERO(&connfds);
		FD_SET(listensocket, &connfds);
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		r = select(listensocket + 1, &connfds, NULL, NULL, &tv);
		if (*do_exit)
			goto close;
		else if (r)
		{
			socklen_t rlen = sizeof(remote);
			controlSocket = accept(listensocket, (struct sockaddr *)&remote, &rlen);
			haveControlSocket = 1;
			break;
		}
	}

	setsockopt(controlSocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
	fprintf(stderr, "control client accepted!\n");

	while (1)
	{
		unsigned char tmp[1024];
		int len;
		int buflen = 0;
		int reglen = 0;
		int tuner_gain_count = 0, tuner_type = 0;

		//Big Endian / Network Byte Order
        len = 2; // For the first two bytes

		switch (CommState)
		{
		case ST_IDLE:
			goto sleep;

		case ST_DEVICE_RELEASED:
			len = prepareIntCommand(txbuf, len, IND_DEVICE_RELEASED, 1, 1);
			CommState = ST_IDLE; // wait for socket to close
			break;

		case ST_SERIALS_REQUESTED: // 1st command
			buflen = prepareSerialsList(tmp);
			len = prepareStringCommand(txbuf, len, IND_SERIAL, tmp, buflen);
			CommState = ST_IDLE; // wait for the next command changing the state
			break;

		case ST_DEVICE_CREATED:
			len = prepareStringCommand(txbuf, len, IND_MAGIC_STRING, (uint8_t*)"RTL0", 4);
			len = prepareStringCommand(txbuf, len, IND_RX_STRING,    (uint8_t*)"RTL0", 4);
			len = prepareIntCommand(txbuf, len, IND_RX_TYPE, 5, 1);
			len = prepareIntCommand(txbuf, len, IND_BIT_WIDTH, 1, 1);
			len = prepareIntCommand(txbuf, len, IND_WELCOME, 1, 1);
			tuner_type = rtlsdr_get_tuner_type(dev);
			if (tuner_type >= 0)
				len = prepareIntCommand(txbuf, len, IND_RX_TYPE, tuner_type, 1);
			tuner_gain_count = rtlsdr_get_tuner_gains(dev, NULL);
			if (tuner_gain_count >= 0)
				len = prepareIntCommand(txbuf, len, IND_GAIN_COUNT, tuner_gain_count, 1);
			fprintf(stderr, "tuner_gain_count = %d\n", tuner_gain_count);
			pthread_mutex_lock(&mut);
			CommState = ST_WELCOME_SENT;
			pthread_cond_signal(&cond);
			pthread_mutex_unlock(&mut);
			//fall through
		case ST_WELCOME_SENT:
			if(rtlsdr_get_tuner_i2c_register(dev, reg_values, &reglen, &tuner_gain) == 0)
			{
#ifdef DEBUG
				if(old_gain != tuner_gain)
				{
					old_gain = tuner_gain;
				}
#endif
				len = prepareIntCommand(txbuf, len, IND_GAIN, tuner_gain-30, 2); // -3 dB Korrektur für Qirx
				len = prepareStringCommand(txbuf, len, REPORT_I2C_REGS, reg_values, reglen);
			}
			//len = prepareIntCommand(txbuf, len, IND_LNA_STATE, lna_state, 1);
			len = prepareIntCommand(txbuf, len, IND_OVERLOAD_A, overload, 1);
			break;

		default:
			goto sleep;
			break;
		}

		// total length of the sent buffer
		txbuf[0] = (len >> 8) & 0xff;
		txbuf[1] = len & 0xff;

		if (!sendBuffer(controlSocket, (char *)txbuf, len, do_exit))
			break;
sleep:
		if (*do_exit)
			break;
		usleep(wait);
	}
close:
	if (haveControlSocket)
		closesocket(controlSocket);
	closesocket(listensocket);
	fprintf(stderr, "Control thread terminated\n");
	return 0;
}
