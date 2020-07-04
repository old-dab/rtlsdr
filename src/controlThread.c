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
#include "getopt/getopt.h"
#endif

#ifdef NEED_PTHREADS_WORKARROUND
#define HAVE_STRUCT_TIMESPEC
#endif
#include <pthread.h>

#include "rtl-sdr.h"
#include "rtl_tcp.h"
#include "controlThread.h"
#include "convenience/convenience.h"

#include "tuner_r82xx.h"

extern int	rtlsdr_demod_read_regs(rtlsdr_dev_t *dev, uint16_t page, uint16_t addr, unsigned char *data, uint8_t len);
extern void print_demod_register(rtlsdr_dev_t *dev, uint16_t page);

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")

typedef int socklen_t;

#else
#define closesocket close
#define SOCKADDR struct sockaddr
#define SOCKET int
#define SOCKET_ERROR -1
#endif

#define MAX_I2C_REGISTERS  256
#define TX_BUF_LEN (5+MAX_I2C_REGISTERS) //1 command, 2 tuner_gain, 2 len


ctrl_thread_data_t ctrl_thread_data;

void *ctrl_thread_fn(void *arg)
{
	unsigned char reg_values [MAX_I2C_REGISTERS];
	unsigned char txbuf [TX_BUF_LEN];
	int r = 1;
	struct timeval tv = { 1,0 };
	struct linger ling = { 1,0 };
	SOCKET listensocket;
	SOCKET controlSocket;
	int haveControlSocket = 0;
	struct sockaddr_in local, remote;
	socklen_t rlen;
	int i, agc_val;
	unsigned char x;
	LARGE_INTEGER StartingTime, EndingTime;
	LARGE_INTEGER Frequency;
	int64_t Microseconds = 0;

	int error = 0;
	int len, result, tuner_gain;
	fd_set connfds;
	fd_set writefds;
	int bytesleft, bytessent, index;
	int old_gain = 0;

	ctrl_thread_data_t *data = (ctrl_thread_data_t *)arg;

	rtlsdr_dev_t *dev = data->dev;
	int port = data->port;
	int wait = data->wait;
	int report_i2c = data->report_i2c;
	char *addr = data->addr;
	int* do_exit = data->pDoExit;
	u_long blockmode = 1;
	int retval;

	memset(reg_values, 0, MAX_I2C_REGISTERS);

	memset(&local, 0, sizeof(local));
	local.sin_family = AF_INET;
	local.sin_port = htons(port);
	local.sin_addr.s_addr = inet_addr(addr);

	listensocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	setsockopt(listensocket, SOL_SOCKET, SO_REUSEADDR, (char *)&r, sizeof(int));
	setsockopt(listensocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
	retval = bind(listensocket, (struct sockaddr *)&local, sizeof(local));
	if (retval == SOCKET_ERROR)
		error = 1;
#ifdef _WIN32
	ioctlsocket(listensocket, FIONBIO, &blockmode);
#else
	r = fcntl(listensocket, F_GETFL, 0);
	r = fcntl(listensocket, F_SETFL, r | O_NONBLOCK);
#endif

	while (1) {
		//printf("listening on Control port %d...\n", port);
		retval = listen(listensocket, 1);
		if (retval == SOCKET_ERROR)
			error = 1;
		while (1) {
			FD_ZERO(&connfds);
			FD_SET(listensocket, &connfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(listensocket + 1, &connfds, NULL, NULL, &tv);
			if (*do_exit) {
				goto close;
			}
			else if (r) {
				rlen = sizeof(remote);
				controlSocket = accept(listensocket, (struct sockaddr *)&remote, &rlen);
				haveControlSocket = 1;
				break;
			}
			//QueryPerformanceCounter(&StartingTime);
			//QueryPerformanceFrequency(&Frequency);
			result = rtlsdr_get_tuner_i2c_register(dev, reg_values, &len, &tuner_gain);
			tuner_gain = (tuner_gain + 5) / 10;
			if(old_gain != tuner_gain)
			{
				printf("gain = %2d dB\r", tuner_gain);
				//print_demod_register(dev, 3);
				old_gain = tuner_gain;
			}
			/*QueryPerformanceCounter(&EndingTime);
			Microseconds = EndingTime.QuadPart - StartingTime.QuadPart;
			Microseconds *= 1000000;
			Microseconds /= Frequency.QuadPart;
			printf("Microseconds %d\n",(int)Microseconds);*/
		}

		setsockopt(controlSocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));

		printf("Control client accepted!\n");
		usleep(5000000);

		while (1) {

			/* check if i2c reporting is to be (de)activated */
			if ( report_i2c && !data->report_i2c )
				report_i2c = 0;
			else if ( !report_i2c && data->report_i2c )
				report_i2c = 1;

			/* @TODO: check if something else has to be transmitted */
			if ( !report_i2c )
				goto sleep;

			len = 0;
			result = rtlsdr_get_tuner_i2c_register(dev, reg_values, &len, &tuner_gain);
			if(old_gain != tuner_gain)
			{
				//printf("len = %d, tuner_gain = %d\n", len, tuner_gain);
				//print_demod_register(dev, 3);
				old_gain = tuner_gain;
			}
			memset(txbuf, 0, TX_BUF_LEN);
			if (result)
				goto sleep;

			//Big Endian / Network Byte Order
			txbuf[0] = REPORT_I2C_REGS;
			txbuf[1] = ((len+2) >> 8) & 0xff;
			txbuf[2] = (len+2) & 0xff;
			//txbuf[1] = ((len+5) >> 8) & 0xff;
			//txbuf[2] = (len+5) & 0xff;
			txbuf[3] = (tuner_gain >> 8) & 0xff;
			txbuf[4] = tuner_gain & 0xff;
			/* now the message contents */
			memcpy(&txbuf[5], reg_values, len);
			len += 5;
			/*agc_val = 0;
			for(i=0; i<8; i++)
			{
				rtlsdr_demod_read_regs(dev, 3, 0x05, &x, 1);
				agc_val += x;
				printf("%02x ", x);
			}
			printf("\n");
			txbuf[len] = agc_val / 8;
			len += 1;
			rtlsdr_demod_read_regs(dev, 3, 0x59, txbuf+len, 2);
			len += 2;*/
			//print_demod_register(dev, 3);

			/* now start (possibly blocking) transmission */
			bytessent = 0;
			bytesleft = len;
			index = 0;

			while (bytesleft > 0) {
				FD_ZERO(&writefds);
				FD_SET(controlSocket, &writefds);
				tv.tv_sec = 1;
				tv.tv_usec = 0;
				r = select(controlSocket + 1, NULL, &writefds, NULL, &tv);
				if (r) {
					bytessent = send(controlSocket, (const char*)&txbuf[index], bytesleft, 0);
					bytesleft -= bytessent;
					index += bytessent;
				}
				if (bytessent == SOCKET_ERROR || *do_exit) {
					goto close;
				}
			}
sleep:
			usleep(wait);
		}
close:
		if (haveControlSocket)
			closesocket(controlSocket);
		if (*do_exit)
		{
			closesocket(listensocket);
			printf("Control Thread terminates\n");
			break;
		}
	}
	return 0;
}
