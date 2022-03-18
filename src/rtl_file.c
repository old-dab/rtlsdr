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
#include <time.h>
#include <math.h>

#ifdef DEBUG
#include <stdarg.h>
#endif

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
#include "convenience/convenience.h"
#include "version.h"

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")
typedef int socklen_t;
#else
#define closesocket close
#define SOCKADDR struct sockaddr
#define SOCKET int
#define SOCKET_ERROR -1
#define CTRL_C_EVENT        0
#define CTRL_BREAK_EVENT    1
#define CTRL_CLOSE_EVENT    2
#endif

#define DEFAULT_BUF_LENGTH	(64 * 512)

static SOCKET s;

static pthread_mutex_t ll_mutex;
static pthread_cond_t cond;

struct llist {
	char *data;
	size_t len;
	struct llist *next;
};

typedef struct { /* structure size must be multiple of 2 bytes */
	char magic[4];
	uint32_t tuner_type;
	uint32_t tuner_gain_count;
} dongle_info_t;


static int verbosity = 0;

static int global_numq = 0;
static struct llist *ll_buffers = 0;
static int llbuf_num = 500;

static volatile int do_exit = 0;

static void usage(void)
{
	printf("\n"
		"Usage:\t[-a listen address]\n"
		"\t[-f <filename>\n"
		"\t[-e send endless (default: 0)]\n"
		"\t[-l length of single buffer in units of 512 samples (default: 256)]\n"
		"\t[-n max number of linked list buffers to keep (default: 500)]\n"
		"\t[-p listen port (default: 1234)]\n"
		"\t[-s samplerate in Hz (default: 2048000 Hz)]\n"
		"\t[-v increase verbosity (default: 0)]\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI sighandler(int signum)
{
	if (CTRL_C_EVENT == signum)
	{
		printf("CTRL-C caught, exiting!\n");
		do_exit = 1;
		return TRUE;
	}
	else if (CTRL_CLOSE_EVENT == signum)
	{
		printf("SIGQUIT caught, exiting!\n");
		do_exit = 1;
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	printf("Signal (%d) caught, ask for exit!\n", signum);
	do_exit = 1;
}
#endif

static void file_callback(unsigned char *buf, uint32_t len)
{
	//printf("Read %u bytes\n", len);
	if(!do_exit)
	{
		struct llist *rpt = (struct llist*)malloc(sizeof(struct llist));
		rpt->data = (char*)malloc(len);
		memcpy(rpt->data, buf, len);
		rpt->len = len;
		rpt->next = NULL;
		pthread_mutex_lock(&ll_mutex);
		if (ll_buffers == NULL)
		{
			ll_buffers = rpt;
		}
		else
		{
			struct llist *cur = ll_buffers;
			int num_queued = 0;

			while (cur->next != NULL)
			{
				cur = cur->next;
				num_queued++;
			}
			if(llbuf_num && llbuf_num == num_queued-2)
			{
				struct llist *curelem;

				free(ll_buffers->data);
				curelem = ll_buffers->next;
				free(ll_buffers);
				ll_buffers = curelem;
			}

			cur->next = rpt;

			if ( verbosity )
			{
				if (num_queued > global_numq)
					printf("ll+, now %d\n", num_queued);
				else if (num_queued < global_numq)
					printf("ll-, now %d\n", num_queued);
			}

			global_numq = num_queued;
		}
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&ll_mutex);
	}
}

static inline int64_t getTime(void)
{
	struct timeval  tv;

    gettimeofday (&tv, NULL);
    return ((int64_t)tv. tv_sec * 1000000 + (int64_t)tv. tv_usec);
}

static int read_file(FILE *file, uint32_t buf_len, uint32_t sample_rate, int endless)
{
	unsigned char *buf = NULL;
	uint32_t len;
	int32_t period;
	int64_t fileLength, filepos = 0;
	int progress, progress_old = 0;
	int64_t	nextStop;

	if (!buf_len || buf_len % 512 != 0) /* len must be multiple of 512 */
		buf_len = DEFAULT_BUF_LENGTH;
	buf = calloc(buf_len * sizeof(unsigned char *), 1);
	fseek(file, 0, SEEK_END);
	fileLength = ftell(file);
	printf("File length = %d\n", (int)fileLength);
    fseek(file, 0, SEEK_SET);
	period  = ((int64_t)buf_len * 500000) / sample_rate;
	//printf("period=%u\n",period);
	nextStop = getTime();

	while(!do_exit)
	{
		nextStop += period;
        if (nextStop - getTime() > 0)
        	usleep ((int)(nextStop - getTime()));
		len = fread(buf, 1, buf_len, file);
		filepos += len;
		if(len < buf_len)
		{
			if(endless)
			{
			    fseek (file, 0, SEEK_SET);
			    filepos = 0;
			    progress = 0;
			    progress_old = 0;
			}
			else
			{
				printf("End of file!\n");
				do_exit = 1;
			}
		}
		else
			file_callback(buf, buf_len);
        progress = (float)filepos * 100 / fileLength;
        if(progress > progress_old)
        {
        	printf("Done: %d%%  \r", progress);
        	progress_old = progress;
		}
	}
	free(buf);
	return 0;
}

static void *tcp_worker(void *arg)
{
	struct llist *curelem,*prev;
	int bytesleft,bytessent, index;
	struct timeval tv= {1,0};
	struct timespec ts;
	struct timeval tp;
	fd_set writefds;
	int r = 0;
	(void) arg;

	while(1)
	{
		if(do_exit)
			pthread_exit(0);

		pthread_mutex_lock(&ll_mutex);
		gettimeofday(&tp, NULL);
		ts.tv_sec  = tp.tv_sec+1;
		ts.tv_nsec = tp.tv_usec * 1000;
		r = pthread_cond_timedwait(&cond, &ll_mutex, &ts);
		if(r == ETIMEDOUT)
		{
			pthread_mutex_unlock(&ll_mutex);
			printf("worker cond timeout\n");
			sighandler(CTRL_CLOSE_EVENT);
			pthread_exit(NULL);
		}

		curelem = ll_buffers;
		ll_buffers = 0;
		pthread_mutex_unlock(&ll_mutex);

		while(curelem != 0)
		{
			bytesleft = curelem->len;
			index = 0;
			bytessent = 0;
			while(bytesleft > 0)
			{
				FD_ZERO(&writefds);
				FD_SET(s, &writefds);
				tv.tv_sec = 1;
				tv.tv_usec = 0;
				r = select(s+1, NULL, &writefds, NULL, &tv);
				if(r)
				{
					bytessent = send(s,  &curelem->data[index], bytesleft, 0);
					bytesleft -= bytessent;
					index += bytessent;
				}
				if(bytessent == SOCKET_ERROR || do_exit)
				{
#ifdef _WIN32
					printf("worker socket bye (%d), do_exit:%d\n", WSAGetLastError(), do_exit);
#else
					printf("worker socket bye, do_exit:%d\n", do_exit);
#endif
					sighandler(CTRL_CLOSE_EVENT);
					pthread_exit(NULL);
				}
			}
			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}
	}
	return NULL;
}


#ifdef _WIN32
#define __attribute__(x)
#pragma pack(push, 1)
#endif
struct command{
	unsigned char cmd;
	unsigned int param;
}__attribute__((packed));
#ifdef _WIN32
#pragma pack(pop)
#endif

static void *command_worker(void *arg)
{
	int left, received = 0;
	fd_set readfds;
	struct command cmd={0, 0};
	struct timeval tv= {1, 0};
	int r = 0;
	uint32_t param;
	(void) arg;

	while(1)
	{
		left=sizeof(cmd);
		while(left >0)
		{
			FD_ZERO(&readfds);
			FD_SET(s, &readfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(s+1, &readfds, NULL, NULL, &tv);
			if(r)
			{
				received = recv(s, (char*)&cmd+(sizeof(cmd)-left), left, 0);
				left -= received;
			}
			if(received == SOCKET_ERROR || do_exit)
			{
#ifdef _WIN32
				printf("comm recv bye (%d), do_exit:%d\n", WSAGetLastError(), do_exit);
#else
				printf("comm recv bye, do_exit:%d\n", do_exit);
#endif
				sighandler(CTRL_CLOSE_EVENT);
				pthread_exit(NULL);
			}
		}

		param = ntohl(cmd.param);
		switch(cmd.cmd)
		{
		case SET_FREQUENCY://0x01
			printf("set freq %u\n", param);
			break;
		case SET_SAMPLE_RATE://0x02
			printf("set sample rate %u\n", param);
			break;
		case SET_GAIN_MODE://0x03
			printf("set gain mode %u\n", param);
			break;
		case SET_GAIN://0x04
			printf("set gain %u\n", param);
			break;
		case SET_FREQUENCY_CORRECTION://0x05
			printf("set freq correction to %d ppm\n", (int)param);
			break;
		case SET_IF_STAGE://0x06
			printf("set if stage %d gain %d\n", param >> 16, (short)(param & 0xffff));
			break;
		case SET_TEST_MODE://0x07
			printf("set test mode %u\n", param);
			break;
		case SET_AGC_MODE://0x08
			printf("set agc mode %u\n", param);
			break;
		case SET_DIRECT_SAMPLING://0x09
			printf("set direct sampling %u\n", param);
			break;
		case SET_OFFSET_TUNING://0x0a
			printf("set offset tuning %d\n", (int)param);
			break;
		case SET_RTL_CRYSTAL://0x0b
			printf("set rtl xtal %u\n", param);
			break;
		case SET_TUNER_CRYSTAL://0x0c
			printf("set tuner xtal %u\n", param);
			break;
		case SET_TUNER_GAIN_BY_INDEX://0x0d
			printf("set tuner gain by index %u\n", param);
			break;
		case SET_BIAS_TEE://0x0e
			printf("set bias tee %u\n", param);
			break;
		case SET_TUNER_BANDWIDTH://0x40
			printf("set tuner bandwidth to %u Hz\n", param);
			break;
		case SET_I2C_TUNER_REGISTER://0x43
			printf("set i2c register x%03X to x%03X with mask x%02X\n", (param >> 20) & 0xfff, param & 0xfff, (param >> 12) & 0xff );
			break;
		case SET_SIDEBAND://0x46
			printf("set to %s sideband\n", param ? "upper" : "lower");
			break;
		case REPORT_I2C_REGS://0x48
			break;
		case SET_DITHERING://0x49
			printf("%sable dithering\n", param ? "en" : "dis");
			break;
		case SET_001_PPM://0x4a
			printf("set freq correction to %0.2f ppm\n", (int)param/100.0);
			break;
		default:
			break;
		}
		cmd.cmd = 0xff;
	}
	return NULL;
}


int main(int argc, char **argv)
{
	int r, opt;
	char* addr = "127.0.0.1";
	int port = 1234;
	uint32_t samp_rate = 2048000;
	struct sockaddr_in local, remote;
	uint32_t buf_len = 256 * 512;
	struct llist *curelem,*prev;
	pthread_attr_t attr;
	void *status;
	struct timeval tv = {1,0};
	struct linger ling = {1,0};
	SOCKET listensocket;
	socklen_t rlen;
	fd_set readfds;
	dongle_info_t dongle_info;
	pthread_t tcp_worker_thread;
	pthread_t command_thread;
	FILE *file = NULL;
	char *filename = NULL;
	int endless = 0;
#ifdef _WIN32
	u_long blockmode = 1;
	WSADATA wsd;
	WSAStartup(MAKEWORD(2,2), &wsd);
#else
	struct sigaction sigact, sigign;
#endif

	printf("rtl_file, an I/Q spectrum server for I/Q files\n");

	while ((opt = getopt(argc, argv, "a:b:e:f:l:s:vp:")) != -1)
	{
		switch (opt)
		{
		case 'a':
			addr = optarg;
			break;
		case 'e':
			endless = atoi(optarg);
			break;
		case 'f':
			filename = optarg;
			break;
		case 'l':
			buf_len = 512 * atoi(optarg);
			break;
		case 'n':
			llbuf_num = atoi(optarg);
			break;
		case 'p':
			port = atoi(optarg);
			break;
		case 's':
			samp_rate = atoi(optarg);
			break;
		case 'v':
			++verbosity;
			break;
		default:
			usage();
			break;
		}
	}
	if (argc < optind)
		usage();

	if (verbosity)
		printf("verbosity set to %d\n", verbosity);

	if (!filename)
		usage();
	file = fopen(filename, "rb");
	if (!file)
	{
		printf("Cannot open %s!\n",filename);
		return -1;
	}
	else
		printf("%s open\n",filename);

#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigign.sa_handler = SIG_IGN;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigign, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

	pthread_mutex_init(&ll_mutex, NULL);
	pthread_cond_init(&cond, NULL);

	memset(&local,0,sizeof(local));
	local.sin_family = AF_INET;
	local.sin_port = htons(port);
	local.sin_addr.s_addr = inet_addr(addr);

	listensocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	r = 1;
	setsockopt(listensocket, SOL_SOCKET, SO_REUSEADDR, (char *)&r, sizeof(int));
	setsockopt(listensocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
	bind(listensocket,(struct sockaddr *)&local,sizeof(local));

#ifdef _WIN32
	ioctlsocket(listensocket, FIONBIO, &blockmode);
#else
	r = fcntl(listensocket, F_GETFL, 0);
	r = fcntl(listensocket, F_SETFL, r | O_NONBLOCK);
#endif

	printf("listening...\n");
	printf("Use the device argument 'rtl_tcp=%s:%d' in OsmoSDR "
	       "(gr-osmosdr) source\n"
	       "to receive samples in GRC and control "
	       "rtl_tcp parameters (frequency, gain, ...).\n",
	       addr, port);
	listen(listensocket,1);

	while(1)
	{
		FD_ZERO(&readfds);
		FD_SET(listensocket, &readfds);
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		r = select(listensocket+1, &readfds, NULL, NULL, &tv);
		if(do_exit)
			goto out;
		else if(r)
		{
			rlen = sizeof(remote);
			s = accept(listensocket,(struct sockaddr *)&remote, &rlen);
			break;
		}
	}

	setsockopt(s, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));

	printf("client accepted!\n");

	memset(&dongle_info, 0, sizeof(dongle_info));
	memcpy(&dongle_info.magic, "RTL0", 4);
	r = send(s, (const char *)&dongle_info, sizeof(dongle_info), 0);
	if (sizeof(dongle_info) != r)
		printf("failed to send dongle information\n");

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	r = pthread_create(&tcp_worker_thread, &attr, tcp_worker, NULL);
	r = pthread_create(&command_thread, &attr, command_worker, NULL);
	pthread_attr_destroy(&attr);

	r = read_file(file, buf_len, samp_rate, endless);

	pthread_join(tcp_worker_thread, &status);
	pthread_join(command_thread, &status);

	closesocket(s);

	printf("all threads dead..\n");

	curelem = ll_buffers;
	ll_buffers = 0;
	while(curelem != 0)
	{
		prev = curelem;
		curelem = curelem->next;
		free(prev->data);
		free(prev);
	}
	global_numq = 0;

out:
	if (file)
		fclose(file);
	closesocket(listensocket);
	closesocket(s);
#ifdef _WIN32
	WSACleanup();
#endif
	printf("bye!\n");
	return r >= 0 ? r : -r;
}
