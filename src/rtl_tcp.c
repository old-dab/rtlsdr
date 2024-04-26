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
#ifndef CLK_TCK
#define	CLK_TCK	sysconf(_SC_CLK_TCK)	/* clock ticks per second */
#endif
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
#include "controlThread.h"
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

static ctrl_thread_data_t ctrldata;

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

struct iq_state
{
	int show_level;
	int plot_count;
	int correct_iq;
	float effective;
	float dbi;
	float levelI;
	float levelQ;
	float ratio;
};

#define DC_OFFSET (float)127.38

static rtlsdr_dev_t *dev = NULL;

static int verbosity = 0;
static clock_t time1 = 0;
static int global_numq = 0;
static struct llist *ll_buffers = 0;
static int llbuf_num = 500;
static int gain_mode = 0;

static volatile int do_exit = 0;
static volatile int ctrlC_exit = 0;

int overload = 0;

#ifdef DEBUG
static void show_adc_level(uint8_t *buf, int len, struct iq_state *iq)
{
	int i;
	float U;

    for (i = 0; i < len; i+=2)
    {
		U = (float)buf[i]-DC_OFFSET;
        iq->effective += iq->ratio * (U*U - iq->effective);
	}
	iq->plot_count += 1;
	if(iq->plot_count==125)
	{
		float dbi;
		U = sqrt(iq->effective) * 3.5;
		dbi = 20 * log10(U);
		if(dbi >= iq->dbi+0.01f || dbi <= iq->dbi-0.01f)
		{
			fprintf(stderr, "I = %.1f mV = %.2f dBmV\n", U, dbi);
			iq->dbi = dbi;
		}
		iq->plot_count = 0;
	}
}
#endif

void usage(void)
{
	fprintf(stderr, "rtl_tcp, an I/Q spectrum server for RTL2832 based DVB-T receivers\n"
		   "Version %d.%d.%d.%d for QIRX, %s\n",
		   RTLSDR_MAJOR, RTLSDR_MINOR, RTLSDR_MICRO, RTLSDR_NANO, __DATE__);
	fprintf(stderr, "rtlsdr library %d.%d.%d.%d %s\n\n",
		rtlsdr_get_version()>>24, rtlsdr_get_version()>>16 & 0xFF,
		rtlsdr_get_version()>>8 & 0xFF, rtlsdr_get_version() & 0xFF,
		rtlsdr_get_ver_id() );
	fprintf(stderr, "Usage:\t[-a listen address (default: 127.0.0.1)]\n"
		"\t[-b number of buffers (default: 15, set by library)]\n"
		"\t[-c correct I/Q-Ratio\n"
		"\t[-d device index or serial (default: 0)]\n"
		"\t[-f frequency to tune to [Hz]]\n"
		"\t[-g gain in dB (default: 0 for auto)]\n"
		"\t    0 = hardware AGC, <0 = software AGC, >0 = gain in dB\n"
		"\t[-k calibrate image rejection for R820T/R828D and store the results in EEPROM\n"
		"\t[-l length of single buffer in units of 512 samples (default: 256)]\n"
		"\t[-n max number of linked list buffers to keep (default: 500)]\n"
		"\t[-o set offset tuning\n"
		"\t[-p listen port (default: 1234)]\n"
		"\t[-r response port (default: listen port + 1)]\n"
		"\t[-s samplerate in Hz (default: 2048000 Hz)]\n"
		"\t[-u upper sideband for R820T/R828D (default: lower sideband)]\n"
		"\t[-v increase verbosity (default: 0)]\n"
		"\t[-w tuner bandwidth in Hz\n"
		"\t[-D direct_sampling_mode (default: 0, 1 = I, 2 = Q, 3 = I below threshold, 4 = Q below threshold)]\n"
		"\t[-D direct_sampling_threshold_frequency (default: 0 use tuner specific frequency threshold for 3 and 4)]\n"
#ifdef DEBUG
		"\t[-L show ADC level\n"
		"\t[-I debug input from keyboard\n"
#endif
		"\t[-P ppm_error (default: 0)]\n"
		"\t[-T enable bias-T on GPIO PIN 0 (works for rtl-sdr.com v3/v4 dongles)]\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "CTRL-C caught, exiting!\n");
		do_exit = 1;
		ctrlC_exit = 1;
		rtlsdr_cancel_async(dev);
		return TRUE;
	}
	else if (CTRL_CLOSE_EVENT == signum) {
		fprintf(stderr, "SIGQUIT caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal (%d) caught, ask for exit!\n", signum);
	rtlsdr_cancel_async(dev);
	do_exit = 1;
}
#endif

static void iqBalance(uint8_t *buf, int len, struct iq_state *iq)
{
	int pos;
	float iq_ratio;

    for (pos = 0; pos < len; pos+=2)
    {
        iq->levelI += iq->ratio * (fabsf((float)buf[pos]-DC_OFFSET) - iq->levelI);
        iq->levelQ += iq->ratio * (fabsf((float)buf[pos+1]-DC_OFFSET) - iq->levelQ);
	}
	iq_ratio = iq->levelI / iq->levelQ;
	//fprintf(stderr, "I = %f, Q = %f, I/Q = %f%%\n", iq->levelI, iq->levelQ, iq_ratio);
	if(iq_ratio > 1.01)
	{
	    for (pos = 0; pos < len; pos+=2)
			buf[pos] = ((float)buf[pos]-DC_OFFSET) / iq_ratio + DC_OFFSET;
	}
	else if(iq_ratio < 0.99)
	{
	    for (pos = 1; pos < len; pos+=2)
			buf[pos] = ((float)buf[pos]-DC_OFFSET) * iq_ratio + DC_OFFSET;
	}
}

static int detect_overload(uint8_t *buf, int len)
{
	int overload_count = 0;
	for(int i=0; i<len; i++)
	{
		if ((buf[i] == 0) || (buf[i] == 255))
			overload_count++;
	}
	return (8000 * overload_count >= len);
}

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	struct iq_state *iq = ctx;
	clock_t time2;

	if (!ctx) {
		return;}

	if(!do_exit) {
		struct llist *rpt = (struct llist*)malloc(sizeof(struct llist));
		rpt->data = (char*)malloc(len);
		time2 = clock();
		if((time2 - time1) >= (CLK_TCK / 4)) // >= 0.25s
		{
			overload = detect_overload(buf, len);
			time1 = time2;
		}
		if(iq->correct_iq)
			iqBalance(buf, len, iq);
		memcpy(rpt->data, buf, len);
		rpt->len = len;
		rpt->next = NULL;
#ifdef DEBUG
		if(iq->show_level)
			show_adc_level(buf, len, iq);
#endif
		pthread_mutex_lock(&ll_mutex);
		if (ll_buffers == NULL) {
			ll_buffers = rpt;
		} else {
			struct llist *cur = ll_buffers;
			int num_queued = 0;

			while (cur->next != NULL) {
				cur = cur->next;
				num_queued++;
			}
			if(llbuf_num && llbuf_num == num_queued-2){
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
					fprintf(stderr, "ll+, now %d\n", num_queued);
				else if (num_queued < global_numq)
					fprintf(stderr, "ll-, now %d\n", num_queued);
			}

			global_numq = num_queued;
		}
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&ll_mutex);
	}
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

	while(1) {
		if(do_exit)
			pthread_exit(0);

		pthread_mutex_lock(&ll_mutex);
		gettimeofday(&tp, NULL);
		ts.tv_sec  = tp.tv_sec+1;
		ts.tv_nsec = tp.tv_usec * 1000;
		r = pthread_cond_timedwait(&cond, &ll_mutex, &ts);
		if(r == ETIMEDOUT) {
			pthread_mutex_unlock(&ll_mutex);
			fprintf(stderr, "worker cond timeout\n");
			sighandler(CTRL_CLOSE_EVENT);
			pthread_exit(NULL);
		}

		curelem = ll_buffers;
		ll_buffers = 0;
		pthread_mutex_unlock(&ll_mutex);

		while(curelem != 0) {
			bytesleft = curelem->len;
			index = 0;
			bytessent = 0;
			while(bytesleft > 0) {
				FD_ZERO(&writefds);
				FD_SET(s, &writefds);
				tv.tv_sec = 1;
				tv.tv_usec = 0;
				r = select(s+1, NULL, &writefds, NULL, &tv);
				if(r) {
					bytessent = send(s,  &curelem->data[index], bytesleft, 0);
					bytesleft -= bytessent;
					index += bytessent;
				}
				if(bytessent == SOCKET_ERROR || do_exit) {
#ifdef _WIN32
					fprintf(stderr, "worker socket bye (%d), do_exit:%d\n", WSAGetLastError(), do_exit);
#else
					fprintf(stderr, "worker socket bye, do_exit:%d\n", do_exit);
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

	while(1) {
		left=sizeof(cmd);
		while(left >0) {
			FD_ZERO(&readfds);
			FD_SET(s, &readfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(s+1, &readfds, NULL, NULL, &tv);
			if(r) {
				received = recv(s, (char*)&cmd+(sizeof(cmd)-left), left, 0);
				left -= received;
			}
			if(received == SOCKET_ERROR || do_exit) {
#ifdef _WIN32
				fprintf(stderr, "comm recv bye (%d), do_exit:%d\n", WSAGetLastError(), do_exit);
#else
				fprintf(stderr, "comm recv bye, do_exit:%d\n", do_exit);
#endif
				sighandler(CTRL_CLOSE_EVENT);
				pthread_exit(NULL);
			}
		}

		param = ntohl(cmd.param);
		switch(cmd.cmd) {
		case SET_FREQUENCY://0x01
			fprintf(stderr, "set freq %u\n", param);
			rtlsdr_set_center_freq(dev, param);
			break;
		case SET_SAMPLE_RATE://0x02
			fprintf(stderr, "set sample rate %u\n", param);
			rtlsdr_set_sample_rate(dev, param);
			break;
		case SET_GAIN_MODE://0x03
			fprintf(stderr, "set gain mode %u\n", param);
			gain_mode = param;
			rtlsdr_set_tuner_gain_mode(dev, param);
			break;
		case SET_GAIN://0x04
			fprintf(stderr, "set gain %u\n", param);
			if(gain_mode == 1)
				rtlsdr_set_tuner_gain(dev, param);
			break;
		case SET_FREQUENCY_CORRECTION://0x05
			fprintf(stderr, "set freq correction to %d ppm\n", (int)param);
			rtlsdr_set_freq_correction(dev, (int)param);
			break;
		case SET_IF_STAGE://0x06
			fprintf(stderr, "set if stage %d gain %d\n", param >> 16, (short)(param & 0xffff));
			if(gain_mode == 1)
				rtlsdr_set_tuner_if_gain(dev, param >> 16, (short)(param & 0xffff));
			break;
		case SET_TEST_MODE://0x07
			fprintf(stderr, "set test mode %u\n", param);
			rtlsdr_set_testmode(dev, param);
			break;
		case SET_AGC_MODE://0x08
			fprintf(stderr, "set agc mode %u\n", param);
			rtlsdr_set_agc_mode(dev, param);
			break;
		case SET_DIRECT_SAMPLING://0x09
			fprintf(stderr, "set direct sampling %u\n", param);
			rtlsdr_set_direct_sampling(dev, param);
			break;
		case SET_OFFSET_TUNING://0x0a
			fprintf(stderr, "set offset tuning %d\n", (int)param);
			rtlsdr_set_offset_tuning(dev, (int)param);
			break;
		case SET_RTL_CRYSTAL://0x0b
			fprintf(stderr, "set rtl xtal %u\n", param);
			rtlsdr_set_xtal_freq(dev, param, 0);
			break;
		case SET_TUNER_CRYSTAL://0x0c
			fprintf(stderr, "set tuner xtal %u\n", param);
			rtlsdr_set_xtal_freq(dev, 0, param);
			break;
		case SET_TUNER_GAIN_BY_INDEX://0x0d
			fprintf(stderr, "set tuner gain by index %u\n", param);
			if(gain_mode == 1)
				rtlsdr_set_tuner_gain_index(dev, param);
			break;
		case SET_BIAS_TEE://0x0e
			fprintf(stderr, "set bias tee %u\n", param);
			rtlsdr_set_bias_tee(dev, param);
			break;
		case SET_TUNER_BANDWIDTH://0x40
			fprintf(stderr, "set tuner bandwidth to %u Hz\n", param);
			verbose_set_bandwidth(dev, param);
			break;
		case SET_I2C_TUNER_REGISTER://0x43
			fprintf(stderr, "set i2c register x%03X to x%03X with mask x%02X\n", (param >> 20) & 0xfff, param & 0xfff, (param >> 12) & 0xff );
			rtlsdr_set_tuner_i2c_register(dev, (param >> 20) & 0xfff, (param >> 12) & 0xff, param & 0xfff);
			break;
		case SET_SIDEBAND://0x46
			fprintf(stderr, "set to %s sideband\n", param ? "upper" : "lower");
			rtlsdr_set_tuner_sideband(dev, param);
			break;
		case REPORT_I2C_REGS://0x48
			if(param)
				param = 1;
			ctrldata.report_i2c = param;  /* (de)activate reporting */
			fprintf(stderr, "read registers %d\n", param);
			fprintf(stderr, "activating response channel on port %d with %s I2C reporting\n",
					ctrldata.port, (param ? "active" : "inactive") );
			break;
		case SET_DITHERING://0x49
			fprintf(stderr, "%sable dithering\n", param ? "en" : "dis");
			rtlsdr_set_dithering(dev, param);
			break;
		case SET_FREQUENCY_CORRECTION_PPB://0x83
			fprintf(stderr, "set freq correction to %0.3f ppm\n", (int)param/1000.0);
			rtlsdr_set_freq_correction_ppb(dev, (int)param);
			break;
		default:
			break;
		}
		cmd.cmd = 0xff;
	}
	return NULL;
}

#ifdef DEBUG
static void *kb_thread_fn(void *arg)
{
	char keybuf[80];
	int agc = 0;
	int len;

	while (!do_exit) {
		usleep(100000);
	    if(scanf("%s", keybuf) < 1)
			return NULL;
	    len = strlen(keybuf);
	    if(len == 1)
	    	switch (keybuf[0])
	    	{
				case '0':
				case '1':
				case '2':
				case '3':
				case '4':
					print_demod_register(dev, keybuf[0]-'0');
					break;
				case '5':
					print_usb_register(dev, 0x2000);
					break;
				case '6':
					print_usb_register(dev, 0x2100);
					break;
				case '7':
					print_usb_register(dev, 0x3000);
					break;
				case '8':
					print_usb_register(dev, 0xfc00);
					break;
				case '9':
					print_usb_register(dev, 0xfd00);
					break;
				case 'a':
					if(agc) agc = 0;
					else agc = 1;
					rtlsdr_set_agc_mode(dev, agc);
					fprintf(stderr, "set agc mode %u\n", agc);
					break;
				case 'r':
					rtlsdr_reset_demod(dev);
					fprintf(stderr, "reset demod\n");
					break;
			}
		else if((len == 6) && (keybuf[0] == 'w'))
		{
			unsigned int val, page, adr;
			if (sscanf(keybuf+1,"%x",&val) == 1)
			{
				page = (val >> 16) & 0xf;
				adr = (val >> 8) & 0xff;
				val = val & 0xff;
				fprintf(stderr, "Write Page=%d, Adress=%x, Value=%x\n", page, adr, val);
				rtlsdr_demod_write_reg(dev, page, adr, val, 1);
			}
		}
	}
	return NULL;
}
#endif

int main(int argc, char **argv)
{
	int r, opt, i;
	char* addr = "127.0.0.1";
	int port = 1234;
	pthread_t thread_ctrl; //-cs- for periodically reading the register values
	int port_resp = 1;
	int report_i2c = 1;
	int do_exit_thrd_ctrl = 0;
	int cal_imr = 0;
	uint32_t frequency = 100000000, samp_rate = 2048000;
	enum rtlsdr_ds_mode ds_mode = RTLSDR_DS_IQ;
	uint32_t ds_temp, ds_threshold = 0;
	struct sockaddr_in local, remote;
	uint32_t buf_num = 0;
	int sideband = 0;
	int offset_tuning = 0;
	/* buf_len:
	 * must be multiple of 512 - else it will be overwritten
	 * in rtlsdr_read_async() in librtlsdr.c with DEFAULT_BUF_LENGTH (= 64*512)
	 *
	 * -> 512*512 -> 1048 ms @ 250 kS  or  81.92 ms @ 3.2 MS (internal default)
	 * ->  32*512 ->   65 ms @ 250 kS  or   5.12 ms @ 3.2 MS (new default)
	 */
	uint32_t buf_len = 256 * 512;
	int dev_index = 0;
	int dev_given = 0;
	int gain = 0;
	float ppm_error = 0;
	struct llist *curelem,*prev;
	pthread_attr_t attr;
	void *status;
	struct timeval tv = {1,0};
	struct linger ling = {1,0};
	struct iq_state iq = {0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 1E-05f};
	SOCKET listensocket;
	socklen_t rlen;
	fd_set readfds;
	dongle_info_t dongle_info;
	int gains[100];
	uint32_t bandwidth = 0;
	int enable_biastee = 0;
	pthread_t tcp_worker_thread;
	pthread_t command_thread;
	uint64_t StartTime, EndTime;
#ifdef DEBUG
	pthread_t kb_thread;
	int enable_kb_thread = 0;
#endif
#ifdef _WIN32
	u_long blockmode = 1;
	WSADATA wsd;
	i = WSAStartup(MAKEWORD(2,2), &wsd);
#else
	struct sigaction sigact, sigign;
#endif

	/*fprintf(stderr, "rtl_tcp, an I/Q spectrum server for RTL2832 based DVB-T receivers\n"
		   "Version %d.%d.%d.%d for QIRX, %s\n\n",
		   RTLSDR_MAJOR, RTLSDR_MINOR, RTLSDR_MICRO, RTLSDR_NANO, __DATE__);*/

#ifdef DEBUG
	while ((opt = getopt(argc, argv, "a:b:cd:f:g:kl:n:op:us:vr:w:D:LITP:h")) != -1) {
#else
	while ((opt = getopt(argc, argv, "a:b:cd:f:g:kl:n:op:us:vr:w:D:TP:h")) != -1) {
#endif
		switch (opt) {
		case 'a':
			addr = optarg;
			break;
		case 'b':
			buf_num = atoi(optarg);
			break;
		case 'c':
			iq.correct_iq = 1;
			break;
		case 'd':
			dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			frequency = (uint32_t)atofs(optarg);
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10); /* tenths of a dB */
			break;
		case 'k':
			++cal_imr;
			break;
		case 'l':
			buf_len = 512 * atoi(optarg);
			break;
		case 'n':
			llbuf_num = atoi(optarg);
			break;
		case 'o':
			offset_tuning = 1;
			break;
		case 'p':
			port = atoi(optarg);
			break;
		case 'r':
			port_resp = atoi(optarg);
			break;
		case 's':
			samp_rate = (uint32_t)atofs(optarg);
			break;
		case 'u':
			sideband = 1;
			break;
		case 'v':
			++verbosity;
			break;
		case 'w':
			bandwidth = (uint32_t)atofs(optarg);
			break;
		case 'D':
			ds_temp = (uint32_t)( atofs(optarg) + 0.5 );
			if (ds_temp <= RTLSDR_DS_Q_BELOW)
				ds_mode = (enum rtlsdr_ds_mode)ds_temp;
			else
				ds_threshold = ds_temp;
			break;
#ifdef DEBUG
		case 'L':
			iq.show_level = 1;
			break;
		case 'I':
			enable_kb_thread = 1;
			break;
#endif
		case 'P':
			ppm_error = atof(optarg);
			break;
		case 'T':
			enable_biastee = 1;
			break;
		default:
			usage();
			break;
		}
	}

	if (argc < optind)
		usage();

	if (verbosity)
		fprintf(stderr, "verbosity set to %d\n", verbosity);

	if (!dev_given) {
		dev_index = verbose_device_search("0");
	}

	if (dev_index < 0) {
	    exit(1);
	}

	rtlsdr_cal_imr(cal_imr);

	gettimeofday(&tv, NULL);
	StartTime = (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
	rtlsdr_open(&dev, (uint32_t)dev_index);
	if (NULL == dev) {
	fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}
	gettimeofday(&tv, NULL);
	EndTime = (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
	fprintf(stderr, "%d msec\n", (int)(EndTime-StartTime));

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

	if(iq.correct_iq == 1)
		fprintf(stderr, "Correct I/Q balance\n");
	/* Set the tuner error */
	verbose_ppm_set(dev, ppm_error);

	/* Set the sample rate */
	verbose_set_sample_rate(dev, samp_rate);

	/* Set direct sampling with threshold */
	rtlsdr_set_ds_mode(dev, ds_mode, ds_threshold);

	/* Set the frequency */
	r = rtlsdr_set_center_freq(dev, frequency);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set center freq.\n");
	else
		fprintf(stderr, "Tuned to %i Hz.\n", frequency);

	if (gain == 0) {
		// Enable automatic gain
		gain_mode = 0;
		verbose_auto_gain(dev);
		rtlsdr_set_agc_mode(dev, 1);
		fprintf(stderr, "Set agc mode 1\n");
	}
	else if(gain < 0) {
		fprintf(stderr, "Set software AGC\n");
		gain_mode = 2;
		rtlsdr_set_tuner_gain_mode(dev, 2);
	} else {
		// Enable manual gain
		gain_mode = 1;
		gain = nearest_gain(dev, gain);
		verbose_gain_set(dev, gain);
	}

	if(sideband)
	{
		rtlsdr_set_tuner_sideband(dev, sideband);
		fprintf(stderr, "Set to upper sideband\n");
	}
	verbose_set_bandwidth(dev, bandwidth);
	if(offset_tuning)
	{
		fprintf(stderr, "set offset tuning\n");
		rtlsdr_set_offset_tuning(dev, 1);
	}
	rtlsdr_set_bias_tee(dev, enable_biastee);
	if (enable_biastee)
		fprintf(stderr, "activated bias-T on GPIO PIN 0\n");

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dev);

	pthread_mutex_init(&ll_mutex, NULL);
	pthread_cond_init(&cond, NULL);
#ifdef DEBUG
	if(enable_kb_thread)
		pthread_create(&kb_thread, NULL, &kb_thread_fn, NULL);
#endif
	if ( port_resp == 1 )
		port_resp = port + 1;
	ctrldata.port = port_resp;
	ctrldata.dev = dev;
	ctrldata.addr = addr;
	ctrldata.wait = 500000; /* = 0.5 sec */
	ctrldata.report_i2c = report_i2c;
	ctrldata.pDoExit = &do_exit_thrd_ctrl;
	if( port_resp )
	{
		if ( port_resp != (port+1))
			fprintf(stderr, "activating response channel on port %d with %s I2C reporting\n",
					port_resp, (report_i2c ? "active" : "inactive") );
		pthread_create(&thread_ctrl, NULL, &ctrl_thread_fn, &ctrldata);
	}

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

	while(1) {
		fprintf(stderr, "listening on port %d...\n", port);
		/*fprintf(stderr, "Use the device argument 'rtl_tcp=%s:%d' in OsmoSDR "
		       "(gr-osmosdr) source\n"
		       "to receive samples in GRC and control "
		       "rtl_tcp parameters (frequency, gain, ...).\n",
		       addr, port);*/
		listen(listensocket,1);

		while(1) {
			FD_ZERO(&readfds);
			FD_SET(listensocket, &readfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(listensocket+1, &readfds, NULL, NULL, &tv);
			if(do_exit) {
				goto out;
			} else if(r) {
				rlen = sizeof(remote);
				s = accept(listensocket,(struct sockaddr *)&remote, &rlen);
				break;
			}
		}

		setsockopt(s, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));

		fprintf(stderr, "client accepted!\n");

		memset(&dongle_info, 0, sizeof(dongle_info));
		memcpy(&dongle_info.magic, "RTL0", 4);

		r = rtlsdr_get_tuner_type(dev);
		if (r >= 0)
			dongle_info.tuner_type = htonl(r);
		r = rtlsdr_get_tuner_gains(dev, gains);
		if (r >= 0)
			dongle_info.tuner_gain_count = htonl(r);
		if (verbosity)
		{
			fprintf(stderr, "Supported gain values (%d): ", r);
			for (i = 0; i < r; i++)
				fprintf(stderr, "%.1f ", gains[i] / 10.0);
			fprintf(stderr, "\n");
		}

		r = send(s, (const char *)&dongle_info, sizeof(dongle_info), 0);
		if (sizeof(dongle_info) != r)
			fprintf(stderr, "failed to send dongle information\n");

		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		r = pthread_create(&tcp_worker_thread, &attr, tcp_worker, NULL);
		r = pthread_create(&command_thread, &attr, command_worker, NULL);
		pthread_attr_destroy(&attr);

		r = rtlsdr_read_async(dev, rtlsdr_callback, (void *)&iq, buf_num, buf_len);

		pthread_join(tcp_worker_thread, &status);
		pthread_join(command_thread, &status);

		closesocket(s);

		fprintf(stderr, "all threads dead..\n");
		curelem = ll_buffers;
		ll_buffers = 0;

		while(curelem != 0) {
			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}

		if (!ctrlC_exit) do_exit = 0;
		global_numq = 0;
	}

out:
	closesocket(listensocket);
	if ( port_resp ) {
		do_exit_thrd_ctrl = 1;
		ctrldata.pDoExit = &do_exit_thrd_ctrl;
		pthread_join(thread_ctrl, &status);
	}
#ifdef DEBUG
	if(enable_kb_thread)
		pthread_join(kb_thread, &status);
#endif
	if(dev)
		rtlsdr_close(dev);
	closesocket(s);
#ifdef _WIN32
	WSACleanup();
#endif
	fprintf(stderr, "bye!\n");
	return r >= 0 ? r : -r;
}
