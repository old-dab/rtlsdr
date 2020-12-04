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

#ifdef DEBUG
#include <stdarg.h>
#include <math.h>
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

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")

typedef int socklen_t;

#else
#define closesocket close
#define SOCKADDR struct sockaddr
#define SOCKET int
#define SOCKET_ERROR -1
#endif

#include "controlThread.h"

static ctrl_thread_data_t ctrldata;

static SOCKET s;

static pthread_t tcp_worker_thread;
static pthread_t command_thread;
static pthread_cond_t exit_cond;
static pthread_mutex_t exit_cond_lock;

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

static rtlsdr_dev_t *dev = NULL;

static int verbosity = 0;

static int global_numq = 0;
static struct llist *ll_buffers = 0;
static int llbuf_num = 500;

static volatile int do_exit = 0;

#ifdef DEBUG
static FILE *gnuplotPipe; /* Pipe for communicating with gnuplot  */
static va_list vargs;  /* Holds information about variable arguments */
static int use_gnuplot = 0; /* Use gnuplot or not (optional) */

/*!
 * Execute gnuplot commands through the opened pipe.
 *
 * \param format string (command) and format specifiers
 * \return 0 on success
 */
static int gnuplot_exec(char *format, ...)
{
	va_start(vargs, format);
  	vfprintf(gnuplotPipe, format, vargs);
  	va_end(vargs);
	return 0;
}

/*!
 * Open gnuplot pipe.
 * Set labels & title.
 * Exits on failure at opening gnuplot pipe.
 *
 * \return 0 on success
 * \return 1 on given -D argument (don't use gnuplot)
 */
static int configure_gnuplot(){
	if (!use_gnuplot)
		return 1;
	gnuplotPipe = popen("gnuplot -persist", "w");
	if (!gnuplotPipe) {
		printf("Failed to open gnuplot pipe.");
		exit(1);
	}
	gnuplot_exec("set title 'Spannung am AD-Wandler' enhanced\n");
	gnuplot_exec("set xlabel 'Samples'\n");
	gnuplot_exec("set ylabel 'Spannung (mV)'\n");
	return 0;
}
#endif

void usage(void)
{
	printf("\n"
		"Usage:\t[-a listen address]\n"
		"\t[-b number of buffers (default: 15, set by library)]\n"
		"\t[-c correct I/Q-Ratio for E4000\n"
		"\t[-d device index or serial (default: 0)]\n"
		"\t[-f frequency to tune to [Hz]]\n"
		"\t[-g gain in dB (default: 0 for auto)]\n"
		"\t[-l length of single buffer in units of 512 samples (default: 64)]\n"
		"\t[-n max number of linked list buffers to keep (default: 500)]\n"
		"\t[-p listen port (default: 1234)]\n"
		"\t[-r response port (default: listen port + 1)]\n"
		"\t[-s samplerate in Hz (default: 2048000 Hz)]\n"
		"\t[-u upper sideband for R820T/R828D (default: lower sideband)]\n"
		"\t[-v increase verbosity (default: 0)]\n"
		"\t[-w rtlsdr tuner bandwidth [Hz]]\n"
		"\t[-D direct_sampling_mode (default: 0, 1 = I, 2 = Q, 3 = I below threshold, 4 = Q below threshold)]\n"
		"\t[-D direct_sampling_threshold_frequency (default: 0 use tuner specific frequency threshold for 3 and 4)]\n"
#ifdef DEBUG
		"\t[-G use Gnuplot\n"
		"\t[-I activate infrared remote control\n"
#endif
		"\t[-P ppm_error (default: 0)]\n"
		"\t[-T enable bias-T on GPIO PIN 0 (works for rtl-sdr.com v3 dongles)]\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dev);
#ifdef DEBUG
		if(use_gnuplot)
			pclose(gnuplotPipe);
#endif
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	rtlsdr_cancel_async(dev);
#ifdef DEBUG
		if(use_gnuplot)
			pclose(gnuplotPipe);
#endif
	do_exit = 1;
}
#endif

static int plot_count = 0;
static int correct_iq = 0;
static int iq_ratio = 100, old_iq_ratio = 100;

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	if(!do_exit) {
		struct llist *rpt = (struct llist*)malloc(sizeof(struct llist));
		rpt->data = (char*)malloc(len);
		memcpy(rpt->data, buf, len);
		rpt->len = len;
		rpt->next = NULL;
		plot_count++;
		if(plot_count==200)
		{
#ifdef DEBUG
			if(use_gnuplot)
			{
				float dbi, dbq;
				int i, ui, uq;
				unsigned char u;
				unsigned char min=255, max=0;
				int size = 200;

				for(i=0; i<size; i++)
				{
					u = buf[2*i];
					if (u < min)
						min = u;
					if (u > max)
						max = u;
				}
				ui = 7*(max - min)/2;
				dbi = 20 * log10(ui/2.828);
				for(i=0; i<size; i++)
				{
					u = buf[2*i+1];
					if (u < min)
						min = u;
					if (u > max)
						max = u;
				}
				uq = 7*(max - min)/2;
				dbq = 20 * log10(uq/2.828);
				printf("I = %dmVss = %.1fdBmV, Q = %dmVss = %.1fdBmV\n", ui, dbi, uq, dbq);

				gnuplot_exec("plot '-' smooth frequency with linespoints lt -1 notitle\n");
				for(i=0; i<size; i++)
					gnuplot_exec("%d	%f\n", i, 3.5*(buf[2*i]-127.5));
				/**!
				 * Stop giving points to gnuplot with 'e' command.
				 * Have to flush the output buffer for [read -> graph] persistence.
				 */
				gnuplot_exec("e\n");
				fflush(gnuplotPipe);
			}
#endif
			if(correct_iq)
			{
				int u;
				uint32_t i;
				uint32_t sum_i = 0, sum_q = 0;
				for(i=0; i<len/2; i++)
				{
					u = (buf[2*i]*2)-255;
					if(u<0) u = -u;
					sum_i += u; //Betrag der I-Abtastwerte wird addiert
					u = (buf[2*i+1]*2)-255;
					if(u<0) u = -u;
					sum_q += u;	//Betrag der Q-Abtastwerte wird addiert
				}
				iq_ratio = (sum_i * 100) / sum_q;
				if(iq_ratio != old_iq_ratio)
				{
					printf("I/Q = %3d%%  \r", iq_ratio);
					old_iq_ratio = iq_ratio;
				}
			}
			plot_count = 0;
		}
		if(iq_ratio > 100)
		{
			int u;
			uint32_t i;
			for(i=0; i<len/2; i++)
			{
				u = (buf[2*i]*2)-255;
				u = (100 * u) / iq_ratio;
				rpt->data[2*i] = (u + 255)/2;
			}
		}
		else if(iq_ratio < 100)
		{
			int u;
			uint32_t i;
			for(i=0; i<len/2; i++)
			{
				u = (buf[2*i+1]*2)-255;
				u = (iq_ratio * u) / 100;
				rpt->data[2*i+1] = (u + 255)/2;
			}
		}

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

static void *tcp_worker(void *arg)
{
	struct llist *curelem,*prev;
	int bytesleft,bytessent, index;
	struct timeval tv= {1,0};
	struct timespec ts;
	struct timeval tp;
	fd_set writefds;
	int r = 0;

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
			printf("worker cond timeout\n");
			sighandler(0);
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
						printf("worker socket bye\n");
						sighandler(0);
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

extern void print_demod_register(rtlsdr_dev_t *dev, uint8_t page);

static int set_gain_by_index(rtlsdr_dev_t *_dev, unsigned int index)
{
	int res = 0;
	int* gains;
	int count = rtlsdr_get_tuner_gains(_dev, NULL);

	if (count > 0 && (unsigned int)count > index) {
		gains = malloc(sizeof(int) * count);
		count = rtlsdr_get_tuner_gains(_dev, gains);

		res = rtlsdr_set_tuner_gain(_dev, gains[index]);
		if (verbosity)
			fprintf(stderr, "set tuner gain to %.1f dB\n", gains[index] / 10.0);

		free(gains);
	}

	return res;
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
				printf("comm recv bye\n");
				sighandler(0);
				pthread_exit(NULL);
			}
		}

		param = ntohl(cmd.param);
		switch(cmd.cmd) {
		case SET_FREQUENCY://0x01
			printf("set freq %u\n", param);
			rtlsdr_set_center_freq(dev, param);
			break;
		case SET_SAMPLE_RATE://0x02
			printf("set sample rate %u\n", param);
			rtlsdr_set_sample_rate(dev, param);
			break;
		case SET_GAIN_MODE://0x03
			printf("set gain mode %u\n", param);
			rtlsdr_set_tuner_gain_mode(dev, param);
			break;
		case SET_GAIN://0x04
			printf("set gain %u\n", param);
			rtlsdr_set_tuner_gain(dev, param);
			break;
		case SET_FREQUENCY_CORRECTION://0x05
			printf("set freq correction %d\n", (int)param);
			rtlsdr_set_freq_correction(dev, (int)param);
			break;
		case SET_IF_STAGE://0x06
			printf("set if stage %d gain %d\n", param >> 16, (short)(param & 0xffff));
			rtlsdr_set_tuner_if_gain(dev, param >> 16, (short)(param & 0xffff));
			break;
		case SET_TEST_MODE://0x07
			printf("set test mode %u\n", param);
			rtlsdr_set_testmode(dev, param);
			break;
		case SET_AGC_MODE://0x08
			printf("set agc mode %u\n", param);
			rtlsdr_set_agc_mode(dev, param);
			break;
		case SET_DIRECT_SAMPLING://0x09
			printf("set direct sampling %u\n", param);
			rtlsdr_set_direct_sampling(dev, param);
			break;
		case SET_OFFSET_TUNING://0x0a
			printf("set offset tuning %d\n", (int)param);
			rtlsdr_set_offset_tuning(dev, (int)param);
			break;
		case SET_RTL_CRYSTAL://0x0b
			printf("set rtl xtal %u\n", param);
			rtlsdr_set_xtal_freq(dev, param, 0);
			break;
		case SET_TUNER_CRYSTAL://0x0c
			printf("set tuner xtal %u\n", param);
			rtlsdr_set_xtal_freq(dev, 0, param);
			break;
		case SET_TUNER_GAIN_BY_INDEX://0x0d
			printf("set tuner gain by index %u\n", param);
			set_gain_by_index(dev, param);
			break;
		case SET_BIAS_TEE://0x0e
			printf("set bias tee %u\n", param);
			rtlsdr_set_bias_tee(dev, param);
			break;
		case SET_TUNER_BANDWIDTH://0x40
			printf("set tuner bandwidth to %u Hz\n", param);
			verbose_set_bandwidth(dev, param);
			break;
		case SET_I2C_TUNER_REGISTER://0x43
			printf("set i2c register x%03X to x%03X with mask x%02X\n", (param >> 20) & 0xfff, param & 0xfff, (param >> 12) & 0xff );
			rtlsdr_set_tuner_i2c_register(dev, (param >> 20) & 0xfff, (param >> 12) & 0xff, param & 0xfff);
			break;
		case SET_SIDEBAND://0x46
			printf("set to %s sideband\n", param ? "upper" : "lower");
			rtlsdr_set_tuner_sideband(dev, param);
			break;
		case REPORT_I2C_REGS://0x48
			if(param)
				param = 1;
			ctrldata.report_i2c = param;  /* (de)activate reporting */
			printf("read registers %d\n", param);
			printf("activating response channel on port %d with %s I2C reporting\n",
					ctrldata.port, (param ? "active" : "inactive") );
			break;
		case SET_DITHERING://0x49
			printf("%sable dithering\n", param ? "en" : "dis");
			rtlsdr_set_dithering(dev, param);
			break;
		default:
			break;
		}
		cmd.cmd = 0xff;
	}
	return NULL;
}

#ifdef DEBUG
struct ir_thread_data
{
	rtlsdr_dev_t *dev;
	int ir_table;
};

static void *ir_thread_fn(void *arg)
{
	int r = 0;
	int i;
	uint8_t buf[128];
	unsigned int wait_usec = 80000;
	struct ir_thread_data *data = (struct ir_thread_data *)arg;
	rtlsdr_dev_t *dev = data->dev;

	while (!do_exit) {
		usleep(wait_usec);

		r = rtlsdr_ir_query(dev, buf, sizeof(buf));
		if (r < 0) {
			fprintf(stderr, "rtlsdr_ir_query failed: %d\n", r);
		}
		if(r == 70) //NEC code
		{
			uint32_t code = 0;
			int leading = 0;
			int page = -1;
			for (i = 0; i < r; i++)
			{
				int pulse = buf[i] >> 7;
				int duration = (buf[i] & 0x7f) * 50;
				if(i == 0) //leading pulse burst
				{
					if (pulse == 1)
						leading += duration;
					else
						break;
				}
				if(i == 1) //leading pulse burst
				{
					if (pulse == 1)
						leading += duration;
					else
						break;
					if ((leading < 8000) || (leading > 10000))
						break;
				}
				if(i == 2) //space
				{
					if ((pulse == 1) || (duration < 4000) || (duration > 5000))
						break;
				}
				if ((i > 2) && ((i & 1) == 1)) //pulse burst
				{
					if ((pulse == 0) || (duration < 300) || (duration > 1000))
						break;
				}
				if ((i > 3) && ((i & 1) == 0)) //space
				{
					if ((pulse == 1) || (duration < 300) || (duration > 3000))
						break;
					if(duration < 1000)
						code = code >> 1;
					else
						code = (code >> 1) | 0x80000000;
				}
			}
			printf("NEC code 0x%0x\n",code);
			switch ((code >> 16) & 0xff) {
				case 0x12:
					page = 0;
					break;
				case 0x09:
					page = 1;
					break;
				case 0x1d:
					page = 2;
					break;
				case 0x1f:
					page = 3;
					break;
				case 0x0d:
					page = 4;
					break;
			}
			if(page >= 0 && page < 5)
				print_demod_register(dev, page);


		}
		else if(r == 6) //Repeat code
		{
			int leading = 0;
			for (i = 0; i < r; i++)
			{
				int pulse = buf[i] >> 7;
				int duration = (buf[i] & 0x7f) * 50;
				if(i == 0) //leading pulse burst
				{
					if (pulse == 1)
						leading += duration;
					else
						break;
				}
				if(i == 1) //leading pulse burst
				{
					if (pulse == 1)
						leading += duration;
					else
						break;
					if ((leading < 8000) || (leading > 10000))
						break;
				}
				if(i == 2) //space
				{
					if ((pulse == 1) || (duration < 1500) || (duration > 3000))
						break;
				}
				if (i == 3) //pulse burst
				{
					if ((pulse == 0) || (duration < 300) || (duration > 1000))
						break;
				}
			}
			printf("NEC Repeat code\n");
		}
		/*else if(r>0)
		{
			int j;
			printf("length=%d\n",r);
	 		for (i = 0; i < r; i++)
	 		{
				int pulse = buf[i] >> 7;
				int duration = buf[i] & 0x7f;
				for (j = 0; j < duration; ++j)
					printf("%d", pulse);
			}
			if (r != 0) printf("\n");
		}*/
	}
	return NULL;
}
#endif

int main(int argc, char **argv)
{
	int r, opt, i;
	char* addr = "127.0.0.1";
	int port = 1234;
	pthread_t thread_ir;
	pthread_t thread_ctrl; //-cs- for periodically reading the register values
	int port_resp = 1;
	int report_i2c = 1;
	int do_exit_thrd_ctrl = 0;
	int cal_imr = 1;

	uint32_t frequency = 100000000, samp_rate = 2048000;
	enum rtlsdr_ds_mode ds_mode = RTLSDR_DS_IQ;
	uint32_t ds_temp, ds_threshold = 0;
	struct sockaddr_in local, remote;
	uint32_t buf_num = 0;
	int sideband = 0;
	/* buf_len:
	 * must be multiple of 512 - else it will be overwritten
	 * in rtlsdr_read_async() in librtlsdr.c with DEFAULT_BUF_LENGTH (= 16*32 *512 = 512 *512)
	 *
	 * -> 512*512 -> 1048 ms @ 250 kS  or  81.92 ms @ 3.2 MS (internal default)
	 * ->  32*512 ->   65 ms @ 250 kS  or   5.12 ms @ 3.2 MS (new default)
	 */
	uint32_t buf_len = 64 * 512;
	int dev_index = 0;
	int dev_given = 0;
	int gain = 0;
	int ppm_error = 0;
	struct llist *curelem,*prev;
	pthread_attr_t attr;
	void *status;
	struct timeval tv = {1,0};
	struct linger ling = {1,0};
	SOCKET listensocket;
	socklen_t rlen;
	fd_set readfds;
	u_long blockmode = 1;
	dongle_info_t dongle_info;
	int gains[100];
	uint32_t bandwidth = 0;
	int enable_biastee = 0;
#ifdef DEBUG
	int port_ir = 0;
#endif

#ifdef _WIN32
	WSADATA wsd;
	i = WSAStartup(MAKEWORD(2,2), &wsd);
#else
	struct sigaction sigact, sigign;
#endif

	printf("rtl_tcp, an I/Q spectrum server for RTL2832 based DVB-T receivers\n"
		   "Version 0.94 for QIRX, %s\n\n", __DATE__);

#ifdef DEBUG
	while ((opt = getopt(argc, argv, "a:b:cd:f:g:l:n:O:p:us:vr:w:D:GITP:")) != -1) {
#else
	while ((opt = getopt(argc, argv, "a:b:cd:f:g:l:n:O:p:us:vr:w:D:TP:")) != -1) {
#endif
		switch (opt) {
		case 'a':
			addr = optarg;
			break;
		case 'b':
			buf_num = atoi(optarg);
			break;
		case 'c':
			correct_iq = 1;
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
		case 'l':
			buf_len = 512 * atoi(optarg);
			break;
		case 'n':
			llbuf_num = atoi(optarg);
			break;
		case 'p':
			port = atoi(optarg);
			break;
		case 'r':
			port_resp = atoi(optarg);
			if(port_resp == 0)
				cal_imr = 0;
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
		case 'G':
			use_gnuplot = 1;
			break;
#endif
		case 'I':
			port_ir = 1;
			break;
		case 'P':
			ppm_error = atoi(optarg);
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
	rtlsdr_open(&dev, (uint32_t)dev_index);
	if (NULL == dev) {
	fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}

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
		verbose_auto_gain(dev);
		rtlsdr_set_agc_mode(dev, 1);
		printf("set agc mode 1\n");

	} else {
		// Enable manual gain
		gain = nearest_gain(dev, gain);
		verbose_gain_set(dev, gain);
	}

	if(sideband)
	{
		rtlsdr_set_tuner_sideband(dev, sideband);
		fprintf(stderr, "Set to upper sideband\n");
	}
	verbose_set_bandwidth(dev, bandwidth);

	rtlsdr_set_bias_tee(dev, enable_biastee);
	if (enable_biastee)
		fprintf(stderr, "activated bias-T on GPIO PIN 0\n");

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dev);

#ifdef DEBUG
	configure_gnuplot();
#endif

	pthread_mutex_init(&exit_cond_lock, NULL);
	pthread_mutex_init(&ll_mutex, NULL);
	pthread_mutex_init(&exit_cond_lock, NULL);
	pthread_cond_init(&cond, NULL);
	pthread_cond_init(&exit_cond, NULL);
#ifdef DEBUG
	if(port_ir)
	{
		struct ir_thread_data data = {.dev = dev, .ir_table = port_ir};
		pthread_create(&thread_ir, NULL, &ir_thread_fn, (void *)(&data));
	}
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
		printf("listening...\n");
		printf("Use the device argument 'rtl_tcp=%s:%d' in OsmoSDR "
		       "(gr-osmosdr) source\n"
		       "to receive samples in GRC and control "
		       "rtl_tcp parameters (frequency, gain, ...).\n",
		       addr, port);
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

		printf("client accepted!\n");

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
			printf("failed to send dongle information\n");

		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		r = pthread_create(&tcp_worker_thread, &attr, tcp_worker, NULL);
		r = pthread_create(&command_thread, &attr, command_worker, NULL);
		pthread_attr_destroy(&attr);

		r = rtlsdr_read_async(dev, rtlsdr_callback, NULL, buf_num, buf_len);

		pthread_join(tcp_worker_thread, &status);
		pthread_join(command_thread, &status);

		closesocket(s);

		printf("all threads dead..\n");
		curelem = ll_buffers;
		ll_buffers = 0;

		while(curelem != 0) {
			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}

		do_exit = 0;
		global_numq = 0;
	}

out:
	rtlsdr_close(dev);
	closesocket(listensocket);
	if ( port_resp ) {
		do_exit_thrd_ctrl = 1;
		ctrldata.pDoExit = &do_exit_thrd_ctrl;
		pthread_join(thread_ctrl, &status);
	}
	closesocket(s);
#ifdef _WIN32
	WSACleanup();
#endif
	printf("bye!\n");
	return r >= 0 ? r : -r;
}
