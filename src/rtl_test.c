/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * rtl_test, test and benchmark tool
 *
 * Copyright (C) 2012-2014 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012-2014 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2014 by Michael Tatarinov <kukabu@gmail.com>
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
#include <math.h>

#ifndef _WIN32
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#else
#include <windows.h>
#include "getopt/getopt.h"
#endif

#include "rtl-sdr.h"
#include "convenience/convenience.h"
#include "version.h"

#define DEFAULT_SAMPLE_RATE		2048000

#define MHZ(x)					((x)*1000*1000)

#define PPM_DURATION			10
#define PPM_DUMP_TIME			5

struct time_generic
/* holds all the platform specific values */
{
#ifndef _WIN32
	time_t tv_sec;
	long tv_nsec;
#else
	long tv_sec;
	long tv_nsec;
	int init;
	LARGE_INTEGER frequency;
	LARGE_INTEGER ticks;
#endif
};

static enum {
	NO_BENCHMARK,
	TUNER_BENCHMARK,
	PPM_BENCHMARK
} test_mode = NO_BENCHMARK;

static int do_exit = 0;
static rtlsdr_dev_t *dev = NULL;

static uint32_t samp_rate = DEFAULT_SAMPLE_RATE;

static uint32_t total_samples = 0;
static uint32_t dropped_samples = 0;

static unsigned int ppm_duration = PPM_DURATION;

void usage(void)
{
	fprintf(stderr, "rtl_test, a benchmark tool for RTL2832 based DVB-T receivers\n"
		   "Version %d.%d.%d.%d, %s\n",
		   RTLSDR_MAJOR, RTLSDR_MINOR, RTLSDR_MICRO, RTLSDR_NANO, __DATE__);
	fprintf(stderr, "rtlsdr library %d.%d.%d.%d %s\n\n",
		rtlsdr_get_version()>>24, rtlsdr_get_version()>>16 & 0xFF,
		rtlsdr_get_version()>>8 & 0xFF, rtlsdr_get_version() & 0xFF,
		rtlsdr_get_ver_id() );
	fprintf(stderr, "Usage:\t[-b number of buffers (default: 15, set by library)]\n"
		"\t[-d device_index or serial (default: 0)]\n"
		"\t[-l length of single buffer in units of 512 samples (default: 64)]\n"
		"\t[-s samplerate (default: 2048000 Hz)]\n"
		"\t[-t enable Elonics E4000 tuner benchmark]\n"
		"\t[-p[seconds] enable PPM error measurement (default: 10 seconds)]\n"
		"\t[-S force sync output (default: async)]\n");
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	rtlsdr_cancel_async(dev);
}
#endif

static void underrun_test(unsigned char *buf, uint32_t len, int mute)
{
	uint32_t i, lost = 0;
	static uint8_t bcnt, uninit = 1;

	if (uninit)
	{
		bcnt = buf[0];
		uninit = 0;
	}
	for (i = 0; i < len; i++)
	{
		if(bcnt != buf[i])
		{
			if (buf[i] > bcnt)
				lost += (buf[i] - bcnt);
			else
				lost += (bcnt - buf[i]);
			bcnt = buf[i];
		}
		bcnt++;
	}

	total_samples += len;
	dropped_samples += lost;
	if (mute)
		return;
	if (lost)
		fprintf(stderr, "lost at least %d of %d bytes\n", lost, len);

}

#ifdef _WIN32
static int ppm_gettime(struct time_generic *tg)
{
	int rv;
	int64_t frac;
	if (!tg->init) {
		QueryPerformanceFrequency(&tg->frequency);
		tg->init = 1;
	}
	rv = QueryPerformanceCounter(&tg->ticks);
	tg->tv_sec = tg->ticks.QuadPart / tg->frequency.QuadPart;
	frac = (int64_t)(tg->ticks.QuadPart - (tg->tv_sec * tg->frequency.QuadPart));
	tg->tv_nsec = (long)(frac * 1000000000L / (int64_t)tg->frequency.QuadPart);
	return !rv;
}
#else
static int ppm_gettime(struct time_generic *tg)
{
	int rv = ENOSYS;
#ifdef __unix__
	struct timespec ts;

	rv = clock_gettime(CLOCK_MONOTONIC, &ts);
	tg->tv_sec = ts.tv_sec;
	tg->tv_nsec = ts.tv_nsec;
#else
	struct timeval tv;

	rv = gettimeofday(&tv, NULL);
	tg->tv_sec = tv.tv_sec;
	tg->tv_nsec = tv.tv_usec * 1000;
#endif
	return rv;
}
#endif


static int ppm_report(uint64_t nsamples, uint64_t interval)
{
	double real_rate, ppm;

	real_rate = nsamples * 1e9 / interval;
	ppm = 1e6 * (real_rate / (double)samp_rate - 1.);
	return (int)round(ppm);
}

static void ppm_test(uint32_t len)
{
	static uint64_t nsamples = 0;
	static uint64_t interval = 0;
	static uint64_t nsamples_total = 0;
	static uint64_t interval_total = 0;
	static struct time_generic ppm_now;
	static struct time_generic ppm_recent;
	static enum {
		PPM_INIT_NO,
		PPM_INIT_DUMP,
		PPM_INIT_RUN
	} ppm_init = PPM_INIT_NO;

	ppm_gettime(&ppm_now);

	if (ppm_init != PPM_INIT_RUN) {
		/*
		 * Kyle Keen wrote:
		 * PPM_DUMP_TIME throws out the first N seconds of data.
		 * The dongle's PPM is usually very bad when first starting up,
		 * typically incorrect by more than twice the final value.
		 * Discarding the first few seconds allows the value to stabilize much faster.
		*/
		if (ppm_init == PPM_INIT_NO) {
			ppm_recent.tv_sec = ppm_now.tv_sec + PPM_DUMP_TIME;
			ppm_init = PPM_INIT_DUMP;
			return;
		}
		if (ppm_init == PPM_INIT_DUMP && ppm_recent.tv_sec < ppm_now.tv_sec)
			return;
		ppm_recent = ppm_now;
		ppm_init = PPM_INIT_RUN;
		return;
	}

	nsamples += (uint64_t)(len / 2UL);
	interval = (uint64_t)(ppm_now.tv_sec - ppm_recent.tv_sec);
	if (interval < ppm_duration)
		return;
	interval *= 1000000000UL;
	interval += (int64_t)(ppm_now.tv_nsec - ppm_recent.tv_nsec);
	nsamples_total += nsamples;
	interval_total += interval;
	fprintf(stderr, "real sample rate: %i current PPM: %i cumulative PPM: %i\n",
		(int)((1000000000UL * nsamples) / interval),
		ppm_report(nsamples, interval),
		ppm_report(nsamples_total, interval_total));
	ppm_recent = ppm_now;
	nsamples = 0;
}

static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
	underrun_test(buf, len, 0);

	if (test_mode == PPM_BENCHMARK)
		ppm_test(len);
}

void e4k_benchmark(void)
{
	uint32_t freq, gap_start = 0, gap_end = 0;
	uint32_t range_start = 0, range_end = 0;

	fprintf(stderr, "Benchmarking E4000 PLL...\n");

	/* find tuner range start */
	for (freq = MHZ(70); freq > MHZ(1); freq -= MHZ(1)) {
		if (rtlsdr_set_center_freq(dev, freq) < 0) {
			range_start = freq;
			break;
		}
	}

	/* find tuner range end */
	for (freq = MHZ(2000); freq < MHZ(2300UL); freq += MHZ(1)) {
		if (rtlsdr_set_center_freq(dev, freq) < 0) {
			range_end = freq;
			break;
		}
	}

	/* find start of L-band gap */
	for (freq = MHZ(1000); freq < MHZ(1300); freq += MHZ(1)) {
		if (rtlsdr_set_center_freq(dev, freq) < 0) {
			gap_start = freq;
			break;
		}
	}

	/* find end of L-band gap */
	for (freq = MHZ(1300); freq > MHZ(1000); freq -= MHZ(1)) {
		if (rtlsdr_set_center_freq(dev, freq) < 0) {
			gap_end = freq;
			break;
		}
	}

	fprintf(stderr, "E4K range: %i to %i MHz\n",
		range_start/MHZ(1) + 1, range_end/MHZ(1) - 1);

	fprintf(stderr, "E4K L-band gap: %i to %i MHz\n",
		gap_start/MHZ(1), gap_end/MHZ(1));
}

void r82xx_benchmark(void)
{
	uint32_t freq;
	uint32_t range_start = 0, range_end = 0;

	fprintf(stderr, "Benchmarking R82xx PLL...\n");

	/* find tuner range start */
	for (freq = MHZ(70); freq > MHZ(1); freq -= MHZ(1)) {
		if (rtlsdr_set_center_freq(dev, freq) < 0) {
			range_start = freq;
			break;
		}
	}

	/* find tuner range end */
	for (freq = MHZ(1770); freq < MHZ(2000); freq += MHZ(1)) {
		if (rtlsdr_set_center_freq(dev, freq) < 0) {
			range_end = freq;
			break;
		}
	}

	fprintf(stderr, "R8XX range: %i to %i MHz\n",
		range_start/MHZ(1) + 1, range_end/MHZ(1) - 1);

}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	int n_read, r, opt, i;
	int sync_mode = 0;
	uint8_t *buffer;
	uint32_t buf_num = 0;
	int dev_index = 0;
	int dev_given = 0;
	uint32_t buf_len = 64 * 512;
	int count;
	int gains[100];
	int tuner_type;

	while ((opt = getopt(argc, argv, "d:s:b:O:l:tp::Sh")) != -1) {
		switch (opt) {
		case 'b':
			buf_num = atoi(optarg);
			break;
		case 'd':
			dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 's':
			samp_rate = (uint32_t)atofs(optarg);
			break;
		case 'l':
			buf_len = 512 * atoi(optarg);
			break;
		case 't':
			test_mode = TUNER_BENCHMARK;
			break;
		case 'p':
			test_mode = PPM_BENCHMARK;
			if (optarg)
				ppm_duration = atoi(optarg);
			break;
		case 'S':
			sync_mode = 1;
			break;
		case 'h':
		default:
			usage();
			break;
		}
	}

	buffer = malloc(buf_len * sizeof(uint8_t));

	if (!dev_given) {
		dev_index = verbose_device_search("0");
	}

	if (dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dev, (uint32_t)dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}

#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif
	count = rtlsdr_get_tuner_gains(dev, NULL);
	fprintf(stderr, "Supported gain values (%d): ", count);

	count = rtlsdr_get_tuner_gains(dev, gains);
	for (i = 0; i < count; i++)
		fprintf(stderr, "%.1f ", gains[i] / 10.0);
	fprintf(stderr, "\n");

	/* Set the sample rate */
	verbose_set_sample_rate(dev, samp_rate);

	if (test_mode == TUNER_BENCHMARK) {
		tuner_type = rtlsdr_get_tuner_type(dev);
		switch (tuner_type) {
		case RTLSDR_TUNER_E4000:
			e4k_benchmark();
			break;
		case RTLSDR_TUNER_R820T:
		case RTLSDR_TUNER_R828D:
			r82xx_benchmark();
			break;
		default:
			fprintf(stderr, "No supported tuner found\n");
		}
		goto exit;
	}

	/* Enable test mode */
	r = rtlsdr_set_testmode(dev, 1);

	if ((test_mode == PPM_BENCHMARK) && !sync_mode) {
		fprintf(stderr, "Reporting PPM error measurement every %u seconds...\n", ppm_duration);
		fprintf(stderr, "Press ^C after a few minutes.\n");
	}

	if (test_mode == NO_BENCHMARK) {
		fprintf(stderr, "\nInfo: This tool will continuously"
				" read from the device, and report if\n"
				"samples get lost. If you observe no "
				"further output, everything is fine.\n\n");
	}

	/* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dev);

	if (sync_mode) {
		fprintf(stderr, "Reading samples in sync mode...\n");
		fprintf(stderr, "(Samples are being lost but not reported.)\n");
		while (!do_exit) {
			r = rtlsdr_read_sync(dev, buffer, buf_len, &n_read);
			if (r < 0) {
				fprintf(stderr, "WARNING: sync read failed.\n");
				break;
			}

			if ((uint32_t)n_read < buf_len) {
				fprintf(stderr, "Short read, samples lost, exiting!\n");
				break;
			}
			underrun_test(buffer, n_read, 1);
		}
	} else {
		fprintf(stderr, "Reading samples in async mode...\n");
		r = rtlsdr_read_async(dev, rtlsdr_callback, NULL, buf_num, buf_len);
	}

	if (do_exit) {
		fprintf(stderr, "\nUser cancel, exiting...\n");
		fprintf(stderr, "Samples per million lost (minimum): %i\n", (int)(1000000L * dropped_samples / total_samples));
	}
	else
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);

exit:
	rtlsdr_close(dev);
	free (buffer);

	return r >= 0 ? r : -r;
}
