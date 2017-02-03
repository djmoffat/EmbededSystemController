/* Needed to avoid an ALSA bug with C99 */
#define _POSIX_C_SOURCE		200809L

/* Needed to made usleep declaration visible with C99 */
#define _BSD_SOURCE

#include <glob.h>
#include <sched.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <errno.h>
#include <getopt.h>
#include <ctype.h>
#include <alloca.h>
#include <alsa/asoundlib.h>
#include <sys/time.h>
#include "sensor.h"

#include <math.h>
#include <signal.h>
#include <sndfile.h>

#include "audio.h"
#include <signal.h>
// this only if on BBB, which is an ARM platform
#ifdef __arm__
//mem fix
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>


#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", \
  __LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)
#endif

snd_pcm_uframes_t frames;
snd_pcm_uframes_t period_size = 8;						/* period length in frames */
snd_pcm_uframes_t buffer_size = 128;					/* ring buffer length in frames */

static char *device = "plughw:0,0";                     /* playback device */
static snd_pcm_format_t format = SND_PCM_FORMAT_S16;    /* sample format */
static unsigned int rate = 44100;                       /* stream rate */
static unsigned int channels = 2;                       /* count of channels */
//static unsigned int buffer_time = 500000;               /* ring buffer length in us */
static unsigned int period_time = 100000;               /* period time in us */
static int verbose = 0;                                 /* verbose flag */
static int resample = 1;                                /* enable alsa-lib resampling */
static int period_event = 0;                            /* produce poll event after each period */
//static snd_pcm_sframes_t buffer_size;
//static snd_pcm_sframes_t period_size;
static snd_output_t *output = NULL;

/*global audio vars*/
snd_pcm_t *handle;
int err, morehelp;
snd_pcm_hw_params_t *hwparams;
snd_pcm_sw_params_t *swparams;
char *pdevice = "hw:0,0";
char *cdevice = "hw:0,0";
int method = 0;
signed short *samples;
unsigned int chn;
snd_pcm_channel_area_t *areas;
int block = 0;
//int buffer_size = 596;            /* auto */
//int period_size = 8;            /* auto */
int latency_min = 32;           /* in frames / 2 */
int latency_max = 2048;         /* in frames / 2 */
int latency;
int use_poll = 1;


float gAmplitude	= 1.0;
float gFrequency	= 440.0;
float *gSampleBuffer;

int gShouldStop = 0;
int debug = 0;

unsigned long long gFirstSeconds, gFirstMicroseconds;

/* Drum samples are pre-loaded in these buffers. Length of each
 * buffer is given in gDrumSampleBufferLengths.
 */
float *gDrumSampleBuffers[NUMBER_OF_DRUMS];
int gDrumSampleBufferLengths[NUMBER_OF_DRUMS];

/* Patterns indicate which drum(s) should play on which beat.
 * Each element of gPatterns is an array, whose length is given
 * by gPatternLengths.
 */
int *gPatterns[NUMBER_OF_PATTERNS];
int gPatternLengths[NUMBER_OF_PATTERNS];

// Handle Ctrl-C
void interrupt_handler(int var)
{
	gShouldStop = 1;
}

static void generate_sine(const snd_pcm_channel_area_t *areas,
                          snd_pcm_uframes_t offset,
                          int count)
{
	unsigned char *samples[channels];
	int steps[channels];
	unsigned int chn;
	int format_bits = snd_pcm_format_width(format);
	unsigned int maxval = (1 << (format_bits - 1)) - 1;
	int bps = format_bits / 8;  /* bytes per sample */
	int phys_bps = snd_pcm_format_physical_width(format) / 8;
	int big_endian = snd_pcm_format_big_endian(format) == 1;
	int to_unsigned = snd_pcm_format_unsigned(format) == 1;
	int is_float = (format == SND_PCM_FORMAT_FLOAT_LE ||
					format == SND_PCM_FORMAT_FLOAT_BE);

	/* verify and prepare the contents of areas */
	for (chn = 0; chn < channels; chn++) {
			if ((areas[chn].first % 8) != 0) {
					printf("areas[%i].first == %i, aborting...\n", chn, areas[chn].first);
					exit(EXIT_FAILURE);
			}
			samples[chn] = /*(signed short *)*/(((unsigned char *)areas[chn].addr) + (areas[chn].first / 8));
			if ((areas[chn].step % 16) != 0) {
					printf("areas[%i].step == %i, aborting...\n", chn, areas[chn].step);
					exit(EXIT_FAILURE);
			}
			steps[chn] = areas[chn].step / 8;
			samples[chn] += offset * steps[chn];
	}

	/* Call the user render function */
	render((float)rate, channels, count, gSampleBuffer);

	/* fill the channel areas, interleaving the
	 * sample buffers if necessary.
	 */
	for(int n = 0; n < count; n++) {
		for(chn = 0; chn < channels; chn++) {
			union {
					float f;
					int i;
			} fval;

			int res, i;
			if (is_float) {
					fval.f = gSampleBuffer[n*channels + chn];
					res = fval.i;
			} else {
				res = maxval * gSampleBuffer[n*channels + chn];
			}
			if (to_unsigned)
					res ^= 1U << (format_bits - 1);
			/* Generate data in native endian format */
			if (big_endian) {
					for (i = 0; i < bps; i++)
							*(samples[chn] + phys_bps - 1 - i) = (res >> i * 8) & 0xff;
			} else {
					for (i = 0; i < bps; i++)
							*(samples[chn] + i) = (res >>  i * 8) & 0xff;
			}
			samples[chn] += steps[chn];
		}
	}

}

static int set_hwparams(snd_pcm_t *handle,
                        snd_pcm_hw_params_t *params,
                        snd_pcm_access_t access)
{
        unsigned int rrate;
        snd_pcm_uframes_t size;
        int err, dir;
        /* choose all parameters */
        err = snd_pcm_hw_params_any(handle, params);
        if (err < 0) {
                printf("Broken configuration for playback: no configurations available: %s\n", snd_strerror(err));
                return err;
        }
        /* set hardware resampling */
        err = snd_pcm_hw_params_set_rate_resample(handle, params, resample);
        if (err < 0) {
                printf("Resampling setup failed for playback: %s\n", snd_strerror(err));
                return err;
        }
        /* set the interleaved read/write format */
        err = snd_pcm_hw_params_set_access(handle, params, access);
        if (err < 0) {
                printf("Access type not available for playback: %s\n", snd_strerror(err));
                return err;
        }
        /* set the sample format */
        err = snd_pcm_hw_params_set_format(handle, params, format);
        if (err < 0) {
                printf("Sample format not available for playback: %s\n", snd_strerror(err));
                return err;
        }
        /* set the count of channels */
        err = snd_pcm_hw_params_set_channels(handle, params, channels);
        if (err < 0) {
                printf("Channels count (%i) not available for playbacks: %s\n", channels, snd_strerror(err));
                return err;
        }
        /* set the stream rate */
        rrate = rate;
        err = snd_pcm_hw_params_set_rate_near(handle, params, &rrate, 0);
        if (err < 0) {
                printf("Rate %iHz not available for playback: %s\n", rate, snd_strerror(err));
                return err;
        }
        if (rrate != rate) {
                printf("Rate doesn't match (requested %iHz, get %iHz)\n", rate, err);
                return -EINVAL;
        }



        snd_pcm_hw_params_get_period_size_min(params, &frames, &dir);
  		printf("\nMin Period size: %d frames\n", (int)frames);
  		snd_pcm_hw_params_get_period_size_max(params, &frames, &dir);
  		printf("Max Period size: %d frames\n", (int)frames);

  		snd_pcm_hw_params_get_buffer_size_min(params, &frames);
  		printf("Min Buffer size: %d frames\n", (int)frames);
  		snd_pcm_hw_params_get_buffer_size_max(params, &frames);
  		printf("Max Buffer size: %d frames\n", (int)frames);



        /* set the period size */
        err = snd_pcm_hw_params_set_period_size(handle, params, period_size, dir);
        if (err < 0) {
                printf("Unable to set period size %i for playback: %s\n", (int)period_size, snd_strerror(err));
                return err;
        }
        // check it
        err = snd_pcm_hw_params_get_period_size(params, &size, &dir);
        if (err < 0) {
                printf("Unable to get period size for playback: %s\n", snd_strerror(err));
                return err;
        }
        period_size = size;
        printf("----------period size: %d frames\n", (int)period_size);

        /* set the buffer size */
        err = snd_pcm_hw_params_set_buffer_size	(handle, params, buffer_size);
        if (err < 0) {
                printf("Unable to set buffer size %i for playback: %s\n", (int)buffer_size, snd_strerror(err));
                return err;
        }
        // check it
        err = snd_pcm_hw_params_get_buffer_size(params, &size);
        if (err < 0) {
                printf("Unable to get buffer size for playback: %s\n", snd_strerror(err));
                return err;
        }
        buffer_size = size;
        printf("----------buffer size: %d frames\n", (int)buffer_size);


      /* We want to loop for n seconds */
	  snd_pcm_hw_params_get_period_time(params, &period_time, &dir);

	  /* write the parameters to device */
	  err = snd_pcm_hw_params(handle, params);
	  if (err < 0) {
		  printf("Unable to set hw params for playback: %s\n", snd_strerror(err));
		  return err;
	  }
	  return 0;
}

static int set_swparams(snd_pcm_t *handle, snd_pcm_sw_params_t *swparams)
{
        int err;
        /* get the current swparams */
        err = snd_pcm_sw_params_current(handle, swparams);
        if (err < 0) {
                printf("Unable to determine current swparams for playback: %s\n", snd_strerror(err));
                return err;
        }
        /* start the transfer when the buffer is almost full: */
        /* (buffer_size / avail_min) * avail_min */
        err = snd_pcm_sw_params_set_start_threshold(handle, swparams, (buffer_size / period_size) * period_size);
        if (err < 0) {
                printf("Unable to set start threshold mode for playback: %s\n", snd_strerror(err));
                return err;
        }
        /* allow the transfer when at least period_size samples can be processed */
        /* or disable this mechanism when period event is enabled (aka interrupt like style processing) */
        err = snd_pcm_sw_params_set_avail_min(handle, swparams, period_event ? buffer_size : period_size);
        if (err < 0) {
                printf("Unable to set avail min for playback: %s\n", snd_strerror(err));
                return err;
        }
        /* enable period events when requested */
        if (period_event) {
                err = snd_pcm_sw_params_set_period_event(handle, swparams, 1);
                if (err < 0) {
                        printf("Unable to set period event: %s\n", snd_strerror(err));
                        return err;
                }
        }
        /* write the parameters to the playback device */
        err = snd_pcm_sw_params(handle, swparams);
        if (err < 0) {
                printf("Unable to set sw params for playback: %s\n", snd_strerror(err));
                return err;
        }
        return 0;
}


// Read from the audio capture device
long readbuf(snd_pcm_t *handle, char *buf, long len, size_t *frames)
{
        long r;
        if (!block) {
                do {
                        r = snd_pcm_readi(handle, buf, len);
                } while (r == -EAGAIN);
                if (r > 0) {
                        *frames += r;
                }
                // printf("read = %li\n", r);
        } else {
                int frame_bytes = (snd_pcm_format_width(format) / 8) * channels;
                do {
                        r = snd_pcm_readi(handle, buf, len);
                        if (r > 0) {
                                buf += r * frame_bytes;
                                len -= r;
                                *frames += r;
                        }
                        // printf("r = %li, len = %li\n", r, len);
                } while (r >= 1 && len > 0);
        }
        // showstat(handle, 0);
        return r;
}

/*
 *   Underrun and suspend recovery
 */
static int xrun_recovery(snd_pcm_t *handle, int err)
{
        if (verbose)
                printf("stream recovery\n");
        if (err == -EPIPE) {    /* under-run */
                err = snd_pcm_prepare(handle);
                if (err < 0)
                        printf("Can't recovery from underrun, prepare failed: %s\n", snd_strerror(err));
                return 0;
        } else if (err == -ESTRPIPE) {
                while ((err = snd_pcm_resume(handle)) == -EAGAIN)
                        sleep(1);       /* wait until the suspend flag is released */
                if (err < 0) {
                        err = snd_pcm_prepare(handle);
                        if (err < 0)
                                printf("Can't recovery from suspend, prepare failed: %s\n", snd_strerror(err));
                }
                return 0;
        }
        return err;
}

/*
 *   Transfer method - write only
 */
static int write_loop(snd_pcm_t *handle,
                      signed short *samples,
                      snd_pcm_channel_area_t *areas)
{
		//getchar();
		//printf("___________________write loop\n");
        //double phase = 0;
        signed short *ptr;
        int err, cptr;
        while (!gShouldStop)
        {
                generate_sine(areas, 0, period_size);
                ptr = samples;
                cptr = period_size;
                while (cptr > 0) {
                        err = snd_pcm_writei(handle, ptr, cptr);
                        if (err == -EAGAIN)
                                continue;
                        if (err < 0) {
                                if (xrun_recovery(handle, err) < 0) {
                                        printf("Write error: %s\n", snd_strerror(err));
                                        exit(EXIT_FAILURE);
                                }
                                break;  /* skip one period */
                        }
                        ptr += err * channels;
                        cptr -= err;
                }
        }

        return 0;
}

/*
 *   Transfer method - write and wait for room in buffer using poll
 */
static int wait_for_poll(snd_pcm_t *handle, struct pollfd *ufds, unsigned int count)
{
        unsigned short revents;
        while (1) {
                poll(ufds, count, -1);
                snd_pcm_poll_descriptors_revents(handle, ufds, count, &revents);
                if (revents & POLLERR)
                        return -EIO;
                if (revents & POLLOUT)
                        return 0;
        }

        return 0;
}

static int write_and_poll_loop(snd_pcm_t *handle,
                               signed short *samples,
                               snd_pcm_channel_area_t *areas)
{
        struct pollfd *ufds;
        //double phase = 0;
        signed short *ptr;
        int err, count, cptr, init;
        count = snd_pcm_poll_descriptors_count (handle);
        if (count <= 0) {
                printf("Invalid poll descriptors count\n");
                return count;
        }
        ufds = malloc(sizeof(struct pollfd) * count);
        if (ufds == NULL) {
                printf("No enough memory\n");
                return -ENOMEM;
        }
        if ((err = snd_pcm_poll_descriptors(handle, ufds, count)) < 0) {
                printf("Unable to obtain poll descriptors for playback: %s\n", snd_strerror(err));
                return err;
        }
        init = 1;
        while (1) {
                if (!init) {
                        err = wait_for_poll(handle, ufds, count);
                        if (err < 0) {
                                if (snd_pcm_state(handle) == SND_PCM_STATE_XRUN ||
                                    snd_pcm_state(handle) == SND_PCM_STATE_SUSPENDED) {
                                        err = snd_pcm_state(handle) == SND_PCM_STATE_XRUN ? -EPIPE : -ESTRPIPE;
                                        if (xrun_recovery(handle, err) < 0) {
                                                printf("Write error: %s\n", snd_strerror(err));
                                                exit(EXIT_FAILURE);
                                        }
                                        init = 1;
                                } else {
                                        printf("Wait for poll failed\n");
                                        return err;
                                }
                        }
                }
                generate_sine(areas, 0, period_size);
                ptr = samples;
                cptr = period_size;
                while (cptr > 0) {
                        err = snd_pcm_writei(handle, ptr, cptr);
                        if (err < 0) {
                                if (xrun_recovery(handle, err) < 0) {
                                        printf("Write error: %s\n", snd_strerror(err));
                                        exit(EXIT_FAILURE);
                                }
                                init = 1;
                                break;  /* skip one period */
                        }
                        if (snd_pcm_state(handle) == SND_PCM_STATE_RUNNING)
                                init = 0;
                        ptr += err * channels;
                        cptr -= err;
                        if (cptr == 0)
                                break;
                        /* it is possible, that the initial buffer cannot store */
                        /* all data from the last period, so wait awhile */
                        err = wait_for_poll(handle, ufds, count);
                        if (err < 0) {
                                if (snd_pcm_state(handle) == SND_PCM_STATE_XRUN ||
                                    snd_pcm_state(handle) == SND_PCM_STATE_SUSPENDED) {
                                        err = snd_pcm_state(handle) == SND_PCM_STATE_XRUN ? -EPIPE : -ESTRPIPE;
                                        if (xrun_recovery(handle, err) < 0) {
                                                printf("Write error: %s\n", snd_strerror(err));
                                                exit(EXIT_FAILURE);
                                        }
                                        init = 1;
                                } else {
                                        printf("Wait for poll failed\n");
                                        return err;
                                }
                        }
                }
        }

        return 0;
}

/*
 *   Transfer method - asynchronous notification
 */
struct async_private_data {
        signed short *samples;
        snd_pcm_channel_area_t *areas;
        double phase;
};

static void async_callback(snd_async_handler_t *ahandler)
{
        snd_pcm_t *handle = snd_async_handler_get_pcm(ahandler);
        struct async_private_data *data = snd_async_handler_get_callback_private(ahandler);
        signed short *samples = data->samples;
        snd_pcm_channel_area_t *areas = data->areas;
        snd_pcm_sframes_t avail;
        int err;

        avail = snd_pcm_avail_update(handle);
        while (avail >= period_size) {
                generate_sine(areas, 0, period_size);
                err = snd_pcm_writei(handle, samples, period_size);
                if (err < 0) {
                        printf("Write error: %s\n", snd_strerror(err));
                        exit(EXIT_FAILURE);
                }
                if (err != period_size) {
                        printf("Write error: written %i expected %li\n", err, period_size);
                        exit(EXIT_FAILURE);
                }
                avail = snd_pcm_avail_update(handle);
        }
}

static int async_loop(snd_pcm_t *handle,
                      signed short *samples,
                      snd_pcm_channel_area_t *areas)
{
        struct async_private_data data;
        snd_async_handler_t *ahandler;
        int err, count;
        data.samples = samples;
        data.areas = areas;
        data.phase = 0;
        err = snd_async_add_pcm_handler(&ahandler, handle, async_callback, &data);
        if (err < 0) {
                printf("Unable to register async handler\n");
                exit(EXIT_FAILURE);
        }
        for (count = 0; count < 2; count++) {
                generate_sine(areas, 0, period_size);
                err = snd_pcm_writei(handle, samples, period_size);
                if (err < 0) {
                        printf("Initial write error: %s\n", snd_strerror(err));
                        exit(EXIT_FAILURE);
                }
                if (err != period_size) {
                        printf("Initial write error: written %i expected %li\n", err, period_size);
                        exit(EXIT_FAILURE);
                }
        }
        if (snd_pcm_state(handle) == SND_PCM_STATE_PREPARED) {
                err = snd_pcm_start(handle);
                if (err < 0) {
                        printf("Start error: %s\n", snd_strerror(err));
                        exit(EXIT_FAILURE);
                }
        }
        /* because all other work is done in the signal handler,
           suspend the process */
        while (1) {
                sleep(1);
        }

        return 0;
}

/*
 *   Transfer method - asynchronous notification + direct write
 */
static void async_direct_callback(snd_async_handler_t *ahandler)
{
        snd_pcm_t *handle = snd_async_handler_get_pcm(ahandler);
        //struct async_private_data *data = snd_async_handler_get_callback_private(ahandler);
        const snd_pcm_channel_area_t *my_areas;
        snd_pcm_uframes_t offset, frames, size;
        snd_pcm_sframes_t avail, commitres;
        snd_pcm_state_t state;
        int first = 0, err;

        while (1) {
                state = snd_pcm_state(handle);
                if (state == SND_PCM_STATE_XRUN) {
                        err = xrun_recovery(handle, -EPIPE);
                        if (err < 0) {
                                printf("XRUN recovery failed: %s\n", snd_strerror(err));
                                exit(EXIT_FAILURE);
                        }
                        first = 1;
                } else if (state == SND_PCM_STATE_SUSPENDED) {
                        err = xrun_recovery(handle, -ESTRPIPE);
                        if (err < 0) {
                                printf("SUSPEND recovery failed: %s\n", snd_strerror(err));
                                exit(EXIT_FAILURE);
                        }
                }
                avail = snd_pcm_avail_update(handle);
                if (avail < 0) {
                        err = xrun_recovery(handle, avail);
                        if (err < 0) {
                                printf("avail update failed: %s\n", snd_strerror(err));
                                exit(EXIT_FAILURE);
                        }
                        first = 1;
                        continue;
                }
                if (avail < period_size) {
                        if (first) {
                                first = 0;
                                err = snd_pcm_start(handle);
                                if (err < 0) {
                                        printf("Start error: %s\n", snd_strerror(err));
                                        exit(EXIT_FAILURE);
                                }
                        } else {
                                break;
                        }
                        continue;
                }
                size = period_size;
                while (size > 0) {
                        frames = size;
                        err = snd_pcm_mmap_begin(handle, &my_areas, &offset, &frames);
                        if (err < 0) {
                                if ((err = xrun_recovery(handle, err)) < 0) {
                                        printf("MMAP begin avail error: %s\n", snd_strerror(err));
                                        exit(EXIT_FAILURE);
                                }
                                first = 1;
                        }
                        generate_sine(my_areas, offset, frames);
                        commitres = snd_pcm_mmap_commit(handle, offset, frames);
                        if (commitres < 0 || (snd_pcm_uframes_t)commitres != frames) {
                                if ((err = xrun_recovery(handle, commitres >= 0 ? -EPIPE : commitres)) < 0) {
                                        printf("MMAP commit error: %s\n", snd_strerror(err));
                                        exit(EXIT_FAILURE);
                                }
                                first = 1;
                        }
                        size -= frames;
                }
        }
}

static int async_direct_loop(snd_pcm_t *handle,
                             signed short *samples ATTRIBUTE_UNUSED,
                             snd_pcm_channel_area_t *areas ATTRIBUTE_UNUSED)
{
        struct async_private_data data;
        snd_async_handler_t *ahandler;
        const snd_pcm_channel_area_t *my_areas;
        snd_pcm_uframes_t offset, frames, size;
        snd_pcm_sframes_t commitres;
        int err, count;
        data.samples = NULL;    /* we do not require the global sample area for direct write */
        data.areas = NULL;      /* we do not require the global areas for direct write */
        data.phase = 0;
        err = snd_async_add_pcm_handler(&ahandler, handle, async_direct_callback, &data);
        if (err < 0) {
                printf("Unable to register async handler\n");
                exit(EXIT_FAILURE);
        }
        for (count = 0; count < 2; count++) {
                size = period_size;
                while (size > 0) {
                        frames = size;
                        err = snd_pcm_mmap_begin(handle, &my_areas, &offset, &frames);
                        if (err < 0) {
                                if ((err = xrun_recovery(handle, err)) < 0) {
                                        printf("MMAP begin avail error: %s\n", snd_strerror(err));
                                        exit(EXIT_FAILURE);
                                }
                        }
                        generate_sine(my_areas, offset, frames);
                        commitres = snd_pcm_mmap_commit(handle, offset, frames);
                        if (commitres < 0 || (snd_pcm_uframes_t)commitres != frames) {
                                if ((err = xrun_recovery(handle, commitres >= 0 ? -EPIPE : commitres)) < 0) {
                                        printf("MMAP commit error: %s\n", snd_strerror(err));
                                        exit(EXIT_FAILURE);
                                }
                        }
                        size -= frames;
                }
        }
        err = snd_pcm_start(handle);
        if (err < 0) {
                printf("Start error: %s\n", snd_strerror(err));
                exit(EXIT_FAILURE);
        }
        /* because all other work is done in the signal handler,
           suspend the process */
        while (1) {
                sleep(1);
        }

        return 0;
}

/*
 *   Transfer method - direct write only
 */
static int direct_loop(snd_pcm_t *handle,
                       signed short *samples ATTRIBUTE_UNUSED,
                       snd_pcm_channel_area_t *areas ATTRIBUTE_UNUSED)
{
        //double phase = 0;
        const snd_pcm_channel_area_t *my_areas;
        snd_pcm_uframes_t offset, frames, size;
        snd_pcm_sframes_t avail, commitres;
        snd_pcm_state_t state;
        int err, first = 1;
        while (1) {
                state = snd_pcm_state(handle);
                if (state == SND_PCM_STATE_XRUN) {
                        err = xrun_recovery(handle, -EPIPE);
                        if (err < 0) {
                                printf("XRUN recovery failed: %s\n", snd_strerror(err));
                                return err;
                        }
                        first = 1;
                } else if (state == SND_PCM_STATE_SUSPENDED) {
                        err = xrun_recovery(handle, -ESTRPIPE);
                        if (err < 0) {
                                printf("SUSPEND recovery failed: %s\n", snd_strerror(err));
                                return err;
                        }
                }
                avail = snd_pcm_avail_update(handle);
                if (avail < 0) {
                        err = xrun_recovery(handle, avail);
                        if (err < 0) {
                                printf("avail update failed: %s\n", snd_strerror(err));
                                return err;
                        }
                        first = 1;
                        continue;
                }
                if (avail < period_size) {
                        if (first) {
                                first = 0;
                                err = snd_pcm_start(handle);
                                if (err < 0) {
                                        printf("Start error: %s\n", snd_strerror(err));
                                        exit(EXIT_FAILURE);
                                }
                        } else {
                                err = snd_pcm_wait(handle, -1);
                                if (err < 0) {
                                        if ((err = xrun_recovery(handle, err)) < 0) {
                                                printf("snd_pcm_wait error: %s\n", snd_strerror(err));
                                                exit(EXIT_FAILURE);
                                        }
                                        first = 1;
                                }
                        }
                        continue;
                }
                size = period_size;
                while (size > 0) {
                        frames = size;
                        err = snd_pcm_mmap_begin(handle, &my_areas, &offset, &frames);
                        if (err < 0) {
                                if ((err = xrun_recovery(handle, err)) < 0) {
                                        printf("MMAP begin avail error: %s\n", snd_strerror(err));
                                        exit(EXIT_FAILURE);
                                }
                                first = 1;
                        }
                        generate_sine(my_areas, offset, frames);
                        commitres = snd_pcm_mmap_commit(handle, offset, frames);
                        if (commitres < 0 || (snd_pcm_uframes_t)commitres != frames) {
                                if ((err = xrun_recovery(handle, commitres >= 0 ? -EPIPE : commitres)) < 0) {
                                        printf("MMAP commit error: %s\n", snd_strerror(err));
                                        exit(EXIT_FAILURE);
                                }
                                first = 1;
                        }
                        size -= frames;
                }
        }

        return 0;
}

/*
 *   Transfer method - direct write only using mmap_write functions
 */
static int direct_write_loop(snd_pcm_t *handle,
                             signed short *samples,
                             snd_pcm_channel_area_t *areas)
{
        //double phase = 0;
        signed short *ptr;
        int err, cptr;
        while (1) {
                generate_sine(areas, 0, period_size);
                ptr = samples;
                cptr = period_size;
                while (cptr > 0) {
                        err = snd_pcm_mmap_writei(handle, ptr, cptr);
                        if (err == -EAGAIN)
                                continue;
                        if (err < 0) {
                                if (xrun_recovery(handle, err) < 0) {
                                        printf("Write error: %s\n", snd_strerror(err));
                                        exit(EXIT_FAILURE);
                                }
                                break;  /* skip one period */
                        }
                        ptr += err * channels;
                        cptr -= err;
                }
        }

        return 0;
}

/*
 *
 */
struct transfer_method {
        const char *name;
        snd_pcm_access_t access;
        int (*transfer_loop)(snd_pcm_t *handle,
                             signed short *samples,
                             snd_pcm_channel_area_t *areas);
};

static struct transfer_method transfer_methods[] =
{
        { "write", 					SND_PCM_ACCESS_RW_INTERLEAVED, 		write_loop },
        { "write_and_poll", 		SND_PCM_ACCESS_RW_INTERLEAVED, 		write_and_poll_loop },
        { "async", 					SND_PCM_ACCESS_RW_INTERLEAVED, 		async_loop },
        { "async_direct", 			SND_PCM_ACCESS_MMAP_INTERLEAVED, 	async_direct_loop },
        { "direct_interleaved", 	SND_PCM_ACCESS_MMAP_INTERLEAVED, 	direct_loop },
        { "direct_noninterleaved", 	SND_PCM_ACCESS_MMAP_NONINTERLEAVED, direct_loop },
        { "direct_write", 			SND_PCM_ACCESS_MMAP_INTERLEAVED, 	direct_write_loop },
        { NULL, 					SND_PCM_ACCESS_RW_INTERLEAVED, 		NULL }
};



//-----------------------------------------------------------------------------------------------------------
// Audio cape memory fix, only if on BBB, which is an ARM platform
//-----------------------------------------------------------------------------------------------------------
#ifdef __arm__
void pokeMem(const char *argv1, const char *argv2, const char *argv3)
{
	int fd;
    void *map_base, *virt_addr;
	unsigned long read_result, writeval;
	off_t target;
	int access_type = 'w';

	int _verbose = 0;

	target = strtoul(argv1, 0, 0);

	access_type = tolower(argv2[0]);

    if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;
    if(_verbose)
        	printf("/dev/mem opened.\n");
    fflush(stdout);

    /* Map one page */
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;
    if(_verbose)
        	printf("Memory mapped at address %p.\n", map_base);
    fflush(stdout);

    virt_addr = map_base + (target & MAP_MASK);
    switch(access_type) {
		case 'b':
			read_result = *((unsigned char *) virt_addr);
			break;
		case 'h':
			read_result = *((unsigned short *) virt_addr);
			break;
		case 'w':
			read_result = *((unsigned long *) virt_addr);
			break;
		default:
			fprintf(stderr, "Illegal data type '%c'.\n", access_type);
			exit(2);
	}
    if(_verbose)
    	printf("Value at address 0x%X (%p): 0x%X\n", (unsigned int) target, virt_addr, (unsigned int) read_result);
    fflush(stdout);


	writeval = strtoul(argv3, 0, 0);
	switch(access_type) {
		case 'b':
			*((unsigned char *) virt_addr) = writeval;
			read_result = *((unsigned char *) virt_addr);
			break;
		case 'h':
			*((unsigned short *) virt_addr) = writeval;
			read_result = *((unsigned short *) virt_addr);
			break;
		case 'w':
			*((unsigned long *) virt_addr) = writeval;
			read_result = *((unsigned long *) virt_addr);
			break;
	}
    if(_verbose)
    	printf("Written 0x%X; readback 0x%X\n", (unsigned int) writeval, (unsigned int) read_result);
	fflush(stdout);


	if(munmap(map_base, MAP_SIZE) == -1) FATAL;
    close(fd);
    return;
}
#endif
//-----------------------------------------------------------------------------------------------------------

/* This function handles initialising the analog inputs */
int analogInit() {
	FILE *ActivateAnalogHnd   = NULL;
	char *activateAnalogPath;
	int analogIsSet          = 0;

	// suport var for init
	// cape-bone-iio > /sys/devices/bone_capemgr.*/slots
	char startPath[60]    = "";

	glob_t  globbuf;

	// first: activate analog pins on cape manager
	// cape-bone-iio > /sys/devices/bone_capemgr.*/slots

	// we have to look for the semi-random number the BBB has initialized the bone_capemgr with “‘[value of *]
	// to reach /slots and set cape-bone-iio
	// to do so, we try many values, checking if file exists

	// these are fixed

	strcpy(startPath, "/sys/devices/bone_capemgr.*/slots");

	glob( startPath, 0, NULL, &globbuf);

	if(globbuf.gl_pathc >0)
	{
	if (globbuf.gl_pathc == 1 )
	{
	  activateAnalogPath = globbuf.gl_pathv[0];

	  // check if file is existing
	  if((ActivateAnalogHnd = fopen(activateAnalogPath, "r+")) != NULL)
	  {
		// we found that current capemgr num

		fwrite("cape-bone-iio", sizeof(char), 13, ActivateAnalogHnd); // activate pins

		analogIsSet = 1;

		printf("Analog Pins activated via cape-bone-iio at path\n");

		fclose(ActivateAnalogHnd); // close file
	  }
	}
	//else
	  //printf("toomany", );
	}

	globfree(&globbuf);

	if(analogIsSet == 0)
	{
		printf("cannot find bone_capemgr\n");
		return 1;
	}

	return 0;
}

/* This function reads an analog value from the given input.
 * You can use it in your code for the potentiometer and accelerometer. */
int analogRead(int input) {
	FILE *AnalogInHnd   = NULL;
	//char analogInPath[60] = "";
	//int helperNumFound = 0;

	// suport var for init
	// cape-bone-iio > /sys/devices/bone_capemgr.*/slots
	char startPath[60]    = "";
	char testReadPath[60] = "";

	glob_t  globbuf;

	// support vars for pin reading
	long lSize, newlSize;
	char * buffer;

	int result = 0;

	// second: read from pins
	// /sys/devices/ocp.2/helper.X/AINY

	// we have to look for the semi-random number the BBB has initialized the helper with “‘[value of X]
	// this is a dir which contains all the analog pins
	// so, we try to read from a first time from pin 0, trying many numbers until we find an existing file

	// these are fixed
	strcpy(startPath, "/sys/devices/ocp.2/helper.*");

	glob( startPath, 0, NULL, &globbuf);

	if(globbuf.gl_pathc >0)
	{
		if (globbuf.gl_pathc == 1 )
		{
			snprintf(testReadPath, 60, "%s/AIN%d", globbuf.gl_pathv[0], input);

			//strcpy(analogInPath, globbuf.gl_pathv[0]);
			//strcat(analogInPath, "/AIN");

			//printf("%s\n", analogInPath);

			//strcpy(testReadPath, analogInPath);
			//strcat(testReadPath, gAnalogIn);	// create pin file path

			lSize = 4096;	// bit resolution
			buffer = (char*) malloc (sizeof(char)*lSize); // set correct buff dim

			// check if file is existing
			if((AnalogInHnd = fopen(testReadPath, "rb")) != NULL)
			{
				// we found that current helper num
				// helperNumFound = 1;
				//printf("Analog Pins can be read at path %s\n", testReadPath);

				// prepare read buffer to test reading
				fseek (AnalogInHnd , 0 , SEEK_END);
				newlSize = ftell (AnalogInHnd);
				rewind (AnalogInHnd);

				// if resolution is different
				if(lSize < newlSize)
				{
					buffer = (char*) realloc(buffer, (sizeof(char)*newlSize) ); // reset correct buff dim
					lSize = newlSize;
				}

				if( fread (buffer, 1, lSize, AnalogInHnd)>0 )
					result = atoi(buffer);

				//printf("Test reading of Pin %s gives: %i\n", gAnalogIn, atoi(buffer));

				fclose(AnalogInHnd); // close file
			}

			if(buffer != NULL)
				free (buffer);
		}
		else
			printf("too many helpers\n");
	}
	else
		printf("helper not found\n");

	globfree(&globbuf);

	return result;
}

/* This function cleans up the analog inputs */
void analogCleanup() {

}

//-----------------------------------------------------------------------------------------------------------

void *audioGenerator(void *data)
{
    snd_pcm_hw_params_alloca(&hwparams);
    snd_pcm_sw_params_alloca(&swparams);

    err = snd_output_stdio_attach(&output, stdout, 0);
    if (err < 0) {
            printf("Output failed: %s\n", snd_strerror(err));
            exit(EXIT_FAILURE);
    }
    printf("Playback device is %s\n", device);
    printf("Stream parameters are %iHz, %s, %i channels\n", rate, snd_pcm_format_name(format), channels);
    //printf("Sine wave rate is %.4fHz and %.4fHz\n", gFrequency[0], gFrequency[1]);
    printf("Using transfer method: %s\n", transfer_methods[method].name);
    if ((err = snd_pcm_open(&handle, device, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
            printf("Playback open error: %s\n", snd_strerror(err));
            exit(EXIT_FAILURE);
    }

    if ((err = set_hwparams(handle, hwparams, transfer_methods[method].access)) < 0) {
            printf("Setting of hwparams failed: %s\n", snd_strerror(err));
            exit(EXIT_FAILURE);
    }
    if ((err = set_swparams(handle, swparams)) < 0) {
            printf("Setting of swparams failed: %s\n", snd_strerror(err));
            exit(EXIT_FAILURE);
    }
    if (verbose > 0)
            snd_pcm_dump(handle, output);
    samples = malloc((period_size * channels * snd_pcm_format_physical_width(format)) / 8);
    if (samples == NULL) {
            printf("No enough memory\n");
            exit(EXIT_FAILURE);
    }

    gSampleBuffer = malloc(period_size * channels * sizeof(float));
    if(gSampleBuffer == NULL) {
    	printf("Not enough memory\n");
    	exit(EXIT_FAILURE);
    }
    memset(gSampleBuffer, 0, period_size * channels * sizeof(float));

    areas = calloc(channels, sizeof(snd_pcm_channel_area_t));
    if (areas == NULL) {
            printf("No enough memory\n");
            exit(EXIT_FAILURE);
    }
    for (chn = 0; chn < channels; chn++) {
            areas[chn].addr = samples;
            areas[chn].first = chn * snd_pcm_format_physical_width(format);
            areas[chn].step = channels * snd_pcm_format_physical_width(format);
    }


	// this only if on BBB, which is an ARM platform
	#ifdef __arm__
	if(strcmp(device, "plughw:0,0")==0)
		pokeMem("0x480380ac", "w", "0x100");
	#endif

	initialise((float)rate, channels);

    err = transfer_methods[method].transfer_loop(handle, samples, areas);
    if (err < 0)
            printf("Transfer failed: %s\n", snd_strerror(err));
    free(areas);
    free(samples);
    free(gSampleBuffer);
    snd_pcm_close(handle);

    cleanup();
	pthread_exit(NULL);
}

void audioListener(void *data){


	// Set latency
    snd_pcm_t *phandle, *chandle;
    size_t frames_in, frames_out;
    int ok;
    ssize_t r;

	        latency = latency_min - 4;

	        // Allocate buffers
	        char* buffer = malloc((latency_max * snd_pcm_format_width(format) / 8) * 2);
	        float* floatBuffer = malloc(latency_max * sizeof(float) * 2);

	        // Set the priority of the audio thread and use round-robin scheduler
	        setscheduler();

	        // Print stats
	        printf("Playback device is %s\n", pdevice);
	        printf("Capture device is %s\n", cdevice);
	        printf("Parameters are %iHz, %s, %i channels, %s mode\n", rate, snd_pcm_format_name(format), channels, block ? "blocking" : "non-blocking");
	        printf("Poll mode: %s\n", use_poll ? "yes" : "no");
	  // Open capture and playback devices
	        if ((err = snd_pcm_open(&phandle, pdevice, SND_PCM_STREAM_PLAYBACK, block ? 0 : SND_PCM_NONBLOCK)) < 0) {
	                printf("Playback open error: %s\n", snd_strerror(err));
//	                return 0;
	        }
	        if ((err = snd_pcm_open(&chandle, cdevice, SND_PCM_STREAM_CAPTURE, block ? 0 : SND_PCM_NONBLOCK)) < 0) {
	                printf("Record open error: %s\n", snd_strerror(err));
//	                return 0;
	        }

	        // Call the user initialisation code
	        initialise((float)rate, channels);

	        // Set up interrupt handler to catch Control-C
	        signal(SIGINT, interrupt_handler);

	        while (!gShouldStop) {
	                frames_in = frames_out = 0;
	                if (setparams(phandle, chandle, &latency) < 0)
	                        break;
	                if ((err = snd_pcm_link(chandle, phandle)) < 0) {
	                        printf("Streams link error: %s\n", snd_strerror(err));
	                        exit(0);
	                }
	                if (snd_pcm_format_set_silence(format, buffer, latency*channels) < 0) {
	                        fprintf(stderr, "silence error\n");
	                        break;
	                }
	                if (writebuf(phandle, buffer, latency, &frames_out) < 0) {
	                        fprintf(stderr, "write error\n");
	                        break;
	                }
	                if (writebuf(phandle, buffer, latency, &frames_out) < 0) {
	                        fprintf(stderr, "write error\n");
	                        break;
	                }
	                if ((err = snd_pcm_start(chandle)) < 0) {
	                        printf("Go error: %s\n", snd_strerror(err));
	                        exit(0);
	                }

	                ok = 1;
	                while (ok && !gShouldStop) {
						if (use_poll) {
								/* use poll to wait for next event */
								snd_pcm_wait(chandle, 1000);
						}
						if ((r = readbuf(chandle, buffer, latency, &frames_in)) < 0)
							ok = 0;
						else {
							// Convert short (16-bit) samples to float
							short *shortBuffer = (short *)buffer;
							int n;

							for(n = 0; n < r * channels; n++)
								floatBuffer[n] = (float)shortBuffer[n] / 32768.0;

							render((float)rate, channels, r, floatBuffer);

							// Convert float back to short
							for(n = 0; n < r * channels; n++)
								shortBuffer[n] = (short)(floatBuffer[n] * 32768.0);

//							if (writebuf(phandle, buffer, r, &frames_out) < 0)
//									ok = 0;
						}
	                }

	                snd_pcm_drop(chandle);
	                snd_pcm_nonblock(phandle, 0);
	                snd_pcm_drain(phandle);
	                snd_pcm_nonblock(phandle, !block ? 1 : 0);

	                if (ok) {
						break;
	                }
	                printf("Underrun; trying again\n");
	                snd_pcm_unlink(chandle);
	                snd_pcm_hw_free(phandle);
	                snd_pcm_hw_free(chandle);
	        }

	        printf("Cleaning up...\n");

	        // Clean up and deallocate
	        cleanup();
	        snd_pcm_close(phandle);
	        snd_pcm_close(chandle);
	        free(floatBuffer);
	        free(buffer);

}

int setparams_bufsize(snd_pcm_t *handle,
                      snd_pcm_hw_params_t *params,
                      snd_pcm_hw_params_t *tparams,
                      snd_pcm_uframes_t bufsize,
                      const char *id)
{
        int err;
        snd_pcm_uframes_t periodsize;
        snd_pcm_hw_params_copy(params, tparams);
        periodsize = bufsize * 2;
        err = snd_pcm_hw_params_set_buffer_size_near(handle, params, &periodsize);
        if (err < 0) {
                printf("Unable to set buffer size %li for %s: %s\n", bufsize * 2, id, snd_strerror(err));
                return err;
        }
        if (period_size > 0)
                periodsize = period_size;
        else
                periodsize /= 2;
        err = snd_pcm_hw_params_set_period_size_near(handle, params, &periodsize, 0);
        if (err < 0) {
                printf("Unable to set period size %li for %s: %s\n", periodsize, id, snd_strerror(err));
                return err;
        }
        return 0;
}

int setparams_set(snd_pcm_t *handle,
                  snd_pcm_hw_params_t *params,
                  snd_pcm_sw_params_t *swparams,
                  const char *id)
{
        int err;
        snd_pcm_uframes_t val;
        err = snd_pcm_hw_params(handle, params);
        if (err < 0) {
                printf("Unable to set hw params for %s: %s\n", id, snd_strerror(err));
                return err;
        }
        err = snd_pcm_sw_params_current(handle, swparams);
        if (err < 0) {
                printf("Unable to determine current swparams for %s: %s\n", id, snd_strerror(err));
                return err;
        }
        err = snd_pcm_sw_params_set_start_threshold(handle, swparams, 0x7fffffff);
        if (err < 0) {
                printf("Unable to set start threshold mode for %s: %s\n", id, snd_strerror(err));
                return err;
        }
        if (!block)
                val = 4;
        else
                snd_pcm_hw_params_get_period_size(params, &val, NULL);
        err = snd_pcm_sw_params_set_avail_min(handle, swparams, val);
        if (err < 0) {
                printf("Unable to set avail min for %s: %s\n", id, snd_strerror(err));
                return err;
        }
        err = snd_pcm_sw_params(handle, swparams);
        if (err < 0) {
                printf("Unable to set sw params for %s: %s\n", id, snd_strerror(err));
                return err;
        }
        return 0;
}

//long writebuf(snd_pcm_t *handle, char *buf, int len, size_t *frames)
//{
//        long r;
//        while (len > 0) {
//                r = snd_pcm_writei(handle, buf, len);
//                if (r == -EAGAIN)
//                        continue;
//                // printf("write = %li\n", r);
//                if (r < 0)
//                        return r;
//                // showstat(handle, 0);
//                buf += r * 4;
//                len -= r;
//                *frames += r;
//        }
//        return 0;
//}


int setparams(snd_pcm_t *phandle, snd_pcm_t *chandle, int *bufsize)
{
        int err, last_bufsize = *bufsize;
        snd_pcm_hw_params_t *pt_params, *ct_params;     /* templates with rate, format and channels */
        snd_pcm_hw_params_t *p_params, *c_params;
        snd_pcm_sw_params_t *p_swparams, *c_swparams;
        snd_pcm_uframes_t p_size, c_size, p_psize, c_psize;
        unsigned int p_time, c_time;
        unsigned int val;
        snd_pcm_hw_params_alloca(&p_params);
        snd_pcm_hw_params_alloca(&c_params);
        snd_pcm_hw_params_alloca(&pt_params);
        snd_pcm_hw_params_alloca(&ct_params);
        snd_pcm_sw_params_alloca(&p_swparams);
        snd_pcm_sw_params_alloca(&c_swparams);
        if ((err = setparams_stream(phandle, pt_params, "playback")) < 0) {
                printf("Unable to set parameters for playback stream: %s\n", snd_strerror(err));
                exit(0);
        }
        if ((err = setparams_stream(chandle, ct_params, "capture")) < 0) {
                printf("Unable to set parameters for playback stream: %s\n", snd_strerror(err));
                exit(0);
        }
        if (buffer_size > 0) {
                *bufsize = buffer_size;
                goto __set_it;
        }
      __again:
        if (buffer_size > 0)
                return -1;
        if (last_bufsize == *bufsize)
                *bufsize += 4;
        last_bufsize = *bufsize;
        if (*bufsize > latency_max)
                return -1;
      __set_it:
        if ((err = setparams_bufsize(phandle, p_params, pt_params, *bufsize, "playback")) < 0) {
                printf("Unable to set sw parameters for playback stream: %s\n", snd_strerror(err));
                exit(0);
        }
        if ((err = setparams_bufsize(chandle, c_params, ct_params, *bufsize, "capture")) < 0) {
                printf("Unable to set sw parameters for playback stream: %s\n", snd_strerror(err));
                exit(0);
        }
        snd_pcm_hw_params_get_period_size(p_params, &p_psize, NULL);
        if (p_psize > (unsigned int)*bufsize)
                *bufsize = p_psize;
        snd_pcm_hw_params_get_period_size(c_params, &c_psize, NULL);
        if (c_psize > (unsigned int)*bufsize)
                *bufsize = c_psize;
        snd_pcm_hw_params_get_period_time(p_params, &p_time, NULL);
        snd_pcm_hw_params_get_period_time(c_params, &c_time, NULL);
        if (p_time != c_time)
                goto __again;
        snd_pcm_hw_params_get_buffer_size(p_params, &p_size);
        if (p_psize * 2 > p_size) {
                snd_pcm_hw_params_get_periods_min(p_params, &val, NULL);
                if (val > 2) {
                        printf("playback device does not support 2 periods per buffer\n");
                        exit(0);
                }
                goto __again;
        }
        snd_pcm_hw_params_get_buffer_size(c_params, &c_size);
        if (c_psize * 2 > c_size) {
                snd_pcm_hw_params_get_periods_min(c_params, &val, NULL);
                if (val > 2 ) {
                        printf("capture device does not support 2 periods per buffer\n");
                        exit(0);
                }
                goto __again;
        }
        if ((err = setparams_set(phandle, p_params, p_swparams, "playback")) < 0) {
                printf("Unable to set sw parameters for playback stream: %s\n", snd_strerror(err));
                exit(0);
        }
        if ((err = setparams_set(chandle, c_params, c_swparams, "capture")) < 0) {
                printf("Unable to set sw parameters for playback stream: %s\n", snd_strerror(err));
                exit(0);
        }
        if ((err = snd_pcm_prepare(phandle)) < 0) {
                printf("Prepare error: %s\n", snd_strerror(err));
                exit(0);
        }
        //snd_pcm_dump(phandle, output);
        //snd_pcm_dump(chandle, output);
        fflush(stdout);
        return 0;
}




//-----------------------------------------------------------------------------------------------------------







void help(void)
{
        int k;
        printf(
"Usage: pcm [OPTION]... [FILE]...\n"
"-h,--help      	help\n"
"-D,--device    	playback device\n"
"-r,--rate      	stream rate in Hz\n"
"-c,--channels  	count of channels in stream\n"
"-b,--buffer    	ring buffer size in frames\n"
"-p,--period    	period size in frames\n"
"-l,--poll			use poll (0 or 1; default 1)\n"
"-P,--pdevice		playback device\n"
"-C,--cdevice   	capture device\n"
"-m,--method    	transfer method\n"
"-o,--format    	sample format\n"
"-k, --block		use block mode\n"
//"-v,--verbose   	show the PCM setup parameters\n"
"-n,--noresample  	do not resample\n"
"-e,--pevent    	enable poll event after each period\n"
"\n");
        printf("Recognized sample formats are:");
        for (k = 0; k < SND_PCM_FORMAT_LAST; ++k) {
                const char *s = snd_pcm_format_name(k);
                if (s)
                        printf(" %s", s);
        }
        printf("\n");
        printf("Recognized transfer methods are:");
        for (k = 0; transfer_methods[k].name; k++)
                printf(" %s", transfer_methods[k].name);
        printf("\n");
}

int parseArguments(int argc, char *argv[])
{
	struct option long_option[] =
	{
		{"help", 0, NULL, 'h'},
		{"device", 1, NULL, 'D'},
		{"rate", 1, NULL, 'r'},
		{"channels", 1, NULL, 'c'},
		{"buffer", 1, NULL, 'b'},
		{"period", 1, NULL, 'p'},
        {"pdevice", 1, NULL, 'P'},
        {"cdevice", 1, NULL, 'C'},
        {"use_poll", 1, NULL, 'l'},
        {"method", 1, NULL, 'm'},
		{"format", 1, NULL, 'o'},
		{"block",  0, NULL, 'k'},
		//{"verbose", 1, NULL, 'v'},
		{"noresample", 1, NULL, 'n'},
		{"pevent", 1, NULL, 'e'},
		{NULL, 0, NULL, 0},
	};


	morehelp = 0;
	while (1) {
			int c;
			if ((c = getopt_long(argc, argv, "hD:r:c:b:P:l:C:kp:m:o:ne", long_option, NULL)) < 0)
					break;
			switch (c) {
			case 'h':
					morehelp++;
					break;
			case 'D':
					device = strdup(optarg);
					break;
            case 'P':
                    pdevice = strdup(optarg);
                    break;
            case 'C':
                    cdevice = strdup(optarg);
                    break;
            case 'l':
                    use_poll = atoi(optarg);
                    break;
            case 'r':
					rate = atoi(optarg);
					rate = rate < 4000 ? 4000 : rate;
					rate = rate > 196000 ? 196000 : rate;
					break;
			case 'c':
					channels = atoi(optarg);
					channels = channels < 1 ? 1 : channels;
					channels = channels > 2 ? 2 : channels;
					break;
			case 'b':
					buffer_size = atoi(optarg);
					break;
			case 'k':
			        block = 1;
			        break;
			case 'p':
					period_size = atoi(optarg);
					break;
			case 'm':
					for (method = 0; transfer_methods[method].name; method++)
									if (!strcasecmp(transfer_methods[method].name, optarg))
									break;
					if (transfer_methods[method].name == NULL)
							method = 0;
					break;
			case 'o':
					for (format = 0; format < SND_PCM_FORMAT_LAST; format++) {
							const char *format_name = snd_pcm_format_name(format);
							if (format_name)
									if (!strcasecmp(format_name, optarg))
									break;
					}
					if (format == SND_PCM_FORMAT_LAST)
							format = SND_PCM_FORMAT_S16;
					if (!snd_pcm_format_linear(format) &&
						!(format == SND_PCM_FORMAT_FLOAT_LE ||
						  format == SND_PCM_FORMAT_FLOAT_BE)) {
							printf("Invalid (non-linear/float) format %s\n",
								   optarg);
							return 1;
					}
					break;
			/*case 'v':
					verbose = 1;
					break;*/
			case 'n':
					resample = 0;
					break;
			case 'e':
					period_event = 1;
					break;
			}
	}
	if (morehelp) {
			help();
			return -1;
	}

	return 0;
}

/* Function which returns the time since start of the program
 * in (fractional) seconds.
 */
double getCurrentTime(void) {
	unsigned long long result;
	struct timeval tv;

	gettimeofday(&tv, NULL);
	result = (tv.tv_sec - gFirstSeconds) * 1000000ULL + (tv.tv_usec - gFirstMicroseconds);
	return (double)result / 1000000.0;
}

int initDrums() {
	/* Load drums from WAV files */
	SNDFILE *sndfile ;
	SF_INFO sfinfo ;
	char filename[64];

	for(int i = 0; i < NUMBER_OF_DRUMS; i++) {
		snprintf(filename, 64, "drums/drum%d.wav", i);

		if (!(sndfile = sf_open (filename, SFM_READ, &sfinfo))) {
			printf("Couldn't open file %s\n", filename);

			/* Free already loaded sounds */
			for(int j = 0; j < i; j++)
				free(gDrumSampleBuffers[j]);
			return 1;
		}

		if (sfinfo.channels != 1) {
			printf("Error: %s is not a mono file\n", filename);

			/* Free already loaded sounds */
			for(int j = 0; j < i; j++)
				free(gDrumSampleBuffers[j]);
			return 1;
		}

		gDrumSampleBufferLengths[i] = sfinfo.frames;
		gDrumSampleBuffers[i] = malloc(gDrumSampleBufferLengths[i] * sizeof(float));
		if(gDrumSampleBuffers[i] == NULL) {
			printf("Error: couldn't allocate buffer for %s\n", filename);

			/* Free already loaded sounds */
			for(int j = 0; j < i; j++)
				free(gDrumSampleBuffers[j]);
			return 1;
		}

		int subformat = sfinfo.format & SF_FORMAT_SUBMASK;
		int readcount = sf_read_float(sndfile, gDrumSampleBuffers[i], gDrumSampleBufferLengths[i]);

		/* Pad with zeros in case we couldn't read whole file */
		for(int k = readcount; k < gDrumSampleBufferLengths[i]; k++)
			gDrumSampleBuffers[i][k] = 0;

		if (subformat == SF_FORMAT_FLOAT || subformat == SF_FORMAT_DOUBLE) {
			double	scale ;
			int 	m ;

			sf_command (sndfile, SFC_CALC_SIGNAL_MAX, &scale, sizeof (scale)) ;
			if (scale < 1e-10)
				scale = 1.0 ;
			else
				scale = 32700.0 / scale ;
			printf("Scale = %f\n", scale);

			for (m = 0; m < gDrumSampleBufferLengths[i]; m++)
				gDrumSampleBuffers[i][m] *= scale;
		}

		sf_close(sndfile);
	}

	return 0;
}

void cleanupDrums() {
	for(int i = 0; i < NUMBER_OF_DRUMS; i++)
		free(gDrumSampleBuffers[i]);
}

void initPatterns() {
	int pattern0[16] = {0x01, 0x40, 0, 0, 0x02, 0, 0, 0, 0x20, 0, 0x01, 0, 0x02, 0, 0x04, 0x04};
	int pattern1[32] = {0x09, 0, 0x04, 0, 0x06, 0, 0x04, 0,
		 0x05, 0, 0x04, 0, 0x06, 0, 0x04, 0x02,
		 0x09, 0, 0x20, 0, 0x06, 0, 0x20, 0,
		 0x05, 0, 0x20, 0, 0x06, 0, 0x20, 0};
	int pattern2[16] = {0x11, 0, 0x10, 0x01, 0x12, 0x40, 0x04, 0x40, 0x11, 0x42, 0x50, 0x01, 0x12, 0x21, 0x30, 0x20};
	int pattern3[32] = {0x81, 0x80, 0x80, 0x80, 0x01, 0x80, 0x80, 0x80, 0x81, 0, 0, 0, 0x41, 0x80, 0x80, 0x80,
		0x81, 0x80, 0x80, 0, 0x41, 0, 0x80, 0x80, 0x81, 0x80, 0x80, 0x80, 0xC1, 0, 0, 0};
	int pattern4[16] = {0x81, 0x02, 0, 0x81, 0x0A, 0, 0xA1, 0x10, 0xA2, 0x11, 0x46, 0x41, 0xC5, 0x81, 0x81, 0x89};

	gPatternLengths[0] = 16;
	gPatterns[0] = malloc(gPatternLengths[0] * sizeof(int));
	memcpy(gPatterns[0], pattern0, gPatternLengths[0] * sizeof(int));

	gPatternLengths[1] = 32;
	gPatterns[1] = malloc(gPatternLengths[1] * sizeof(int));
	memcpy(gPatterns[1], pattern1, gPatternLengths[1] * sizeof(int));

	gPatternLengths[2] = 16;
	gPatterns[2] = malloc(gPatternLengths[2] * sizeof(int));
	memcpy(gPatterns[2], pattern2, gPatternLengths[2] * sizeof(int));

	gPatternLengths[3] = 32;
	gPatterns[3] = malloc(gPatternLengths[3] * sizeof(int));
	memcpy(gPatterns[3], pattern3, gPatternLengths[3] * sizeof(int));

	gPatternLengths[4] = 32;
	gPatterns[4] = malloc(gPatternLengths[4] * sizeof(int));
	memcpy(gPatterns[4], pattern3, gPatternLengths[4] * sizeof(int));

	gPatternLengths[5] = 16;
	gPatterns[5] = malloc(gPatternLengths[5] * sizeof(int));
	memcpy(gPatterns[5], pattern4, gPatternLengths[5] * sizeof(int));
}

void cleanupPatterns() {
	for(int i = 0; i < NUMBER_OF_PATTERNS; i++)
		free(gPatterns[i]);
}

int main(int argc, char *argv[])
{
	pthread_t sensorThread, audioThread;
	struct timeval tv;

	if(parseArguments(argc, argv)!=0)
	{
		printf("Exit program\n");
		return 1;
	}
	latency = latency_min-4;

	gettimeofday(&tv, NULL);
	gFirstSeconds = tv.tv_sec;
	gFirstMicroseconds = tv.tv_usec;

	gShouldStop = 0;

    // Set up interrupt handler to catch Control-C
    signal(SIGINT, interrupt_handler);

//    if(initDrums()) {
//    	printf("Unable to load drum sounds. Check that you have all the WAV files!\n");
//    	exit(1);
//    }
	if(analogInit()) {
		printf("Initialising analog input failed. Aborting.\n");
		exit(1);
	}
	initialise(rate, channels);
	pthread_create(&sensorThread, NULL, sensorLoop, NULL);
//	pthread_create(&audioThread, NULL, audioGenerator, NULL);

	(void) pthread_join(sensorThread, NULL);
	(void) pthread_join(audioThread, NULL);
	analogCleanup();
	cleanup();
//	cleanupPatterns();
//	cleanupDrums();

	printf("Bye bye\n");
	return 0;
}
