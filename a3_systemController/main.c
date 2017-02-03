
/*
 *  Latency test program
 *
 *     Author: Jaroslav Kysela <perex@perex.cz>
 *
 *     Author of bandpass filter sweep effect:
 *             Maarten de Boer <mdeboer@iua.upf.es>
 *
 *  This small demo program can be used for measuring latency between
 *  capture and playback. This latency is measured from driver (diff when
 *  playback and capture was started). Scheduler is set to SCHED_RR.
 *
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/* Needed to avoid an ALSA bug with C99 */
#define _POSIX_C_SOURCE		200809L

#define _BSD_SOURCE


#include <stdio.h>
#include <glob.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
//#include <bits/sched.h>
#include <asm-generic/errno.h>
#include <getopt.h>
#include <alsa/asoundlib.h>
#include <sys/time.h>
#include <math.h>
#include <alloca.h>
#include <signal.h>
#include "audio.h"
#include "sensor.h"


char *pdevice = "hw:0,0";
char *cdevice = "hw:0,0";
snd_pcm_format_t format = SND_PCM_FORMAT_S16_LE;
int rate = 44100;
int channels = 2;
int buffer_size = 596;            /* auto */
int period_size = 8;            /* auto */
int latency_min = 32;           /* in frames / 2 */
int latency_max = 2048;         /* in frames / 2 */
int loop_sec = 30;              /* seconds */
int block = 0;                  /* block mode */
int use_poll = 1;
int resample = 1;
int gShouldStop = 0;
snd_output_t *output = NULL;
char *buffer;
float *floatBuffer;
snd_pcm_t *phandle, *chandle;
int err, latency, morehelp;
int ok;



// Handle Ctrl-C
void interrupt_handler(int var)
{
	gShouldStop = 1;
}

int setparams_stream(snd_pcm_t *handle,
                     snd_pcm_hw_params_t *params,
                     const char *id)
{
        int err;
        unsigned int rrate;
        err = snd_pcm_hw_params_any(handle, params);
        if (err < 0) {
                printf("Broken configuration for %s PCM: no configurations available: %s\n", snd_strerror(err), id);
                return err;
        }
        err = snd_pcm_hw_params_set_rate_resample(handle, params, resample);
        if (err < 0) {
                printf("Resample setup failed for %s (val %i): %s\n", id, resample, snd_strerror(err));
                return err;
        }
        err = snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
        if (err < 0) {
                printf("Access type not available for %s: %s\n", id, snd_strerror(err));
                return err;
        }
        err = snd_pcm_hw_params_set_format(handle, params, format);
        if (err < 0) {
                printf("Sample format not available for %s: %s\n", id, snd_strerror(err));
                return err;
        }
        err = snd_pcm_hw_params_set_channels(handle, params, channels);
        if (err < 0) {
                printf("Channels count (%i) not available for %s: %s\n", channels, id, snd_strerror(err));
                return err;
        }
        rrate = rate;
        err = snd_pcm_hw_params_set_rate_near(handle, params, &rrate, 0);
        if (err < 0) {
                printf("Rate %iHz not available for %s: %s\n", rate, id, snd_strerror(err));
                return err;
        }
        if ((int)rrate != rate) {
                printf("Rate doesn't match (requested %iHz, get %iHz)\n", rate, err);
                return -EINVAL;
        }
        return 0;
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

// Set the parameters for the capture and playback streams
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

// Set the scheduler to the highest priority
void setscheduler(void)
{
        struct sched_param sched_param;
        if (sched_getparam(0, &sched_param) < 0) {
                printf("Scheduler getparam failed...\n");
                return;
        }
        sched_param.sched_priority = sched_get_priority_max(SCHED_RR);
        if (!sched_setscheduler(0, SCHED_RR, &sched_param)) {
                printf("Scheduler set to Round Robin with priority %i...\n", sched_param.sched_priority);
                fflush(stdout);
                return;
        }
        printf("!!!Scheduler set to Round Robin with priority %i FAILED!!!\n", sched_param.sched_priority);
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
//                 printf("read = %li\n", r);
        } else {
                int frame_bytes = (snd_pcm_format_width(format) / 8) * channels;
                do {
                        r = snd_pcm_readi(handle, buf, len);
                        if (r > 0) {
                                buf += r * frame_bytes;
                                len -= r;
                                *frames += r;
                        }
//                         printf("r = %li, len = %li\n", r, len);
                } while (r >= 1 && len > 0);
        }
        // showstat(handle, 0);
        return r;
}

// Write to the audio playback device
long writebuf(snd_pcm_t *handle, char *buf, long len, size_t *frames)
{
        long r;
        while (len > 0) {
                r = snd_pcm_writei(handle, buf, len);
                if (r == -EAGAIN)
                        continue;
                // printf("write = %li\n", r);
                if (r < 0)
                        return r;
                // showstat(handle, 0);
                buf += r * 4;
                len -= r;
                *frames += r;
        }
        return 0;
}
//--------------------------------------------------------------------------
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

//--------------------------------------------------------------------------
void audioLoop(){

	ssize_t r;
	size_t frames_in, frames_out;
	// Set the priority of the audio thread and use round-robin scheduler
	setscheduler();

	// Set up interrupt handler to catch Control-C
	signal(SIGINT, interrupt_handler);
	//    set_realtime_priority(2, SCHED_RR);

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
				if ((r = readbuf(chandle, buffer, latency, &frames_in)) < 0){
					ok = 0;
//					printf("underrunCode 1\n");
				}
				else {
					// Convert short (16-bit) samples to float
					short *shortBuffer = (short *)buffer;
					int n;

					for(n = 0; n < r * channels; n++)
						floatBuffer[n] = (float)shortBuffer[n] / 32768.0; //(short / 2^15)

//					printf("rate %d \n", rate);
//					printf("Channels %d \n", channels);
//					printf("r %d\n", r);
//					printf("floatBuffer %d \n", floatBuffer[0]);


					render(rate, channels, r, floatBuffer);
//					printf("Post Render\n");
					// Convert float back to short
					for(n = 0; n < r * channels; n++)
						shortBuffer[n] = (short)(floatBuffer[n] * 32768.0);

					if (writebuf(phandle, buffer, r, &frames_out) < 0){
							ok = 0;
//						printf("underrunCode 2\n");
					}

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
}


void help(void)
{
	printf(
		"Usage: latency [OPTION]... [FILE]...\n"
		"-h,--help      help\n"
		"-P,--pdevice   playback device\n"
		"-C,--cdevice   capture device\n"
		"-f,--frequency crossover frequency in Hz\n"
		"-r,--rate      rate\n"
		"-B,--buffer    buffer size in frames\n"
		"-E,--period    period size in frames\n"
		"-b,--block     use block mode\n"
		"-p,--poll      use poll (0 or 1; default 1)\n"
		"-o,--order     set the filter order (even numbers only) \n "
		);
}
int main(int argc, char *argv[])
{
        struct option long_option[] =
        {
                {"help", 0, NULL, 'h'},
                {"pdevice", 1, NULL, 'P'},
                {"cdevice", 1, NULL, 'C'},
                {"rate", 1, NULL, 'r'},
                {"buffer", 1, NULL, 'B'},
                {"period", 1, NULL, 'E'},
                {"block", 0, NULL, 'b'},
                {"poll", 1, NULL, 'p'},
                {"frequency", 0, NULL, 'f'},
                {"order", 0, NULL, 'o'},
                {NULL, 0, NULL, 0},
        };



        float frequency = 1000.0;
        int filterOrder = 1;



        morehelp = 0;
        while (1) {
                int c;
                if ((c = getopt_long(argc, argv, "hP:C:f:r:B:E:bp:no:", long_option, NULL)) < 0)
                        break;
                switch (c) {
                case 'h':
                        morehelp++;
                        break;
                case 'P':
                        pdevice = strdup(optarg);
                        break;
                case 'C':
                        cdevice = strdup(optarg);
                        break;
                case 'f':
                		frequency = atof(optarg);
                		if(frequency < 20.0)
                			frequency = 20.0;
                		if(frequency > 5000.0)
                			frequency = 5000.0;
                		break;
                case 'r':
                        err = atoi(optarg);
                        rate = err >= 4000 && err < 200000 ? err : 44100;
                        break;
                case 'B':
                        err = atoi(optarg)/2;
                        buffer_size = err;// >= 32 && err < 200000 ? err : 0;
                        break;
                case 'E':
                        err = atoi(optarg);
                        period_size = err;// >= 32 && err < 200000 ? err : 0;
                        break;
                case 'b':
                        block = 1;
                        break;
                case 'p':
                        use_poll = atoi(optarg);
                        break;
                case 'n':
                        resample = 0;
                        break;
                case 'o':
                	filterOrder = (int)(atof(optarg)/2);
                	break;

                }
        }

        if (morehelp) {
			help();
			return 0;
        }

        err = snd_output_stdio_attach(&output, stdout, 0);
        if (err < 0) {
                printf("Output failed: %s\n", snd_strerror(err));
                return 0;
        }

        // Set latency
        latency = latency_min - 4;

        // Allocate buffers
        buffer = malloc((latency_max * snd_pcm_format_width(format) / 8) * 2);
        floatBuffer = malloc(latency_max * sizeof(float) * 2);


        // Print stats
        printf("Playback device is %s\n", pdevice);
        printf("Capture device is %s\n", cdevice);
        printf("Parameters are %iHz, %s, %i channels, %s mode\n", rate, snd_pcm_format_name(format), channels, block ? "blocking" : "non-blocking");
        printf("Poll mode: %s\n", use_poll ? "yes" : "no");

        // Open capture and playback devices
        if ((err = snd_pcm_open(&phandle, pdevice, SND_PCM_STREAM_PLAYBACK, block ? 0 : SND_PCM_NONBLOCK)) < 0) {
                printf("Playback open error: %s\n", snd_strerror(err));
                return 0;
        }
        if ((err = snd_pcm_open(&chandle, cdevice, SND_PCM_STREAM_CAPTURE, block ? 0 : SND_PCM_NONBLOCK)) < 0) {
                printf("Record open error: %s\n", snd_strerror(err));
                return 0;
        }
    	pthread_t sensorThread, audioThread;
        signal(SIGINT, interrupt_handler);

        // Call the user initialisation code
    	if(analogInit()) {
    		printf("Initialising analog input failed. Aborting.\n");
    		exit(1);
    	}
        initialise((float)rate, channels);

        pthread_create(&sensorThread, NULL, sensorLoop, NULL);
        pthread_create(&audioThread, NULL, audioLoop, NULL);

        	(void) pthread_join(sensorThread, NULL);
        	(void) pthread_join(audioThread, NULL);
//THREAD HERE
        printf("Cleaning up...\n");

        // Clean up and deallocate
    	analogCleanup();
    	cleanup(channels);
    	snd_pcm_close(phandle);
        snd_pcm_close(chandle);
        free(floatBuffer);
        free(buffer);

        return 0;
}


