#include <mpfr.h>
#include <math.h>
#include <stdint.h>
#include <pthread.h>
#include "sensor.h"




/* M_PI is not declared in all C implementations... */
#ifndef		M_PI
#define		M_PI		3.14159265358979323846264338
#endif

typedef short  Q0n16;
typedef int32_t Q32n0;
typedef int32_t Q24n8;
typedef int32_t Q16n16;
typedef int32_t Q8n24;

typedef int64_t Q48n16;
typedef int64_t Q32n32;
typedef int64_t Q16n48;

/* Initialisation function, called once at the beginning */
void initialise(int sampleRate, int numChannels);

/* Render function, called when new samples are needed */
void render(int sampleRate, int numChannels, int numFrames, float *sampleBuffer);
void calcCrossCoef();
void calcParaCoef(int index);
void applyCrossCoef(float inputSample, float *outputSample);
void applyParaCoef(float* inputSample, float* outputSample);
void updateHardwareControl(int aPin);
void compressor(float inputSignal);
void setDelayReadPointer();

void cleanup(int numChannels);
