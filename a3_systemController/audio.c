#include "audio.h"
//#include "fixedPoint.h"

#include <math.h>
#include <stddef.h>
#include <mpfr.h>


#define DELAY_SIZE 2
#define ANALOGMAX 1800
#define NUMPARAEQ 4
#define DELAYSPEED 16



//extern float gCrossoverFrequency;
//extern int **gAnalogData;
extern int gButtonFlag;
extern float gControlValues[NUMANALOG][NUMSWITCHV];



int gwritePointer;
float* gxBuffer;
float* gyLBuffer;
float* gyHBuffer;
float** delayBuffer;
float gDelayReadPointer[2] = {0.0};
int gDelayWritePointer[2] = {0};
float gCurrentDelay0 = 0;
float gCurrentDelay1 = 0;

float** paraXBuffer; // paraXBuffer[dlyLine][filterIndex]
float** paraYBuffer; // paraYBuffer[dlyLine][filterIndex]
int* xBufferPointer; //xBufferPointer[filterIndex]
int* yBufferPointer; //yBufferPointer[filterIndex]

float gLPcoefficients[5]; /*structure: a1, a2, b0, b1, b2*/
float gHPcoefficients[5]; /*structure: a1, a2, b0, b1, b2*/

float gParaCoef[7][NUMPARAEQ]; /*structure: a1, a2, b0, b1, b2, c0, d0 per parametric eq, where c0 = filter gain, d0 = original gain*/
int gSampleRate;
int filterOrder = 2;

// Compressor Global Variables
float gCompGain, yL_prev, aAttack, aRelease;
pthread_mutex_t paraCoef;
pthread_mutex_t crossCoef;



void initialise(int sampleRate, int numChannels) {
	// Memory allocations
	gSampleRate = sampleRate;

	gxBuffer = (float *)malloc((int)DELAY_SIZE*sizeof(float));
	gyLBuffer = (float *)malloc((int)DELAY_SIZE*sizeof(float));
	gyHBuffer = (float *)malloc((int)DELAY_SIZE*sizeof(float));
	delayBuffer = (float **)malloc((int)numChannels*sizeof(float*));

	for(int channel = 0; channel < numChannels; channel++){
		delayBuffer[channel] = (float *)malloc((int)sampleRate*sizeof(float));
	}


	xBufferPointer = (int *)malloc((int)NUMPARAEQ*sizeof(int));
	yBufferPointer = (int *)malloc((int)NUMPARAEQ*sizeof(int));

	paraXBuffer = (float **)malloc((int)DELAY_SIZE*sizeof(float*)) ;
	paraYBuffer = (float **)malloc((int)DELAY_SIZE*sizeof(float*)) ;
	for(int i = 0; i < DELAY_SIZE; i++){
		paraXBuffer[i] = (float *)malloc((int)NUMPARAEQ*sizeof(float));
		paraYBuffer[i] = (float *)malloc((int)NUMPARAEQ*sizeof(float));
	}
	// Create Mutex's for filter coefficients
	pthread_mutex_init(&paraCoef, NULL);
	pthread_mutex_init(&crossCoef, NULL);
	if(gxBuffer == NULL || gyLBuffer == NULL || gyHBuffer == NULL || delayBuffer == NULL ){
		printf ("Error creating delay buffer");
	}

	// initialise variables
	for(int i = 0; i < DELAY_SIZE; i++){
		gyLBuffer[i] = 0.0;
		gyHBuffer[i] = 0.0;
		gxBuffer[i]= 0.0;
	}

	for(int init = 0; init < NUMPARAEQ; init++){
		xBufferPointer[init] = 0;
		yBufferPointer[init] = 0;
	}
	gwritePointer = 0;
	setDelayReadPointer();

	// Initialise filter coefficients
	calcCrossCoef();
	for(int paraEqIndex = 0; paraEqIndex < NUMPARAEQ; paraEqIndex++)
		calcParaCoef(paraEqIndex);
}



void render(int sampleRate, int numChannels, int numFrames, float *sampleBuffer) {
	float sampleCross[2];
	float samplePara[2];
	float sampleOut[2];

	// Loop through every audio frame
	for(int frame = 0; frame < numFrames; frame++){
			// Define input as 0, to collect sum data
			float xn0 = 0.0;
			// create sample buffer for LP/HP per order
			for(int channel = 0; channel < numChannels; channel++){
				// Sum inputs to create mono track
				xn0 = xn0 + sampleBuffer[numChannels * frame + channel];
			}
			xn0 = xn0/numChannels; // Normalise mono input
			// Calculate compression value
			compressor(xn0);
			// Apply audio crossover
			applyCrossCoef(xn0, &sampleCross);

			//delay each output channel, with fractional delay
			delayBuffer[0][gDelayWritePointer[0]] = sampleCross[0];
			delayBuffer[1][gDelayWritePointer[1]] = sampleCross[1];
			setDelayReadPointer();
			float ceilPercent0 = ceil(gDelayReadPointer[0]) - (gDelayReadPointer[0]);
			float ceilPercent1 = ceil(gDelayReadPointer[1]) - (gDelayReadPointer[1]);
			samplePara[0] = delayBuffer[0][(int)floor(gDelayReadPointer[0])] * (1-ceilPercent0) + delayBuffer[0][(int)ceil(gDelayReadPointer[0])] * ceilPercent0 ;
			samplePara[1] = delayBuffer[1][(int)floor(gDelayReadPointer[1])] * (1-ceilPercent1) + delayBuffer[1][(int)ceil(gDelayReadPointer[1])] * ceilPercent1 ;

			// Apply parametric eqs
			sampleOut[0] = samplePara[0];
			sampleOut[1] = samplePara[1];
			applyParaCoef(&samplePara, &sampleOut);

			// Produce sample buffer with channel audio, audio gain and compressor gain.
			sampleBuffer[numChannels * frame + 0] = sampleOut[0]* gControlValues[1][0] * gCompGain;
			sampleBuffer[numChannels * frame + 1] = sampleOut[1]* gControlValues[3][0] * gCompGain;

			// update read and write pointers
			gDelayWritePointer[0]++;
			if(gDelayWritePointer[0] >= sampleRate)
				gDelayWritePointer[0] = 0;
			gDelayWritePointer[1]++;
			if(gDelayWritePointer[1] >= sampleRate)
				gDelayWritePointer[1] = 0;

			gDelayReadPointer[0]++;
			if(gDelayReadPointer[0] >= sampleRate)
				gDelayReadPointer[0] -= sampleRate;
			gDelayReadPointer[1]++;
			if(gDelayReadPointer[1] >= sampleRate)
				gDelayReadPointer[1] -= sampleRate;
	}
}

void setDelayReadPointer(){
		// Read hardware control values
		int delayValue0 = gControlValues[2][0];
		int delayValue1 = gControlValues[4][0];

		float delaySize0 = delayValue0 - gCurrentDelay0;
		float delaySize1 = delayValue1 - gCurrentDelay1;

		if(gCurrentDelay0 == (float)delayValue0){ // If current delay is as required - do nothing
		}
		else if((gCurrentDelay0 - delayValue0) > 1){ // If current delay is larger than required delay
			// move current delay closer to required delay
			gDelayReadPointer[0] += delaySize0/DELAYSPEED;
			gCurrentDelay0 += delaySize0/DELAYSPEED;
		}
		else if(((float)delayValue0 - gCurrentDelay0) > 1){// If current delay is smaller than required delay
			// move current delay closer to required delay

			gDelayReadPointer[0] += (delaySize0/DELAYSPEED);
			gCurrentDelay0 += (delaySize0/DELAYSPEED);
		}
		else if((gCurrentDelay0 - (float)delayValue0 < 1) || ((float)delayValue0 - gCurrentDelay0) < 1){ // If current delay is very close to required delay
			// Set current delay to required delay
			gDelayReadPointer[0] = (gDelayWritePointer[0] - delayValue0 + gSampleRate) % gSampleRate;
			gCurrentDelay0 = (float)delayValue0;
			delaySize0 = 0.0;
		}
		if(gCurrentDelay1 == (float)delayValue1){// If current delay is as required - do nothing
		}
		else if((gCurrentDelay1 - (float)delayValue1) > 1){ // If current delay is larger than required delay
			// move current delay closer to required delay
			gDelayReadPointer[1] += delaySize1/DELAYSPEED;
			gCurrentDelay1 += delaySize1/DELAYSPEED;
		}
		else if(((float)delayValue1 - gCurrentDelay1) > 1){// If current delay is smaller than required delay
			// move current delay closer to required delay
			gDelayReadPointer[1] += (delaySize1/DELAYSPEED);
			gCurrentDelay1 += (delaySize1/DELAYSPEED);
		}
		else if((gCurrentDelay1 - (float)delayValue1 < 1) || ((float)delayValue1 - gCurrentDelay1) < 1){ // If current delay is very close to required delay
			// Set current delay to required delay
			gDelayReadPointer[1] = (gDelayWritePointer[1] - delayValue1 + gSampleRate) % gSampleRate;
			gCurrentDelay1 = (float)delayValue1;
			delaySize1 = 0.0;
		}

		// update read pointers
		if(gDelayReadPointer[0] >= gSampleRate)
			gDelayReadPointer[0] = (gDelayReadPointer[0] - (float)gSampleRate);
		if(gDelayReadPointer[1] >= gSampleRate)
			gDelayReadPointer[1] = (gDelayReadPointer[1] - (float)gSampleRate);
}

void compressor(float inputSignal){
	// Taken From DAFx Module Code Examples
	float x_g, x_l, y_g, y_l;

	float T = gControlValues[0][1];
	aAttack = gControlValues[3][1];
	aRelease = gControlValues[4][1];

	// Level detection- estimate level using peak detector
	if (fabs(inputSignal) < 0.000001)
		x_g = -120;
	else
		x_g = 20*log10(fabs(inputSignal));
	// Gain computer- static apply input/output curve
	if (x_g >= T)
		y_g = T + (x_g - T) / gControlValues[1][1];
	else
		y_g = x_g;
	x_l = x_g - y_g;
	// Ballistics- smoothing of the gain
	if (x_l > yL_prev)
		y_l = aAttack * yL_prev + (1 - aAttack ) * x_l ;
	else
		y_l = aRelease* yL_prev + (1 - aRelease) * x_l;
	// find control
	gCompGain = pow(10,(gControlValues[5][1] - y_l)/20);
	yL_prev=y_l;

}

void calcCrossCoef(){
	// read hardware input
	float omegaC = M_PI * gControlValues[0][0];
	printf(" omegaC = %f \n", omegaC);

	if(omegaC >= 1){ // If crossover frequency > 1Hz
		// Begin to calculate coefficients
		float omegaCS = omegaC*omegaC;
		float kappa = omegaC / (tan(omegaC/(float)gSampleRate));

		float kappaS = kappa*kappa;
		float invdelta = 1/(kappaS + omegaCS + 2*kappa*omegaC);
		float la1 = omegaCS * invdelta;
		float ha1 = kappaS*invdelta;
		float b1 = (-2*kappaS + 2*omegaCS) * invdelta;
		float b2 = (-2*kappa*omegaC + kappaS + omegaCS) * invdelta;
		// lock thread so coefficeints cannot be changed while being applied
		pthread_mutex_lock(&crossCoef);
		// update crossover coefficients global variable
		gLPcoefficients[2] = gLPcoefficients[4] = la1;
		gLPcoefficients[3] = 2*la1;
		gHPcoefficients[2] = gHPcoefficients[4] = ha1;
		gHPcoefficients[3] = -2*ha1;
		gLPcoefficients[0] = gHPcoefficients[0] = b1;
		gLPcoefficients[1] = gHPcoefficients[1] = b2;
		// unlock thread
		pthread_mutex_unlock(&crossCoef);

	}
	else{ // If crossover frequency is < 1Hz
		// lock thread so coefficeints cannot be changed while being applied
		pthread_mutex_lock(&crossCoef);
		// Set coefficents as an all pass. No filter applied
		gHPcoefficients[2] = gLPcoefficients[2] = 1;
		gHPcoefficients[0] = gHPcoefficients[1] = gHPcoefficients[3] = gHPcoefficients[4] = 0;
		gLPcoefficients[0] = gLPcoefficients[1] = gLPcoefficients[3] = gLPcoefficients[4] = 0;
		// unlock thread
		pthread_mutex_unlock(&crossCoef);

	}
}

void calcParaCoef(int index){
	// perform precalculation and read in suitable hardware controls
	int channel, ctrlIndex;
	float omega, A, zeta, gamma, beta, alpha, a0Inv, cOmegaInvA0;

	if(index <=1)
		channel = 2;
	else
		channel = 3;

	if((index % 2) == 0)
		ctrlIndex = 0;
	else
		ctrlIndex = 3;

	if(gControlValues[ctrlIndex+2][channel] == 0){ //If Filter gain = 0dB,
		//lock thread
		pthread_mutex_lock(&paraCoef);
		// Set filter coefficents as all pass through
		gParaCoef[0][index] = 0;
		gParaCoef[1][index] = 0;
		gParaCoef[2][index] = 0;
		gParaCoef[3][index] = 0;
		gParaCoef[4][index] = 0;
		gParaCoef[5][index] = 0;
		gParaCoef[6][index] = 1;
		printf("Parametric Filter %d has crossoverFreq = %f Hz, Q = %f and gain = %f dB  \n", index, gControlValues[ctrlIndex][channel], gControlValues[ctrlIndex+1][channel], gControlValues[ctrlIndex+2][channel]);
		// Unlock thread
		pthread_mutex_unlock(&paraCoef);

	}
	else{ // If filter gain is not 0 dB
		// Precalculate some coefficient values, based on hardware inputs
		omega = 2 * M_PI * gControlValues[ctrlIndex][channel] / gSampleRate;
		A = pow(10,(gControlValues[ctrlIndex+2][channel]/20));
		zeta = 1/(1+A);
		beta = 0.5*((1-zeta*tan(omega/(2*gControlValues[ctrlIndex+1][channel])))/(1+zeta*tan(omega/(2*gControlValues[ctrlIndex+1][channel]))));
		gamma = (0.5+beta)*cos(omega);
		// lock thread
		pthread_mutex_lock(&paraCoef);
		// update filter coefficients global variable
		gParaCoef[0][index] = -2*gamma;
		gParaCoef[1][index] = 2*beta ;
		gParaCoef[2][index] = 0.5-beta;
		gParaCoef[3][index] = 0;
		gParaCoef[4][index] = -(0.5-beta);
		gParaCoef[5][index] = A - 1.0;
		gParaCoef[6][index] = 1;
		printf("Parametric Filter %d has crossoverFreq = %f Hz, Q = %f and gain = %f dB  \n", index, gControlValues[ctrlIndex][channel], gControlValues[ctrlIndex+1][channel], gControlValues[ctrlIndex+2][channel]);
		// unlock thread
		pthread_mutex_unlock(&paraCoef);
	}

}

void applyCrossCoef(float inputSample, float *outputSample)
{
	// Lock crossover coefficients to this thread
	pthread_mutex_lock(&crossCoef);
	// Calculate output samples for both outputs
	float YLsample = gLPcoefficients[0]*gyLBuffer[gwritePointer % DELAY_SIZE]
	               + gLPcoefficients[1]*gyLBuffer[(gwritePointer+1) % DELAY_SIZE];
	float XLsample = gLPcoefficients[2]*inputSample
			       + gLPcoefficients[3]*gxBuffer[gwritePointer]
			       + gLPcoefficients[4]*gxBuffer[(gwritePointer+1)%DELAY_SIZE];

	outputSample[0] = XLsample - YLsample;
	gyLBuffer[gwritePointer] = outputSample[0];

	float YHsample = gHPcoefficients[0]*gyHBuffer[gwritePointer % DELAY_SIZE]
	               + gHPcoefficients[1]*gyHBuffer[(gwritePointer+1) % DELAY_SIZE];
	float XHsample = gHPcoefficients[2]*inputSample
			       + gHPcoefficients[3]*gxBuffer[gwritePointer]
			       + gHPcoefficients[4]*gxBuffer[(gwritePointer+1)%DELAY_SIZE];
	// unlock thread
	pthread_mutex_unlock(&crossCoef);

	outputSample[1] = -1 * (XHsample - YHsample);
	// Invert Channel as part of LR spec.
	 gyHBuffer[gwritePointer] = outputSample[1];

	// Update pointers
	gwritePointer++;
	if(gwritePointer >= DELAY_SIZE)
		gwritePointer = 0;
	gyLBuffer[gwritePointer] = outputSample[0];
	gyHBuffer[gwritePointer] = outputSample[1];
	gxBuffer[gwritePointer] = inputSample;
}

void applyParaCoef(float* inputSample, float* outputSample){
	// perform precalculations
	int prevChannel = 0;
	int channel = 0;
	float ySample, xSample;

	// loop through all parametric filters
	for(int i = 0; i < NUMPARAEQ; i++){
		// ensure correct filter is being implemented
		channel = floor(i/2);
		// lock thread
		pthread_mutex_lock(&paraCoef);
		// Calculate filter outputs
		ySample = gParaCoef[0][i] * paraYBuffer[(yBufferPointer[i]+0) % DELAY_SIZE][i]
		               + gParaCoef[1][i] * paraYBuffer[(yBufferPointer[i]+1) % DELAY_SIZE][i];
		xSample = gParaCoef[2][i] * inputSample[channel]
				       + gParaCoef[3][i] * paraXBuffer[(xBufferPointer[i]+0) % DELAY_SIZE][i]
				       + gParaCoef[4][i] * paraXBuffer[(xBufferPointer[i]+1) % DELAY_SIZE][i];
		// lock thread
		pthread_mutex_unlock(&paraCoef);

		// save previous values and update pointers
		outputSample[channel] = ((xSample - ySample)*gParaCoef[5][i]) + (inputSample[channel] * gParaCoef[6][i]);
		paraYBuffer[(yBufferPointer[i]+0) % DELAY_SIZE][i] = outputSample[channel] ;

		yBufferPointer[i]++;
		if(yBufferPointer[i] >= DELAY_SIZE)
			yBufferPointer[i] = 0;

		xBufferPointer[i]++;
		if(xBufferPointer[i] >= DELAY_SIZE)
			xBufferPointer[i] = 0;

		paraYBuffer[gwritePointer][i] = outputSample[channel];
		paraXBuffer[gwritePointer][i] = inputSample[channel];
		inputSample[channel] = outputSample[channel];
	}
}

void updateHardwareControl(int aPin){

	// If hardware controls have been changed that effect any filters,
	// recalculate the filter coefficients,
	if(gButtonFlag == 0){
		if(aPin == 0)
			calcCrossCoef();
	}
	else if(gButtonFlag == 1){

	}
	else if(gButtonFlag == 2){
		if(aPin <= 2){
			calcParaCoef(0);
		}
		else{
			calcParaCoef(1);
		}

	}
	else if(gButtonFlag == 3){
		if(aPin <= 2){
			calcParaCoef(2);
		}
		else{
			calcParaCoef(3);
		}
	}

	//otherwise do nothing.

	/*  *****************************************************************
	 * 		Note regarding hardware controls
	 *  gAnalogData[chann][swMode] Mapping
	 *  Switch Mode 0 - Master Crossover, Gain,  Delay (Xover, G1, Dly1, G2, Dly2, Nothing)
	 *  Switch Mode 1 - Compressor (Threshold, Ratio, Nothing, Attack, Decay, Gain)
	 *  Switch Mode 2 - EQ High (Freq1, Q1, Gain1, Freq2, Q2, Gain2)
	 *  Switch Mode 3 - EQ Low  (Freq1, Q1, Gain1, Freq2, Q2, Gain2)
	 *  *****************************************************************
	 */
}


void cleanup(int numChannels) {
	// Free all memory allocation
	free(gxBuffer);
	free(gyLBuffer);
	free(gyHBuffer);
	free(xBufferPointer);
	free(yBufferPointer);

	for(int channel = 0; channel < numChannels; channel++){
		free(delayBuffer[channel]);
	}
	for(int i = 0; i < DELAY_SIZE; i++){
		free(paraXBuffer[i]);
		free(paraYBuffer[i]);
	}

	free(delayBuffer);
	free(paraXBuffer);
	free(paraYBuffer);
}
