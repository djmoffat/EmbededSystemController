


#include <stdio.h>
#include <pthread.h>
#include "sensor.h"
#include "SimpleGPIO.h"
extern int gShouldStop;
extern int gSampleRate;
int gButtonFlag = 0;
int gAnalogControl = 0;
float gControlValues[NUMANALOG][NUMSWITCHV];
int gLEDStatus = 0;
int medianData[MEDFILTERSIZE];
/*  *****************************************************************
 *  gAnalogData Mapping
 *  Switch Mode 0 - Master Crossover, Gain,  Delay (Xover, Q?, G1, Dly1, G2, Dly2)
 *  Switch Mode 1 - Compressor (Threshold, Ratio, Knee, Attack, Decay, Gain)
 *  Switch Mode 2 - EQ High (Freq1, Q1, Gain1, Freq2, Q2, Gain2)
 *  Switch Mode 3 - EQ Low  (Freq1, Q1, Gain1, Freq2, Q2, Gain2)
 *  *****************************************************************
 */


float gCrossoverFrequency = 1000;


//	function taken from http://www.tutorialspoint.com/c_standard_library/c_function_qsort.htm
int cmpfunc (const void * a, const void * b){
   return ( *(int*)a - *(int*)b );
}


int median(int* data, int size){
	qsort(data, size, sizeof(int), cmpfunc);
	int centerValue = floor(size/2);
	return data[centerValue];
}

int readAnalogPin(int pin){
	//read analog pins 3 times and take median
	for(int i = 0; i < MEDFILTERSIZE; i++)
		medianData[i] = analogRead(pin);
	int result = median(medianData, MEDFILTERSIZE)+1;
	return result;
}


void updateControlValues(int aPin){
//	printf("buttonFlag = %d, aPin = %d", gButtonFlag, aPin);
	if(gButtonFlag == 0){
	// Master Control Values
		if(aPin == 0)
			gControlValues[aPin][gButtonFlag] = ((6000 * (float) gAnalogData[aPin][gButtonFlag]/(float)ANALOGMAX)-10); 			// Crossover Frequency
		else if(aPin == 1 || aPin == 3)
			gControlValues[aPin][gButtonFlag] = ((2 * (float) gAnalogData[aPin][gButtonFlag]/(float)ANALOGMAX));					// L or R Channel Gain
		else if(aPin == 2 || aPin == 4)
			gControlValues[aPin][gButtonFlag] = (0.5 * gSampleRate * (float) gAnalogData[aPin][gButtonFlag]/(float)ANALOGMAX);	// L or R Channel Delay
	}
	else if(gButtonFlag == 1){
		// Compression
		if(aPin == 0)
			gControlValues[aPin][gButtonFlag] = -60 * ((float) gAnalogData[aPin][gButtonFlag]/(float)ANALOGMAX);										// Threshold
		else if(aPin == 1)
			gControlValues[aPin][gButtonFlag]  = 500 * (((float) gAnalogData[aPin][gButtonFlag] * (float) gAnalogData[aPin][gButtonFlag])/((float)ANALOGMAX * (float)ANALOGMAX)); // Ratio
		else if(aPin == 2)
			gControlValues[aPin][gButtonFlag] =  (float) gAnalogData[aPin][gButtonFlag]/(float)ANALOGMAX;												// Knee
		else if(aPin == 3)
			gControlValues[aPin][gButtonFlag] = exp(-1/(0.001 * gSampleRate * (((80 * (float) gAnalogData[aPin][gButtonFlag]/(float)ANALOGMAX)))));	// Attack Time
		else if(aPin == 4)
			gControlValues[aPin][gButtonFlag] = exp(-1/(0.001 * gSampleRate * (((80 * (float) gAnalogData[aPin][gButtonFlag]/(float)ANALOGMAX)))));	// Release Time
		else if(aPin == 5)
			gControlValues[aPin][gButtonFlag] = 40 * ((float) gAnalogData[aPin][gButtonFlag]/(float)ANALOGMAX);										// Make-up Gain
	}
	else if(gButtonFlag == 2 || gButtonFlag == 3){
		// Left Channel Parametric
		if(gButtonFlag == 2 && (aPin == 0 || aPin == 3 ))
			gControlValues[aPin][gButtonFlag] = ((gControlValues[0][0]* (float) gAnalogData[aPin][gButtonFlag]/(float)ANALOGMAX));		// Frequency
		else if(gButtonFlag == 3 && (aPin == 0 || aPin == 3 ))
			gControlValues[aPin][gButtonFlag]  = (((10000-gControlValues[0][0]) *(float) gAnalogData[aPin][gButtonFlag] /(float)ANALOGMAX))+gControlValues[0][0];	// Frequency
		else if(aPin == 1 || aPin == 4)
			gControlValues[aPin][gButtonFlag] = (10 * ((float) gAnalogData[aPin][gButtonFlag]/(float)ANALOGMAX));							// Quality Factor
		else if(aPin == 2 || aPin == 5){
			gControlValues[aPin][gButtonFlag] = ((12 * (float) gAnalogData[aPin][gButtonFlag]/(float)ANALOGMAX)) - 6;					// Gain

		}
	}
}


void *sensorLoop(void *data) {

	int debug = 0;				// Debug statement for debug printing
	int potentWindowRange = 10;

	int buttonPins[NUMBUTTONS] = {14, 15};
	int tempButtonStatus;
	int buttonStatus = 0;
	int buttonPrevStatus = 0;
	int buttonPrevPrevStatus = 0;

	int ledPins[NUMLEDS] = {31, 30, 48, 49, 3, 2};
	int analogPins[NUMANALOG]= {0,1,2,3,4,5};


	// Export and Set Directions of GPIO
	for(int bPin = 0; bPin < NUMBUTTONS; bPin++){
		//Export and set Buttons
		if(gpio_export(buttonPins[bPin])) {
			printf("An error occurred exporting Button %d \n", bPin);
		}
		if(gpio_set_dir(buttonPins[bPin], 0)) {
			printf("An error occurred with the direction of Button %d \n", bPin);
    	}

		if(gpio_get_value(buttonPins[bPin], &tempButtonStatus)) {
			printf("Unable to read input value from Button %d \n", bPin);
			return 1;
		}
		buttonStatus |= (1 << bPin);
	}

	for(int lPin = 0; lPin < NUMLEDS; lPin++){
		//Export and Set LEDs
		if(gpio_export(ledPins[lPin])) {
			printf("An error occurred exporting LED %d \n", lPin);
		}
	    if(gpio_set_dir(ledPins[lPin], 1)) {
	        printf("An error occurred with the direction of LED %d \n", lPin);
	    }
	}


	// TODO: Initialise gAnalogData to reasonable start values (Or save values? or start from terminal input? ect.)
	for(int aPin = 0; aPin < NUMANALOG; aPin++){
		// preallocate gAnalogData
	    for(int bIndex = 0; bIndex < NUMSWITCHV; bIndex++){
	    	if(bIndex == 0 && (aPin == 1 || aPin == 3))
	    		gAnalogData[aPin][bIndex] = (int)ANALOGMAX/2;
	    	else if((bIndex == 2 || bIndex == 3) && (aPin == 2 || aPin == 5))
	    		gAnalogData[aPin][bIndex] = (int)ANALOGMAX/2;
	    	else
	    		gAnalogData[aPin][bIndex] = 1;
	    	gButtonFlag = bIndex;
	    	updateControlValues(aPin);
	    }
	}
	gButtonFlag = buttonStatus;

	// Control Loop
	while(!gShouldStop) {

		// Read Buttons and Set gButtonFlag
		buttonPrevStatus = buttonStatus;
		buttonStatus = 0;
		for(int b = 0; b < NUMBUTTONS; b++){
			//read button value and look for difference
			if(gpio_get_value(buttonPins[b], &tempButtonStatus)) {
				printf("Unable to read input value from Button %d \n", b);
				return 1;
			}
			if(tempButtonStatus)
				buttonStatus |= (1 << b);
			else
				buttonStatus &= ~(1 << b);
		}
		// if Button status has changed, reset analog control and set flag to new value
		if ((buttonPrevStatus)	!= (buttonStatus)){
			gAnalogControl = 0;
			gButtonFlag = buttonStatus;
			printf("Button Status now %d \n", gButtonFlag);
		}
		// Read Analog Pins to gAnalogData[potentiometer][gButtonFlag]
//		printf("button = %d, ", gButtonFlag);
		for(int aPin = 0; aPin < NUMANALOG; aPin++){
			int temp = readAnalogPin(analogPins[aPin]);
			// if current potentiometer in control, update value of gAnalogData
//			printf(" analogIn = %d, temp = %d, gAnalogData = %d ", aPin, temp, gAnalogData[aPin][gButtonFlag]);
			if(gAnalogControl & (1<<aPin)){
				if((gAnalogData[aPin][gButtonFlag] != temp)){// && ((gAnalogData[aPin][gButtonFlag] - temp > 2)||(temp - gAnalogData[aPin][gButtonFlag] > 2))){
					gAnalogData[aPin][gButtonFlag] = temp;
					updateControlValues(aPin);
					updateHardwareControl(aPin);
				}

			}
			// If within window range of stored value, take control and update gAnalogControl
			else if((temp + potentWindowRange >= gAnalogData[aPin][gButtonFlag]) &&
					(temp - potentWindowRange <= gAnalogData[aPin][gButtonFlag]) ){
				gAnalogControl |= (1<<aPin);
				gAnalogData[aPin][gButtonFlag] = temp;
				updateControlValues(aPin);
				updateHardwareControl(aPin);
			}
		}
//		printf("\n");
//		printf("in setLLED Status loop\n");

		// Set Analog Values based on gLEDStatus, taken direct from gAnalogControl
		gLEDStatus = gAnalogControl;
		int ledStatus = 0;
		for(int led = 0; led < NUMLEDS; led++){
			if(gLEDStatus & (1<<led))
				ledStatus = 1;
			else
				ledStatus = 0;
			if(gpio_set_value(ledPins[led], ledStatus)) {
				printf("Unable to set LED %d to value %d \n", led, (gLEDStatus & (1<<led)));
			}
		}

//		printf("gAnalogData = %d, %d, %d, %d, %d, %d, %d \n", gButtonFlag,gAnalogData[0][gButtonFlag]
//		                                                               ,gAnalogData[1][gButtonFlag]
//		                                                               ,gAnalogData[2][gButtonFlag]
//		                                                               ,gAnalogData[3][gButtonFlag]
//		                                                               ,gAnalogData[4][gButtonFlag]
//		                                                               ,gAnalogData[5][gButtonFlag] );

		usleep(1000);	/* Wait 1ms to avoid checking too quickly */
	//End of control loop
	}


	// unexport LEDs
	for(int lPin = 0; lPin < NUMANALOG; lPin++){
		if(gpio_unexport(ledPins[lPin])) {
			printf("An error occurred unexporting LED %d \n", lPin);
			return 1;
		}
	}
	// unexport buttons
	for(int bPin = 0; bPin < NUMBUTTONS; bPin++){
		if(gpio_unexport(buttonPins[bPin])) {
			printf("An error occurred unexporting Button %d \n", bPin);
		}
	}

	//exit control thread
	pthread_exit(NULL); /* Don't change this */
}
