
#ifndef SENSOR_H_
#define SENSOR_H_

#ifndef		M_PI
#define		M_PI		3.14159265358979323846264338
#endif
#define NUMBUTTONS 2
#define NUMSWITCHV 4
#define NUMLEDS 6
#define NUMANALOG 6
#define MEDFILTERSIZE 3
#define ANALOGMAX 1800


int gAnalogData[NUMANALOG][NUMSWITCHV];


void *sensorLoop(void *data);
void updateControlValues(int aPin);

int analogRead(int input);
int median(int* data, int size);
int readAnalogPin(int pin);
int cmpfunc (const void * a, const void * b);


/* Get current time since start of program (if needed) */
double getCurrentTime(void);


#endif /* SENSOR_H_ */
