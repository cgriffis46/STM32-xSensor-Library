/*
 * anemometer.h
 *
 *  Created on: Mar 11, 2026
 *      Author: coryg
 */

#ifndef ANEMOMETER_H_
#define ANEMOMETER_H_

#include <stm32l4xx_hal.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

using namespace std;

namespace wxutil {

class wx_anemometer {

private:

int WX_ANEMOMETER_PIN;


// Instantiate the anemometer only once.

bool wx_begin;

//volatile unsigned long *wx_wind_counter_read;
//volatile unsigned long *wx_read_time;

//volatile unsigned long *wx_last_counter_update;



// wx_wind_samples[counter]
//
// a wind gust is defined as the wind speed
// averaged over a 3 second period. We will
// loop through a structure and consider only
// samples taken over the last 3 seconds. If
// the oldest sample is more than 3 seconds old,
// we will return 0.
//
// If we take more than the maximum samples
// (wind speed is incredibly fast) then we will
// average all samples in the structure.
//
// Add support code also to

// resetCounters() will clear all counters

void resetCounters();

/*
   getInstantWind() will update the last read and last counter variables.
   This means if outside programs call it, it could disrupt takeSample();
   Solution is to make it a private call.
*/

float getInstantWind();

//float gustSpeed();

//float getAverageSpeed();


// incrementCounter() will be the interrupt request attached to the reed switch.


public:

// constructor

wx_anemometer();
void begin();
void irq();
void takeSample();
float getAverageWind();
void main();

//static void incrementCounter();

// static void incrementCounter();

int WX_ANEMOMETER_MAX_SAMPLES;

//int WX_ANEMOMETER_MAX_SAMPLES;

//int WX_ANEMOMETER_MAX_SAMPLES;

//void readCounter();
//float getWindSpeed();
//float getGustSpeed();
//void getAverageSpeed();

void printSamples();
//void printCounters();
int getCounter();

};


} /* namespace wxutil */

#endif /* ANEMOMETER_H_ */
