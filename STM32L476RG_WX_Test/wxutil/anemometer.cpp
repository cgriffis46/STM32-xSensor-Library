#include <anemometer.h>

namespace wxutil {


/*
	By: Cory Griffis
	03/12/2017
*/

//#include <Arduino.h>
//#include "wx_anemometer.h"

/*
	This code will connect to a Fine-Offset reed-switch based
	Anemometer and compute wind speed.

	Interrupt pin will increment a counter each time the reed
	switch closes.

	By: Cory Griffis

*/

//int counter;

wx_anemometer::wx_anemometer() {
	wx_begin = false;
}

// setup here.

//WX_ANEMOMETER_PIN = wx_wind_pin;

typedef struct {
	uint32_t sample_time;
	float sample_speed;

} wx_wind_sample;

wx_wind_sample wx_wind_samples[10];

//std::list<wx_wind_sample> wx_wind_samples;

//	wx_wind_counter will be the counter we increment
//	according to the interrupt.

//	volatile unsigned long wx_wind_counter;

static volatile int wx_wind_counter;

//	wx_last_read_time will be used to ensure 3000 millis
//	between read times.

//	volatile unsigned long wx_wind_read_time;

static volatile uint32_t wx_wind_last_update;
static volatile uint32_t wx_wind_last_counter;
static volatile uint32_t wx_wind_irq_time;

// sample_counter will be the index counter for the array.

volatile int sample_counter;

void wx_anemometer::irq(void)
{
	//Serial.println("Increment");
	uint32_t current_time = HAL_GetTick();
	if(current_time - wx_wind_irq_time > 10){
		wx_wind_counter++;
		wx_wind_irq_time = current_time;
	}
	takeSample();
}

/*

	The Anemometer will pulse at 1.492 MPH Per Pulse Per Second.
	Calculate Wind Speed by calculating the time between pulses.

	seconds/pulse can be calculated by (Most Recent Pulse Time - Last Pulse Time)
	pulses/second will be 1/(Most Recent Pulse Time - Last Pulse Time)
	Instant MPH will be 1.492 * 1/(Most Recent Pulse Time - Last Pulse Time)

	Average wind speed can be calculated,
	1.492 * (sample pulses)/(sample time)

	Average speed is a bit trickier because we would need to track each pulse
	and only factor in the pulses inside our sample time.

*/

float wx_anemometer::getInstantWind(){

unsigned long current_time = HAL_GetTick();
unsigned long sample_period = current_time - wx_wind_last_update;
unsigned long sample_counts = wx_wind_counter - wx_wind_last_counter;
float current_speed = 0;

if(sample_period>0){
	current_speed = (((float)sample_counts)/((float)sample_period/1000.0))*1.492;
}

	wx_wind_last_update = current_time;
	wx_wind_last_counter = wx_wind_counter;

	return current_speed;

}

/*
	takeSample() will be used to call getInstantWind() and setup
	the wx_wind_samples[] structure.
*/

void wx_anemometer::takeSample()
{
	if(sample_counter+1>=10)
	{
		sample_counter = 0;
	}
	else
	{
		sample_counter++;
	}

	wx_wind_samples[sample_counter].sample_time = HAL_GetTick();
	wx_wind_samples[sample_counter].sample_speed = getInstantWind();

}

/*

getAverageWind() will loop through the wx_wind_samples[] array
and calculate an average wind speed from the last 3 seconds.

*/

float wx_anemometer::getAverageWind()
{

	float avg_wind_speed = 0;
	uint32_t current_time = HAL_GetTick();
	int samples_for_avg = 0;

for (int counter = 0; counter<10; counter++){
	if(current_time-wx_wind_samples[counter].sample_time<3000){
		samples_for_avg++;
		avg_wind_speed+=wx_wind_samples[counter].sample_speed;
	}
}
if(samples_for_avg>0){
	avg_wind_speed = avg_wind_speed/((float)samples_for_avg);
}
else
{
	avg_wind_speed = 0.00;
}
	return avg_wind_speed;
}

void wx_anemometer::begin(){
if(wx_begin == false){
	wx_begin = true;
	resetCounters();
}
}


/*
	------------------------------------------------------------
	printCounters() will be a function for testing and debugging
	the sample index.
	------------------------------------------------------------
*/

void wx_anemometer::printSamples()
{

//void wx_anemometer::printCounters(){

//Serial.print("Counter = ");
//Serial.println(wx_wind_counter);

//Serial.print("Last Update = ");
//Serial.println(wx_wind_last_update);

//Serial.println(wx_wind_read_time);

//Serial.println("Samples:");

// This section will loop through wx_wind_samples and
// print each value for testing and debugging

for(int counter = 0; counter<10; counter++){

//Serial.print("Sample Read Time = ");
//Serial.println(wx_wind_samples[counter].sample_time);

//Serial.print("Sample Speed = ");
//Serial.println(wx_wind_samples[counter].sample_speed);

}

}

int wx_anemometer::getCounter()
{
		return wx_wind_counter;
}

void wx_anemometer::resetCounters(){

// Clear values in the wx_wind_sample struct.
for (int counter = 0; counter<=10; counter++)
{
	wx_wind_samples[counter].sample_time = 0;
	wx_wind_samples[counter].sample_speed = 0;
}

// set the counter and the last update time.

wx_wind_irq_time = HAL_GetTick();
wx_wind_last_update = HAL_GetTick();

sample_counter = 0;

wx_wind_last_counter = 0;
wx_wind_counter = 0;

}


} /* namespace wxutil */
