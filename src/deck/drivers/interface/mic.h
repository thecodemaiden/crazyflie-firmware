/**
 * Read input from a microphone using DMA from the ADC
 * Then do an FFT to find the dominant frequency
 **/

#ifndef __MICDECK_H__
#define __MICDECK_H__

#define BUFFER_SIZE 2048
#define SAMPLE_RATE (48000)
#define ADC_SAMPLE_PERIOD ((84000000L/SAMPLE_RATE)-1)
#define ADC_OFFSET (2047.0f) //12-bit ADC
// we want to look from 12kHz up
#define START_BIN (BUFFER_SIZE >> 2)
#endif
