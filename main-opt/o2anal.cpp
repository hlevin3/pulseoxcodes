/* o2anal.cpp
 * This is the analysis code from Daresh slighlty repackaged
 * as a function.  The text of the code is mostly exactly the same.
 * Signature:
 * bool maybe_get_pulseOxy(float *heartrate, float* SaO2value, 
 *      int *bufstatus, long redDatum, long irDatum, FLOAT alpha)
 * where:
 *    return value is true if new values of heartrate, SaO2value are found
 *    bufstatus is always set:
 *        bufstatus = 0 ; the irdiff buffer is being used
 *        bufstatus = 1 ; the irdiff buffer is full
 *        bufstatus = 2 ; the irdiff buffer is was initialized
 *        bufstatus = 3 ; the irdiff buffer is being filled
 * (this was all so the caller could print the same status messages that
 * the original code did)
 * 
 * alpha= scaling factor for R
 * 
*/

#include <Arduino.h>
#include <float.h>
#include <CircularBuffer.h>
#include <KickSort.h>

#include "o2anal.h"

// ******** COEFFICIENTS FOR COMPUTATION of SaO2 from R
#define E_HB_RED 6510   // Extinction coefficient of red channel for Hb
#define E_HB_IR 602     // Extinction coefficient of ir channel for Hb
#define E_HBO2_RED 942  // Extinction coefficient of red channel for HbO2
#define E_HBO2_IR 1204  // Extinction coefficient of ir channel for HbO2

// ******* Templates for functions defined later in this file
template<typename T>
float median(T* data, byte dataLength);

template<typename T, size_t LENGTH>
float calculateMean(CircularBuffer<T, LENGTH> &data);

// ******** Globals from original analysis code (could be static here)

// **+** PARAMETERS  ******
/* Size of the buffer for incoming red and IR data. */
const byte BUFFER_SIZE = 63;

/* Window size for data calculated during each heart beat event. */
const byte BEATS_WINDOW_SIZE = 10;

/** Buffer to store signal differentials for the IR channel. */
CircularBuffer<int, BUFFER_SIZE> irDiffs;

/** Buffer to store signal differentials for the red channel. */
CircularBuffer<int, BUFFER_SIZE> redDiffs;

/** Buffer storing most recent SaO2 calculations with each heart beat. */
CircularBuffer<float, BEATS_WINDOW_SIZE> SaO2s;

/** Buffer storing most recent heart rate calculations each heart beat. */
CircularBuffer<int, BEATS_WINDOW_SIZE> beatIntervals;

/** 
 * Most recent red and IR channel raw signals. 
 * These are populated from the sensor on each read.
 */
uint32_t irDatum, redDatum = 0;

/** Red and IR channel raw signals captured during the previous read. */
uint32_t irLastDatum, redLastDatum = 0;

/** 
 *  Aggregated differentials between consecutive reads whose differentials
 *  have the same mathematical sign. This is used to track the total
 *  change in signal between inflections in the data.
 */
int aggIRDiff, aggRedDiff = 0;

/** Timestap for the most recent data point. */
unsigned long timestamp = 0;

/**
 * The timestamp associated with the last change in sign for the derivative
 * of the IR signal.
 */
unsigned long lastTimestamp = 0;

/**
 * Elapsed time in milliseconds since the last heart beat.
 */
unsigned int lastBeat = 0;

/** Flag indicating whether a heart beat event has been triggered. */
bool beatEvent = false;

/** 
 *  Intensity ratios for the red and IR channels, which are calculated
 *  independently in time based on when each signal reaches a local
 *  minimum after a heart beat event has been identified.
 */
float redIntensityRatio, irIntensityRatio = 0;

/** 
 * Flags indicating whether the intensity ratios for the red and IR 
 * channels have been calculated and captured. Once both have been captured,
 * the values are used to calculate the total signal ratio and SpO2.
 */
bool redBeatCaptured, irBeatCaptured = false;


/* bool maybe_get_pulseOxy(float *heartrate, float *SaO2value, int *bufstatus,
                           long redDatum, long irDatum, float alpha)
    returns true if red and ir beats are detected and also returns the 
    associated heartrate and SaO2 values.
    returns false if no new estimate of HR and SPO2 are available this step

    possible returned bufstatus values are as follows:
        0: irdiff buffer is being used normally
        1: the irdiff buffer is full
        2: the irdiff buffer was just initialized
        3: the irdiff buffer is being filled
    (The purpose of the bufstatus variable was to help development 
    and testing of the code)
  Inputs:
    *heartrate is a pointer to the location to store the heartrate
    *SaO2value is a pointer to the location to store the SaO2 value
    redDatum is the integer ADC reading for the red light signal
    irDataun is the integer ADC reading for the infrared light signal
    (note that either red or ir signals can be background subtracted)
    alpha = scaling factor for R
*/
bool maybe_get_pulseOxy(float *heartrate, float* SaO2value, int *bufstatus,
                        long redDatum, long irDatum, float alpha){
    bool success; // return value

  timestamp = millis();

  int redDiff = redDatum - redLastDatum;
  int irDiff = irDatum - irLastDatum;

  redDiffs.push(redDiff);
  irDiffs.push(irDiff);

  // Check if signs of differences match the aggregate differences. If they
  // match, then continue to add to the aggregate. If they do not, this
  // represents an inflection in the data, and the aggregate values should
  // be reset. An inflection after a beat event has been flagged also signals
  // the end of the heart beat, in which case the relative change in signal
  // during the heart beat event is calculated and stored. This is determined
  // for the red and IR channels independently in time, as one can lag behind
  // the other.
  if ((redDiff >= 0) != (aggRedDiff >= 0)) {
    if (beatEvent && !redBeatCaptured) {
      redIntensityRatio = (float) aggRedDiff / redLastDatum;
      redBeatCaptured = true;     
    }
    
    aggRedDiff = redDiff;
  } else {
    aggRedDiff += redDiff;
  }

  if ((irDiff >= 0) != (aggIRDiff >= 0)) {
    if (beatEvent && !irBeatCaptured) {
      irIntensityRatio = (float) aggIRDiff / irLastDatum;
      irBeatCaptured = true;

      // Push the elapsed time between this beat and the last and then 
      // update the timestamp for the last beat. The IR channel was
      // chosen to track heart beats as the change in signal is greater
      // compared to the red channel.
      beatIntervals.push(lastTimestamp - lastBeat);
      lastBeat = lastTimestamp;
    }
    
    aggIRDiff = irDiff;
    lastTimestamp = timestamp;
  } else {
    aggIRDiff += irDiff;
  }


  // Check to see if beat data for both the red and IR channels are available
  // and calculate SpO2, if so.
  success = redBeatCaptured && irBeatCaptured;
  if (success) {
    float intensityRatio = redIntensityRatio / irIntensityRatio;
    float SaO2 = (E_HB_RED - (E_HB_IR * alpha*intensityRatio)) / ((E_HB_RED - E_HBO2_RED) - (E_HB_IR - E_HBO2_IR) *alpha*intensityRatio);
    
    SaO2s.push(SaO2);

    float meanSaO2 = calculateMean(SaO2s);
    float meanHeartRate = 60.0 / (calculateMean(beatIntervals) / 1000);

    *SaO2value = meanSaO2;
    *heartrate = meanHeartRate;
    
    beatEvent = false;
    redBeatCaptured = false;
    irBeatCaptured = false;
  }

  // Calculate the average and stdev for the current window of data once the
  // buffer is full. This is used to determine whether the most recent change
  // in signal is statistically significant to trigger a heart beat event.
  // The IR channel is chosen for this purpose as the signal has a larger  
  // range compared to the red channel.
  float mean = 0;
  float stdev = 0;

  *bufstatus = 0; // ordinary operation
  if(irDiffs.available() == 0) {  // ie no available slots in irDiffs
    int dataLength = irDiffs.size();
    for(int index = 0;index < dataLength; index++) {
      mean += (float)(irDiffs[index]) / dataLength;
    }

    float variance = 0;
    for(int index = 0;index < dataLength; index++) {
      variance += pow(irDiffs[index] - mean, 2);
    }

    stdev = sqrt(variance / dataLength);

    *bufstatus = 1; //buffer full
  }
  else if(irDiffs.size() == 1) {  // else if there is only 1 entry in irDiffs
      *bufstatus=2; // starting to collect data
  }
  else {
      *bufstatus=3; // filling the buffer (but not yet full)
  }

  // Determine if last data point qualifies as a heart beat event.
  // A heart beat is identified whenever the change in signal is negative
  // and greater than two standard deviations away from the mean change
  // across the current window of data.
  float threshold = stdev * -2;
  if (stdev != 0 && irDiff < 0 && irDiff < threshold) {
    beatEvent = true;
    }  
  redLastDatum = redDatum;
  irLastDatum = irDatum; 

  return success;
}


/** 
 * Calculate the median of a dataset. 
 * 
 * @param data        A pointer to the beginning of the dataset array.
 * @param dataLength  The size of the data set.
 * 
 * @returns   The median of the data set.
 */
template<typename T>
float median(T* data, byte dataLength) {
  if (dataLength % 2 == 0) {
    return (data[dataLength / 2 - 1] + data[dataLength / 2]) / 2.0;
  }

  return data[dataLength / 2];
}

/**
 * Calculate the mean for the dataset using only values that do not
 * qualify as outliers. Outliers are considered values that fall
 * outside the Tukey fences (1.5 * IQR above and below the 3rd and 
 * 1st quartiles, respectively).
 * 
 * @param data    The dataset to analyze as CircularBuffer.
 * 
 * @returns   The mean value of the dataset, excluding outliers.
 */
template<typename T, size_t LENGTH>
float calculateMean(CircularBuffer<T, LENGTH> &data) {
  T *sortedData = (T*) malloc(sizeof(data[0] * LENGTH));
  for (unsigned int index = 0; index < LENGTH; index++) {
    sortedData[index] = data[index]; 
  }

  KickSort<T>::quickSort(sortedData, LENGTH, KickSort_Dir::ASCENDING);
  float Q1 = median(sortedData, LENGTH / 2);
  float Q3 = median(sortedData + (int) ceil(LENGTH / 2.0), LENGTH / 2);
  float IQR = Q3 - Q1;
  
  float lowerLimit = Q1 - (1.5 * IQR);
  float upperLimit = Q3 + (1.5 * IQR);

  int validPoints = 0;
  float sum = 0;
  for (unsigned int index = 0; index < LENGTH; index++) {
    if (data[index] >= lowerLimit && data[index] <= upperLimit) {
      sum += data[index];
      validPoints++;
    }
  }
  
  free(sortedData);

  return sum / validPoints;
}

/**
 * Algorithm: (original form) -- OLD COMMENT (RCG)
 *  1. Keep track of differences back [2.5s - depends on sampling rate] and last absolute value
 *  2. Wait until buffer is full
 *  3. Analyze buffer for stdev
 *  4. If latest values triggers threshold, set flag
 *  5. Continue to read differences until it becomes positive
 *      a. Use last absolute value as baseline
 *      b. Move backwards in buffer and add all negative consecutive values
 *      c. Calculate delta/baseline for channel
 *  6. Check if delta/baseline for both channels are set, and if so:
 *      a. Calculate SpO2 and add to SpO2 window
 *      b. Reset both to 0
 *
 */
