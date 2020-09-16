/**
 * Copied from sound_motors_cf2.c with heavy modification
 **/
#include <stdbool.h>

/* FreeRtos includes */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "timers.h"
#include "worker.h"

/* Crazyflie driver includes */
#include "config.h"
#include "param.h"
#include "log.h"
#include "motor_sound.h"
#include "motors.h"
// TODO: convert all variables to camelCase

#define MSG_LENGTH 4
// how many task ticks to wait between chirp symbols
#define PAUSE_LENGTH 2 
static struct {
  uint16_t topF;
  uint16_t bottomF;
  uint16_t dF;
  uint8_t message;
  uint8_t msgCounter;
} _msgParams;

MotorSoundParameters _chirpParams = {16000, 500, 2000};

static bool isInit=false;
static uint8_t message = 0xaa;

static bool requestChirp = false;
static uint16_t startFreq = 0;
static uint16_t endFreq = 0;
static int16_t fStep = 0;
static uint16_t lastFreq = 0;
static uint16_t motorFreq = 0;

static bool inPause = false;
static uint8_t pauseTicks = 0;

# define SND_TASK_INTERVAL 50

static bool doingMsg = false;

static xTimerHandle timer;

static void setupNextChirp()
{
  // we don't check msgCounter here........
  pauseTicks = 0;
  bool isDownChirp = !(_msgParams.message & (1 << _msgParams.msgCounter));
  if (isDownChirp){
    startFreq = _msgParams.topF;
    endFreq = _msgParams.bottomF;
    fStep = -_msgParams.dF;
  } else {
    // upchirp
    startFreq = _msgParams.bottomF;
    endFreq = _msgParams.topF;
    fStep = _msgParams.dF;
  }
  motorFreq = startFreq;
  _msgParams.msgCounter += 1;
}

static void setupMessage()
{
  int16_t totalFChange = (int16_t)((float)_chirpParams.chirpSlope * _chirpParams.chirpLen / 1000.0f);
  _msgParams.dF = (int16_t)((float)_chirpParams.chirpSlope * SND_TASK_INTERVAL / 1000.0f);
  _msgParams.bottomF = _chirpParams.centerFreq - totalFChange/2;
  _msgParams.topF = _chirpParams.centerFreq + totalFChange/2;
  _msgParams.message = message;
  _msgParams.msgCounter = 0;
  inPause = false;
}

static void updateChirpFreq()
 {

  if (!doingMsg && requestChirp) {
    doingMsg = true;
    requestChirp = false;

    setupMessage();
    setupNextChirp();
  } 

  if (motorFreq != lastFreq) {
#if ALL_MOTOR_CHIRP
#pragma message "Using all motors to chirp"
    for (int ii=0; ii<4; ii++) motorsSetFrequency(ii,motorFreq);
#else
#pragma message "Using M4 only for chirp"
      motorsSetFrequency(3, motorFreq); 
#endif
      lastFreq = motorFreq;
  }

  if (doingMsg) {
    if (inPause) {
      pauseTicks += 1;
      if (pauseTicks > PAUSE_LENGTH) {
        inPause = false;
	setupNextChirp();
      } else {
        return; // don't do anything if we were pausing
      }
    }
    // we were not pausing, see if we're done with this symbol
    bool finished = (fStep < 0 && motorFreq < endFreq) || (fStep > 0 && motorFreq > endFreq);
    if (finished) { // symbol done
      motorFreq = 0;
      if (_msgParams.msgCounter >= MSG_LENGTH) {
        // we have finished all the symbols in the message
        doingMsg = false;
      } else {
        // we just have to wait a few ticks to the next symbol
        inPause = true;
      }
    } else {
      // we weren't done with the symbol
      motorFreq += fStep;
    }
  }
 }

static void motorSoundTimer(xTimerHandle timer)
{
  workerSchedule(updateChirpFreq, NULL);
}

void motorSoundInit(void)
{
  if (isInit) {
    return;
  }

  timer = xTimerCreate("ChpTask", M2T(SND_TASK_INTERVAL), pdTRUE, NULL, motorSoundTimer);
  isInit = (timer != 0);
  xTimerStart(timer, 100);

}

bool motorSoundTest(void)
{
  return isInit;
}

const MotorSoundParameters *currentMotorParams()
{
  return &_chirpParams;
}
PARAM_GROUP_START(chirp)
PARAM_ADD(PARAM_UINT16, length, &(_chirpParams.chirpLen))
PARAM_ADD(PARAM_UINT16, center, &(_chirpParams.centerFreq))
PARAM_ADD(PARAM_INT16, slope, &(_chirpParams.chirpSlope))
PARAM_ADD(PARAM_UINT8, message, &message)
PARAM_ADD(PARAM_UINT8, goChirp, &requestChirp)
PARAM_GROUP_STOP(chirp)
LOG_GROUP_START(chirp)
LOG_ADD(LOG_UINT16, freq, &motorFreq)
LOG_ADD(LOG_INT16, dF, &fStep)
LOG_GROUP_STOP(chirp)
