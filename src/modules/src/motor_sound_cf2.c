/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
 * This file created by Adeola Bannis, Carnegie Mellon University
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * sound_motors_cf2.c - Module used to play melodies and system sounds though a buzzer
 */

#include <stdbool.h>

/* FreeRtos includes */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "timers.h"

/* Crazyflie driver includes */
#include "config.h"
#include "param.h"
#include "log.h"
#include "motor_sound.h"
#include "motors.h"

// For Ubicomp, we only modulate the signal from 2 motors at once
// Either motors M2 and M3 together (same timer) or motors M1 and
// M4 separately


#define DEFAULT_FREQ 11000
#define CHIRP_LENGTH (20)


static bool isInit=false;
static bool monoMode = true;

static uint16_t lastF1 = 0;
static uint16_t lastF2 = 0;

static uint16_t motorF1 = 0;
static uint16_t motorF2 = 0;

static uint16_t chirpF1 = DEFAULT_FREQ;
static uint16_t chirpF2 = DEFAULT_FREQ;

// we never change the destination frequency in the middle of chirping
static uint16_t lastChirpF1 = 0;
static uint16_t lastChirpF2 = 0;
static uint16_t chirpDf1 = 0;
static uint16_t chirpDf2 = 0;
static bool wasMono = true;

static uint16_t chirpLenMs = 1000;
static uint16_t lastChpLen = 1000;
static int8_t chirpTicks = CHIRP_LENGTH;

static bool go_chirp = false;
static bool doing_chirp = false;
static uint8_t chirpCounter = 0;

# define MTR_TASK_INTERVAL 50
// How many 'ticks' we have in a chirp
// Task runs every MTR_TASK_INTERVAL ms


static xTimerHandle timer;

static void motorSoundTimer(xTimerHandle timer)
{
  static uint16_t waitCounter = 0;
  uint32_t accum; // just a temp variable
  if (waitCounter < 100)  {
      waitCounter++;
      return;
  }

  // did we change the chirp length?
  if (chirpLenMs != lastChpLen) {
    chirpTicks = (int)((chirpLenMs+MTR_TASK_INTERVAL-1)/(MTR_TASK_INTERVAL));
    lastChpLen = chirpLenMs;
    if (chirpTicks < 5) chirpTicks = 5;
  } 

  if (!doing_chirp && go_chirp) {
    doing_chirp = true;
    go_chirp = false;
    
    chirpCounter = chirpTicks;

    if (chirpF1 == 0) chirpF1 = DEFAULT_FREQ;
    if (chirpF2 == 0) chirpF2 = DEFAULT_FREQ;

    if (motorF1 == 0) motorF1 = chirpF1 - 2000;
    if (motorF2 == 0) motorF2 = chirpF2 - 2000;

    lastChirpF1 = chirpF1; lastChirpF2 = chirpF2;
    chirpDf1 = chirpF1 - motorF1; chirpDf2 = chirpF2 - motorF2;
    wasMono = monoMode;
  }

  if (doing_chirp) {
     accum = (chirpCounter*chirpDf1)/chirpTicks;
     motorF1 = lastChirpF1 - accum;
     if (!wasMono) {
        accum = (chirpCounter*chirpDf2)/chirpTicks;
        motorF2 = lastChirpF2 - accum;
     }
     chirpCounter -= 1;
     if (chirpCounter == 0) {
       doing_chirp = false;
     }
  }

  // whether we are chirping or not, update the motors
    // changing either of the linked motors' frquency changes them both
   if (lastF1 != motorF1) {
     if (monoMode)
       motorsSetFrequency(1, motorF1);
     else
       motorsSetFrequency(0, motorF1);

      lastF1 = motorF1;
    }

   if (!monoMode && lastF2 != motorF2) {
     motorsSetFrequency(3, motorF2);
     lastF2 = motorF2;
   }

}

void motorSoundInit(void)
{
  if (isInit) {
    return;
  }

  timer = xTimerCreate("MtrSndT", M2T(MTR_TASK_INTERVAL), pdTRUE, NULL, motorSoundTimer);
  isInit = (timer != 0);
  xTimerStart(timer, 100);

}

bool motorSoundTest(void)
{
  return isInit;
}

PARAM_GROUP_START(mtrsnd)
PARAM_ADD(PARAM_UINT16, f1, &motorF1)
PARAM_ADD(PARAM_UINT16, f2, &motorF2)
PARAM_ADD(PARAM_UINT16, chpF1, &chirpF1)
PARAM_ADD(PARAM_UINT16, chpF2, &chirpF2)
PARAM_ADD(PARAM_UINT16, chpLen, &chirpLenMs)

PARAM_ADD(PARAM_UINT8, mono, &monoMode)
PARAM_ADD(PARAM_UINT8, goChirp, &go_chirp)
PARAM_GROUP_STOP(mtrsnd)
