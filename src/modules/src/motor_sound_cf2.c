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

#define SND_TASK_INTERVAL 50 // milliseconds
static bool isInit=false;
static uint16_t last_freq = 0;
static uint16_t motor_freq = 0;
static float chirpdF = 0;
static uint16_t startF = 0;
static uint16_t endF = 0;
static bool inChirp = false;

/**
 * Main chirp parameters
 */
static uint16_t chirpSlope = 5000; // Hz/s
static uint16_t chirpLen = 1000; // in milliseconds
static uint16_t chirpCenter = 15000; // Hz
//*********
static bool requestChirp = false;




static xTimerHandle timer;

static void motorSoundTimer(xTimerHandle timer)
{
  static uint16_t waitCounter = 0;
  if (waitCounter < 100)  {
      waitCounter++;
  } else {
    if (!inChirp && requestChirp) {
      inChirp = true;
      requestChirp = false;
      // collect the parameters and let's goooo
      float totalFChange = ((float)chirpLen*chirpSlope/1000.0f); // the total chirp bandwidth
      uint16_t chirpTicks = ((chirpLen+SND_TASK_INTERVAL-1)/SND_TASK_INTERVAL); // err on the side of more ticks per chirp
      chirpdF = totalFChange/chirpTicks; // how much the frequency changes each task tick
      startF = chirpCenter - (totalFChange/2);
      endF = chirpCenter + (totalFChange/2);
      motor_freq = startF;
    } else 
      if (inChirp) {
        if (motor_freq >= endF) {
          motor_freq = 0;
          inChirp = false;
        } else { 
          motor_freq = (uint16_t)(motor_freq + chirpdF);
        }
      }


    if (motor_freq != last_freq) {
      motorsSetFrequency(motor_freq); 
      last_freq = motor_freq;
    }
  }   
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

PARAM_GROUP_START(mtrsnd)
PARAM_ADD(PARAM_UINT16, centerF, &chirpCenter)
PARAM_ADD(PARAM_UINT16, chpLen, &chirpLen)
PARAM_ADD(PARAM_UINT16, chpSlope, &chirpSlope)
PARAM_ADD(PARAM_UINT8, doChirp, &requestChirp)
PARAM_GROUP_STOP(mtrsnd)
