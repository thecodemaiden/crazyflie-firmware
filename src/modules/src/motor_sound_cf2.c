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

static bool isInit=false;
static uint16_t last_freq = 0;
static uint16_t motor_freq = 0;
static uint16_t last_chirpdF = 0;

//static float chirpHolder = 0;

static uint16_t chirpdF = 5000;
static bool go_chirp = false;
static uint8_t chirpCounter = 0;

# define MTR_TASK_INTERVAL 50
// How many 'ticks' we have in a chirp
// Task runs every MTR_TASK_INTERVAL ms
#define CHIRP_LENGTH (20)

static bool doing_chirp = false;

static xTimerHandle timer;

static void motorSoundTimer(xTimerHandle timer)
{
  static uint16_t waitCounter = 0;
  if (waitCounter < 100)  {
      waitCounter++;
  } else {
    if (!doing_chirp && go_chirp) {
      doing_chirp = true;
      chirpCounter = 0;
      last_chirpdF = chirpdF;
      go_chirp = false;
    } 
    if (doing_chirp) {
      if (chirpCounter > CHIRP_LENGTH) {
        doing_chirp = false;
        motor_freq = 0;
      } else {
	motor_freq = 10000+ 100*chirpCounter;//((uint16_t) chirpHolder);
        chirpCounter += 1;
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

  timer = xTimerCreate("MtrSndT", M2T(MTR_TASK_INTERVAL), pdTRUE, NULL, motorSoundTimer);
  isInit = (timer != 0);
  xTimerStart(timer, 100);

}

bool motorSoundTest(void)
{
  return isInit;
}

PARAM_GROUP_START(chirp)
PARAM_ADD(PARAM_UINT16, f1, &motor_freq)
PARAM_ADD(PARAM_UINT16, chLen, &chirpdF)
PARAM_ADD(PARAM_UINT8, goChirp, &go_chirp)
PARAM_GROUP_STOP(chirp)
