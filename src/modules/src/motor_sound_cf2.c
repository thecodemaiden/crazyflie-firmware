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
#include "music.h"

static bool isInit=false;
static uint16_t last_freq[4] = {0};
static uint16_t motor_freq[4] = {PROP_FREQ, PROP_FREQ, PROP_FREQ, PROP_FREQ};
static bool sound_on;

#ifdef PLAY_MUSIC

static uint32_t m_ctr[4] = {0,0,0,0};
static const Melody *m_ptr[4] = {NULL, NULL, NULL, NULL};
static uint32_t m_idx[4] = {0,0,0,0};


static void melodyplayer(uint32_t * mi, Melody * m);

static Melody mary = {.bpm = 60, .delay=1, .notes={{B5, Q}, {A5, Q}, {G5, Q}, {A5, Q},
    {B5, Q}, {B5, Q}, {B5, H}, {A5, Q}, {A5, Q}, {A5, H}, {B5, Q}, {B5, Q}, {B5, H},  
    REPEAT}}; 

static Melody valkyries = {.bpm = 140, .delay = 1, .notes = {{Gb5, Q}, {B5, Q},
    {Gb5, S}, {B5, E},  {D6, Q}, {B5, Q}, {D6, Q}, {B5, S}, {D6, E}, {Gb6, Q},  
    {D6, Q}, {Gb6, Q}, {D6, S}, {Gb6, E}, {A6, Q}, {A5, Q}, {D6, Q}, {A5, S},  
    {D6, E}, {Gb6, H}, 
    REPEAT}};

#define ULTRA_FREQ 15000
#define INFRA_FREQ 20

static Melody ultra = {.bpm = 60, .delay=1, .notes={{ULTRA_FREQ, Q}, {OFF, H}, {ULTRA_FREQ, Q}, {OFF, Q}, REPEAT}};
static Melody infra = {.bpm = 60, .delay=1, .notes={{INFRA_FREQ, Q}, {OFF, H}, {INFRA_FREQ, Q}, {OFF, Q}, REPEAT}};
#endif


static xTimerHandle timer;

#ifdef SINGLE_MOTOR
static void motorSoundOff(uint32_t motorNum)
{
  motorsBeep(motorNum, false, 0);
}

static void motorSoundTone(uint32_t motorNum, uint16_t frequency)
{
  motorsBeep(motorNum, true, frequency);
}
#endif

static void motorSoundTimer(xTimerHandle timer)
{
  static uint16_t waitCounter = 0;
  if (waitCounter < 100)  {
      waitCounter++;
  } else {
    //uint8_t i = 0;
    //for (; i<4; i++){
	  if (motor_freq[0] != last_freq[0]) {
#define MFREQ_SET_ONE
#ifdef MFREQ_SET_ONE
	    motorsSetFrequency(3, motor_freq[0]);
#else
	    motorsSetAllFrequency(motor_freq[0]); 
#endif
	    last_freq[0] = motor_freq[0];
      }
    //}
  }
}

void motorSoundInit(void)
{
  if (isInit) {
    return;
  }

#ifdef PLAY_MUSIC
  m_ptr[0] = &ultra;
#endif

  timer = xTimerCreate("MtrSndT", M2T(50), pdTRUE, NULL, motorSoundTimer);
  isInit = (timer != 0);
  xTimerStart(timer, 100);

}

bool motorSoundTest(void)
{
  return isInit;
}

#ifdef PLAY_MUSIC
static void melodyplayer(uint32_t * mi, Melody * m) {
  uint16_t tone = m->notes[(*mi)].tone;
  uint16_t duration = m->notes[(*mi)].duration;

  if (m_ctr[0] == 0) {
    if (tone == 0xFE) {
      // Turn off buzzer since we're at the end
      (*mi) = 0;
    } else if (tone == 0xFF) {
      // Loop the melody
      (*mi) = 0;
    } else {
      // Play current note
      motorSoundTone(MOTOR0, tone); 
      m_ctr[0] = (20 * 4 * 60) / (m->bpm * duration) - 1;
      (*mi)++;
    }
  } else {
    if (m_ctr[0] == 1) {
      motorSoundOff(MOTOR0);
    }
    m_ctr[0]--;
  }
}
#endif

/*

static uint16_t siren_start = 2000;
static uint16_t siren_freq = 2000;
static uint16_t siren_stop = 4000;
static int16_t siren_step = 40;
static void siren(uint32_t counter, uint32_t * mi, Melody * melody)
{
  siren_freq += siren_step;
  if (siren_freq > siren_stop) {
    siren_step *= -1;
    siren_freq = siren_stop;
  }
  if (siren_freq < siren_start) {
    siren_step *= -1;
    siren_freq = siren_start;
  }
  buzzerOn(siren_freq);
}

static int pitchid;
static int rollid;
static int pitch;
static int roll;
static int tilt_freq;
static int tilt_ratio;
static void tilt(uint32_t counter, uint32_t * mi, Melody * melody)
{
  pitchid = logGetVarId("stabilizer", "pitch");
  rollid = logGetVarId("stabilizer", "roll");

  pitch = logGetInt(pitchid);
  roll = logGetInt(rollid);
  tilt_freq = 0;
  tilt_ratio = 127;

  if (abs(pitch) > 5) {
    tilt_freq = 3000 - 50 * pitch;
  }

  buzzerOn(tilt_freq);
}

*/

PARAM_GROUP_START(mtrsnd)
PARAM_ADD(PARAM_UINT16, freq1, (motor_freq))
PARAM_ADD(PARAM_UINT16, freq2, (motor_freq+1))
PARAM_ADD(PARAM_UINT16, freq3, (motor_freq+2))
PARAM_ADD(PARAM_UINT16, freq4, (motor_freq+3))
PARAM_ADD(PARAM_UINT8, enable, &sound_on)
PARAM_GROUP_STOP(mtrsnd)
