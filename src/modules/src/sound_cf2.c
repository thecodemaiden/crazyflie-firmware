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
 * sound_cf2.c - Module used to play melodies and system sounds though a buzzer
 */

#include <stdbool.h>

/* FreeRtos includes */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "timers.h"

#include "config.h"
#include "param.h"
#include "log.h"
#include "sound.h"
#include "buzzer.h"
#include "music.h" // note freq values

static bool isInit=false;

static uint32_t neffect = 0;
static uint32_t sys_effect = 0;
static uint32_t user_effect = 0;

static Melody range_slow = {.bpm = 120, .delay = 1, .notes = {{C4, H}, {D4, H}, {E4, H}, {F4, H}, {G4, H}, {A4, H}, {B4, H}, REPEAT}};
static Melody range_fast = {.bpm = 120, .delay = 1, .notes = {{C4, S}, {D4, S}, {E4, S}, {F4, S}, {G4, S}, {A4, S}, {B4, S}, REPEAT}};
static Melody startup = {.bpm = 120, .delay = 1, .notes = {{C6, S}, {C6, S}, STOP}};
static Melody calibrated = {.bpm = 120, .delay = 1, .notes = {{C4, S}, {E4, S}, {G4, S}, {C5, E}, STOP}};
static Melody chg_done = {.bpm = 120, .delay = 1, .notes = {{D4, Q}, {A4, Q}, STOP}};
static Melody lowbatt = {.bpm = 120, .delay = 1, .notes = {{D4, E}, {A4, E}, {D4, E}, REPEAT}};
static Melody usb_disconnect = {.bpm = 120, .delay = 1, .notes = {{C4, E}, STOP}};
static Melody usb_connect = {.bpm = 120, .delay = 1, .notes = {{A4, E}, STOP}};
static Melody factory_test = {.bpm = 120, .delay = 1, .notes = {{A1, Q}, {OFF, S}, {A2, Q}, {OFF, S}, REPEAT}};
static Melody starwars = {.bpm = 120, .delay = 1, .notes = {{A3, Q}, {A3, Q}, {A3, Q}, {F3, ES}, {C4, S},
    {A3, Q}, {F3, ES}, {C4, S}, {A3, H},
    {E4, Q}, {E4, Q}, {E4, Q}, {F4, ES}, {C4, S},
    {Ab3, Q}, {F3, ES}, {C4, S}, {A3, H},
    {A4, Q}, {A3, ES}, {A3, S}, {A4, Q}, {Ab4, ES}, {G4, S},
    {Gb4, S}, {E4, S}, {F4, E}, {0, E}, {Bb3, E}, {Eb4, Q}, {D4, ES}, {Db4, S},
    {C4, S}, {B3, S}, {C4, E}, {0, E}, {F3, E}, {Ab3, Q}, {F3, ES}, {A3, S},
    {C4, Q}, {A3, ES}, {C4, S}, {E4, H},
    {A4, Q}, {A3, ES}, {A3, S}, {A4, Q}, {Ab4, ES}, {G4, S},
    {Gb4, S}, {E4, S}, {F4, E}, {0, E}, {Bb3, E}, {Eb4, Q}, {D4, ES}, {Db4, S},
    {Gb4, S}, {E4, S}, {F4, E}, {0, E}, {Bb3, E}, {Eb4, Q}, {D4, ES}, {Db4, S},
    {C4,S}, {B3, S}, {C4, E}, {0, E}, {F3, E}, {Ab3, Q}, {F3, ES}, {C4, S},
    {A3, Q}, {F3, ES}, {C4, S}, {A3, H}, {0, H},
    REPEAT}};
static Melody valkyries = {.bpm = 140, .delay = 1, .notes = {{Gb3, Q}, {B3, Q},
    {Gb3, S}, {B3, E},  {D4, Q}, {B3, Q}, {D4, Q}, {B3, S}, {D4, E}, {Gb4, Q},  
    {D4, Q}, {Gb4, Q}, {D4, S}, {Gb4, E}, {A4, Q}, {A3, Q}, {D4, Q}, {A3, S},  
    {D4, E}, {Gb4, H}, 
    REPEAT}};

typedef void (*BuzzerEffect)(uint32_t timer, uint32_t * mi, Melody * melody);

static void off(uint32_t counter, uint32_t * mi, Melody * m) {
  buzzerOff();
}

static void turnCurrentEffectOff() {
  if (sys_effect != 0) {
    sys_effect = 0;
  } else {
    user_effect = 0;
  }
}

static uint32_t mcounter = 0;
static void melodyplayer(uint32_t counter, uint32_t * mi, Melody * m) {
  uint16_t tone = m->notes[(*mi)].tone;
  uint16_t duration = m->notes[(*mi)].duration;

  if (mcounter == 0) {
    if (tone == 0xFE) {
      // Turn off buzzer since we're at the end
      (*mi) = 0;
      turnCurrentEffectOff();
    } else if (tone == 0xFF) {
      // Loop the melody
      (*mi) = 0;
    } else {
      // Play current note
      buzzerOn(tone);
      mcounter = (100 * 4 * 60) / (m->bpm * duration) - 1;
      (*mi)++;
    }
  } else {
    if (mcounter == 1) {
        buzzerOff();
    }
    mcounter--;
  }
}

static uint8_t static_ratio = 0;
static uint16_t static_freq = 4000;
static void bypass(uint32_t counter, uint32_t * mi, Melody * melody)
{
  buzzerOn(static_freq);
}

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

typedef struct {
  BuzzerEffect call;
  uint32_t mi;
  Melody * melody;
} EffectCall;

static EffectCall effects[] = {
    [SND_OFF] = {.call = &off},
    [FACTORY_TEST] = {.call = &melodyplayer, .melody = &factory_test},
    [SND_USB_CONN] = {.call = &melodyplayer, .melody = &usb_connect},
    [SND_USB_DISC] = {.call = &melodyplayer, .melody = &usb_disconnect},
    [SND_BAT_FULL] = {.call = &melodyplayer, .melody = &chg_done},
    [SND_BAT_LOW] = {.call = &melodyplayer, .melody = &lowbatt},
    [SND_STARTUP] = {.call = &melodyplayer, .melody = &startup},
    [SND_CALIB] = {.call = &melodyplayer, .melody = &calibrated},
    {.call = &melodyplayer, .melody = &range_slow},
    {.call = &melodyplayer, .melody = &range_fast},
    {.call = &melodyplayer, .melody = &starwars},
    {.call = &melodyplayer, .melody = &valkyries},
    {.call = &bypass},
    {.call = &siren},
    {.call = &tilt}
};

static xTimerHandle timer;
static uint32_t counter = 0;

static void soundTimer(xTimerHandle timer)
{
  int effect;
  counter++;

  if (sys_effect != 0) {
    effect = sys_effect;
  } else {
    effect = user_effect;
  }

  if (effects[effect].call != 0) {
    effects[effect].call(counter * 10, &effects[effect].mi, effects[effect].melody);
  }
}

void soundInit(void)
{
  if (isInit) {
    return;
  }

  neffect = sizeof(effects) / sizeof(effects[0]) - 1;

  timer = xTimerCreate("SoundTimer", M2T(10), pdTRUE, NULL, soundTimer);
  xTimerStart(timer, 100);

  isInit = true;
}

bool soundTest(void)
{
  return isInit;
}

void soundSetEffect(uint32_t effect)
{
  sys_effect = effect;
}

void soundSetFreq(uint32_t freq) {

}

PARAM_GROUP_START(sound)
PARAM_ADD(PARAM_UINT8, effect, &user_effect)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_ADD(PARAM_UINT16, freq, &static_freq)
PARAM_ADD(PARAM_UINT8, ratio, &static_ratio)
PARAM_GROUP_STOP(sound)
