/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * motors.c - Motor driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 */

#include <stdbool.h>

/* ST includes */
#include "stm32fxxx.h"

#include "motors.h"
#include "pm.h"

//FreeRTOS includes
#include "task.h"

//Logging includes
#include "log.h"

static uint16_t motorsBLConvBitsTo16(uint16_t bits);
static uint16_t motorsBLConv16ToBits(uint16_t bits);
static uint16_t motorsConvBitsTo16(uint16_t bits, uint16_t period);
static uint16_t motorsConv16ToBits(uint16_t bits, uint16_t period);

uint16_t motor_ratios[] = {0, 0, 0, 0};
uint16_t motor_conv_ratios[] = {0,0,0,0};
uint32_t motor_periods[]= {MOTORS_PWM_PERIOD, MOTORS_PWM_PERIOD, MOTORS_PWM_PERIOD, MOTORS_PWM_PERIOD};

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

#include "motors_def_cf2.c"

const MotorPerifDef** motorMap;  /* Current map configuration */

const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

const uint16_t testsound[NBR_OF_MOTORS] = {A6, A7, F7, D7 };

static bool isInit = false;

/* Private functions */

static uint16_t motorsBLConvBitsTo16(uint16_t bits)
{
  return (0xFFFF * (bits - MOTORS_BL_PWM_CNT_FOR_HIGH) / MOTORS_BL_PWM_CNT_FOR_HIGH);
}

static uint16_t motorsBLConv16ToBits(uint16_t bits)
{
  return (MOTORS_BL_PWM_CNT_FOR_HIGH + ((bits * MOTORS_BL_PWM_CNT_FOR_HIGH) / 0xFFFF));
}

static uint16_t motorsConvBitsTo16(uint16_t bits, uint16_t period)
{
 // given the pwm period, map the capture compare to 0-65536
  float dutyCycle = (float)(bits)/(period+1);
  return (uint16_t)(dutyCycle*(UINT16_MAX));
}

static uint16_t motorsConv16ToBits(uint16_t bits, uint16_t period)
{
// given the pwm period, map the ratio (0-65536) to the proper period
  float dutyCycle = (float)bits/(UINT16_MAX);
  return (uint16_t)((period+1)*dutyCycle);
}


/* Public functions */

//Initialization. Will set all motors ratio to 0%
void motorsInit(const MotorPerifDef** motorMapSelect)
{
  int i;
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  if (isInit)
  {
    // First to init will configure it
    return;
  }

  motorMap = motorMapSelect;

  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    //Clock the gpio and the timers
    MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPerif, ENABLE);
    MOTORS_RCC_TIM_CMD(motorMap[i]->timPerif, ENABLE);

    // Configure the GPIO for the timer output
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = MOTORS_GPIO_MODE;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = motorMap[i]->gpioOType;
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);

    //Map timers to alternate functions
    MOTORS_GPIO_AF_CFG(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, motorMap[i]->gpioAF);

    //Timer configuration
    TIM_TimeBaseStructure.TIM_Period = motorMap[i]->timPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = motorMap[i]->timPrescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(motorMap[i]->tim, &TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(motorMap[i]->tim, ENABLE);

    // PWM channels configuration (All identical!)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = motorMap[i]->timPolarity;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    // Configure Output Compare for PWM
    motorMap[i]->ocInit(motorMap[i]->tim, &TIM_OCInitStructure);
    motorMap[i]->preloadConfig(motorMap[i]->tim, TIM_OCPreload_Enable);

    MOTORS_TIM_DBG_CFG(motorMap[i]->timDbgStop, ENABLE);
    //Enable the timer PWM outputs
    TIM_CtrlPWMOutputs(motorMap[i]->tim, ENABLE);
  }

  // Start the timers
  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    TIM_Cmd(motorMap[i]->tim, ENABLE);
  }

  isInit = true;
}

void motorsDeInit(const MotorPerifDef** motorMapSelect)
{
  int i;
  GPIO_InitTypeDef GPIO_InitStructure;

  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    // Configure default
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);

    //Map timers to alternate functions
    GPIO_PinAFConfig(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, 0x00);

    //Deinit timer
    TIM_DeInit(motorMap[i]->tim);
  }
}

bool motorsTest(void)
{
  int i;

  for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
  {
    if (motorMap[i]->drvType == BRUSHED)
    {
#ifdef ACTIVATE_STARTUP_SOUND
      motorsBeep(MOTORS[i], true, testsound[i], 12000);
      vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
      motorsBeep(MOTORS[i], false, 0, 0);
      vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#else
      motorsSetRatio(MOTORS[i], 8000);
      vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
      motorsSetRatio(MOTORS[i], 0);
      vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#endif
    }
  }

  return isInit;
}

// Ithrust is thrust mapped for 65536 <==> 60 grams
void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
  if (isInit) {
    uint16_t ratio;

    ASSERT(id < NBR_OF_MOTORS);

    ratio = ithrust;

  #ifdef ENABLE_THRUST_BAT_COMPENSATED
    if (motorMap[id]->drvType == BRUSHED)
    {
      float thrust = ((float)ithrust / 65536.0f) * 60;
      float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
      float supply_voltage = pmGetBatteryVoltage();
      float percentage = volts / supply_voltage;
      percentage = percentage > 1.0f ? 1.0f : percentage;
      ratio = percentage * UINT16_MAX;
    }
  #endif
    motor_ratios[id] = ratio;
    uint16_t newRatio = motorsConv16ToBits(ratio, motor_periods[id]);
    motor_conv_ratios[id] = newRatio;
    if (motorMap[id]->drvType == BRUSHLESS)
    {
      motorMap[id]->setCompare(motorMap[id]->tim, motorsBLConv16ToBits(ratio));
    }
    else
    {
      motorMap[id]->setCompare(motorMap[id]->tim, newRatio);
    }
  }
}

int motorsGetRatio(uint32_t id)
{
  int ratio;

  ASSERT(id < NBR_OF_MOTORS);
  if (motorMap[id]->drvType == BRUSHLESS)
  {
    ratio = motorsBLConvBitsTo16(motorMap[id]->getCompare(motorMap[id]->tim));
  }
  else
  {
    ratio = motorsConvBitsTo16(motorMap[id]->getCompare(motorMap[id]->tim), motor_periods[id]);
  }

  return ratio;
}

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
  uint16_t newPeriod;
  uint16_t newRatio;

  ASSERT(id < NBR_OF_MOTORS);

  if (enable)
  {
    newPeriod = (uint16_t)(MOTORS_TIM_CLK_FREQ / frequency);
    newRatio = motorsConv16ToBits(ratio, newPeriod);
  }
  else
  {
    newPeriod = motorMap[id]->timPeriod;
    newRatio = 0;
  }

  motor_ratios[id] = ratio;
  motor_conv_ratios[id] = newRatio;
  motor_periods[id] = newPeriod;
  // Timer configuration
  TIM_CtrlPWMOutputs(motorMap[id]->tim, DISABLE);
  motorMap[id]->setCompare(motorMap[id]->tim, newRatio);
  TIM_SetAutoreload(motorMap[id]->tim, newPeriod);
  TIM_CtrlPWMOutputs(motorMap[id]->tim, ENABLE);
}

void motorsSetFrequency( uint16_t frequency)
{
  // ignore the id for now
  uint16_t period;
  uint16_t newRatio;
  bool turnOn = frequency > 0;
 
    if (turnOn)
    {
      period = (uint16_t)(MOTORS_TIM_CLK_FREQ / frequency);
    }
    else
    {
      period = MOTORS_PWM_PERIOD;
    }

      for (int id=0; id < NBR_OF_MOTORS; id++) {
        if (motor_periods[id] != period) {
        // don't make changes unless we must
        motor_periods[id] = period;

	    newRatio = motorsConv16ToBits(motor_ratios[id], period);
	    motor_conv_ratios[id] = newRatio;
	    TIM_CtrlPWMOutputs(motorMap[id]->tim, DISABLE);
	    motorMap[id]->setCompare(motorMap[id]->tim, newRatio);
	    TIM_SetAutoreload(motorMap[id]->tim, period);
	    TIM_CtrlPWMOutputs(motorMap[id]->tim, ENABLE);
      }
    }

}

LOG_GROUP_START(pwm)
LOG_ADD(LOG_UINT16, rat1, motor_conv_ratios)
LOG_ADD(LOG_UINT16, rat2, motor_conv_ratios+1)
LOG_ADD(LOG_UINT16, rat3, motor_conv_ratios+2)
LOG_ADD(LOG_UINT16, rat4, motor_conv_ratios+3)
LOG_ADD(LOG_UINT16, per1, motor_periods)
LOG_ADD(LOG_UINT16, per2, motor_periods+1)
LOG_ADD(LOG_UINT16, per3, motor_periods+2)
LOG_ADD(LOG_UINT16, per4, motor_periods+3)
LOG_GROUP_STOP(pwm)
