
#include "mic.h"
#include "deck.h"
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "worker.h"
#include "log.h"


static bool isInit = false;

static uint16_t bufferValues[BUFFER_SIZE];
static int headPtr = BUFFER_SIZE-1;
static float lastSum = 0;

static xTimerHandle timer;
static void micReadTimer(xTimerHandle timer);
void micInit(DeckInfo *info)
{
  if (isInit) return;

  pinMode(DECK_GPIO_TX2, INPUT); // this is the mic analog output pin on PA2
  // zero out the buffer
  for (int ii=0; ii < BUFFER_SIZE; ii++) {
    bufferValues[ii] = 0;
  }
  adcInit();

  isInit = true;
  timer = xTimerCreate("micTimer", M2T(25), pdTRUE, NULL, micReadTimer);
  xTimerStart(timer,100);
}

bool micTest()
{
  return true; // not sure what to do yet
}

/*** periodic analog read + process ***/

void updateMicBuffer()
{
  //static int tailPtr = 0;
  //tailPtr = headPtr+1;
  //if (tailPtr == BUFFER_SIZE) tailPtr = 0;

// TODO: use the power of static to keep oldest value around?
  //float oldestValue = (float)bufferValues[tailPtr];
  float accum = 0;
  uint16_t nextValue;
  nextValue = analogRead(DECK_GPIO_SCK);
  float fv = (float)(nextValue) - 2047.0f;
  
  bufferValues[headPtr] = nextValue;
  headPtr = headPtr+1;
  if (headPtr == BUFFER_SIZE) headPtr = 0;

  //lastSum = lastSum - (oldestValue*oldestValue);
  //lastSum = lastSum + ((float)nextValue *(float)nextValue);
  lastSum = fv;
}

static void micReadTimer(xTimerHandle timer)
{
  workerSchedule(updateMicBuffer, NULL);
}


const DeckDriver micDriver = {
  .vid = 0,
  .pid = 0,
  .name = "analogMic",
  .usedGpio = DECK_USING_PA5,
  .init = micInit,
  .test = micTest,
};

DECK_DRIVER(micDriver);

LOG_GROUP_START(mic)
  LOG_ADD(LOG_FLOAT, energy, &lastSum)
LOG_GROUP_STOP(mic)
