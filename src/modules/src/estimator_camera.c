
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "physicalConstants.h"

#include "statsCnt.h"
#define DEBUG_MODULE "ESTCAM"
#include "debug.h"
//
#define ONE_SECOND 1000
static STATS_CNT_RATE_DEFINE(measurementAppendedCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(measurementNotAppendedCounter, ONE_SECOND);
// Direct measurements of Crazyflie position
static positionMeasurement_t _internalPosition;
static xQueueHandle posDataQueue;
static float alphaX = 0.5;
static float alphaY = 0.5;
static float alphaZ = 0.5;
static float initialX = 0.5;
static float initialY = 0.5;
static float initialZ = 0.5;
static uint8_t resetEstimation;
STATIC_MEM_QUEUE_ALLOC(posDataQueue, 10, sizeof(positionMeasurement_t));
static bool isInit=false;

static inline bool stateEstimatorHasPositionMeasurement(positionMeasurement_t *pos) {
  return (pdTRUE == xQueueReceive(posDataQueue, pos, 0));
}

void estimatorCameraInit() {
  posDataQueue = STATIC_MEM_QUEUE_CREATE(posDataQueue);
  isInit = true;
}

bool estimatorCameraTest() {
  return isInit;
}

bool estimatorCameraEnqueuePosition(const positionMeasurement_t *pos) 
{
  portBASE_TYPE result;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(posDataQueue, pos, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD();
    }
  } else {
    result = xQueueSend(posDataQueue, pos, 0);
  }

  if (result == pdTRUE) {
    STATS_CNT_RATE_EVENT(&measurementAppendedCounter);
    return true;
  } else {
    STATS_CNT_RATE_EVENT(&measurementNotAppendedCounter);
    return true;
  }
}

void estimatorCamera(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick)
{
 // I totally ignore the sensors, HAHA
  if (resetEstimation) {
    resetEstimation = 0;
    _internalPosition.x = initialX;
    _internalPosition.y = initialY;
    _internalPosition.z = initialZ;
  }
  positionMeasurement_t pos;
  while (stateEstimatorHasPositionMeasurement(&pos))
  {
    _internalPosition.x = alphaX*_internalPosition.x + (1-alphaX)*pos.x;
    _internalPosition.y = alphaY*_internalPosition.y + (1-alphaY)*pos.y;
    _internalPosition.z = alphaZ*_internalPosition.z + (1-alphaZ)*pos.z;
  }
  state->position.x = _internalPosition.x;
  state->position.y = _internalPosition.y;
  state->position.z = _internalPosition.z;
}

PARAM_GROUP_START(campos)
  PARAM_ADD(PARAM_FLOAT, alphaX, &alphaX)
  PARAM_ADD(PARAM_FLOAT, alphaY, &alphaY)
  PARAM_ADD(PARAM_FLOAT, alphaZ, &alphaZ)
  PARAM_ADD(PARAM_UINT8, resetEstimation, &resetEstimation)
  PARAM_ADD(PARAM_FLOAT, initialX, &initialX)
  PARAM_ADD(PARAM_FLOAT, initialY, &initialY)
  PARAM_ADD(PARAM_FLOAT, initialZ, &initialZ)
PARAM_GROUP_STOP(campos)

LOG_GROUP_START(campos)
  STATS_CNT_RATE_LOG_ADD(rtApnd, &measurementAppendedCounter)
  STATS_CNT_RATE_LOG_ADD(rtRej, &measurementNotAppendedCounter)
LOG_GROUP_STOP(campos)
