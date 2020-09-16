
#ifndef __ESTIMATOR_CAMERA_H__
#define __ESTIMATOR_CAMERA_H__

#include <stdint.h>
#include "stabilizer_types.h"

void estimatorCameraInit(void);
bool estimatorCameraTest(void);
void estimatorCamera(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);

bool estimatorCameraEnqueuePosition(const positionMeasurement_t *pos);

void estimatorCameraGetEstimatedPos(point_t* pos);

#endif //__ESTIMATOR_CAMERA_H__
