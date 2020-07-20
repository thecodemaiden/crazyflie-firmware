#ifndef __MOTOR_SOUND_H__
#define __MOTOR_SOUND_H__

// The motor sound module lets me change the pwm parameters of the motors
// and set up chirp frequencies

typedef struct {
  uint16_t centerFreq;
  uint16_t chirpLen;
  uint16_t chirpSlope;
} MotorSoundParameters;


void motorSoundInit(void);

bool motorSoundTest(void);

const MotorSoundParameters *currentMotorParams();


#endif
