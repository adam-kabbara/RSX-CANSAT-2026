#ifndef __SERVO_H_
#define __SERVO_H_

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
}
#endif

class Servo
{

private:

	/* ~~~~~~~~~~~~~~~~~~~~~~~~~ Variable ~~~~~~~~~~~~~~~~~~~~~~~~~ */
	/* :::::::::::::: Calculation ::::::::::::: */
	float               _angRatio;

	/* ::::::::::::::: PWM timer :::::::::::::: */
	TIM_HandleTypeDef  *_htim;
	uint32_t            _channel;

public:

	/* ~~~~~~~~~~~~~~~~~~~~~~~~~ Variable ~~~~~~~~~~~~~~~~~~~~~~~~~ */
	/* :::::::::::::: Servo param ::::::::::::: */
	uint16_t            PPMPulseWidth;
	uint16_t            MinPPMPulseWidth;
	uint16_t            MaxPPMPulseWidth;

	float               Angle;
	float               MaxAngle;

	/* ~~~~~~~~~~~~~~~~~~~~~~~~ Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~ */
	/* :::::::::::::: Constructor ::::::::::::: */
	Servo();
	Servo(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t minPPMPulseWidth = 1000, uint16_t maxPPMPulseWidth = 2000, float maxAngle = 180.0F);

	/* ::::::::::::::::: Init ::::::::::::::::: */
	void Init(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t minPPMPulseWidth = 1000, uint16_t maxPPMPulseWidth = 2000, float maxAngle = 180.0F);

	/* ::::::::::::::: Position ::::::::::::::: */
	void SetPPMPulseWidth(uint16_t Width);
	void SetAngle(float ang);

};

#endif /* __SERVO_H_ */
