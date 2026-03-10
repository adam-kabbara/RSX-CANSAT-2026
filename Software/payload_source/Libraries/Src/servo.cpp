#include "servo.hpp"

Servo::Servo(){}
Servo::Servo(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t minPPMPulseWidth, uint16_t maxPPMPulseWidth, float maxAngle) : _htim(htim), _channel(channel), MinPPMPulseWidth(minPPMPulseWidth), MaxPPMPulseWidth(maxPPMPulseWidth), MaxAngle(maxAngle)
{
	_angRatio = (((float)MaxPPMPulseWidth - (float)MinPPMPulseWidth) / MaxAngle);
}

/* ::::::::::::::::: Init ::::::::::::::::: */
void Servo::Init(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t minPPMPulseWidth, uint16_t maxPPMPulseWidth, float maxAngle)
{

	/* ~~~~~~~~~~~~~~~ PWM timer ~~~~~~~~~~~~~~ */
	_htim            = htim;
	_channel         = channel;

	/* ~~~~~~~~~~~~~~~~~ Param ~~~~~~~~~~~~~~~~ */
	MinPPMPulseWidth = minPPMPulseWidth;
	MaxPPMPulseWidth = maxPPMPulseWidth;
	MaxAngle         = maxAngle;

	/* ~~~~~~~~~~~~~~ Calculation ~~~~~~~~~~~~~ */
	_angRatio        = ((MaxPPMPulseWidth - MinPPMPulseWidth) / MaxAngle);

	HAL_TIM_PWM_Start(htim, channel);

}

/* ::::::::::::::: Position ::::::::::::::: */
void Servo::SetPPMPulseWidth(uint16_t width)
{
	__HAL_TIM_SET_COMPARE(_htim, _channel, PPMPulseWidth = width);
}

void Servo::SetAngle(float ang)
{

	Angle = ang;

	if (Angle >= MaxAngle)
	{

		Angle = MaxAngle;
		__HAL_TIM_SET_COMPARE(_htim, _channel, (uint32_t)(MinPPMPulseWidth + (_angRatio * Angle) - 1));

	}
	else
	{
		__HAL_TIM_SET_COMPARE(_htim, _channel, (uint32_t)(MinPPMPulseWidth + (_angRatio * Angle)));
	}

}
