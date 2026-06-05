#include "servo.hpp"

Servo::Servo(){}

/* ::::::::::::::::: Init ::::::::::::::::: */
void Servo::Init(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t minPPMPulseWidth, uint16_t maxPPMPulseWidth, float maxAngle, float minAngle)
{

	/* ~~~~~~~~~~~~~~~ PWM timer ~~~~~~~~~~~~~~ */
	_htim            = htim;
	_channel         = channel;

	/* ~~~~~~~~~~~~~~~~~ Param ~~~~~~~~~~~~~~~~ */
	MinPPMPulseWidth = minPPMPulseWidth;
	MaxPPMPulseWidth = maxPPMPulseWidth;
	MaxAngle         = maxAngle;
	MinAngle         = minAngle;

	/* ~~~~~~~~~~~~~~ Calculation ~~~~~~~~~~~~~ */
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    if (MaxPPMPulseWidth > arr) {
        MaxPPMPulseWidth = (uint16_t)arr;
    }

    _angRatio = ((float)(MaxPPMPulseWidth - MinPPMPulseWidth) / (MaxAngle - MinAngle));

    HAL_TIM_PWM_Start(htim, channel);

}

/* ::::::::::::::: Position ::::::::::::::: */
void Servo::SetPPMPulseWidth(uint16_t width)
{
	if (width > MaxPPMPulseWidth) width = MaxPPMPulseWidth;
	if (width < MinPPMPulseWidth) width = MinPPMPulseWidth;
	__HAL_TIM_SET_COMPARE(_htim, _channel, PPMPulseWidth = width);
}

void Servo::SetAngle(float ang){
    Angle = ang;

    if (Angle > MaxAngle) Angle = MaxAngle;
    if (Angle < MinAngle) Angle = MinAngle;

    uint32_t compare = (uint32_t)(MinPPMPulseWidth + (_angRatio * (Angle - MinAngle)));
    __HAL_TIM_SET_COMPARE(_htim, _channel, compare);
}
