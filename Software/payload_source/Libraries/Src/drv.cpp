#include "drv.hpp"

DRV::DRV(){}

void DRV::Init(TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *port, uint16_t pin)
{
	_htim            = htim;
	_channel         = channel;
	_port            = port;
	_pin             = pin;

	HAL_TIM_PWM_Start(htim, channel);
}

void DRV::motor_run(uint8_t direction, uint32_t time_ms)
{
    HAL_GPIO_WritePin(_port, _pin, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);

    __HAL_TIM_SET_COUNTER(_htim, 0);

    __HAL_TIM_SET_AUTORELOAD(_htim, time_ms);

    __HAL_TIM_SET_COMPARE(_htim, _channel, 1);

    HAL_TIM_OnePulse_Start(_htim, _channel);
}

void DRV::motor_stop()
{
	HAL_TIM_OnePulse_Stop(_htim, _channel);
	HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET);
}
