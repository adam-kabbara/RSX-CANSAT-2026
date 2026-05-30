#include "drv.hpp"
#include "main.h"
DRV::DRV()
    : _htim(nullptr), _channel(0), _port(nullptr), _pin(0), _motor_stop_tick(0), _motor_running(0)
{
}

DRV::DRV(TIM_HandleTypeDef *htim, uint32_t channel)
    : _htim(htim), _channel(channel), _port(nullptr), _pin(0), _motor_stop_tick(0), _motor_running(0)
{
}

void DRV::Init(TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *port, uint16_t pin)
{
    _htim = htim;
    _channel = channel;
    _port = port;
    _pin = pin;
}

void DRV::motor_run(uint8_t direction, uint32_t time_ms)
{
    _motor_stop_tick = HAL_GetTick() + time_ms;
    _motor_running = 1;

    HAL_GPIO_WritePin(SERVO_WING_DIR_GPIO_Port, SERVO_WING_DIR_Pin,
                      direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SERVO_WING_PWM_GPIO_Port, SERVO_WING_PWM_Pin,
                      direction ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void DRV::motor_stop()
{
    _motor_running = 0;
    HAL_GPIO_WritePin(SERVO_WING_PWM_GPIO_Port, SERVO_WING_PWM_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SERVO_WING_DIR_GPIO_Port, SERVO_WING_DIR_Pin, GPIO_PIN_RESET);
}

void DRV::motor_update()
{
    if (_motor_running && HAL_GetTick() >= _motor_stop_tick)
    {
        motor_stop();
    }
}