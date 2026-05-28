#include "drv.h"
#include "main.h"

static volatile uint32_t motor_stop_tick = 0;
static volatile uint8_t  motor_running   = 0;

void motor_run(uint8_t direction, uint32_t time_ms)
{ // direction: 1 = forward, 0 = reverse; time_ms: how long to run in milliseconds (non-blocking — call motor_update() in main loop)
    motor_stop_tick = HAL_GetTick() + time_ms;
    motor_running   = 1;

    HAL_GPIO_WritePin(SERVO_WING_DIR_GPIO_Port, SERVO_WING_DIR_Pin,
                      direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SERVO_WING_PWM_GPIO_Port, SERVO_WING_PWM_Pin,
                      direction ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void motor_stop()
{
    motor_running = 0;
    HAL_GPIO_WritePin(SERVO_WING_PWM_GPIO_Port, SERVO_WING_PWM_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SERVO_WING_DIR_GPIO_Port, SERVO_WING_DIR_Pin, GPIO_PIN_RESET);
}

void motor_update()
{
    if (motor_running && HAL_GetTick() >= motor_stop_tick)
    {
        motor_stop();
    }
}
