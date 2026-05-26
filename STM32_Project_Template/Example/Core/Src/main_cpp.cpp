#include "main.h"

extern TIM_HandleTypeDef htim1;

static volatile int32_t motor_ticks_remaining = 0;

// direction: 1 = forward, 0 = reverse
// time_ms:   how long to run in milliseconds (non-blocking)
void motor_run(uint8_t direction, uint32_t time_ms)
{
    HAL_TIM_Base_Stop_IT(&htim1);  // cancel any in-progress run

    HAL_GPIO_WritePin(SERVO_WING_DIR_GPIO_Port, SERVO_WING_DIR_Pin,
                      direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SERVO_WING_PWM_GPIO_Port, SERVO_WING_PWM_Pin, GPIO_PIN_SET);

    motor_ticks_remaining = (int32_t)time_ms;
    HAL_TIM_Base_Start_IT(&htim1);
}

// TIM1 fires every 1 ms — decrement counter and coast-stop when done
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1) return;

    if (--motor_ticks_remaining <= 0) {
        HAL_GPIO_WritePin(SERVO_WING_PWM_GPIO_Port, SERVO_WING_PWM_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SERVO_WING_DIR_GPIO_Port, SERVO_WING_DIR_Pin, GPIO_PIN_RESET);
        HAL_TIM_Base_Stop_IT(htim);
    }
}

void setup()
{
    // Reconfigure TIM1 for 1 ms period: 170 MHz / 170 / 1000 = 1 kHz overflow
    htim1.Init.Prescaler = 169;
    htim1.Init.Period    = 999;
    HAL_TIM_Base_Init(&htim1);

    printf("Setup completed.\n");
}

void loop()
{
    while (1)
    {
        printf("Motor ON (forward, 2s)\r\n");
        motor_run(1, 2000);         // returns immediately
        HAL_Delay(5000);            // 5 s between runs
    }
}
