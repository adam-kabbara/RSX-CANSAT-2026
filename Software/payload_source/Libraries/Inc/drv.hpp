#ifndef __DRV_H_
#define __DRV_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
}
#endif

class DRV
{

private:

	TIM_HandleTypeDef  *_htim;
	uint32_t            _channel;
	GPIO_TypeDef       *_port;
	uint16_t            _pin;

	// non-blocking motor control state
	uint32_t            _motor_stop_tick;
	uint8_t             _motor_running;

public:

	DRV();
	DRV(TIM_HandleTypeDef *htim, uint32_t channel);

	void Init(TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *port, uint16_t pin);

	// direction: 1 = forward, 0 = reverse
	void motor_run(uint8_t direction, uint32_t time_ms);

	void motor_stop();

	// call periodically from main loop to stop motor when time elapsed
	void motor_update();

};

#endif /* __DRV_H_ */
