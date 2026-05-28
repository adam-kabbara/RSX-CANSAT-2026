#ifndef __DRV_H_
#define __DRV_H_

#include <stdint.h>

// direction: 1 = forward, 0 = reverse
// time_ms:   how long to run in milliseconds (non-blocking — call motor_update() in main loop)
void motor_run(uint8_t direction, uint32_t time_ms);
void motor_stop();
void motor_update();

#endif /* __DRV_H_ */
