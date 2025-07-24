#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>

// 전역 변수 선언
extern uint8_t gGlobal_triggoutState1;
extern uint8_t gGlobal_triggoutState2;

// GPIO 함수 프로토타입
int TriggerPin_1(void);
int TriggerPin_2(void);

#endif /* GPIO_H_ */
