#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>

// 전역 변수 선언
extern uint8_t gGlobal_sum;
extern uint8_t gGlobal_sum1;
extern uint8_t gGlobal_sum2;
extern uint8_t gGlobal_sendData[12];
extern uint8_t gGlobal_Buffer[2048];
extern uint32_t gGlobal_usbLen;
extern uint8_t gGlobal_crc1;
extern uint8_t gGlobal_crc2;

// 유틸리티 함수 프로토타입
char hextoascii(uint8_t local_toconv);
uint8_t asciitohex(char local_toconv);
void hex2ascii(void);
uint8_t calculate_crc8(void);
uint8_t Pad_calculate_crc8(void);

#endif /* UTIL_H_ */
