#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <string.h>

// 전역 변수 선언
extern uint8_t gGlobal_Buffer[2048];
extern uint8_t gGlobal_usbToggle;
extern uint32_t gGlobal_usbLen;
extern uint8_t gGlobal_trigginState1;
extern uint8_t gGlobal_trigginState2;
extern uint8_t gGlobal_crc1;
extern uint8_t gGlobal_crc2;
extern int gGlobal_Rxindx;
extern uint8_t gGlobal_rxData;
extern uint8_t gGlobal_rxBuffer[100];
extern uint8_t gGlobal_tmpBuffer[100];
extern uint8_t test[2048];
extern uint8_t testbuf[100];

// UART 함수 프로토타입
int USB_minipc(void);
void RX_DATA(void);

#endif /* UART_H_ */
