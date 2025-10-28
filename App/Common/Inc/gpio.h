// 공통 버튼 처리 함수

#include "def.h"
#include "util.h"

void ProcessButtonPress(uint8_t buttonPressed, uint8_t expectedState, uint8_t commandCode, uint8_t dataIndex);
void ProcessMemoryLongPress(uint8_t commandCode);
void ProcessUpButtonLongPress(void);
void SetButtonState(uint8_t buttonPressed, uint8_t stateValue);
void ProcessTriggers(void);
uint8_t GetTriggerOutState(bool trigger_out1, bool trigger_out2);