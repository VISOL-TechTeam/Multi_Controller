#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>
#include <stdbool.h>

// 버튼 상태 정의
typedef enum {
    BUTTON_STATE_IDLE = 0,
    BUTTON_STATE_UP = 1,
    BUTTON_STATE_DOWN = 2,
    BUTTON_STATE_POWER = 3,
    BUTTON_STATE_BOOST = 4,
    BUTTON_STATE_PCMODE = 5,
    BUTTON_STATE_MEMORY1 = 6,
    BUTTON_STATE_MEMORY2 = 7,
    BUTTON_STATE_MEMORY3 = 8,
    BUTTON_STATE_MEMORY4 = 9
} ButtonState_t;

// 버튼 상태 구조체
typedef struct {
    uint8_t upButton;
    uint8_t downButton;
    uint8_t powerButton;
    uint8_t boostButton;
    uint8_t pcmodeButton;
    uint8_t memory1Button;
    uint8_t memory2Button;
    uint8_t memory3Button;
    uint8_t memory4Button;
} ButtonInputs_t;

// 타이머 및 카운터 구조체
typedef struct {
    int keyCount;
    int longKeycount;
    unsigned short keyTimer;
    unsigned short longKey;
    unsigned short ledTimer;
    unsigned short ledCount;
} TimerCounters_t;

// 트리거 상태 구조체
typedef struct {
    uint8_t triggIn1;
    uint8_t triggIn2;
    uint8_t trigginState1;
    uint8_t trigginState2;
    uint8_t triggerState;
    uint8_t triggerState2;
} TriggerStates_t;

// 로터리 인코더 구조체
typedef struct {
    int currentStateCLK;
    int lastStateCLK;
    uint8_t dtState;
} EncoderStates_t;

// 통신 데이터 구조체
typedef struct {
    uint8_t sendData[12];
    uint8_t crc1;
    uint8_t crc2;
} CommData_t;

// 전체 시스템 상태 구조체
typedef struct {
    ButtonInputs_t buttons;
    TimerCounters_t timers;
    TriggerStates_t triggers;
    EncoderStates_t encoder;
    CommData_t comm;
    uint8_t state;
    bool is_complete_memory;
    bool is_complete_up_long;
    uint8_t triggoutState1;
    uint8_t triggoutState2;
} SystemState_t;

// 전역 시스템 상태 변수
extern SystemState_t g_systemState;

// GPIO 함수 프로토타입
int TriggerPin_1(void);
int TriggerPin_2(void);

// 버튼 처리 함수들
void ProcessAllButtons(void);
void ProcessEncoder(void);
void ProcessLEDState(void);
void ProcessTriggers(void);

// 시스템 상태 초기화
void InitSystemState(void);

#endif /* GPIO_H_ */
