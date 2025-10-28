#ifndef DEF_H_
#define DEF_H_

#ifndef __cplusplus

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "util.h"
#include "gpio.h"

#include "usbd_cdc_if.h"

// 엔코더 관련 정의 추가
#define ENCODER_COUNT_MAX 2000000000
#define ENCODER_COUNT_MIN -2000000000
#define ENCODER_SPEED_THRESHOLD 10 // 빠른 회전 감지 임계값

// 버튼 상태 정의
typedef enum
{
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
typedef struct
{
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
typedef struct
{
    int keyCount;
    int longKeycount;
    unsigned short keyTimer;
    unsigned short longKey;
    unsigned short ledTimer;
    unsigned short ledCount;
} TimerCounters_t;

// 트리거 상태 구조체
typedef struct
{
    volatile uint8_t trigger_in1;
    volatile uint8_t trigger_in2;
    volatile uint8_t trigger_in_Old1;
    volatile uint8_t trigger_in_Old2;
    volatile uint8_t trigger_in_setup_1;
    volatile uint8_t trigger_in_setup_2;
    volatile uint8_t trigger_out1;
    volatile uint8_t trigger_out2;
} TriggerStates_t;

enum
{
    ALL_EDGE = 0,
    LOW_EDGE = 1,
    HIGH_EDGE = 2,
};

// 로터리 인코더 구조체
typedef struct
{
    int currentStateCLK;
    int lastStateCLK;
    uint8_t dtState;
    uint8_t last_valid_direction;
    uint32_t last_action_time;
} EncoderStates_t;

// 통신 데이터 구조체
typedef struct
{
    uint8_t sendData[12];
    uint8_t crc1;
    uint8_t crc2;
} CommData_t;

// 전체 시스템 상태 구조체
typedef struct
{
    ButtonInputs_t buttons;
    TimerCounters_t timers;
    volatile TriggerStates_t triggers;
    EncoderStates_t encoder;
    CommData_t comm;
    uint8_t state;
    bool is_complete_memory;
    bool is_complete_up_long;
    uint8_t triggoutState1;
    uint8_t triggoutState2;
    bool enable_buzzer;
} SystemState_t;

// 전역 시스템 상태 변수
extern SystemState_t g_systemState;

#endif

#endif
