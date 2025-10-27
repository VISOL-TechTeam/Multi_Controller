#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>
#include <stdbool.h>

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

// 엔코더 상수 정의
#define ENCODER_COUNT_MAX 999
#define ENCODER_COUNT_MIN -999
#define ENCODER_DEBOUNCE_TIME_MS 20      // 디바운스 시간 증가 (5ms -> 15ms)
#define ENCODER_MIN_PULSE_INTERVAL_MS 10 // 최소 펄스 간격 증가 (20ms -> 50ms)
#define ENCODER_SPEED_TIMEOUT_MS 100
#define ENCODER_SPEED_SAMPLE_PERIOD_MS 50
#define ENCODER_DIRECTION_LOCK_TIME_MS 500       // 방향 잠금 시간 증가 (200ms -> 500ms)
#define ENCODER_MIN_SAME_DIRECTION_COUNT 2       // 방향 잠금을 위한 최소 연속 카운트 (3 -> 2)
#define ENCODER_DIRECTION_CONFIDENCE_THRESHOLD 3 // 방향 신뢰도 임계값

// 로터리 인코더 구조체 (개선된 버전)
typedef struct
{
    // 기존 호환성을 위한 필드들
    int currentStateCLK;
    int lastStateCLK;
    uint8_t dtState;
    uint8_t last_valid_direction;
    uint32_t last_action_time;

    // 새로운 고정밀 엔코더 필드들
    volatile int32_t count;             // 엔코더 카운트
    volatile uint8_t direction;         // 0: 시계방향, 1: 반시계방향
    volatile int32_t speed;             // 속도 (counts per second)
    volatile uint32_t last_change_time; // 마지막 변화 시간
    volatile int32_t pulse_count;       // 펄스 카운트

    // 상태 관리
    volatile uint8_t current_state;       // 현재 A,B 상태 (bit1:A, bit0:B)
    volatile uint8_t last_state;          // 이전 A,B 상태
    volatile uint32_t last_debounce_time; // 디바운스 시간
    volatile uint8_t stable_state;        // 안정화된 상태

    // 속도 계산용
    volatile uint32_t speed_calc_time;  // 속도 계산 시간
    volatile int32_t speed_pulse_count; // 속도 계산용 펄스 카운트

    // 연속 감지 방지용
    volatile uint32_t last_pulse_time;    // 마지막 펄스 시간
    volatile uint8_t consecutive_count;   // 연속 감지 카운터
    volatile int8_t last_direction_delta; // 마지막 방향 변화값

    // 방향 잠금 메커니즘
    volatile uint8_t direction_lock;       // 방향 잠금 (1: 시계방향, 2: 반시계방향, 0: 잠금 없음)
    volatile uint32_t direction_lock_time; // 방향 잠금 시작 시간
    volatile uint8_t same_direction_count; // 연속 같은 방향 카운터

    // 방향 신뢰도 메커니즘
    volatile uint8_t cw_confidence;      // 시계방향 신뢰도 카운터
    volatile uint8_t ccw_confidence;     // 반시계방향 신뢰도 카운터
    volatile uint8_t dominant_direction; // 우세한 방향 (1: 시계, 2: 반시계, 0: 없음)
    
    // 방향 오동작 보정 메커니즘
    volatile uint8_t correction_count;   // 방향 치환 카운터 (최대 3회)
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
    
    uint8_t enable_buzzer;
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

// 새로운 고정밀 엔코더 함수들
void ProcessEncoderAdvanced(void);
void InitEncoderAdvanced(void);
int32_t GetEncoderCount(void);
uint8_t GetEncoderDirection(void);
int32_t GetEncoderSpeed(void);
void ResetEncoderCount(void);
uint32_t GetEncoderLastChangeTime(void);
int32_t GetAndResetEncoderPulseCount(void);

// 시스템 상태 초기화
void InitSystemState(void);

#endif /* GPIO_H_ */
