#include "gpio.h"
#include "main.h"
#include "usbd_cdc_if.h"

// 전역 시스템 상태 변수 정의
SystemState_t g_systemState = {0};

// 외부 함수 프로토타입
extern uint8_t Pad_calculate_crc8(void);
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

uint8_t _get_trigger_out_state(bool trigger_out1, bool trigger_out2);
static void _long_buttom_buzzer(void);

// 시스템 상태 초기화
void InitSystemState(void)
{
    // 모든 상태를 0으로 초기화 (이미 위에서 {0}으로 초기화됨)
    // 필요시 특별한 초기값 설정
    g_systemState.encoder.lastStateCLK = HAL_GPIO_ReadPin(Dial_B_GPIO_Port, Dial_B_Pin);
    
    // 고정밀 엔코더 초기화
    InitEncoderAdvanced();
}

int TriggerPin_1(void)
{
    // HAL_GPIO_TogglePin(Trigger_OUT_1_GPIO_Port, Trigger_OUT_1_Pin);
    if (g_systemState.triggoutState1 == 1)
    {
        HAL_GPIO_WritePin(Trigger_OUT_1_GPIO_Port, Trigger_OUT_1_Pin, GPIO_PIN_SET); // 트리거 1 출력 HIGH
    }
    if (g_systemState.triggoutState1 == 2)
    {
        HAL_GPIO_WritePin(Trigger_OUT_1_GPIO_Port, Trigger_OUT_1_Pin, GPIO_PIN_RESET); // 트리거 1 출력 LOW
    }
    return 0;
}

int TriggerPin_2(void)
{
    if (g_systemState.triggoutState2 == 1)
    {
        HAL_GPIO_WritePin(Trigger_OUT_2_GPIO_Port, Trigger_OUT_2_Pin, GPIO_PIN_SET); // 트리거 2 출력 HIGH
    }
    if (g_systemState.triggoutState2 == 2)
    {
        HAL_GPIO_WritePin(Trigger_OUT_2_GPIO_Port, Trigger_OUT_2_Pin, GPIO_PIN_RESET); // 트리거 2 출력 LOW
    }
    return 0;
}

// 공통 버튼 처리 함수
__attribute__((optimize("O0"))) static void ProcessButtonPress(uint8_t buttonPressed, uint8_t expectedState, uint8_t commandCode, uint8_t dataIndex)
{
    if (buttonPressed == 1 && g_systemState.state == expectedState)
    {
        g_systemState.comm.sendData[dataIndex] = commandCode;

        g_systemState.comm.sendData[8] = _get_trigger_out_state(g_systemState.triggers.trigger_out1, g_systemState.triggers.trigger_out2);
        Pad_calculate_crc8();
        g_systemState.comm.sendData[9] = g_systemState.comm.crc1;
        g_systemState.comm.sendData[10] = g_systemState.comm.crc2;
        CDC_Transmit_FS((uint8_t *)g_systemState.comm.sendData, sizeof(g_systemState.comm.sendData));
        g_systemState.comm.sendData[dataIndex] = 0x30;
        g_systemState.timers.keyCount = 0;
        g_systemState.timers.keyTimer = 0;
        g_systemState.state = 0;
    }
}

// 메모리 버튼 긴 누름 처리 함수
__attribute__((optimize("O0"))) static void ProcessMemoryLongPress(uint8_t commandCode)
{
    if (g_systemState.timers.longKeycount == 1)
    {
        g_systemState.comm.sendData[5] = commandCode;

        g_systemState.comm.sendData[8] = _get_trigger_out_state(g_systemState.triggers.trigger_out1, g_systemState.triggers.trigger_out2);
        Pad_calculate_crc8();
        g_systemState.comm.sendData[9] = g_systemState.comm.crc1;
        g_systemState.comm.sendData[10] = g_systemState.comm.crc2;
        CDC_Transmit_FS((uint8_t *)g_systemState.comm.sendData, sizeof(g_systemState.comm.sendData));
        g_systemState.comm.sendData[5] = 0x30;
        g_systemState.timers.longKey = 0;
        g_systemState.timers.longKeycount = 0;
        g_systemState.state = 0;
        g_systemState.is_complete_memory = true;
        _long_buttom_buzzer();
    }
}

// Up 버튼 긴 누름 처리 함수
static void ProcessUpButtonLongPress(void)
{
    if (g_systemState.timers.longKeycount == 1)
    {
        g_systemState.comm.sendData[4] = 0x50;

        g_systemState.comm.sendData[8] = _get_trigger_out_state(g_systemState.triggers.trigger_out1, g_systemState.triggers.trigger_out2);
        Pad_calculate_crc8();
        g_systemState.comm.sendData[9] = g_systemState.comm.crc1;
        g_systemState.comm.sendData[10] = g_systemState.comm.crc2;
        CDC_Transmit_FS((uint8_t *)g_systemState.comm.sendData, sizeof(g_systemState.comm.sendData));
        g_systemState.comm.sendData[4] = 0x30;
        g_systemState.timers.longKey = 0;
        g_systemState.timers.longKeycount = 0;
        g_systemState.state = 0;
        g_systemState.is_complete_up_long = true;
        _long_buttom_buzzer();
    }
}

// 버튼 상태 설정 함수
__attribute__((optimize("O0"))) static void SetButtonState(uint8_t buttonPressed, uint8_t stateValue)
{
    if (buttonPressed == 0 && g_systemState.timers.keyCount > 70 && g_systemState.timers.keyCount < 3000)
    {
        g_systemState.state = stateValue;
        g_systemState.timers.longKey = 0;
        g_systemState.timers.longKeycount = 0;
    }
}

// 통합 버튼 처리 함수
void ProcessAllButtons(void)
{

    // 버튼 상태에 따른 처리
    switch (g_systemState.state)
    {
    case BUTTON_STATE_IDLE:
        // 버튼 누름 감지 및 상태 설정
        SetButtonState(g_systemState.buttons.upButton, BUTTON_STATE_UP);
        SetButtonState(g_systemState.buttons.downButton, BUTTON_STATE_DOWN);
        SetButtonState(g_systemState.buttons.powerButton, BUTTON_STATE_POWER);
        SetButtonState(g_systemState.buttons.boostButton, BUTTON_STATE_BOOST);
        SetButtonState(g_systemState.buttons.pcmodeButton, BUTTON_STATE_PCMODE);
        SetButtonState(g_systemState.buttons.memory1Button, BUTTON_STATE_MEMORY1);
        SetButtonState(g_systemState.buttons.memory2Button, BUTTON_STATE_MEMORY2);
        SetButtonState(g_systemState.buttons.memory3Button, BUTTON_STATE_MEMORY3);
        SetButtonState(g_systemState.buttons.memory4Button, BUTTON_STATE_MEMORY4);

        break;
    case BUTTON_STATE_UP:
        if (g_systemState.is_complete_up_long == false)
            ProcessButtonPress(g_systemState.buttons.upButton, BUTTON_STATE_UP, 0x31, 4);

        HAL_GPIO_TogglePin(System_LED_GPIO_Port, System_LED_Pin);
        ProcessUpButtonLongPress();
        break;

    case BUTTON_STATE_DOWN:
        ProcessButtonPress(g_systemState.buttons.downButton, BUTTON_STATE_DOWN, 0x32, 4);
        break;

    case BUTTON_STATE_POWER:
        ProcessButtonPress(g_systemState.buttons.powerButton, BUTTON_STATE_POWER, 0x34, 4);
        break;

    case BUTTON_STATE_BOOST:
        ProcessButtonPress(g_systemState.buttons.boostButton, BUTTON_STATE_BOOST, 0x35, 4);
        break;

    case BUTTON_STATE_PCMODE:
        ProcessButtonPress(g_systemState.buttons.pcmodeButton, BUTTON_STATE_PCMODE, 0x33, 4);
        break;

    case BUTTON_STATE_MEMORY1:
        if (g_systemState.is_complete_memory == false)
            ProcessButtonPress(g_systemState.buttons.memory1Button, BUTTON_STATE_MEMORY1, 0x31, 5);

        ProcessMemoryLongPress(0x32);
        break;

    case BUTTON_STATE_MEMORY2:
        if (g_systemState.is_complete_memory == false)
            ProcessButtonPress(g_systemState.buttons.memory2Button, BUTTON_STATE_MEMORY2, 0x33, 5);

        ProcessMemoryLongPress(0x34);
        break;

    case BUTTON_STATE_MEMORY3:
        if (g_systemState.is_complete_memory == false)
            ProcessButtonPress(g_systemState.buttons.memory3Button, BUTTON_STATE_MEMORY3, 0x35, 5);

        ProcessMemoryLongPress(0x36);
        break;

    case BUTTON_STATE_MEMORY4:
        if (g_systemState.is_complete_memory == false)
            ProcessButtonPress(g_systemState.buttons.memory4Button, BUTTON_STATE_MEMORY4, 0x37, 5);

        ProcessMemoryLongPress(0x38);
        break;

    default:
        g_systemState.state = BUTTON_STATE_IDLE;
        break;
    }

    // 모든 버튼이 눌리지 않은 상태 확인
    if (g_systemState.buttons.upButton == 1 && g_systemState.buttons.downButton == 1 && g_systemState.buttons.powerButton == 1 &&
        g_systemState.buttons.boostButton == 1 && g_systemState.buttons.pcmodeButton == 1 && g_systemState.buttons.memory1Button == 1 &&
        g_systemState.buttons.memory2Button == 1 && g_systemState.buttons.memory3Button == 1 && g_systemState.buttons.memory4Button == 1)
    {
        g_systemState.timers.keyCount = 0;
        g_systemState.timers.keyTimer = 0;
        g_systemState.state = 0;
        g_systemState.timers.longKey = 0;
        g_systemState.timers.longKeycount = 0;
        g_systemState.is_complete_memory = false;
        g_systemState.is_complete_up_long = false;
        return;
    }
}


/**
 * @brief 고정밀 엔코더 초기화
 */
void InitEncoderAdvanced(void)
{
    
    // 엔코더 상태 초기화
    g_systemState.encoder.count = 0;
    g_systemState.encoder.direction = 0;
    g_systemState.encoder.speed = 0;
    g_systemState.encoder.last_change_time = HAL_GetTick();
    g_systemState.encoder.pulse_count = 0;
    
    // 현재 GPIO 상태 읽기 및 초기화
    g_systemState.encoder.current_state = (HAL_GPIO_ReadPin(Dial_A_GPIO_Port, Dial_A_Pin) << 1) |
                                         HAL_GPIO_ReadPin(Dial_B_GPIO_Port, Dial_B_Pin);
    g_systemState.encoder.last_state = g_systemState.encoder.current_state;
    g_systemState.encoder.stable_state = g_systemState.encoder.current_state;
    g_systemState.encoder.last_debounce_time = HAL_GetTick();
    
    // 속도 계산 초기화
    g_systemState.encoder.speed_calc_time = HAL_GetTick();
    g_systemState.encoder.speed_pulse_count = 0;
    
    // 연속 감지 방지 초기화
    g_systemState.encoder.last_pulse_time = HAL_GetTick();
    g_systemState.encoder.consecutive_count = 0;
    g_systemState.encoder.last_direction_delta = 0;
    
    // 방향 잠금 메커니즘 초기화
    g_systemState.encoder.direction_lock = 0;
    g_systemState.encoder.direction_lock_time = HAL_GetTick();
    g_systemState.encoder.same_direction_count = 0;
    
    // 방향 신뢰도 메커니즘 초기화
    g_systemState.encoder.cw_confidence = 0;
    g_systemState.encoder.ccw_confidence = 0;
    g_systemState.encoder.dominant_direction = 0;
    
    // 방향 오동작 보정 메커니즘 초기화
    g_systemState.encoder.correction_count = 0;
    
    // 기존 호환성을 위한 초기화
    g_systemState.encoder.lastStateCLK = HAL_GPIO_ReadPin(Dial_B_GPIO_Port, Dial_B_Pin);
    g_systemState.encoder.dtState = 0;
    g_systemState.encoder.last_valid_direction = 0;
    g_systemState.encoder.last_action_time = HAL_GetTick();
}

/**
 * @brief 개선된 엔코더 처리 함수 (기존 로직 기반)
 * 
 * 기존 ProcessEncoder 로직을 참조하여 안정성을 개선한 버전
 * - 기존 상태 머신 방식 유지
 * - 디바운싱 및 타이밍 검증 강화
 * - 연속 감지 방지 로직 추가
 */
void ProcessEncoderAdvanced(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // 방향 잠금 해제 확인 (일정 시간 후 잠금 해제)
    if (g_systemState.encoder.direction_lock != 0 && 
        (current_time - g_systemState.encoder.direction_lock_time) > ENCODER_DIRECTION_LOCK_TIME_MS)
    {
        g_systemState.encoder.direction_lock = 0;
        g_systemState.encoder.same_direction_count = 0;
        g_systemState.encoder.cw_confidence = 0;
        g_systemState.encoder.ccw_confidence = 0;
        g_systemState.encoder.dominant_direction = 0;
        g_systemState.encoder.correction_count = 0; // 방향 잠금 해제 시 치환 카운터 리셋
    }
    
    // 현재 CLK(B) 핀 상태 읽기 (기존 로직과 동일)
    g_systemState.encoder.currentStateCLK = HAL_GPIO_ReadPin(Dial_B_GPIO_Port, Dial_B_Pin);
    
    // CLK 상태가 변경되었을 때만 처리
    if (g_systemState.encoder.currentStateCLK != g_systemState.encoder.lastStateCLK)
    {
        // 최소 간격 확인 (연속 감지 방지)
        if ((current_time - g_systemState.encoder.last_pulse_time) >= ENCODER_MIN_PULSE_INTERVAL_MS)
        {
            uint8_t dtValue = HAL_GPIO_ReadPin(Dial_A_GPIO_Port, Dial_A_Pin);
            
            // 시계방향 회전 감지 (기존 로직 기반, 방향 신뢰도 추가)
            if (dtValue != g_systemState.encoder.currentStateCLK && g_systemState.encoder.currentStateCLK == 1)
            {
                // 방향 신뢰도 증가
                g_systemState.encoder.cw_confidence++;
                if (g_systemState.encoder.ccw_confidence > 0)
                    g_systemState.encoder.ccw_confidence--;
                
                // 우세한 방향 결정
                if (g_systemState.encoder.cw_confidence >= ENCODER_DIRECTION_CONFIDENCE_THRESHOLD)
                {
                    g_systemState.encoder.dominant_direction = 1;
                }
                
                // 방향 잠금 확인: 반시계방향으로 잠금되어 있거나 우세한 방향이 반시계면 무시
                if (g_systemState.encoder.direction_lock == 2 || 
                    (g_systemState.encoder.dominant_direction == 2 && g_systemState.encoder.ccw_confidence > g_systemState.encoder.cw_confidence))
                {
                    g_systemState.encoder.lastStateCLK = g_systemState.encoder.currentStateCLK;
                    return;
                }
                
                // 최소한의 검증만 수행 (반시계방향과 동일한 수준)
                if (g_systemState.encoder.last_valid_direction != 2 || 
                    (current_time - g_systemState.encoder.last_action_time) > 250)
                {
                    // 31방향(반시계) 락이 걸려있는지 확인
                    bool is_ccw_locked = (g_systemState.encoder.direction_lock == 2);
                    bool should_correct = false;
                    
                    // 31방향으로 락이 걸려있고 치환 횟수가 3회 미만인 경우
                    if (is_ccw_locked && g_systemState.encoder.correction_count < 3)
                    {
                        should_correct = true;
                        g_systemState.encoder.correction_count++;
                    }
                    
                    // 시계방향 회전 처리
                    g_systemState.encoder.count++;
                    g_systemState.encoder.direction = 0;
                    
                    // 순환 범위 제한
                    if (g_systemState.encoder.count > ENCODER_COUNT_MAX)
                        g_systemState.encoder.count = 0;
                    
                    // LED 상태 업데이트 (치환 시에는 31방향 LED로 표시)
                    if (should_correct)
                    {
                        HAL_GPIO_WritePin(Dial_LED_1_GPIO_Port, Dial_LED_1_Pin, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(Dial_LED_2_GPIO_Port, Dial_LED_2_Pin, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(Dial_LED_3_GPIO_Port, Dial_LED_3_Pin, GPIO_PIN_RESET);
                        g_systemState.comm.sendData[6] = 0x31; // 32를 31로 치환
                    }
                    else
                    {
                        HAL_GPIO_WritePin(Dial_LED_2_GPIO_Port, Dial_LED_2_Pin, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(Dial_LED_3_GPIO_Port, Dial_LED_3_Pin, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(Dial_LED_1_GPIO_Port, Dial_LED_1_Pin, GPIO_PIN_RESET);
                        g_systemState.comm.sendData[6] = 0x32;
                    }
                    
                    // 통신 패킷 전송
                    g_systemState.comm.sendData[8] = _get_trigger_out_state(g_systemState.triggers.trigger_out1, g_systemState.triggers.trigger_out2);
                    Pad_calculate_crc8();
                    g_systemState.comm.sendData[9] = g_systemState.comm.crc1;
                    g_systemState.comm.sendData[10] = g_systemState.comm.crc2;
                    CDC_Transmit_FS((uint8_t *)g_systemState.comm.sendData, sizeof(g_systemState.comm.sendData));
                    g_systemState.comm.sendData[6] = 0x30;
                    
                    // 상태 업데이트
                    g_systemState.timers.ledTimer = 0;
                    g_systemState.timers.ledCount = 0;
                    g_systemState.encoder.last_valid_direction = should_correct ? 2 : 1; // 치환 시에는 31방향으로 기록
                    g_systemState.encoder.last_action_time = current_time;
                    g_systemState.encoder.last_pulse_time = current_time;
                    g_systemState.encoder.last_change_time = current_time;
                    g_systemState.encoder.pulse_count++;
                    g_systemState.encoder.speed_pulse_count++;
                    
                    // 방향 잠금 메커니즘 (시계방향) - 치환된 경우는 잠금 변경하지 않음
                    if (!should_correct)
                    {
                        if (g_systemState.encoder.direction_lock == 1)
                        {
                            g_systemState.encoder.same_direction_count++;
                        }
                        else
                        {
                            g_systemState.encoder.direction_lock = 1;
                            g_systemState.encoder.direction_lock_time = current_time;
                            g_systemState.encoder.same_direction_count = 1;
                            g_systemState.encoder.correction_count = 0; // 방향 전환 시 치환 카운터 리셋
                        }
                    }
                }
            }
            // 반시계방향 회전을 위한 중간 상태 감지
            else if (dtValue == 0 && g_systemState.encoder.currentStateCLK == 0)
            {
                g_systemState.encoder.dtState = 1;
            }
            // 반시계방향 회전 완료 감지 (기존 로직 기반, 방향 신뢰도 추가)
            else if (g_systemState.encoder.dtState == 1 && dtValue == 1 && 
                     g_systemState.encoder.currentStateCLK == 1 && 
                     (current_time - g_systemState.encoder.last_action_time) < 150)
            {
                // 방향 신뢰도 증가
                g_systemState.encoder.ccw_confidence++;
                if (g_systemState.encoder.cw_confidence > 0)
                    g_systemState.encoder.cw_confidence--;
                
                // 우세한 방향 결정
                if (g_systemState.encoder.ccw_confidence >= ENCODER_DIRECTION_CONFIDENCE_THRESHOLD)
                {
                    g_systemState.encoder.dominant_direction = 2;
                }
                
                // 방향 잠금 확인: 시계방향으로 잠금되어 있으면 무시하지 않음 (31방향 전환 허용)
                // 단, 우세한 방향이 시계방향이고 신뢰도가 높으면 무시
                if (g_systemState.encoder.dominant_direction == 1 && 
                    g_systemState.encoder.cw_confidence > (g_systemState.encoder.ccw_confidence + 2))
                {
                    g_systemState.encoder.dtState = 0;
                    return;
                }
                
                // 기존 로직과 동일한 검증 (조건 완화)
                if (g_systemState.encoder.last_valid_direction != 1 || 
                    (current_time - g_systemState.encoder.last_action_time) > 50)
                {
                    // 반시계방향 회전 처리
                    g_systemState.encoder.count--;
                    g_systemState.encoder.direction = 1;
                    
                    // 순환 범위 제한
                    if (g_systemState.encoder.count < ENCODER_COUNT_MIN)
                        g_systemState.encoder.count = 0;
                    
                    // LED 상태 업데이트
                    HAL_GPIO_WritePin(Dial_LED_1_GPIO_Port, Dial_LED_1_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(Dial_LED_2_GPIO_Port, Dial_LED_2_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(Dial_LED_3_GPIO_Port, Dial_LED_3_Pin, GPIO_PIN_RESET);
                    g_systemState.comm.sendData[6] = 0x31;
                    
                    // 통신 패킷 전송
                    g_systemState.comm.sendData[8] = _get_trigger_out_state(g_systemState.triggers.trigger_out1, g_systemState.triggers.trigger_out2);
                    Pad_calculate_crc8();
                    g_systemState.comm.sendData[9] = g_systemState.comm.crc1;
                    g_systemState.comm.sendData[10] = g_systemState.comm.crc2;
                    CDC_Transmit_FS((uint8_t *)g_systemState.comm.sendData, sizeof(g_systemState.comm.sendData));
                    g_systemState.comm.sendData[6] = 0x30;
                    
                    // 상태 업데이트
                    g_systemState.timers.ledTimer = 0;
                    g_systemState.timers.ledCount = 0;
                    g_systemState.encoder.dtState = 0;
                    g_systemState.encoder.last_valid_direction = 2;
                    g_systemState.encoder.last_action_time = current_time;
                    g_systemState.encoder.last_pulse_time = current_time;
                    g_systemState.encoder.last_change_time = current_time;
                    g_systemState.encoder.pulse_count++;
                    g_systemState.encoder.speed_pulse_count++;
                    
                    // 방향 잠금 메커니즘 (반시계방향) - 시계방향에서 전환 시에는 잠금 설정하지 않음
                    if (g_systemState.encoder.direction_lock == 2)
                    {
                        g_systemState.encoder.same_direction_count++;
                        // 31방향이 계속되면 치환 카운터 리셋 (정상 동작으로 판단)
                        if (g_systemState.encoder.same_direction_count >= 2)
                        {
                            g_systemState.encoder.correction_count = 0;
                        }
                    }
                    else if (g_systemState.encoder.last_valid_direction != 1)
                    {
                        // 이전 방향이 시계방향이 아닐 때만 반시계방향 잠금 설정
                        g_systemState.encoder.direction_lock = 2;
                        g_systemState.encoder.direction_lock_time = current_time;
                        g_systemState.encoder.same_direction_count = 1;
                        g_systemState.encoder.correction_count = 0; // 새로운 31방향 잠금 시작 시 치환 카운터 리셋
                    }
                    // 시계방향에서 반시계방향으로 전환 시에는 잠금 설정하지 않음
                }
            }
        }
        
        // 타임아웃 처리 (기존 로직 기반)
        if ((current_time - g_systemState.encoder.last_action_time) > 250)
        {
            g_systemState.encoder.last_valid_direction = 0;
            g_systemState.encoder.last_action_time = current_time;
        }
    }
    
    // 상태 업데이트
    g_systemState.encoder.lastStateCLK = g_systemState.encoder.currentStateCLK;
    
    // 속도 계산 (주기적으로)
    if ((current_time - g_systemState.encoder.speed_calc_time) >= ENCODER_SPEED_SAMPLE_PERIOD_MS)
    {
        uint32_t time_diff = current_time - g_systemState.encoder.speed_calc_time;
        
        if (time_diff > 0)
        {
            // 속도 계산: counts per second
            g_systemState.encoder.speed = (g_systemState.encoder.speed_pulse_count * 1000) / time_diff;
            
            // 다음 계산을 위해 리셋
            g_systemState.encoder.speed_pulse_count = 0;
            g_systemState.encoder.speed_calc_time = current_time;
        }
    }
    
    // 속도 타임아웃 처리
    if ((current_time - g_systemState.encoder.last_change_time) > ENCODER_SPEED_TIMEOUT_MS)
    {
        g_systemState.encoder.speed = 0;
    }
}

/**
 * @brief 엔코더 카운트 값 반환
 */
int32_t GetEncoderCount(void)
{
    return g_systemState.encoder.count;
}

/**
 * @brief 엔코더 방향 반환
 */
uint8_t GetEncoderDirection(void)
{
    return g_systemState.encoder.direction;
}

/**
 * @brief 엔코더 속도 반환
 */
int32_t GetEncoderSpeed(void)
{
    return g_systemState.encoder.speed;
}

/**
 * @brief 엔코더 카운트 리셋
 */
void ResetEncoderCount(void)
{
    g_systemState.encoder.count = 0;
    g_systemState.encoder.speed = 0;
    g_systemState.encoder.pulse_count = 0;
    g_systemState.encoder.speed_pulse_count = 0;
    g_systemState.encoder.consecutive_count = 0;
    g_systemState.encoder.last_pulse_time = HAL_GetTick();
    g_systemState.encoder.last_direction_delta = 0;
    g_systemState.encoder.direction_lock = 0;
    g_systemState.encoder.direction_lock_time = HAL_GetTick();
    g_systemState.encoder.same_direction_count = 0;
    g_systemState.encoder.cw_confidence = 0;
    g_systemState.encoder.ccw_confidence = 0;
    g_systemState.encoder.dominant_direction = 0;
    g_systemState.encoder.correction_count = 0;
}

/**
 * @brief 엔코더 마지막 변화 시간 반환
 */
uint32_t GetEncoderLastChangeTime(void)
{
    return g_systemState.encoder.last_change_time;
}

/**
 * @brief 엔코더 펄스 카운트 반환 및 리셋
 */
int32_t GetAndResetEncoderPulseCount(void)
{
    int32_t count = g_systemState.encoder.pulse_count;
    g_systemState.encoder.pulse_count = 0;
    return count;
}

// 기존 로터리 인코더 처리 함수 (호환성 유지)
void ProcessEncoder(void)
{
    g_systemState.encoder.currentStateCLK = HAL_GPIO_ReadPin(Dial_B_GPIO_Port, Dial_B_Pin);
    if (g_systemState.encoder.currentStateCLK != g_systemState.encoder.lastStateCLK)
    {
        uint8_t dtValue = HAL_GPIO_ReadPin(Dial_A_GPIO_Port, Dial_A_Pin);

        if (dtValue != g_systemState.encoder.currentStateCLK && g_systemState.encoder.currentStateCLK == 1)
        {
            // 시계방향 회전
            HAL_GPIO_WritePin(Dial_LED_2_GPIO_Port, Dial_LED_2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Dial_LED_3_GPIO_Port, Dial_LED_3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Dial_LED_1_GPIO_Port, Dial_LED_1_Pin, GPIO_PIN_RESET);
            g_systemState.comm.sendData[6] = 0x32;

            g_systemState.comm.sendData[8] = _get_trigger_out_state(g_systemState.triggers.trigger_out1, g_systemState.triggers.trigger_out2);
            Pad_calculate_crc8();
            g_systemState.comm.sendData[9] = g_systemState.comm.crc1;
            g_systemState.comm.sendData[10] = g_systemState.comm.crc2;
            CDC_Transmit_FS((uint8_t *)g_systemState.comm.sendData, sizeof(g_systemState.comm.sendData));
            g_systemState.comm.sendData[6] = 0x30;
            g_systemState.timers.ledTimer = 0;
            g_systemState.timers.ledCount = 0;
            g_systemState.encoder.last_valid_direction = 1;
            g_systemState.encoder.last_action_time = HAL_GetTick();
        }
        else if (dtValue == 0 && g_systemState.encoder.currentStateCLK == 0)
        {
            g_systemState.encoder.dtState = 1;
        }
        else if (g_systemState.encoder.dtState == 1 && dtValue == 1 && g_systemState.encoder.currentStateCLK == 1 && HAL_GetTick() - g_systemState.encoder.last_action_time < 150)
        {

            if (g_systemState.encoder.last_valid_direction != 1 || (HAL_GetTick() - g_systemState.encoder.last_action_time) > 250)
            {
                // 반시계방향 회전
                HAL_GPIO_WritePin(Dial_LED_1_GPIO_Port, Dial_LED_1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(Dial_LED_2_GPIO_Port, Dial_LED_2_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(Dial_LED_3_GPIO_Port, Dial_LED_3_Pin, GPIO_PIN_RESET);
                g_systemState.comm.sendData[6] = 0x31;

                g_systemState.comm.sendData[8] = _get_trigger_out_state(g_systemState.triggers.trigger_out1, g_systemState.triggers.trigger_out2);
                Pad_calculate_crc8();
                g_systemState.comm.sendData[9] = g_systemState.comm.crc1;
                g_systemState.comm.sendData[10] = g_systemState.comm.crc2;
                CDC_Transmit_FS((uint8_t *)g_systemState.comm.sendData, sizeof(g_systemState.comm.sendData));
                g_systemState.comm.sendData[6] = 0x30;
                g_systemState.timers.ledTimer = 0;
                g_systemState.timers.ledCount = 0;
                g_systemState.encoder.dtState = 0;
                g_systemState.encoder.last_valid_direction = 2;
                g_systemState.encoder.last_action_time = HAL_GetTick();
            }
            else
            {
            }
        }

        if (HAL_GetTick() - g_systemState.encoder.last_action_time > 250)
        {
            g_systemState.encoder.last_valid_direction = 0;
            g_systemState.encoder.last_action_time = HAL_GetTick();
        }
    }
    g_systemState.encoder.lastStateCLK = g_systemState.encoder.currentStateCLK;
}

// LED 상태 처리 함수
void ProcessLEDState(void)
{
    if (g_systemState.timers.ledCount == 70)
    {
        HAL_GPIO_WritePin(Dial_LED_1_GPIO_Port, Dial_LED_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Dial_LED_2_GPIO_Port, Dial_LED_2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Dial_LED_3_GPIO_Port, Dial_LED_3_Pin, GPIO_PIN_SET);
        if (g_systemState.timers.ledCount == 100)
        {
            g_systemState.timers.ledCount = 0;
        }
    }
}

// 트리거 처리 함수
__attribute__((optimize("O0"))) void ProcessTriggers(void)
{
    bool send_packet = false;
    uint8_t active_trigger_num = 0;

    // 트리거 1 처리
    // CPC1017 릴레이에 의해 0 -> 1 하강엣지 트리거 | 1 -> 0 상승엣지 트리거로 반전되어 인식한다
    // 1 = LOW , 0 = HIGH | 0->1 하강엣지 트리거 | 1->0 상승엣지 트리거
    switch (g_systemState.triggers.trigger_in1)
    {
    case 1:
        if ((g_systemState.triggers.trigger_in1 != g_systemState.triggers.trigger_in_Old1) &&
            (g_systemState.triggers.trigger_in_setup_1 == ALL_EDGE || g_systemState.triggers.trigger_in_setup_1 == HIGH_EDGE))
        {
            g_systemState.comm.sendData[7] = 0x32; // TR1 폴링
            send_packet = true;
            active_trigger_num = 1;
        }
        else if (g_systemState.triggers.trigger_in1 != g_systemState.triggers.trigger_in_Old1)
        {
            g_systemState.triggers.trigger_in_Old1 = g_systemState.triggers.trigger_in1;
        }
        break;

    case 0:
        if ((g_systemState.triggers.trigger_in1 != g_systemState.triggers.trigger_in_Old1) &&
            (g_systemState.triggers.trigger_in_setup_1 == ALL_EDGE || g_systemState.triggers.trigger_in_setup_1 == LOW_EDGE))
        {
            g_systemState.comm.sendData[7] = 0x31; // TR1 라이징
            send_packet = true;
            active_trigger_num = 1;
        }
        else if (g_systemState.triggers.trigger_in1 != g_systemState.triggers.trigger_in_Old1)
        {
            g_systemState.triggers.trigger_in_Old1 = g_systemState.triggers.trigger_in1;
        }
        break;
    }

    // 트리거 2 처리 (트리거 1에서 이미 패킷이 준비된 경우에는 스킵)
    if (send_packet == false)
    {
        switch (g_systemState.triggers.trigger_in2)
        {
        case 1: // 하강엣지 트리거
            if (g_systemState.triggers.trigger_in2 != g_systemState.triggers.trigger_in_Old2 &&
                (g_systemState.triggers.trigger_in_setup_2 == ALL_EDGE || g_systemState.triggers.trigger_in_setup_2 == HIGH_EDGE))
            {
                g_systemState.comm.sendData[7] = 0x34;
                send_packet = true;
                active_trigger_num = 2;
            }
            else if (g_systemState.triggers.trigger_in2 != g_systemState.triggers.trigger_in_Old2)
            {
                g_systemState.triggers.trigger_in_Old2 = g_systemState.triggers.trigger_in2;
            }
            break;

        case 0: // 상승엣지 트리거
            if (g_systemState.triggers.trigger_in2 != g_systemState.triggers.trigger_in_Old2 &&
                (g_systemState.triggers.trigger_in_setup_2 == ALL_EDGE || g_systemState.triggers.trigger_in_setup_2 == LOW_EDGE))
            {
                g_systemState.comm.sendData[7] = 0x33;
                send_packet = true;
                active_trigger_num = 2;
            }
            else if (g_systemState.triggers.trigger_in2 != g_systemState.triggers.trigger_in_Old2)
            {
                g_systemState.triggers.trigger_in_Old2 = g_systemState.triggers.trigger_in2;
            }
            break;
        }
    }

    if (send_packet == true)
    {
        send_packet = false;
        g_systemState.comm.sendData[8] = _get_trigger_out_state(g_systemState.triggers.trigger_out1, g_systemState.triggers.trigger_out2);
        Pad_calculate_crc8();
        g_systemState.comm.sendData[9] = g_systemState.comm.crc1;
        g_systemState.comm.sendData[10] = g_systemState.comm.crc2;
        CDC_Transmit_FS((uint8_t *)g_systemState.comm.sendData, sizeof(g_systemState.comm.sendData));

        g_systemState.comm.sendData[7] = 0x30;

        if (active_trigger_num == 1)
        {
            g_systemState.triggers.trigger_in_Old1 = g_systemState.triggers.trigger_in1;
        }
        else if (active_trigger_num == 2)
        {
            g_systemState.triggers.trigger_in_Old2 = g_systemState.triggers.trigger_in2;
        }
    }
}

uint8_t _get_trigger_out_state(bool trigger_out1, bool trigger_out2)
{
    return 0x30 + trigger_out1 + (trigger_out2 << 1);
}

static void _long_buttom_buzzer(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_Delay(150);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}