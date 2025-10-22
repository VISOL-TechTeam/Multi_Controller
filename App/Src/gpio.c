#include "gpio.h"
#include "main.h"
#include "usbd_cdc_if.h"

// 전역 시스템 상태 변수 정의
SystemState_t g_systemState = {0};

// 외부 함수 프로토타입
extern uint8_t Pad_calculate_crc8(void);
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

uint8_t _get_trigger_out_state(bool trigger_out1, bool trigger_out2);

// 시스템 상태 초기화
void InitSystemState(void)
{
    // 모든 상태를 0으로 초기화 (이미 위에서 {0}으로 초기화됨)
    // 필요시 특별한 초기값 설정
    g_systemState.encoder.lastStateCLK = HAL_GPIO_ReadPin(Dial_B_GPIO_Port, Dial_B_Pin);
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
    }
}

// 버튼 상태 설정 함수
__attribute__((optimize("O0"))) static void SetButtonState(uint8_t buttonPressed, uint8_t stateValue)
{
    if (buttonPressed == 0 && g_systemState.timers.keyCount > 70 && g_systemState.timers.keyCount < 3000)
    {
        g_systemState.state = stateValue;
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

// 로터리 인코더 처리 함수
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
