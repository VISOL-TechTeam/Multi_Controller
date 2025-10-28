#include "gpio.h"

extern uint32_t Buzzer_timer;
extern uint8_t Long_Buzzer;

static void _long_buttom_buzzer(void);

// 공통 버튼 처리 함수
inline void ProcessButtonPress(uint8_t buttonPressed, uint8_t expectedState, uint8_t commandCode, uint8_t dataIndex)
{
    if (buttonPressed == 1 && g_systemState.state == expectedState)
    {
        g_systemState.comm.sendData[dataIndex] = commandCode;

        g_systemState.comm.sendData[8] = GetTriggerOutState(g_systemState.triggers.trigger_out1, g_systemState.triggers.trigger_out2);
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
inline void ProcessMemoryLongPress(uint8_t commandCode)
{
    if (g_systemState.timers.longKeycount == 1)
    {
        g_systemState.comm.sendData[5] = commandCode;

        g_systemState.comm.sendData[8] = GetTriggerOutState(g_systemState.triggers.trigger_out1, g_systemState.triggers.trigger_out2);
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
inline void ProcessUpButtonLongPress(void)
{
    if (g_systemState.timers.longKeycount == 1)
    {
        g_systemState.comm.sendData[4] = 0x50;

        g_systemState.comm.sendData[8] = GetTriggerOutState(g_systemState.triggers.trigger_out1, g_systemState.triggers.trigger_out2);
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
inline void SetButtonState(uint8_t buttonPressed, uint8_t stateValue)
{
    if (buttonPressed == 0 && g_systemState.timers.keyCount > 70 && g_systemState.timers.keyCount < 3000)
    {
        g_systemState.state = stateValue;
        g_systemState.timers.longKey = 0;
        g_systemState.timers.longKeycount = 0;
    }
}

// 트리거 처리 함수
inline void ProcessTriggers(void)
{
    bool send_packet = false;
    uint8_t active_trigger_num = 0;

    // 트리거 1 처리
    // CPC1017 릴레이에 의해 0 -> 1 하강엣지 트리거 | 1 -> 0 상승엣지 트리거로 반전되어 인식한다
    switch (g_systemState.triggers.trigger_in1)
    {
    case 1: // 1 = LOW , 0 = HIGH | 0->1 하강엣지 트리거 | 1->0 상승엣지 트리거
        if (g_systemState.triggers.trigger_in1 != g_systemState.triggers.trigger_in_Old1 &&
            (g_systemState.triggers.trigger_in_setup_1 == ALL_EDGE || g_systemState.triggers.trigger_in_setup_1 == LOW_EDGE))
        {
            g_systemState.comm.sendData[7] = 0x32; // TR1 폴링
            send_packet = true;
            active_trigger_num = 1;
        }
        break;

    case 0: // 상승엣지 트리거
        if (g_systemState.triggers.trigger_in1 != g_systemState.triggers.trigger_in_Old1 &&
            (g_systemState.triggers.trigger_in_setup_1 == ALL_EDGE || g_systemState.triggers.trigger_in_setup_1 == HIGH_EDGE))
        {
            g_systemState.comm.sendData[7] = 0x31; // TR1 라이징
            send_packet = true;
            active_trigger_num = 1;
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
                (g_systemState.triggers.trigger_in_setup_2 == ALL_EDGE || g_systemState.triggers.trigger_in_setup_2 == LOW_EDGE))
            {
                g_systemState.comm.sendData[7] = 0x34;
                send_packet = true;
                active_trigger_num = 2;
            }
            break;

        case 0: // 상승엣지 트리거
            if (g_systemState.triggers.trigger_in2 != g_systemState.triggers.trigger_in_Old2 &&
                (g_systemState.triggers.trigger_in_setup_2 == ALL_EDGE || g_systemState.triggers.trigger_in_setup_2 == HIGH_EDGE))
            {
                g_systemState.comm.sendData[7] = 0x33;
                send_packet = true;
                active_trigger_num = 2;
            }
            break;
        }
    }

    if (send_packet == true)
    {
        send_packet = false;
        g_systemState.comm.sendData[8] = GetTriggerOutState(g_systemState.triggers.trigger_out1, g_systemState.triggers.trigger_out2);
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

inline uint8_t GetTriggerOutState(bool trigger_out1, bool trigger_out2)
{
    return 0x30 + trigger_out1 + (trigger_out2 << 1);
}

static inline void _long_buttom_buzzer(void)
{
    if (g_systemState.enable_buzzer == 1)
    {
        Long_Buzzer = 1;
        Buzzer_timer = 0;
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    }
}