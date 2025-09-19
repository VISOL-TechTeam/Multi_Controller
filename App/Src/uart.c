#include "uart.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "util.h"

extern UART_HandleTypeDef huart1; // UART1 핸들

// 데이터 수집 관련 변수들
static uint8_t collectBuffer[2048]; // 데이터 수집 버퍼
static uint16_t collectIndex = 0;   // 데이터 수집 인덱스
static uint8_t isCollecting = 0;    // 데이터 수집 상태 플래그

int USB_minipc(void)
{
    if (gGlobal_usbToggle == 1)
    {
        // 데이터 수집 시작 또는 계속 수집
        if (!isCollecting)
        {
            isCollecting = 1;
            collectIndex = 0;
        }

        // 현재 받은 데이터를 수집 버퍼에 추가 (버퍼 오버플로우 방지 강화)
        for (uint32_t i = 0; i < gGlobal_usbLen && collectIndex < sizeof(collectBuffer) - 1; i++)
        {
            collectBuffer[collectIndex++] = gGlobal_Buffer[i];
        }

        if (collectBuffer[0] == 0x02 && collectBuffer[collectIndex - 1] == 0x03)
        {
            if (collectBuffer[1] == 0xA4)
            {
                calculate_crc8();
                if (collectBuffer[collectIndex - 3] == gGlobal_crc1 && collectBuffer[collectIndex - 2] == gGlobal_crc2)
                {
                    if (collectBuffer[8] == 0x31)
                    {
                        // gGlobal_ledState = 0;
                        // TriggerPin_2();
                        TriggerPin_1();
                    }
                    if (collectBuffer[8] == 0x32)
                    {
                        // gGlobal_ledState = 1;
                        TriggerPin_2();
                    }
                }
                else if (collectBuffer[collectIndex - 3] != gGlobal_crc1 || collectBuffer[collectIndex - 2] != gGlobal_crc2)
                {
                    // 버퍼 초기화 및 수집 상태 리셋
                    memset(collectBuffer, 0, sizeof(collectBuffer));
                    collectIndex = 0;
                    isCollecting = 0;
                }
            }
            else if (collectBuffer[1] == 0xA3)
            {
                // 트리거 설정 명령 수신
                if (collectBuffer[4] == 0x31)
                {
                    g_systemState.triggers.trigger_in_setup_1 = 1; // 하강엣지 트리거
                }
                else if (collectBuffer[4] == 0x32)
                {
                    g_systemState.triggers.trigger_in_setup_1 = 2; // 상승엣지 트리거
                }

                if (collectBuffer[5] == 0x31)
                {
                    g_systemState.triggers.trigger_in_setup_2 = 1; // 하강엣지 트리거
                }
                else if (collectBuffer[5] == 0x32)
                {
                    g_systemState.triggers.trigger_in_setup_2 = 2; // 상승엣지 트리거
                }

                if (collectBuffer[6] == 0x31)
                {
                    HAL_GPIO_WritePin(Trigger_OUT_1_GPIO_Port, Trigger_OUT_1_Pin, GPIO_PIN_SET);
                    g_systemState.triggoutState1 = 1; // HIGH 출력
                }
                else if (collectBuffer[6] == 0x32)
                {
                    HAL_GPIO_WritePin(Trigger_OUT_1_GPIO_Port, Trigger_OUT_1_Pin, GPIO_PIN_RESET);
                    g_systemState.triggoutState1 = 2; // LOW 출력
                }

                if (collectBuffer[7] == 0x31)
                {
                    HAL_GPIO_WritePin(Trigger_OUT_2_GPIO_Port, Trigger_OUT_2_Pin, GPIO_PIN_SET);
                    g_systemState.triggoutState2 = 1; // HIGH 출력
                }
                else if (collectBuffer[7] == 0x32)
                {
                    HAL_GPIO_WritePin(Trigger_OUT_2_GPIO_Port, Trigger_OUT_2_Pin, GPIO_PIN_RESET);
                    g_systemState.triggoutState2 = 2; // LOW 출력
                }
                // TriggerPin_1();
                CDC_Transmit_FS(collectBuffer, collectIndex);
                collectIndex = 0;
                isCollecting = 0;
            }
            else if (collectBuffer[collectIndex - 1] == 0x03 && collectBuffer[1] != 0xA4) // collectBuffer[i]
            {
                // 종료 문자를 찾았으므로 UART로 전송
                HAL_UART_Transmit(&huart1, collectBuffer, collectIndex, 10000);
                // sprintf(testbuf, "%s", collectIndex);
                // CDC_Transmit_FS(testbuf, collectIndex);
                // 버퍼 초기화 및 수집 상태 리셋
                memset(collectBuffer, 0, sizeof(collectBuffer));
                collectIndex = 0;
                isCollecting = 0;
            }
            else if (collectBuffer[collectIndex - 1] == 0x03 && collectBuffer[1] == 0xA4) // collectBuffer[i]
            {
                // 종료 문자를 찾았으므로 UART로 전송, 버퍼 초기화 및 수집 상태 리셋
                memset(collectBuffer, 0, sizeof(collectBuffer));
                collectIndex = 0;
                isCollecting = 0;
            }
            else if (collectBuffer[collectIndex - 1] != 0x03) // collectBuffer[i]
            {
                // 버퍼 초기화 및 수집 상태 리셋
                memset(collectBuffer, 0, sizeof(collectBuffer));
                collectIndex = 0;
                isCollecting = 0;
            }
        }
        else if (collectBuffer[0] != 0x02)
        {
            memset(collectBuffer, 0, sizeof(collectBuffer));
            collectIndex = 0;
            isCollecting = 0;
        }
        gGlobal_usbToggle = 0;
    }
    return 0;
}

void RX_DATA(void)
{
    CDC_Transmit_FS((uint8_t *)&gGlobal_rxData, 1);
}
