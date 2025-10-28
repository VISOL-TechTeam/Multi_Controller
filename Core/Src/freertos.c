/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "cmsis_os.h"
#include "usb_device.h"
#include "encoder.h"
#include "def.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for encoderTask */
osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes = {
    .name = "encoderTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for uartTask */
osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes = {
    .name = "uartTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for buttonTask */
osThreadId_t buttonTaskHandle;
const osThreadAttr_t buttonTask_attributes = {
    .name = "buttonTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

// 전역 시스템 상태 변수 정의
SystemState_t g_systemState = {0};

extern UART_HandleTypeDef huart1;

extern uint8_t gGlobal_usbToggle;
extern uint8_t gGlobal_Buffer[2048];
extern uint32_t gGlobal_usbLen;

extern uint32_t Buzzer_timer;
extern uint32_t Buzzer_count;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartDefaultTask(void *argument);
void StartEncoderTask(void *argument);
void StartUartTask(void *argument);
void StartButtonTask(void *argument);

static int TriggerPin_1(void);
static int TriggerPin_2(void);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void RTOS_Init(void)
{
  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of encoderTask */
  encoderTaskHandle = osThreadNew(StartEncoderTask, NULL, &encoderTask_attributes);

  /* creation of uartTask */
  uartTaskHandle = osThreadNew(StartUartTask, NULL, &uartTask_attributes);

  /* creation of buttonTask */
  buttonTaskHandle = osThreadNew(StartButtonTask, NULL, &buttonTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  UNUSED(argument);
  /* init code for USB_DEVICE */
  /* USER CODE BEGIN 5 */

  /* USER CODE BEGIN StartDefaultTask */
  static int32_t old_encoder_count = 0;
  int32_t current_encoder_count = 0;
  uint32_t current_time = 0;

  // 카운터 안정화를 위한 변수들
  static int32_t pending_count_change = 0; // 누적된 카운트 변화
  static uint32_t stable_timer_start = 0;  // 마지막 엔코더 변화 시간

  /* Infinite loop */
  for (;;)
  {
    if (g_systemState.comm.sendData[0] != 0x02)
    { // 통신 데이터 초기화
      g_systemState.comm.sendData[0] = 0x02;
      g_systemState.comm.sendData[1] = 0xA4;
      g_systemState.comm.sendData[2] = 0x32;
      g_systemState.comm.sendData[3] = 0x34;
      g_systemState.comm.sendData[4] = 0x30;  // key
      g_systemState.comm.sendData[5] = 0x30;  // key
      g_systemState.comm.sendData[6] = 0x30;  // dial
      g_systemState.comm.sendData[7] = 0x30;  // trigger
      g_systemState.comm.sendData[8] = 0x30;  // trigger
      g_systemState.comm.sendData[9] = 0x30;  // crc
      g_systemState.comm.sendData[10] = 0x30; // crc
      g_systemState.comm.sendData[11] = 0x03;
    }

    current_time = HAL_GetTick();

    // 엔코더 값 가져오기
    current_encoder_count = Encoder_GetCount();

    // 엔코더 카운트 변화 감지 및 누적
    if (current_encoder_count != old_encoder_count)
    {
      int32_t count_diff = current_encoder_count - old_encoder_count;
      pending_count_change += count_diff;
      stable_timer_start = current_time;
      old_encoder_count = current_encoder_count;
    }

    // 누적된 변화 처리 (디밍 잠금 상태가 아닐 때만)
    if (pending_count_change != 0)
    {
      // 누적된 변화가 4 이상일 때만 처리
      if (abs(pending_count_change) > 3)
      {
        // 디밍 값 조정
        if (pending_count_change > 0)
        {
          // uint8_t dimming_temp = (Dimming + step > 100) ? 100 : Dimming + step;
          // Dimming = dimming_temp % 5 != 0 ? (dimming_temp / 5 * 5) : dimming_temp;

          // 반시계방향 회전
          HAL_GPIO_WritePin(Dial_LED_1_GPIO_Port, Dial_LED_1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(Dial_LED_2_GPIO_Port, Dial_LED_2_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(Dial_LED_3_GPIO_Port, Dial_LED_3_Pin, GPIO_PIN_RESET);
          g_systemState.comm.sendData[6] = 0x31;

          g_systemState.comm.sendData[8] = GetTriggerOutState(g_systemState.triggers.trigger_out1, g_systemState.triggers.trigger_out2);
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

          pending_count_change -= 4;
        }
        else
        {
          // uint8_t dimming_temp = (Dimming < step) ? 0 : Dimming - step;
          // Dimming = dimming_temp % 5 != 0 ? (dimming_temp / 5 * 5) : dimming_temp;
          // 시계방향 회전

          HAL_GPIO_WritePin(Dial_LED_2_GPIO_Port, Dial_LED_2_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(Dial_LED_3_GPIO_Port, Dial_LED_3_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(Dial_LED_1_GPIO_Port, Dial_LED_1_Pin, GPIO_PIN_RESET);
          g_systemState.comm.sendData[6] = 0x32;

          g_systemState.comm.sendData[8] = GetTriggerOutState(g_systemState.triggers.trigger_out1, g_systemState.triggers.trigger_out2);
          Pad_calculate_crc8();
          g_systemState.comm.sendData[9] = g_systemState.comm.crc1;
          g_systemState.comm.sendData[10] = g_systemState.comm.crc2;
          CDC_Transmit_FS((uint8_t *)g_systemState.comm.sendData, sizeof(g_systemState.comm.sendData));
          g_systemState.comm.sendData[6] = 0x30;
          g_systemState.timers.ledTimer = 0;
          g_systemState.timers.ledCount = 0;
          g_systemState.encoder.last_valid_direction = 1;
          g_systemState.encoder.last_action_time = HAL_GetTick();

          pending_count_change += 4;
        }
      }
      else if ((current_time - stable_timer_start) > 200)
      {
        pending_count_change = 0; // 타임아웃으로 리셋
        Encoder_ResetCount();
      }
    }
    else
    {
      pending_count_change = 0; // 타임아웃으로 리셋
      Encoder_ResetCount();
    }

    // LED 상태 변화
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
    osDelay(5);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartEncoderTask */
/**
 * @brief Function implementing the encoderTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartEncoderTask */
void StartEncoderTask(void *argument)
{
  UNUSED(argument);
  /* USER CODE BEGIN StartEncoderTask */
  /* Infinite loop */

  // 외부 디밍 잠금 변수들 선언

  // 4x 디코딩을 위한 변수들
  uint8_t encoder_state = 0;
  uint8_t last_encoder_state = 0;
  uint32_t current_time;

  // 엔코더 상태 변수들
  static volatile int32_t encoder_count = 0;
  static volatile uint8_t encoder_direction = 0;
  static volatile uint32_t encoder_last_change_time = 0;

  // 4x 디코딩 테이블
  static const int8_t encoder_table[16] = {
      0, -1, 1, 0, // 00의 이전 상태에서 올 때
      1, 0, 0, -1, // 01의 이전 상태에서 올 때
      -1, 0, 0, 1, // 10의 이전 상태에서 올 때
      0, 1, -1, 0  // 11의 이전 상태에서 올 때
  };

  // 초기 상태 설정
  osDelay(10);
  encoder_state = (HAL_GPIO_ReadPin(Dial_A_GPIO_Port, Dial_A_Pin) << 1) |
                  HAL_GPIO_ReadPin(Dial_B_GPIO_Port, Dial_B_Pin);
  last_encoder_state = encoder_state;
  /* Infinite loop */
  for (;;)
  {

    current_time = HAL_GetTick();

    // GPIO 상태 읽기
    encoder_state = (HAL_GPIO_ReadPin(Dial_A_GPIO_Port, Dial_A_Pin) << 1) |
                    HAL_GPIO_ReadPin(Dial_B_GPIO_Port, Dial_B_Pin);

    // 상태 변경 감지 및 처리
    if (encoder_state != last_encoder_state)
    {
      int8_t direction_delta = encoder_table[(last_encoder_state << 2) | encoder_state];

      if (direction_delta != 0)
      {
        // 엔코더 카운트 및 방향 업데이트
        encoder_count += direction_delta;
        encoder_direction = (direction_delta > 0) ? 0 : 1;

        // 순환 범위 제한 (-999 ~ +999)
        if (encoder_count > ENCODER_COUNT_MAX)
          encoder_count = 0;
        else if (encoder_count < ENCODER_COUNT_MIN)
          encoder_count = 0;

        // 타임스탬프 업데이트 및 펄스 카운트 증가
        encoder_last_change_time = current_time;
        Encoder_AddPulseCount(1);
      }

      last_encoder_state = encoder_state;
    }

    // 전역 엔코더 값 업데이트
    Encoder_SetCount(encoder_count);
    Encoder_SetDirection(encoder_direction);
    Encoder_SetLastChangeTime(encoder_last_change_time);
    osDelay(1); // 1ms 주기로 실행
  }
  /* USER CODE END StartEncoderTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
 * @brief Function implementing the uartTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  UNUSED(argument);
  /* USER CODE BEGIN StartUartTask */
  // 데이터 수집 관련 변수들
  uint8_t collectBuffer[2048]; // 데이터 수집 버퍼
  uint16_t collectIndex = 0;   // 데이터 수집 인덱스
  uint8_t isCollecting = 0;    // 데이터 수집 상태 플래그
  /* Infinite loop */
  for (;;)
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
          uint8_t chk = 0;
          chk += collectBuffer[1];
          chk += collectBuffer[2];
          chk += collectBuffer[3];
          chk += collectBuffer[4];
          chk += collectBuffer[5];
          chk += collectBuffer[6];
          chk += collectBuffer[7];
          if (hextoascii(chk >> 4 & 0x0F) == collectBuffer[8] && hextoascii(chk & 0x0F) == collectBuffer[9])
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
            else if (collectBuffer[4] == 0x30)
            {
              g_systemState.triggers.trigger_in_setup_1 = 0; // 모든 엣지 감지지
            }

            if (collectBuffer[5] == 0x31)
            {
              g_systemState.triggers.trigger_in_setup_2 = 1; // 하강엣지 트리거
            }
            else if (collectBuffer[5] == 0x32)
            {
              g_systemState.triggers.trigger_in_setup_2 = 2; // 상승엣지 트리거
            }
            else if (collectBuffer[5] == 0x30)
            {
              g_systemState.triggers.trigger_in_setup_2 = 0; // 모든 엣지 감지지
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

            if (collectBuffer[4] == 0xFF && collectBuffer[5] == 0xFF && collectBuffer[6] == 0xFF && collectBuffer[7] == 0xFF)
            {
              g_systemState.enable_buzzer = 0;
            }
            else if (collectBuffer[4] == 0xFE && collectBuffer[5] == 0xFE && collectBuffer[6] == 0xFE && collectBuffer[7] == 0xFE)
            {
              g_systemState.enable_buzzer = 1;
            }
            // TriggerPin_1();
            collectIndex = 0;
            isCollecting = 0;
          }
          else
          {
            collectIndex = 0;
            isCollecting = 0;
          }
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
    osDelay(50);
  }
  /* USER CODE END StartUartTask */
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

void StartButtonTask(void *argument)
{
  UNUSED(argument);
  /* USER CODE BEGIN StartButtonTask */

  g_systemState.enable_buzzer = 1;
  /* Infinite loop */
  for (;;)
  {

    // GPIO 입력 상태 업데이트
    g_systemState.triggers.trigger_in1 = HAL_GPIO_ReadPin(Trigger_IN_1_GPIO_Port, Trigger_IN_1_Pin);
    g_systemState.triggers.trigger_in2 = HAL_GPIO_ReadPin(Trigger_IN_2_GPIO_Port, Trigger_IN_2_Pin);
    g_systemState.triggers.trigger_out1 = HAL_GPIO_ReadPin(Trigger_OUT_1_GPIO_Port, Trigger_OUT_1_Pin);
    g_systemState.triggers.trigger_out2 = HAL_GPIO_ReadPin(Trigger_OUT_2_GPIO_Port, Trigger_OUT_2_Pin);
    g_systemState.buttons.upButton = HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin);
    g_systemState.buttons.downButton = HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin);
    g_systemState.buttons.powerButton = HAL_GPIO_ReadPin(POWRAY_ON_GPIO_Port, POWRAY_ON_Pin);
    g_systemState.buttons.boostButton = HAL_GPIO_ReadPin(BOOST_GPIO_Port, BOOST_Pin);
    g_systemState.buttons.pcmodeButton = HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin);
    g_systemState.buttons.memory1Button = HAL_GPIO_ReadPin(Memory_1_GPIO_Port, Memory_1_Pin);
    g_systemState.buttons.memory2Button = HAL_GPIO_ReadPin(Memory_2_GPIO_Port, Memory_2_Pin);
    g_systemState.buttons.memory3Button = HAL_GPIO_ReadPin(Memory_3_GPIO_Port, Memory_3_Pin);
    g_systemState.buttons.memory4Button = HAL_GPIO_ReadPin(Memory_4_GPIO_Port, Memory_4_Pin);

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
    }

    // 부저 처리
    if ((g_systemState.state != BUTTON_STATE_IDLE && (g_systemState.timers.keyCount > 50 && g_systemState.timers.keyCount < 100)) && g_systemState.enable_buzzer == 1)
    {
      Buzzer_timer = 0;
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    }

    ProcessTriggers();

    osDelay(10);
  }
  /* USER CODE END StartButtonTask */
}

/* USER CODE END Application */
