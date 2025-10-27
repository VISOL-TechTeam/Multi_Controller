/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 메인 프로그램
 * @description    : STM32F103 기반의 멀티 컨트롤러 프로그램
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
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h" // CDC_Transmit_FS 함수 선언을 위해 추가
#include "def.h"		 // App 폴더의 모든 기능 함수들 포함
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// USB를 통한 데이터 전송을 위한 write 함수 재정의
int _write(int fd, char *str, int len)
{
	UNUSED(fd);
	CDC_Transmit_FS((uint8_t *)str, len); // USB를 통해 전송
	return len;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// 버튼 및 상태 변수들
uint8_t gGlobal_Buffer[2048];		// USB 통신용 버퍼
uint8_t gGlobal_usbToggle;			// USB 토글 상태
uint32_t gGlobal_usbLen;			// USB 데이터 길이
uint8_t gGlobal_upButton = 0;		// 위 버튼 상태
uint8_t gGlobal_downButton = 0;		// 아래 버튼 상태
uint8_t gGlobal_powerButton = 0;	// 전원 버튼 상태
uint8_t gGlobal_boostButton = 0;	// 부스트 버튼 상태
uint8_t gGlobal_pcmodeButton = 0;	// PC 모드 버튼 상태
uint8_t gGlobal_memory1Button = 0;	// 메모리 1 버튼 상태
uint8_t gGlobal_memory2Button = 0;	// 메모리 2 버튼 상태
uint8_t gGlobal_memory3Button = 0;	// 메모리 3 버튼 상태
uint8_t gGlobal_memory4Button = 0;	// 메모리 4 버튼 상태
uint8_t gGlobal_triggIn1 = 0;		// 트리거 입력 1 상태
uint8_t gGlobal_triggIn2 = 0;		// 트리거 입력 2 상태
uint8_t gGlobal_trigginState1 = 0;	// 트리거 입력 1 설정 상태 (0: 미설정, 1: 하강엣지, 2: 상승엣지)
uint8_t gGlobal_trigginState2 = 0;	// 트리거 입력 2 설정 상태 (0: 미설정, 1: 하강엣지, 2: 상승엣지)
uint8_t gGlobal_triggoutState1 = 0; // 트리거 출력 1 상태 (0: 미설정, 1: HIGH, 2: LOW)
uint8_t gGlobal_triggoutState2 = 0; // 트리거 출력 2 상태 (0: 미설정, 1: HIGH, 2: LOW)
uint8_t gGlobal_state = 0;			// 버튼 처리 상태
uint8_t gGlobal_sum;				// CRC 계산용 변수
uint8_t gGlobal_sum1;				// CRC 계산용 변수
uint8_t gGlobal_sum2;				// CRC 계산용 변수
uint8_t gGlobal_triggerState = 0;	// 트리거 1 처리 상태 (0: 대기, 1: 활성, 5: 상승엣지 대기)
uint8_t gGlobal_triggerState2 = 0;	// 트리거 2 처리 상태 (10: 대기, 1: 활성, 15: 상승엣지 대기)
uint8_t gGlobal_ledState = 0;		// LED 상태 (0: 미설정, 1: 활성)
uint8_t dtState = 0;
uint32_t get_usb_data_time = 0;
uint8_t stx_flag = 0;

// 통신 및 버퍼 관련 변수들
int gGlobal_Rxindx;				 // 수신 데이터 인덱스
int gGlobal_keyCount = 0;		 // 키 입력 카운터
int gGlobal_longKeycount = 0;	 // 긴 키 입력 카운터
uint8_t gGlobal_rxData;			 // UART 수신 데이터 버퍼
uint8_t gGlobal_rxBuffer[100];	 // UART 수신 처리 버퍼
uint8_t gGlobal_tmpBuffer[100];	 // 임시 데이터 버퍼
uint8_t gGlobal_usbBuffer[2048]; // USB 통신 버퍼
uint8_t gGlobal_sendData[12];	 // 데이터 전송 버퍼
uint8_t gGlobal_sendDatacpy[12]; // 데이터 전송 버퍼 복사본
uint8_t gGlobal_crc1 = 0;		 // CRC 상위 바이트
uint8_t gGlobal_crc2 = 0;		 // CRC 하위 바이트

// 로터리 인코더 관련 변수들
int counter = 0;				   // 인코더 카운터
int currentStateCLK;			   // 현재 클럭 상태
int lastStateCLK;				   // 이전 클럭 상태
unsigned long lastButtonPress = 0; // 마지막 버튼 누름 시간
uint8_t MSG[30] = {'\0'};		   // 메시지 버퍼

// 타이머 관련 변수들
// static unsigned short gGlobal_keyTimer = 0;		// 키 입력 타이머
static unsigned short gGlobal_jog = 0;			// 조그 타이머
static unsigned short gGlobal_jogTimer = 0;		// 조그 동작 타이머
static unsigned short gGlobal_encoderTimer = 0; // LED 제어 타이머
static unsigned short gGlobal_encoderCount = 0; // LED 깜빡임 카운터
static unsigned short Buzzer_count = 0;			// 부저 동작 카운터
uint32_t Buzzer_timer = 0;			// 부저 타이머
uint8_t Long_Buzzer = 0;			// 긴 버튼 누름 부저 타이머

// 테스트 및 디버깅용 버퍼
uint8_t gGlobal_resetBuf[100]; // 리셋 버퍼
uint8_t testbuf[100];		   // 테스트용 임시 버퍼

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;   // 타이머1 핸들
UART_HandleTypeDef huart1; // UART1 핸들

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
void SystemClock_Config(void);

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/**
 * @brief  메인 함수
 * @note   프로그램의 진입점
 * @param  None
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */
	HAL_GPIO_WritePin(Dial_LED_1_GPIO_Port, Dial_LED_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Dial_LED_2_GPIO_Port, Dial_LED_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Dial_LED_3_GPIO_Port, Dial_LED_3_Pin, GPIO_PIN_SET);

	// 시스템 상태 초기화
	InitSystemState();

	// 통신 데이터 초기화
	g_systemState.comm.sendData[0] = 0x02;
	g_systemState.comm.sendData[1] = 0xA4;
	g_systemState.comm.sendData[2] = 0x32;
	g_systemState.comm.sendData[3] = 0x34;
	g_systemState.comm.sendData[4] = 0x30;	// key
	g_systemState.comm.sendData[5] = 0x30;	// key
	g_systemState.comm.sendData[6] = 0x30;	// dial
	g_systemState.comm.sendData[7] = 0x30;	// trigger
	g_systemState.comm.sendData[8] = 0x30;	// trigger
	g_systemState.comm.sendData[9] = 0x30;	// crc
	g_systemState.comm.sendData[10] = 0x30; // crc
	g_systemState.comm.sendData[11] = 0x03;
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USB_DEVICE_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, &gGlobal_rxData, 1);

	HAL_TIM_Base_Start_IT(&htim1);
	g_systemState.triggers.trigger_in_Old1 = HAL_GPIO_ReadPin(Trigger_IN_1_GPIO_Port, Trigger_IN_1_Pin);
	g_systemState.triggers.trigger_in_Old2 = HAL_GPIO_ReadPin(Trigger_IN_2_GPIO_Port, Trigger_IN_2_Pin);

	HAL_GPIO_WritePin(Trigger_OUT_1_GPIO_Port, Trigger_OUT_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Trigger_OUT_2_GPIO_Port, Trigger_OUT_2_Pin, GPIO_PIN_RESET);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if (g_systemState.comm.sendData[0] != 0x02)
		{ // 통신 데이터 초기화
			g_systemState.comm.sendData[0] = 0x02;
			g_systemState.comm.sendData[1] = 0xA4;
			g_systemState.comm.sendData[2] = 0x32;
			g_systemState.comm.sendData[3] = 0x34;
			g_systemState.comm.sendData[4] = 0x30;	// key
			g_systemState.comm.sendData[5] = 0x30;	// key
			g_systemState.comm.sendData[6] = 0x30;	// dial
			g_systemState.comm.sendData[7] = 0x30;	// trigger
			g_systemState.comm.sendData[8] = 0x30;	// trigger
			g_systemState.comm.sendData[9] = 0x30;	// crc
			g_systemState.comm.sendData[10] = 0x30; // crc
			g_systemState.comm.sendData[11] = 0x03;
		}
		USB_minipc();

		// 로터리 인코더 처리 (고정밀 버전)
		ProcessEncoderAdvanced();

		// LED 상태 처리
		ProcessLEDState();

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

		// 버튼 처리
		ProcessAllButtons();

		// 부저 처리
		if ((g_systemState.state != BUTTON_STATE_IDLE && (g_systemState.timers.keyCount > 50 && g_systemState.timers.keyCount < 100)))
		{
			Buzzer_timer = 0;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		}

		// 트리거 처리
		ProcessTriggers();
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 36000 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, BUZZER_Pin | System_LED_Pin | USB_SW_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, Trigger_OUT_1_Pin | Trigger_OUT_2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, Dial_LED_1_Pin | Dial_LED_2_Pin | Dial_LED_3_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : BUZZER_Pin System_LED_Pin USB_SW_Pin */
	GPIO_InitStruct.Pin = BUZZER_Pin | System_LED_Pin | USB_SW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : Trigger_IN_1_Pin Trigger_IN_2_Pin */
	GPIO_InitStruct.Pin = Trigger_IN_1_Pin | Trigger_IN_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : Trigger_OUT_1_Pin Trigger_OUT_2_Pin */
	GPIO_InitStruct.Pin = Trigger_OUT_1_Pin | Trigger_OUT_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : Dial_LED_1_Pin Dial_LED_2_Pin Dial_LED_3_Pin */
	GPIO_InitStruct.Pin = Dial_LED_1_Pin | Dial_LED_2_Pin | Dial_LED_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : Dial_A_Pin Dial_B_Pin BOOST_Pin POWRAY_ON_Pin
							 DOWN_Pin UP_Pin Memory_2_Pin Memory_3_Pin
							 Memory_4_Pin MODE_Pin */
	GPIO_InitStruct.Pin = Dial_A_Pin | Dial_B_Pin | BOOST_Pin | POWRAY_ON_Pin | DOWN_Pin | UP_Pin | Memory_2_Pin | Memory_3_Pin | Memory_4_Pin | MODE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : RIGHT_Pin Power_SW_Pin LEFT_Pin */
	GPIO_InitStruct.Pin = RIGHT_Pin | Power_SW_Pin | LEFT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// UART 수신 완료 콜백 함수
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		RX_DATA();
		HAL_UART_Receive_IT(&huart1, &gGlobal_rxData, 1);
	}
}

// 타이머 콜백 함수
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	UNUSED(htim);

	// ms 단위
	g_systemState.timers.keyTimer += 1;
	g_systemState.timers.longKey += 1;
	gGlobal_jog += 1;
	g_systemState.timers.ledTimer += 1;
	Buzzer_timer += 1;
	gGlobal_encoderTimer += 1;

	if (gGlobal_jog >= 1)
	{
		gGlobal_jog = 0;
		gGlobal_jogTimer++;
	}
	if (g_systemState.timers.keyTimer >= 1)
	{
		g_systemState.timers.keyTimer = 0;
		g_systemState.timers.keyCount++;
	}
	if (g_systemState.timers.longKey >= 3000 && g_systemState.timers.longKeycount == 0)
	{
		g_systemState.timers.longKey = 0;
		g_systemState.timers.longKeycount++;
	}
	if (g_systemState.timers.ledTimer >= 10)
	{
		g_systemState.timers.ledTimer = 0;
		g_systemState.timers.ledCount++;
		if (g_systemState.timers.ledCount >= 100)
		{
			HAL_GPIO_TogglePin(System_LED_GPIO_Port, System_LED_Pin);
			g_systemState.timers.ledCount = 0;
		}
	}
	if (gGlobal_encoderTimer >= 1)
	{
		gGlobal_encoderTimer = 0;
		gGlobal_encoderCount++;
	}
	if ((Buzzer_timer >= 20 && Long_Buzzer != 1) || (Buzzer_timer >= 300 && Long_Buzzer == 1))
	{
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		Buzzer_timer = 0;
		Long_Buzzer = 0;
		Buzzer_count++;
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* 사용자 코드 시작 - 디버그 모드에서만 실행 */
	while (1)
	{
		NVIC_SystemReset();
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* 사용자 코드 시작 - 어설션 실패 시 실행 */
	while (1)
	{
	}
	/* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */
