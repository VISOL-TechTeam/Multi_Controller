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

// 통신 및 버퍼 관련 변수들
int gGlobal_Rxindx;				 // 수신 데이터 인덱스
int gGlobal_keyCount = 0;		 // 키 입력 카운터
int gGlobal_longKeycount = 0;	 // 긴 키 입력 카운터
uint8_t gGlobal_rxData[2];		 // UART 수신 데이터 버퍼
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
static unsigned short gGlobal_keyTimer = 0;		// 키 입력 타이머
static unsigned short gGlobal_longKey = 0;		// 긴 키 입력 타이머
static unsigned short gGlobal_jog = 0;			// 조그 타이머
static unsigned short gGlobal_jogTimer = 0;		// 조그 동작 타이머
static unsigned short gGlobal_ledTimer = 0;		// LED 제어 타이머
static unsigned short gGlobal_ledCount = 0;		// LED 깜빡임 카운터
static unsigned short gGlobal_encoderTimer = 0; // LED 제어 타이머
static unsigned short gGlobal_encoderCount = 0; // LED 깜빡임 카운터
static unsigned short Buzzer_timer = 0;			// 부저 타이머
static unsigned short Buzzer_count = 0;			// 부저 동작 카운터

// 테스트 및 디버깅용 버퍼
uint8_t test[2048];			   // 테스트용 대용량 버퍼
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
	gGlobal_sendData[0] = 0x02;
	gGlobal_sendData[1] = 0xA4;
	gGlobal_sendData[2] = 0x32;
	gGlobal_sendData[3] = 0x34;
	gGlobal_sendData[4] = 0x30;	 // key
	gGlobal_sendData[5] = 0x30;	 // key
	gGlobal_sendData[6] = 0x30;	 // dial
	gGlobal_sendData[7] = 0x30;	 // trigger
	gGlobal_sendData[8] = 0x30;	 // trigger
	gGlobal_sendData[9] = 0x30;	 // crc
	gGlobal_sendData[10] = 0x30; // crc
	gGlobal_sendData[11] = 0x03;
	lastStateCLK = CLK;
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

	HAL_TIM_Base_Start_IT(&htim1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_TIM_Base_Start_IT(&htim1);
		HAL_UART_Receive_IT(&huart1, (uint8_t *)gGlobal_rxData, 1);
		USB_minipc();
		currentStateCLK = CLK;
		if (currentStateCLK != lastStateCLK) // && currentStateCLK == 1
		{
			// printf("currentStateCLK = %d, DT = %d\r\n", currentStateCLK, DT);
			if (DT != currentStateCLK && currentStateCLK == 1)
			{
				HAL_GPIO_WritePin(Dial_LED_2_GPIO_Port, Dial_LED_2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Dial_LED_3_GPIO_Port, Dial_LED_3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Dial_LED_1_GPIO_Port, Dial_LED_1_Pin, GPIO_PIN_RESET);
				// counter++;
				gGlobal_sendData[6] = 0x32;
				Pad_calculate_crc8();
				gGlobal_sendData[9] = gGlobal_crc1;
				gGlobal_sendData[10] = gGlobal_crc2;
				CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
				gGlobal_sendData[6] = 0x30;
				gGlobal_ledTimer = 0;
				gGlobal_ledCount = 0;
			}
			if (DT == 0 && currentStateCLK == 0) // else
			{
				dtState = 1; // 1
			}
			if (dtState == 1 && DT == 1 && currentStateCLK == 1)
			{
				HAL_GPIO_WritePin(Dial_LED_1_GPIO_Port, Dial_LED_1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Dial_LED_2_GPIO_Port, Dial_LED_2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Dial_LED_3_GPIO_Port, Dial_LED_3_Pin, GPIO_PIN_RESET);
				// counter--;
				gGlobal_sendData[6] = 0x31;
				Pad_calculate_crc8();
				gGlobal_sendData[9] = gGlobal_crc1;
				gGlobal_sendData[10] = gGlobal_crc2;
				CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
				gGlobal_sendData[6] = 0x30;
				gGlobal_ledTimer = 0;
				gGlobal_ledCount = 0;
				dtState = 0;
			}
		}
		lastStateCLK = currentStateCLK;

		if (gGlobal_ledCount == 70)
		{
			HAL_GPIO_WritePin(Dial_LED_1_GPIO_Port, Dial_LED_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Dial_LED_2_GPIO_Port, Dial_LED_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Dial_LED_3_GPIO_Port, Dial_LED_3_Pin, GPIO_PIN_SET);
			if (gGlobal_ledCount == 100)
			{
				gGlobal_ledCount = 0;
			}
		}

		gGlobal_triggIn1 = HAL_GPIO_ReadPin(Trigger_IN_1_GPIO_Port, Trigger_IN_1_Pin);
		gGlobal_triggIn2 = HAL_GPIO_ReadPin(Trigger_IN_2_GPIO_Port, Trigger_IN_2_Pin);
		gGlobal_upButton = HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin);
		gGlobal_downButton = HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin);
		gGlobal_powerButton = HAL_GPIO_ReadPin(POWRAY_ON_GPIO_Port, POWRAY_ON_Pin);
		gGlobal_boostButton = HAL_GPIO_ReadPin(BOOST_GPIO_Port, BOOST_Pin);
		gGlobal_pcmodeButton = HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin);
		gGlobal_memory1Button = HAL_GPIO_ReadPin(Memory_1_GPIO_Port, Memory_1_Pin);
		gGlobal_memory2Button = HAL_GPIO_ReadPin(Memory_2_GPIO_Port, Memory_2_Pin);
		gGlobal_memory3Button = HAL_GPIO_ReadPin(Memory_3_GPIO_Port, Memory_3_Pin);
		gGlobal_memory4Button = HAL_GPIO_ReadPin(Memory_4_GPIO_Port, Memory_4_Pin);
		// printf("gGlobal_keyCount = %d, gGlobal_state = %d\r\n", gGlobal_keyCount, gGlobal_state);
		if (gGlobal_upButton == 0)
		{
			if (gGlobal_keyCount > 70 && gGlobal_keyCount < 3000)
			{
				gGlobal_state = 1;
			}
			HAL_GPIO_WritePin(System_LED_GPIO_Port, System_LED_Pin, GPIO_PIN_RESET);
			if (gGlobal_longKeycount == 1)
			{
				gGlobal_sendData[4] = 0x50;
				Pad_calculate_crc8();
				gGlobal_sendData[9] = gGlobal_crc1;
				gGlobal_sendData[10] = gGlobal_crc2;
				CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
				gGlobal_sendData[4] = 0x30;
				gGlobal_longKey = 0;
				gGlobal_longKeycount = 0;
				gGlobal_state = 0;
				gGlobal_sendData[4] = 0x30;
			}
		}
		if (gGlobal_upButton == 1 && gGlobal_state == 1)
		{
			gGlobal_sendData[4] = 0x31;
			Pad_calculate_crc8();
			gGlobal_sendData[9] = gGlobal_crc1;
			gGlobal_sendData[10] = gGlobal_crc2;
			CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
			gGlobal_sendData[4] = 0x30;
			gGlobal_keyCount = 0;
			gGlobal_keyTimer = 0;
			gGlobal_state = 0;
		}

		if (gGlobal_downButton == 0)
		{
			if (gGlobal_keyCount > 70 && gGlobal_keyCount < 3000)
			{
				gGlobal_state = 2;
			}
		}
		if (gGlobal_downButton == 1 && gGlobal_state == 2)
		{
			gGlobal_sendData[4] = 0x32;
			Pad_calculate_crc8();
			gGlobal_sendData[9] = gGlobal_crc1;
			gGlobal_sendData[10] = gGlobal_crc2;
			CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
			gGlobal_sendData[4] = 0x30;
			gGlobal_keyCount = 0;
			gGlobal_keyTimer = 0;
			gGlobal_state = 0;
		}
		if (gGlobal_powerButton == 0)
		{
			if (gGlobal_keyCount > 70 && gGlobal_keyCount < 3000)
			{
				gGlobal_state = 3;
			}
		}
		if (gGlobal_powerButton == 1 && gGlobal_state == 3)
		{
			gGlobal_sendData[4] = 0x34;
			Pad_calculate_crc8();
			gGlobal_sendData[9] = gGlobal_crc1;
			gGlobal_sendData[10] = gGlobal_crc2;
			CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
			gGlobal_sendData[4] = 0x30;
			gGlobal_keyCount = 0;
			gGlobal_keyTimer = 0;
			gGlobal_state = 0;
		}
		if (gGlobal_boostButton == 0)
		{
			if (gGlobal_keyCount > 70 && gGlobal_keyCount < 3000)
			{
				gGlobal_state = 4;
			}
		}
		if (gGlobal_boostButton == 1 && gGlobal_state == 4)
		{
			gGlobal_sendData[4] = 0x35;
			Pad_calculate_crc8();
			gGlobal_sendData[9] = gGlobal_crc1;
			gGlobal_sendData[10] = gGlobal_crc2;
			CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
			gGlobal_sendData[4] = 0x30;
			gGlobal_keyCount = 0;
			gGlobal_keyTimer = 0;
			gGlobal_state = 0;
		}
		if (gGlobal_pcmodeButton == 0)
		{
			if (gGlobal_keyCount > 70 && gGlobal_keyCount < 3000)
			{
				gGlobal_state = 5;
			}
		}
		if (gGlobal_pcmodeButton == 1 && gGlobal_state == 5)
		{
			gGlobal_sendData[4] = 0x33;
			Pad_calculate_crc8();
			gGlobal_sendData[9] = gGlobal_crc1;
			gGlobal_sendData[10] = gGlobal_crc2;
			CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
			gGlobal_sendData[4] = 0x30;
			gGlobal_keyCount = 0;
			gGlobal_keyTimer = 0;
			gGlobal_state = 0;
		}
		if (gGlobal_memory1Button == 0)
		{
			if (gGlobal_keyCount > 70 && gGlobal_keyCount < 3000)
			{
				gGlobal_state = 6;
			}
			if (gGlobal_longKeycount == 1)
			{
				gGlobal_sendData[5] = 0x32;
				Pad_calculate_crc8();
				gGlobal_sendData[9] = gGlobal_crc1;
				gGlobal_sendData[10] = gGlobal_crc2;
				CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
				gGlobal_sendData[5] = 0x30;
				gGlobal_longKey = 0;
				gGlobal_longKeycount = 0;
				gGlobal_state = 0;
			}
		}
		if (gGlobal_memory1Button == 1 && gGlobal_state == 6)
		{
			gGlobal_sendData[5] = 0x31;
			Pad_calculate_crc8();
			gGlobal_sendData[9] = gGlobal_crc1;
			gGlobal_sendData[10] = gGlobal_crc2;
			CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
			gGlobal_sendData[5] = 0x30;
			gGlobal_keyCount = 0;
			gGlobal_keyTimer = 0;
			gGlobal_state = 0;
		}
		if (gGlobal_memory2Button == 0)
		{
			if (gGlobal_keyCount > 70 && gGlobal_keyCount < 3000)
			{
				gGlobal_state = 7;
			}
			if (gGlobal_longKeycount == 1)
			{
				gGlobal_sendData[5] = 0x34;
				Pad_calculate_crc8();
				gGlobal_sendData[9] = gGlobal_crc1;
				gGlobal_sendData[10] = gGlobal_crc2;
				CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
				gGlobal_sendData[5] = 0x30;
				gGlobal_longKey = 0;
				gGlobal_longKeycount = 0;
				gGlobal_state = 0;
			}
		}
		if (gGlobal_memory2Button == 1 && gGlobal_state == 7)
		{
			gGlobal_sendData[5] = 0x33;
			Pad_calculate_crc8();
			gGlobal_sendData[9] = gGlobal_crc1;
			gGlobal_sendData[10] = gGlobal_crc2;
			CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
			gGlobal_sendData[5] = 0x30;
			gGlobal_keyCount = 0;
			gGlobal_keyTimer = 0;
			gGlobal_state = 0;
		}
		if (gGlobal_memory3Button == 0)
		{
			if (gGlobal_keyCount > 70 && gGlobal_keyCount < 3000)
			{
				gGlobal_state = 8;
			}
			if (gGlobal_longKeycount == 1)
			{
				gGlobal_sendData[5] = 0x36;
				Pad_calculate_crc8();
				gGlobal_sendData[9] = gGlobal_crc1;
				gGlobal_sendData[10] = gGlobal_crc2;
				CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
				gGlobal_sendData[5] = 0x30;
				gGlobal_longKey = 0;
				gGlobal_longKeycount = 0;
				gGlobal_state = 0;
			}
		}
		if (gGlobal_memory3Button == 1 && gGlobal_state == 8)
		{
			gGlobal_sendData[5] = 0x35;
			Pad_calculate_crc8();
			gGlobal_sendData[9] = gGlobal_crc1;
			gGlobal_sendData[10] = gGlobal_crc2;
			CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
			gGlobal_sendData[5] = 0x30;
			gGlobal_keyCount = 0;
			gGlobal_keyTimer = 0;
			gGlobal_state = 0;
		}
		if (gGlobal_memory4Button == 0)
		{
			if (gGlobal_keyCount > 70 && gGlobal_keyCount < 3000)
			{
				gGlobal_state = 9;
			}
			if (gGlobal_longKeycount == 1)
			{
				gGlobal_sendData[5] = 0x38;
				Pad_calculate_crc8();
				gGlobal_sendData[9] = gGlobal_crc1;
				gGlobal_sendData[10] = gGlobal_crc2;
				CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
				gGlobal_sendData[5] = 0x30;
				gGlobal_longKey = 0;
				gGlobal_longKeycount = 0;
				gGlobal_state = 0;
			}
		}
		if (gGlobal_memory4Button == 1 && gGlobal_state == 9)
		{
			gGlobal_sendData[5] = 0x37;
			Pad_calculate_crc8();
			gGlobal_sendData[9] = gGlobal_crc1;
			gGlobal_sendData[10] = gGlobal_crc2;
			CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
			gGlobal_sendData[5] = 0x30;
			gGlobal_keyCount = 0;
			gGlobal_keyTimer = 0;
			gGlobal_state = 0;
		}
		if (gGlobal_upButton == 1 && gGlobal_downButton == 1 && gGlobal_powerButton == 1 && gGlobal_boostButton == 1 && gGlobal_pcmodeButton && gGlobal_memory1Button && gGlobal_memory2Button && gGlobal_memory3Button && gGlobal_memory4Button)
		{
			gGlobal_keyCount = 0;
			gGlobal_keyTimer = 0;
			gGlobal_state = 0;
			gGlobal_longKey = 0;
			gGlobal_longKeycount = 0;
		}
		if (gGlobal_keyCount == 50)
		{
			Buzzer_timer = 0;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		}
		if (gGlobal_triggIn1 == 0 && gGlobal_trigginState1 == 1 && gGlobal_triggerState == 0) // 트리거 1 하강엣지 검출
		{
			gGlobal_sendData[7] = 0x31;
			Pad_calculate_crc8();
			gGlobal_sendData[9] = gGlobal_crc1;
			gGlobal_sendData[10] = gGlobal_crc2;
			CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
			gGlobal_sendData[7] = 0x30;
			gGlobal_triggerState = 1;
		}
		if (gGlobal_triggIn1 == 1 && gGlobal_trigginState1 == 2 && gGlobal_triggerState == 5) // 트리거 1 상승엣지 검출
		{
			gGlobal_sendData[7] = 0x31;
			Pad_calculate_crc8();
			gGlobal_sendData[9] = gGlobal_crc1;
			gGlobal_sendData[10] = gGlobal_crc2;
			CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
			gGlobal_sendData[7] = 0x30;
			gGlobal_triggerState = 1;
		}
		if (gGlobal_triggIn2 == 0 && gGlobal_trigginState2 == 1 && gGlobal_triggerState2 == 10)
		{
			gGlobal_sendData[8] = 0x32;
			Pad_calculate_crc8();
			gGlobal_sendData[9] = gGlobal_crc1;
			gGlobal_sendData[10] = gGlobal_crc2;
			CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
			gGlobal_sendData[8] = 0x30;
			gGlobal_triggerState2 = 1;
		}
		if (gGlobal_triggIn2 == 1 && gGlobal_trigginState2 == 2 && gGlobal_triggerState2 == 15)
		{
			gGlobal_sendData[8] = 0x32;
			Pad_calculate_crc8();
			gGlobal_sendData[9] = gGlobal_crc1;
			gGlobal_sendData[10] = gGlobal_crc2;
			CDC_Transmit_FS((uint8_t *)gGlobal_sendData, sizeof(gGlobal_sendData));
			gGlobal_sendData[8] = 0x30;
			gGlobal_triggerState2 = 1;
		}
		if (gGlobal_triggIn1 == 1 && gGlobal_trigginState1 == 1) // 트리거 1 하강엣지 대기 상태로 복귀
		{
			gGlobal_triggerState = 0;
		}
		if (gGlobal_triggIn1 == 0 && gGlobal_trigginState1 == 2) // 트리거 1 상승엣지 대기 상태로 설정
		{
			gGlobal_triggerState = 5;
		}
		if (gGlobal_triggIn2 == 1 && gGlobal_trigginState2 == 1) // 트리거 2 하강엣지 대기 상태로 복귀
		{
			gGlobal_triggerState2 = 10;
		}
		if (gGlobal_triggIn2 == 0 && gGlobal_trigginState2 == 2) // 트리거 2 상승엣지 대기 상태로 설정
		{
			gGlobal_triggerState2 = 15;
		}
		// printf("gGlobal_triggIn1 = %d, gGlobal_trigginState1 = %d\r\n", gGlobal_triggIn1, gGlobal_trigginState1);
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		RX_DATA();
		HAL_UART_Receive_IT(&huart1, (uint8_t *)gGlobal_rxData, 1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	UNUSED(htim);

	// ms 단위
	gGlobal_keyTimer += 1;
	gGlobal_longKey += 1;
	gGlobal_jog += 1;
	gGlobal_ledTimer += 1;
	Buzzer_timer += 1;
	gGlobal_encoderTimer += 1;

	if (gGlobal_jog >= 1)
	{
		gGlobal_jog = 0;
		gGlobal_jogTimer++;
	}
	if (gGlobal_keyTimer >= 1)
	{
		gGlobal_keyTimer = 0;
		gGlobal_keyCount++;
	}
	if (gGlobal_longKey >= 3000)
	{
		gGlobal_keyTimer = 0;
		gGlobal_longKeycount++;
	}
	if (gGlobal_ledTimer >= 10)
	{
		gGlobal_ledTimer = 0;
		gGlobal_ledCount++;
	}
	if (gGlobal_encoderTimer >= 1)
	{
		gGlobal_encoderTimer = 0;
		gGlobal_encoderCount++;
	}
	if (Buzzer_timer >= 20)
	{
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		Buzzer_timer = 0;
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
	__disable_irq();
	while (1)
	{
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
