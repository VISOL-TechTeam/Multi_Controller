#include "gpio.h"
#include "main.h"

int TriggerPin_1(void)
{
	//HAL_GPIO_TogglePin(Trigger_OUT_1_GPIO_Port, Trigger_OUT_1_Pin);
	if (gGlobal_triggoutState1 == 1)
	{
		HAL_GPIO_WritePin(Trigger_OUT_1_GPIO_Port, Trigger_OUT_1_Pin, GPIO_PIN_SET); // 트리거 1 출력 HIGH
	}
	if (gGlobal_triggoutState1 == 2)
	{
		HAL_GPIO_WritePin(Trigger_OUT_1_GPIO_Port, Trigger_OUT_1_Pin, GPIO_PIN_RESET); // 트리거 1 출력 LOW
	}
	return 0;
}

int TriggerPin_2(void)
{
	if (gGlobal_triggoutState2 == 1)
	{
		HAL_GPIO_WritePin(Trigger_OUT_2_GPIO_Port, Trigger_OUT_2_Pin, GPIO_PIN_SET); // 트리거 2 출력 HIGH
	}
	if (gGlobal_triggoutState2 == 2)
	{
		HAL_GPIO_WritePin(Trigger_OUT_2_GPIO_Port, Trigger_OUT_2_Pin, GPIO_PIN_RESET); // 트리거 2 출력 LOW
	}
	return 0;
}
