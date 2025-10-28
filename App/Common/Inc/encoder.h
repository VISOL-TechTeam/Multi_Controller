/*
 * encoder.h
 *
 *  Created on: 2025-07-17
 *      Author: GH Choi
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "def.h"

#define ENCODER_DEBOUNCE_TIME_MS_BASE 3 // 기본 채터링 방지 시간 (ms)
#define ENCODER_DEBOUNCE_TIME_MS_FAST 1 // 고속시 채터링 방지 시간 (ms)
#define ENCODER_SPEED_CALC_TIME_MS 100  // 속도 계산 주기 (ms)
#define ENCODER_SPEED_THRESHOLD_FAST 50 // 고속 모드 임계값

int32_t Encoder_GetCount(void);
uint8_t Encoder_GetDirection(void);
int32_t Encoder_GetSpeed(void);
void Encoder_ResetCount(void);
void Encoder_Init(void);
uint32_t Encoder_GetLastChangeTime(void);
int32_t Encoder_GetAndResetPulseCount(void);
void Encoder_SetSpeed(int32_t speed);
void Encoder_SetCount(int32_t count);
void Encoder_SetDirection(uint8_t direction);
void Encoder_SetLastChangeTime(uint32_t time);
void Encoder_SetPulseCount(int32_t count);
void Encoder_AddPulseCount(int32_t count);

#endif /* ENCODER_H_ */
