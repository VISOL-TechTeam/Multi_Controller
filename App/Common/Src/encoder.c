#include "encoder.h"

static volatile int32_t encoder_count = 0;
static volatile int32_t encoder_speed = 0;
static volatile uint8_t encoder_direction = 0;
static volatile uint32_t encoder_last_change_time = 0;  // 마지막 변화 시간
static volatile int32_t encoder_pulse_count = 0;  // 펄스 카운트

/**
 * @brief 엔코더 카운트 값 반환
 * @return 현재 엔코더 카운트
 */
int32_t Encoder_GetCount(void)
{
  return encoder_count;
}

/**
 * @brief 엔코더 방향 반환
 * @return 0: 정방향, 1: 역방향
 */
uint8_t Encoder_GetDirection(void)
{
  return encoder_direction;
}

/**
 * @brief 엔코더 속도 반환
 * @return 엔코더 속도 (counts per second)
 */
int32_t Encoder_GetSpeed(void)
{
  return encoder_speed;
}

/**
 * @brief 엔코더 카운트 리셋
 */
void Encoder_ResetCount(void)
{
  encoder_count = 0;
  encoder_speed = 0;
}

/**
 * @brief 엔코더 초기화 함수
 */
void Encoder_Init(void)
{
  
  // 기본 변수들 초기화
  encoder_count = 0;
  encoder_speed = 0;
  encoder_direction = 0;
  encoder_last_change_time = 0;
  encoder_pulse_count = 0;

}

/**
 * @brief 엔코더 마지막 변화 시간 반환
 * @return 마지막 변화 시간
 */
uint32_t Encoder_GetLastChangeTime(void)
{
  return encoder_last_change_time;
}

/**
 * @brief 엔코더 펄스 카운트 반환 및 리셋
 * @return 펄스 카운트
 */
int32_t Encoder_GetAndResetPulseCount(void)
{
  int32_t count = encoder_pulse_count;
  encoder_pulse_count = 0;
  return count;
}

/**
 * @brief 엔코더 속도 설정 (TASK에서 계산한 값)
 * @param speed 설정할 속도
 */
void Encoder_SetSpeed(int32_t speed)
{
  encoder_speed = speed;
}

/**
 * @brief 엔코더 카운트 설정 (TASK에서 계산한 값)
 * @param count 설정할 카운트
 */
void Encoder_SetCount(int32_t count)
{
  encoder_count = count;
}

/**
 * @brief 엔코더 방향 설정 (TASK에서 계산한 값)
 * @param direction 설정할 방향
 */
void Encoder_SetDirection(uint8_t direction)
{
  encoder_direction = direction;
}

/**
 * @brief 엔코더 마지막 변화 시간 설정 (TASK에서 계산한 값)
 * @param time 설정할 시간
 */
void Encoder_SetLastChangeTime(uint32_t time)
{
  encoder_last_change_time = time;
}

/**
 * @brief 엔코더 펄스 카운트 설정 (TASK에서 계산한 값)
 * @param count 설정할 펄스 카운트
 */
void Encoder_SetPulseCount(int32_t count)
{
  encoder_pulse_count = count;
}

/**
 * @brief 엔코더 펄스 카운트에 값 추가 (TASK에서 계산한 값)
 * @param count 추가할 펄스 카운트
 */
void Encoder_AddPulseCount(int32_t count)
{
  encoder_pulse_count += count;
}
