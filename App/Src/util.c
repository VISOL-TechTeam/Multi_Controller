#include "util.h"
#include "main.h"
#include "gpio.h"

char hextoascii(uint8_t local_toconv) // hex value to ascii character
{
	if (local_toconv <= 9)
		return local_toconv + '0';
	else if (local_toconv >= 10 && local_toconv <= 15)
		return local_toconv - 10 + 'A';
	else
		return '?'; // 잘못된 값
}

uint8_t asciitohex(char local_toconv) // ascii character to hex value
{
	if (local_toconv >= '0' && local_toconv <= '9')
		return local_toconv - '0';
	else if (local_toconv >= 'A' && local_toconv <= 'F')
		return local_toconv - 'A' + 10;
	else if (local_toconv >= 'a' && local_toconv <= 'f')
		return local_toconv - 'a' + 10;
	else
		return 0xFF; // 잘못된 문자
}

void hex2ascii(void)
{
	g_systemState.comm.crc1 = hextoascii(gGlobal_sum1);
	g_systemState.comm.crc2 = hextoascii(gGlobal_sum2);
	gGlobal_sum = 0;
}

uint8_t calculate_crc8(void)
{
	for (int i = 1; i < (int)(gGlobal_usbLen - 3); i++)
	{
		gGlobal_sum += gGlobal_Buffer[i];
	}
	gGlobal_sum1 = gGlobal_sum >> 4;
	gGlobal_sum2 = gGlobal_sum << 4;
	gGlobal_sum2 = gGlobal_sum2 >> 4;
	hex2ascii();
	return gGlobal_sum;
}

uint8_t Pad_calculate_crc8(void)
{
	gGlobal_sum = 0; // 초기화 추가
	for (int i = 1; i < 9; i++)
	{
		gGlobal_sum += g_systemState.comm.sendData[i];
	}
	gGlobal_sum1 = gGlobal_sum >> 4;
	gGlobal_sum2 = gGlobal_sum << 4;
	gGlobal_sum2 = gGlobal_sum2 >> 4;
	hex2ascii();
	return gGlobal_sum;
}