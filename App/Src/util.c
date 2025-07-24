#include "util.h"
#include "main.h"

char hextoascii(char local_toconv) // toconv
{
	if (local_toconv < 0x0A)
		local_toconv += 0x30;
	else
		local_toconv += 0x37;
	return (local_toconv);
}

void hex2ascii(void)
{
	gGlobal_crc1 = hextoascii(gGlobal_sum1);
	gGlobal_crc2 = hextoascii(gGlobal_sum2);
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
	for (int i = 1; i < 9; i++)
	{
		gGlobal_sum += gGlobal_sendData[i];
	}
	gGlobal_sum1 = gGlobal_sum >> 4;
	gGlobal_sum2 = gGlobal_sum << 4;
	gGlobal_sum2 = gGlobal_sum2 >> 4;
	hex2ascii();
	return gGlobal_sum;
}