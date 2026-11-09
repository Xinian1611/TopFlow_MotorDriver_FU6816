#include "crc16.h"

unsigned short crc16_ccitt(const unsigned char *buf, unsigned int len)   /* 名字沿用 f105 */
{
	unsigned short crc = 0;
	unsigned int counter;
	for (counter = 0; counter < len; counter++) {
		crc ^= (unsigned short)(*buf++) << 8;
		{
			unsigned char i;
			for (i = 0; i < 8; i++)
				crc = (crc & 0x8000) ? (unsigned short)((crc << 1) ^ 0x1021) : (unsigned short)(crc << 1);
		}
	}
	return crc;
}
