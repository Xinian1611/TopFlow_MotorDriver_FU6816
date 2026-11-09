#ifndef _CRC16_H_
#define _CRC16_H_
  
/* 与领导 f105 的 crc16_ccitt 算法完全一致：
   init=0, poly=0x1021, MSB first, 无结果异或。表已去掉（省 512B code 空间）*/
extern unsigned short crc16_ccitt(const unsigned char *buf, unsigned int len);

#endif /* _CRC16_H_ */


