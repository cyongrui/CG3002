#include "crc.h"

unsigned char CRC8(const unsigned char *data, unsigned char len) {
	unsigned char crc = 0x00;
  	while (len--) {
	    unsigned char extract = *data++;
	    for (unsigned char tempI = 8; tempI; tempI--) {
	    	unsigned char sum = (crc ^ extract) & 0x01;
	      	crc >>= 1;
	      	if (sum) {
	       		crc ^= 0x8C;
	      	}
	      	extract >>= 1;
	    }
	}
  	return crc;
}

