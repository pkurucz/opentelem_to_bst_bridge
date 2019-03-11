
#include "util.h"
#include <Arduino.h>

void hex_print(uint8_t byte) {
	if (byte < 16)
		Serial.print('0');
	Serial.print(byte, HEX);		
	Serial.print(' ');
}

char hex_digit(uint8_t byte) {
	byte &= 0x0F;

	if(byte < 10) {
		return '0' + byte;
	} else {
		return 'A' + (byte - 10);
	}
}

