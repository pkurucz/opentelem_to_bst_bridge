/* 
 * General helper functions and defines
 *
 * ------------------------------------------------
 *
 * Copyright (C) 2019 Paul Kurucz
 * 
 * License info: See the LICENSE file at the repo top level
 *
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 */

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

