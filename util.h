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

#include <stdbool.h>
#include <stdint.h>

#ifndef _UTIL_H
#define _UTIL_H

/* ----------------------------------------------------- */

#ifndef MIN
#define MIN(_A, _B) ( ((_A) < (_B)) ? (_A) : (_B) )
#endif

#ifndef MAX
#define MAX(_A, _B) ( ((_A) > (_B)) ? (_A) : (_B) )
#endif

/* ----------------------------------------------------- */

#ifndef htons
#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                   ((x)>> 8 & 0x00FF) )
#endif

#ifndef ntohs
#define ntohs(x) htons(x)
#endif

#ifndef htonl
#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#endif

#ifndef ntohl
#define ntohl(x) htonl(x)
#endif


/* ----------------------------------------------------- */

#define LED_ON() digitalWrite(LED_BUILTIN, HIGH)
#define LED_OFF() digitalWrite(LED_BUILTIN, LOW)

/* ----------------------------------------------------- */

extern void hex_print(uint8_t byte);
extern char hex_digit(uint8_t byte);

/* ----------------------------------------------------- */

#endif /* _UTIL_H */

