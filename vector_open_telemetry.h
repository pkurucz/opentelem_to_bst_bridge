/*
 * Eagletree Vector Open Telemetry Protocol Handler
 *  Datatype definitions and helper functions
 *  
 * Definitions taken from:
 *   https://www.rcgroups.com/forums/showthread.php?2585582-Vector-Open-Telemetry-and-DragonLink-Advanced-support%21
 *   Version: Vector Open Telemetry Revision 0
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

// Vector Open Telemetry Revision 0
// NOTES:
// 1) UART protocol is 8N1 (8 bits, no parity bit, 1 stop bit), 57600 baud, 3.3V input/outputs levels (input is NOT 5V tolerant!)
// 2) all fields BIG-ENDIAN byte order
// 3) The VECTOR_OPEN_TELEMETRY packet is sent as frequently as every 80mS, but timing will vary considerably
// 4) To enable telemetry output on the Vector's UART port, select the "Open Telm" option
//    for the "Configure the UART port for" stick menu item, under the ""EagleEyes and Telemetry" OSD menu

// Vector UART Pinout (using standard Vector "BUS" cable colors):
// Yellow: RX (Receive data TO the Vector - note that this connection is not needed)
// Orange: TX (Transmit data FROM the Vector)
// Black: Ground
// Red: 5V Out, 150mA max (from Vector PSU or backup power input - do not exceed 1A total load on Vector PSU! Don't connect this wire unless the device receiving the telemetry requires power from the Vector)
// IMPORTANT: NEVER connect the telemetry cable to any Vector port other than UART!  Doing so can damage your equipment!

/*THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 */

#include <stdint.h>
#include <stdbool.h>

#ifndef _TELEMETRY_PUBLIC
#define _TELEMETRY_PUBLIC

/* ----------------------------------------------------- */

#pragma pack(push, 1) // force byte alignment

#define VECTOR_OPEN_TELEMETRY_REVISION 0
#define VECTOR_TELEMETRY_PACKET_START_CODE  0xB01EDEAD
#define VTELEM_CRC_INIT 0xFFFF

typedef enum {
	VECTOR_FLIGHT_MODE_2D,
	VECTOR_FLIGHT_MODE_2D_ALT_HOLD,
	VECTOR_FLIGHT_MODE_2D_HEADING_HOLD,
	VECTOR_FLIGHT_MODE_2D_ALT_HEADING_HOLD,
	VECTOR_FLIGHT_MODE_LOITER,
	VECTOR_FLIGHT_MODE_3D,
	VECTOR_FLIGHT_MODE_3D_HEADING_HOLD,
	VECTOR_FLIGHT_MODE_RTH,
	VECTOR_FLIGHT_MODE_LAND,
	VECTOR_FLIGHT_MODE_CARTESIAN,
	VECTOR_FLIGHT_MODE_CARTESIAN_LOITER,
	VECTOR_FLIGHT_MODE_POLAR,
	VECTOR_FLIGHT_MODE_POLAR_LOITER,
	VECTOR_FLIGHT_MODE_CENTER_STICK,
	VECTOR_FLIGHT_MODE_OFF,
	VECTOR_FLIGHT_MODE_WAYPOINT,
	VECTOR_FLIGHT_MODE_MAX
} VECTOR_FLIGHT_MODES;

typedef struct {
	int32_t LatitudeX1E7; // ( degree * 10,000,000 )
	int32_t LongitudeX1E7; // (degree * 10,000,000 )
	uint32_t DistanceFromHomeMX10; // horizontal GPS distance from home point, in meters X 10 (decimeters)
	uint16_t GroundspeedKPHX10; // ( km/h * 10 )
	uint16_t CourseDegrees; // GPS course over ground, in degrees
	int32_t GPSAltitudecm; // ( GPS altitude, using WGS-84 ellipsoid, cm)
	uint8_t HDOPx10; // GPS HDOP * 10
	uint8_t SatsInUse; // satellites used for navigation
} VECTOR_GPS_TELEMETRY;

typedef struct {
	int16_t PitchDegrees;
	int16_t RollDegrees;
	int16_t YawDegrees;
} VECTOR_ATTITUDE;

typedef struct {
	int16_t AccelXCentiGrav;
	int16_t AccelYCentiGrav;
	int16_t AccelZCentiGrav;
} VECTOR_ACCELERATION;

typedef struct
{
	int32_t BaroAltitudecm;  // zero referenced (from home position) barometric altitude in cm
	uint16_t AirspeedKPHX10; // KPH * 10, requires optional pitot sensor
	int16_t ClimbRateMSX100; // meters/second * 100
	uint16_t RPM; // requires optional RPM sensor
	VECTOR_ATTITUDE Attitude;
	VECTOR_ACCELERATION Acceleration;
	uint16_t PackVoltageX100;
	uint16_t VideoTxVoltageX100;
	uint16_t CameraVoltageX100;
	uint16_t RxVoltageX100;
	uint16_t PackCurrentX10;
	int16_t TempDegreesCX10; // degrees C * 10, from optional temperature sensor
	uint16_t mAHConsumed;
	uint16_t CompassDegrees; // either magnetic compass reading (if compass enabled) or filtered GPS course over ground if not
	uint8_t RSSIPercent;
	uint8_t LQPercent;
} VECTOR_SENSOR_TELEMETRY;

typedef struct
{
	uint32_t StartCode;
	uint32_t TimestampMS;  // timestamp in milliseconds
	VECTOR_SENSOR_TELEMETRY SensorTelemetry;
	VECTOR_GPS_TELEMETRY GPSTelemetry;
	uint8_t PresentFlightMode; // present flight mode, as defined in VECTOR_FLIGHT_MODES
	uint8_t RFU[24];     // reserved for future use
	uint16_t CRC;
} VECTOR_OPEN_TELEMETRY;

/* ----------------------------------------------------- */

extern VECTOR_OPEN_TELEMETRY vot_telemetry;
extern bool vot_telemetry_valid;
extern const char * const vot_flight_mode_strings[];

/* ----------------------------------------------------- */

extern void vot_init(void);
extern void vot_handler_task(void);

/* ----------------------------------------------------- */


#pragma pack(pop)
#endif // _TELEMETRY_PUBLIC
