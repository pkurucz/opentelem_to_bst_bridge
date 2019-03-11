/* Eagletree Vector Open Telemetry Protocol Handler
 *  Datatype definitions and helper functions
 */

#include <stdint.h>
#include <Arduino.h>
#include "config.h"
#include "util.h"
#include "vector_open_telemetry.h"

/* ----------------------------------------------------- */
/*                   Local Defines                       */

// Calculate the CRC as each byte is received instead of 
// as a block after all data is received.
#define VOT_STREAMING_CRC

// Use the hardware UART instead of a SoftwareSerial one
//#define VOT_HARDWARE_UART
#define VOT_ALTSOFTSERIAL_UART   /* Pins: TX: 9, RX: 8, Unusable PWM: 10 */

// Debug prints of the received data.  Turn off for a deployed release
//#define VOT_PRINT_TELEMETRY_DATA

#define VOT_TELEMETRY_TIMEOUT_MS (3 * 1000)

/* ----------------------------------------------------- */
/*                   Utility Macros                      */

#define VOT_NTOHL(_X) (_X) = ntohl((_X))
#define VOT_NTOHS(_X) (_X) = ntohs((_X))


/* ----------------------------------------------------- */
/*                  Global Variables                     */
VECTOR_OPEN_TELEMETRY vot_telemetry;
bool vot_telemetry_valid;

unsigned long vot_telemetry_prev_timestamp;
unsigned long vot_telemetry_timestamp;
uint32_t vot_prev_TimestampMS;

const char * const vot_flight_mode_strings[] = {
	"2D",
	"2D_AH",
	"2D_HH",
	"2D_AH_HH",
	"LOITER",
	"3D",
	"3D_HH",
	"RTH",
	"LAND",
	"CART",
	"CART_LOITER",
	"POLAR",
	"POLAR_LOITER",
	"CENTER_STICK",
	"OFF",
	"WAYPOINT",
	"MAX",
};

/* ----------------------------------------------------- */
/*        Local Variables and function prototypes        */

static uint8_t parse_state = 0;
static uint8_t parse_buffer[sizeof(VECTOR_OPEN_TELEMETRY)];
static uint16_t parse_crc;


static void vot_ntoh(void);
static uint16_t vot_CRC16Worker(uint16_t icrc, uint8_t r0);

#if !defined(VOT_STREAMING_CRC)
static uint16_t vot_CalculateCRC(uint8_t * pPacket, uint8_t Size, uint16_t InitCRC);
#endif


/* ----------------------------------------------------- */
/*               UART interface prototype                */

#if defined(VOT_HARDWARE_UART)
#define VOT_UART Serial
#elif defined(VOT_ALTSOFTSERIAL_UART)
#include <AltSoftSerial.h>
AltSoftSerial vot_uart;
#define VOT_UART vot_uart
#else
#include <SoftwareSerial.h>
SoftwareSerial vot_uart(8, 9);
#define VOT_UART vot_uart
#endif


/* ----------------------------------------------------- */

#if defined(VOT_PRINT_TELEMETRY_DATA)
static void vot_print_telemetry_data(void);
#endif

#if defined(VOT_PRINT_TELEMETRY_DATA)
static const char * const vot_flightmodes[] = {
	"2D",
	"2D_ALT_HOLD",
	"2D_HEADING_HOLD",
	"2D_ALT_HEADING_HOLD",
	"LOITER",
	"3D",
	"3D_HEADING_HOLD",
	"RTH",
	"LAND",
	"CARTESIAN",
	"CARTESIAN_LOITER",
	"POLAR",
	"POLAR_LOITER",
	"CENTER_STICK",
	"OFF",
	"WAYPOINT",
	"UNKNOWN",
};
#endif

/* ----------------------------------------------------- */

void vot_init(void) {
#if !defined(VOT_HARDWARE_UART)
  /* Init the uart class to the right baud */
	VOT_UART.begin(57600);
#endif

  /* Set state variables to sane defaults */
	parse_state = 0;
	vot_telemetry_valid = false;
	vot_telemetry_timestamp = millis();
	vot_telemetry_prev_timestamp = vot_telemetry_timestamp;
	vot_prev_TimestampMS = 0;
	memset(&vot_telemetry, 0, sizeof(vot_telemetry));

}

void vot_handler_task(void) {
	while(VOT_UART.available()) {
		uint8_t ch = VOT_UART.read();

		switch(parse_state) {
			/* VECTOR_TELEMETRY_PACKET_START_CODE = 0xB01EDEAD */
			/* We manually match the first 4 bytes, this lets  */
			/* is quickly re-sync if the pattern happens to    */
			/* occur in the data stream as well.               */
			case 0:
				if(ch != 0xB0) {parse_state = 0; continue;}
				parse_crc = VTELEM_CRC_INIT;
				break;
			case 1:
				if(ch != 0x1E) {parse_state = 0; continue;}
				break;
			case 2:
				if(ch != 0xDE) {parse_state = 0; continue;}
				break;
			case 3:
				if(ch != 0xAD) {parse_state = 0; continue;}
				break;
			default:
				break;
		}

		/* Store each byte */
		parse_buffer[parse_state] = ch;

		/* Compute the running CRC */
#if defined(VOT_STREAMING_CRC)
		if(parse_state < (sizeof(parse_buffer) - sizeof(vot_telemetry.CRC)) ) {
	    	parse_crc = vot_CRC16Worker(parse_crc, ch);
		}
#endif

		/* Check if we received enough data */
		parse_state++;
		if(parse_state >= sizeof(parse_buffer)) {
			uint16_t * crc = (uint16_t *) &parse_buffer[offsetof(VECTOR_OPEN_TELEMETRY, CRC)];

			/* Calculate the CRC as one block (if we weren't in streaming mode) */
#if !defined(VOT_STREAMING_CRC)
			parse_crc = vot_CalculateCRC((uint8_t *)&vot_telemetry, offsetof(VECTOR_OPEN_TELEMETRY, CRC), VTELEM_CRC_INIT);
#endif

			/* Make sure the CRC's match */
			if(parse_crc == *crc) {
				vot_prev_TimestampMS = vot_telemetry.TimestampMS;

				memcpy(&vot_telemetry, parse_buffer, sizeof(vot_telemetry));

				/* Byte swap to the local endianness */
				vot_ntoh();

				vot_telemetry_valid = true;
				vot_telemetry_prev_timestamp = vot_telemetry_timestamp;
				vot_telemetry_timestamp = millis();

				/* Debug print out the telemetry data */
#if defined(VOT_PRINT_TELEMETRY_DATA)
				vot_print_telemetry_data();
#endif
        Serial.print('.');
//				Serial.println(F("VOT!"));

			} else {
        Serial.println('*');
//        Serial.println(F("!VOT_CRC!"));
			}
			parse_state = 0;
		}
	}

	/* Clear the telemetry valid flag if the data is stale */
	if(vot_telemetry_valid && ((millis() - vot_telemetry_timestamp) > VOT_TELEMETRY_TIMEOUT_MS)) {
		vot_telemetry_valid = false;
    Serial.println('!');
  //		Serial.println(F("!VOT_TELEM_TIMEOUT!"));
	}


}

/* ----------------------------------------------------- */

#if defined(VOT_PRINT_TELEMETRY_DATA)

#define VOT_TELEM_PRINT(_X, _FORMAT) \
	do { \
		Serial.print(F(#_X)); \
		Serial.print(F(": ")); \
		Serial.println(vot_telemetry._X, (_FORMAT)); \
	} while (0)

void vot_print_telemetry_data(void) {
	Serial.println(F("VOT RX:"));

#if 0
	Serial.print(F("sizeof: "));
	Serial.println(sizeof(vot_telemetry), DEC);
#endif

	VOT_TELEM_PRINT(CRC, HEX);
	if(parse_crc != vot_telemetry.CRC) Serial.print(F("\t*** "));
	Serial.print(F("parse_crc: "));
	Serial.print(parse_crc, HEX);

	/* --------------------------------------- */

	VOT_TELEM_PRINT(StartCode, HEX);
	VOT_TELEM_PRINT(TimestampMS, DEC);

	/* --------------------------------------- */

	VOT_TELEM_PRINT(SensorTelemetry.BaroAltitudecm, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.AirspeedKPHX10, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.ClimbRateMSX100, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.RPM, DEC);

	VOT_TELEM_PRINT(SensorTelemetry.Attitude.PitchDegrees, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.Attitude.RollDegrees, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.Attitude.YawDegrees, DEC);

	VOT_TELEM_PRINT(SensorTelemetry.Acceleration.AccelXCentiGrav, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.Acceleration.AccelYCentiGrav, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.Acceleration.AccelZCentiGrav, DEC);

	VOT_TELEM_PRINT(SensorTelemetry.PackVoltageX100, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.VideoTxVoltageX100, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.CameraVoltageX100, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.RxVoltageX100, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.PackCurrentX10, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.TempDegreesCX10, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.mAHConsumed, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.CompassDegrees, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.RSSIPercent, DEC);
	VOT_TELEM_PRINT(SensorTelemetry.LQPercent, DEC);

	VOT_TELEM_PRINT(GPSTelemetry.LatitudeX1E7, DEC);
	VOT_TELEM_PRINT(GPSTelemetry.LongitudeX1E7, DEC);
	VOT_TELEM_PRINT(GPSTelemetry.DistanceFromHomeMX10, DEC);
	VOT_TELEM_PRINT(GPSTelemetry.GroundspeedKPHX10, DEC);
	VOT_TELEM_PRINT(GPSTelemetry.CourseDegrees, DEC);
	VOT_TELEM_PRINT(GPSTelemetry.GPSAltitudecm, DEC);
	VOT_TELEM_PRINT(GPSTelemetry.HDOPx10, DEC);
	VOT_TELEM_PRINT(GPSTelemetry.SatsInUse, DEC);

	/* --------------------------------------- */

	Serial.print(F("PresentFlightMode: "));
	if(vot_telemetry.PresentFlightMode < VECTOR_FLIGHT_MODE_MAX) {
		Serial.println(vot_flightmodes[vot_telemetry.PresentFlightMode]);
	} else {
		Serial.println(vot_flightmodes[VECTOR_FLIGHT_MODE_MAX]);
	}

//	delay(300);
}
#endif /* defined(VOT_PRINT_TELEMETRY_DATA) */


/* Byte swap to the local endianness */
void vot_ntoh(void) {
	VOT_NTOHL(vot_telemetry.StartCode);
//	VOT_NTOHL(vot_telemetry.TimestampMS);  // The timestamp appears to be swapped already...

//	VECTOR_SENSOR_TELEMETRY SensorTelemetry;
		VOT_NTOHL(vot_telemetry.SensorTelemetry.BaroAltitudecm);
		VOT_NTOHS(vot_telemetry.SensorTelemetry.AirspeedKPHX10);
		VOT_NTOHS(vot_telemetry.SensorTelemetry.ClimbRateMSX100);
		VOT_NTOHS(vot_telemetry.SensorTelemetry.RPM);

//		VECTOR_ATTITUDE Attitude;
			VOT_NTOHS(vot_telemetry.SensorTelemetry.Attitude.PitchDegrees);
			VOT_NTOHS(vot_telemetry.SensorTelemetry.Attitude.RollDegrees);
			VOT_NTOHS(vot_telemetry.SensorTelemetry.Attitude.YawDegrees);

//		VECTOR_ACCELERATION Acceleration;
			VOT_NTOHS(vot_telemetry.SensorTelemetry.Acceleration.AccelXCentiGrav);
			VOT_NTOHS(vot_telemetry.SensorTelemetry.Acceleration.AccelYCentiGrav);
			VOT_NTOHS(vot_telemetry.SensorTelemetry.Acceleration.AccelZCentiGrav);

		VOT_NTOHS(vot_telemetry.SensorTelemetry.PackVoltageX100);
		VOT_NTOHS(vot_telemetry.SensorTelemetry.VideoTxVoltageX100);
		VOT_NTOHS(vot_telemetry.SensorTelemetry.CameraVoltageX100);
		VOT_NTOHS(vot_telemetry.SensorTelemetry.RxVoltageX100);
		VOT_NTOHS(vot_telemetry.SensorTelemetry.PackCurrentX10);
		VOT_NTOHS(vot_telemetry.SensorTelemetry.TempDegreesCX10);
		VOT_NTOHS(vot_telemetry.SensorTelemetry.mAHConsumed);
		VOT_NTOHS(vot_telemetry.SensorTelemetry.CompassDegrees);

//	VECTOR_GPS_TELEMETRY GPSTelemetry;
		VOT_NTOHL(vot_telemetry.GPSTelemetry.LatitudeX1E7);
		VOT_NTOHL(vot_telemetry.GPSTelemetry.LongitudeX1E7);
		VOT_NTOHL(vot_telemetry.GPSTelemetry.DistanceFromHomeMX10);
		VOT_NTOHS(vot_telemetry.GPSTelemetry.GroundspeedKPHX10);
		VOT_NTOHS(vot_telemetry.GPSTelemetry.CourseDegrees);
		VOT_NTOHL(vot_telemetry.GPSTelemetry.GPSAltitudecm);

}

void vot_hex_print(uint8_t byte) {
	if (byte < 16) Serial.print("0");
	Serial.print(byte, HEX);
}


/* ----------------------------------------------------- */

// sample code for calculating the CRC
//
// OpenTelem.CRC == CalculateCRC((uint8_t *)&OpenTelem, offsetof(VECTOR_OPEN_TELEMETRY, CRC), VTELEM_CRC_INIT);

static uint16_t vot_CRC16Worker(uint16_t icrc, uint8_t r0)
{
  union {
    uint16_t crc16; // 16-bit CRC
    struct {
      uint8_t crcl, crch;
    } s;
  } u;

  uint8_t a1;    // scratch byte

  u.crc16 = icrc;

  r0 = r0 ^ u.s.crch;
  a1 = u.s.crcl;
  u.s.crch = r0;
  u.s.crch = (u.s.crch << 4) | (u.s.crch >> 4);
  u.s.crcl = u.s.crch ^ r0;
  u.crc16 &= 0x0ff0;
  r0 ^= u.s.crch;
  a1 ^= u.s.crcl;
  u.crc16 <<= 1;
  u.s.crcl ^= r0;
  u.s.crch ^= a1;

  return (u.crc16);
}

#if !defined(VOT_STREAMING_CRC)
static uint16_t vot_CalculateCRC(uint8_t * pPacket, uint8_t Size, uint16_t InitCRC)
{
  uint16_t i;
  uint16_t CRC;

  CRC = InitCRC;

  for (i = 0; i < Size; i++) {

    CRC = vot_CRC16Worker(CRC, pPacket[i]);
  }

  return CRC;
}
#endif

/* ----------------------------------------------------- */

