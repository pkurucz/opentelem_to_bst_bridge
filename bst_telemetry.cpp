/* TeamBlacksheepBlacksheepTelemetry Protocol
 *  Datatype definitions and helper functions
 *  Most of the datatype definitions were
 *  taken from the bst files in the COLIBRI_RACE 
 *  directory in the betaflight repo.
 */

#include <stdbool.h>
#include <stdint.h>
#include <Wire.h>
#include <Arduino.h>
#include "util.h"
#include "config.h"
#include "vector_open_telemetry.h"
#include "bst_telemetry.h"

/* ----------------------------------------------------- */
/*                    Local Defines                     */

/* Enable for verbose debug lovin */
//#define BST_DEBUG

/* -------------- */

#define BST_BUFFER_SIZE                              128

#define BST_PROTOCOL_VERSION                         0
#define API_VERSION_MAJOR                            1 // increment when major changes are made
#define API_VERSION_MINOR                            13 // increment when any change is made, reset to zero when major changes are released after changing API_VERSION_MAJOR
#define API_VERSION_LENGTH                           2

/* Configure the CRC peripheral to use the polynomial x8 + x7 + x6 + x4 + x2 + 1 */
#define BST_CRC_POLYNOMIAL                              0xD5

/* I2C Addresses */    
#define I2C_ADDR_TBS_CORE_PNP_PRO                    0x80
#define I2C_ADDR_RESERVED                            0x8A
#define I2C_ADDR_PNP_PRO_DIDITAL_CURRENT_SENSOR      0xC0
#define I2C_ADDR_PNP_PRO_GPS                         0xC2
#define I2C_ADDR_TSB_BLACKBOX                        0xC4
#define I2C_ADDR_CLEANFLIGHT_FC                      0xC8
#define I2C_ADDR_CROSSFIRE_UHF_RECEIVER              0xEC

/* MSP Frame address */
#define PUBLIC_ADDRESS                               0x00

/* Frame Types */
#define GPS_POSITION_FRAME_ID               0x02    /* Len: 15 bytes (PX4) */
#define GPS_TIME_FRAME_ID                   0x03
#define BATTERY_STATUS_FRAME_ID             0x08
#define CROSSFIRE_RSSI_FRAME_ID             0x14
#define RC_CHANNEL_FRAME_ID                 0x15
#define RC_CHANNELS_PACKED_FRAME_ID         0x16    /* From: PX4, Len: 22 bytes, 11 bits per channel * 16 channels */
#define FC_ATTITUDE_FRAME_ID                0x1E    /* Len: 6 bytes (PX4) */
#define CLEANFLIGHT_MODE_FRAME_ID           0x20
#define FLIGHT_MODE_FRAME_ID                0x21    /* From: PX4,  */

/* CLEANFLIGHT_MODE_FRAME_ID bitfields */ 
#define BST_FLAG_ARMED             (0x01 << 0)
#define BST_FLAG_ANGLE_MODE        (0x01 << 1)
#define BST_FLAG_HORIZON_MODE      (0x01 << 2)
#define BST_FLAG_BARO_MODE         (0x01 << 3)
#define BST_FLAG_MAG_MODE          (0x01 << 4)
#define BST_FLAG_AIR_MODE          (0x01 << 5)
#define BST_FLAG_RANGEFINDER_MODE  (0x01 << 6)
#define BST_FLAG_FAILSAFE_MODE     (0x01 << 7)

#define BST_SENSOR_ACC             (0x01 << 0)
#define BST_SENSOR_BARO            (0x01 << 1)
#define BST_SENSOR_MAG             (0x01 << 2)
#define BST_SENSOR_GPS             (0x01 << 3)
#define BST_SENSOR_RANGEFINDER     (0x01 << 4)

#define USABLE_TIMER_CHANNEL_COUNT 8


/* ----------------------------------------------------- */
/*        Local Variables and function prototypes        */
static uint8_t bst_crc = 0;

static uint8_t bst_write_buffer[BST_BUFFER_SIZE];
static uint8_t bst_write_buffer_ptr;

static uint8_t bst_i2c_buffer[BST_BUFFER_SIZE] = {0};
static uint8_t bst_i2c_buffer_index = 0;
static uint8_t bst_i2c_buffer_len = 0;

/* -- Timestamp variables -- */
static uint32_t bst_RPY_TimestampMS;
static uint32_t bst_GPS_TimestampMS;
static uint32_t bst_Battery_TimestampMS;
static uint32_t bst_FC_Mode_TimestampMS;
static uint32_t bst_RC_Chan_TimestampMS;

/* -- Utility functions -- */
static void bst_calc_crc(uint8_t data_in);

static void bst_reset_buffer(uint8_t address);
static void bst_buffer8(uint8_t data);
static void bst_buffer16(uint16_t data);
static void bst_buffer32(uint32_t data);

static bool bst_write_busy(void);
static bool bst_master_write(uint8_t* data);
static void bst_write_loop(void);

/* -- Data conversion and output functions -- */

/* Vector translation/write functions */
static bool bst_write_vector_gps(void);
static bool bst_write_vector_battery(void);
static bool bst_write_vector_rpy(void);
static bool bst_write_vector_fc_mode(void);

/* Test write functions */
#if 0
static bool bst_read_rc_channels(void);
static bool bst_write_rc_channels(void);
#endif

/* ----------------------------------------------------- */

#define BST_HEX_PRINT(_X) do { hex_print((_X)); } while(0)
#define BST_PRINT(...) do { Serial.print(__VA_ARGS__); } while(0)
#define BST_PRINTLN(...) do { Serial.println(__VA_ARGS__); } while(0)

#ifdef BST_DEBUG
#define BST_HEX_DPRINT(_X) do { hex_print((_X)); } while(0)
#define BST_DPRINT(...) do { Serial.print(__VA_ARGS__); } while(0)
#define BST_DPRINTLN(...) do { Serial.println(__VA_ARGS__); } while(0)
#else
#define BST_HEX_DPRINT(_X) do{ } while(0) 
#define BST_DPRINT(...) do{ } while(0)
#define BST_DPRINTLN(...) do{ } while(0)
#endif


/* ----------------------------------------------------- */

void bst_init(void) {

	/* I2C Init, make sure the internal pull-ups are enabled */
#if 0
	pinMode (SDA, INPUT_PULLUP);
	pinMode (SCL, INPUT_PULLUP);
	digitalWrite(SDA, 1);
	digitalWrite(SCL, 1);
#endif

	/* Use Digital Pins 16 and 17 with external pull-up resistors to pull up SCL and SDA */
	pinMode (16, OUTPUT);
	pinMode (17, OUTPUT);
	digitalWrite(16, 1);
	digitalWrite(17, 1);

	Wire.begin();
	Wire.setClock(100000);

	/* Timestamp variables to keep track of the last time we sent an update over BST */
	bst_RPY_TimestampMS = 0;
	bst_GPS_TimestampMS = 0;
	bst_Battery_TimestampMS = 0;
	bst_FC_Mode_TimestampMS = 0;
	bst_RC_Chan_TimestampMS = 0;
}


void bst_handler_task(void) {

	/* Handle overall I2C tasks */
    bst_write_loop();

	/* Let other system tasks run while I2C is busy */
	if (bst_write_busy()) return;

	/* ---- Queue up the next I2C transaction ----- */
//	bst_write_vector_gps();
//	return;

	/* Telemetry update priority is based on the order things appear here */
	if(!vot_telemetry_valid) return;

	if(bst_RPY_TimestampMS != vot_telemetry.TimestampMS) {
		bst_RPY_TimestampMS = vot_telemetry.TimestampMS;

		LED_ON();

		bst_write_vector_rpy();
		return;
	}

	if(bst_GPS_TimestampMS != vot_telemetry.TimestampMS) {
		bst_GPS_TimestampMS = vot_telemetry.TimestampMS;

		bst_write_vector_gps();
		return;
	}

	if(bst_Battery_TimestampMS != vot_telemetry.TimestampMS) {
		bst_Battery_TimestampMS = vot_telemetry.TimestampMS;

		bst_write_vector_battery();
		return;
	}

	if(bst_FC_Mode_TimestampMS != vot_telemetry.TimestampMS) {
		bst_FC_Mode_TimestampMS = vot_telemetry.TimestampMS;

		bst_write_vector_fc_mode();
		return;
	}

#if 0
	if(bst_RC_Chan_TimestampMS != vot_telemetry.TimestampMS) {
		bst_RC_Chan_TimestampMS = vot_telemetry.TimestampMS;

		bst_read_rc_channels();
		return;
	}
#endif

	LED_OFF();


}

/* ----------------------------------------------------- */

static void bst_calc_crc(uint8_t data_in)
{
	/* polynomial = x^8+x^7+x^6+x^4+x^2+1 = x^8+x^7+x^6+x^4+x^2+X^0 */
	uint8_t polynomial = BST_CRC_POLYNOMIAL;
	bool MSB_Flag;

	/* Step through each bit of the BYTE (8-bits) */
	for (uint8_t i = 0; i < 8; i++) {
		/* Clear the Flag */
		MSB_Flag = false;

		/* MSB_Set = 80; */
		if (bst_crc & 0x80) {
			MSB_Flag = true;
		}

		bst_crc <<= 1;

		/* MSB_Set = 80; */
		if (data_in & 0x80) {
			bst_crc++;
		}
		data_in <<= 1;

		if (MSB_Flag == true) {
			bst_crc ^= polynomial;
		}
	}
}

/* ----------------------------------------------------- */


static void bst_reset_buffer(uint8_t address)
{   
    bst_write_buffer[0] = address;
    bst_write_buffer_ptr = 2;
}

static void bst_buffer8(uint8_t data)
{   
    bst_write_buffer[bst_write_buffer_ptr++] = data;
    bst_write_buffer[1] = bst_write_buffer_ptr;
}

static void bst_buffer16(uint16_t data)
{   
    bst_buffer8((uint8_t)(data >> 8));
    bst_buffer8((uint8_t)(data >> 0));
}

static void bst_buffer32(uint32_t data)
{   
    bst_buffer16((uint16_t)(data >> 16));
    bst_buffer16((uint16_t)(data >> 0));
}

static bool bst_write_busy(void)
{
    if (bst_i2c_buffer_len)
        return true;
    else
        return false;
}

static bool bst_master_write(uint8_t* data) 
{       
    if (bst_i2c_buffer_len==0) {
        bst_crc = 0;
        bst_i2c_buffer_index = 0;
        bst_i2c_buffer[0] = *data;
        bst_i2c_buffer[1] = *(data+1);
        bst_i2c_buffer_len = bst_i2c_buffer[1] + 2;
        for (uint8_t i=2; i<bst_i2c_buffer_len; i++) {
            if (i==(bst_i2c_buffer_len-1)) {
                bst_calc_crc(0);
                bst_i2c_buffer[i] = bst_crc;
            } else {
                bst_i2c_buffer[i] = *(data+i);
                bst_calc_crc((uint8_t)bst_i2c_buffer[i]);
            }
        }
        return true;
    }   
    return false;
}   
        
static void bst_write_loop(void)
{
    if (bst_i2c_buffer_len && bst_i2c_buffer_index==0) {
		BST_DPRINT(F("I2C Write, bst_i2c_buffer_len="));
		BST_DPRINT(bst_i2c_buffer_len, DEC);
		BST_DPRINT(F(" ... "));

		Wire.beginTransmission(bst_i2c_buffer[0]);
        bst_i2c_buffer_index = 1;

		BST_HEX_DPRINT(bst_i2c_buffer[0]);

		for(int i = 1; i < bst_i2c_buffer_len; i++) {
			BST_HEX_DPRINT(bst_i2c_buffer[bst_i2c_buffer_index]);
			Wire.write(bst_i2c_buffer[bst_i2c_buffer_index]);

			bst_i2c_buffer_index++;
		}

		uint8_t error = Wire.endTransmission();

		switch(error) {
			case 0:
				BST_DPRINTLN(F("Success"));
				break;
			case 1:
				BST_DPRINTLN(F("1-Data too long"));
				break;
			case 2:
				BST_DPRINTLN(F("2-NACK on addr"));
				break;
			case 3:
				BST_DPRINTLN(F("3-NACK on data"));
				break;
			default:
				BST_DPRINT(error,DEC);
				BST_DPRINTLN(F("-Other error"));
				break;
		}

		bst_i2c_buffer_index = 0;
		bst_i2c_buffer_len = 0;
    }
}

static bool bst_write_vector_gps(void)
{

	uint32_t lat = vot_telemetry.GPSTelemetry.LatitudeX1E7; // BST: LatitudeX1E7
	uint32_t lon = vot_telemetry.GPSTelemetry.LongitudeX1E7; // BST: LongitudeX1E7
	uint16_t speed = vot_telemetry.GPSTelemetry.GroundspeedKPHX10; // BST: GroundspeedKPHX10
	uint16_t alt = vot_telemetry.GPSTelemetry.GPSAltitudecm / 100; // BST: GPSAltitudeM
	uint16_t altitude = alt + 1000; // BST: in Meters, +1000 added as offset
	uint8_t numOfSat = vot_telemetry.GPSTelemetry.SatsInUse; // BST: Number of Sats

#if 1
	/* Send coordinates as 0 - 2X3.14159(PI)X10000 */
	/* This is the default for auto-discovery... */
	uint16_t gpsHeading = (62832 * ((uint32_t)vot_telemetry.GPSTelemetry.CourseDegrees)) / 360;
#else
	/* Send coordinates as 0 - 360X100 degrees */
	/* Scaling needs to be done in the Taranis. Ratio = 25.5 works */
	uint16_t gpsHeading = 100 * vot_telemetry.GPSTelemetry.CourseDegrees;
#endif

#ifdef BST_CONFIG_TELEM_USE_VECTOR_AIRSPEED
	speed = vot_telemetry.SensorTelemetry.AirspeedKPHX10; // BST: AirspeedKPHX10, requires optional pitot sensor
#endif

	BST_DPRINT(F("Write GPS Position... "));
	BST_DPRINT(altitude, DEC);
	BST_DPRINT(F(", "));
	BST_DPRINTLN(gpsHeading, DEC);

	bst_reset_buffer(PUBLIC_ADDRESS);
	bst_buffer8(GPS_POSITION_FRAME_ID);
	bst_buffer32(lat); // Status: Complete
	bst_buffer32(lon); // Status: Complete
	bst_buffer16(speed); // Status: ( km/h * 10 )
	bst_buffer16(gpsHeading); // Status: ?
	bst_buffer16(altitude); // Status: OK, in Meters, +1000 added as offset
	bst_buffer8(numOfSat); // Status: OK
	bst_buffer8(0x00); // Status: ?
		
	return bst_master_write(bst_write_buffer);
}

static bool bst_write_vector_battery(void)
{
	uint16_t voltage = vot_telemetry.SensorTelemetry.PackVoltageX100 / 10; // BST: VoltageX10
	uint16_t current = vot_telemetry.SensorTelemetry.PackCurrentX10; // BST: CurrentX10
	uint32_t mAHConsumed = vot_telemetry.SensorTelemetry.mAHConsumed; // BST: mAHConsumed

	BST_DPRINTLN(F("Write Battery Status..."));

	bst_reset_buffer(PUBLIC_ADDRESS);
	bst_buffer8(BATTERY_STATUS_FRAME_ID);
	bst_buffer16(voltage); // Status: Complete
	bst_buffer16(current); // Status: Complete
	bst_buffer8(mAHConsumed >> 16); // Status: Complete
	bst_buffer8(mAHConsumed >> 8);
	bst_buffer8(mAHConsumed);
	// bst_buffer8(mAHConsumed); // Battery Percentage... this only appears in CRSF code so for now we don't use this

	return bst_master_write(bst_write_buffer);
}


static bool bst_write_vector_rpy(void)
{
	/* Send coordinates as +/- 3.14159(PI)X10000 */
	/* This is the default for auto-discovery... */
	int16_t X = (31416 * ((int32_t)vot_telemetry.SensorTelemetry.Attitude.PitchDegrees)) / 180; 
	int16_t Y = (31416 * ((int32_t)vot_telemetry.SensorTelemetry.Attitude.RollDegrees)) / 180;
	int16_t Z = (31416 * ((int32_t)vot_telemetry.SensorTelemetry.Attitude.YawDegrees)) / 180;

	BST_DPRINTLN(F("Write RPY..."));

	bst_reset_buffer(PUBLIC_ADDRESS);
	bst_buffer8(FC_ATTITUDE_FRAME_ID);
	bst_buffer16(X); // Status: Complete
	bst_buffer16(Y); // Status: Complete
	bst_buffer16(Z); // Status: Complete

	return bst_master_write(bst_write_buffer);
}

#if 0
static bool bst_read_rc_channels(void)
{
	const int len = 32;

	BST_PRINT(F("Read RC Pos..."));

	Wire.requestFrom(PUBLIC_ADDRESS, len);    // request len bytes from slave device

	while(Wire.available())    // slave may send less than requested
	{	 
		char c = Wire.read();    // receive a byte as character
		BST_HEX_PRINT(c);         // print the character
		BST_PRINT(' ');
	}

	BST_PRINTLN(F(""));

	return true;
}

static bool bst_write_rc_channels(void)
{
	uint8_t i = 0;

	BST_PRINTLN(F("Write RC Pos..."));

	bst_reset_buffer(PUBLIC_ADDRESS);
	bst_buffer8(RC_CHANNEL_FRAME_ID);
	for (i = 0; i < (USABLE_TIMER_CHANNEL_COUNT-1); i++) {
		bst_buffer16(100 * i);
	}
	
	return bst_master_write(bst_write_buffer);
}
#endif

static bool bst_write_vector_fc_mode(void)
{
	uint8_t fm = MIN(vot_telemetry.PresentFlightMode, VECTOR_FLIGHT_MODE_MAX);  // Clamp the flight mode index to the legit range
	const char * fm_str = vot_flight_mode_strings[fm];
	uint8_t len = MIN(15, strlen(fm_str));  // Maximum string length of 16 bytes including the NULL

	BST_PRINT(F("Write FC Mode..."));
	BST_HEX_PRINT(vot_telemetry.PresentFlightMode);
	BST_PRINTLN(F(""));

	bst_reset_buffer(PUBLIC_ADDRESS);
	bst_buffer8(FLIGHT_MODE_FRAME_ID);

	for(int i = 0; i < len; i++) bst_buffer8(fm_str[i]);
	bst_buffer8(0); // NULL Terminate the string

	return bst_master_write(bst_write_buffer);
}




