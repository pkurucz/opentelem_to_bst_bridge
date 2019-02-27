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
/*                   Local Datatypes                     */

#define ENABLE_BST_DEBUG 0

#if ENABLE_BST_DEBUG

#define BST_PRINT(...) \
	do { \
		Serial.print(__VA_ARGS__); \
	} while(0)

#define BST_PRINTLN(...) \
	do { \
		Serial.println(__VA_ARGS__); \
	} while(0)

#define BST_VPRINT(...) \
	do { \
		if(1) { \
			Serial.print(__VA_ARGS__); \
		} \
	} while(0)

#define BST_VPRINTLN(...) \
	do { \
		if(1) { \
			Serial.println(__VA_ARGS__); \
		} \
	} while(0)


#define BST_DPRINT(...) \
	do { \
		BST_VPRINT(__VA_ARGS__); \
	} while(0)

#define BST_DPRINTLN(...) \
	do { \
		BST_VPRINTLN(__VA_ARGS__); \
	} while(0)

#else

#define BST_PRINT(...)  do{ } while(0)
#define BST_PRINTLN(...)  do{ } while(0)

#define BST_VPRINT(...)  do{ } while(0)
#define BST_VPRINTLN(...)  do{ } while(0)

#define BST_DPRINT(...)  do{ } while(0)
#define BST_DPRINTLN(...)  do{ } while(0)

#endif

/* ----------------------------------------------------- */
/*        Local Variables and function prototypes        */
static volatile uint8_t CRC8 = 0;

static void bst_crc8Cal(uint8_t data_in);
static void bstMasterStartBuffer(uint8_t address);
static void bstMasterWrite8(uint8_t data);
static void bstMasterWrite16(uint16_t data);
static void bstMasterWrite32(uint32_t data);
static bool bstWriteBusy(void);
static bool bstMasterWrite(uint8_t* data);
static void bstMasterWriteLoop(void);

static bool writeGpsPositionFrameToBST(void);
static bool writeBatteryStatusToBST(void);
static bool writeRollPitchYawToBST(void);
static bool writeRCChannelToBST(void);
static bool writeFCModeToBST(void);


/* ----------------------------------------------------- */

void hex_print(uint8_t byte) {
	if (byte < 16)
		BST_DPRINT("0");
	BST_DPRINT(byte, HEX);		
	BST_DPRINT(" ");
}

uint32_t bst_RPY_TimestampMS;
uint32_t bst_GPS_TimestampMS;
uint32_t bst_Battery_TimestampMS;
uint32_t bst_FCMode_TimestampMS;

void bst_init(void) {

	/* I2C Init, make sure the internal pull-ups are enabled */
#if 0
	pinMode (SDA, INPUT_PULLUP);
	pinMode (SCL, INPUT_PULLUP);
	digitalWrite(SDA, 1);
	digitalWrite(SCL, 1);
#endif

	/* Use Digital Pins 16 and 17 with external pull-up resistors to pull up SCL and SDA */
#if 1
	pinMode (16, OUTPUT);
	pinMode (17, OUTPUT);
	digitalWrite(16, 1);
	digitalWrite(17, 1);
#endif

	Wire.begin();
	Wire.setClock(100000);

	/* Timestamp variables to keep track of the last time we sent an update over BST */
	bst_RPY_TimestampMS = 0;
	bst_GPS_TimestampMS = 0;
	bst_Battery_TimestampMS = 0;
	bst_FCMode_TimestampMS = 0;
}


void bst_handler_task(void) {

	/* Handle overall I2C tasks */
    bstMasterWriteLoop();

	/* Let other system tasks run while I2C is busy */
	if (bstWriteBusy()) return;

	/* ---- Queue up the next I2C transaction ----- */
//	writeGpsPositionFrameToBST();
//	return;

	/* Telemetry update priority is based on the order things appear here */
	if(!vot_telemetry_valid) return;

	if(bst_RPY_TimestampMS != vot_telemetry.TimestampMS) {
		bst_RPY_TimestampMS = vot_telemetry.TimestampMS;

		LED_ON();

		writeRollPitchYawToBST();
		return;
	}

	if(bst_GPS_TimestampMS != vot_telemetry.TimestampMS) {
		bst_GPS_TimestampMS = vot_telemetry.TimestampMS;

		writeGpsPositionFrameToBST();
		return;
	}

	if(bst_Battery_TimestampMS != vot_telemetry.TimestampMS) {
		bst_Battery_TimestampMS = vot_telemetry.TimestampMS;

		writeBatteryStatusToBST();
		return;
	}

	LED_OFF();

#if 0
	writeFCModeToBST();
	writeRCChannelToBST();
#endif


#if 0
	Serial.println("Scanning...");
	i2c_scan();
#endif

}

/* ----------------------------------------------------- */

static void bst_crc8Cal(uint8_t data_in)
{
	/* Polynom = x^8+x^7+x^6+x^4+x^2+1 = x^8+x^7+x^6+x^4+x^2+X^0 */
	uint8_t Polynom = BST_CRC_POLYNOM;
	bool MSB_Flag;

	/* Step through each bit of the BYTE (8-bits) */
	for (uint8_t i = 0; i < 8; i++) {
		/* Clear the Flag */
		MSB_Flag = false;

		/* MSB_Set = 80; */
		if (CRC8 & 0x80) {
			MSB_Flag = true;
		}

		CRC8 <<= 1;

		/* MSB_Set = 80; */
		if (data_in & 0x80) {
			CRC8++;
		}
		data_in <<= 1;

		if (MSB_Flag == true) {
			CRC8 ^= Polynom;
		}
	}
}

/* ----------------------------------------------------- */

static uint8_t masterWriteBufferPointer;
static uint8_t masterWriteData[BST_BUFFER_SIZE];

static uint8_t dataBuffer[BST_BUFFER_SIZE] = {0};
static uint8_t dataBufferPointer = 0;
static uint8_t bstWriteDataLen = 0;


static void bstMasterStartBuffer(uint8_t address)
{   
    masterWriteData[0] = address;
    masterWriteBufferPointer = 2;
}

static void bstMasterWrite8(uint8_t data)
{   
    masterWriteData[masterWriteBufferPointer++] = data;
    masterWriteData[1] = masterWriteBufferPointer;
}

static void bstMasterWrite16(uint16_t data)
{   
    bstMasterWrite8((uint8_t)(data >> 8));
    bstMasterWrite8((uint8_t)(data >> 0));
}

static void bstMasterWrite32(uint32_t data)
{   
    bstMasterWrite16((uint16_t)(data >> 16));
    bstMasterWrite16((uint16_t)(data >> 0));
}

static bool bstWriteBusy(void)
{
    if (bstWriteDataLen)
        return true;
    else
        return false;
}

static bool bstMasterWrite(uint8_t* data) 
{       
    if (bstWriteDataLen==0) {
        CRC8 = 0;
        dataBufferPointer = 0;
        dataBuffer[0] = *data;
        dataBuffer[1] = *(data+1);
        bstWriteDataLen = dataBuffer[1] + 2;
        for (uint8_t i=2; i<bstWriteDataLen; i++) {
            if (i==(bstWriteDataLen-1)) {
                bst_crc8Cal(0);
                dataBuffer[i] = CRC8;
            } else {
                dataBuffer[i] = *(data+i);
                bst_crc8Cal((uint8_t)dataBuffer[i]);
            }
        }
        return true;
    }   
    return false;
}   
        
static void bstMasterWriteLoop(void)
{
    if (bstWriteDataLen && dataBufferPointer==0) {
		BST_DPRINT("I2C Write, bstWriteDataLen=");
		BST_DPRINT(bstWriteDataLen, DEC);
		BST_DPRINT(" ... ");

		Wire.beginTransmission(dataBuffer[0]);
        dataBufferPointer = 1;

#if ENABLE_BST_DEBUG
		hex_print(dataBuffer[0]);
#endif

		for(int i = 1; i < bstWriteDataLen; i++) {
#if ENABLE_BST_DEBUG
			hex_print(dataBuffer[dataBufferPointer]);
#endif
			Wire.write(dataBuffer[dataBufferPointer]);

			dataBufferPointer++;
		}

		uint8_t error = Wire.endTransmission();

#if ENABLE_BST_DEBUG
		switch(error) {
			case 0:
				BST_DPRINTLN("Success");
				break;
			case 1:
				BST_DPRINTLN("1-Data too long");
				break;
			case 2:
				BST_DPRINTLN("2-NACK on addr");
				break;
			case 3:
				BST_DPRINTLN("3-NACK on data");
				break;
			default:
				BST_DPRINT(error,DEC);
				BST_DPRINTLN("-Other error");
				break;
		}
#endif

		dataBufferPointer = 0;
		bstWriteDataLen = 0;
    }
}


/* 
	 0: addr
	 1: len - Number of bytes including addr and len, doesn't include crc
	 2: frame id
	 3: lat
	 4: lat
	 5: lat
	 6: lat
	 7: lon
	 8: lon
	 9: lon
	10: lon
	11: speed
	12: speed
	13: heading
	14: heading
	15: alt
	16: alt
	17: numsat
	18: zero
	19: crc8
*/

static bool writeGpsPositionFrameToBST(void)
{

	uint32_t lat = vot_telemetry.GPSTelemetry.LatitudeX1E7; // BST: LatitudeX1E7
	uint32_t lon = vot_telemetry.GPSTelemetry.LongitudeX1E7; // BST: LongitudeX1E7

#if 1
	uint16_t speed = vot_telemetry.SensorTelemetry.AirspeedKPHX10; // BST: AirspeedKPHX10, requires optional pitot sensor
#else
	uint16_t speed = vot_telemetry.GPSTelemetry.GroundspeedKPHX10; // BST: GroundspeedKPHX10
#endif

#if 1
	/* Send coordinates as 0 - 2X3.14159(PI)X10000 */
	/* This is the default for auto-discovery... */
	uint16_t gpsHeading = (62832 * ((uint32_t)vot_telemetry.GPSTelemetry.CourseDegrees)) / 360;
#else
	/* Send coordinates as 0 - 360X100 degrees */
	/* Scaling needs to be done in the Taranis. Ratio = 25.5 works */
	uint16_t gpsHeading = 100 * vot_telemetry.GPSTelemetry.CourseDegrees;
#endif

#if 1
	uint16_t alt = vot_telemetry.GPSTelemetry.GPSAltitudecm / 100; // BST: GPSAltitudeM
#else
	uint16_t alt = vot_telemetry.GPSTelemetry.GPSAltitudecm / 100; // BST: GPSAltitudeM
#endif
	uint16_t altitude = alt + 1000; // BST: in Meters, +1000 added as offset

	uint8_t numOfSat = vot_telemetry.GPSTelemetry.SatsInUse; // BST: Number of Sats
		
	BST_VPRINT("Write GPS Position... ");
	BST_VPRINT(altitude, DEC);
	BST_VPRINT(", ");
	BST_VPRINTLN(gpsHeading, DEC);

	bstMasterStartBuffer(PUBLIC_ADDRESS);
	bstMasterWrite8(GPS_POSITION_FRAME_ID);
	bstMasterWrite32(lat); // Status: Complete
	bstMasterWrite32(lon); // Status: Complete
	bstMasterWrite16(speed); // Status: ( km/h * 10 )
	bstMasterWrite16(gpsHeading); // Status: ?
	bstMasterWrite16(altitude); // Status: OK, in Meters, +1000 added as offset
	bstMasterWrite8(numOfSat); // Status: OK
	bstMasterWrite8(0x00); // Status: ?
		
	return bstMasterWrite(masterWriteData);
}

static bool writeBatteryStatusToBST(void)
{
	uint16_t voltage = vot_telemetry.SensorTelemetry.PackVoltageX100 / 10; // BST: VoltageX10
	uint16_t current = vot_telemetry.SensorTelemetry.PackCurrentX10; // BST: CurrentX10
	uint32_t mAHConsumed = vot_telemetry.SensorTelemetry.mAHConsumed; // BST: mAHConsumed

	BST_VPRINTLN("Write Battery Status...");

	bstMasterStartBuffer(PUBLIC_ADDRESS);
	bstMasterWrite8(BATTERY_STATUS_FRAME_ID);
	bstMasterWrite16(voltage); // Status: Complete
	bstMasterWrite16(current); // Status: Complete
	bstMasterWrite8(mAHConsumed >> 16); // Status: Complete
	bstMasterWrite8(mAHConsumed >> 8);
	bstMasterWrite8(mAHConsumed);
	// bstMasterWrite8(mAHConsumed); // Battery Percentage... this only appears in CRSF code so for now we don't use this

	return bstMasterWrite(masterWriteData);
}


static bool writeRollPitchYawToBST(void)
{
#if 1
	/* Send coordinates as +/- 3.14159(PI)X10000 */
	/* This is the default for auto-discovery... */
	int16_t X = (31416 * ((int32_t)vot_telemetry.SensorTelemetry.Attitude.PitchDegrees)) / 180; 
	int16_t Y = (31416 * ((int32_t)vot_telemetry.SensorTelemetry.Attitude.RollDegrees)) / 180;
	int16_t Z = (31416 * ((int32_t)vot_telemetry.SensorTelemetry.Attitude.YawDegrees)) / 180;
#else
	/* Send coordinates as +/- 180X100 degrees */
	/* Scaling needs to be done in the Taranis. Ratio = 25.5 works */
	int16_t X = vot_telemetry.SensorTelemetry.Attitude.PitchDegrees * 100; 
	int16_t Y = vot_telemetry.SensorTelemetry.Attitude.RollDegrees * 100;
	int16_t Z = vot_telemetry.SensorTelemetry.Attitude.YawDegrees * 100;
#endif

	BST_VPRINTLN("Write RPY...");

	bstMasterStartBuffer(PUBLIC_ADDRESS);
	bstMasterWrite8(FC_ATTITUDE_FRAME_ID);
	bstMasterWrite16(X); // Status: Complete
	bstMasterWrite16(Y); // Status: Complete
	bstMasterWrite16(Z); // Status: Complete

	return bstMasterWrite(masterWriteData);
}

static bool writeRCChannelToBST(void)
{
#if 0
	uint8_t i = 0;
	bstMasterStartBuffer(PUBLIC_ADDRESS);
	bstMasterWrite8(RC_CHANNEL_FRAME_ID);
	for (i = 0; i < (USABLE_TIMER_CHANNEL_COUNT-1); i++) {
		bstMasterWrite16(rcData[i]);
	}
	
	return bstMasterWrite(masterWriteData);
#else
	BST_VPRINTLN("Write RC Pos...");
	return true;
#endif
}

static bool writeFCModeToBST(void)
{
	uint8_t flags = 0;
	uint8_t sensors = 0;

	flags = 0 |
#if 0
	        IS_ENABLED(ARMING_FLAG(ARMED)) |
	        IS_ENABLED(FLIGHT_MODE(ANGLE_MODE)) << 1 |
	        IS_ENABLED(FLIGHT_MODE(HORIZON_MODE)) << 2 |
	        IS_ENABLED(FLIGHT_MODE(BARO_MODE)) << 3 |
	        IS_ENABLED(FLIGHT_MODE(MAG_MODE)) << 4 |
	        IS_ENABLED(IS_RC_MODE_ACTIVE(BOXAIRMODE)) << 5 |
	        IS_ENABLED(FLIGHT_MODE(RANGEFINDER_MODE)) << 6 |
	        IS_ENABLED(FLIGHT_MODE(FAILSAFE_MODE)) << 7 |
#else
	        0x01 |  /* Dummy flag for ARMED */
#endif
	        0;
	
	sensors = 0 |
#if 0
	          sensors(SENSOR_ACC) |
	          sensors(SENSOR_BARO) << 1 |
	          sensors(SENSOR_MAG) << 2 |
	          sensors(SENSOR_GPS) << 3 |
	          sensors(SENSOR_RANGEFINDER) << 4 |
#else
	          0x09 |  /* Dummy flag for ACC and GPS */
#endif
	          0;

	BST_VPRINTLN("Write FC Mode...");
	bstMasterStartBuffer(PUBLIC_ADDRESS);
	bstMasterWrite8(CLEANFLIGHT_MODE_FRAME_ID);
	bstMasterWrite8(flags); // Status: ?
	bstMasterWrite8(sensors); // Status: ?
	
	return bstMasterWrite(masterWriteData);
}


/* ----------------------------------------------------- */


#if 0
void i2c_scan(void)
{
	uint8_t error, address;
	int nDevices;
	
	nDevices = 0;
	for(address = 0; address < 127; address++ )
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();
		
		if (error == 0)
		{
			BST_PRINT("I2C device found at address 0x");
			if (address<16)
				BST_PRINT("0");
			BST_PRINTLN(address,HEX);		
			nDevices++;
		}
		else if (error==4)
		{
			BST_PRINT("Unknown error at address 0x");
			if (address<16)
				BST_PRINT("0");
			BST_PRINTLN(address,HEX);
		}    
	}
	if (nDevices == 0)
		BST_PRINTLN("No I2C devices found\n");
	else
		BST_PRINTLN("done\n");

	delay(5000);           // wait 5 seconds for next scan
}
#endif
