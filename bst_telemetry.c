/* TeamBlacksheepBlacksheepTelemetry Protocol
 *  Datatype definitions and helper functions
 *  Most of the datatype definitions were
 *  taken from the bst files in the COLIBRI_RACE 
 *  directory in the betaflight repo.
 */

#include <stdbool.h>
#include "bst_telemetry.h"

/* ----------------------------------------------------- */

volatile uint8_t CRC8 = 0;

/* ----------------------------------------------------- */

void bst_crc8Cal(uint8_t data_in)
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
    bstMasterWrite16((uint8_t)(data >> 16));
    bstMasterWrite16((uint8_t)(data >> 0));
}


#if 0
static int32_t lat = 0;
static int32_t lon = 0;
static uint16_t alt = 0;
static uint8_t numOfSat = 0;

bool writeGpsPositionPrameToBST(void)
{
    if ((lat != gpsSol.llh.lat) || (lon != gpsSol.llh.lon) || (alt != gpsSol.llh.alt) || (numOfSat != gpsSol.numSat)) {
        lat = gpsSol.llh.lat;
        lon = gpsSol.llh.lon;
        alt = gpsSol.llh.alt;
        numOfSat = gpsSol.numSat;
        uint16_t speed = (gpsSol.groundSpeed * 9 / 25);
        uint16_t gpsHeading = 0;
        uint16_t altitude = 0;
        gpsHeading = gpsSol.groundCourse * 10;
        altitude = alt * 10 + 1000;

        bstMasterStartBuffer(PUBLIC_ADDRESS);
        bstMasterWrite8(GPS_POSITION_FRAME_ID);
        bstMasterWrite32(lat);
        bstMasterWrite32(lon);
        bstMasterWrite16(speed);
        bstMasterWrite16(gpsHeading);
        bstMasterWrite16(altitude);
        bstMasterWrite8(numOfSat);
        bstMasterWrite8(0x00);

        return bstMasterWrite(masterWriteData);
    } else
        return false;
}

bool writeRollPitchYawToBST(void)
{
    int16_t X = -attitude.values.pitch * (M_PIf / 1800.0f) * 10000;
    int16_t Y = attitude.values.roll * (M_PIf / 1800.0f) * 10000;
    int16_t Z = 0;//radiusHeading * 10000;

    bstMasterStartBuffer(PUBLIC_ADDRESS);
    bstMasterWrite8(FC_ATTITUDE_FRAME_ID);
    bstMasterWrite16(X);
    bstMasterWrite16(Y);
    bstMasterWrite16(Z);

    return bstMasterWrite(masterWriteData);
}

bool writeRCChannelToBST(void)
{
    uint8_t i = 0;
    bstMasterStartBuffer(PUBLIC_ADDRESS);
    bstMasterWrite8(RC_CHANNEL_FRAME_ID);
    for (i = 0; i < (USABLE_TIMER_CHANNEL_COUNT-1); i++) {
        bstMasterWrite16(rcData[i]);
    }

    return bstMasterWrite(masterWriteData);
}

bool writeFCModeToBST(void)
{
    uint8_t tmp = 0;
    tmp = IS_ENABLED(ARMING_FLAG(ARMED)) |
           IS_ENABLED(FLIGHT_MODE(ANGLE_MODE)) << 1 |
           IS_ENABLED(FLIGHT_MODE(HORIZON_MODE)) << 2 |
           IS_ENABLED(FLIGHT_MODE(BARO_MODE)) << 3 |
           IS_ENABLED(FLIGHT_MODE(MAG_MODE)) << 4 |
           IS_ENABLED(IS_RC_MODE_ACTIVE(BOXAIRMODE)) << 5 |
           IS_ENABLED(FLIGHT_MODE(RANGEFINDER_MODE)) << 6 |
           IS_ENABLED(FLIGHT_MODE(FAILSAFE_MODE)) << 7;

    bstMasterStartBuffer(PUBLIC_ADDRESS);
    bstMasterWrite8(CLEANFLIGHT_MODE_FRAME_ID);
    bstMasterWrite8(tmp);
    bstMasterWrite8(sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_RANGEFINDER) << 4);

    return bstMasterWrite(masterWriteData);
}

/*************************************************************************************************/
#define UPDATE_AT_02HZ ((1000 * 1000) / 2)
static uint32_t next02hzUpdateAt_1 = 0;

#define UPDATE_AT_20HZ ((1000 * 1000) / 20)
static uint32_t next20hzUpdateAt_1 = 0;

static uint8_t sendCounter = 0;

void taskBstMasterProcess(timeUs_t currentTimeUs)
{
    if (coreProReady) {
        if (currentTimeUs >= next02hzUpdateAt_1 && !bstWriteBusy()) {
            writeFCModeToBST();
            next02hzUpdateAt_1 = currentTimeUs + UPDATE_AT_02HZ;
        }
        if (currentTimeUs >= next20hzUpdateAt_1 && !bstWriteBusy()) {
            if (sendCounter == 0)
                writeRCChannelToBST();
            else if (sendCounter == 1)
                writeRollPitchYawToBST();
            sendCounter++;
            if (sendCounter > 1)
                sendCounter = 0;
            next20hzUpdateAt_1 = currentTimeUs + UPDATE_AT_20HZ;
        }
#ifdef USE_GPS
        if (sensors(SENSOR_GPS) && !bstWriteBusy())
            writeGpsPositionPrameToBST();
#endif

    }
    bstMasterWriteLoop();
    if (isRebootScheduled) {
        stopMotors();
        systemReset();
    }
    resetBstChecker(currentTimeUs);
}


#endif



/* ----------------------------------------------------- */


