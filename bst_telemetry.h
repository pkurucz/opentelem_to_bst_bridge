/* TeamBlacksheepBlacksheepTelemetry Protocol
 *  Datatype definitions and helper functions
 *  Most of the datatype definitions were
 *  taken from the bst files in the COLIBRI_RACE 
 *  directory in the betaflight repo.
 */

#include <stdint.h>

#ifndef _BST_TELEMETRY_H
#define _BST_TELEMETRY_H

/* ----------------------------------------------------- */


#define BST_BUFFER_SIZE                              128

#define BST_PROTOCOL_VERSION                         0
#define API_VERSION_MAJOR                            1 // increment when major changes are made
#define API_VERSION_MINOR                            13 // increment when any change is made, reset to zero when major changes are released after changing API_VERSION_MAJOR
#define API_VERSION_LENGTH                           2

/* Configure the CRC peripheral to use the polynomial x8 + x7 + x6 + x4 + x2 + 1 */
#define BST_CRC_POLYNOM                              0xD5

/* I2C Addresses */    
#define I2C_ADDR_TBS_CORE_PNP_PRO                    0x80
#define I2C_ADDR_RESERVED                            0x8A
#define I2C_ADDR_PNP_PRO_DIDITAL_CURRENT_SENSOR      0xC0
#define I2C_ADDR_PNP_PRO_GPS                         0xC2
#define I2C_ADDR_TSB_BLACKBOX                        0xC4
#define I2C_ADDR_CROSSFIRE_UHF_RECEIVER              0xEC
#define I2C_ADDR_CLEANFLIGHT_FC                      0xC8

/* MSP Frame address - Yes BST is MSP over i2c */
#define PUBLIC_ADDRESS                               0x00

/* Frame Types */
#define GPS_POSITION_FRAME_ID               0x02
#define GPS_TIME_FRAME_ID                   0x03
#define FC_ATTITUDE_FRAME_ID                0x1E
#define RC_CHANNEL_FRAME_ID                 0x15
#define CROSSFIRE_RSSI_FRAME_ID             0x14
#define CLEANFLIGHT_MODE_FRAME_ID           0x20
#define BATTERY_STATUS_FRAME_ID             0x08

/* ----------------------------------------------------- */


extern void bst_crc8Cal(uint8_t data_in);


/* ----------------------------------------------------- */

#endif /* _BST_TELEMETRY_H */

