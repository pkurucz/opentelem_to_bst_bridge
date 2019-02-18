/* TeamBlacksheepBlacksheepTelemetry Protocol
 *  Datatype definitions and helper functions
 *  Most of the datatype definitions were
 *  taken from the bst files in the COLIBRI_RACE 
 *  directory in the betaflight repo.
 */

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
#define I2C_ADDR_CLEANFLIGHT_FC                      0xC8
#define I2C_ADDR_CROSSFIRE_UHF_RECEIVER              0xEC

/* MSP Frame address - Yes BST is MSP over i2c */
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

/* ----------------------------------------------------- */

extern void bst_init(void);
extern void bst_handler_task(void);

/* ----------------------------------------------------- */

#endif /* _BST_TELEMETRY_H */

