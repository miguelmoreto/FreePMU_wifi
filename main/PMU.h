/*
 * PMU_datatypes.h
 *
 * Defines the structure of the PMU data frames.
 *
 *  Created on: 15 de fev. de 2022
 *      Author: moreto
 */

#ifndef MAIN_PMU_H_
#define MAIN_PMU_H_
#include <stdio.h>
#include <stdint.h>

/* Definitions */

#define WIFI_CONFIG 1

#if(WIFI_CONFIG == 1)
//#define USE_DHCP
#define EXAMPLE_ESP_WIFI_SSID      "TurdusRufiventris"
#define EXAMPLE_ESP_WIFI_PASS      "14608114"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5
#define WIFI_IP_FIXED				"192.168.15.81"
#define WIFI_GATEWAY_IP				"192.168.15.1"
#define WIFI_DNS_IP					"8.8.8.8"
#define WIFI_MASK					"255.255.255.0"
#elif(WIFI_CONFIG == 2)
//#define USE_DHCP
#define EXAMPLE_ESP_WIFI_SSID      "FreePMUpdc"
#define EXAMPLE_ESP_WIFI_PASS      "freepmu2022"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5
#define WIFI_IP_FIXED				"150.162.16.200"
#define WIFI_GATEWAY_IP				"150.162.16.254"
#define WIFI_DNS_IP					"8.8.8.8"
#define WIFI_MASK					"255.255.255.0"
#endif

#define IEEEC37118			2011	// Choose IEEE C37.118 std version: 2005 or 2011
#define ENABLE_HARMONICS

#define TCP_PORT 4712
#define UDP_PORT 4713

/* Frame SYNC bytes */
#define A_SYNC_AA 0xAA		// Leading byte
#if (IEEEC37118 == 2005)
#define A_SYNC_DATA 0x01	// Data frame
#define A_SYNC_HDR 0x11		// Header frame
#define A_SYNC_CFG1 0x21	// Configuration frame 1
#define A_SYNC_CFG2 0x31	// Configuration frame 2
#define A_SYNC_CFG3 0x51	// Configuration frame 3 (optional)
#define A_SYNC_CMD 0x41		// Command frame
#elif (IEEEC37118 == 2011)
#define A_SYNC_DATA 0x02	// Data frame
#define A_SYNC_HDR 0x12		// Header frame
#define A_SYNC_CFG1 0x22	// Configuration frame 1
#define A_SYNC_CFG2 0x32	// Configuration frame 2
#define A_SYNC_CFG3 0x52	// Configuration frame 3 (optional)
#define A_SYNC_CMD 0x42		// Command frame
#endif
#define NOMINAL_FREQ 60

#define TIME_BASE 0x000F4240	// 1 000 000 us (used to calculate the FracSec)
#define SEND_1_FRAME_CMD 0x0F01	// Command to send one data frame.
#define TEMP_FRAME_CMD 0x0F02	// Command to send one data frame.


#if (NOMINAL_FREQ == 50)
#define PMU_FPS 25	// Frame per second rate
#else
#define PMU_FPS 30	// Frame per second rate
#endif

#define SEND_DATA_FRAME_PERIOD_MS	1000/PMU_FPS
#define FRAC_SEC_DELTA 		TIME_BASE/PMU_FPS // This value is also used to set the High
											  //Resolution Timer period (it is in microseconds).
// Uncomment if the CRC check is present on the frames or not.
//#define USE_CRC_CHECK

/* Data types */

typedef struct _command_s {
	uint16_t sync;		// Sync byte followed by frame type and version number (AA41 hex)
	uint16_t framesize; // Number of bytes in frame
	uint16_t idcode;	// PMU/PDC ID data stream number, 16-bit integer
	uint32_t soc;		// SOC time stamp
	uint32_t fracsec;	// Fraction of Second and Time Quality
	uint16_t cmd;		// Command being sent to the PMU/PDC
#ifdef USE_CRC_CHECK
	uint16_t chk;		// CRC-CCITT
#endif
} pmu_cmd_frame_struct_t;

#if 0
typedef struct _config_s {
	uint16_t sync;			// Sync byte followed by frame type and version number (sync byte is 0xAA)
	uint16_t framesize;	// Number of bytes in frame
	uint16_t id;		// PMU/PDC ID data stream number, 16-bit integer
	uint32_t soc;		// SOC time stamp
	uint32_t fracsec;	// Fraction of Second and Time Quality
	uint32_t time_base;	// Resolution of FRACSEC time stamp
	uint16_t num_pmu;	// The number of PMUs included in the data frame
	char stn[16];		// Station Name―16 bytes in ASCII format
	uint16_t idcode;	// Data source ID number identifies source of each data block
	uint16_t format;	// Data format in data frames, 16-bit flag


} pmu_cfg_frame_struct_t;
#endif


typedef struct _cmdMessage
{
 uint16_t pmuid;
 uint16_t command;
} cmdMessage_t;

/* Function prototypes */

uint16_t PMU_config_frame_init(uint16_t pmuid, uint8_t config, uint32_t SOC, uint32_t FracSec);

uint16_t PMU_data_frame_update(uint16_t pmuid, uint8_t config, uint32_t SOC, uint32_t FracSec);

uint16_t ComputeCRC(unsigned char *Message, uint16_t MessLen);

#endif /* MAIN_PMU_H_ */
