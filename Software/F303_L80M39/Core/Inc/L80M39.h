#include "stm32f3xx_hal.h"
#include <stdlib.h>

#ifndef INC_L80M39_H_
#define INC_L80M39_H_

#define L80M39_MESSAGE_LENGTH 6

enum AntennaStatus {
	ANT_UNKNOWN,
	OPEN,
	OK,
	SHORT
};

enum MessageType {
	GPRMC,
	GPVTG,
	GPGGA,
	GPGSA,
	GPGSV,
	GPGLL,
	GPTXT,
	MES_UNKNOWN
};

typedef enum AntennaStatus AntennaStatus_t;

struct L80M39 {
	AntennaStatus_t antennaStatus;
	float datetime;
	float latitude;
	float longitude;
	char latitudeLoc;
	char longitudeLoc;
};

typedef struct L80M39 L80M39_t;

int L80M39_init(L80M39_t* gps);

int L80M39_parse(L80M39_t* gps, uint8_t* uartMessage, int messageSize);

int L80M39_parseGPRMC(L80M39_t* gps, uint8_t* uartMessage, int start, int end);

int L80M39_parseGPVTG(L80M39_t* gps, uint8_t* uartMessage, int start, int end);

int L80M39_parseGPGGA(L80M39_t* gps, uint8_t* uartMessage, int start, int end);

int L80M39_parseGPGSA(L80M39_t* gps, uint8_t* uartMessage, int start, int end);

int L80M39_parseGPGSV(L80M39_t* gps, uint8_t* uartMessage, int start, int end);

int L80M39_parseGPGLL(L80M39_t* gps, uint8_t* uartMessage, int start, int end);

int L80M39_parseGPTXT(L80M39_t* gps, uint8_t* uartMessage, int start, int end);

enum MessageType L80M39_parseMessageType(uint8_t* uartMessage, int start);

#endif /* INC_L80M39_H_ */
