#include "L80M39.h"

#define messageMaxLength 82;

int L80M39_init(L80M39_t* gps) {
	gps->antennaStatus = ANT_UNKNOWN;
	gps->datetime = 0;
	gps->latitude = 0;
	gps->longitude = 0;
	gps->latitudeLoc = '0';
	gps->longitudeLoc = '0';
}

int L80M39_parse(L80M39_t* gps, uint8_t* uartMessage, int messageSize) {
	uint8_t messageStartDetected = 0;
	uint8_t startIndex = -1;
	for (int index = 0; index < messageSize; index++) {
		if (uartMessage[index] == '$') {
			if (messageStartDetected == 0) {
				messageStartDetected = 1;
				startIndex = index;
			} else {
				enum MessageType messageType = L80M39_parseMessageType(uartMessage, startIndex);
				if (messageType == GPTXT) {
					L80M39_parseGPTXT(gps, uartMessage, startIndex, index - 1);
				} else if (messageType == GPGGA) {
					L80M39_parseGPGGA(gps, uartMessage, startIndex, index - 1);
				}
				startIndex = index;
			}
		}
	}
	return 0;
}

int L80M39_parseGPRMC(L80M39_t* gps, uint8_t* uartMessage, int start, int end) {
	return 0;
}

int L80M39_parseGPVTG(L80M39_t* gps, uint8_t* uartMessage, int start, int end) {
	return 0;
}

int L80M39_parseGPGGA(L80M39_t* gps, uint8_t* uartMessage, int start, int end) {
	uint8_t commasDiscovered = 0;
	int parseStartIndex = -1;
	for (int index = start; index < end; index++) {
		if (uartMessage[index] == ',') {
			commasDiscovered++;
			if (parseStartIndex != -1 && index - parseStartIndex > 1) {
				uint8_t bytes_size = index - parseStartIndex - 1;
				char bytes[bytes_size];
				memcpy(bytes, &uartMessage[parseStartIndex + 1], bytes_size * sizeof(uint8_t));
				if (commasDiscovered == 2) {
					gps->datetime = atof(&bytes);
				} else if (commasDiscovered == 3) {
					// Latitude
					gps->latitude = atof(&bytes);
				} else if (commasDiscovered == 4) {
					// Latitude - N/S
					gps->latitudeLoc = bytes[0];
				} else if (commasDiscovered == 5) {
					// Longitude
					gps->longitude = atof(&bytes);
				} else if (commasDiscovered == 6) {
					// Longitude - W/E
					gps->longitudeLoc = bytes[0];
				}
			}
			parseStartIndex = index;
		}
	}
	return 0;
}

int L80M39_parseGPGSA(L80M39_t* gps, uint8_t* uartMessage, int start, int end) {
	return 0;
}

int L80M39_parseGPGSV(L80M39_t* gps, uint8_t* uartMessage, int start, int end) {
	return 0;
}

int L80M39_parseGPGLL(L80M39_t* gps, uint8_t* uartMessage, int start, int end) {
	return 0;
}

int L80M39_parseGPTXT(L80M39_t* gps, uint8_t* uartMessage, int start, int end) {
	for (int index = 0; index < end - 3; index++) {
		uint8_t char1 = uartMessage[index];
		uint8_t char2 = uartMessage[index + 1];
		uint8_t char3 = uartMessage[index + 2];
		if (char1 == 'A' && char2 == 'N' && char3 == 'T') {
			// parse next part 10
			if (index + 11 <= end) {
				uint8_t char10 = uartMessage[index + 10];
				uint8_t char11 = uartMessage[index + 11];
				if (char10 == 'O' && char11 == 'P') {
					gps->antennaStatus = OPEN;
				} else if (char10 == 'O' && char11 == 'K') {
					gps->antennaStatus = OK;
				} else {
					gps->antennaStatus = SHORT;
				}
			}
		}
	}
	return 0;
}

enum MessageType L80M39_parseMessageType(uint8_t* uartMessage, int start) {
	enum MessageType result = MES_UNKNOWN;
	uint8_t char1 = uartMessage[start + 3];
	uint8_t char2 = uartMessage[start + 4];
	uint8_t char3 = uartMessage[start + 5];
	if (char1 == 'R' && char2 == 'M' && char3 == 'C') {
		result = GPRMC;
	} else if (char1 == 'V' && char2 == 'T' && char3 == 'G') {
		result = GPVTG;
	} else if (char1 == 'G' && char2 == 'G' && char3 == 'A') {
		result = GPGGA;
	} else if (char1 == 'G' && char2 == 'S' && char3 == 'A') {
		result = GPGSA;
	} else if (char1 == 'G' && char2 == 'S' && char3 == 'V') {
		result = GPGSV;
	} else if (char1 == 'G' && char2 == 'L' && char3 == 'L') {
		result = GPGLL;
	} else if (char1 == 'T' && char2 == 'X' && char3 == 'T') {
		result = GPTXT;
	}
	return result;
}
