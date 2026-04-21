/*
 * basiclib.h
 *
 *  Created on: Feb 19, 2026
 *      Author: E
 */

#ifndef INC_BASICLIB_H_
#define INC_BASICLIB_H_

union telempacket{
	struct {
		uint8_t ID;
		uint8_t status; // also command
		uint16_t batt_volts;
		uint32_t uptime;
	} r;
	uint8_t data[1+1+2+4];
};


#endif /* INC_BASICLIB_H_ */
