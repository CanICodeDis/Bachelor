/*
 * ZeroCrossTimings.h
 *
 *  Created on: 12.08.2018
 *      Author: waldi
 */

#ifndef ZEROCROSSTIMINGS_H_
#define ZEROCROSSTIMINGS_H_

struct ZeroCrossing{
	uint16_t count;
	struct ZeroCrossing *prev, *next;
};

#endif /* ZEROCROSSTIMINGS_H_ */
