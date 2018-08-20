/*
 * RPMList.h
 *
 *  Created on: 12.08.2018
 *      Author: waldi
 */

#ifndef RPMLIST_H_
#define RPMLIST_H_

struct RPMList{
	uint16_t RPM;
	struct RPMList *prev, *next;
};


#endif /* RPMLIST_H_ */
