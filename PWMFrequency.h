/*
 * PWMFrequency.h
 *
 *  Created on: 12.08.2018
 *      Author: waldi
 */

#ifndef PWMFREQUENCY_H_
#define PWMFREQUENCY_H_
#define NB_PWMFREQ 5

enum PWMFreq { _10kHhz, _50kHz, _80kHz, _100kHz, _150kHz};

uint16_t ARRValue[] = {9559,1919,1199,959,639};
#endif /* PWMFREQUENCY_H_ */
