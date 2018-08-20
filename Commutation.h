/*
 * Commutation.h
 *
 *  Created on: 14.08.2018
 *      Author: waldi
 */

#ifndef COMMUTATION_H_
#define COMMUTATION_H_

struct commData
{
uint8_t LIA,LIB,LIC;	//Lowside-state
uint8_t OCRA,OCRB,OCRC; //output-compare enable
uint16_t CapA,CapB,CapC;
uint32_t stream;
uint8_t capturePolarity;
uint32_t DMAMemAdress;
struct commData *next, *prev;
};

#endif /* COMMUTATION_H_ */
