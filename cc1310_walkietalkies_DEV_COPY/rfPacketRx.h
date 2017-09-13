/*
 * rfPacketRx.h
 *
 *  Created on: Aug 1, 2017
 *      Author: a0227556
 */

#ifndef RFPACKETRX_H_
#define RFPACKETRX_H_


void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion,
    void *completedADCBuffer, uint32_t completedChannel);


#endif /* RFPACKETRX_H_ */
