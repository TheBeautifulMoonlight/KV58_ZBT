/*
 * isr.h
 *
 *  Created on: Feb 18, 2018
 *      Author: ZBT
 */

#ifndef ISR_H_
#define ISR_H_
void Error_handler(unsigned char* Log);
void HardFault_Handler(void);
void VSYNC_IRQ(void);
#endif /* ISR_H_ */
