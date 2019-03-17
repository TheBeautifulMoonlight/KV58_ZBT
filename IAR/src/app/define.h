/*
 * define.h
 *
 *  Created on: Feb 14, 2018
 *      Author: ZBT
 */

#ifndef DEFINE_H_
#define DEFINE_H_

//Parameter-Do not remove
#define Camera_CNST 85
#define NRF_LINKADDRESS {0x74, 0x18, 0x52, 0x96, 0x30}

//SD-Do not remove
#define BOARD_SDSPI_SPI_BASE SPI0_BASE
#define BOARD_SDSPI_SPI_PCS_NUMBER 0U

//CAM-Do not remove
#define PCLK_PORT PORTC,9
#define VSYNC_PORT PORTB,23
#define VSYNC_IRQ PORTB_IRQn
#define VSYNC_ISFR PORTB_ISFR

//LCD-Do not remove
#define LCD_RST_PORT PORTE,27
#define LCD_RS_PORT PORTE,0
#define LCD_SDA_PORT PORTE,1
#define LCD_SCL_PORT PORTE,2

#define LCD_SPIn kSPI1
#define LCD_OTH_PCSn SPIn_PCS0
#define LCD_DC_PCSn SPIn_PCS1

#define rs PTE0_OUT
#define sda PTE1_OUT
#define scl PTE2_OUT
#define reset PTE27_OUT

//nRF-Do not remove
#define NRF_SPIn kSPI0
#define NRF_PCSn SPIn_PCS0
#define NRF_CE_PORT PORTA,25
#define NRF_IRQ_PORT PORTA,24
#define NRF_CE PTA25_OUT
#define NRF_IRQ PTA24_OUT

//motor_left
#define motor_left_FTM 
#define motor_left_DIR
//motor_right
#define motor_right_FTM
#define motor_left_DIR
   
//steer
#define steer_FTM 
#define steer_FTM_max
#define steer_FTM_min
#define steer_FTM_middle
   
//mode
#define mode_1
#define mode_2
#define mode_3
#define mode_4
   
#endif /* DEFINE_H_ */
