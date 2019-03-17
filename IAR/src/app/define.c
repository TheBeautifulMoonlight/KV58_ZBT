#include "include.h"

//ȫ�ֱ�������
extern unsigned char Image_USE[CAMERA_H + 1][CAMERA_DMA_NUM];
extern unsigned char Image_TMP[CAMERA_H + 1][CAMERA_DMA_NUM];
u8 img_read_flag = 0;		//ͼ��״̬
unsigned char image[CAMERA_H + 1][CAMERA_W + 1]={{0}};
unsigned char image_temp[CAMERA_H + 1][CAMERA_W + 1]={{0}};

void gpio_set (PORT_Type* ptn, u8 n,u8 data)
  {
	GPIO_Type* port;
	ASSERT( (n < 32)  && (data < 2), "gpio_set find error");
	switch((u32)ptn)
  	{
  		case PORTA_BASE:
    		SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
    		port = GPIOA;
    		break;
  		case PORTB_BASE:
    		SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    		port = GPIOB;
    		break;
  		case PORTC_BASE:
    		SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
    		port = GPIOC;
    		break;
  		case PORTD_BASE:
    		SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
    		port = GPIOD;
    		break;
  		case PORTE_BASE:
    		SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
    		port = GPIOE;
    		break;
  		default:
    		break;
  	}
	if(data == 1)
	  GPIO_PDOR_REG(port) |= (1<<n);
	else
	  GPIO_PDOR_REG(port) &= ~(1<<n);
  }

void two_read_image()//img_read_flag == 1ʱ��image_tempͼƬ;0ʱimage
  {
	int i,j,k;
	img_read_flag = 0;
	for(i = 0;i < CAMERA_H ;i++)
  	{
   		for(j = 0;j < CAMERA_DMA_NUM; j++)
     			for(k = 0;k < 8; k++)
				  {  if(Image_USE[i][j]&(0x80>>k))              
          			 	image_temp[i][j*8+k]=1;	//дͼ������
        			 else
          			 	image_temp[i][j*8+k]=0;		//дͼ������  
      			  }
  	}
	img_read_flag = 1;
	for(i = 0;i < CAMERA_H;i++)
	  for(j = 0;j<AMERA_W; j++)
		image[i][j] = image_temp[i][j];
  }