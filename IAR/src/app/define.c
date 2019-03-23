#include "include.h"

//ȫ�ֱ�������
extern unsigned char Image_USE[CAMERA_H + 1][CAMERA_DMA_NUM];
extern unsigned char Image_TMP[CAMERA_H + 1][CAMERA_DMA_NUM];
u8 img_read_flag = 0;		//ͼ��״̬
unsigned char image[CAMERA_H + 1][CAMERA_W + 1]={{0}};
unsigned char image_temp[CAMERA_H + 1][CAMERA_W + 1]={{0}};

//������
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

void two_read_image(void)//img_read_flag == 1ʱ��image_tempͼƬ;0ʱimage
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


typedef struct{
	float limit;	//����޷�
	float target;	//Ŀ����
	float feedback;	//������
	float Kp;		
	float Ki;
	float Kd;
	float eSum;
	float e0;		//��ǰ���
	float e1;		//��һ�����
}PID_position_type;

typedef struct{
        float limit;
        float target;
        float feedback;
        float Kp;
        float Ki;
        float Kd;
        float Ek;
        float Ek1;
        float Ek2;
        float last_out;
}PID_increment_type;

float PID_position(PID_position_type *p)
 {
   float pe,ie,de;
   float out=0;
   p->e0 = p->target - p->feedback;
   p->eSum += p->e0;
   de = p->e0 - p->e1;
   pe = p->e0;
   ie = p->eSum;
   p->e1 = p->e0;
   out = pe*(p->Kp) + ie*(p->Ki) + de*(p->Kd);
   out = range(out,-1*p->limit,p->limit);
   return out;
}

float PID_increment(PID_increment_type *p)
{
  float pe,ie,de;
  float out = 0;
  p->Ek = p->target - p->feedback;
  out = (p->Kp*p->Ek)-(p->Ki*p->Ek1)+(p->Kd*p->Ek2)+p->last_out;
  p->Ek2 = p->Ek1;
  p->Ek1 = p->Ek;
  p->last_out = out;
  return out;
}


//������ʼ��
int m_line[target_row + 1];
int stop_point = 100;
int point_flag = 0;
int l_line[target_row + 1];
int r_line[target_row + 1];
int width[target_row + 1];
float width_0 = 85;//55;//80;//90
float width_1=1.2;//1.1;//1.2;
void init_line(void)
{
  int i;
  
  for(i=80;i< target_row ;i++)//����ֵ             
  { 
      r_line[i]  = MAX_line-1;
      l_line[i]   = 0;
      m_line[i] = MAX_line/2;
      width[i]  = (int)(width_0+width_1*i);     //���ö�̬·��
      
  }
} 



void search_line_image(void)
{
  int i,j;
  init_line();
  stop_point = 120;
  point_flag = 0;
  {  if(i==target_row-1)
       j=MAX_line/2;
     else
       j=m_line[i+1];
     if(j<2)
       j=2;
     if(j>MAX_line-2)
       j=MAX_line-2;
     while(j>=2)
     {
       if((image[i][j]==0)&&(image[i][j-1]==1)&&(iamge[i][j-2]==1))
       {  l_line[i]=j;break;}
       j--;
     }
     if(i==target_row-1)
       j=MAX_line/2;
     else
       j=m_line[i+1];
     if(j<2)
       j=2;
     if(j>MAX_line-2)
       j=MAX_line-2;
     while(j<=MAX_line-2)
     {
       if(image[i][j]==0&&image[i][j+1]==1&&image[i][j+2]==1)
       {  r_line[i]=j;break;}
       j++;
     }
     if(l_line[i]!=0 && r_line[i]!=(MAX_line-1))//�����жϣ�û�ж���
     {
       point_flag = 1;
       m_line[i] = (l_line[i] + r_line[i])/2;
       if(i < target_row-1)
         if(r_line[i+1]- r_line[i]-l_line[i+1]+l_line[i]>8)
           if((m_line[i+1]-m_line[i]>5)||(m_line[i+1]-m_line[i]<-5))
              m_line[i] = m_line[i+1];
     } 
     if(l_line[i]==0 && r_line[i]!=(MAX_line-1))//��߶���
     {
       point_flag = 1;
       if(i == target_row-1)
         m_line[i] = r_line[i]-width[i]/2-2;
       else if((r_line[i]-l_line[i]) >=(r_line[i+1]-l_line[i+1]))//����ͻ��
         m_line[i] = m_line[i+1];
       else
         m_line[i] = r_line[i] - width[i]/2 - 2;//���
     }
     else if(l_line[i]!=0 && r_line[i]==(MAX_line-1))//�ұ߶���
     {
       point_flag = 1;
       if(i == target_row-1)
         m_line[i] =l_line[i]+width[i]/2+2;
       else if((r_line[i]-l_line[i]) >=(r_line[i+1]-l_line[i+1]))//����ͻ��
         m_line[i] = m_line[i+1];
       else
         m_line[i] = l_line[i] + width[i]/2 + 2;//���
     }
     else if(l_line[i]==0 && r_line[i]==(MAX_line-1))//ȫ��
       if(i == target_row - 1)
         m_line[i] = MAX_line/2;
       else
         m_line[i] = m_line[i+1];
     if((r_line[i]<70 || l_line[i]>249)&&i>120)
     {  stop_point = i;break;}
  }
}

void search_line_image_temp(void)
{
  int i,j;
  init_line();
  stop_point = 120;
  point_flag = 0;
  {  if(i==target_row-1)
       j=MAX_line/2;
     else
       j=m_line[i+1];
     if(j<2)
       j=2;
     if(j>MAX_line-2)
       j=MAX_line-2;
     while(j>=2)
     {
       if((image_temp[i][j]==0)&&(image_temp[i][j-1]==1)&&(iamge_temp[i][j-2]==1))
       {  l_line[i]=j;break;}
       j--;
     }
     if(i==target_row-1)
       j=MAX_line/2;
     else
       j=m_line[i+1];
     if(j<2)
       j=2;
     if(j>MAX_line-2)
       j=MAX_line-2;
     while(j<=MAX_line-2)
     {
       if(image_temp[i][j]==0&&image_temp[i][j+1]==1&&image_temp[i][j+2]==1)
       {  r_line[i]=j;break;}
       j++;
     }
     if(l_line[i]!=0 && r_line[i]!=(MAX_line-1))//�����жϣ�û�ж���
     {
       point_flag = 1;
       m_line[i] = (l_line[i] + r_line[i])/2;
       if(i < target_row-1)
         if(r_line[i+1]- r_line[i]-l_line[i+1]+l_line[i]>8)
           if((m_line[i+1]-m_line[i]>5)||(m_line[i+1]-m_line[i]<-5))
              m_line[i] = m_line[i+1];
     } 
     if(l_line[i]==0 && r_line[i]!=(MAX_line-1))//��߶���
     {
       point_flag = 1;
       if(i == target_row-1)
         m_line[i] = r_line[i]-width[i]/2-2;
       else if((r_line[i]-l_line[i]) >=(r_line[i+1]-l_line[i+1]))//����ͻ��
         m_line[i] = m_line[i+1];
       else
         m_line[i] = r_line[i] - width[i]/2 - 2;//���
     }
     else if(l_line[i]!=0 && r_line[i]==(MAX_line-1))//�ұ߶���
     {
       point_flag = 1;
       if(i == target_row-1)
         m_line[i] =l_line[i]+width[i]/2+2;
       else if((r_line[i]-l_line[i]) >=(r_line[i+1]-l_line[i+1]))//����ͻ��
         m_line[i] = m_line[i+1];
       else
         m_line[i] = l_line[i] + width[i]/2 + 2;//���
     }
     else if(l_line[i]==0 && r_line[i]==(MAX_line-1))//ȫ��
       if(i == target_row - 1)
         m_line[i] = MAX_line/2;
       else
         m_line[i] = m_line[i+1];
     if((r_line[i]<70 || l_line[i]>249)&&i>120)
     {  stop_point = i;break;}
  }
}