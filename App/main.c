/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       main.c
 * @brief      ɽ��K60 ƽ̨������
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"


#define eUart3 1


extern void PIT0_IRQHandler(void);
extern void PIT1_IRQHandler(void);
void PORTB_IRQHandler(void);        //PORTD�˿��жϷ�����
void PORTE_IRQHandler(void);
void uart0_handler(void);

uint32 count;
uint32 count1;
uint8 table[1024];
uint16 table3[128];
uint8 KeyVal;
uint8 PIT1_flag;
uint8 s[];
uint8 show[];


/*!
 *  @brief      main����
 *  @since      v5.0
 *  @note       FTM PWM ����
 */
void main(void)
{
	DisableInterrupts;

	
           
   

    ftm_pwm_init(FTM0, FTM_CH3,200*1000,30);        //��ʼ�� FTM PWM ��ʹ�� FTM0_CH3��Ƶ��Ϊ200k ��ռ�ձ�Ϊ 30 / FTM0_PRECISON
                                                    // vcan_port_cfg.h �� ���� FTM0_CH3 ��ӦΪ PTA6
   // ftm_pwm_init(FTM0, FTM_CH0,200*1000,80);
    //ftm_pwm_init(FTM0, FTM_CH1,200*1000,20);
    //ftm_pwm_init(FTM0, FTM_CH2,200*1000,80);
    
#if eUart3
		 uart_init(UART0,115200);  
	   uart_putstr	 (UART0 ,"\n\n\n�����жϲ��ԣ�");			//�����ַ���
	   set_vector_handler(UART0_RX_TX_VECTORn,uart0_handler);	// �����жϸ�λ�������ж���������
	   uart_rx_irq_en (UART0);								   //�����ڽ����ж�
#endif

#if 0
		  port_init(PTB3, ALT1 | IRQ_FALLING | PULLUP );		  //��ʼ�� PTD7 �ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
		set_vector_handler(PORTB_VECTORn ,PORTB_IRQHandler);	//����PORTE���жϸ�λ����Ϊ PORTE_IRQHandler
		enable_irq (PORTB_IRQn);								//ʹ��PORTE�ж�
	
		pit_init_us(PIT0, 1);
		set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);		//����PIT0���жϷ�����Ϊ PIT0_IRQHandler
	  enable_irq (PIT0_IRQn);								  //ʹ��PIT0�ж�
	  
		pit_init_ms(PIT1,1);
		set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);		//����PIT0���жϷ�����Ϊ PIT0_IRQHandler
	  enable_irq (PIT1_IRQn);								  //ʹ��PIT0�ж�
		
	

			
	//	key_init(KEY_U); //key_u ��Ӧ PTB3
		
#endif

          OLED_Init();
         
         OLED_Prints(0,0,"yhcvgcghchg");      
          gpio_init(PTB21,GPO,1);
	gpio_init(PTB1,GPO,0);
	gpio_init(PTB3,GPO,0);
	gpio_init(PTB5,GPO,1);

        OLED_Prints(0,1,"yhcvgcghchg");      
	EnableInterrupts;



    while(1)
    {
    	//sprintf((char *)show,"Temp:%dHum:%d",10,10);
	//	printf(show);
//        DELAY_MS(500);
//        ftm_pwm_duty(FTM0, FTM_CH3,30);     //����ռ�ձ� Ϊ 30 / FTM0_PRECISON
//        DELAY_MS(500);
//        ftm_pwm_duty(FTM0, FTM_CH3,60);     //����ռ�ձ�   60 / FTM0_PRECISON
    }
}
/*!
 *  @brief      PORTD�˿��жϷ�����
 *  @since      v5.0
 */
void PORTB_IRQHandler(void)
{

#if 1       // �������룬���ַ����ɹ�ѡ��

    uint8  n = 0;    //���ź�
    n = 3;
    if(PORTB_ISFR & (1 << n))           //PTD7 �����ж�
    {
        PORTB_ISFR  = (1 << n);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */
     
			
        /*  ����Ϊ�û�����  */
    }
#else
    PORT_FUNC(D,7,key_handler);
#endif
}
void PORTE_IRQHandler(void)
{
    uint8  n;    //���ź�
    uint32 flag;

    flag = PORTE_ISFR;
    PORTE_ISFR  = ~0;                                   //���жϱ�־λ

    n = 27;
    if(flag & (1 << n))                                 //PTE27�����ж�
    {
       
    }
}

void PIT0_IRQHandler(void)
{
    
    
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ
}
void PIT1_IRQHandler(void)
{
   
   
   PIT_Flag_Clear(PIT1);       //���жϱ�־λ


   
}

void uart0_handler(void)
{
    char ch;
    UARTn_e uratn = UART0;

    if(UART_S1_REG(UARTN[uratn]) & UART_S1_RDRF_MASK)   //�������ݼĴ�����
    {
        //�û���Ҫ�����������
        uart_getchar   (UART0, &ch);                    //���޵ȴ�����1���ֽ�
        uart_putchar   (UART0 , ch);                    //�����ַ���
    }
}

