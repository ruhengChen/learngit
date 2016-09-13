/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K60 平台主程序
 * @author     山外科技
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"


#define eUart3 1


extern void PIT0_IRQHandler(void);
extern void PIT1_IRQHandler(void);
void PORTB_IRQHandler(void);        //PORTD端口中断服务函数
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
 *  @brief      main函数
 *  @since      v5.0
 *  @note       FTM PWM 测试
 */
void main(void)
{
	DisableInterrupts;

	
           
   

    ftm_pwm_init(FTM0, FTM_CH3,200*1000,30);        //初始化 FTM PWM ，使用 FTM0_CH3，频率为200k ，占空比为 30 / FTM0_PRECISON
                                                    // vcan_port_cfg.h 里 配置 FTM0_CH3 对应为 PTA6
   // ftm_pwm_init(FTM0, FTM_CH0,200*1000,80);
    //ftm_pwm_init(FTM0, FTM_CH1,200*1000,20);
    //ftm_pwm_init(FTM0, FTM_CH2,200*1000,80);
    
#if eUart3
		 uart_init(UART0,115200);  
	   uart_putstr	 (UART0 ,"\n\n\n接收中断测试：");			//发送字符串
	   set_vector_handler(UART0_RX_TX_VECTORn,uart0_handler);	// 设置中断复位函数到中断向量表里
	   uart_rx_irq_en (UART0);								   //开串口接收中断
#endif

#if 0
		  port_init(PTB3, ALT1 | IRQ_FALLING | PULLUP );		  //初始化 PTD7 管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
		set_vector_handler(PORTB_VECTORn ,PORTB_IRQHandler);	//设置PORTE的中断复位函数为 PORTE_IRQHandler
		enable_irq (PORTB_IRQn);								//使能PORTE中断
	
		pit_init_us(PIT0, 1);
		set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);		//设置PIT0的中断服务函数为 PIT0_IRQHandler
	  enable_irq (PIT0_IRQn);								  //使能PIT0中断
	  
		pit_init_ms(PIT1,1);
		set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);		//设置PIT0的中断服务函数为 PIT0_IRQHandler
	  enable_irq (PIT1_IRQn);								  //使能PIT0中断
		
	

			
	//	key_init(KEY_U); //key_u 对应 PTB3
		
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
//        ftm_pwm_duty(FTM0, FTM_CH3,30);     //设置占空比 为 30 / FTM0_PRECISON
//        DELAY_MS(500);
//        ftm_pwm_duty(FTM0, FTM_CH3,60);     //设置占空比   60 / FTM0_PRECISON
    }
}
/*!
 *  @brief      PORTD端口中断服务函数
 *  @since      v5.0
 */
void PORTB_IRQHandler(void)
{

#if 1       // 条件编译，两种方法可供选择

    uint8  n = 0;    //引脚号
    n = 3;
    if(PORTB_ISFR & (1 << n))           //PTD7 触发中断
    {
        PORTB_ISFR  = (1 << n);        //写1清中断标志位

        /*  以下为用户任务  */
     
			
        /*  以上为用户任务  */
    }
#else
    PORT_FUNC(D,7,key_handler);
#endif
}
void PORTE_IRQHandler(void)
{
    uint8  n;    //引脚号
    uint32 flag;

    flag = PORTE_ISFR;
    PORTE_ISFR  = ~0;                                   //清中断标志位

    n = 27;
    if(flag & (1 << n))                                 //PTE27触发中断
    {
       
    }
}

void PIT0_IRQHandler(void)
{
    
    
    PIT_Flag_Clear(PIT0);       //清中断标志位
}
void PIT1_IRQHandler(void)
{
   
   
   PIT_Flag_Clear(PIT1);       //清中断标志位


   
}

void uart0_handler(void)
{
    char ch;
    UARTn_e uratn = UART0;

    if(UART_S1_REG(UARTN[uratn]) & UART_S1_RDRF_MASK)   //接收数据寄存器满
    {
        //用户需要处理接收数据
        uart_getchar   (UART0, &ch);                    //无限等待接受1个字节
        uart_putchar   (UART0 , ch);                    //发送字符串
    }
}

