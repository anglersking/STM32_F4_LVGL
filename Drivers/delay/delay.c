#include "delay.h"
#include "stm32f4xx.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//??????OS,??????????????????ucos?????????.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//???OS??????	  
#endif


static u8  fac_us=0;							//us?????????			   
static u16 fac_ms=0;							//ms?????????,??os??,????????????ms??
	
#if SYSTEM_SUPPORT_OS							//???SYSTEM_SUPPORT_OS??????,???????OS??(??????UCOS).
//??delay_us/delay_ms??????OS??????????????OS????????????????
//??????3??????:
//    delay_osrunning:??????OS??????????????,????????????????????
//delay_ostickspersec:??????OS??????????,delay_init????????????????????systick
// delay_osintnesting:??????OS?????????,?????????????????,delay_ms????????????????????
//?????3??????:
//  delay_osschedlock:????????OS???????,???????
//delay_osschedunlock:???????OS???????,???????????
//    delay_ostimedly:????OS???,???????????????.

//?????????UCOSII??UCOSIII?????,????OS,?????????????
//???UCOSII
#ifdef 	OS_CRITICAL_METHOD						//OS_CRITICAL_METHOD??????,???????UCOSII				
#define delay_osrunning		OSRunning			//OS?????????,0,??????;1,??????
#define delay_ostickspersec	OS_TICKS_PER_SEC	//OS??????,???????????
#define delay_osintnesting 	OSIntNesting		//?????????,???????????
#endif

//???UCOSIII
#ifdef 	CPU_CFG_CRITICAL_METHOD					//CPU_CFG_CRITICAL_METHOD??????,???????UCOSIII	
#define delay_osrunning		OSRunning			//OS?????????,0,??????;1,??????
#define delay_ostickspersec	OSCfg_TickRate_Hz	//OS??????,???????????
#define delay_osintnesting 	OSIntNestingCtr		//?????????,???????????
#endif


//us??????,??????????(??????us?????)
void delay_osschedlock(void)
{
#ifdef CPU_CFG_CRITICAL_METHOD   			//???UCOSIII
	OS_ERR err; 
	OSSchedLock(&err);						//UCOSIII????,??????????????us???
#else										//????UCOSII
	OSSchedLock();							//UCOSII????,??????????????us???
#endif
}

//us??????,??????????
void delay_osschedunlock(void)
{	
#ifdef CPU_CFG_CRITICAL_METHOD   			//???UCOSIII
	OS_ERR err; 
	OSSchedUnlock(&err);					//UCOSIII????,???????
#else										//????UCOSII
	OSSchedUnlock();						//UCOSII????,???????
#endif
}

//????OS???????????????
//ticks:??????????
void delay_ostimedly(u32 ticks)
{
#ifdef CPU_CFG_CRITICAL_METHOD
	OS_ERR err; 
	OSTimeDly(ticks,OS_OPT_TIME_PERIODIC,&err);//UCOSIII?????????????
#else
	OSTimeDly(ticks);						//UCOSII???
#endif 
}
 
//systick????????,???OS????
void SysTick_Handler(void)
{	
	if(delay_osrunning==1)					//OS???????,?????????????????
	{
		OSIntEnter();						//???????
		OSTimeTick();       				//????ucos???????????               
		OSIntExit();       	 				//????????????????
	}
}
#endif

//???????????
//?????OS?????,???????????OS????????
//SYSTICK????????AHB????1/8
//SYSCLK:????????
void delay_init(u8 SYSCLK)
{
    fac_us = SYSCLK / 8;
    fac_ms = fac_us * 1000;
}

#if SYSTEM_SUPPORT_OS 						//?????????OS.
//???nus
//nus:??????us??.	
//nus:0~204522252(??????2^32/fac_us@fac_us=21)	    								   
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;				//LOAD???	    	 
	ticks=nus*fac_us; 						//?????????? 
	delay_osschedlock();					//???OS???????????us???
	told=SysTick->VAL;        				//??????????????
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//??????????SYSTICK??????????????????????.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//?????/????????????,?????.
		}  
	};
	delay_osschedunlock();					//???OS????											    
}  
//???nms
//nms:??????ms??
//nms:0~65535
void delay_ms(u16 nms)
{	
	if(delay_osrunning&&delay_osintnesting==0)//???OS?????????,????????????????(????????????????)	    
	{		 
		if(nms>=fac_ms)						//???????????OS????????????? 
		{ 
   			delay_ostimedly(nms/fac_ms);	//OS???
		}
		nms%=fac_ms;						//OS???????????????????,?????????????    
	}
	delay_us((u32)(nms*1000));				//?????????
}
#else  //????ucos?
//???nus
//nus???????us??.	
//???:nus???,???????798915us(??????2^24/fac_us@fac_us=21)
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; 				//??????	  		 
	SysTick->VAL=0x00;        				//????????
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //??????? 	 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//????????   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; //????????
	SysTick->VAL =0X00;       				//???????? 
}
//???nms
//???nms????
//SysTick->LOAD?24??????,????,???????:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK????Hz,nms????ms
//??168M??????,nms<=798ms 
void delay_xms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;			//??????(SysTick->LOAD?24bit)
	SysTick->VAL =0x00;           			//????????
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //??????? 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//????????   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //????????
	SysTick->VAL =0X00;     		  		//????????	  	    
} 
//???nms 
//nms:0~65535
void delay_ms(u16 nms)
{
    HAL_Delay(nms);
} 
#endif
			 



































