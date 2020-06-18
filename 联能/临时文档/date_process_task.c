/*
*********************************************************************************************************
*
*	模块名称 : 主程序模块
*	文件名称 : main.c
*	版    本 : V1.1
*	说    明 : 
*
*	修改记录 :
*		版本号   日期         作者        说明
*		V1.0    2018-12-12   Eric2013     1. CMSIS软包版本 V5.4.0
*                                     2. HAL库版本 V1.3.0
*
*   V1.1    2019-04-01   suozhang     1. add FreeRTOS V10.20
*
*	Copyright (C), 2018-2030, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/	
#include "bsp.h"			/* 底层硬件驱动 */
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"  
#include "bsp_spi_txrx.h"

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/

//float PT_Temp[AD7606_ADCHS];  //温度值
void UpdateTempParameter(float yy,uint8_t i);


//volatile uint32_t wp[16];
//volatile uint8_t jiaoyansum[16];
volatile uint16_t wave_packege_flag=0;
//uint8_t WaveToSend[AD7606_ADCHS][1024];
volatile uint32_t currentSAMPLEblock=0;
void UpdateWave(float yy,uint8_t i) //????,??yy??????i???
{   

}


void UpdateAccelerationData(float yyy,uint8_t i,uint32_t DataIndex)
{
	if(config.DataToBoardMode==PARAMETERMODE)  //在特征值模式下，且正在采集
		{
		 emu_data[currentSAMPLEblock][i][DataIndex]=yyy;//yyy;
		}else if(config.DataToBoardMode==WAVEMODE)
		{  
		emu_data[currentSAMPLEblock][i][DataIndex]=yyy;
//		 UpdateWave(yyy,i); 
		}	
	
}

int32_t halfref=0;
uint32_t ActualIndex;
uint32_t ActualIndex_flag = 0;
int32_t AD_ZERO[AD7606_ADCHS],AD_ZEROlowpass[AD7606_ADCHS],AD_INTER[AD7606_ADCHS],lastdata[AD7606_ADCHS],filtercounter[AD7606_ADCHS];
extern  SemaphoreHandle_t SAMPLEDATA_ready;

extern volatile uint32_t CurrentAD7606DataCounter;
extern  SemaphoreHandle_t AD7606_ready;
uint32_t sprase_counter[12],StoreDateIndex[12];
void emu_sprase_index()
{
//	uint32_t i=0;
//	for(i=0;i<AD7606_ADCHS;i++){  //某方面限制了基准采样率为51200，因为8通道必须同步采样，所以8个通道只能一个基准采样率
//			Parameter.sparse_index[i]=config.ADfrequence/config.channel_freq[i];
//		}
}
float test_buf_yy[8];

extern uint8_t spi_data_come[SLAVE_MACHINE_NUM];
extern SemaphoreHandle_t sem_spi_idle;
extern uint8_t board_all_read;
extern uint8_t board_all_ready_ad7606data;



float  Lp_25600_10000_b[5]={ 0.39869412171357016,  1.5947764868542806 ,   2.3921647302814208 , 1.5947764868542806 ,0.39869412171357016};
float 	Lp_25600_10000_a[5]={ 1.0,   2.2187395070138969 ,    2.0828312653867189 ,  0.91853546937436725, 0.15899970564213978};

float  Lp_25600_5000_b[5]={ 0.043217558470443444 , 0.17287023388177378   ,  0.25930535082266065  , 0.17287023388177378   ,0.043217558470443444 };
float 	Lp_25600_5000_a[5]={ 1.0,  -0.85564656528907435  ,   0.71831872707070521  , -0.20391645050367335 , 0.032725224249137763};

float  Lp_25600_2500_b[5]={ 0.004449635394285294, 0.017798541577141176 , 0.026697812365711761 ,0.017798541577141176 ,0.004449635394285294};
float 	Lp_25600_2500_a[5]={ 1.0,  -2.4072749353490863  ,   2.3745446399080028  , -1.0912521085249238 ,  0.19517657027457161};

float  Lp_25600_1600_b[5]={0.00093349861295484449, 0.0037339944518193779 ,  0.0056009916777290669  , 0.0037339944518193779  ,0.00093349861295484449};
float 	Lp_25600_1600_a[5]={ 1.0,   -2.9768443336967323    ,    3.4223095293776389    ,  -1.7861066002180392   , 0.35557738234440978  };

float  Lp_25600_600_b[5]={ 0.000024419565298383412,  0.000097678261193533648 ,   0.00014651739179030046  , 0.000097678261193533648 ,0.000024419565298383412};
float 	Lp_25600_600_a[5]={ 1.0,   -3.6153527020161977     ,    4.9183157191391063     ,  -2.982871345869222     ,  0.68029904179108813   };


volatile float Lp_xn_1[8]={0.0l,0.0l,0.0l,0.0l,0.0l,0.0l};
volatile float Lp_xn_2[8]={0.0l,0.0l,0.0l,0.0l,0.0l,0.0l};
volatile float Lp_xn_3[8]={0.0l,0.0l,0.0l,0.0l,0.0l,0.0l};
volatile float Lp_xn_4[8]={0.0l,0.0l,0.0l,0.0l,0.0l,0.0l};

volatile float Lp_yn_1[8]={0.0l,0.0l,0.0l,0.0l,0.0l,0.0l};
volatile float Lp_yn_2[8]={0.0l,0.0l,0.0l,0.0l,0.0l,0.0l};
volatile float Lp_yn_3[8]={0.0l,0.0l,0.0l,0.0l,0.0l,0.0l};
volatile float Lp_yn_4[8]={0.0l,0.0l,0.0l,0.0l,0.0l,0.0l};



void AD7606_TASK(void)
{
    uint32_t i,j,ii;
		int16_t *p;  //????????16???
		int32_t y;
		float intery= 0; 
	  float volatile vy=0;//速度中间变量
	//	uint32_t si=0;
	  uint32_t FreFactor=1;
		float yy=0;
		float yy_temp=0;
	  halfref=32768;
		
  	for(i=0;i<AD7606_ADCHS;i++){
		  AD_ZERO[i]=(uint64_t)32768*16384;  // tao 10s 


			AD_ZEROlowpass[i]=0u;
			AD_INTER[i]=0;

			Parameter.vs[i]=0;

			lastdata[i]=0;
//			wp[i]=0;
//			jiaoyansum[i]=0;		
			filtercounter[i]=0;

			sprase_counter[i]=0;


			StoreDateIndex[i]=0;
			Parameter.ReciprocalofRange[i]=32768/config.floatrange[i];
			Parameter.sparse_index[i]=config.ADfrequence/config.channel_freq[i];//采样率以板卡为基准
		
			//板卡采样率如何设置？  读取时又该如何？  按板卡1/2/3？ 分别可以不同？  还是统一？  还是每个通道？
		}

		while(1)
		{		

				xSemaphoreTake(AD7606_ready, portMAX_DELAY);
			
				SCB_InvalidateDCache_by_Addr ((uint32_t *)&Ad7606_Data[CurrentAD7606DataCounter], 2*AD7606SAMPLEPOINTS*AD7606_ADCHS);

				p=(int16_t *)&Ad7606_Data[CurrentAD7606DataCounter];

				for(j=0;j<AD7606SAMPLEPOINTS;j++){
					for(i=0;i<AD7606_ADCHS;i++){
						y=(int32_t)((*(int16_t*)p));//
		
						if(config.interface_type[i]==TYPE_IEPE){	
								 //直接使用上次的低通值既可以					
//							 AD_ZERO[i]=y+(((int64_t)AD_ZERO[i]*32767)>>15);
//							 y=y-(int32_t)(AD_ZERO[i]>>15); //AD_ZEROlowpass[i]
						}
						else if(config.interface_type[i]==TYPE_4_20MA)
						{
						}
					 if(config.channel_freq[i]==25600)
					 {
						 intery = y;
						 //////10K   0.73648238794566001    0.47296477589131991 /////
//						intery = Lp_25600_10000_b[0]*y + Lp_25600_10000_b[1]*Lp_xn_1[i] + Lp_25600_10000_b[2]*Lp_xn_2[i]+ Lp_25600_10000_b[3]*Lp_xn_3[i]+Lp_25600_10000_b[4]*Lp_xn_4[i]
//													- Lp_25600_10000_a[1]*Lp_yn_1[i] -Lp_25600_10000_a[2]*Lp_yn_2[i]- Lp_25600_10000_a[3]*Lp_yn_3[i]- Lp_25600_10000_a[4]*Lp_yn_4[i];
//						Lp_yn_4[i]=Lp_yn_3[i];
//						Lp_yn_3[i]=Lp_yn_2[i];
//						Lp_yn_2[i]=Lp_yn_1[i];
//						Lp_yn_1[i]=intery;
//						 
//						Lp_xn_4[i] = Lp_xn_3[i];
//						Lp_xn_3[i] = Lp_xn_2[i];
//						Lp_xn_2[i] = Lp_xn_1[i];
//						Lp_xn_1[i] = y;						 
						FreFactor = 0;
					 }else if(config.channel_freq[i]==12800)
					 {
						intery = Lp_25600_5000_b[0]*y + Lp_25600_5000_b[1]*Lp_xn_1[i] + Lp_25600_5000_b[2]*Lp_xn_2[i]+ Lp_25600_5000_b[3]*Lp_xn_3[i]+Lp_25600_5000_b[4]*Lp_xn_4[i]
													- Lp_25600_5000_a[1]*Lp_yn_1[i] -Lp_25600_5000_a[2]*Lp_yn_2[i]- Lp_25600_5000_a[3]*Lp_yn_3[i]- Lp_25600_5000_a[4]*Lp_yn_4[i];
						Lp_yn_4[i]=Lp_yn_3[i];
						Lp_yn_3[i]=Lp_yn_2[i];
						Lp_yn_2[i]=Lp_yn_1[i];
						Lp_yn_1[i]=intery;
						 
						Lp_xn_4[i] = Lp_xn_3[i];
						Lp_xn_3[i] = Lp_xn_2[i];
						Lp_xn_2[i] = Lp_xn_1[i];
						Lp_xn_1[i] = y;		
							FreFactor = 1;
					 }else if(config.channel_freq[i]==6400)
					 {
						intery = Lp_25600_2500_b[0]*y + Lp_25600_2500_b[1]*Lp_xn_1[i] + Lp_25600_2500_b[2]*Lp_xn_2[i]+ Lp_25600_2500_b[3]*Lp_xn_3[i]+Lp_25600_2500_b[4]*Lp_xn_4[i]
													- Lp_25600_2500_a[1]*Lp_yn_1[i] -Lp_25600_2500_a[2]*Lp_yn_2[i]- Lp_25600_2500_a[3]*Lp_yn_3[i]- Lp_25600_2500_a[4]*Lp_yn_4[i];
						Lp_yn_4[i]=Lp_yn_3[i];
						Lp_yn_3[i]=Lp_yn_2[i];
						Lp_yn_2[i]=Lp_yn_1[i];
						Lp_yn_1[i]=intery;
						 
						Lp_xn_4[i] = Lp_xn_3[i];
						Lp_xn_3[i] = Lp_xn_2[i];
						Lp_xn_2[i] = Lp_xn_1[i];
						Lp_xn_1[i] = y;	
						FreFactor = 2;
					 }else if(config.channel_freq[i]==3200)
					 {
						intery = Lp_25600_1600_b[0]*y + Lp_25600_1600_b[1]*Lp_xn_1[i] + Lp_25600_1600_b[2]*Lp_xn_2[i]+ Lp_25600_1600_b[3]*Lp_xn_3[i]+Lp_25600_1600_b[4]*Lp_xn_4[i]
													- Lp_25600_1600_a[1]*Lp_yn_1[i] -Lp_25600_1600_a[2]*Lp_yn_2[i]- Lp_25600_1600_a[3]*Lp_yn_3[i]- Lp_25600_1600_a[4]*Lp_yn_4[i];
						Lp_yn_4[i]=Lp_yn_3[i];
						Lp_yn_3[i]=Lp_yn_2[i];
						Lp_yn_2[i]=Lp_yn_1[i];
						Lp_yn_1[i]=intery;
						 
						Lp_xn_4[i] = Lp_xn_3[i];
						Lp_xn_3[i] = Lp_xn_2[i];
						Lp_xn_2[i] = Lp_xn_1[i];
						Lp_xn_1[i] = y;	
						FreFactor = 3;
					 }else if(config.channel_freq[i]==1600)
					 {
						intery = Lp_25600_600_b[0]*y + Lp_25600_600_b[1]*Lp_xn_1[i] + Lp_25600_600_b[2]*Lp_xn_2[i]+ Lp_25600_600_b[3]*Lp_xn_3[i]+Lp_25600_600_b[4]*Lp_xn_4[i]
													- Lp_25600_600_a[1]*Lp_yn_1[i] -Lp_25600_600_a[2]*Lp_yn_2[i]- Lp_25600_600_a[3]*Lp_yn_3[i]- Lp_25600_600_a[4]*Lp_yn_4[i];
						Lp_yn_4[i]=Lp_yn_3[i];
						Lp_yn_3[i]=Lp_yn_2[i];
						Lp_yn_2[i]=Lp_yn_1[i];
						Lp_yn_1[i]=intery;
						 
						Lp_xn_4[i] = Lp_xn_3[i];
						Lp_xn_3[i] = Lp_xn_2[i];
						Lp_xn_2[i] = Lp_xn_1[i];
						Lp_xn_1[i] = y;	
						FreFactor = 4;
					 }									 						
							 /**************************抽样部分************************/		
						sprase_counter[i]++;
						if(sprase_counter[i]>=Parameter.sparse_index[i]) 
						{
							sprase_counter[i]=0;
								//用来记录储存数据的下标
							yy=(float)intery*0.15258789f;//*config.floatscale[i]-config.floatadjust[i])*config.floatadc[i];//*config.floatscale[i];//*0.1f;//-config.floatscale[i];//-500.0f; // mv???0.038146f						
//							yy=(float)y*0.30517578f;;
							StoreDateIndex[i] = ActualIndex>>FreFactor;
//							UpdateAccelerationData(yy,ii,i,StoreDateIndex[i]);//ii代表板卡  i代表通道
							
							emu_data[currentSAMPLEblock][i][StoreDateIndex[i]]=yy;//yyy;
							
							StoreDateIndex[i]++;							
						}

						else if(config.interface_type[i]==TYPE_NONE)
						{
							halfref=y;
						}				 			
						p++;       
				}
					ActualIndex++;   //原则上应该加个中断锁的，特别是当特征值模式下，写这个下标参数为0时，后来改了一个方案				
				
						if(ActualIndex>=config.ADfrequence){ //config.ADfrequence,多采一秒数据，进行滤波分析
//							RTC_ReadClock();	/* 读时钟，结果在 g_tRTC */
							for(i=0;i<AD7606_ADCHS;i++){
								StoreDateIndex[i]=0;
								sprase_counter[i]=0; //全部归0
							}					
							 ActualIndex=0;
							 
//							if(ii == board_all_ready_ad7606data)
//								{
									currentSAMPLEblock=(currentSAMPLEblock+1)%2;
									xSemaphoreGive(SAMPLEDATA_ready);
//								}

						}
					}


	}

}
uint32_t TEMPwp[AD7606_ADCHS];
float Temp_sum[AD7606_ADCHS];
void UpdateTempParameter(float yy,uint8_t i)
{   
//		if(config.interface_type[i]==TYPE_PT){
//		if(TEMPwp[i]<config.channel_freq[i]){
//		Temp_sum[i]+=yy;
//		TEMPwp[i]++;
//		}else	  
//		{
//		 Parameter.average[i]=Temp_sum[i]/config.channel_freq[i];//???127??,?????????????0.0078125f;//(yy+(Parameter.Abs_average[i]*255/256))/256; 	
//		 Temp_sum[i]=0;
//		 TEMPwp[i]=0;
//		}
//	 }
//		else if(config.interface_type[i]==TYPE_4_20MA){
//		if(TEMPwp[i]<config.channel_freq[i]){
//		Temp_sum[i]+=yy;
//		TEMPwp[i]++;
//		}else	  
//		{
//		 Parameter.average[i]=Temp_sum[i]/config.channel_freq[i]*0.004-4;//*config.floatscale[i];//???127??,?????????????0.0078125f;//(yy+(Parameter.Abs_average[i]*255/256))/256; 	
//		 Temp_sum[i]=0;
//		 TEMPwp[i]=0;
//		}
//	 }
}


	//A＝0.0038623139728
		//R(t)＝R0(1＋At)
//		(1+At) = R(t) / R0;
//		(R(t) / R0) - 1 = At;
//		A＝0.0038623139728
//		((R(t) / R0) - 1) / A = t;

void Send_Temp(void)
{
}
