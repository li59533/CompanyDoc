/*
*********************************************************************************************************
*
*	ģ������ : ������ģ��
*	�ļ����� : main.c
*	��    �� : V1.1
*	˵    �� : 
*
*	�޸ļ�¼ :
*		�汾��   ����         ����        ˵��
*		V1.0    2018-12-12   Eric2013     1. CMSIS����汾 V5.4.0
*                                     2. HAL��汾 V1.3.0
*
*   V1.1    2019-04-01   suozhang     1. add FreeRTOS V10.20
*
*	Copyright (C), 2018-2030, ���������� www.armfly.com
*
*********************************************************************************************************
*/	
#include "bsp.h"			/* �ײ�Ӳ������ */
#include "app.h"			/* �ײ�Ӳ������ */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"
#include "arm_math.h"
#define SPEED_FLAG_HIGH 1500//2500
#define SPEED_FLAG_LOW 300//900
extern volatile uint8_t currentblock ;
float I_4_20MA[8];

extern  SemaphoreHandle_t SAMPLEDATA_ready;

float PT_Voltage[AD7606_ADCHS];
extern float PT_Temp[AD7606_ADCHS];  //�¶�ֵ
arm_rfft_fast_instance_f32 S;
uint32_t fftSize,ifftFlag;
void Init_FFT(void)
{
 
 fftSize = config.ADfrequence; //????

 ifftFlag = 0;//???

// arm_rfft_fast_init_f32(&S, fftSize);
}



void updateParameter(uint8_t i)
{

	if(config.interface_type[i]==TYPE_IEPE)
	{
		Parameter.Mean[i]=Parameter.S_sum[i]*Parameter.ReciprocalofEMUnumber[i];//??
		float tt=Parameter.SS_sum[i]*Parameter.ReciprocalofEMUnumber[i];  //
		if(Parameter.SS_sum[i]==0) Parameter.SS_sum[i]=0.01f;
		if(tt<0) tt=-tt;
		arm_sqrt_f32(tt,(float32_t *)Parameter.EffectiveValue+i);
#ifdef USE_FLITER
		if(Parameter.EffectiveValue[i]<0.25f)
		{
		Parameter.PeakValue[i]=0;
		Parameter.Inter_Skew[i]=0;
		Parameter.Kurtosis[i]=0; 		
		Parameter.Abs_average[i]=0;
		Parameter.MaxValue[i]=0;			
		Parameter.InterIIMarginIndex[i]=0;
		Parameter.WaveformIndex[i]=0; //????
		Parameter.PeakIndex[i]=0;//????
		Parameter.PulseIndex[i]=0;//????
		Parameter.Skew[i]=0;//????	 
		Parameter.KurtosisIndex[i]=0;//????
		Parameter.MarginIndex[i]=0;
					
		}
		else
#endif
		{		
		//Parameter.ReciprocalofEMUnumber ?????? Parameter.ReciprocalofEMUnumber=1/config.ADfrequence; 
		Parameter.PeakValue[i]=Parameter.InterMAX[i]-Parameter.InterMIN[i];
		Parameter.Inter_Skew[i]=Parameter.SSS_sum[i]*Parameter.ReciprocalofEMUnumber[i];
		Parameter.Kurtosis[i]=Parameter.SSSS_sum[i]*Parameter.ReciprocalofEMUnumber[i]; //??
		
		Parameter.Abs_average[i]=Parameter.Abs_S_average[i]*Parameter.ReciprocalofEMUnumber[i];//?????
		
		Parameter.InterIIMarginIndex[i]=(Parameter.Inter_MarginIndex[i]*Parameter.ReciprocalofEMUnumber[i])*(Parameter.Inter_MarginIndex[i]*Parameter.ReciprocalofEMUnumber[i]);
		if(Parameter.InterMAX[i]<(-Parameter.InterMIN[i])) Parameter.MaxValue[i]=-Parameter.InterMIN[i];
		else Parameter.MaxValue[i]=Parameter.InterMAX[i]; //????
		if(Parameter.Abs_average[i]==0) Parameter.Abs_average[i]=0.01f;
		Parameter.WaveformIndex[i]=Parameter.EffectiveValue[i]/Parameter.Abs_average[i]; //????
		Parameter.PeakIndex[i]=Parameter.MaxValue[i]/Parameter.EffectiveValue[i];//????
		Parameter.PulseIndex[i]=Parameter.MaxValue[i]/Parameter.Abs_average[i];//????
		Parameter.Skew[i]=Parameter.Inter_Skew[i]/(Parameter.EffectiveValue[i]*Parameter.EffectiveValue[i]*Parameter.EffectiveValue[i]);//????	 
		Parameter.KurtosisIndex[i]=Parameter.Kurtosis[i]/(Parameter.EffectiveValue[i]*Parameter.EffectiveValue[i]*Parameter.EffectiveValue[i]*Parameter.EffectiveValue[i]);//????
		Parameter.MarginIndex[i]=Parameter.MaxValue[i]/Parameter.InterIIMarginIndex[i];
		}
	}else 
		{  //
		 Parameter.Abs_average[i]=Parameter.S_sum[i]*Parameter.ReciprocalofEMUnumber[i];
		}
}



void InitIntermediateVariable(uint8_t i) // ???????,??????? ???????
{ 
	Parameter.S_average[i]=0;
	Parameter.alarm[i]=0;
	Parameter.backlash[i]=0;
	Parameter.gate[i]=0;
	Parameter.s[i]=0;
	Parameter.as[i]=0;
	Parameter.InterMAX[i]=0;
//	Parameter.MaxValue[i]=0;
	Parameter.InterMIN[i]=0;
	Parameter.Inter_MarginIndex[i]=0;
	Parameter.Abs_S_average[i]=0;
	//Parameter.Abs_average[i]=0;
	Parameter.S_sum[i]=0;
	Parameter.SS_sum[i]=0;
	Parameter.SSS_sum[i]=0;
	Parameter.SSSS_sum[i]=0;
	
}
float (*p)(double,uint32_t);

double (*p_highpass)(double,uint32_t);
extern uint32_t SAMPLEblock;

#define numStages 2 /* 2��IIR�˲��ĸ��� */
float32_t testOutput[16384] __attribute__ ((section ("RW_IRAM2"),  zero_init));//__attribute__((at(0x24000000)));//����axiram /* �˲������� */

float32_t IIRStateF32[4*numStages]; /* ״̬���棬��СnumTaps + blockSize - 1*/
/* ������˹��ͨ�˲���ϵ�� 140Hz */
const float32_t IIRCoeffs16384_10HP[5*numStages] = {
1 , -2 , 1 , 1.9970544779472288  , -0.99706916323414285  ,    
1 , -2 , 1 , 1.99292431260138    , -0.99293896751723321        
};
const float sacle16384_10HP=0.99853091029534291f*0.99646582002965323f;


const float32_t IIRCoeffs8192_10HP[5*numStages] = {
1 , -2 , 1 , 1.9940882916386464  , -0.99414694661750846     ,    
1 , -2 , 1 , 1.9858693073382856  , -0.98592772056037459    
};
const float sacle8192_10HP=0.99705880956403869f*0.99294925697466507f;


const float32_t IIRCoeffs4096_10HP[5*numStages] = {
1 , -2 , 1 , 1.9880944609167057  , -0.98832839281398177   ,    
1 , -2 , 1 , 1.9718208800618793  , -0.97205289710565135    
};
const float sacle4096_10HP = 0.99410571343267184f*0.98596844429188268f;

arm_biquad_casd_df1_inst_f32 S_test;

volatile float hp_yy[8],SpeedInter[8];
extern volatile uint32_t currentSAMPLEblock;


const int32_t df_temp_table[]=		{-200, 	-100,		00,		100,		200,	 300,	400,	500,	600,	700,800,900,1000,1100};
const int32_t df_resister_table[]=		{921,			960,		1000,	1039,	1078,	1116,	1155,	1194,	1232,1271,1309,1347,1385,1423}; 
uint32_t df_table_num=sizeof(df_temp_table)/sizeof(int32_t);

float zhuansu_junzhi = 0.0;


uint8_t UP_OR_LOW = 0;//ת�ٸߵ͵�ƽ
uint32_t SPEED_COUNT[8][SPEED_FLAG_COUNT];//5S��ת�ټ���
uint32_t SPEED_ROTATE_COUNT[8];
uint32_t SPEED_ROTATE_COUNT_FLAG[8];

float V_DATA[8];

void EmuData(void)
{ 
	int32_t t;
	float df;
	float absyy=0;
  float INTERsqrt=0;
	float yy,vy;
	uint32_t cc = 0;
	uint32_t MIDDLE_SPEED_SUM=0;
	uint32_t SAMPLEblock=(currentSAMPLEblock+1)%2;;
//	SPEED_COUNT = 0;
	zhuansu_junzhi = 0;	
	float ScaleValue=1;

	{
		
		for(uint32_t j=0;j<AD7606_ADCHS;j++)
		{  
			if(config.interface_type[j]==TYPE_IEPE)
			{
				Parameter.ReciprocalofEMUnumber[j]=1.0f/config.channel_freq[j]*config.ADtime;  //����Ƶ�ʵĵ���
				Parameter.ReciprocalofADfrequence[j]=1.0f/config.channel_freq[j]; 
				if(((config.DataToSendChannel>>j)&0x01)==0)  //
				{
					continue;
				}
				Parameter.vs[j]=0;
				hp_yy[j]=0;
				/* ��ʼ�� */
				switch(config.channel_freq[j])
				{
				case 16384:
	//				if(j==3)
					{
					arm_biquad_cascade_df1_init_f32(&S_test, numStages, (float32_t *)&IIRCoeffs16384_10HP[0], (float32_t
				*)&IIRStateF32[0]);
				/* IIR�˲� */
				arm_biquad_cascade_df1_f32(&S_test,(float32_t *)&emu_data[SAMPLEblock][j][0], testOutput, config.channel_freq[j]);
	//			arm_biquad_cascade_df1_f32(&S_test,(float32_t *)testOutput1, testOutput, config.ADfrequence*2);
				
				/*����ϵ�� */
				ScaleValue = sacle16384_10HP;
					}
					break;
				case 8192:
					arm_biquad_cascade_df1_init_f32(&S_test, numStages, (float32_t *)&IIRCoeffs8192_10HP[0], (float32_t
				*)&IIRStateF32[0]);
				/* IIR�˲� */
				arm_biquad_cascade_df1_f32(&S_test,(float32_t *)&emu_data[SAMPLEblock][j][0], testOutput,  config.channel_freq[j]);
				/*����ϵ�� */
				 ScaleValue = sacle8192_10HP;
					break;
				case 4096:
					arm_biquad_cascade_df1_init_f32(&S_test, numStages, (float32_t *)&IIRCoeffs4096_10HP[0], (float32_t
				*)&IIRStateF32[0]);
				/* IIR�˲� */
				arm_biquad_cascade_df1_f32(&S_test,(float32_t *)&emu_data[SAMPLEblock][j][0], testOutput, config.channel_freq[j]);
	//			arm_biquad_cascade_df1_f32(&S_test,(float32_t *)testOutput1, testOutput, config.ADfrequence*2);
				
				/*����ϵ�� */
				ScaleValue = sacle4096_10HP;
					break;
				default:
	//			arm_biquad_cascade_df1_init_f32(&S_test, numStages, (float32_t *)&IIRCoeffs12800_10HP[0], (float32_t
	//			*)&IIRStateF32[0]);
	//			/* IIR�˲� */
	//			arm_biquad_cascade_df1_f32(&S_test,(float32_t *)&emu_data[SAMPLEblock][j][0], testOutput,  config.channel_freq[j]);
	//			/*����ϵ�� */
	//			 ScaleValue =sacle12800_10HP;
				break;
				}
				
				for(uint32_t i=0;i<config.channel_freq[j];i++)	 
				{
					hp_yy[j]+=testOutput[i]*ScaleValue*1000*Parameter.ReciprocalofADfrequence[j];
	//				hp_yy[j]=testOutput[i];
					hp_yy[j]=hp_yy[j]*0.9999f;//й��ֱ������
					if(i>=0.5f*config.channel_freq[j])
					{	
	//				ReceiveSamplesPeriod[SAMPLEblock][j][i]=testOutput[i];					
	//				hp_yy[j]=testOutput[i]*ScaleValue;//1000*Parameter.ReciprocalofADfrequence;
	//				ReceiveSamplesPeriod[SAMPLEblock][j][i]=hp_yy[j];
						Parameter.vs[j]+=hp_yy[j]*hp_yy[j];	
					}
				}
					
				float tt=Parameter.vs[j]*Parameter.ReciprocalofEMUnumber[j]*2;//1000000*Parameter.ReciprocalofEMUnumber;
				arm_sqrt_f32(tt,(float32_t *)Parameter.fv+j);
				Parameter.vs[j]=0; //?????0		
			}
			else if(config.interface_type[j]==TYPE_PT)//�¶ȼ���
			{
			PT_Temp[j] = Parameter.average[j]*config.floatscale[j] - config.floatadjust[j];
				
			PT_Temp[j] = (PT_Temp[j]/1000.0 - 1)*258.9121462f;
				if((PT_Temp[j] > 160.0) || (PT_Temp[j] < -30.0))
				{
					PT_Temp[j] = 188.0f;
				}

			}
			else if(config.interface_type[j]==TYPE_PT100)//PT100�¶ȼ��� 
			{
			PT_Temp[j] = Parameter.average[j]*config.floatscale[j] + config.floatadjust[j];
				
			PT_Temp[j] = (PT_Temp[j]-100.2)/0.385f;
				if((PT_Temp[j] > 210.0) || (PT_Temp[j] < -50.0))
				{
					PT_Temp[j] = 1000.0f;
				}

			}
			else if(config.interface_type[j]==TYPE_4_20MA)//�¶ȼ���
			{
//			Parameter.I_4_20MA[j] 
				I_4_20MA[j]= (Parameter.average[j]*config.floatadc[j]- config.floatadjust[j])*config.floatscale[j];
				
//					I_4_20MA[j]= (Parameter.average[j]*config.floatadc[j]- config.floatadjust[j])/config.floatscale[j]-(4.0f/config.floatscale[j]+20);
				
				if((I_4_20MA[j] > 180.0)|| (I_4_20MA[j] < -30.0))
				{
					I_4_20MA[j] = 199.0f;
				}
//				if(I_4_20MA[j] > 20.0)// || (I_4_20MA[j] < 0.0))
//				{
//					I_4_20MA[j] = 199.0f;
//				}

			}
			else if(config.interface_type[j]==TYPE_SPEED)//ת�ټ���
			{			
				if(((config.DataToSendChannel>>j)&0x01)==0)  //?????????
				{
					continue;
				}

//				switch(j)
//				{
//					case 0:
//						break;
//					case 1:
//						break;
//					case 2:
//						break;
//					case 3:
//						break;
//					case 4:
//						break;
//					case 5:
//						break;
//					case 6:
//						break;
//					case 7:
//						break;
//				}
				
				if((emu_data[SAMPLEblock][j][0] > SPEED_FLAG_HIGH) )
				{	
					UP_OR_LOW = 1;
				}else if((emu_data[SAMPLEblock][j][0] < SPEED_FLAG_LOW))
				{
					UP_OR_LOW = 0;
				}
//				if(zhuansu_junzhi>1000)
//				{
//					for(uint32_t k = config.ADfrequence;k<config.ADfrequence*(config.ADtime+1);k++)										
					for(uint32_t k = 0;k<config.channel_freq[j];k++)
					{

						switch(UP_OR_LOW)
						{
							case 0:
									if(emu_data[SAMPLEblock][j][k] > SPEED_FLAG_HIGH)
									{
										SPEED_COUNT[j][SPEED_ROTATE_COUNT[j]]++;
										UP_OR_LOW=1;
									}					
								break;
							case 1:
									if(emu_data[SAMPLEblock][j][k] < SPEED_FLAG_LOW)
									{
										SPEED_COUNT[j][SPEED_ROTATE_COUNT[j]]++;
										UP_OR_LOW=0;
									}	
								break;
							default:
								break;			
						}							
					}

					SPEED_ROTATE_COUNT[j]++;//�����±�,�ڼ���
					SPEED_ROTATE_COUNT_FLAG[j]++;//�����˼��Σ�����һ�������ż���
					if(SPEED_ROTATE_COUNT_FLAG[j] > SPEED_FLAG_COUNT_LESS1)//�޸ĳ�5���ʼ����ת��
					{
						if(SPEED_ROTATE_COUNT[j]>SPEED_FLAG_COUNT_LESS1)
						{												
							SPEED_ROTATE_COUNT[j] = 0;
							SPEED_COUNT[j][SPEED_ROTATE_COUNT[j]] = 0;
						}else 
						{
							SPEED_COUNT[j][SPEED_ROTATE_COUNT[j]] = 0;
						}
						for(cc = 0; cc < SPEED_FLAG_COUNT; cc++)
						{
							MIDDLE_SPEED_SUM += SPEED_COUNT[j][cc];
						}

						Parameter.SPEED_ROTATE[j] =  MIDDLE_SPEED_SUM/8*config.floatscale[j];// MIDDLE_SPEED_SUM/SPEED_FLAG_COUNT_LESS1*config.floatscale[j];	

						SPEED_ROTATE_COUNT_FLAG[j]=50;
						
						MIDDLE_SPEED_SUM=0;


					}
			
				
			}
			else if(config.interface_type[j]==TYPE_V)//�¶ȼ���
			{
				V_DATA[j]= Parameter.average[j]*config.floatscale[j];

			}			
			
			
		}
		for(uint32_t j=0;j<AD7606_ADCHS;j++)
		{  
			if(config.interface_type[j]==TYPE_IEPE)
			{
				if(((config.DataToSendChannel>>j)&0x01)==0)  //?????????
				{
					continue;
				}
				 InitIntermediateVariable(j); //???????
				 for(uint32_t i=0;i<config.channel_freq[j];i++)	 
				{
				 yy=emu_data[SAMPLEblock][j][i];
				 Parameter.S_average[j]+=yy;
				}
			}			
		}
		for(uint32_t j=0;j<AD7606_ADCHS;j++)
		{
			if(config.interface_type[j]==TYPE_IEPE)
			{
				Parameter.average[j]=Parameter.S_average[j]*Parameter.ReciprocalofEMUnumber[j];//?????
				if(0)
				{
					NeedRestartCollect();
				}
			}

		}
		for(uint32_t j=0;j<AD7606_ADCHS;j++)	 {
			if(config.interface_type[j]==TYPE_IEPE)
			{
				if(((config.DataToSendChannel>>j)&0x01)==0)  //?????????
				{
					continue;
				}
				for(uint32_t i=0;i<config.channel_freq[j];i++)
				{
					yy=emu_data[SAMPLEblock][j][i]-Parameter.average[j];
					
					if(Parameter.InterMAX[j]<yy) Parameter.InterMAX[j]=yy;
					if(Parameter.InterMIN[j]>yy) Parameter.InterMIN[j]=yy;
					if(yy<0) absyy=-yy;
					 else absyy=yy;
					Parameter.Abs_S_average[j]+=absyy;
					arm_sqrt_f32(absyy,&INTERsqrt);
					Parameter.Inter_MarginIndex[j]+=INTERsqrt;
					Parameter.S_sum[j]+=yy;
					Parameter.SS_sum[j]+=yy*yy;
					Parameter.SSS_sum[j]+=(absyy*absyy*absyy);
					Parameter.SSSS_sum[j]+=(yy*yy*yy*yy);  	 
				}
			}
		}
		for(uint32_t j=0;j<AD7606_ADCHS;j++)
		{
			updateParameter(j);
//			if(Parameter.EffectiveValue[j]>25)
//			{
//				NeedRestartCollect();
//			}
			InitIntermediateVariable(j); 
		}
	}

}




uint8_t BoardParameter_withtime(void)  //��������ֵ
{
	uint8_t sendbuf[500];
	uint8_t * floatdata;
	uint32_t empty_Acceleration_ADCHS=0;   //δ����ͨ��
	sendbuf[0]=0x7e;
	sendbuf[1]=0x42;
	uint32_t iii=0;
	
	sendbuf[4]=g_tRTC.Year;
	sendbuf[5]=g_tRTC.Year>>8;
	sendbuf[6]=g_tRTC.Mon;
	sendbuf[7]=g_tRTC.Day; //ʱ����32λ��ʾ
	sendbuf[8]=g_tRTC.Hour;
	sendbuf[9]=g_tRTC.Min;
	sendbuf[10]=g_tRTC.Sec; //ʱ����32λ��ʾ
		
	for(uint32_t ii=0;ii<AD7606_ADCHS;ii++)
	{
		if(((config.DataToSendChannel>>ii)&0x01)==0)  //δʹ�ܵ�ͨ��������
		{
		 empty_Acceleration_ADCHS++;
		 continue;
		}

		sendbuf[11+50*iii]=ii+1;
		sendbuf[12+50*iii]=0x0c;
		
		if(config.interface_type[ii] == TYPE_IEPE)
		{
			 floatdata=(uint8_t *)&Parameter.PeakValue[ii];//[0];
		}
		else	if(config.interface_type[ii] == TYPE_PT)
		{
			 floatdata=(uint8_t *)&PT_Temp[ii];//[0];
		}
		else	if(config.interface_type[ii] == TYPE_PT100)
		{
			 floatdata=(uint8_t *)&PT_Temp[ii];//[0];
		}
		else	if(config.interface_type[ii] == TYPE_SPEED)
		{
			 floatdata=(uint8_t *)&	Parameter.SPEED_ROTATE[ii];//[0];
		}
		else	if(config.interface_type[ii] == TYPE_4_20MA)
		{
			 floatdata=(uint8_t *)&I_4_20MA[ii];
		}
		else	if(config.interface_type[ii] == TYPE_V)
		{
			 floatdata=(uint8_t *)&V_DATA[ii];
		}

		sendbuf[13+50*iii]=*floatdata;
		sendbuf[14+50*iii]=*(floatdata+1);
		sendbuf[15+50*iii]=*(floatdata+2);
		sendbuf[16+50*iii]=*(floatdata+3);
		floatdata=(uint8_t *)&Parameter.MaxValue[ii];
		sendbuf[17+50*iii]=*floatdata;
		sendbuf[18+50*iii]=*(floatdata+1);
		sendbuf[19+50*iii]=*(floatdata+2);
		sendbuf[20+50*iii]=*(floatdata+3);
		floatdata=(uint8_t *)&Parameter.Abs_average[ii];
		sendbuf[21+50*iii]=*floatdata;
		sendbuf[22+50*iii]=*(floatdata+1);
		sendbuf[23+50*iii]=*(floatdata+2);
		sendbuf[24+50*iii]=*(floatdata+3);
		floatdata=(uint8_t *)&Parameter.EffectiveValue[ii];
		sendbuf[25+50*iii]=*floatdata;
		sendbuf[26+50*iii]=*(floatdata+1);
		sendbuf[27+50*iii]=*(floatdata+2);
		sendbuf[28+50*iii]=*(floatdata+3);
		floatdata=(uint8_t *)&Parameter.Kurtosis[ii];
		sendbuf[29+50*iii]=*floatdata;
		sendbuf[30+50*iii]=*(floatdata+1);
		sendbuf[31+50*iii]=*(floatdata+2);
		sendbuf[32+50*iii]=*(floatdata+3);
		floatdata=(uint8_t *)&Parameter.WaveformIndex[ii];
		sendbuf[33+50*iii]=*floatdata;
		sendbuf[34+50*iii]=*(floatdata+1);
		sendbuf[35+50*iii]=*(floatdata+2);
		sendbuf[36+50*iii]=*(floatdata+3);
		floatdata=(uint8_t *)&Parameter.PeakIndex[ii];
		sendbuf[37+50*iii]=*floatdata;
		sendbuf[38+50*iii]=*(floatdata+1);
		sendbuf[39+50*iii]=*(floatdata+2);
		sendbuf[40+50*iii]=*(floatdata+3);
		floatdata=(uint8_t *)&Parameter.PulseIndex[ii];
		sendbuf[41+50*iii]=*floatdata;
		sendbuf[42+50*iii]=*(floatdata+1);
		sendbuf[43+50*iii]=*(floatdata+2);
		sendbuf[44+50*iii]=*(floatdata+3);
		floatdata=(uint8_t *)&Parameter.MarginIndex[ii];
		sendbuf[45+50*iii]=*floatdata;
		sendbuf[46+50*iii]=*(floatdata+1);
		sendbuf[47+50*iii]=*(floatdata+2);
		sendbuf[48+50*iii]=*(floatdata+3);
		floatdata=(uint8_t *)&Parameter.KurtosisIndex[ii];
		sendbuf[49+50*iii]=*floatdata;
		sendbuf[50+50*iii]=*(floatdata+1);
		sendbuf[51+50*iii]=*(floatdata+2);
		sendbuf[52+50*iii]=*(floatdata+3);
		floatdata=(uint8_t *)&Parameter.Skew[ii];
		sendbuf[53+50*iii]=*floatdata;
		sendbuf[54+50*iii]=*(floatdata+1);
		sendbuf[55+50*iii]=*(floatdata+2);
		sendbuf[56+50*iii]=*(floatdata+3);
		floatdata=(uint8_t *)&Parameter.fv[ii];
		sendbuf[57+50*iii]=*floatdata;
		sendbuf[58+50*iii]=*(floatdata+1);
		sendbuf[59+50*iii]=*(floatdata+2);
		sendbuf[60+50*iii]=*(floatdata+3);
		iii++;
	} 
	iii--;
  uint32_t length=50*(AD7606_ADCHS-empty_Acceleration_ADCHS)+7;
	
  sendbuf[2]=(uint8_t)length;
	sendbuf[3]=(uint8_t)(length>>8);
	sendbuf[61+50*iii]=0;
	for(uint8_t i=1;i<(61+50*iii);i++)
	sendbuf[61+50*iii]+=sendbuf[i];
	sendbuf[62+50*iii]=0x7e;

	WriteDataToTXDBUF(sendbuf,(63+50*iii));//��
	//���ͼ��ٶȸ��ٶȣ��������������淢���ٶ�
	
  //�¶�
	
	//ת��

	return 1;
}
//uint8_t BoardParameter_withtime_Once_Channel(uint8_t channel_num)  //���͵�ͨ������ֵ
//{
//	uint8_t sendbuf[65];
//	uint32_t empty_Acceleration_ADCHS=0;   //δ����ͨ��
//	sendbuf[0]=0x7e;
//	sendbuf[1]=0x42;
//	uint32_t iii=0;
//	
//	sendbuf[4]=g_tRTC.Year;
//	sendbuf[5]=g_tRTC.Year>>8;
//	sendbuf[6]=g_tRTC.Mon;
//	sendbuf[7]=g_tRTC.Day; //ʱ����32λ��ʾ
//	sendbuf[8]=g_tRTC.Hour;
//	sendbuf[9]=g_tRTC.Min;
//	sendbuf[10]=g_tRTC.Sec; //ʱ����32λ��ʾ
//		

//		sendbuf[11]=channel_num+1;
//		sendbuf[12]=0x0c;
//		

//		uint8_t * floatdata=(uint8_t *)&Parameter.PeakValue[channel_num];//[0];
//		sendbuf[13]=*floatdata;
//		sendbuf[14]=*(floatdata+1);
//		sendbuf[15]=*(floatdata+2);
//		sendbuf[16]=*(floatdata+3);
//		floatdata=(uint8_t *)&Parameter.MaxValue[channel_num];
//		sendbuf[17]=*floatdata;
//		sendbuf[18]=*(floatdata+1);
//		sendbuf[19]=*(floatdata+2);
//		sendbuf[20]=*(floatdata+3);
//		floatdata=(uint8_t *)&Parameter.Abs_average[channel_num];
//		sendbuf[21]=*floatdata;
//		sendbuf[22]=*(floatdata+1);
//		sendbuf[23]=*(floatdata+2);
//		sendbuf[24]=*(floatdata+3);
//		floatdata=(uint8_t *)&Parameter.EffectiveValue[channel_num];
//		sendbuf[25]=*floatdata;
//		sendbuf[26]=*(floatdata+1);
//		sendbuf[27]=*(floatdata+2);
//		sendbuf[28]=*(floatdata+3);
//		floatdata=(uint8_t *)&Parameter.Kurtosis[channel_num];
//		sendbuf[29]=*floatdata;
//		sendbuf[30]=*(floatdata+1);
//		sendbuf[31]=*(floatdata+2);
//		sendbuf[32]=*(floatdata+3);
//		floatdata=(uint8_t *)&Parameter.WaveformIndex[channel_num];
//		sendbuf[33]=*floatdata;
//		sendbuf[34]=*(floatdata+1);
//		sendbuf[35]=*(floatdata+2);
//		sendbuf[36]=*(floatdata+3);
//		floatdata=(uint8_t *)&Parameter.PeakIndex[channel_num];
//		sendbuf[37]=*floatdata;
//		sendbuf[38]=*(floatdata+1);
//		sendbuf[39]=*(floatdata+2);
//		sendbuf[40]=*(floatdata+3);
//		floatdata=(uint8_t *)&Parameter.PulseIndex[channel_num];
//		sendbuf[41]=*floatdata;
//		sendbuf[42]=*(floatdata+1);
//		sendbuf[43]=*(floatdata+2);
//		sendbuf[44]=*(floatdata+3);
//		floatdata=(uint8_t *)&Parameter.MarginIndex[channel_num];
//		sendbuf[45]=*floatdata;
//		sendbuf[46]=*(floatdata+1);
//		sendbuf[47]=*(floatdata+2);
//		sendbuf[48]=*(floatdata+3);
//		floatdata=(uint8_t *)&Parameter.KurtosisIndex[channel_num];
//		sendbuf[49]=*floatdata;
//		sendbuf[50]=*(floatdata+1);
//		sendbuf[51]=*(floatdata+2);
//		sendbuf[52]=*(floatdata+3);
//		floatdata=(uint8_t *)&Parameter.Skew[channel_num];
//		sendbuf[53]=*floatdata;
//		sendbuf[54]=*(floatdata+1);
//		sendbuf[55]=*(floatdata+2);
//		sendbuf[56]=*(floatdata+3);
//		floatdata=(uint8_t *)&Parameter.fv[channel_num];
//		sendbuf[57]=*floatdata;
//		sendbuf[58]=*(floatdata+1);
//		sendbuf[59]=*(floatdata+2);
//		sendbuf[60]=*(floatdata+3);


//  uint32_t length=57;
//	
//  sendbuf[2]=(uint8_t)length;
//	sendbuf[3]=(uint8_t)(length>>8);
//	sendbuf[61]=0;
//	for(uint8_t i=1;i<61;i++)
//	sendbuf[61]+=sendbuf[i];
//	sendbuf[62]=0x7e;

//	WriteDataToTXDBUF(sendbuf,(3));//��
//	//���ͼ��ٶȸ��ٶȣ��������������淢���ٶ�
//	
//  //�¶�
//	
//	//ת��

//	return 1;
//}
//uint8_t BoardTemp_withtime(uint8_t channel_num)  //�����¶�
//{
//	uint8_t sendbuf[20];
//	sendbuf[0]=0x7e;
//	sendbuf[1]=0x43;
//	uint32_t iii=0;
//	
//	sendbuf[4]=g_tRTC.Year;
//	sendbuf[5]=g_tRTC.Year>>8;
//	sendbuf[6]=g_tRTC.Mon;
//	sendbuf[7]=g_tRTC.Day; //ʱ����32λ��ʾ
//	sendbuf[8]=g_tRTC.Hour;
//	sendbuf[9]=g_tRTC.Min;
//	sendbuf[10]=g_tRTC.Sec; //ʱ����32λ��ʾ

//	sendbuf[11]=channel_num+1;//ͨ����
////�¶�ֵ
//	uint8_t * floatdata=(uint8_t *)&PT_Temp[channel_num];//[0];
//	sendbuf[12]=*floatdata;
//	sendbuf[13]=*(floatdata+1);
//	sendbuf[14]=*(floatdata+2);
//	sendbuf[15]=*(floatdata+3);

//  uint32_t length=12;
//	
//  sendbuf[2]=(uint8_t)length;
//	sendbuf[3]=(uint8_t)(length>>8);
//	sendbuf[16]=0;
//	for(uint8_t i=1;i<17;i++)
//	sendbuf[16]+=sendbuf[i];
//	
//	sendbuf[17]=0x7e;

//	WriteDataToTXDBUF(sendbuf,18);//
//	return 1;
//}
//uint8_t BoardRotate_withtime(uint8_t channel_num)  //����ת��
//{
//	uint8_t sendbuf[20];
//	sendbuf[0]=0x7e;
//	sendbuf[1]=0x44;
//	uint32_t iii=0;
//	
//	sendbuf[4]=g_tRTC.Year;
//	sendbuf[5]=g_tRTC.Year>>8;
//	sendbuf[6]=g_tRTC.Mon;
//	sendbuf[7]=g_tRTC.Day; //ʱ����32λ��ʾ
//	sendbuf[8]=g_tRTC.Hour;
//	sendbuf[9]=g_tRTC.Min;
//	sendbuf[10]=g_tRTC.Sec; //ʱ����32λ��ʾ

//	sendbuf[11]=channel_num+1;//ͨ����
////ת��
//	uint8_t * floatdata=(uint8_t *)&Parameter.SPEED_ROTATE[channel_num];//[0];
//	sendbuf[12]=*floatdata;
//	sendbuf[13]=*(floatdata+1);
//	sendbuf[14]=*(floatdata+2);
//	sendbuf[15]=*(floatdata+3);

//  uint32_t length=12;
//	
//  sendbuf[2]=(uint8_t)length;
//	sendbuf[3]=(uint8_t)(length>>8);
//	sendbuf[16]=0;
//	for(uint8_t i=1;i<17;i++)
//	sendbuf[16]+=sendbuf[i];
//	
//	sendbuf[17]=0x7e;

//	WriteDataToTXDBUF(sendbuf,18);//
//	return 1;
//}
uint16_t Auto_Wave_packege_flag=0;
uint8_t PeriodWaveToSend_Continuous[1500] __attribute__((at(0xC03C0000)));  //���ȫ�ֱ������ĺܲ��ʣ��Ժ���Ŀ������malloc,�´�����ʲôʱ��
void BoardAutoPeroidWave(void)
{ 
	uint8_t checksum=0;
  uint32_t wpp_continuous=0;
	int16_t sendperioddata=0;
	uint32_t SAMPLEblock=(currentSAMPLEblock+1)%2;;

	Auto_Wave_packege_flag=0;
	uint16_t buflength=PERIODBOARDPOINTS+1+7+2;//1���ֽ�ͨ���� 7���ֽ�ʱ�䣬2���ֽڰ���
 for(uint32_t ii=0;ii<Acceleration_ADCHS;ii++){
	 if(((config.DataToSendChannel>>ii)&0x01)==0)  //δʹ�ܵ�ͨ��������
	 {
		 continue;
	 }
	 for(uint32_t i=0;i<(config.channel_freq[ii]);i++) //*config.ADtime
	 {
			sendperioddata=(int16_t)(emu_data[SAMPLEblock][ii][i]*Parameter.ReciprocalofRange[ii]);
			
			PeriodWaveToSend_Continuous[wpp_continuous+14]=sendperioddata;
			PeriodWaveToSend_Continuous[wpp_continuous+15]=sendperioddata>>8;
			checksum+=(PeriodWaveToSend_Continuous[wpp_continuous+14]+PeriodWaveToSend_Continuous[wpp_continuous+15]);			 
			wpp_continuous=wpp_continuous+2;			
		if(wpp_continuous>(PERIODBOARDPOINTS-1)) {	
			PeriodWaveToSend_Continuous[0]=0x7e;//TELid;
			PeriodWaveToSend_Continuous[1]=0x70;//TELid>>8;	
			PeriodWaveToSend_Continuous[2]=buflength;//TELid>>16;
			PeriodWaveToSend_Continuous[3]=(uint8_t)(buflength>>8);// TELid>>24; //2,3????????482??
			PeriodWaveToSend_Continuous[4]=ii+1;// TELid>>24;
			PeriodWaveToSend_Continuous[5]=g_tRTC.Year;
			PeriodWaveToSend_Continuous[6]=g_tRTC.Year>>8;
			PeriodWaveToSend_Continuous[7]=g_tRTC.Mon;
			PeriodWaveToSend_Continuous[8]=g_tRTC.Day; //ʱ����32λ��ʾ
			PeriodWaveToSend_Continuous[9]=g_tRTC.Hour;
			PeriodWaveToSend_Continuous[10]=g_tRTC.Min;
			PeriodWaveToSend_Continuous[11]=g_tRTC.Sec; //ʱ����32λ��ʾ		
			PeriodWaveToSend_Continuous[12]=Auto_Wave_packege_flag; //ʱ����32λ��ʾ
			PeriodWaveToSend_Continuous[13]=Auto_Wave_packege_flag>>8; //ʱ����32λ��ʾ
			for(uint32_t ii=1;ii<14;ii++)
			checksum+=PeriodWaveToSend_Continuous[ii];  //adch�Ǵ�1��ʼ��
			PeriodWaveToSend_Continuous[14+PERIODBOARDPOINTS]=checksum;  //2,3????????482??
			PeriodWaveToSend_Continuous[15+PERIODBOARDPOINTS]=0x7e; 
			Auto_Wave_packege_flag++;
			WriteDataToTXDBUF(PeriodWaveToSend_Continuous,PERIODBOARDPOINTS+16);	
			wpp_continuous=0;
			checksum=0;
//			vTaskDelay (pdMS_TO_TICKS(100));   
			}	 
		}
	}
  

}

static uint16_t BoardPeroidWave_packege_flag=0;  //�����ΰ���

extern RTC_T Requirdperiodwave_tRTC;
uint8_t PeriodWaveToSend[1500] __attribute__((at(0xC03D0000))); 
void BoardPeroidWave(void)
{ 
	uint8_t checksum_once=0;
  uint32_t wpp_once=0;
//	unsigned char *floatdata;
  uint32_t SAMPLEblock=(currentSAMPLEblock+1)%2;
	int16_t send_perioddata_once=0;
//	BoardPeroidWave_packege_flag = 0;
	uint16_t buflength_once=PERIODBOARDPOINTS+1+7+2;//1���ֽ�ͨ���� 7���ֽ�ʱ�䣬2���ֽڰ���
	for(uint32_t ii=0;ii<Acceleration_ADCHS;ii++){
	if(((config.DataToSendChannel>>ii)&0x01)==0)  //δʹ�ܵ�ͨ��������
	{
		continue;
	}
	if(((config.RequirePeriodChannel>>ii)&0x01)==0)  //δ�����ͨ��������
	{
	 continue;
	}
	for(uint32_t i=0;i<(config.channel_freq[ii]);i++) //*config.ADtime
	 {
		//ReceiveSamplesPeriod[SAMPLEblock][ii][i]=100;
		send_perioddata_once=(int16_t)(emu_data[SAMPLEblock][ii][i]*Parameter.ReciprocalofRange[ii]);
		
		PeriodWaveToSend[wpp_once+14]=send_perioddata_once;
		PeriodWaveToSend[wpp_once+15]=send_perioddata_once>>8;
		checksum_once+=(PeriodWaveToSend[wpp_once+14]+PeriodWaveToSend[wpp_once+15]);			 
		wpp_once=wpp_once+2;			
  if(wpp_once>(PERIODBOARDPOINTS-1)) {			
		PeriodWaveToSend[0]=0x7e;//TELid;
		PeriodWaveToSend[1]=0x30;//TELid>>8;	
		PeriodWaveToSend[2]=buflength_once;//TELid>>16;
		PeriodWaveToSend[3]=(uint8_t)(buflength_once>>8);// TELid>>24; //2,3????????482??
		PeriodWaveToSend[4]=ii;// TELid>>24;
		PeriodWaveToSend[5]=Requirdperiodwave_tRTC.Year;
		PeriodWaveToSend[6]=Requirdperiodwave_tRTC.Year>>8;
		PeriodWaveToSend[7]=Requirdperiodwave_tRTC.Mon;
		PeriodWaveToSend[8]=Requirdperiodwave_tRTC.Day; //ʱ����32λ��ʾ
		PeriodWaveToSend[9]=Requirdperiodwave_tRTC.Hour;
		PeriodWaveToSend[10]=Requirdperiodwave_tRTC.Min;
		PeriodWaveToSend[11]=Requirdperiodwave_tRTC.Sec; //ʱ����32λ��ʾ		
		PeriodWaveToSend[12]=BoardPeroidWave_packege_flag; //ʱ����32λ��ʾ
		PeriodWaveToSend[13]=BoardPeroidWave_packege_flag>>8; //ʱ����32λ��ʾ
		for(uint32_t ii=1;ii<14;ii++)
		checksum_once+=PeriodWaveToSend[ii];  //adch�Ǵ�1��ʼ��
		PeriodWaveToSend[14+PERIODBOARDPOINTS]=checksum_once;  //2,3????????482??
		PeriodWaveToSend[15+PERIODBOARDPOINTS]=0x7e; 
		BoardPeroidWave_packege_flag++;
		WriteDataToTXDBUF(PeriodWaveToSend,PERIODBOARDPOINTS+16);	
		wpp_once=0;
		checksum_once=0;
//		vTaskDelay (pdMS_TO_TICKS(1));   
		}	 
	}
}

}


void DATA_EMU_TASK ( void )
{ 
  volatile uint8_t firsttime=0;

 while(1)
 {
  while(!(xSemaphoreTake(SAMPLEDATA_ready, portMAX_DELAY) == pdTRUE))
			{};                // 		 
//		if(config.DataToBoardMode==PARAMETERMODE)

//				bsp_LedStatue(1,1);
				/*					�������߼� */	
				if(Parameter.PeroidWaveTransmissionCounter!=0)
				{
					Parameter.PeroidWaveTransmissionCounter--;
					BoardPeroidWave();
				}else 
				{
					BoardPeroidWave_packege_flag=0;
					Parameter.PeroidWaveTransmissionCounter=0;
				}
				/*				�������߼� */		

				/*				�Զ����Ͳ����߼�            */		
				if(config.PeriodTransimissonStatus==TRUE)
				{																					  
					if((Parameter.AutoPeriodTransmissonCounter+1)>=config.PeriodTransimissonCounter)
					{
						
						BoardAutoPeroidWave();//BoardAutoPeroidWave();  //��������ֵģʽ����Ч			
					}
					
				}
				/*				�Զ����Ͳ��θ�����ֵ�߼�            */	
				if(config.ParameterTransimissonStatus==TRUE)
				{
					if((Parameter.AutoPeriodTransmissonCounter+1)>=config.PeriodTransimissonCounter)
					{
						EmuData();

						BoardParameter_withtime();	//����ֵ									

					}		
				}		

				
				if(config.PeriodTransimissonStatus==TRUE || config.ParameterTransimissonStatus==TRUE)
				{
					if((Parameter.AutoPeriodTransmissonCounter+1)>=config.PeriodTransimissonCounter)
					{										
						Parameter.AutoPeriodTransmissonCounter=0;
					}else
					{
						Parameter.AutoPeriodTransmissonCounter++;
					}
				}
				
				
//				if(isAblePeroidWaveAutoTransmission()&&(config.DataToBoardMode==PARAMETERMODE))  //�Զ��ϴ�����
//		
//				{					
//						EmuData();

//						BoardParameter_withtime();	//����ֵ				

//						BoardAutoPeroidWave();//BoardAutoPeroidWave();  //��������ֵģʽ����Ч																	 
//					
//						DisablePeroidWaveAutoTransmission();
//				}

//					bsp_LedStatue(2,1);
				
		
   	
			
	}
}
/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
