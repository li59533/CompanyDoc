#include "stdio.h"
#include "math.h"
#include "mathlib.h"
#include "dsplib.h"
#include "string.h"
#include "DSPF_sp_cfftr2_dit.h"
#include "gpio.h"
#include "hw_syscfg0_C6748.h"
#include "soc_C6748.h"
#include "dspcache.h"
#include "upp_main.h"
#include "data_struct.h"
int test_flag = 0;

//#pragma DATA_ALIGN(Structural_Vibration_temp, 8)
//float Structural_Vibration_temp[16384];

extern int fftnumber_Mechanical_vibration;
extern int fftnumber_Air_noise;
extern int fftnumber_Platform_vibration;
extern int fftnumber_Equipment_vibration;


void fun_freq_band_function_1(int n_fc_n_case4,int index,int fs);
void fun_freq_band_function_2(int n_fc_n_case4,int index,int fs);
void fun_freq_band_function_3(int n_fc_n_case4,int index,int fs);
void fun_freq_band_function_4(int n_fc_n_case4,int index,int fs);
int n_fc_n_case4_01 = 0;
int n_fc_n_case4_02 = 0;
int n_fc_n_case4_03 = 0;
int n_fc_n_case4_04 = 0;

void DSPF_sp_cfftr2_dit_cn(float* x, float* w, unsigned short n);

extern float UPP_BUFFER_01[11][65536];
extern float UPP_BUFFER_02[11][65536];
extern float UPP_BUFFER_03[11][65536];
extern float UPP_BUFFER_04[11][65536];


#define  pi (3.1415926)
#define Wm_number  (8192)
#define WA_number  (32768)

//fft后输出
//#pragma DATA_SECTION(x_asm_test,".x_sect")
#pragma DATA_ALIGN(x_asm_test, 8)
float x_asm_test[65536];

//wm计权后输出
//#pragma DATA_SECTION(Out_01,".x_sect")
#pragma DATA_ALIGN(Out_01, 8)
float Out_01[Wm_number];
//#pragma DATA_SECTION(Out_1_01,".x_sect")
#pragma DATA_ALIGN(Out_1_01, 8)
float Out_1_01[Wm_number];
//#pragma DATA_SECTION(Out_2_01,".x_sect")
#pragma DATA_ALIGN(Out_2_01, 8)
float Out_2_01[Wm_number];
//wA计权后输出
//#pragma DATA_SECTION(IIR_Out_01,".x_sect")
#pragma DATA_ALIGN(IIR_Out_01, 8)
float IIR_Out_01[WA_number];
//#pragma DATA_SECTION(IIR_Out_1_01,".x_sect")
#pragma DATA_ALIGN(IIR_Out_1_01, 8)
float IIR_Out_1_01[WA_number];
//#pragma DATA_SECTION(IIR_Out_2_01,".x_sect")
#pragma DATA_ALIGN(IIR_Out_2_01, 8)
float IIR_Out_2_01[WA_number];
//#pragma DATA_SECTION(IIR_Out_3_01,".x_sect")
#pragma DATA_ALIGN(IIR_Out_3_01, 8)
float IIR_Out_3_01[WA_number];


extern float W[32768];

extern float Wm[16384];

extern float WA[65536];

extern float Wx[32768];
////% 输入参数：
////% x：原始时域信号（未计权、未加窗）；
////% fs：采样频率，建议采用2^n；
////% nfft：傅立叶变换的计算样本长度，建议采用2^n
////% ch_type：通道类型，1-设备振动  2-人体（平台）振动  3-空气噪声
////% win_type：时域信号x的窗函数类型，0-矩形窗或未加窗  1-汉宁窗  2-海明窗

void fun_specAnalysis(float x_fun[11][65536],int fs,int nfft,int ch_type,int which_array);
void fun_specAnalysis_02(float x_fun[11][65536],int fs,int nfft,int ch_type,int which_array);
void fun_specAnalysis_03(float x_fun[11][65536],int fs,int nfft,int ch_type,int which_array);
void fun_specAnalysis_04(float x_fun[11][65536],int fs,int nfft,int ch_type,int which_array);
void fun_isofilwa(float x[11][65536],int fs,int which_array);
void fun_freq_band(int nc[],int n);
void fun_isofilwm(float x[],int fs);
void fft_cale_Wx(float x_fun[11][65536],int fft_n,int which_array);
//void fft_cale_Wx(float input_buf[],int fft_n);
void fft_cale_W(float input_buf[],int fft_n);
void fft_cale_Wm(float input_buf[],int fft_n);
void fft_cale_WA(float input_buf[],int fft_n);

void bit_rev(float x[], int N);
void gen_w_r2(float w[], int N);


extern float win_32768_data[32768];
extern float win_16384_data[32768];
extern float win_8192_data[8192];
extern float win_4096_data[4096];
//#pragma DATA_SECTION(octSpec_2,".x_sect")
#pragma DATA_ALIGN(octSpec_2, 8)
float octSpec_2[35];//计权后三分之一倍频程
//#pragma DATA_SECTION(linSpec_2,".x_sect")
#pragma DATA_ALIGN(linSpec_2, 8)
float linSpec_2[32768];//计权后fft
float total_1;//计权后加速度有效值
float total_2;//计权后总振级
//#pragma DATA_SECTION(speed_linSpec_2,".x_sect")
#pragma DATA_ALIGN(speed_linSpec_2, 8)
float speed_linSpec_2[32768];//计权后速度频谱
float speed_total_1;//计权速度有效值

//#pragma DATA_SECTION(unweight_octSpec_2,".x_sect")
#pragma DATA_ALIGN(unweight_octSpec_2, 8)
float unweight_octSpec_2[35];//未计权三分之一倍频程
//#pragma DATA_SECTION(unweight_linSpec_2,".x_sect")
#pragma DATA_ALIGN(unweight_linSpec_2, 8)
float unweight_linSpec_2[32768];//未计权fft
float unweight_total_1 = 0;//未计权加速度有效值
float unweight_total_2 = 0;//未计权总振级
float unweight_speed_total_1 = 0;//未计权速度有效值

//#pragma DATA_SECTION(octSpec_2_02,".x_sect")
#pragma DATA_ALIGN(octSpec_2_02, 8)
float octSpec_2_02[35];//计权后三分之一倍频程
//#pragma DATA_SECTION(linSpec_2_02,".x_sect")
#pragma DATA_ALIGN(linSpec_2_02, 8)
float linSpec_2_02[32768];//计权后fft
float total_1_02 = 0;//计权后速度有效值
float total_2_02 = 0;//计权后总振级
//#pragma DATA_SECTION(speed_linSpec_2_02,".x_sect")
#pragma DATA_ALIGN(speed_linSpec_2_02, 8)
float speed_linSpec_2_02[32768];//计权后速度频谱
float speed_total_1_02 = 0;//计权速度有效值


//#pragma DATA_SECTION(unweight_octSpec_2_02,".x_sect")
#pragma DATA_ALIGN(unweight_octSpec_2_02, 8)
float unweight_octSpec_2_02[35];//未计权三分之一倍频程
//#pragma DATA_SECTION(unweight_linSpec_2_02,".x_sect")
#pragma DATA_ALIGN(unweight_linSpec_2_02, 8)
float unweight_linSpec_2_02[32768];//未计权fft
float unweight_total_1_02 = 0;//未计权加速度有效值
float unweight_total_2_02 = 0;//未计权总振级
float unweight_speed_total_1_02 = 0;//未计权速度有效值


//#pragma DATA_SECTION(octSpec_2_03,".x_sect")
#pragma DATA_ALIGN(octSpec_2_03, 8)
float octSpec_2_03[35];//计权后三分之一倍频程
//#pragma DATA_SECTION(linSpec_2_03,".x_sect")
#pragma DATA_ALIGN(linSpec_2_03, 8)
float linSpec_2_03[32768];//计权后fft
float total_1_03 = 0;//计权后速度有效值
float total_2_03 = 0;//计权后总振级
//#pragma DATA_SECTION(speed_linSpec_2_03,".x_sect")
#pragma DATA_ALIGN(speed_linSpec_2_03, 8)
float speed_linSpec_2_03[32768];//计权后速度频谱
float speed_total_1_03 = 0;//计权速度有效值

//#pragma DATA_SECTION(unweight_octSpec_2_03,".x_sect")
#pragma DATA_ALIGN(unweight_octSpec_2_03, 8)
float unweight_octSpec_2_03[35];//未计权三分之一倍频程
//#pragma DATA_SECTION(unweight_linSpec_2_03,".x_sect")
#pragma DATA_ALIGN(unweight_linSpec_2_03, 8)
float unweight_linSpec_2_03[32768];//未计权fft
float unweight_total_1_03 = 0;//未计权速度有效值
float unweight_total_2_03 = 0;//未计权总振级
float unweight_speed_total_1_03 = 0;//未计权速度有效值

//#pragma DATA_SECTION(octSpec_2_04,".x_sect")
#pragma DATA_ALIGN(octSpec_2_04, 8)
float octSpec_2_04[35];//计权后三分之一倍频程
//#pragma DATA_SECTION(linSpec_2_04,".x_sect")
#pragma DATA_ALIGN(linSpec_2_04, 8)
float linSpec_2_04[32768];//计权后fft
float total_1_04 = 0;//计权后速度有效值
float total_2_04 = 0;//计权后总振级
//#pragma DATA_SECTION(speed_linSpec_2_04,".x_sect")
#pragma DATA_ALIGN(speed_linSpec_2_04, 8)
float speed_linSpec_2_04[32768];//计权后速度频谱
float speed_total_1_04 = 0;//计权速度有效值

//#pragma DATA_SECTION(unweight_octSpec_2_04,".x_sect")
#pragma DATA_ALIGN(unweight_octSpec_2_04, 8)
float unweight_octSpec_2_04[35];//未计权三分之一倍频程
//#pragma DATA_SECTION(unweight_linSpec_2_04,".x_sect")
#pragma DATA_ALIGN(unweight_linSpec_2_04, 8)
float unweight_linSpec_2_04[32768];//未计权fft
float unweight_total_1_04 = 0;//未计权速度有效值
float unweight_total_2_04 = 0;//未计权总振级
float unweight_speed_total_1_04 = 0;//未计权速度有效值




//#pragma DATA_SECTION(X,".x_sect")
#pragma DATA_ALIGN(X, 8)
float X[65536];


//机械结构
//#pragma DATA_ALIGN(f_lim, 8)
//float f_lim[44];

#pragma DATA_ALIGN(ID_f_lim_case_1, 8)
int ID_f_lim_case_1[50];
#pragma DATA_ALIGN(ID_f_lim_case_2, 8)
int ID_f_lim_case_2[50];
#pragma DATA_ALIGN(ID_f_lim_case_3, 8)
int ID_f_lim_case_3[50];
#pragma DATA_ALIGN(ID_f_lim_case_4, 8)
int ID_f_lim_case_4[50];

//空气噪声
//#pragma DATA_SECTION(ID_f_lim_3,".x_sect")
//#pragma DATA_ALIGN(ID_f_lim_3, 8)
//int ID_f_lim_3[32]={10,13,16,19,24,30,37,46,58,72,90,113,142,179,225,283,355,447,562,707,890,1120,1410,1775,2234,2812,3540,4456,5609,7061,8889,11200};

////设备振动
////#pragma DATA_SECTION(ID_f_lim_1,".x_sect")
//#pragma DATA_ALIGN(ID_f_lim_1, 8)
//int ID_f_lim_1[32]={6,7,9,10,13,16,19,24,30,37,46,58,72,90,113,142,179,225,283,355,447,562,707,890,1120,1410,1775,2234,2812,3540,4456,5614};
////人体振动
////#pragma DATA_SECTION(ID_f_lim_2,".x_sect")
//#pragma DATA_ALIGN(ID_f_lim_2, 8)
//int ID_f_lim_2[21]={9,10,13,16,19,24,30,37,46,58,73,91,114,144,181,227,285,359,451,568,715};

extern int ID_f_lim_3[32];
extern int ID_f_lim_2[21];
extern int ID_f_lim_1[32];

#pragma DATA_ALIGN(Total_Frequency, 8)
float Total_Frequency[44]={1, 1.25, 1.6, 2, 2.5, 3.15, 4, 5, 6.3, 8, 10,12.5, 16, 20, 25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500, 16000, 20000};
int filterfrequency_lowerlimit_index_01 = 0;
int filterfrequency_upperlimit_index_01 = 0;
int filterfrequency_lowerlimit_index_02 = 0;
int filterfrequency_upperlimit_index_02 = 0;
int filterfrequency_lowerlimit_index_03 = 0;
int filterfrequency_upperlimit_index_03 = 0;
int filterfrequency_lowerlimit_index_04 = 0;
int filterfrequency_upperlimit_index_04 = 0;

extern float coef_win_4096;
extern float coef_win_8192;
extern float coef_win_16384;
extern float coef_win_32768;

extern float e_fft_32768;
extern float e_fft_16384;
extern float e_fft_8192;
extern float e_fft_4096;
extern float  e_fs_32768;
extern float  e_fs_16384;
extern float  e_fs_65536;
extern float  e_fs_4096;

//#pragma DATA_SECTION(Pxx,".x_sect")
#pragma DATA_ALIGN(Pxx, 8)
float Pxx[32768];
#pragma DATA_ALIGN(Pxxx, 8)
float Pxxx[8192];
//#pragma DATA_SECTION(x_win,".x_sect")
#pragma DATA_ALIGN(x_win, 8)
float x_win[32768];

//#pragma DATA_ALIGN(A, 8)
//float A[16384];

//float temp_Amplitude[16384];
//float temp_wave[16384]={};

extern sampling_mudole_channel_struct sampling_mudole_channel_01;
extern sampling_mudole_channel_struct sampling_mudole_channel_02;
extern sampling_mudole_channel_struct sampling_mudole_channel_03;
extern sampling_mudole_channel_struct sampling_mudole_channel_04;

#pragma DATA_ALIGN(halfband, 8)
float halfband[8194];
#pragma DATA_ALIGN(realInt, 8)
float realInt[9000];
#pragma DATA_ALIGN(imgInt, 8)
float imgInt[9000];


//extern float test_cal_buf[16384];

void fun_specAnalysis(float x_fun[11][65536],int fs,int nfft,int ch_type,int which_array)
{

    int f;
	float df_case4 = ((float)fs/nfft);
	float dw;
	dw = 2 * pi * df_case4;

	int indexLow = sampling_mudole_channel_01.filterfrequency_lowerlimit / df_case4;
	int indexHigh = sampling_mudole_channel_01.filterfrequency_upperlimit / df_case4;


    int n_fc = 0;
    int i = 0;
    int j;
    float df = 0.0;
    int m = 0;
    float NB = 0.0;
    float sum_Pxx = 0;
    float sum_Pxx_total = 0;
    float sum_Pxx_total_speed = 0;
    float sum_total_2 = 0;
    int k = 0;
    int fft_num = 0;
    float temp = 0;
    float pingtai[8192] = {0};


//    for(i = 0; i < 16384; i++)
//    {
//        temp_wave[i] = sin(2*5*3.1415*(i/4096.0));
//    }


    switch(ch_type)
    {
    case 1://设备振动
//       n_fc = 31;
       fft_num = 16384;
       df=2; //df = fs*e_fft_16384;
       NB = 1.5;//窗函数噪声带宽修正系数
       for(i = 0; i < nfft; i++)
       {
           x_win[i] = x_fun[which_array][i]*win_16384_data[i];
       }
       fft_cale_W(x_win,fft_num);

       for(i=0;i<ID_f_lim_case_1[n_fc_n_case4_01];i++)
	   {
		   halfband[i]=(i)*dw;
	   }
       for(i = 0; i < 16384; i++)
       {
           X[i] = x_asm_test[i]*e_fs_32768;
       }
       for(i=1;i<ID_f_lim_case_1[n_fc_n_case4_01];i++)
       {
    	   realInt[i] = X[2*i + 1];
    	   imgInt[i]  = (-1)*X[2*i];
           realInt[i]=realInt[i]/halfband[i];
           imgInt[i]=imgInt[i]/halfband[i];
//    	   if(i<indexLow || i>indexHigh)
//    	   {
//    		   realInt[i] = 0;
//    		   imgInt[i]  = 0;
//    	   }
    	   realInt[nfft - 1 -i] = 0;
    	   imgInt[nfft - 1 - i] = 0;
       }
       realInt[0] = 0;
       imgInt[0]  = 0;
       realInt[nfft - 1] = 0;
       imgInt[nfft - 1]  = 0;
       for(i = 0; i < ID_f_lim_case_1[n_fc_n_case4_01]; i++)
       {
           Pxx[i] = 4*(X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*coef_win_16384;
       }

       for(i = 0; i < ID_f_lim_case_1[n_fc_n_case4_01]; i++)
       {
    	   Pxxx[i] = 4*(realInt[i]*realInt[i]+imgInt[i]*imgInt[i])*coef_win_16384*1000000;
       }
       for(k = indexLow; k <= indexHigh; k++)
       {
           temp=Pxxx[k]*2;
           sum_Pxx_total_speed += temp;
       }
       //速度有效值
       unweight_speed_total_1 = sqrtsp(sum_Pxx_total_speed);


       for(i = 1; i <= n_fc_n_case4_01; i++)
       {
           sum_Pxx = 0.0;
           for(k = ID_f_lim_case_1[i-1]-1; k < ID_f_lim_case_1[i]-1; k++)
           {
               sum_Pxx = sum_Pxx + Pxx[k];
           }
           unweight_octSpec_2[i-1] = 10*log10sp(sum_Pxx*2) +120;
       }
       for(k = 0; k < indexLow; k++)
       {
           unweight_linSpec_2[m++] = 0;
       }
       for(k = indexLow; k <= indexHigh; k++)
       {
           temp=Pxx[k]*df;
           sum_Pxx_total += temp;
           unweight_linSpec_2[m++] = sqrtsp(temp*NB);
       }

       unweight_total_1 = sqrtsp(sum_Pxx_total);
       //未计权速度有效值
//       unweight_speed_total_1 = unweight_total_1*0.028;
       for(i = 0; i < n_fc_n_case4_01; i++)
       {
           sum_total_2 += powsp(10,unweight_octSpec_2[i]*0.1);
       }
       unweight_total_2 = 10*log10sp(sum_total_2);

        break;
    case 2://人体（平台）振动
//          n_fc = 20;
        fft_num = 8192;

        for(i = 7; i >=0; i--)
        {
            for(j = 0; j < 1024; j++)
            {
                pingtai[j+1024*i] =  x_fun[which_array][j];
            }
            if(which_array == 0)
            {
                which_array = 10;
            }else
            {
                 which_array--;
            }
        }

        fun_isofilwm(pingtai, fs);//在时域内对x进行Wm计权
        df =0.125;//= (float)fs*e_fft_8192;
           NB = 1.5;//窗函数噪声带宽修正系数
           for(i = 0; i < nfft; i++)
           {
               x_win[i] = Out_2_01[i]*win_8192_data[i];
           }
           fft_cale_Wm(x_win,fft_num);
           for(i = 0; i < nfft; i++)
           {
               X[i] = x_asm_test[i]*e_fs_16384;
           }
           for(i = 0; i < ID_f_lim_case_1[n_fc_n_case4_01]; i++)
           {
               Pxx[i] = (X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*196*coef_win_8192;
           }
           for(i = 1; i <= n_fc_n_case4_01; i++)
           {
               sum_Pxx = 0.0;
               for(k = ID_f_lim_case_1[i-1]-1; k < ID_f_lim_case_1[i]-1; k++)
               {
                   sum_Pxx = sum_Pxx + Pxx[k];
               }
               octSpec_2[i-1] = 10*log10(sum_Pxx*df) +120;
           }
           for(k = 0; k < indexLow; k++)
           {
               linSpec_2[m] = 0;
               speed_linSpec_2[m] = 0;
               m++;
           }
           for(k = indexLow; k <= indexHigh; k++)
           {
               temp=Pxx[k]*df;
               sum_Pxx_total += temp;

               linSpec_2[m] = sqrt(temp*NB);
               speed_linSpec_2[m] = linSpec_2[m]*0.0505152556;
               m++;
           }
           total_1 = sqrt(sum_Pxx_total);//计权后速度有效值
           speed_total_1 = total_1*0.0357142857;//计权后加速度有效值
           for(i = 0; i < n_fc_n_case4_01; i++)
           {
               sum_total_2 += pow(10,octSpec_2[i]*0.1);
           }
           total_2 = 10*log10(sum_total_2);

        break;

    case 3://空气噪声
//        n_fc = 31;
        fft_num = 32768;
        fun_isofilwa(x_fun, fs,which_array);//在时域内对x进行WA计权
        df = (float)fs*e_fft_32768;
           NB = 1.5;//窗函数噪声带宽修正系数
           for(i = 0; i < nfft; i++)
           {
               x_win[i] = IIR_Out_3_01[i]*win_32768_data[i];
           }
           fft_cale_WA(x_win,fft_num);
           for(i = 0; i < nfft; i++)
           {
               X[i] = x_asm_test[i]*e_fs_65536;
           }
           for(i = 0; i < ID_f_lim_case_1[n_fc_n_case4_01]; i++)
           {
               Pxx[i] = 2*(X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*df*coef_win_32768;
           }
           for(i = 1; i <= n_fc_n_case4_01; i++)
           {
               sum_Pxx = 0.0;
               for(k = ID_f_lim_case_1[i-1]-1; k < ID_f_lim_case_1[i]-1; k++)
               {
                   sum_Pxx = sum_Pxx + Pxx[k];
               }
               octSpec_2[i-1] = 10*log10(sum_Pxx*df) +93.9794;
           }

           for(k = 0; k < indexLow; k++)
           {
               linSpec_2[m++] = 0;
           }
           for(k = indexLow; k <= indexHigh; k++)
           {
               temp=Pxx[k]*df;
               sum_Pxx_total += temp;
               linSpec_2[m++] = sqrt(temp*NB);
           }
           total_1 =sqrt(sum_Pxx_total);
           for(i = 0; i < n_fc_n_case4_01; i++)
           {
               sum_total_2 += pow(10,octSpec_2[i]*0.1);
           }
           total_2 = 10*log10(sum_total_2);


        break;
    default:
//        for(i = 4; i >= 0; i--)
//          {
//              for(j = 0; j < 4096; j++)
//              {
//            	  Structural_Vibration_temp[j+4096*i] =  x_fun[which_array][j];
//              }
//              if(which_array == 0)
//              {
//                  which_array = 10;
//              }else
//              {
//                  which_array--;
//              }
//          }

        fft_num = 16384;
        //生成名义中心频率与频带界限值

        df=0.25;//(float)fs/nfft; //df = fs*e_fft_16384;
       NB = 1.5;//窗函数噪声带宽修正系数
//       for(i = 0; i < nfft; i++)
//       {
//           x_win[i] = Structural_Vibration_temp[i]*win_4096_data[i];
////           x_win[i] = test_cal_buf[i]*win_4096_data[i];
//       }
       fft_cale_Wx(x_fun,nfft,which_array);

       for(i=0;i<nfft;i++)
       {
    	   X[i] = x_asm_test[i]*e_fs_4096;
       }
       for(i=0;i<ID_f_lim_case_1[n_fc_n_case4_01];i++)
	   {
		   halfband[i]=(i)*dw;
	   }
       for(i=1;i<ID_f_lim_case_1[n_fc_n_case4_01];i++)
       {
    	   realInt[i] = X[2*i + 1];
    	   imgInt[i]  = (-1)*X[2*i];
           realInt[i]=realInt[i]/halfband[i];
           imgInt[i]=imgInt[i]/halfband[i];
//    	   if(i<indexLow || i>indexHigh)
//    	   {
//    		   realInt[i] = 0;
//    		   imgInt[i]  = 0;
//    	   }
    	   realInt[nfft - 1 -i] = 0;
    	   imgInt[nfft - 1 - i] = 0;
       }
       realInt[0] = 0;
       imgInt[0]  = 0;
       realInt[nfft - 1] = 0;
       imgInt[nfft - 1]  = 0;

       for(i = 0; i < ID_f_lim_case_1[n_fc_n_case4_01]; i++)
       {
    	   Pxx[i] = 2*df*(realInt[i]*realInt[i]+imgInt[i]*imgInt[i])*coef_win_4096*1000000;
       }
//       for(i = 0; i < ID_f_lim_case_1[n_fc_n_case4_01]; i++)
//       {
//    	   Pxx[i] = 2*df*(X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*coef_win_4096;//*1000000;
//       }
//       for(i = 1; i < nfft/2+1; i++)
//       {
//    	   Pxx[i] = (Pxx[i] / ((0.5*pi*i)*(0.5*pi*i)))*1000000;
//       }
       for(i = 1; i <= n_fc_n_case4_01; i++)
       {
           sum_Pxx = 0.0;
           for(k = ID_f_lim_case_1[i-1]-1; k < ID_f_lim_case_1[i]-1; k++)
           {
               sum_Pxx = sum_Pxx + Pxx[k];
           }
           unweight_octSpec_2[i-1] = 10*log10(sum_Pxx*df) +120;
       }
       for(k = 0; k < indexLow; k++)
       {
           unweight_linSpec_2[m++] = 0;
       }
       for(k = indexLow; k <= indexHigh; k++)//从下限频率开始 将-1去掉
       {
           temp=Pxx[k]*df;
		   sum_Pxx_total += temp;
           //速度线谱
           unweight_linSpec_2[m++] = sqrt(temp*NB);
       }
       //未计权速度有效值
       unweight_speed_total_1 = sqrt(sum_Pxx_total);

//       for(i = 0; i < n_fc_n_case4_01; i++)
//       {
//           sum_total_2 += pow(10,unweight_octSpec_2[i]*0.1);
//       }
       //未计权总振级
//       unweight_total_2 = 10*log10(sum_total_2);
    	break;
    }
}
void fun_specAnalysis_02(float x_fun[11][65536],int fs,int nfft,int ch_type,int which_array)
{
	float df_case4 = ((float)fs/nfft);
	float dw;
	dw = 2 * pi * df_case4;
	float sum_Pxx_total_speed = 0;
	int indexLow = sampling_mudole_channel_02.filterfrequency_lowerlimit / df_case4;
	int indexHigh = sampling_mudole_channel_02.filterfrequency_upperlimit / df_case4;

    int n_fc = 0;
    int i = 0;
    int j = 0;
    float df = 0.0;
    int m = 0;
    float NB = 0.0;
    float sum_Pxx = 0;
    float sum_Pxx_total = 0;

    float sum_total_2 = 0;
    int k = 0;
    int fft_num = 0;
    float temp = 0;
    float pingtai[8192] = {0};
    switch(ch_type)
    {
    case 1://设备振动
//        n_fc = 31;
        fft_num = 16384;
        df = 2;
           NB = 1.5;//窗函数噪声带宽修正系数
           for(i = 0; i < nfft; i++)
           {
               x_win[i] = x_fun[which_array][i]*win_16384_data[i];
           }
           fft_cale_W(x_win,fft_num);

//           for(i = 0; i < 16386; i++)
//           {
//               X[i] = x_asm_test[i]*e_fs_32768;
//           }


           for(i=0;i<ID_f_lim_case_2[n_fc_n_case4_02];i++)
    	   {
    		   halfband[i]=(i)*dw;
    	   }

           for(i = 0; i < nfft; i++)
           {
               X[i] = x_asm_test[i]*e_fs_32768;
           }

           for(i=1;i<ID_f_lim_case_2[n_fc_n_case4_02];i++)
           {
        	   realInt[i] = X[2*i + 1];
        	   imgInt[i]  = (-1) * X[2*i];
               realInt[i]=realInt[i]/halfband[i];
               imgInt[i]=imgInt[i]/halfband[i];
//        	   if(i<indexLow || i>indexHigh)
//        	   {
//        		   realInt[i] = 0;
//        		   imgInt[i]  = 0;
//        	   }
        	   realInt[nfft - 1 -i] = 0;
        	   imgInt[nfft - 1 - i] = 0;
           }
           realInt[0] = 0;
           imgInt[0]  = 0;
           realInt[nfft - 1] = 0;
           imgInt[nfft - 1]  = 0;

           for(i = 0; i < ID_f_lim_case_2[n_fc_n_case4_02]; i++)
           {
        	   Pxxx[i] = 4*(realInt[i]*realInt[i]+imgInt[i]*imgInt[i])*coef_win_16384*1000000;
           }
           for(k = indexLow; k <= indexHigh; k++)
           {
               temp=Pxxx[k]*2;
               sum_Pxx_total_speed += temp;
           }
           //速度有效值
           unweight_speed_total_1_02 = sqrtsp(sum_Pxx_total_speed);




           for(i = 0; i < ID_f_lim_case_2[n_fc_n_case4_02]; i++)
           {
               Pxx[i] = 4*(X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*coef_win_16384;
           }
           for(i = 1; i <= n_fc_n_case4_02; i++)
           {
               sum_Pxx = 0.0;
               for(k = ID_f_lim_case_2[i-1]-1; k < ID_f_lim_case_2[i]-1; k++)
               {
                   sum_Pxx = sum_Pxx + Pxx[k];
               }
               unweight_octSpec_2_02[i-1] = 10*log10sp(sum_Pxx*2)+120;
           }
           for(k = 0; k < indexLow; k++)
           {
               unweight_linSpec_2_02[m++] = 0;
           }
           for(k = indexLow; k <= indexHigh; k++)
           {
               temp=Pxx[k]*2;
                sum_Pxx_total += temp;
               unweight_linSpec_2_02[m++] = sqrtsp(temp*NB);
           }
           //加速度有效值
           unweight_total_1_02 = sqrtsp(sum_Pxx_total);
           for(i = 0; i < n_fc_n_case4_02; i++)
           {
               sum_total_2 += powsp(10,unweight_octSpec_2_02[i]*0.1);
           }
           unweight_total_2_02 = 10*log10sp(sum_total_2);

        break;
    case 2://人体（平台）振动
//          n_fc = 20;
          fft_num = 8192;

          for(i = 7; i >= 0; i--)
          {
              for(j = 0; j < 1024; j++)
              {
                  pingtai[j+1024*i] =  x_fun[which_array][j];
              }
              if(which_array == 0)
              {
                 which_array = 10;
              }else
              {
                  which_array--;
              }
          }
          fun_isofilwm(pingtai, fs);//在时域内对x进行Wm计权
          df = (float)fs*e_fft_8192;
          NB = 1.5;//窗函数噪声带宽修正系数

           for(i = 0; i < nfft; i++)
           {
               x_win[i] = Out_2_01[i]*win_8192_data[i];
           }

           fft_cale_Wm(x_win,fft_num);

           for(i = 0; i < nfft; i++)
           {
               X[i] = x_asm_test[i]*e_fs_16384;
           }
           for(i = 0; i < ID_f_lim_case_2[n_fc_n_case4_02]; i++)
           {
               Pxx[i] = (X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*df*1568*coef_win_8192;
           }

           for(i = 1; i <= n_fc_n_case4_02; i++)
           {
               sum_Pxx = 0.0;
               for(k = ID_f_lim_case_2[i-1]-1; k < ID_f_lim_case_2[i]-1; k++)
               {
                   sum_Pxx = sum_Pxx + Pxx[k];
               }
               octSpec_2_02[i-1] = 10*log10(sum_Pxx*df) +120;
           }

           for(k = 0; k < indexLow; k++)
           {
               speed_linSpec_2_02[m] = 0;
               linSpec_2_02[m] = 0;
               m++;
           }
           for(k = indexLow; k <= indexHigh; k++)
           {
               temp=Pxx[k]*df;
                sum_Pxx_total += temp;
               linSpec_2_02[m] = sqrt(temp*NB);
                speed_linSpec_2_02[m] = linSpec_2_02[m]*0.05051525560719337239846433622955;
                m++;
           }
           total_1_02 = sqrt(sum_Pxx_total);
           speed_total_1_02 = total_1_02*0.0357142857;;
           for(i = 0; i < n_fc_n_case4_02; i++)
           {
               sum_total_2 += pow(10,octSpec_2_02[i]*0.1);
           }
           total_2_02 = 10*log10(sum_total_2);

        break;

    case 3://空气噪声
//           n_fc = 31;
           fft_num = 32768;
           fun_isofilwa(x_fun, fs,which_array);//在时域内对x进行WA计权
           df = (float)fs*e_fft_32768;
           NB = 1.5;//窗函数噪声带宽修正系数

           for(i = 0; i < nfft; i++)
           {
               x_win[i] = IIR_Out_3_01[i]*win_32768_data[i];
           }

           fft_cale_WA(x_win,fft_num);

           for(i = 0; i < nfft; i++)
           {
               X[i] = x_asm_test[i]*e_fs_65536;
           }
           for(i = 0; i < ID_f_lim_case_2[n_fc_n_case4_02]; i++)
           {
               Pxx[i] = 2*(X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*df*coef_win_32768;
           }

           for(i = 1; i <= n_fc_n_case4_02; i++)
           {
               sum_Pxx = 0.0;
               for(k = ID_f_lim_case_2[i-1]-1; k < ID_f_lim_case_2[i]-1; k++)
               {
                   sum_Pxx = sum_Pxx + Pxx[k];
               }
               octSpec_2_02[i-1] = 10*log10(sum_Pxx*df)+93.9794;
           }

           for(k = 0; k < indexLow; k++)
           {
               linSpec_2_02[m++] = 0;
           }
           for(k = indexLow; k <= indexHigh; k++)
           {
               temp=Pxx[k]*df;
                sum_Pxx_total += temp;
               linSpec_2_02[m++] = sqrt(temp*NB);
           }
           total_1_02 = sqrt(sum_Pxx_total);

           for(i = 0; i < n_fc_n_case4_02; i++)
           {
               sum_total_2 += pow(10,octSpec_2_02[i]*0.1);
           }
           total_2_02 = 10*log10(sum_total_2);

        break;
    default:
//        for(i = 3; i >= 0; i--)
//          {
//              for(j = 0; j < 4096; j++)
//              {
//            	  Structural_Vibration_temp[j+4096*i] =  x_fun[which_array][j];
//              }
//              if(which_array == 0)
//              {
//                  which_array = 10;
//              }else
//              {
//                  which_array--;
//              }
//          }
        fft_num = 16384;
        df=(float)fs/nfft; //df = fs*e_fft_16384;
       NB = 1.5;//窗函数噪声带宽修正系数
//       for(i = 0; i < nfft; i++)
//       {
//           x_win[i] = Structural_Vibration_temp[i]*win_4096_data[i];
//       }
       fft_cale_Wx(x_fun,nfft,which_array);

       for(i=0;i<nfft;i++)
       {
    	   X[i] = x_asm_test[i]*e_fs_4096;
       }
       for(i=0;i<ID_f_lim_case_2[n_fc_n_case4_02];i++)
	   {
		   halfband[i]=(i)*dw;
	   }
       for(i=1;i<ID_f_lim_case_2[n_fc_n_case4_02];i++)
       {
    	   realInt[i] = x_asm_test[2*i + 1]*e_fs_4096;
    	   imgInt[i]  = (-1)*x_asm_test[2*i]*e_fs_4096;
           realInt[i]=realInt[i]/halfband[i];
           imgInt[i]=imgInt[i]/halfband[i];
//    	   if(i<indexLow || i>indexHigh)
//    	   {
//    		   realInt[i] = 0;
//    		   imgInt[i]  = 0;
//    	   }
    	   realInt[nfft - 1 -i] = 0;
    	   imgInt[nfft - 1 - i] = 0;
       }
       realInt[0] = 0;
       imgInt[0]  = 0;
       realInt[nfft - 1] = 0;
       imgInt[nfft - 1]  = 0;


       for(i = 0; i < ID_f_lim_case_2[n_fc_n_case4_02]; i++)
       {
    	   Pxx[i] = 2*df*(realInt[i]*realInt[i]+imgInt[i]*imgInt[i])*coef_win_4096*1000000;
       }

//       for(i = 0; i < ID_f_lim_case_2[n_fc_n_case4_02]; i++)
//       {
//    	   Pxx[i] = 2*df*(X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*coef_win_4096;//*1000000;
//       }
//       for(i = 1; i < nfft/2+1; i++)
//       {
//    	   Pxx[i] = (Pxx[i] / ((0.5*pi*i)*(0.5*pi*i)))*1000000;
//       }



       for(i = 1; i <= n_fc_n_case4_02; i++)
       {
           sum_Pxx = 0.0;
           for(k = ID_f_lim_case_2[i-1]-1; k < ID_f_lim_case_2[i]-1; k++)
           {
               sum_Pxx = sum_Pxx + Pxx[k];
           }
           unweight_octSpec_2_02[i-1] = 10*log10(sum_Pxx*df) +120;
       }
       for(k = 0; k < indexLow; k++)
       {
    	   unweight_linSpec_2_02[m++] =0;
       }

       for(k = indexLow; k <= indexHigh; k++)//从下限频率开始 将-1去掉
//       for(k = ID_f_lim_case_2[0]-1; k < ID_f_lim_case_2[n_fc_n_case4_02]-1; k++)
       {
           temp=Pxx[k]*df;
           sum_Pxx_total += temp;
           unweight_linSpec_2_02[m++] = sqrt(temp*NB);
       }
       //未计权速度有效值
       unweight_speed_total_1_02 = sqrt(sum_Pxx_total);

//       for(i = 0; i < n_fc_n_case4_02; i++)
//       {
//           sum_total_2 += pow(10,unweight_octSpec_2_02[i]*0.1);
//       }
//       unweight_total_2_02 = 10*log10(sum_total_2);
    	break;
    }
}

void fun_specAnalysis_03(float x_fun[11][65536],int fs,int nfft,int ch_type,int which_array)
{
	float df_case4 = ((float)fs/nfft);
	float dw;
	dw = 2 * pi * df_case4;
	float sum_Pxx_total_speed = 0;
	int indexLow = sampling_mudole_channel_03.filterfrequency_lowerlimit / df_case4;
	int indexHigh = sampling_mudole_channel_03.filterfrequency_upperlimit / df_case4;

    int n_fc = 0;
    int i = 0;
    int j = 0;
    float df = 0.0;
    int m = 0;
    float NB = 0.0;

    float sum_Pxx = 0;
    float sum_Pxx_total = 0;
    float sum_total_2 = 0;
    int k = 0;
    int fft_num = 0;
    float temp = 0;
    float pingtai[8192] = {0};
    switch(ch_type)
    {
    case 1://设备振动
//           n_fc = 31;
           fft_num = 16384;
           df = 2;
           NB = 1.5;//窗函数噪声带宽修正系数
           for(i = 0; i < nfft; i++)
           {
               x_win[i] = x_fun[which_array][i]*win_16384_data[i];
           }
           fft_cale_W(x_win,fft_num);

           for(i=0;i<ID_f_lim_case_3[n_fc_n_case4_03];i++)
    	   {
    		   halfband[i]=(i)*dw;
    	   }

           for(i = 0; i < nfft; i++)
           {
               X[i] = x_asm_test[i]*e_fs_32768;
           }

           for(i=1;i<ID_f_lim_case_3[n_fc_n_case4_03];i++)
           {
        	   realInt[i] = X[2*i + 1];
        	   imgInt[i]  = (-1)*X[2*i];
               realInt[i]=realInt[i]/halfband[i];
               imgInt[i]=imgInt[i]/halfband[i];
//        	   if(i<indexLow || i>indexHigh)
//        	   {
//        		   realInt[i] = 0;
//        		   imgInt[i]  = 0;
//        	   }
        	   realInt[nfft - 1 -i] = 0;
        	   imgInt[nfft - 1 - i] = 0;
           }
           realInt[0] = 0;
           imgInt[0]  = 0;
           realInt[nfft - 1] = 0;
           imgInt[nfft - 1]  = 0;



           for(i = 0; i < ID_f_lim_case_3[n_fc_n_case4_03]; i++)
           {
               Pxx[i] = 4*(X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*coef_win_16384;
           }

           for(i = 5; i < ID_f_lim_case_3[n_fc_n_case4_03]; i++)
           {
        	   Pxxx[i] = 4*(realInt[i]*realInt[i]+imgInt[i]*imgInt[i])*coef_win_16384*1000000;
           }
           for(k = indexLow; k <= indexHigh; k++)
           {
               temp=Pxxx[k]*2;
               sum_Pxx_total_speed += temp;
           }
           //速度有效值
           unweight_speed_total_1_03 = sqrtsp(sum_Pxx_total_speed);


           for(i = 1; i <= n_fc_n_case4_03; i++)
           {
               sum_Pxx = 0.0;
               for(k = ID_f_lim_case_3[i-1]-1; k < ID_f_lim_case_3[i]-1; k++)
               {
                   sum_Pxx = sum_Pxx + Pxx[k];
               }
               unweight_octSpec_2_03[i-1] = 10*log10sp(sum_Pxx*2) +120;
           }
           for(k = 0; k < indexLow; k++)
           {
               unweight_linSpec_2_03[m++] = 0;
           }
           for(k = indexLow; k < indexHigh; k++)
           {
               temp=Pxx[k]*df;
               sum_Pxx_total += temp;
               unweight_linSpec_2_03[m++] = sqrtsp(temp*NB);
           }
           unweight_total_1_03 = sqrtsp(sum_Pxx_total);
           //速度有效值
//           unweight_speed_total_1_03 = unweight_total_1_03*0.028;

           for(i = 0; i < n_fc_n_case4_03; i++)
           {
               sum_total_2 += powsp(10,unweight_octSpec_2_03[i]*0.1);
           }
           unweight_total_2_03 = 10*log10sp(sum_total_2);

        break;
    case 2://人体（平台）振动
//           n_fc = 20;
           fft_num = 8192;

           for(i = 7; i >= 0; i--)
            {
                for(j = 0; j < 1024; j++)
                {
                    pingtai[j+1024*i] =  x_fun[which_array][j];
                }
                if(which_array == 0)
                {
                   which_array = 10;
                }else
                {
                    which_array--;
                }
            }

           fun_isofilwm(pingtai,fs);//在时域内对x进行Wm计权

           df = (float)fs*e_fft_8192;
           NB = 1.5;//窗函数噪声带宽修正系数

           for(i = 0; i < nfft; i++)
           {
               x_win[i] = Out_2_01[i]*win_8192_data[i];
           }
           fft_cale_Wm(x_win,fft_num);
           for(i = 0; i < nfft; i++)
           {
               X[i] = x_asm_test[i]*e_fs_16384;
           }
           for(i = 0; i < ID_f_lim_case_3[n_fc_n_case4_03]; i++)
           {
               Pxx[i] = (X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*df*1568*coef_win_8192;
           }

           for(i = 1; i <= n_fc_n_case4_03; i++)
           {
               sum_Pxx = 0.0;
               for(k = ID_f_lim_case_3[i-1]-1; k < ID_f_lim_case_3[i]-1; k++)
               {
                   sum_Pxx = sum_Pxx + Pxx[k];
               }
               octSpec_2_03[i-1] = 10*log10(sum_Pxx*df) +120;
           }

           for(k = 0; k < indexLow; k++)
           {
               linSpec_2_03[m] = 0;
               speed_linSpec_2_03[m] = 0;
               m++;
           }
           for(k = indexLow; k <= indexHigh; k++)
           {
               temp=Pxx[k]*df;
               sum_Pxx_total += temp;
               linSpec_2_03[m] = sqrt(temp*NB);
                speed_linSpec_2_03[m] = linSpec_2_03[m]*0.05051525560719337239846433622955;
                m++;
           }
           total_1_03 = sqrt(sum_Pxx_total);
           speed_total_1_03 = total_1_03 *0.0357142857;
           for(i = 0; i < n_fc_n_case4_03; i++)
           {
               sum_total_2 += pow(10,octSpec_2_03[i]*0.1);
           }
           total_2_03 = 10*log10(sum_total_2);



        break;

    case 3://空气噪声
//           n_fc = 31;
           fft_num = 32768;
           fun_isofilwa(x_fun,fs,which_array);//在时域内对x进行WA计权
           df = (float)fs*e_fft_32768;
           NB = 1.5;//窗函数噪声带宽修正系数

           for(i = 0; i < nfft; i++)
           {
               x_win[i] = IIR_Out_3_01[i]*win_32768_data[i];
           }

           fft_cale_WA(x_win,fft_num);

           for(i = 0; i < nfft; i++)
           {
               X[i] = x_asm_test[i]*e_fs_65536;
           }
           for(i = 0; i < ID_f_lim_case_3[n_fc_n_case4_03]; i++)
           {
               Pxx[i] = 2*(X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*df*coef_win_32768;
           }

           for(i = 1; i <= n_fc_n_case4_03; i++)
           {
               sum_Pxx = 0.0;
               for(k = ID_f_lim_case_3[i-1]-1; k < ID_f_lim_case_3[i]-1; k++)
               {
                   sum_Pxx = sum_Pxx + Pxx[k];
               }
               octSpec_2_03[i-1] = 10*log10(sum_Pxx*df) +93.9794;
           }

           for(k = 0; k < indexLow; k++)
           {
               linSpec_2_03[m++] = 0;
           }
           for(k = indexLow; k <= indexHigh; k++)
           {
               temp=Pxx[k]*df;
               sum_Pxx_total += temp;
               linSpec_2_03[m++] = sqrt(temp*NB);
           }
           total_1_03 = sqrt(sum_Pxx_total);

           for(i = 0; i < n_fc_n_case4_03; i++)
           {
               sum_total_2 += pow(10,octSpec_2_03[i]*0.1);
           }
           total_2_03 = 10*log10(sum_total_2);

        break;
    default:
//        for(i = 3; i >= 0; i--)
//          {
//              for(j = 0; j < 4096; j++)
//              {
//            	  Structural_Vibration_temp[j+4096*i] =  x_fun[which_array][j];
//              }
//              if(which_array == 0)
//              {
//                  which_array = 10;
//              }else
//              {
//                  which_array--;
//              }
//          }
        fft_num = 16384;
        df=(float)fs/nfft; //df = fs*e_fft_16384;
       NB = 1.5;//窗函数噪声带宽修正系数
//       for(i = 0; i < nfft; i++)
//       {
//           x_win[i] = Structural_Vibration_temp[i]*win_4096_data[i];
//       }
       fft_cale_Wx(x_fun,nfft,which_array);
       for(i=0;i<nfft;i++)
       {
    	   X[i] = x_asm_test[i]*e_fs_4096;
       }
       for(i=0;i<ID_f_lim_case_3[n_fc_n_case4_03];i++)
	   {
		   halfband[i]=(i)*dw;
	   }
       for(i=1;i<ID_f_lim_case_3[n_fc_n_case4_03];i++)
       {
    	   realInt[i] = x_asm_test[2*i + 1]*e_fs_4096;
    	   imgInt[i]  = (-1)*x_asm_test[2*i]*e_fs_4096;
           realInt[i]=realInt[i]/halfband[i];
           imgInt[i]=imgInt[i]/halfband[i];
//    	   if(i<indexLow || i>indexHigh)
//    	   {
//    		   realInt[i] = 0;
//    		   imgInt[i]  = 0;
//    	   }
    	   realInt[nfft - 1 -i] = 0;
    	   imgInt[nfft - 1 - i] = 0;
       }
       realInt[0] = 0;
       imgInt[0]  = 0;
       realInt[nfft - 1] = 0;
       imgInt[nfft - 1]  = 0;


       for(i = 0; i < ID_f_lim_case_3[n_fc_n_case4_03]; i++)
       {
    	   Pxx[i] = 2*df*(realInt[i]*realInt[i]+imgInt[i]*imgInt[i])*coef_win_4096*1000000;
       }
//       for(i = 0; i < ID_f_lim_case_3[n_fc_n_case4_03]; i++)
//       {
//    	   Pxx[i] = 2*df*(X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*coef_win_4096;//*1000000;
//       }
//       for(i = 1; i < nfft/2+1; i++)
//       {
//    	   Pxx[i] = (Pxx[i] / ((0.5*pi*i)*(0.5*pi*i)))*1000000;
//       }



       for(i = 1; i <= n_fc_n_case4_03; i++)
       {
           sum_Pxx = 0.0;
           for(k = ID_f_lim_case_3[i-1]-1; k < ID_f_lim_case_3[i]-1; k++)
           {
               sum_Pxx = sum_Pxx + Pxx[k];
           }
           unweight_octSpec_2_03[i-1] = 10*log10(sum_Pxx*df) +120;
       }
       for(k = 0; k < indexLow; k++)
       {
    	   unweight_linSpec_2_03[m++] = 0;
       }
//       for(k = ID_f_lim_case_3[0]-1; k < ID_f_lim_case_3[n_fc_n_case4_03]-1; k++)
       for(k = indexLow; k <= indexHigh; k++)//从下限频率开始 将-1去掉
       {
           temp=Pxx[k]*df;
           sum_Pxx_total += temp;
           unweight_linSpec_2_03[m++] = sqrt(temp*NB);
       }
       //未计权速度有效值
       unweight_speed_total_1_03 = sqrt(sum_Pxx_total);

//       for(i = 0; i < n_fc_n_case4_03; i++)
//       {
//           sum_total_2 += pow(10,unweight_octSpec_2_03[i]*0.1);
//       }
//       unweight_total_2_03 = 10*log10(sum_total_2);
    	break;
    }
    //calculate_ok_03 = 1; 100 hz  100mv
}
void fun_specAnalysis_04(float x_fun[11][65536],int fs,int nfft,int ch_type,int which_array)
{
	float df_case4 = ((float)fs/nfft);
	float dw;
	dw = 2 * pi * df_case4;
	float sum_Pxx_total_speed = 0;
	int indexLow = sampling_mudole_channel_04.filterfrequency_lowerlimit / df_case4;
	int indexHigh = sampling_mudole_channel_04.filterfrequency_upperlimit / df_case4;

    int n_fc = 0;
    int i = 0;
    int  j = 0;
    float df = 0.0;
    int m = 0;
    float NB = 0.0;

    float sum_Pxx = 0;
    float sum_Pxx_total = 0;
    float sum_total_2 = 0;
    int k = 0;
    int fft_num = 0;
    float temp = 0;
    float pingtai[8192] = {0};
    switch(ch_type)
    {
    case 1://设备振动
//           n_fc = 31;
           fft_num = 16384;

           df =2;
           NB = 1.5;//窗函数噪声带宽修正系数
           for(i = 0; i < nfft; i++)
           {
               x_win[i] = x_fun[which_array][i]*win_16384_data[i];
           }

           fft_cale_W(x_win,fft_num);

           for(i=0;i<ID_f_lim_case_4[n_fc_n_case4_04];i++)
    	   {
    		   halfband[i]=(i)*dw;
    	   }

           for(i = 0; i < nfft; i++)
           {
               X[i] = x_asm_test[i]*e_fs_32768;
           }

           for(i=1;i<ID_f_lim_case_4[n_fc_n_case4_04];i++)
           {
        	   realInt[i] = X[2*i + 1];
        	   imgInt[i]  =(-1)*X[2*i];
               realInt[i]=realInt[i]/halfband[i];
               imgInt[i]=imgInt[i]/halfband[i];
//        	   if(i<indexLow || i>indexHigh)
//        	   {
//        		   realInt[i] = 0;
//        		   imgInt[i]  = 0;
//        	   }
        	   realInt[nfft - 1 -i] = 0;
        	   imgInt[nfft - 1 - i] = 0;
           }
           realInt[0] = 0;
           imgInt[0]  = 0;
           realInt[nfft - 1] = 0;
           imgInt[nfft - 1]  = 0;

           for(i = 0; i < ID_f_lim_case_4[n_fc_n_case4_04]; i++)
           {
               Pxx[i] = 4*(X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*coef_win_16384;
           }

           for(i = 0; i < ID_f_lim_case_4[n_fc_n_case4_04]; i++)
           {
        	   Pxxx[i] = 4*(realInt[i]*realInt[i]+imgInt[i]*imgInt[i])*coef_win_16384*1000000;
           }
           for(k = indexLow; k <= indexHigh; k++)
           {
               temp=Pxxx[k]*2;
               sum_Pxx_total_speed += temp;
           }
           //速度有效值
           unweight_speed_total_1_04 = sqrtsp(sum_Pxx_total_speed);


           for(i = 1; i <= n_fc_n_case4_04; i++)
           {
               sum_Pxx = 0.0;
               for(k = ID_f_lim_case_4[i-1]-1; k < ID_f_lim_case_4[i]-1; k++)
               {
                   sum_Pxx = sum_Pxx + Pxx[k];
               }
               unweight_octSpec_2_04[i-1] = 10*log10sp(sum_Pxx*2) +120;
           }
           for(k = 0; k < indexLow; k++)
           {
               unweight_linSpec_2_04[m++] = 0;
           }
           for(k = indexLow; k <= indexHigh; k++)
           {
               temp=Pxx[k]*df;
               sum_Pxx_total += temp;
               unweight_linSpec_2_04[m++] = sqrtsp(temp*NB);
           }
           unweight_total_1_04 = sqrtsp(sum_Pxx_total);

           //加速度有效值
//           unweight_speed_total_1_04 = unweight_total_1_04*0.028;

           for(i = 0; i < n_fc_n_case4_04; i++)
           {
               sum_total_2 += powsp(10,unweight_octSpec_2_04[i]*0.1);
           }
           unweight_total_2_04 = 10*log10sp(sum_total_2);
        break;
    case 2://人体（平台）振动
//          n_fc = 20;
          fft_num = 8192;
          for(i = 7; i >= 0; i--)
            {
                for(j = 0; j < 1024; j++)
                {
                    pingtai[j+1024*i] =  x_fun[which_array][j];
                }
                if(which_array == 0)
                {
                    which_array = 10;
                }else
                {
                    which_array--;
                }
            }
          fun_isofilwm(pingtai,fs);//在时域内对x进行Wm计权
          df = (float)fs*e_fft_8192;
           NB = 1.5;//窗函数噪声带宽修正系数

           for(i = 0; i < nfft; i++)
           {
               x_win[i] = Out_2_01[i]*win_8192_data[i];
           }

           fft_cale_Wm(x_win,fft_num);

           for(i = 0; i < nfft; i++)
           {
               X[i] = x_asm_test[i]*e_fs_16384;
           }
           for(i = 0; i < ID_f_lim_case_4[n_fc_n_case4_04]; i++)
           {
               Pxx[i] = (X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*df*1568*coef_win_8192;
           }

           for(i = 1; i <= n_fc_n_case4_04; i++)
           {
               sum_Pxx = 0.0;
               for(k = ID_f_lim_case_4[i-1]-1; k < ID_f_lim_case_4[i]-1; k++)
               {
                   sum_Pxx = sum_Pxx + Pxx[k];
               }
               octSpec_2_04[i-1] = 10*log10(sum_Pxx*df) +120;
           }

           for(k = 0; k < indexLow; k++)
           {
               linSpec_2_04[m] = 0;
               speed_linSpec_2_04[m] = 0;
               m++;
           }
           for(k = indexLow; k <= indexHigh; k++)
           {
               temp=Pxx[k]*df;
               sum_Pxx_total += temp;
               linSpec_2_04[m] = sqrt(temp*NB);
             speed_linSpec_2_04[m] = linSpec_2_04[m]*0.05051525560719337239846433622955;
             m++;
           }
           total_1_04 = sqrt(sum_Pxx_total);
           speed_total_1_04 = total_1_04 *0.0357142857;
           for(i = 0; i < n_fc_n_case4_04; i++)
           {
               sum_total_2 += pow(10,octSpec_2_04[i]*0.1);
           }
           total_2_04 = 10*log10(sum_total_2);



        break;

    case 3://空气噪声
//           n_fc = 31;
           fft_num = 32768;
           fun_isofilwa(x_fun,fs,which_array);//在时域内对x进行WA计权
           df = (float)fs*e_fft_32768;
           NB = 1.5;//窗函数噪声带宽修正系数

           for(i = 0; i < nfft; i++)
           {
               x_win[i] = IIR_Out_3_01[i]*win_32768_data[i];
           }

           fft_cale_WA(x_win,fft_num);

           for(i = 0; i < nfft; i++)
           {
               X[i] = x_asm_test[i]*e_fs_65536;
           }
           for(i = 0; i < ID_f_lim_case_4[n_fc_n_case4_04]; i++)
           {
               Pxx[i] = 2*(X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*df*coef_win_32768;
           }

           for(i = 1; i <= n_fc_n_case4_04; i++)
           {
               sum_Pxx = 0.0;
               for(k = ID_f_lim_case_4[i-1]-1; k < ID_f_lim_case_4[i]-1; k++)
               {
                   sum_Pxx = sum_Pxx + Pxx[k];
               }
               octSpec_2_04[i-1] = 10*log10(sum_Pxx*df)+93.9794;
           }

           for(k = 0; k < indexLow; k++)
           {
               linSpec_2_04[m++] = 0;
           }
           for(k = indexLow; k <= indexHigh; k++)
           {
               temp=Pxx[k]*df;
               sum_Pxx_total += temp;
               linSpec_2_04[m++] = sqrt(temp*NB);
           }
           total_1_04 = sqrt(sum_Pxx_total);

           for(i = 0; i < n_fc_n_case4_04; i++)
           {
               sum_total_2 += pow(10,octSpec_2_04[i]*0.1);
           }
           total_2_04 = 10*log10(sum_total_2);

        break;
    default:
//        for(i = 3; i >= 0; i--)
//          {
//              for(j = 0; j < 4096; j++)
//              {
//            	  Structural_Vibration_temp[j+4096*i] =  x_fun[which_array][j];
//              }
//              if(which_array == 0)
//              {
//                  which_array = 10;
//              }else
//              {
//                  which_array--;
//              }
//          }
//    	n_fc_n_case4_04 = filterfrequency_upperlimit_index_04 - filterfrequency_lowerlimit_index_04 + 1;
//        n_fc = 31;
        fft_num = 16384;
//        fun_freq_band_function(n_fc_n_case4_04,filterfrequency_lowerlimit_index_04);
        df=(float)fs/nfft; //df = fs*e_fft_16384;
       NB = 1.5;//窗函数噪声带宽修正系数
//       for(i = 0; i < nfft; i++)
//       {
//           x_win[i] = Structural_Vibration_temp[i]*win_4096_data[i];
//       }
       fft_cale_Wx(x_fun,nfft,which_array);
       for(i=0;i<nfft;i++)
       {
    	   X[i] = x_asm_test[i]*e_fs_4096;
       }
       for(i=0;i<ID_f_lim_case_4[n_fc_n_case4_04];i++)
	   {
		   halfband[i]=(i)*dw;
	   }
       for(i=1;i<ID_f_lim_case_4[n_fc_n_case4_04] ;i++)
       {
    	   realInt[i] = x_asm_test[2*i + 1]*e_fs_4096;
    	   imgInt[i]  = (-1)*x_asm_test[2*i]*e_fs_4096;
           realInt[i]=realInt[i]/halfband[i];
           imgInt[i]=imgInt[i]/halfband[i];
//    	   if(i<indexLow || i>indexHigh)
//    	   {
//    		   realInt[i] = 0;
//    		   imgInt[i]  = 0;
//    	   }
    	   realInt[nfft - 1 -i] = 0;
    	   imgInt[nfft - 1 - i] = 0;
       }
       realInt[0] = 0;
       imgInt[0]  = 0;
       realInt[nfft - 1] = 0;
       imgInt[nfft - 1]  = 0;


       for(i = 0; i < ID_f_lim_case_4[n_fc_n_case4_04]; i++)
       {
    	   Pxx[i] = 2*df*(realInt[i]*realInt[i]+imgInt[i]*imgInt[i])*coef_win_4096*1000000;
       }

//       for(i = 0; i < ID_f_lim_case_4[n_fc_n_case4_04]; i++)
//       {
//    	   Pxx[i] = 2*df*(X[i*2]*X[i*2]+X[i*2+1]*X[i*2+1])*coef_win_4096;//*1000000;
//       }
//       for(i = 1; i < nfft/2+1; i++)
//       {
//    	   Pxx[i] = (Pxx[i] / ((0.5*pi*i)*(0.5*pi*i)))*1000000;
//       }

       for(i = 1; i <= n_fc_n_case4_04; i++)
       {
           sum_Pxx = 0.0;
           for(k = ID_f_lim_case_4[i-1]-1; k < ID_f_lim_case_4[i]-1; k++)
           {
               sum_Pxx = sum_Pxx + Pxx[k];
           }
           unweight_octSpec_2_04[i-1] = 10*log10(sum_Pxx*df) +120;
       }
       for(k = 0; k < indexLow; k++)
       {
    	   unweight_linSpec_2_04[m++] = 0;
       }
//       for(k = ID_f_lim_case_4[0]-1; k < ID_f_lim_case_4[n_fc_n_case4_04]-1; k++)
       for(k = indexLow; k <= indexHigh; k++)//从下限频率开始 将-1去掉
       {
           temp=Pxx[k]*df;
           sum_Pxx_total += temp;
           unweight_linSpec_2_04[m++] = sqrt(temp*NB);
       }
       //未计权速度有效值
       unweight_speed_total_1_04 = sqrt(sum_Pxx_total);

//       for(i = 0; i < n_fc_n_case4_04; i++)
//       {
//           sum_total_2 += pow(10,unweight_octSpec_2_04[i]*0.1);
//       }
//       unweight_total_2_04 = 10*log10(sum_total_2);
    	break;
    }
}


void fft_cale_WA(float input_buf[],int fft_n)
{
    unsigned int k;
    int fft_num = fft_n;
    for(k = 0; k < fft_num; k++)
    {
        x_asm_test[2 * k] = input_buf[k];
        x_asm_test[2 * k + 1] =0.0;
    }
    DSPF_sp_cfftr2_dit_cn(x_asm_test, WA, fft_num);
//     DSPF_sp_cfftr2_dit(x_asm_test, WA, fft_num);
     bit_rev(x_asm_test, fft_num);
}
void fft_cale_Wm(float input_buf[],int fft_n)
{
    unsigned int k;
    int fft_num = fft_n;

    for(k = 0; k < fft_num; k++)
    {
        x_asm_test[2 * k] = input_buf[k];
        x_asm_test[2 * k + 1] =0.0;
    }
    DSPF_sp_cfftr2_dit_cn(x_asm_test, Wm, fft_num);
//     DSPF_sp_cfftr2_dit(x_asm_test, Wm, fft_num);
     bit_rev(x_asm_test, fft_num);
}
void fft_cale_W(float input_buf[],int fft_n)
{
    unsigned int k;
    int fft_num = fft_n;
    for(k = 0; k < fft_num; k++)
    {
        x_asm_test[2 * k] = input_buf[k];
        x_asm_test[2 * k + 1] =0.0;
    }
    DSPF_sp_cfftr2_dit_cn(x_asm_test, W, fft_num);
//     DSPF_sp_cfftr2_dit(x_asm_test, W, fft_num);
     bit_rev(x_asm_test, fft_num);

}
void fft_cale_Wx(float x_fun[11][65536],int fft_n,int which_array)
{
     int k,i,j;
    for(i = 3; i >= 0; i--)
      {
          for(j = 0; j < 4096; j++)
          {
        	  x_asm_test[j*2+8192*i] =  x_fun[which_array][j]*win_4096_data[j+4096*i];
          }
          if(which_array == 0)
          {
              which_array = 10;
          }else
          {
              which_array--;
          }
      }
//     for(i = 0; i < 16384; i++)
//     {
//    	 x_asm_test[2*i] =  temp_wave[i]*win_4096_data[i];
//     }
    for(k = 0; k < fft_n; k++)
    {
//        x_asm_test[2 * k] = input_buf[k];
        x_asm_test[2 * k + 1] =0.0;
    }
    DSPF_sp_cfftr2_dit_cn(x_asm_test, Wx, fft_n);
//     DSPF_sp_cfftr2_dit(x_asm_test, Wx, fft_n);
     bit_rev(x_asm_test, fft_n);

}
/* generate real and imaginary twiddle table of size fft_number/2 complex numbers */
void gen_w_r2(float w[], int N)
{
    int i;
    float e = pi * 2.0 / N;

    for(i = 0; i < (N >> 1); i++)
    {
        w[2 * i] = cos(i * e);
        w[2 * i + 1] = sin(i * e);
    }
}

/* bit reverse coefficients */
void bit_rev(float x[], int N)
{
    int i, j, k;
    float rtemp, itemp;
    j = 0;

    for(i=1; i < (N-1); i++)
    {
        k = N >> 1;

        while(k <= j)
        {
            j -= k;
            k >>= 1;
        }

        j += k;

        if(i < j)
        {
            rtemp = x[j*2];
            x[j*2] = x[i*2];
            x[i*2] = rtemp;
            itemp = x[j*2+1];
            x[j*2+1] = x[i*2+1];
            x[i*2+1] = itemp;
        }
    }
}



float x_fun_isofilwm_02[Wm_number];
float x_fun_isofilwm_03[Wm_number];
void fun_isofilwm(float x[],int fs)
{

    float middle_value = 0.0;
    float middle_value_1 = 0.0;
    float middle_value_2 = 0.0;
    int i;
    //系数
    /////fs=1024
      float a1[3] = {1,-1.993107239334227,0.993130912960814};
      float b1[3] = {0.99655953807376,-1.99311907614752,0.99655953807376};
      float a2[3] = {1,-1.162037077179318,0.421304626121855};
      float b2[3] = {0.064816887235634,0.129633774471268,0.064816887235634};
      float a3[2] = {1,-0.965720553955848};
      float b3[2] = {0.017139723022076,0.017139723022076};

     for(i = 0; i < Wm_number ; i++)
     {
         switch(i)
         {
         case 0:
             middle_value = b2[0]*x[i];
             Out_01[i] = middle_value;
             break;
         case 1:
             middle_value = b2[0]*x[i]+b2[1]*x[i-1] -a2[1]*Out_01[i-1];
             Out_01[i] = middle_value;
             break;
         default:
             middle_value =  b2[0]*x[i]+b2[1]*x[i-1] +b2[2]*x[i-2] -a2[1]*Out_01[i-1] - a2[2]*Out_01[i-2];
             Out_01[i] = middle_value;
             break;
         }
          middle_value = 0.0;
      }

      for(i = 0; i < Wm_number ; i++)
      {
          switch(i)
          {
          case 0:
        	  x_fun_isofilwm_02[i] = Out_01[i];
              middle_value_1 = b1[0]*x_fun_isofilwm_02[i];
              Out_1_01[i] = middle_value_1;
              break;
          case 1:
        	  x_fun_isofilwm_02[i] = Out_01[i];
              middle_value_1 = b1[0]*x_fun_isofilwm_02[i]+b1[1]*x_fun_isofilwm_02[i-1] -a1[1]*Out_1_01[i-1];
              Out_1_01[i] = middle_value_1;
              break;
          default:
        	  x_fun_isofilwm_02[i] = Out_01[i];
              middle_value_1 =b1[0]*x_fun_isofilwm_02[i]+b1[1]*x_fun_isofilwm_02[i-1] +b1[2]*x_fun_isofilwm_02[i-2] -a1[1]*Out_1_01[i-1] - a1[2]*Out_1_01[i-2];
              Out_1_01[i] = middle_value_1;
              break;
          }
           middle_value_1 = 0.0;
     }
     for(i = 0; i < Wm_number ; i++)
     {
         switch(i)
         {
         case 0:
        	 x_fun_isofilwm_03[i] = Out_1_01[i];
             middle_value_2 = b3[0]*x_fun_isofilwm_03[i];
             Out_2_01[i] = middle_value_2;
             break;
         case 1:
        	 x_fun_isofilwm_03[i] = Out_1_01[i];
             middle_value_2 = b3[0]*x_fun_isofilwm_03[i]+b3[1]*x_fun_isofilwm_03[i-1] -a3[1]*Out_2_01[i-1];
             Out_2_01[i] = middle_value_2;
             break;
         default:
        	 x_fun_isofilwm_03[i] = Out_1_01[i];
             middle_value_2 = b3[0]*x_fun_isofilwm_03[i]+b3[1]*x_fun_isofilwm_03[i-1] -a3[1]*Out_2_01[i-1];
             Out_2_01[i] = middle_value_2;
             break;
         }
         middle_value_2 = 0.0;

     }

}

float x_fun_isofilwa_02[WA_number];
float x_fun_isofilwa_03[WA_number];
float x_fun_isofilwa_04[WA_number];
void fun_isofilwa(float x[11][65536],int fs,int which_array)
{



    float middle_value = 0.0;
    float middle_value_1 = 0.0;
    float middle_value_2 = 0.0;
    float middle_value_3 = 0.0;

    unsigned int i;
    //fs = 65536
    float wa_a1[3] = {1,-1.996053896249171,0.996057789182874};
    float wa_b1[3] = {0.998027921358011,-1.996055842716022,0.998027921358011};
    float wa_a2[3] = {1,-0.524387868248578,0.068745659091572};
    float wa_b2[3] = {0.136089447710748,0.272178895421497,0.136089447710748};
    float wa_a3[2] = {1,-0.989727427102905};
    float wa_b3[2] = {0.994863713551452,-0.994863713551453};
    float wa_a4[2] = {1,-0.931671676603124};
    float wa_b4[2] = {0.9658358383015,-0.9658358383015};

    //fs=51200
//    float a1[3] = {1,-1.994950382041083,0.994956756701466};
//    float b1[3] = {0.997476784685637,-1.994953569371275,0.997476784685637};
//    float a2[3] = {1,-0.288048789609494,0.020743026298874};
//    float b2[3] = {0.183173559172345,0.36634711834469,0.183173559172345};
//    float a3[2] = {1,-0.986869989749959};
//    float b3[2] = {0.993434994874979,-0.993434994874980};
//    float a4[2] = {1,-0.993434994874980};
//    float b4[2] = {0.95668423017602,-0.95668423017602};
    //fs = 32768
//    float a1[3] = {1,-1.992115570692357,0.992131111748733};
//    float b1[3] = {0.996061670610272,-1.992123341220545,0.996061670610272};
//    float a2[3] = {1,0.155904539979802,0.006076556396578};
//    float b2[3] = {0.290495274094095,0.58099054818819,0.290495274094095};
//    float a3[2] = {1,-0.979559840718931};
//    float b3[2] = {0.989779920359466,-0.989779920359466};
//    float a4[2] = {1,-0.867857878028458};
//    float b4[2] = {0.933928939014229,-0.933928939014229};
     for(i = 0; i < WA_number; i++)
     {
         switch(i)
         {
         case 0:
             middle_value = wa_b2[0]*x[which_array][i];
             IIR_Out_01[i] = middle_value;
             break;
         case 1:
             middle_value = wa_b2[0]*x[which_array][i]+wa_b2[1]*x[which_array][i-1] -wa_a2[1]*IIR_Out_01[i-1];
             IIR_Out_01[i] = middle_value;
             break;
         default:
             middle_value = wa_b2[0]*x[which_array][i]+wa_b2[1]*x[which_array][i-1] +wa_b2[2]*x[which_array][i-2] -wa_a2[1]*IIR_Out_01[i-1] - wa_a2[2]*IIR_Out_01[i-2];
             IIR_Out_01[i] = middle_value;
             break;
         }
         middle_value = 0.0;
     }
     for(i = 0; i < WA_number; i++)
     {
         switch(i)
         {
         case 0:
        	 x_fun_isofilwa_02[i] = IIR_Out_01[i];
             middle_value_1 = wa_b1[0]*x_fun_isofilwa_02[i];
             IIR_Out_1_01[i] = middle_value_1;
             break;
         case 1:
        	 x_fun_isofilwa_02[i] = IIR_Out_01[i];
             middle_value_1 = wa_b1[0]*x_fun_isofilwa_02[i]+wa_b1[1]*x_fun_isofilwa_02[i-1] -wa_a1[1]*IIR_Out_1_01[i-1];
             IIR_Out_1_01[i] = middle_value_1;
             break;
         default:
        	 x_fun_isofilwa_02[i] = IIR_Out_01[i];
             middle_value_1 = wa_b1[0]*x_fun_isofilwa_02[i]+wa_b1[1]*x_fun_isofilwa_02[i-1] +wa_b1[2]*x_fun_isofilwa_02[i-2] -wa_a1[1]*IIR_Out_1_01[i-1] - wa_a1[2]*IIR_Out_1_01[i-2];
             IIR_Out_1_01[i] = middle_value_1;
             break;
         }
          middle_value_1 = 0.0;
     }
     for(i = 0; i < WA_number; i++)
     {
         switch(i)
         {
         case 0:
        	 x_fun_isofilwa_03[i] = IIR_Out_1_01[i];
             middle_value_2 = wa_b3[0]*x_fun_isofilwa_03[i];
             IIR_Out_2_01[i] = middle_value_2;
             break;
         case 1:
        	 x_fun_isofilwa_03[i] = IIR_Out_1_01[i];
             middle_value_2 = wa_b3[0]*x_fun_isofilwa_03[i]+wa_b3[1]*x_fun_isofilwa_03[i-1] -wa_a3[1]*IIR_Out_2_01[i-1];
             IIR_Out_2_01[i] = middle_value_2;
             break;
         default:
        	 x_fun_isofilwa_03[i] = IIR_Out_1_01[i];
             middle_value_2 = wa_b3[0]*x_fun_isofilwa_03[i]+wa_b3[1]*x_fun_isofilwa_03[i-1] -wa_a3[1]*IIR_Out_2_01[i-1];
             IIR_Out_2_01[i] = middle_value_2;
             break;

         }

          middle_value_2 = 0.0;
     }    //计算到这里还是正确的
     for(i = 0; i < WA_number; i++)
     {
         middle_value_3 = 0.0;
         switch(i)
         {
         case 0:
        	 x_fun_isofilwa_04[i] = IIR_Out_2_01[i];
             middle_value_3 = wa_b4[0]*x_fun_isofilwa_04[i];
             IIR_Out_3_01[i] = middle_value_3;
             break;
         case 1:
        	 x_fun_isofilwa_04[i] = IIR_Out_2_01[i];
             middle_value_3 = wa_b4[0]*x_fun_isofilwa_04[i]+wa_b4[1]*x_fun_isofilwa_04[i-1] -wa_a4[1]*IIR_Out_3_01[i-1];
             IIR_Out_3_01[i] = middle_value_3;
             break;
         default:
        	 x_fun_isofilwa_04[i] = IIR_Out_2_01[i];
             middle_value_3 = wa_b4[0]*x_fun_isofilwa_04[i]+wa_b4[1]*x_fun_isofilwa_04[i-1] -wa_a4[1]*IIR_Out_3_01[i-1];
             IIR_Out_3_01[i] = middle_value_3;
             break;
         }


     }
     for(i = 0; i < WA_number; i++)
     {
         IIR_Out_3_01[i] = IIR_Out_3_01[i]*1.258925411;
     }

}


void fun_freq_band(int nc[],int n)
{
}

void DSPF_sp_cfftr2_dit_cn(float* x, float* w, unsigned short n)
{
    unsigned short n2, ie, ia, i, j, k, m;
    float rtemp, itemp, c, s;

    n2 = n;
    ie = 1;

    for(k=n; k > 1; k >>= 1)
    {
        n2 >>= 1;
        ia = 0;

        for(j=0; j < ie; j++)
        {
            c = w[2*j];
            s = w[2*j+1];

            for(i=0; i < n2; i++)
            {
                m = ia + n2;
                rtemp = c * x[2*m] + s * x[2*m+1];
                itemp = c * x[2*m+1] - s * x[2*m];
                x[2*m] = x[2*ia] - rtemp;
                x[2*m+1] = x[2*ia+1] - itemp;
                x[2*ia] = x[2*ia] + rtemp;
                x[2*ia+1] = x[2*ia+1] + itemp;
                ia++;
            }

            ia += n2;
        }
        ie <<= 1;
    }
}


//#pragma DATA_ALIGN(nomi_fc, 8)
//float nomi_fc[44];
void fun_freq_band_function_1(int n_fc_n_case4,int index,int fs)
{
	float f;
	float f_lim[44];
	 float ID_fc[44]={0.0};
    int i;
    float df;
    if(sampling_mudole_channel_01.channel_property == 0x01)
    {
    	df = (float)fs/fftnumber_Mechanical_vibration;
    }
    else if(sampling_mudole_channel_01.channel_property == 0x02)
    {
    	df = (float)fs/fftnumber_Platform_vibration;
    }
    else if(sampling_mudole_channel_01.channel_property == 0x03)
    {
    	df = (float)fs/fftnumber_Air_noise;
    }
    else
    {
    	df = (float)fs/fftnumber_Equipment_vibration;
    }
    for(i = 0; i < n_fc_n_case4; i++)
     {
     	ID_fc[i] = i+index;
     }
    for(i = 0; i < n_fc_n_case4; i++)
    {
    	f_lim[i] = powsp(10,(ID_fc[i]*0.1)-0.05);
    }
    f_lim[n_fc_n_case4] = powsp(10,(ID_fc[n_fc_n_case4-1]*0.1)+0.05);

    for(f = 0; f < fs/2; f+=df)
    {
    	if(f > f_lim[0])
    	{
    		ID_f_lim_case_1[0] = f/df + 1;
    		break;
    	}
    }
    for(i = 1; i <= n_fc_n_case4_01; i++)
    {
        for(f = 0; f < fs/2; f+=df)
        {
        	if(f >= f_lim[i])
        	{
        		ID_f_lim_case_1[i] = f/df + 1;
        		break;
        	}
        }
    }

}
void fun_freq_band_function_2(int n_fc_n_case4,int index,int fs)
{
	float f;
	float f_lim[44];
	 float ID_fc[44]={0.0};
    int i;
    float df;
    if(sampling_mudole_channel_02.channel_property == 0x01)
    {
    	df = (float)fs/fftnumber_Mechanical_vibration;
    }
    else if(sampling_mudole_channel_02.channel_property == 0x02)
    {
    	df = (float)fs/fftnumber_Platform_vibration;
    }
    else if(sampling_mudole_channel_02.channel_property == 0x03)
    {
    	df = (float)fs/fftnumber_Air_noise;
    }
    else
    {
    	df = (float)fs/fftnumber_Equipment_vibration;
    }
    for(i = 0; i < n_fc_n_case4; i++)
     {
     	ID_fc[i] = i+index;
     }
    for(i = 0; i < n_fc_n_case4; i++)
    {
    	f_lim[i] = powsp(10,(ID_fc[i]*0.1)-0.05);
    }
    f_lim[n_fc_n_case4] = powsp(10,(ID_fc[n_fc_n_case4-1]*0.1)+0.05);

    for(f = 0; f < fs/2; f+=df)
    {
    	if(f > f_lim[0])
    	{
    		ID_f_lim_case_2[0] = f/df + 1;
    		break;
    	}
    }
    for(i = 1; i <= n_fc_n_case4_02; i++)
    {
        for(f = 0; f < fs/2; f+=df)
        {
        	if(f >= f_lim[i])
        	{
        		ID_f_lim_case_2[i] = f/df + 1;
        		break;
        	}
        }
    }

}
void fun_freq_band_function_3(int n_fc_n_case4,int index,int fs)
{
	float f;
	float f_lim[44]={0.0};
	 float ID_fc[44]={0.0};
    int i;
    float df;
    if(sampling_mudole_channel_03.channel_property == 0x01)
    {
    	df = (float)fs/fftnumber_Mechanical_vibration;
    }
    else if(sampling_mudole_channel_03.channel_property == 0x02)
    {
    	df = (float)fs/fftnumber_Platform_vibration;
    }
    else if(sampling_mudole_channel_03.channel_property == 0x03)
    {
    	df = (float)fs/fftnumber_Air_noise;
    }
    else
    {
    	df = (float)fs/fftnumber_Equipment_vibration;
    }
    for(i = 0; i < n_fc_n_case4; i++)
     {
     	ID_fc[i] = i+index;
     }
    for(i = 0; i < n_fc_n_case4; i++)
    {
    	f_lim[i] = powsp(10,(ID_fc[i]*0.1)-0.05);
    }
    f_lim[n_fc_n_case4] = powsp(10,(ID_fc[n_fc_n_case4-1]*0.1)+0.05);

    for(f = 0; f < fs/2; f+=df)
    {
    	if(f > f_lim[0])
    	{
    		ID_f_lim_case_3[0] = f/df + 1;
    		break;
    	}
    }
    for(i = 1; i <= n_fc_n_case4_03; i++)
    {
        for(f = 0; f < fs/2; f+=df)
        {
        	if(f >= f_lim[i])
        	{
        		ID_f_lim_case_3[i] = f/df + 1;
        		break;
        	}
        }
    }

}
void fun_freq_band_function_4(int n_fc_n_case4,int index,int fs)
{
	float f;
	float f_lim[44]={0.0};
	 float ID_fc[44]={0.0};
    int i;
    float df;
    if(sampling_mudole_channel_04.channel_property == 0x01)
    {
    	df = (float)fs/fftnumber_Mechanical_vibration;
    }
    else if(sampling_mudole_channel_04.channel_property == 0x02)
    {
    	df = (float)fs/fftnumber_Platform_vibration;
    }
    else if(sampling_mudole_channel_04.channel_property == 0x03)
    {
    	df = (float)fs/fftnumber_Air_noise;
    }
    else
    {
    	df = (float)fs/fftnumber_Equipment_vibration;
    }


    for(i = 0; i < n_fc_n_case4; i++)
     {
     	ID_fc[i] = i+index;
     }

//    int pow_fc,mod_fc;
//    float fc0[10] = {1,1.25,1.6,2,2.5,3.15,4,5,6.3,8};
    for(i = 0; i < n_fc_n_case4; i++)
    {
    	f_lim[i] = powsp(10,(ID_fc[i]*0.1)-0.05);
    }
    f_lim[n_fc_n_case4] = powsp(10,(ID_fc[n_fc_n_case4-1]*0.1)+0.05);

    for(f = 0; f < fs/2; f+=df)
    {
    	if(f > f_lim[0])
    	{
    		ID_f_lim_case_4[0] = f/df + 1;
    		break;
    	}
    }
    for(i = 1; i <= n_fc_n_case4_04; i++)
    {
        for(f = 0; f < fs/2; f+=df)
        {
        	if(f >= f_lim[i])
        	{
        		ID_f_lim_case_4[i] = f/df + 1;
        		break;
        	}
        }
    }

}
void fun_freq_band_1(int fs)
{
	float f_lim_1[44];
    float ID_fc[44]={0.0};
    int nc = 30;
    int i,f;
    float df;
    for(i = 0; i <= 30; i++)
    {
    	ID_fc[i] = i+10;
    }

    for(i = 0; i <= nc; i++)
    {
    	f_lim_1[i] = powsp(10,(ID_fc[i]*0.1)-0.05);
    }

    f_lim_1[nc+1] = powsp(10,(ID_fc[nc]*0.1)+0.05);

    df=fs/fftnumber_Mechanical_vibration;

    for(f = 0; f < fs/2; f+=df)
    {
    	if(f > f_lim_1[0])
    	{
    		ID_f_lim_1[0] = f/df + 1;
    		break;
    	}
    }
    for(i = 1; i <= nc+1; i++)
    {
        for(f = 0; f < fs/2; f+=df)
        {
        	if(f >= f_lim_1[i])
        	{
        		ID_f_lim_1[i] = f/df + 1;
        		break;
        	}
        }
    }
}

void fun_freq_band_2(int fs)
{
	float f_lim_2[44];
    float ID_fc[44]={0.0};
    int nc = 19;
    int i;
    float f;
    float df;
    for(i = 0; i <= 19; i++)
    {
    	ID_fc[i] = i;
    }

    for(i = 0; i <= nc; i++)
    {
    	f_lim_2[i] = powsp(10,(ID_fc[i]*0.1)-0.05);
    }

    f_lim_2[nc+1] = powsp(10,(ID_fc[nc]*0.1)+0.05);

    df=(float)fs/fftnumber_Platform_vibration;

    for(f = 0; f < fs/2; f+=df)
    {
    	if(f > f_lim_2[0])
    	{
    		ID_f_lim_2[0] = f/df + 1;
    		break;
    	}
    }
    for(i = 1; i <= nc+1; i++)
    {
        for(f = 0; f < fs/2; f+=df)
        {
        	if(f >= f_lim_2[i])
        	{
        		ID_f_lim_2[i] = f/df + 1;
        		break;
        	}
        }
    }
}

void fun_freq_band_3(int fs)
{
	float f_lim_3[44];
    float ID_fc[44]={0.0};
    int nc = 30;
    int i;
    float f;
    float df;
//    float fc0[10] = {1,1.25,1.6,2,2.5,3.15,4,5,6.3,8};
    for(i = 0; i <= 30; i++)
    {
    	ID_fc[i] = i+13;
    }

    for(i = 0; i <= nc; i++)
    {
    	f_lim_3[i] = powsp(10,(ID_fc[i]*0.1)-0.05);
    }

    f_lim_3[nc+1] = powsp(10,(ID_fc[nc]*0.1)+0.05);

    df=(float)fs/fftnumber_Air_noise;

    for(f = 0; f < fs/2; f+=df)
    {
    	if(f > f_lim_3[0])
    	{
    		ID_f_lim_3[0] = f/df + 1;
    		break;
    	}
    }
    for(i = 1; i <= nc+1; i++)
    {
        for(f = 0; f < fs/2; f+=df)
        {
        	if(f >= f_lim_3[i])
        	{
        		ID_f_lim_3[i] = f/df + 1;
        		break;
        	}
        }
    }
}
