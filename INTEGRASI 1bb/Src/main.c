/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "MY_ILI9341.h"
#include "TSC2046.h"
#include <stdbool.h>
#include "arm_math.h"

#define SAMPLE 128
//#define SAMPLE 2048
#define FFT_SIZE SAMPLE/2
#define BLOCKSIZE 64
//#define BLOCKSIZE 1000
//#define BLOCKSIZE 1024
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint32_t	blockSize = BLOCKSIZE, i = 0, pIndexV, pIndexI, SpecIndex, exti = 0, a=0,b=0,pp=0, dis=1,oe;
uint32_t beb1=0, beb2=0, beb3=0;
float32_t	pResultVrms, pResultIrms, pResultVmax, pResultImax, pSrcV[BLOCKSIZE+1], pSrcI[BLOCKSIZE+1], Vrmsrms,Vrmsrmsfft, Vrmsmax, Irmsrms,Irmsrmsfft, Irmsmax;
float32_t Irms_fft[FFT_SIZE],Vfft[SAMPLE],Ifft[SAMPLE],OutputVfft[FFT_SIZE],OutputIfft[FFT_SIZE],VrmsFFT, IrmsFFT, Vrms_fft[FFT_SIZE],I_CAL[FFT_SIZE];
float32_t offset,THDv[7],THDi[7],P[7],Q[7],S, Spec[15],Specm[15], Speci[15];
float32_t SpecMin,SpecMax,IPOWER[SAMPLE],VPOWER[SAMPLE],Wh,Current,Power,watthour[SAMPLE];
uint32_t SpecIndex2, rc=0, rca=0;
char rx_buffer[50], tx_buffer[50],textV[50],textV1[50],textV2[50],textV3[50],textV4[50],textI[50],textI1[50],textI2[50],textI3[50],textI4[50];
bool led_state=false;
char Teg[100],Ars[100],Pow[100],Eng[100];
double LAYER1[10],LAYER2[2],LOGSIG[10],TANSIG[2],TampilV[FFT_SIZE],TampilI[FFT_SIZE];
double nrms,nh1,nh3,nh5,nh7,nh9,denom;

arm_cfft_radix2_instance_f32 Radix2;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void SetWaktu();
void StartingDisplay();
void TestLCD();
void HarmonicCalculation();
void RecordData();
void Home_Display();
void Spectrum_Display();
void Load_Display();
void Spectrum_Calculation();
void Load_Conditioning();
void Load_Conditioning2();
void ANN();
void ANN2();
void ANN3();
void Menu();

/* USER CODE END PD */
TS_TOUCH_DATA_Def wazz;
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

//*Beban PC*/ 2
//double CHECK[6]={0,0,0,0,0
//};
//*Beban PC*/ 2
//double CHECK[6]={0.0441,
//0.0327,
//0.026,
//0.0291,
//0.0061
//};
//*Beban Kipas*/ 3
//double CHECK[6]={0.2098,
//0,
//0,
//0,
//0
//};
//*Beban LHE*/ 4
//double CHECK[6]={0.2304,
//0.177,
//0.1065,
//0.0668,
//0.0539
//};
//*Beban PC & Kipas*/ 5
//double CHECK[6]={0.2537,
//0.0262,
//0.014,
//0.0196,
//0.0089
//};
//*Beban LHE & PC*/ 6
//double CHECK[6]={0.2428,
//0.1584,
//0.0428,
//0.0222,
//0.0449
//};
//*Beban Kipas & LHE*/ 7
//double CHECK[6]={0.4256,
//0.1353,
//0.0869,
//0.066,
//0.0272
//};
//*Beban All In*/ 8
//double CHECK[6]={0.4903,
//0.1838,
//0.043,
//0.0351,
//0.0848
//};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
	HAL_ADC_Start_IT(&hadc3);
	ILI9341_Init (&hspi1, GPIOE,GPIO_PIN_10,GPIOE,GPIO_PIN_12,GPIOE,GPIO_PIN_11);
	TSC2046_Begin (&hspi2 , GPIOD, GPIO_PIN_8);
  /* USER CODE END 2 */
	//TSC2046_Calibrate();
	ILI9341_Fill (COLOR_DGRAY);
	ILI9341_setRotation (4);
	TS_TOUCH_DATA_Def wazz;
	/* USER CODE END 2 */
	StartingDisplay();
	watthour[12]=0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		HAL_UART_Receive (&huart3 , (uint8_t *)rx_buffer,50,100);
		SetWaktu();
		//HAL_Delay(200);
    HarmonicCalculation();
		//TestLCD();
		Menu();
		RecordData();
		//Load_Display();

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void HarmonicCalculation(void)
{
		//----------Normalozation Harmonic Voltage-----------//		
		for(a=0;a<FFT_SIZE;a++)
		{
		TampilV[a]=(((OutputVfft[a]/(sqrt(2)*(FFT_SIZE/2)))-(float32_t)0.10084)/(float32_t)4.5739); 
		}
		
//----------Normalozation Harmonic Current-----------//		
		for(b=0;b<FFT_SIZE;b++)
		{
		I_CAL[b]=OutputIfft[b]/((FFT_SIZE/2)*sqrt(2));
		TampilI[b]=(((I_CAL[b])-(float32_t)4.864126475)/(float32_t)253.7943361);
//		TampilI[b]=((OutputIfft[b]-(float32_t)295.329817968466)/(float32_t)11361.9650245603);
		}
		
		//ANN();
		ANN2();
		//ANN3();
		Spectrum_Calculation();

		watthour[12]=watthour[12]+P[4];
		Power=P[4];
		Current=Irms_fft[10];
		if(SpecMax<=0)
		{
		watthour[12]=0;
		Power=0;
		Current=0;
		}	
			
		Wh=watthour[12]/3600;

}
void StartingDisplay(void)
{
	ILI9341_setRotation (4);
	ILI9341_Fill (COLOR_BLACK);
//ILI9341_printText(char text[], int16_t x, int16_t y, uint16_t color, uint16_t bg, uint8_t size)
	ILI9341_printText("Wazz Electronics", 20,100, COLOR_WHITE, COLOR_BLACK, 3);
	HAL_Delay(1500);
	ILI9341_Fill (COLOR_BLACK);
	ILI9341_printText("Smart  Energy", 50,90, COLOR_WHITE, COLOR_BLACK, 3);
	ILI9341_printText("    Meter    ", 50,130, COLOR_WHITE, COLOR_BLACK, 3);
	HAL_Delay(1500);
	ILI9341_Fill(COLOR_BLACK);
}
void SetWaktu(void)
{
		if(sDate.Date==31 && sDate.Month==12 && sTime.Hours==25)
		{
		sTime.Hours=0;
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		sDate.Date=1;
		sDate.Month=RTC_MONTH_JANUARY;
		sDate.WeekDay=sDate.WeekDay+1;
		sDate.Year=sDate.Year+1;
		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		}
	//	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	//	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}
void RecordData(void)
{

	
		if(rx_buffer[0]=='R')
		{
		rc=1;
		rx_buffer[0]=0;
		}
		
		if(rca==101)
		{
		rc=0;
		rca=0;
		sprintf(tx_buffer, "HOOOOOOPPPPP\n");
		HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, strlen(tx_buffer), 1*strlen(tx_buffer));
		}
		
		if(rc==1){
		sprintf(tx_buffer, "*----KIPAS----%d*\n",rca);
		HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, strlen(tx_buffer), 1*strlen(tx_buffer));
		sprintf(textI, "%4.4f\n",Irms_fft[10]);
		HAL_UART_Transmit(&huart3, (uint8_t*)textI, strlen(textI), 1*strlen(textI));
		sprintf(textI, "%4.4f\n",TampilI[1]);
		HAL_UART_Transmit(&huart3, (uint8_t*)textI, strlen(textI), 1*strlen(textI));
		sprintf(textI1, "%4.4f\n",TampilI[3]);
		HAL_UART_Transmit(&huart3, (uint8_t*)textI1, strlen(textI1), 1*strlen(textI1));
		sprintf(textI2, "%4.4f\n",TampilI[5]);
		HAL_UART_Transmit(&huart3, (uint8_t*)textI2, strlen(textI2), 1*strlen(textI2));
		sprintf(textI3, "%4.4f\n",TampilI[7]);
		HAL_UART_Transmit(&huart3, (uint8_t*)textI3, strlen(textI3), 1*strlen(textI3));
		sprintf(textI4, "%4.4f\n",TampilI[9]);
		HAL_UART_Transmit(&huart3, (uint8_t*)textI4, strlen(textI4), 1*strlen(textI4));
		rca++;
		}
}

void TestLCD(void)
{
		HAL_UART_Receive (&huart3 , (uint8_t *)rx_buffer,50,500);
    /* USER CODE BEGIN 3 */
  /* USER CODE BEGIN 3 */ //TES WARNA LCD
  if(rx_buffer[0] == 'B'){
	ILI9341_Fill (COLOR_BLUE);
	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"LCD IS BLUE\n"),500);
	
	sprintf(textV, "%4.2f\n",TampilV[1]);
	HAL_UART_Transmit(&huart3, (uint8_t*)textV, strlen(textV), 1*strlen(textV));		
	rx_buffer[0]=1;}  
	if(rx_buffer[0] == 'Y'){
	ILI9341_Fill (COLOR_YELLOW);
	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"LCD IS YELLOW\n"),500);	
		rx_buffer[0]=1;}
	if(rx_buffer[0] == 'W'){
	ILI9341_Fill (COLOR_WHITE);
	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"LCD IS WHITE\n"),500);	
	rx_buffer[0]=1;}
	if(rx_buffer[0] == 'H'){
	ILI9341_Fill (COLOR_BLACK);
	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"LCD IS BLACK\n"),500);	
	rx_buffer[0]=1;}
	if(rx_buffer[0] == 'O'){
	ILI9341_Fill (COLOR_ORANGE);
	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"LCD IS ORANGE\n"),500);	
	rx_buffer[0]=1;}
	if(rx_buffer[0] == 'R'){
	ILI9341_Fill (COLOR_RED);
	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"LCD IS RED\n"),500);	
	rx_buffer[0]=1;}
	if(rx_buffer[0] == 'G'){
	ILI9341_Fill (COLOR_GREEN);
	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"LCD IS GREEN\n"),500);	
	rx_buffer[0]=1;}
	if(rx_buffer[0] == 'M'){
	ILI9341_Fill (COLOR_MAGENTA);
	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"LCD IS MAGENTA\n"),500);	
	rx_buffer[0]=1;}
	if(rx_buffer[0] == 'm'){
	ILI9341_Fill (COLOR_MAROON );
	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"LCD IS MAROON\n"),500);	
	rx_buffer[0]=1;}
	if(rx_buffer[0] == 'b'){
	ILI9341_Fill (COLOR_BROWN);
	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"LCD IS BROWN\n"),500);	
	rx_buffer[0]=1;}
}
void Home_Display(void)
{
//ILI9341_setRotation (4);
	

ILI9341_printText("    GENERAL  INFORMATION     ", 0, 0, COLOR_WHITE, COLOR_RED, 2);
ILI9341_printText(" Voltage ", 20, 20, COLOR_RED, COLOR_WHITE, 2);
ILI9341_printText(" Current ", 20, 40, COLOR_RED, COLOR_WHITE, 2);
ILI9341_printText("  Power  ", 20, 60, COLOR_RED, COLOR_WHITE, 2);
ILI9341_printText("  Energy ", 20, 80, COLOR_RED, COLOR_WHITE, 2);

ILI9341_printText("     ODD HARMONIC ORDER      ", 0, 100, COLOR_WHITE, COLOR_RED, 2);
ILI9341_printText("   Voltage   ", 0, 120, COLOR_RED, COLOR_WHITE, 2);
ILI9341_printText("V[1]", 0, 140, COLOR_RED, COLOR_WHITE, 2);
ILI9341_printText("V[3]", 0, 160, COLOR_RED, COLOR_WHITE, 2);
ILI9341_printText("V[5]", 0, 180, COLOR_RED, COLOR_WHITE, 2);
ILI9341_printText("V[7]", 0, 200, COLOR_RED, COLOR_WHITE, 2);
ILI9341_printText("V[9]", 0, 220, COLOR_RED, COLOR_WHITE, 2);

ILI9341_printText("   Current   ", 160, 120, COLOR_RED, COLOR_WHITE, 2);
ILI9341_printText("I[1]", 160, 140, COLOR_RED, COLOR_WHITE, 2);
ILI9341_printText("I[3]", 160, 160, COLOR_RED, COLOR_WHITE, 2);
ILI9341_printText("I[5]", 160, 180, COLOR_RED, COLOR_WHITE, 2);
ILI9341_printText("I[7]", 160, 200, COLOR_RED, COLOR_WHITE, 2);
ILI9341_printText("I[9]", 160, 220, COLOR_RED, COLOR_WHITE, 2);

sprintf(Teg, "= %4.2f V   ",Vrms_fft[10]);
ILI9341_printText( Teg, 135, 20, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Ars, "= %4.2f A   ", Current);
ILI9341_printText( Ars, 135, 40, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Pow, "= %4.2f W   ", Power);
ILI9341_printText( Pow, 135, 60, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Eng, "= %4.2f W/h   ", Wh);
ILI9341_printText( Eng, 135, 80, COLOR_WHITE, COLOR_BLACK, 2);

sprintf(Teg, "= %4.2f ",TampilV[1]);
ILI9341_printText( Teg, 50, 140, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Ars, "= %2.4f ",Spec[1]);
ILI9341_printText( Ars, 210, 140, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Teg, "= %4.2f ",TampilV[3]);
ILI9341_printText( Teg, 50, 160, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Ars, "= %2.4f ",Spec[3]);
ILI9341_printText( Ars, 210, 160, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Teg, "= %4.2f ",TampilV[5]);
ILI9341_printText( Teg, 50, 180, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Ars, "= %2.4f ",Spec[5]);
ILI9341_printText( Ars, 210, 180, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Teg, "= %4.2f ",TampilV[7]);
ILI9341_printText( Teg, 50, 200, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Ars, "= %2.4f ",Spec[7]);
ILI9341_printText( Ars, 210, 200, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Teg, "= %4.2f ",TampilV[9]);
ILI9341_printText( Teg, 50, 220, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Ars, "= %2.4f ",Spec[9]);
ILI9341_printText( Ars, 210, 220, COLOR_WHITE, COLOR_BLACK, 2);
}


void Menu(void)
{
		wazz = TSC2046_GetTouchData();
		
		if(wazz.isPressed){			
			ILI9341_Fill (COLOR_BLACK); 
			dis=dis+1;
			if(dis==4)
				{dis=1;}}
		
		
		if(dis==1)
		{Home_Display();}
		if(dis==2)
		{Spectrum_Display();}
		if(dis==3)
		{Load_Display();}
		
		
}
void Spectrum_Calculation(void)
{

			for(a=2,THDi[0]=0;a<(FFT_SIZE/2)-1;a++){	
			if(TampilI[a]<0)
			{
				TampilI[a]=0;
			}
			Irms_fft[13]=(powf(TampilI[a],2.0000));
			THDi[0]=(float32_t)THDi[0]+(float32_t)Irms_fft[13];
			Irms_fft[13]=0;}
			THDi[1]=100*sqrt((THDi[0])/(THDi[0]+pow(TampilI[1],2)));
	
int sc,ss;
		oe=1;
for(sc=0;sc<14;sc++)
{
	Spec[sc]=TampilI[sc];
	if(Spec[sc]<=0)
		{Spec[sc]=0;}
}
	arm_max_f32((float32_t*)&Spec, 14, &SpecMax, &SpecIndex);
	arm_min_f32((float32_t*)&Spec, 14, &SpecMin, &SpecIndex);

int ii;
for(ii=0;ii<14;ii++)
	{Speci[ii]=217-((Spec[ii]/SpecMax)*118);}

}
void Spectrum_Display(void)
{
ILI9341_printText("     HARMONIC ANALYSIS     ", 0, 0, COLOR_WHITE, COLOR_BLUE, 2);
ILI9341_printText(" Current ", 20, 20, COLOR_BLUE, COLOR_WHITE, 2);
ILI9341_printText("   THD   ", 20, 40, COLOR_BLUE, COLOR_WHITE, 2);
ILI9341_printText("     SPECTRUM  HARMONIC      ", 0, 60, COLOR_WHITE, COLOR_BLUE, 2);
	
//ILI9341_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);

ILI9341_printText("MAXIMUM VALUE =", 10, 80, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText("MINIMUM VALUE =", 10, 90, COLOR_WHITE, COLOR_BLACK, 1);
	
ILI9341_printText("A", 5, 125, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText("M", 5, 135, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText("P", 5, 145, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText("E", 5, 155, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText("R", 5, 165, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText("E", 5, 175, COLOR_WHITE, COLOR_BLACK, 1);
	
ILI9341_printText(" Harmonic Order (n) ", 100, 230, COLOR_WHITE, COLOR_BLUE, 1);
	
ILI9341_printText(" 1 ", 20, 220, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText(" 2 ", 40, 220, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText(" 3 ", 60, 220, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText(" 4 ", 80, 220, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText(" 5 ",100, 220, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText(" 6 ",120, 220, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText(" 7 ",140, 220, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText(" 8 ",160, 220, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText(" 9 ",180, 220, COLOR_WHITE, COLOR_BLACK, 1);	
ILI9341_printText(" 10 ",200, 220, COLOR_WHITE, COLOR_BLACK, 1);	
ILI9341_printText(" 11 ",225, 220, COLOR_WHITE, COLOR_BLACK, 1);	
ILI9341_printText(" 12 ",250, 220, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText(" 13 ",275, 220, COLOR_WHITE, COLOR_BLACK, 1);

//Spectrum_Calculation();
if(SpecMax>0){
ILI9341_Fill_Rect(21,99,39,217,COLOR_BLACK);
ILI9341_Fill_Rect(25,Speci[1],35,218,COLOR_WHITE); //1
ILI9341_Fill_Rect(41,99,59,217,COLOR_BLACK);
ILI9341_Fill_Rect(45,Speci[2],55,218,COLOR_WHITE); //2
ILI9341_Fill_Rect(61,99,79,217,COLOR_BLACK);
ILI9341_Fill_Rect(65,Speci[3],75,218,COLOR_WHITE); //3
ILI9341_Fill_Rect(81,99,99,217,COLOR_BLACK);
ILI9341_Fill_Rect(85,Speci[4],95,218,COLOR_WHITE); //4
ILI9341_Fill_Rect(101,99,119,217,COLOR_BLACK);
ILI9341_Fill_Rect(105,Speci[5],115,218,COLOR_WHITE); //5
ILI9341_Fill_Rect(121,99,139,217,COLOR_BLACK);
ILI9341_Fill_Rect(125,Speci[6],135,218,COLOR_WHITE); //6
ILI9341_Fill_Rect(141,99,159,217,COLOR_BLACK);
ILI9341_Fill_Rect(145,Speci[7],155,218,COLOR_WHITE); //7
ILI9341_Fill_Rect(161,99,179,217,COLOR_BLACK);
ILI9341_Fill_Rect(165,Speci[8],175,218,COLOR_WHITE); //8
ILI9341_Fill_Rect(181,99,199,217,COLOR_BLACK);
ILI9341_Fill_Rect(185,Speci[9],195,218,COLOR_WHITE); //9
ILI9341_Fill_Rect(201,99,224,217,COLOR_BLACK);
ILI9341_Fill_Rect(205,Speci[10],220,218,COLOR_WHITE); //10
ILI9341_Fill_Rect(226,99,249,217,COLOR_BLACK);
ILI9341_Fill_Rect(230,Speci[11],245,218,COLOR_WHITE); //11
ILI9341_Fill_Rect(251,99,274,217,COLOR_BLACK);
ILI9341_Fill_Rect(255,Speci[12],270,218,COLOR_WHITE); //12
ILI9341_Fill_Rect(276,99,299,217,COLOR_BLACK);
ILI9341_Fill_Rect(280,Speci[13],295,218,COLOR_WHITE); //13
}
else
{
ILI9341_Fill_Rect(21,99,299,217,COLOR_BLACK);
}

ILI9341_drawLine(15,100,15,218, COLOR_WHITE);
ILI9341_drawLine(15,218,315,218, COLOR_WHITE);	
	
sprintf(Ars, "= %4.2f A   ",Current);
ILI9341_printText( Ars, 135, 20, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Ars, "= %4.2f persen   ", THDi[1]);
ILI9341_printText( Ars, 135, 40, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Ars, " %2.4f A   ",SpecMax);
ILI9341_printText( Ars, 100, 80, COLOR_WHITE, COLOR_BLACK, 1);
sprintf(Ars, " %2.4f A   ",SpecMin);
ILI9341_printText( Ars, 100, 90, COLOR_WHITE, COLOR_BLACK, 1);

}
void Load_Conditioning(void)
{
		
	/*
	if(rx_buffer[0]=='2')
	{
		beb1=beb1+1;
		
		if(beb1>1)
		{beb1=1;}
	rx_buffer[0]=0;
	rx_buffer[1]=0;
	}
	
	if(rx_buffer[0]=='1')
	{beb1=beb1-1;
		if(beb1<=0)
		{beb1=0;}
	rx_buffer[0]=0;
	rx_buffer[1]=0;
	}
	
	if(rx_buffer[0]=='4')
	{beb2=beb2+1;
		if(beb2>1)
		{beb2=1;}
	rx_buffer[0]=0;
	rx_buffer[1]=0;
	}
	
	if(rx_buffer[0]=='3')
	{beb2=beb2-1;
		if(beb2<<0)
		{beb2=0;}
	rx_buffer[0]=0;
	rx_buffer[1]=0;
	}
	
	if(rx_buffer[0]=='6')
	{beb3=beb3+1;
		if(beb3>1)
		{beb3=1;}
	rx_buffer[0]=0;
	rx_buffer[1]=0;
	}
	
	if(rx_buffer[0]=='5')
	{beb3=beb3-1;
		if(beb3<<0)
		{beb3=0;}
	rx_buffer[0]=0;
	rx_buffer[1]=0;
	}
	
	if(beb1==1)
	{
		ILI9341_printText(" OFF ", 150, 100, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 100, COLOR_WHITE, COLOR_BLUE, 3);
	}
	else
	{
	ILI9341_printText(" OFF ", 150, 100, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 100, COLOR_BLACK, COLOR_WHITE, 3);
	}
	
	if(beb2==1)
	{
	ILI9341_printText(" OFF ", 150, 150, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" ON ", 245, 150, COLOR_WHITE, COLOR_BLUE, 3);
	}
	else
	{
	ILI9341_printText(" OFF ", 150, 150, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 150, COLOR_BLACK, COLOR_WHITE, 3);	
	}
	
	if(beb3==1)
	{
	ILI9341_printText(" OFF ", 150, 200, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" ON ", 245, 200, COLOR_WHITE, COLOR_BLUE, 3);		
	}
	else
	{
	ILI9341_printText(" OFF ", 150, 200, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 200, COLOR_BLACK, COLOR_WHITE, 3);		
	}
	*/
	
	//if(denom>=0.8 && denom<=1.2 && Irms_fft[10]<=0.09)	// No Load //NEURON7[ISO]
	if(denom>0.8 && denom<1.2)
	{
		ILI9341_printText(" OFF ", 150, 100, COLOR_WHITE, COLOR_RED, 3);
		ILI9341_printText(" ON ", 245, 100, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" OFF ", 150, 200, COLOR_WHITE, COLOR_RED, 3);
		ILI9341_printText(" ON ", 245, 200, COLOR_BLACK, COLOR_WHITE, 3);	
		ILI9341_printText(" OFF ", 150, 150, COLOR_WHITE, COLOR_RED, 3);
		ILI9341_printText(" ON ", 245, 150, COLOR_BLACK, COLOR_WHITE, 3);	
	}

	if(denom>=1.8 && denom<=2.2) //KIPAS
	//if(Irms_fft[10]>=0.15 && Irms_fft[10]<=0.19 && TampilI[3]<=0.01)//NEURON7[ISO]
	{
	ILI9341_printText(" OFF ", 150, 100, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" ON ", 245, 100, COLOR_WHITE, COLOR_BLUE, 3);

	ILI9341_printText(" OFF ", 150, 150, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 150, COLOR_BLACK, COLOR_WHITE, 3);		
	ILI9341_printText(" OFF ", 150, 200, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 200, COLOR_BLACK, COLOR_WHITE, 3);			
	}
	
	//if(denom>=2.6 && denom<=2.9) //TV
	//if(Irms_fft[10]>0.19 && Irms_fft[10]<=0.22 && TampilI[3]>=0.07)//NEURON7[ISO]
	if(denom>2.8 && denom<3.2)
	{
	ILI9341_printText(" OFF ", 150, 150, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" ON ", 245, 150, COLOR_WHITE, COLOR_BLUE, 3);

	ILI9341_printText(" OFF ", 150, 100, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 100, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" OFF ", 150, 200, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 200, COLOR_BLACK, COLOR_WHITE, 3);		
	}
	
	//if(denom>=3.8 && denom<=4.2) //LHE
	//if(Irms_fft[10]>=0.24 && Irms_fft[10]<0.31 && denom>3)//NEURON7[ISO]
	if(denom>3.8 && denom<4.2)
	{
	ILI9341_printText(" OFF ", 150, 200, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" ON ", 245, 200, COLOR_WHITE, COLOR_BLUE, 3);		

	ILI9341_printText(" OFF ", 150, 100, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 100, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" OFF ", 150, 150, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 150, COLOR_BLACK, COLOR_WHITE, 3);	
	}	

//	if(denom>=4.8 && denom<=5.2) //TV & KIPAS
//if(Irms_fft[10]>=0.33 && Irms_fft[10]<=0.36)	//NEURON7[ISO]
	if(denom>4.8 && denom<5.2)
	{
		ILI9341_printText(" OFF ", 150, 100, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 100, COLOR_WHITE, COLOR_BLUE, 3);
		ILI9341_printText(" OFF ", 150, 150, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 150, COLOR_WHITE, COLOR_BLUE, 3);
	
		ILI9341_printText(" OFF ", 150, 200, COLOR_WHITE, COLOR_RED, 3);
		ILI9341_printText(" ON ", 245, 200, COLOR_BLACK, COLOR_WHITE, 3);		
	}
	
	//if(denom>=5.5 && denom<=5.8) //TV & LAMPU
	//if(denom>=5.5 && denom<=5.8 && Irms_fft[10]>=0.38 && Irms_fft[10]<=0.42 )//NEURON7[ISO]
	if(denom>5.8 && denom<6.2)
	{
	ILI9341_printText(" OFF ", 150, 150, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" ON ", 245, 150, COLOR_WHITE, COLOR_BLUE, 3);
	ILI9341_printText(" OFF ", 150, 200, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" ON ", 245, 200, COLOR_WHITE, COLOR_BLUE, 3);		
		
	ILI9341_printText(" OFF ", 150, 100, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 100, COLOR_BLACK, COLOR_WHITE, 3);
	}
	
	
	//if(denom>=6.8 && denom<=7.2) //KIPAS & LAMPU
//	if(Irms_fft[10]>=0.38 && Irms_fft[10]<=0.43 && denom>=6 && denom<=6.2)	//NEURON7[ISO]
	if(denom>6.8 && denom<7.2)
	{
		ILI9341_printText(" OFF ", 150, 100, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 100, COLOR_WHITE, COLOR_BLUE, 3);
		ILI9341_printText(" OFF ", 150, 200, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 200, COLOR_WHITE, COLOR_BLUE, 3);		
		
		ILI9341_printText(" OFF ", 150, 150, COLOR_WHITE, COLOR_RED, 3);
		ILI9341_printText(" ON ", 245, 150, COLOR_BLACK, COLOR_WHITE, 3);	
	}

//	if(denom>=7.8 && denom<=8.2) //TV & KIPAS & LAMPU
//	if(Irms_fft[10]>=0.53 && Irms_fft[10]<=0.57 )//NEURON7[ISO]
	if(denom>7.8 && denom<8.2)
	{
		ILI9341_printText(" OFF ", 150, 100, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 100, COLOR_WHITE, COLOR_BLUE, 3);
		ILI9341_printText(" OFF ", 150, 200, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 200, COLOR_WHITE, COLOR_BLUE, 3);	
		ILI9341_printText(" OFF ", 150, 150, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 150, COLOR_WHITE, COLOR_BLUE, 3);
	}

	
}


void Load_Conditioning2(void)
{
		
	/*
	if(rx_buffer[0]=='2')
	{
		beb1=beb1+1;
		
		if(beb1>1)
		{beb1=1;}
	rx_buffer[0]=0;
	rx_buffer[1]=0;
	}
	
	if(rx_buffer[0]=='1')
	{beb1=beb1-1;
		if(beb1<=0)
		{beb1=0;}
	rx_buffer[0]=0;
	rx_buffer[1]=0;
	}
	
	if(rx_buffer[0]=='4')
	{beb2=beb2+1;
		if(beb2>1)
		{beb2=1;}
	rx_buffer[0]=0;
	rx_buffer[1]=0;
	}
	
	if(rx_buffer[0]=='3')
	{beb2=beb2-1;
		if(beb2<<0)
		{beb2=0;}
	rx_buffer[0]=0;
	rx_buffer[1]=0;
	}
	
	if(rx_buffer[0]=='6')
	{beb3=beb3+1;
		if(beb3>1)
		{beb3=1;}
	rx_buffer[0]=0;
	rx_buffer[1]=0;
	}
	
	if(rx_buffer[0]=='5')
	{beb3=beb3-1;
		if(beb3<<0)
		{beb3=0;}
	rx_buffer[0]=0;
	rx_buffer[1]=0;
	}
	
	if(beb1==1)
	{
		ILI9341_printText(" OFF ", 150, 100, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 100, COLOR_WHITE, COLOR_BLUE, 3);
	}
	else
	{
	ILI9341_printText(" OFF ", 150, 100, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 100, COLOR_BLACK, COLOR_WHITE, 3);
	}
	
	if(beb2==1)
	{
	ILI9341_printText(" OFF ", 150, 150, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" ON ", 245, 150, COLOR_WHITE, COLOR_BLUE, 3);
	}
	else
	{
	ILI9341_printText(" OFF ", 150, 150, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 150, COLOR_BLACK, COLOR_WHITE, 3);	
	}
	
	if(beb3==1)
	{
	ILI9341_printText(" OFF ", 150, 200, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" ON ", 245, 200, COLOR_WHITE, COLOR_BLUE, 3);		
	}
	else
	{
	ILI9341_printText(" OFF ", 150, 200, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 200, COLOR_BLACK, COLOR_WHITE, 3);		
	}
	*/
	
	if(denom>0.8 && denom<1.2) //No Load
	{
		ILI9341_printText(" OFF ", 150, 100, COLOR_WHITE, COLOR_RED, 3);
		ILI9341_printText(" ON ", 245, 100, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" OFF ", 150, 200, COLOR_WHITE, COLOR_RED, 3);
		ILI9341_printText(" ON ", 245, 200, COLOR_BLACK, COLOR_WHITE, 3);	
		ILI9341_printText(" OFF ", 150, 150, COLOR_WHITE, COLOR_RED, 3);
		ILI9341_printText(" ON ", 245, 150, COLOR_BLACK, COLOR_WHITE, 3);	
	}

	if(denom>=1.8 && denom<=2.2) //PC
	{
	ILI9341_printText(" OFF ", 150, 100, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" ON ", 245, 100, COLOR_WHITE, COLOR_BLUE, 3);

	ILI9341_printText(" OFF ", 150, 150, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 150, COLOR_BLACK, COLOR_WHITE, 3);		
	ILI9341_printText(" OFF ", 150, 200, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 200, COLOR_BLACK, COLOR_WHITE, 3);			
	}
	
	//if(denom>=2.6 && denom<=2.9) //Kipas
	if(denom>2.8 && denom<3.2)
	{
	ILI9341_printText(" OFF ", 150, 150, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" ON ", 245, 150, COLOR_WHITE, COLOR_BLUE, 3);

	ILI9341_printText(" OFF ", 150, 100, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 100, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" OFF ", 150, 200, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 200, COLOR_BLACK, COLOR_WHITE, 3);		
	}
	
	//if(denom>=3.8 && denom<=4.2) //LHE
	if(denom>3.8 && denom<4.2)
	{
	ILI9341_printText(" OFF ", 150, 200, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" ON ", 245, 200, COLOR_WHITE, COLOR_BLUE, 3);		

	ILI9341_printText(" OFF ", 150, 100, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 100, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" OFF ", 150, 150, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 150, COLOR_BLACK, COLOR_WHITE, 3);	
	}	

//	if(denom>=4.8 && denom<=5.2) //Monitor & KIPAS
	if(denom>4.8 && denom<5.2)
	{
		ILI9341_printText(" OFF ", 150, 100, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 100, COLOR_WHITE, COLOR_BLUE, 3);
		ILI9341_printText(" OFF ", 150, 150, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 150, COLOR_WHITE, COLOR_BLUE, 3);
	
		ILI9341_printText(" OFF ", 150, 200, COLOR_WHITE, COLOR_RED, 3);
		ILI9341_printText(" ON ", 245, 200, COLOR_BLACK, COLOR_WHITE, 3);		
	}
	
	//if(denom>=5.5 && denom<=5.8) //PC & LAMPU
	if(denom>5.8 && denom<6.2)
	{
	ILI9341_printText(" OFF ", 150, 100, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" ON ", 245, 100, COLOR_WHITE, COLOR_BLUE, 3);
	
	ILI9341_printText(" OFF ", 150, 200, COLOR_BLACK, COLOR_WHITE, 3);
	ILI9341_printText(" ON ", 245, 200, COLOR_WHITE, COLOR_BLUE, 3);		
		
	ILI9341_printText(" OFF ", 150, 150, COLOR_WHITE, COLOR_RED, 3);
	ILI9341_printText(" ON ", 245, 150, COLOR_BLACK, COLOR_WHITE, 3);
	}
	
	
	//if(denom>=6.8 && denom<=7.2) //KIPAS & LAMPU
	if(denom>6.8 && denom<7.2)
	{
		ILI9341_printText(" OFF ", 150, 150, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 150, COLOR_WHITE, COLOR_BLUE, 3);
		ILI9341_printText(" OFF ", 150, 200, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 200, COLOR_WHITE, COLOR_BLUE, 3);		
		
		ILI9341_printText(" OFF ", 150, 100, COLOR_WHITE, COLOR_RED, 3);
		ILI9341_printText(" ON ", 245, 100, COLOR_BLACK, COLOR_WHITE, 3);	
	}

//	if(denom>=7.8 && denom<=8.2) //TV & KIPAS & LAMPU
//	if(Irms_fft[10]>=0.53 && Irms_fft[10]<=0.57 )//NEURON7[ISO]
	if(denom>7.8 && denom<8.2)
	{
		ILI9341_printText(" OFF ", 150, 100, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 100, COLOR_WHITE, COLOR_BLUE, 3);
		ILI9341_printText(" OFF ", 150, 200, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 200, COLOR_WHITE, COLOR_BLUE, 3);	
		ILI9341_printText(" OFF ", 150, 150, COLOR_BLACK, COLOR_WHITE, 3);
		ILI9341_printText(" ON ", 245, 150, COLOR_WHITE, COLOR_BLUE, 3);
	}

	
}
void ANN(void)
{
	
//--------Normalisasi Input ANN--------------//
	nrms=((2*(Irms_fft[10])/(0.5781))-1);
	nh1=((2*(TampilI[1])/(0.5096))-1);
	nh3=((2*(TampilI[3])/(0.2095))-1);
	nh5=((2*(TampilI[5])/(0.0889))-1);
	nh7=((2*(TampilI[7])/(0.0775))-1);
	nh9=((2*(TampilI[9])/(0.0639))-1);
	
			//--------------LAYER 1--------------// 8 Neuron
	LAYER1[0]=(nrms*14.2109) +(nh1*31.9712) +(nh3*-67.1798)+(nh5*-65.9171)+(nh7*-28.83)  +(nh9*4.019)   +(-50.3135);
	LAYER1[1]=(nrms*-37.62)  +(nh1*-57.1581)+(nh3*42.6421) +(nh5*-12.6324)+(nh7*-14.7227)+(nh9*8.937)   +(6.3582);
	LAYER1[2]=(nrms*-1.3751) +(nh1*4.7549)  +(nh3*44.2802) +(nh5*-13.708) +(nh7*-19.3646)+(nh9*-2.744)  +(-19.6014);
	LAYER1[3]=(nrms*-20.8764)+(nh1*-2.9745) +(nh3*-1.0278) +(nh5*-1.5516) +(nh7*-9.7267) +(nh9*-0.43725)+(9.2935);
	LAYER1[4]=(nrms*244.7532)+(nh1*201.8466)+(nh3*-30.4454)+(nh5*11.7708) +(nh7*3.9845)  +(nh9*-1.9483) +(-126.4218);
	LAYER1[5]=(nrms*-4.3426) +(nh1*138.101) +(nh3*-24.2581)+(nh5*-52.2111)+(nh7*-57.3235)+(nh9*-19.2858)+(-83.9288);
	LAYER1[6]=(nrms*11.3929) +(nh1*-33.22)  +(nh3*25.6788) +(nh5*5.1466)  +(nh7*3.6771)  +(nh9*6.4989)  +(-22.2309);
	LAYER1[7]=(nrms*-62.7838)+(nh1*-38.655) +(nh3*-78.1346)+(nh5*19.2386) +(nh7*12.0053) +(nh9*-15.6732) +(-41.3666);
	
	//---------Fungsi Aktivasi Logaritma Sigmoid-----------//
	LOGSIG[0]=(1/(1+exp(-LAYER1[0])));
	LOGSIG[1]=(1/(1+exp(-LAYER1[1])));
	LOGSIG[2]=(1/(1+exp(-LAYER1[2])));
	LOGSIG[3]=(1/(1+exp(-LAYER1[3])));
	LOGSIG[4]=(1/(1+exp(-LAYER1[4])));
	LOGSIG[5]=(1/(1+exp(-LAYER1[5])));
	LOGSIG[6]=(1/(1+exp(-LAYER1[6])));
	LOGSIG[7]=(1/(1+exp(-LAYER1[7])));

	//--------------LAYER 2--------------//
	LAYER2[0]=(LOGSIG[0]*-8.9663)+(LOGSIG[1]*-0.28768)+(LOGSIG[2]*0.60199)+(LOGSIG[3]*0.00000010254)+(LOGSIG[4]*0.75204)+(LOGSIG[5]*8.5285)+(LOGSIG[6]*-0.000000019187)+(LOGSIG[7]*-0.3143)+(0.14384);
	
	//---------Fungsi Aktivasi Tangensial Sigmoid-----------//
	TANSIG[0]=(2/(1+exp(-2*LAYER2[0]))-1);
	
		//-----DENORMALISASI----//
	denom=((1+TANSIG[0])*((80-10)/(2))+10)/10;
}
void ANN2(void)
{

	/*	//--------Normalisasi Input ANN--------------// SIMULATIONS
	nh1=((2*(CHECK[0])/(0.5268))-1);
	nh3=((2*(CHECK[1])/(0.2096))-1);
	nh5=((2*(CHECK[2])/(0.1324))-1);
	nh7=((2*(CHECK[3])/(0.0856))-1);
	nh9=((2*(CHECK[4])/(0.1071))-1);
	*/
	
	
	//--------Normalisasi Input ANN--------------//
	nh1=((2*(TampilI[1])/(0.5268))-1);
	nh3=((2*(TampilI[3])/(0.2096))-1);
	nh5=((2*(TampilI[5])/(0.1324))-1);
	nh7=((2*(TampilI[7])/(0.0856))-1);
	nh9=((2*(TampilI[9])/(0.1071))-1);
	
	
			//--------------LAYER 1--------------// 10 Neuron
	LAYER1[0]=(nh1*12.3863) +(nh3*7.7602) +(nh5*-0.57335)	+(nh7*0.52246)	+(nh9*-0.37055)	+(6.7037);
	LAYER1[1]=(nh1*41.9333) +(nh3*11.3274)+(nh5*-19.1131)	+(nh7*-12.2181)	+(nh9*14.2491)	+(-39.299);
	LAYER1[2]=(nh1*-4.0641) +(nh3*16.5956)+(nh5*3.2391)		+(nh7*2.2339)		+(nh9*2.3271)		+(5.3828);
	LAYER1[3]=(nh1*2.1)		  +(nh3*32.0598)+(nh5*-0.052186)+(nh7*0.094433)	+(nh9*-0.061798)+(36.9155);
	LAYER1[4]=(nh1*100.2047)+(nh3*-6.683) +(nh5*-52.0685)	+(nh7*-30.1504)	+(nh9*30.3397)	+(15.77);
	LAYER1[5]=(nh1*-16.387) +(nh3*-4.6686)+(nh5*1.8129)		+(nh7*0.41247)	+(nh9*0.66503)	+(-12.9243);
	LAYER1[6]=(nh1*-15.0967)+(nh3*-4.6677)+(nh5*1.5902)		+(nh7*0.29355)	+(nh9*0.61239)	+(-11.7967);
	LAYER1[7]=(nh1*9.3414)  +(nh3*5.8882) +(nh5*-0.38819)	+(nh7*0.41063)	+(nh9*-0.28754)	+(7.9378);
	LAYER1[8]=(nh1*15.4011) +(nh3*8.6509) +(nh5*50.9274)	+(nh7*25.8228)	+(nh9*-37.3325)	+(-10.4811);
	LAYER1[9]=(nh1*-3.2038) +(nh3*0.11375)+(nh5*-8.6354)	+(nh7*2.6579)		+(nh9*14.2764)	+(25.0084);
	
	//---------Fungsi Aktivasi Logaritma Sigmoid-----------//
	LOGSIG[0]=(1/(1+exp(-LAYER1[0])));
	LOGSIG[1]=(1/(1+exp(-LAYER1[1])));
	LOGSIG[2]=(1/(1+exp(-LAYER1[2])));
	LOGSIG[3]=(1/(1+exp(-LAYER1[3])));
	LOGSIG[4]=(1/(1+exp(-LAYER1[4])));
	LOGSIG[5]=(1/(1+exp(-LAYER1[5])));
	LOGSIG[6]=(1/(1+exp(-LAYER1[6])));
	LOGSIG[7]=(1/(1+exp(-LAYER1[7])));
	LOGSIG[8]=(1/(1+exp(-LAYER1[8])));
	LOGSIG[9]=(1/(1+exp(-LAYER1[9])));

	//--------------LAYER 2--------------//
	LAYER2[0]=(LOGSIG[0]*0.078693)+(LOGSIG[1]*13.1682)+(LOGSIG[2]*0.31564)+(LOGSIG[3]*73.9859)+(LOGSIG[4]*1.0398)+(LOGSIG[5]*1.6207)+(LOGSIG[6]*-2.3798)+(LOGSIG[7]*-0.83764)+(LOGSIG[8]*0.43784)+(LOGSIG[9]*-36.4738)+(-37.6504);
	
	//---------Fungsi Aktivasi Tangensial Sigmoid-----------//
	TANSIG[0]=(2/(1+exp(-2*LAYER2[0]))-1);
	
		//-----DENORMALISASI----//
	denom=((1+TANSIG[0])*((80-10)/(2))+10)/10;
}
void ANN3(void)
{

	/*	//--------Normalisasi Input ANN--------------// SIMULATIONS
	nh1=((2*(CHECK[0])/(0.5268))-1);
	nh3=((2*(CHECK[1])/(0.2096))-1);
	nh5=((2*(CHECK[2])/(0.1324))-1);
	nh7=((2*(CHECK[3])/(0.0856))-1);
	nh9=((2*(CHECK[4])/(0.1071))-1);
	*/
	
	
	//--------Normalisasi Input ANN--------------//
	nh1=((2*(TampilI[1])/(0.5268))-1);
	nh3=((2*(TampilI[3])/(0.2096))-1);
	nh5=((2*(TampilI[5])/(0.1324))-1);
	nh7=((2*(TampilI[7])/(0.0856))-1);
	nh9=((2*(TampilI[9])/(0.1071))-1);
	
	
			//--------------LAYER 1--------------// 10 Neuron
	LAYER1[0]=(nh1*4.0375) +(nh3*0.29392) +(nh5*30.6069)	+(nh7*20.3825)	+(nh9*-20.1112)	+(-3.5296);
	LAYER1[1]=(nh1*39.3851) +(nh3*13.1053) +(nh5*-21.1845)	+(nh7*-10.3568)	+(nh9*11.3312)	+(-37.7076);
	LAYER1[2]=(nh1*1.6296) +(nh3*33.2701) +(nh5*0.055486)	+(nh7*0.10137)	+(nh9*0.091234)	+(37.9407);
	LAYER1[3]=(nh1*3.7215) +(nh3*1.2906) +(nh5*30.2253)	+(nh7*20.1441)	+(nh9*-19.9069)	+(-3.5654);
	LAYER1[4]=(nh1*91.5767) +(nh3*7.3533) +(nh5*-43.7818)	+(nh7*-33.0003)	+(nh9*25.001)	+(4.5066);
	LAYER1[5]=(nh1*-18.8743) +(nh3*-8.5107) +(nh5*-0.33644)	+(nh7*-1.2644)	+(nh9*-0.95255)	+(-14.9822);
	LAYER1[6]=(nh1*3.5201) +(nh3*2.2375) +(nh5*30.1407)	+(nh7*20.0335)	+(nh9*-19.7864)	+(-3.8177);
	LAYER1[7]=(nh1*-0.41954) +(nh3*4.424) +(nh5*-0.15732)	+(nh7*-3.2408)	+(nh9*-1.0109)	+(13.8389);
	LAYER1[8]=(nh1*6.0431) +(nh3*-28.8738) +(nh5*25.4228)	+(nh7*23.7651)	+(nh9*-19.5055)	+(16.8474);
	LAYER1[9]=(nh1*-0.00060542) +(nh3*-0.00041619) +(nh5*-0.00053071)	+(nh7*-0.00010885)	+(nh9*0.00023401)	+(4.9227);

	//---------Fungsi Aktivasi Logaritma Sigmoid-----------//
	LOGSIG[0]=(1/(1+exp(-LAYER1[0])));
	LOGSIG[1]=(1/(1+exp(-LAYER1[1])));
	LOGSIG[2]=(1/(1+exp(-LAYER1[2])));
	LOGSIG[3]=(1/(1+exp(-LAYER1[3])));
	LOGSIG[4]=(1/(1+exp(-LAYER1[4])));
	LOGSIG[5]=(1/(1+exp(-LAYER1[5])));
	LOGSIG[6]=(1/(1+exp(-LAYER1[6])));
	LOGSIG[7]=(1/(1+exp(-LAYER1[7])));
	LOGSIG[8]=(1/(1+exp(-LAYER1[8])));
	LOGSIG[9]=(1/(1+exp(-LAYER1[9])));

	//--------------LAYER 2--------------//
	LAYER2[0]=(LOGSIG[0]*-5.1326)+(LOGSIG[1]*13.1017)+(LOGSIG[2]*75.9865)+(LOGSIG[3]*11.6154)+(LOGSIG[4]*1.0396)+(LOGSIG[5]*0.43811)+(LOGSIG[6]*-5.7308)+(LOGSIG[7]*-26.5195)+(LOGSIG[8]*-0.31436)+(LOGSIG[9]*-25.3936)+(-24.8385);

//---------Fungsi Aktivasi Tangensial Sigmoid-----------//
	TANSIG[0]=(2/(1+exp(-2*LAYER2[0]))-1);
	
		//-----DENORMALISASI----//
	denom=((1+TANSIG[0])*((80-10)/(2))+10)/10;
}
void Load_Display(void)
{
//ILI9341_setRotation (4);
//ILI9341_Fill (COLOR_BLUE);
ILI9341_printText("      LOAD MONITORING        ", 0, 0, COLOR_WHITE, COLOR_GREEN2, 2);
ILI9341_printText("  Power  ", 20, 20, COLOR_GREEN2, COLOR_WHITE, 2);
ILI9341_printText("  Energy ", 20, 40, COLOR_GREEN2, COLOR_WHITE, 2);
ILI9341_printText("      LOAD CONDITION         ", 0, 60, COLOR_WHITE, COLOR_GREEN2, 2);
//ILI9341_printText("denom = ", 20, 80, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText(" Monitor", 0, 100, COLOR_WHITE, COLOR_GREEN2, 3);
ILI9341_printText("  Fan   ", 0, 150, COLOR_WHITE, COLOR_GREEN2, 3);
ILI9341_printText("  LHE   ", 0, 200, COLOR_WHITE, COLOR_GREEN2, 3);

//Load_Conditioning();
Load_Conditioning2();
	
sprintf(Teg, "= %4.2f Watt   ",Power);
ILI9341_printText( Teg, 135, 20, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Ars, "= %4.3f W/h   ",Wh);
ILI9341_printText( Ars, 135, 40, COLOR_WHITE, COLOR_BLACK, 2);
//sprintf(Teg, "= %1.4f   ",denom);
//ILI9341_printText( Teg, 40, 80, COLOR_WHITE, COLOR_BLACK, 1);
ILI9341_printText("DENORM =", 10, 80, COLOR_WHITE, COLOR_BLACK, 2);
sprintf(Ars, " %1.4f    ",denom);
ILI9341_printText( Ars, 100, 80, COLOR_WHITE, COLOR_BLACK, 2);
}
/*          WES ENTEK PROGRAME DULURDD          */
/*           GAK USAH DI SCROLL MANEH           */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
