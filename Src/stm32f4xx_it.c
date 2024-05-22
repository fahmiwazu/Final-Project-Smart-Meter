/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32f4xx_it.h"
#include "stdio.h"
#include "math.h"

/* USER CODE BEGIN 0 */
#include "arm_math.h"

#define SAMPLE 128
//#define SAMPLE 2048
#define FFT_SIZE SAMPLE/2
#define BLOCKSIZE 64
//#define BLOCKSIZE 1000
//#define BLOCKSIZE 1024
extern char rx_buffer[50];
extern uint32_t	blockSize, i, pIndexV, pIndexI, exti, a,b,pp;
extern float32_t	pResultVrms, pResultIrms, pResultVmax, pResultImax, pSrcV[BLOCKSIZE+1], pSrcI[BLOCKSIZE+1], Vrmsrms,Vrmsrmsfft, Vrmsmax, Irmsrms,Irmsrmsfft, Irmsmax;
extern float32_t offset,THDv[7],THDi[7],P[7],Q[7],S;
extern float32_t TampilV[FFT_SIZE],TampilI[FFT_SIZE],Vfft[SAMPLE],Ifft[SAMPLE],OutputVfft[FFT_SIZE],OutputIfft[FFT_SIZE],VrmsFFT, IrmsFFT, Vrms_fft[FFT_SIZE],Irms_fft[FFT_SIZE];
extern 	uint32_t beb1, beb2, beb3;
extern float32_t SpecMin,SpecMax,IPOWER[SAMPLE],VPOWER[SAMPLE],watthour[SAMPLE];
/* USER CODE END 0 */
extern		arm_cfft_radix2_instance_f32 Radix2;
extern		float32_t maxValue;
extern		uint32_t maxIndex;
extern  RTC_TimeTypeDef sTime;
extern  RTC_DateTypeDef sDate;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern RTC_HandleTypeDef hrtc;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC_IRQn 1 */
//	pSrcV[i] = HAL_ADC_GetValue(&hadc1);
//	pSrcI[i] = HAL_ADC_GetValue(&hadc2);
	offset = HAL_ADC_GetValue(&hadc3);
	pSrcI[i] = HAL_ADC_GetValue(&hadc1) - offset;
	pSrcV[i] = HAL_ADC_GetValue(&hadc2) - offset;

	Vfft[2*i]=pSrcV[i];
	Vfft[(2*i)+1]=0;	
	Ifft[2*i]=pSrcI[i];
	Ifft[(2*i)+1]=0;
	
	i++;
	
	/******************************************************************************/
	/* This function for calculate moving RMS                                     */
	/******************************************************************************/
	//arm_rms_f32((float32_t *) &pSrcV, blockSize, &pResultVrms);
	//arm_rms_f32((float32_t *) &pSrcI, blockSize, &pResultIrms);
	
	if(i >= blockSize)
	{
		i = 0;

//-----------------------FFT VOLTAGE-----------------------------//		
		arm_cfft_radix2_init_f32(&Radix2, FFT_SIZE, 0, 1);
		arm_cfft_radix2_f32(&Radix2, Vfft);
		arm_cmplx_mag_f32((float32_t *) &Vfft,OutputVfft,FFT_SIZE);	
		for(a=2,Vrms_fft[11]=0;a<=FFT_SIZE+2;a++){	
			Vrms_fft[9]=(pow(Vfft[a],2));
			Vrms_fft[11]=Vrms_fft[11]+Vrms_fft[9];
			Vrms_fft[9]=0;}
		Vrms_fft[0]=sqrt(Vrms_fft[11]/((FFT_SIZE/2)))/(FFT_SIZE/8);
		
	
//----------------------FFT CURRENT------------------------------//
		arm_cfft_radix2_init_f32(&Radix2, FFT_SIZE, 0, 1);
		arm_cfft_radix2_f32(&Radix2, Ifft);
		arm_cmplx_mag_f32((float32_t*)&Ifft,OutputIfft,FFT_SIZE);
		for(b=2,Irms_fft[11]=0;b<=FFT_SIZE+2;b++){ 
			Irms_fft[9]=(pow(Ifft[b],2));
			Irms_fft[11]=Irms_fft[11]+Irms_fft[9];
			Irms_fft[9]=0;}
		Irms_fft[0]=sqrt(Irms_fft[11]/((FFT_SIZE/2)))/(FFT_SIZE/8);		

		P[4]=watthour[4];
			
		for(pp=2,P[4]=0;pp<FFT_SIZE-2;pp++)
		{
			IPOWER[pp]=((((float32_t)Ifft[pp]/(sqrt(2)*(FFT_SIZE/2)))-(float32_t)4.864126475)/(float32_t)253.7943361);
			VPOWER[pp]=((((float32_t)Vfft[pp]/(sqrt(2)*(FFT_SIZE/2)))-(float32_t)0.10084)/(float32_t)4.5739);
			
			P[5]=IPOWER[pp]*VPOWER[pp];
			P[4]=P[4]+P[5];	
			P[5]=0;
		}
		
		
		
		/******************************************************************************/
		/* This function for calculate RMS in windowing & Maximum                     */
		/******************************************************************************/
		arm_rms_f32((float32_t *) &pSrcV, blockSize, &pResultVrms);
		arm_rms_f32((float32_t *) &OutputVfft, blockSize, &VrmsFFT);
//		arm_max_f32((float32_t *) &pSrcV, blockSize, &pResultVmax, &pIndexV);
//		Vrmsrms = (pResultVrms-(float32_t)0.0105)/(float32_t)4.5749;
//    Vrmsrms = (pResultVrms-(float32_t)1.1082)/(float32_t)4.5677; // coba-coba 1000
//		Vrmsrms = (pResultVrms-(float32_t)3.7759)/(float32_t)4.5366 ; //coba-coba 1024
		Vrmsrms = (pResultVrms-(float32_t)0.10084)/(float32_t)4.5739; // coba-coba 64
		Vrmsrmsfft = ((VrmsFFT/(FFT_SIZE/8))-(float32_t)0.10084)/(float32_t)4.5739;
		Vrms_fft[10] =((Vrms_fft[0])-(float32_t)0.10084)/(float32_t)4.5739;
		
//		Vrmsmax = (pResultVmax-(float32_t)15.372)/(float32_t)6.4082;
//		Vrmsmax = (pResultVmax-(float32_t)2064.3)/(float32_t)6.4091;	//no offset
		arm_rms_f32((float32_t *) &pSrcI, blockSize, &pResultIrms);
		arm_rms_f32((float32_t *) &OutputIfft, blockSize, &IrmsFFT);
//		arm_max_f32((float32_t *) &pSrcI, blockSize, &pResultImax, &pIndexI);
//		Irmsrms = (pResultIrms-(float32_t)2.9672)/(float32_t)248.85; // 1000
//		Irmsrms = (pResultIrms-(float32_t)32.947857)/(float32_t)241.204325; //64N TANG IJO
		Irmsrms = (pResultIrms-(float32_t)4.864126475)/(float32_t)253.7943361; //64N		HIOKI
		Irmsrmsfft = ((IrmsFFT/(FFT_SIZE/8))-(float32_t)-1.39776462244555)/(float32_t)254.609739589478; //64N  HIOKI NEW
		Irms_fft[10] = ((Irms_fft[0])-(float32_t)4.864126475)/(float32_t)253.7943361; //64N 	HIOKI
//		Irms_fft[1] = ((Irms_fft[0])-(float32_t)-1.39776462244555)/(float32_t)254.609739589478; //64N 	HIOKI
//		Irms_fft[10] = ((Irms_fft[0])-(float32_t)32.947857)/(float32_t)241.204325; //64N		TANG IJO
	
//---------------------------POWER CALCULATION--------------------------//
	S = Vrmsrms*Irmsrms;

	}
  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles RTC alarms A and B interrupt through EXTI line 17.
  */
void RTC_Alarm_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_Alarm_IRQn 0 */

  /* USER CODE END RTC_Alarm_IRQn 0 */
  HAL_RTC_AlarmIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_Alarm_IRQn 1 */

  /* USER CODE END RTC_Alarm_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream4 global interrupt.
  */
void DMA2_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream4_IRQn 0 */

  /* USER CODE END DMA2_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream4_IRQn 1 */

  /* USER CODE END DMA2_Stream4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
