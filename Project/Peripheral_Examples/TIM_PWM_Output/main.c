/**
  ******************************************************************************
  * @file    TIM_PWM_Output/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"

/** @addtogroup STM32F0_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup TIM_PWM_Output
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t TimerPeriod = 0;
uint16_t ChannelPulse = 0;
uint16_t USART1_BaudRate=9600;
uint32_t  USART_BaudRate=9600;
__IO uint16_t  ADC1ConvertedValue = 0;
uint8_t  ADC_Channel_No=0xff;

#define A0 GPIO_Pin_0					// pc0
#define A1 GPIO_Pin_1					// PC1
#define A2 GPIO_Pin_2					// PC2
#define A3 GPIO_Pin_3					// PC3
#define A4 GPIO_Pin_4					// PC4
#define A5 GPIO_Pin_5					// PC5

#define D0 GPIO_Pin_3					// USART2  Rx   PA3
#define D1 GPIO_Pin_2					// USART2  Tx   PA2

#define D3 GPIO_Pin_11				// pwm0  pb11
#define D5 GPIO_Pin_8					// pwm1	 pb8
#define D6 GPIO_Pin_9					// pwm2	 pb9
#define D9 GPIO_Pin_4					// pwm3  pa4
#define D10 GPIO_Pin_11				// pwm4  pa11
#define D11 GPIO_Pin_5				// pwm5  pb5



//SPI
//#define D10 GPIO_Pin_11					// PA11
//#define D11 GPIO_Pin_5					// PB5
#define D12 GPIO_Pin_4					// PB4 miso
#define D13 GPIO_Pin_3					// PB3 SCk




/* Private function prototypes -----------------------------------------------*/

void TIM_PWM_Config(void);
void AnalogWrite(uint16_t,uint8_t);
void ADC1_CH_Config(void);
void ADC_delay(void);
void Serial1_begin(uint16_t speed);
void Serial_begin(uint32_t speed);
void Serial1_write(uint8_t val);
void Serial_write(uint8_t val);
void SPI_IO_Config(void) ;
void Delay_1s(void);
void Delay_tx(void);
uint8_t SPI_RW(uint8_t uchar);
uint16_t analogRead(uint16_t pin);
uint16_t adc_reading;
uint8_t uData;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */
	
	SPI_IO_Config(); 
	
	//WAIT TO RECEIVE A BYTE
//	
	
	

  /* TIM Configuration */
 // TIM_PWM_Config();

	Serial_begin(115200);
  

  /* Infinite loop */
	
	//AnalogWrite(PA0, 100);
	
	//AnalogWrite(PA1, 100);
	
	//AnalogWrite(D3, 50);
	
	//AnalogWrite(D5, 100);
	
	//AnalogWrite(D6, 100);
		
	//AnalogWrite(D11, 50);
	
	//AnalogWrite(D9, 50);
	
	//AnalogWrite(D10, 50);
	
	//AnalogWrite(PA10, 100);
	
	//AnalogWrite(PA11, 100);
adc_reading =0;	


		
		//Serial_write(0x0D);
		//Serial_write(0x0A);
			Serial_write('A');
		Delay_tx();
		Serial_write('T');
		Delay_tx();
		Delay_1s()	;	
		
			Serial_write('A');
		Delay_tx();
		Serial_write('T');
		Delay_tx();
		Serial_write('+');
	Delay_tx();
		Serial_write('B');
		Delay_tx();
		Serial_write('A');
		Delay_tx();
		Serial_write('U');
		Delay_tx();
		Serial_write('D');
		Delay_tx();
		Serial_write('4');
	
		Delay_1s()	;	
		
		
		
		
	
	Serial_begin(9600);
	Delay_tx();

	
  while (1)
  {
		
		Serial_write('A');
		Delay_tx();
		Serial_write('T');
		Delay_tx();
		Serial_write('+');
	Delay_tx();
		Serial_write('V');
		Delay_tx();
		Serial_write('E');
		Delay_tx();
		Serial_write('R');
		Delay_tx();
		Serial_write('S');
		Delay_tx();
		Serial_write('I');
		Delay_tx();
		Serial_write('O');
		Delay_tx();
		Serial_write('N');
		Delay_tx();


	Delay_1s()	;	
	
		
			while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
   /* Send byte through the SPI1 peripheral */
   SPI_SendData8(SPI1, 0x55);
		
		 /* Wait to receive a byte */
   while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
 
  /* Return the byte read from the SPI bus */
   ADC_Channel_No=SPI_ReceiveData8(SPI1);
				
		 

		
//		if(USART_GetFlagStatus(USART1, USART_IT_RXNE) != RESET)
 //       {
 //           USART_ClearFlag(USART1,USART_IT_RXNE);
//            uData=USART_ReceiveData(USART1);
//				}
	
	
		
	//adc_reading=analogRead(PA0);
	//adc_reading=analogRead(PA1);	
	//adc_reading=analogRead(PA4);
 //adc_reading=analogRead(PA5);		
//  adc_reading=analogRead(PA6);		
	//	adc_reading=analogRead(PA7);	


}
}


// spi io configuration
void SPI_IO_Config(void)   // d10, d11, d12, d13
{
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	
	//RCC_AHBPeriphClockCmd(D10|D11|D12|D13, ENABLE);//????
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);	  

/* Configure SPI1 pins: SCK, MISO and MOSI ---------------------------------*/ 

	GPIO_InitStruct.GPIO_Pin = D11 | D12 | D13; 
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;  
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 
	GPIO_Init(GPIOB, &GPIO_InitStruct); 

  
	/*!< ?? RF_SPI pins: SCK */
/*	
	GPIO_InitStruct.GPIO_Pin = D13;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2; 
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = D11; 
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
	GPIO_InitStruct.GPIO_Pin = D12; 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2; 
	GPIO_Init(GPIOB, &GPIO_InitStruct); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_0); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_0); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_0);
	
*/	

 /*!< ??RF ?CS pin */
 GPIO_InitStruct.GPIO_Pin =D10; 
 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 
 GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 
 GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; 
 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2; 
 GPIO_Init(GPIOA, &GPIO_InitStruct);
 
 
	
	SPI_StructInit( &SPI_InitStruct );
   SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
   SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
   SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
   SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
   SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
   SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
   SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
   SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	 SPI_InitStruct.SPI_CRCPolynomial = 7;
	 SPI_Init(SPI1, &SPI_InitStruct);
	 SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
   SPI_Cmd(SPI1, ENABLE);  //SPI enable
	 	
}



// spi read and write a byte
uint8_t SPI_RW(uint8_t uchar)
 {
   /* Loop while DR register in not emplty */
   while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
 
  /* Send byte through the SPI1 peripheral */
   SPI_SendData8(SPI1, uchar);
     
  /* Wait to receive a byte */
   while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
 
  /* Return the byte read from the SPI bus */
   return SPI_ReceiveData8(SPI1);
 }
 



void TIM_PWM_Config(void)   //pb8, pb9 pwm op
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOB Clocks enable */
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);
  
  /* GPIOA Configuration: Channel 1, 2, 3 and 4 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_2);
	
	
	/* GPIOA Clocks enable */
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* GPIOA Configuration: Channel 1, 4 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_11 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	 GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_4);
   GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);
	
}





void AnalogWrite(uint16_t pin, uint8_t value)
{
	
	  /* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
  TimerPeriod = (SystemCoreClock / 17570 ) - 1;
  /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 */
  ChannelPulse = (uint16_t) (((uint32_t) value * (TimerPeriod - 1)) / 255);
 
  

	if (pin==D5){
			 /* Channel 1 Configuration in PWM mode */
		/* TIM16 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16 , ENABLE);
  
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);
		
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OCInitStructure.TIM_Pulse = ChannelPulse;
  TIM_OC1Init(TIM16, &TIM_OCInitStructure);
		 /* TIM1 counter enable */
  TIM_Cmd(TIM16, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM16, ENABLE);
	}
	
	if (pin==D6){
		
				 /* Channel 1 Configuration in PWM mode */
		/* TIM17 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17 , ENABLE);
  
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
		
				 /* Channel 1 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OCInitStructure.TIM_Pulse = ChannelPulse;
  TIM_OC1Init(TIM17, &TIM_OCInitStructure);
		 /* TIM1 counter enable */
  TIM_Cmd(TIM17, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM17, ENABLE);
	}
	
	if (pin==D3){
			/* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
  
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
		
	/* Channel 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OCInitStructure.TIM_Pulse = ChannelPulse;
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);
		 /* TIM1 counter enable */
  TIM_Cmd(TIM2, ENABLE);

/* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM2, ENABLE);
	}
	
  if (pin==D11){
		
	/* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
  
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
		
	/* Channel 2 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

   TIM_OCInitStructure.TIM_Pulse = ChannelPulse;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  /* TIM3 counter enable */
  TIM_Cmd(TIM3, ENABLE);
 
 /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM3, ENABLE);
	}

	if (pin==D9) {
		
		 
  /* TIM14 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 , ENABLE);
  
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

			 /* Channel 1, 2 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OCInitStructure.TIM_Pulse = ChannelPulse;
  TIM_OC1Init(TIM14, &TIM_OCInitStructure);
		 /* TIM1 counter enable */
  TIM_Cmd(TIM14, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM14, ENABLE);
		
	}
		
	
if (pin==D10) {
	 /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
  
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

			 /* Channel 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OCInitStructure.TIM_Pulse = ChannelPulse;
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
		 /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
		
	}	
	
}



void ADC1_CH_Config(void)
{
  ADC_InitTypeDef          ADC_InitStructure;
  GPIO_InitTypeDef         GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef        TIM_OCInitStructure; 
  
  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  
  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  /* TIM3 Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  /* Configure ADC Channel11 as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /* TIM3 Configuration *******************************************************/
  TIM_DeInit(TIM3);
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_OCStructInit(&TIM_OCInitStructure);
    
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 0xFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  /* TIM3 TRGO selection */
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
  
  /* ADC1 Configuration *******************************************************/
  /* ADCs DeInit */  
  ADC_DeInit(ADC1);
  
  /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits*/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;    
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure); 
  
  /* Convert the ADC1 Channel 11 with 239.5 Cycles as sampling time */ 
  ADC_ChannelConfig(ADC1, ADC_Channel_0 , ADC_SampleTime_28_5Cycles);   

  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);

  /* Enable the auto delay feature */    
  ADC_WaitModeCmd(ADC1, ENABLE); 
  
  /* Enable the Auto power off mode */
  ADC_AutoPowerOffCmd(ADC1, ENABLE); 
  
  /* Enable ADCperipheral[PerIdx] */
  ADC_Cmd(ADC1, ENABLE);     
  
  /* Wait the ADCEN falg */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)); 
  
  /* TIM2 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  
  /* ADC1 regular Software Start Conv */ 
  ADC_StartOfConversion(ADC1);
}




void ADC_Channel_Init(uint16_t pin )
{	
	 ADC_InitTypeDef          ADC_InitStructure;
  GPIO_InitTypeDef         GPIO_InitStructure;
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef        TIM_OCInitStructure; 
 
  /* GPIOC Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	 /* TIM3 Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	
     /* Configure ADC Channelx as analog input */
  GPIO_InitStructure.GPIO_Pin = pin ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	 /* TIM3 Configuration *******************************************************/
  TIM_DeInit(TIM3);
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_OCStructInit(&TIM_OCInitStructure);
    
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 0xFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  /* TIM3 TRGO selection */
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
  	
  
  /* ADC1 Configuration *******************************************************/
   ADC_DeInit(ADC1);
  
  /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits*/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;    
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure); 

}

void ADC_Start_Setting(void)
{
	  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);

  /* Enable the auto delay feature */    
  ADC_WaitModeCmd(ADC1, ENABLE); 
  
  /* Enable the Auto power off mode */
  ADC_AutoPowerOffCmd(ADC1, ENABLE); 
  
  /* Enable ADCperipheral[PerIdx] */
  ADC_Cmd(ADC1, ENABLE);     
  
  /* Wait the ADCEN falg */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)); 
  
  /* TIM2 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  
  /* ADC1 regular Software Start Conv */ 
  ADC_StartOfConversion(ADC1);
	
}



//  ADC conversion 6 channels setting
/*??ADC0*/
void ADC0_Config(void)
{
 ADC_Channel_Init(A0);
  
  /* Convert the ADC1 Channel 0 with 239.5 Cycles as sampling time */ 
  ADC_ChannelConfig(ADC1, ADC_Channel_10 , ADC_SampleTime_28_5Cycles);  

	ADC_Start_Setting();
	ADC_Channel_No=0;

 
}


/*??ADC1*/
void ADC1_Config(void)
{
 ADC_Channel_Init(A1);
  
  /* Convert the ADC1 Channel 11 with 239.5 Cycles as sampling time */ 
  ADC_ChannelConfig(ADC1, ADC_Channel_11 , ADC_SampleTime_28_5Cycles);   

 ADC_Start_Setting();
	ADC_Channel_No=1;
}


void ADC2_Config(void)
{
  ADC_Channel_Init(A2 );
  
  /* Convert the ADC1 Channel 11 with 239.5 Cycles as sampling time */ 
  ADC_ChannelConfig(ADC1, ADC_Channel_12, ADC_SampleTime_28_5Cycles);   

  ADC_Start_Setting();
	ADC_Channel_No=2;
}


void ADC3_Config(void)
{
  ADC_Channel_Init(A3 );
  
  /* Convert the ADC1 Channel 3 with 239.5 Cycles as sampling time */ 
  ADC_ChannelConfig(ADC1, ADC_Channel_13 , ADC_SampleTime_28_5Cycles);   

 ADC_Start_Setting();
	ADC_Channel_No=3;
}

void ADC4_Config(void)
{
   ADC_Channel_Init(A4 );
  
  /* Convert the ADC1 Channel 3 with 239.5 Cycles as sampling time */ 
  ADC_ChannelConfig(ADC1, ADC_Channel_14 , ADC_SampleTime_28_5Cycles);   

ADC_Start_Setting();
	ADC_Channel_No=4;
}

void ADC5_Config(void)
{
   ADC_Channel_Init(A5);
  
  /* Convert the ADC1 Channel 3 with 239.5 Cycles as sampling time */ 
  ADC_ChannelConfig(ADC1, ADC_Channel_15 , ADC_SampleTime_28_5Cycles);   

ADC_Start_Setting();
	ADC_Channel_No=5;
}



uint16_t analogRead(uint16_t pin){
	
	switch(pin)
	{
	case A0:
		if (ADC_Channel_No!=0)ADC0_Config();
	
	/* Get ADC1 converted data */
	//some delay
	ADC_delay();
	
   ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
	return ADC1ConvertedValue;
//	  break;

	case A1:
	if (ADC_Channel_No!=1)ADC1_Config();
	/* Get ADC1 converted data */
		ADC_delay();
   ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
	return ADC1ConvertedValue;
	//  break;
	
	case A2:
	if (ADC_Channel_No!=2)ADC2_Config();
	/* Get ADC1 converted data */
		ADC_delay();
   ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
	return ADC1ConvertedValue;
	//  break;
	
	case A3:
	if (ADC_Channel_No!=3)ADC3_Config();
	/* Get ADC1 converted data */
		ADC_delay();
   ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
	return ADC1ConvertedValue;
//	  break;
	
	case A4:
	if (ADC_Channel_No!=4)ADC4_Config();
	/* Get ADC1 converted data */
		ADC_delay();
   ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
	return ADC1ConvertedValue;
//	  break;
	
	case A5:
	if (ADC_Channel_No!=5)ADC5_Config();
	/* Get ADC1 converted data */
		ADC_delay();
   ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
	return ADC1ConvertedValue;
	//  break;
	
	  default: /* none */
      break;
	
}

	return 0;
	
}	




/*USART1
 USART1 TX-->PA9,RX-->PA10 */
 void Serial1_begin(uint16_t speed)
 {
         GPIO_InitTypeDef  GPIO_InitStructure;
         USART_InitTypeDef USART_InitStructure;
                 
        RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
         RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE );
                 
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);
         GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);        
        /*
         *  USART1_TX -> PA9 , USART1_RX ->        PA10
         */                                
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;                 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
         GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
        GPIO_Init(GPIOA, &GPIO_InitStructure);        
        
        USART_InitStructure.USART_BaudRate = speed;
         USART_InitStructure.USART_WordLength = USART_WordLength_8b;
         USART_InitStructure.USART_StopBits = USART_StopBits_1;
         USART_InitStructure.USART_Parity = USART_Parity_No;
         USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
         USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
         USART_Init(USART1, &USART_InitStructure); 

        USART_Cmd(USART1, ENABLE);
 }
 
 /*USART2
 USART2 TX-->PA2,RX-->PA3 */
  void Serial_begin(uint32_t speed)
 {
         GPIO_InitTypeDef  GPIO_InitStructure;
         USART_InitTypeDef USART_InitStructure;
                 
        RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	 
				RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE );
      
                 
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_1);
         GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_1);        
        /*
         *  USART2_TX -> PA2 , USART2_RX ->        PA3
         */                                
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;                 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
         GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
        GPIO_Init(GPIOA, &GPIO_InitStructure);        
        
        USART_InitStructure.USART_BaudRate = speed;
         USART_InitStructure.USART_WordLength = USART_WordLength_8b;
         USART_InitStructure.USART_StopBits = USART_StopBits_1;
         USART_InitStructure.USART_Parity = USART_Parity_No;
         USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
         USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
         USART_Init(USART2, &USART_InitStructure); 

        USART_Cmd(USART2, ENABLE);
 }

 
 void Serial1_write(uint8_t val) {
	 USART_SendData(USART1, val);
 }
 
  void Serial_write(uint8_t val) {
		USART_SendData(USART2, val);
 }
 
 

void ADC_delay(void){
	uint8_t  i;
	
	for(i=0;i<255;i++){
		__NOP;
		__NOP;
		__NOP;
		__NOP;
		__NOP;
		__NOP;
		}
}

void  Delay_1s(void) {
	uint16_t  i;
	uint8_t  j;
	
	for(j=0;j<120;j++){
	
	for(i=0;i<62550;i++){
		__NOP;
		__NOP;
		__NOP;
		__NOP;
		__NOP;
		__NOP;
		}
	}
	
}


void  Delay_tx(void) {
	uint16_t  i;
	
	for(i=0;i<62550;i++){
		__NOP;
		__NOP;
		__NOP;
		__NOP;
		__NOP;
		__NOP;
		}
	
}



#define I2C_Speed              300000
 #define I2C1_SLAVE_ADDRESS7    0xA0
 #define I2C_PageSize           8



/**********************************************************************/
/*IIC??		         				                              */
/*																	  */
/**********************************************************************/
void GPIO_Configuration(void)
{ 
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);	//??GPIO?? 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 , ENABLE); //??i2c?? 
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8; 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//?????? 
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3; 
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; 
	GPIO_Init(GPIOB , &GPIO_InitStruct); 
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9; 
	GPIO_Init(GPIOB , &GPIO_InitStruct); 
	GPIO_PinAFConfig( GPIOB , GPIO_PinSource8, GPIO_AF_1);
	GPIO_PinAFConfig( GPIOB ,GPIO_PinSource9, GPIO_AF_1); //??????
	}

void I2C_Configuration(void)//i2c????
{ 
	I2C_InitTypeDef I2C_InitStruct;  
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;//???? 
	I2C_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Enable;//???????? 
	I2C_InitStruct.I2C_DigitalFilter = 0x00; 
	I2C_InitStruct.I2C_OwnAddress1 =0x00; 
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;//???? 
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; //?????7bit 
	I2C_InitStruct.I2C_Timing = 0x00210507;//????????? 
	I2C_Cmd(I2C1, ENABLE);//??i2c 
	I2C_Init(I2C1, &I2C_InitStruct);//?????
	}

/**********************************************************************/
/*IIC write a byte         				                              */
/*																	  */
/**********************************************************************/
 //WriteAddr  :????  
//DataToWrite:??
 void I2C_WriteOneByte(uint16_t deviceAddr, unsigned char WriteAddr, unsigned char DataToWrite)
 {    
    I2C_NumberOfBytesConfig(I2C1, 2);
    I2C_SlaveAddressConfig(I2C1, deviceAddr);
    I2C_MasterRequestConfig(I2C1, I2C_Direction_Transmitter);
		I2C_GenerateSTART(I2C1, ENABLE);                          
		I2C_SendData(I2C1, WriteAddr);                
		while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE)) 
    {
         if(I2C_GetFlagStatus(I2C1,I2C_FLAG_NACKF))
         {
             I2C_ClearFlag(I2C1,I2C_FLAG_NACKF);
             I2C_ClearFlag(I2C1,I2C_FLAG_STOPF);
             return;
         }
     }
		I2C_SendData(I2C1, DataToWrite);                                        
		while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE)); 
		I2C_GenerateSTOP(I2C1, ENABLE);                   
} 






/**********************************************************************/
/*IIC???	         				                              	  */
/*																	  */
/**********************************************************************/
//I2C_ReadOneByte
 //ReadAddr:??
 //???:??????
 unsigned char I2C_ReadOneByte(uint16_t deviceAddr, uint8_t ReadAddr)
 {    
     unsigned char temp=0;    
     I2C_NumberOfBytesConfig(I2C1, 1);
     I2C_SlaveAddressConfig(I2C1, deviceAddr);
     I2C_MasterRequestConfig(I2C1, I2C_Direction_Transmitter);
   I2C_GenerateSTART(I2C1, ENABLE);           //START
   I2C_SendData(I2C1, ReadAddr);                   //??
   while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE)) //???????,???NACK,????
     {
         if(I2C_GetFlagStatus(I2C1,I2C_FLAG_NACKF))
         {
             I2C_ClearFlag(I2C1,I2C_FLAG_NACKF);
             I2C_ClearFlag(I2C1,I2C_FLAG_STOPF);
             return 0;
         }
     }
     
     I2C_MasterRequestConfig(I2C1, I2C_Direction_Receiver);
     I2C_GenerateSTART(I2C1, ENABLE);                                 //START
     while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE));
     temp=I2C_ReceiveData(I2C1);
     I2C_GenerateSTOP(I2C1, ENABLE);                                //STOP
     return temp;
 }
 
 


 
 
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/*****************END OF FILE****/
