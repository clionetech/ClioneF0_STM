/**
  ******************************************************************************
  * @file    arduino_adc.c
  * @author  Clione Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   ADC Library for Arduino
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
#include "arduino_adc.h"

uint8_t  ADC_Channel_No=0xff;

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
			ADC_delay();
			return ADC_GetConversionValue(ADC1);

		case A1:
			if (ADC_Channel_No!=1)ADC1_Config();
			
			/* Get ADC1 converted data */
			ADC_delay();
			return ADC_GetConversionValue(ADC1);
		
		case A2:
			if (ADC_Channel_No!=2)ADC2_Config();
			
		/* Get ADC1 converted data */
		ADC_delay();
		return ADC_GetConversionValue(ADC1);
	
		case A3:
			if (ADC_Channel_No!=3)ADC3_Config();
			
			/* Get ADC1 converted data */
			ADC_delay();
			return ADC_GetConversionValue(ADC1);
	
		case A4:
			if (ADC_Channel_No!=4)ADC4_Config();
			
			/* Get ADC1 converted data */
			ADC_delay();
			return ADC_GetConversionValue(ADC1);
	
		case A5:
			if (ADC_Channel_No!=5)ADC5_Config();
			
			/* Get ADC1 converted data */
			ADC_delay();
			return ADC_GetConversionValue(ADC1);
	
	  default: /* none */
      break;
	
}

	return 0;
	
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

