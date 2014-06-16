/**
  ******************************************************************************
  * @file    arduino_adc.h
  * @author  Clione Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   library for Arduino ADC
  *          
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ARDUINO_ADC_H
#define __ARDUINO_ADC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"


#define A0 GPIO_Pin_0					// pc0
#define A1 GPIO_Pin_1					// PC1
#define A2 GPIO_Pin_2					// PC2
#define A3 GPIO_Pin_3					// PC3
#define A4 GPIO_Pin_4					// PC4
#define A5 GPIO_Pin_5					// PC5
	 
	 
void ADC1_CH_Config(void);
void ADC_delay(void);	 
uint16_t analogRead(uint16_t pin);	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
#ifdef __cplusplus
}
#endif

#endif	/* __ARDUINO_ADC_H */

/**
  * @}
  */

/**
  * @}
  */ 

/***************END OF FILE****/
