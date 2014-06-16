/**
  ******************************************************************************
  * @file    arduino_pwm.h
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
#ifndef __ARDUINO_PWM_H
#define __ARDUINO_PWM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"


#define D3 GPIO_Pin_11				// pwm0  pb11
#define D5 GPIO_Pin_8				// pwm1	 pb8
#define D6 GPIO_Pin_9				// pwm2	 pb9
#define D9 GPIO_Pin_4				// pwm3  pa4
#define D10 GPIO_Pin_11				// pwm4  pa11
#define D11 GPIO_Pin_5				// pwm5  pb5


	 
void TIM_PWM_Config(void);
void AnalogWrite(uint16_t,uint8_t);
	 
	 
	 
	 
	 
#ifdef __cplusplus
}
#endif

#endif	/* __ARDUINO_PWM_H */

/**
  * @}
  */

/**
  * @}
  */ 

/***************END OF FILE****/
