/**
  ******************************************************************************
  * @file    arduino_uart.h
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
#ifndef __ARDUINO_UART_H
#define __ARDUINO_UART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"



#define D0 GPIO_Pin_3			// USART2  Rx   PA3
#define D1 GPIO_Pin_2			// USART2  Tx   PA2
	 
	 
void Serial1_begin(uint16_t speed);
void Serial_begin(uint32_t speed);
void Serial1_write(uint8_t val);
void Serial_write(uint8_t val);
void Delay_1s(void);
void Delay_tx(void);
	 
	 
	 
	 
	 
	 
	 
	 
#ifdef __cplusplus
}
#endif

#endif	/* __ARDUINO_UART_H */

/**
  * @}
  */

/**
  * @}
  */ 

/***************END OF FILE****/
