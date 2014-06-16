/**
  ******************************************************************************
  * @file    arduino_spi.h
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
#ifndef __ARDUINO_SPI_H
#define __ARDUINO_SPI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"


//SPI
#define D10 GPIO_Pin_11					// PA11
#define D11 GPIO_Pin_5					// PB5
#define D12 GPIO_Pin_4					// PB4 miso
#define D13 GPIO_Pin_3					// PB3 SCk
	 
	 
void SPI_IO_Config(void) ;
uint8_t SPI_RW(uint8_t uchar);
	 
	 
	 
	 
	 
	 
	 
	 
	 
#ifdef __cplusplus
}
#endif

#endif	/* __ARDUINO_SPI_H */

/**
  * @}
  */

/**
  * @}
  */ 

/***************END OF FILE****/
