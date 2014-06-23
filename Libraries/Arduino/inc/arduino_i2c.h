/**
  ******************************************************************************
  * @file    arduino_i2c.h
  * @author  Clione Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   library for Arduino I2C
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
#ifndef __ARDUINO_I2C_H
#define __ARDUINO_I2C_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"


void GPIO_Configuration(void);
void I2C_Configuration(void);
	 void I2C_EE_Init(void);
 void I2C_WriteOneByte(uint16_t deviceAddr, unsigned char WriteAddr, unsigned char DataToWrite);

 unsigned char I2C_ReadOneByte(uint16_t deviceAddr, uint8_t ReadAddr);
 
	 
	 
	 
	 
	 
	 
	 
#ifdef __cplusplus
}
#endif

#endif	/* __ARDUINO_I2C_H */

/**
  * @}
  */

/**
  * @}
  */ 

/***************END OF FILE****/
