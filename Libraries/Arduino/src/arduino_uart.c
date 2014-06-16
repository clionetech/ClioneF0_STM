/**
  ******************************************************************************
  * @file    arduino_uart.c
  * @author  Clione Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   UART Library for Arduino
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
#include "arduino_uart.h"

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
