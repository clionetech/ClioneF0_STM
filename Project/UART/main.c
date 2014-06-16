/**
  ******************************************************************************
  * @file    UART/main.c 
  * @author  Clione Team
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
#include "arduino_uart.h"

/** @addtogroup STM32F0_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup UART
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/




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
		
	Serial_begin(115200);
  	
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
	
		

}
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
