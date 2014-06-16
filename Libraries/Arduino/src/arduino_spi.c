/**
  ******************************************************************************
  * @file    arduino_spi.c
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
#include "arduino_spi.h"

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

