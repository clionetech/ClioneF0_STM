/**
  ******************************************************************************
  * @file    arduino_i2c.c
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
#include "arduino_i2c.h"

 
/**********************************************************************/
/*IIC??		         				                              */
/*																	  */
/**********************************************************************/
void GPIO_Configuration(void)
{ 
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);	//enable scl sda pb6,pb7 clock 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 , ENABLE); //enable i2c1 clock 
	
	/* Configure I2C1 pins: SCL and SDA */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6; // gpb6-i2c1_scl
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//gpio alternage function mode
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;  // high speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; // open drain mode
	GPIO_Init(GPIOB , &GPIO_InitStruct); 
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;		// gpb7-i2c1_sda
	GPIO_Init(GPIOB , &GPIO_InitStruct); 
	
	GPIO_PinAFConfig( GPIOB , GPIO_PinSource6, GPIO_AF_1); // multi channel
	GPIO_PinAFConfig( GPIOB ,GPIO_PinSource7, GPIO_AF_1); 
	}

void I2C_Configuration(void)//i2c1 setting
{ 
	I2C_InitTypeDef I2C_InitStruct;  
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;// i2c mode
	I2C_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Enable;// 
	I2C_InitStruct.I2C_DigitalFilter = 0x00; 
	I2C_InitStruct.I2C_OwnAddress1 =0x00; 
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;// ack setting
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; //?????7bit 
	I2C_InitStruct.I2C_Timing =0x00210507;// set scl and sda timing
	
	I2C_Cmd(I2C1, ENABLE);				//i2c peripheral enable
	I2C_Init(I2C1, &I2C_InitStruct);//
	
	}

/*******************************************************************************
* Function Name  : I2C_EE_Init
* Description    : Initializes peripherals used by the I2C EEPROM driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_EE_Init(void)
{
  /* GPIO configuration */
  GPIO_Configuration();

  /* I2C configuration */
  I2C_Configuration();
	
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
 

