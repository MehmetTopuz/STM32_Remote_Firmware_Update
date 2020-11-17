/**
  ******************************************************************************
  * @file           : IAP.c
	* @author 				: Mehmet Topuz(mehmettopuz.net)
  * @brief          : In Application Programming Driver
  ******************************************************************************
	** - This driver remotely updates the firmware using ESP8266 and TFTP protocol.
	*
	*                    ------------------------------ FLASH_END_ADDRESS(0x0800FFFF)
	*										 |														|
	*										 |														|
	*										 |														|
	*										 |						APP							|
	*										 |	(49KB reserved for APP)	  |
	*										 |														|
	*										 |														|
	*										 |														|
	*                    ------------------------------ APP_START_ADDRESS(0x08003C00)
	*										 |														|
	*										 |					  IAP							|
	*										 |   (15KB reserved for IAP) 	|
	*										 |														|
	*                    ------------------------------ FLASH_BASE_ADDRESS(0x08000000)
	*														  Flash Memory
	*
  ******************************************************************************
  */

#include "IAP.h"


char RxBuffer[550];
uint8_t Block[530];
uint8_t BinData[512];
uint32_t AppSize = 0;

CRC_HandleTypeDef hcrc;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/***********************************************************/
/*  Using printf() to print strings via UART2   */ 
/***********************************************************/

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 2000);
  return ch;
}

/**
  * @brief  Initialize some peripherals for IAP.
  * @param  None
  * @retval None
  */
void IAP_Init(void)
{
	GPIO_Init();								
	DMA_Init();							// DMA for UART1
	USART1_UART_Init();			// UART1 --> ESP8266
	USART2_UART_Init();			// UART2 --> User Interface
	CRC_Init();
}
/**
  * @brief  Read data from flash memory.
  * @param  adr: 32 bit memory address.
  * @retval 16 bit flash data.
  */
uint16_t Read_Flash(uint32_t  adr)
{
  return *(uint16_t*)adr;
}

/**
  * @brief  This function connects the ESP8266 to the internet.
  * @param  ssid: SSID of Wifi.
	* @param  password: Wifi network password.
  * @retval Connection_Status_Typedef
  */
Connection_Status_Typedef WifiConnect(char *ssid ,char *password)
{
	char txBuffer[30],rxBuffer[10];
	HAL_UART_Transmit(&huart1,(uint8_t *)"ATE0\r\n",strlen("ATE0\r\n"),1000); // turn off echo mode
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1,(uint8_t *)"AT+CWMODE=1\r\n",strlen("AT+CWMODE=1\r\n"),1000);	
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1,(uint8_t *)"AT+CWQAP\r\n",strlen("AT+CWQAP\r\n"),1000);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1,(uint8_t *)txBuffer,sprintf(txBuffer,"AT+CWJAP=\"%s\",\"%s\"\r\n",ssid,password),1000);
	HAL_Delay(100);
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)rxBuffer,6);
	volatile uint32_t TimeOut = 5000;
	volatile uint32_t Time = HAL_GetTick();
	while(HAL_GetTick() - Time < TimeOut)
	{
		for(int i=0;i<6-strlen("OK");i++) 
		{
			if(rxBuffer[i] == 'O' && rxBuffer[i+1] == 'K') // if ESP8266 responds "OK"
			{
				HAL_UART_DMAStop(&huart1);
				return Connection_OK;
			}
		}
	
 }
	printf("Wifi Connection timeout.\n");
	HAL_UART_DMAStop(&huart1);
	return Connection_ERROR;
}

/**
  * @brief  This function connects the ESP8266 to TFTP server.
  * @param  ipAddress: IP address of TFTP server.
	* @param  port: UDP port.
  * @retval Connection_Status_Typedef
  */
Connection_Status_Typedef TFTPServerConnect(char *ipAddress,char *port)
{
	char txBuffer[40],rxBuffer[15];
	
	HAL_UART_Transmit(&huart1,(uint8_t *)"AT+CIPCLOSE\r\n",strlen("AT+CIPCLOSE\r\n"),1000);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart1,(uint8_t *)"AT+CIPMUX=0\r\n",strlen("AT+CIPMUX=0\r\n"),1000); 
	HAL_Delay(100);
//	HAL_UART_Transmit(&huart1,(uint8_t *)"AT+CIFSR\r\n",strlen("AT+CIFSR\r\n"),1000);
//	HAL_Delay(100);
	HAL_UART_Transmit(&huart1,(uint8_t *)txBuffer,sprintf(txBuffer,"AT+CIPSTART=\"UDP\",\"%s\",%s\r\n",ipAddress,port),5000);
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)rxBuffer,15);
	volatile uint32_t TimeOut = 5000;
	volatile uint32_t Time = HAL_GetTick();
	while(HAL_GetTick() - Time < TimeOut)
	{
	for(int i=0;i<15-strlen("CONNECT");i++)
	{
		if(rxBuffer[i]   == 'C' &&
			 rxBuffer[i+1] == 'O' && 
			 rxBuffer[i+2] == 'N' &&
			 rxBuffer[i+3] == 'N' && 
			 rxBuffer[i+4] == 'E' && 
			 rxBuffer[i+5] == 'C' && 
			 rxBuffer[i+6] == 'T')
		{
				HAL_UART_DMAStop(&huart1);
				return Connection_OK;
		}
	}
}
	HAL_UART_DMAStop(&huart1);
	return Connection_ERROR;
}

/**
  * @brief  This function writes 8 bits array to the flash memory.
  * @param  Address: 32 bits memory address.
	* @param  u8Data: 8 bits array.
	* @param  Size: size of array.
  * @retval Flash_Status_Typedef
  */
Flash_Status_Typedef Flash_Write(uint32_t Address , uint8_t *u8Data,uint16_t Size)
{
	FLASH_EraseInitTypeDef EraseInit;
	uint32_t PageError;
	HAL_FLASH_Unlock();
	for(int i=0;i<64;i++)
	{
		if(Address ==(FLASH_BASE_ADDRESS + i*1024))   // if address is equal to start address of any pages.
		{
			EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
			EraseInit.PageAddress = Address;
			EraseInit.NbPages 		= 1;
	
			if(HAL_FLASHEx_Erase(&EraseInit,&PageError) != HAL_OK)
				{
					return Flash_ERROR;
				}	
		}
	}
	for(int i=0;i<Size;i+=2)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address,((u8Data[i+1] << 8) | u8Data[i])) == HAL_OK)
		{
			Address += 2;
		}
		else
		{
			HAL_FLASH_Lock();
			return Flash_ERROR;
		}
	}
	HAL_FLASH_Lock();
	return Flash_OK;
}

/**
  * @brief  This function sends acknowledge packet to the TFTP server.
  * @param  BlockNumber: 16 bits block number of the received block.
  * @retval None.
  */
void ACK(uint16_t BlockNumber) 
{
	TFTP_Opcode opcode = TFTP_ACK;
	uint8_t txBuffer[4] = {0x00,opcode,BlockNumber>>8,BlockNumber}; // TFTP acknowledge packet
	HAL_UART_Transmit(&huart1,(uint8_t *)"AT+CIPSEND=4\r\n",strlen("AT+CIPSEND=4\r\n"),1000);
	HAL_Delay(25);
	HAL_UART_Transmit(&huart1,txBuffer,4,1000);
}

/**
  * @brief  This function reads file from TFTP server and writes to flash memory.
  * @retval None.
  */
void ReceiveHandler(void)
{
	uint32_t AppAddress = APP_START_ADDRESS;
	char BlockSizeString[3];
	uint16_t BlockSizeInt = 0,state=0;
	TFTP_Opcode opcode;
	uint16_t BlockNumber=0,OldBlockNumber = 0;
	int32_t NumberOfBytes=0;
	
	uint32_t TimeOut = 10000;  // 10 seconds timeout
	uint32_t Time;
	while(1)
	{
		Time = HAL_GetTick();
		while(HAL_GetTick() - Time < TimeOut)
		{
			for(int i =0; i<530;i++)
			{
				if(RxBuffer[i]   == '+' &&						// Find "+IPD," characters in the RxBuffer.
					 RxBuffer[i+1] == 'I' &&
					 RxBuffer[i+2] == 'P' &&
					 RxBuffer[i+3] == 'D' &&
					 RxBuffer[i+4] == ',')
					{
						for(int j=0;j<5;j++)
						{
							if(RxBuffer[i+j+5] == ':')
							{		
								BlockSizeInt = atoi(BlockSizeString);

								for(int k=0;k<530;k++)
								{
									if(RxBuffer[i+j+k+6] == 0x0D &&
										 RxBuffer[i+j+k+7] == 0x0A &&
										 RxBuffer[i+j+k+8] == 'O' &&
										 RxBuffer[i+j+k+9] == 'K' )
										{
											state = 1;
											HAL_UART_DMAStop(&huart1);
											break;
										}
										else
										{
											Block[k] = RxBuffer[i+j+k+6];    // get block data from RxBuffer
										}		
								}
								break;
							}
							else
							{
								BlockSizeString[j] = RxBuffer[i+j+5]; // get block size 
						
							}
						}		
						break;
					}
				}
				if(state)   
				{
					state = 0;
					break;
				}
		}
		opcode = (TFTP_Opcode)Block[1];
		BlockNumber = Block[2];
		BlockNumber = (BlockNumber << 8) + Block[3];
		if((BlockNumber > OldBlockNumber) && opcode == TFTP_DATA)
		{
	
			for(int i=0;i<BlockSizeInt-4;i++)
			{
				BinData[i] = Block[i+4];	
			}
			if(Flash_Write(AppAddress,BinData,BlockSizeInt-4) == Flash_OK) // write block to flash memory
			{
				printf("#");
				AppAddress += (BlockSizeInt-4);
				HAL_Delay(300);
				ACK(BlockNumber);													// send acknowledge to tftp server
				OldBlockNumber = BlockNumber;
				memset(RxBuffer,0,550);
				memset(Block,0,530);
				memset(BinData,0,512);
				HAL_UART_Receive_DMA(&huart1,(uint8_t *)RxBuffer,550);
			}
			else
			{
				printf("Flash Write Error!!!\n");
			}
		}
	if(BlockSizeInt < 516)  // the size of the last block will be less than 512 bytes(  data packet = 2 bytes opcode + 2 bytes block number + 512 bytes data)
		break;
	}
	printf("\n");
	if( BlockNumber == 0)
	{
		printf("Firmware not downloaded!.\n");
	}
	else
	{
		printf("Firmware downloaded.\n");
		NumberOfBytes = (BlockNumber-1)*512+BlockSizeInt-4;
	if(NumberOfBytes < 0)
		NumberOfBytes = 0;
	
		AppSize = NumberOfBytes - 7; // CRC + 4 bytes + 0x0D
	}
		
}

/**
  * @brief  This function sends read request to TFTP server.
  * @param  filename: Name of binary file that will download.
  * @retval None.
  */
void ReadRequest(char *filename)
{
	char txBuffer[50],rxBuffer[10];
	TFTP_Opcode opcode = TFTP_RRQ;
	char *TransferMode = "octet";
	uint8_t TxBufferLength,ESPRespond=0;
	
	TxBufferLength = sprintf(txBuffer,"%c%s%c%s%c",(char)opcode,filename,(char)0,TransferMode,(char)0);
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)rxBuffer,10);
	HAL_UART_Transmit(&huart1,(uint8_t *)txBuffer,sprintf(txBuffer,"AT+CIPSEND=%d\r\n",TxBufferLength),1000);
	uint32_t TimeOut = 5000;  			// 5 seconds timeout
	uint32_t Time = HAL_GetTick();
	while(HAL_GetTick() - Time < TimeOut) // wait until ESP sends the ">" character.(max 5 seconds)
	{
		for(int i=0;i<10;i++)
		{
			if(rxBuffer[i] == '>')
			{
				ESPRespond = 1;
				break;
			}
		}
		if(ESPRespond)
			break;
	}
	HAL_UART_DMAStop(&huart1);
	HAL_UART_Transmit(&huart1,(uint8_t *)txBuffer,sprintf(txBuffer,"%c%c%s%c%s%c",(char)0,(char)opcode,filename,(char)0,TransferMode,(char)0),1000);
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)RxBuffer,550);
	HAL_Delay(200); 
	
}

/**
  * @brief  This function calculates the application's checksum.
  * @retval 32 bits Checksum value.
  */
uint32_t CalculateCRC(void)
{
	if(AppSize == 0)
		return 0xFFFFFFFF;
	
	uint32_t WordData;
	uint16_t WordDataH,WordDataL, MSB, LSB;
	
	__HAL_CRC_DR_RESET(&hcrc);
	for(uint32_t i=0; i<AppSize;i+=4)
	{
		MSB = Read_Flash(APP_START_ADDRESS+i) & 0xFF;  // convert little endian to big endian (0x45E3 -> 0xE345)
		LSB = Read_Flash(APP_START_ADDRESS+i) >> 8;
		WordDataH = (MSB << 8) | LSB;
		MSB = Read_Flash(APP_START_ADDRESS+i+2) & 0xFF;
		LSB = Read_Flash(APP_START_ADDRESS+i+2) >> 8;
		WordDataL = (MSB << 8) | LSB;
		WordData = (WordDataH << 16) | WordDataL;			// combine two 16 bit variables.
		
		hcrc.Instance->DR = WordData;					// Load 32 bit flash data into CRC data register.
		
	}
	return hcrc.Instance->DR;	
}


/**
  * @brief  This function reads the application's checksum from the flash memory.
  * @retval 32 bit Checksum value.
  */
uint32_t getCRCfromFile(void)
{
	if(AppSize == 0)
		return 0xFFFFFFFF;
	
	uint32_t CRCval;
	
	CRCval = Read_Flash(APP_START_ADDRESS + AppSize+3);
	CRCval = (Read_Flash(APP_START_ADDRESS + AppSize+5) << 16) | CRCval;
	
	return CRCval;
}



/* CRC init function */
void CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
void USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
void USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
void DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : GPIO_PIN_13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}
