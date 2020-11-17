#ifndef __IAP_H
#define __IAP_H

#ifdef __cplusplus
 extern "C" {
#endif


#include "stm32f1xx_hal.h"
#include <stdlib.h>
#include <string.h>


#define UserLedOn()			  				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET)
#define UserLedOff()							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET)

#define FLASH_BASE_ADDRESS				0x08000000
#define APP_START_ADDRESS					0x08003C00

typedef enum {
  TFTP_RRQ = 1,
  TFTP_WRQ = 2,
  TFTP_DATA = 3,
  TFTP_ACK = 4,
  TFTP_ERROR = 5
}TFTP_Opcode;

typedef enum {
	Connection_OK = 0,
	Connection_ERROR = 1
}Connection_Status_Typedef;

typedef enum {
	Flash_OK = 0,
	Flash_ERROR = 1
}Flash_Status_Typedef;


void IAP_Init(void);
uint16_t Read_Flash(uint32_t  adr);
Connection_Status_Typedef WifiConnect(char *ssid ,char *password);
Connection_Status_Typedef TFTPServerConnect(char *ipAddress,char *port);
void ReadRequest(char *filename);
Flash_Status_Typedef Flash_Write(uint32_t StartAddress , uint8_t *u8Data,uint16_t Size);
void ACK(uint16_t block);
void ReceiveHandler(void);
uint32_t CalculateCRC(void);
uint32_t getCRCfromFile(void);

void GPIO_Init(void);
void DMA_Init(void);
void USART1_UART_Init(void);
void USART2_UART_Init(void);
void CRC_Init(void);



#ifdef __cplusplus
}
#endif
#endif
