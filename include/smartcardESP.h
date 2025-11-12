/**
  ******************************************************************************
  This file is responsible for creating a compatibility layer between the HAL
  Smartcard driver and the ESP32 Soft UART implementation. It defines the transmit
  and receive functions for the Smartcard.
  ******************************************************************************
*/
#include <main.h>
#include "smartcard.h"

/**
  * @brief  Send an amount of data in blocking mode.
  * @note   Sends data of the buffer byte per byte until the entire amount is sent or a timeout occurs. Using the softSerial method
  * @param  pData pointer to data buffer.
  * @param  Size amount of data to be sent.
  * @param  Timeout  Timeout duration.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMARTCARD_Transmit(const uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
  * @brief  Receive an amount of data in blocking mode.
  * @note   Receives data into the buffer byte per byte until the entire amount is received or a timeout occurs. Using the softSerial method
  * @param  pData pointer to data buffer.
  * @param  Size amount of data to be received.
  * @param  Timeout Timeout duration.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMARTCARD_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout);