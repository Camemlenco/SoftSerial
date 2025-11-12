
/**
  ******************************************************************************
  This file is responsible for creating a compatibility layer between the HAL
  Smartcard driver and the ESP32 Soft UART implementation. It defines the transmit
  and receive functions for the Smartcard.
  ******************************************************************************
*/
#include <main.h>
#include <smartcardESP.h>

// Referências externas definidas em main.cpp
extern SoftwareSerial softSerial;
extern uint16_t Di;
extern uint16_t Fi;
extern const int SOFT_SERIAL_PIN;
extern const uint32_t PWM_FREQUENCY1;

/**
  * @brief  Send an amount of data in blocking mode.
  * @note   Sends data of the buffer byte per byte until the entire amount is sent or a timeout occurs. Using the softSerial method
  * @param  pData pointer to data buffer.
  * @param  Size amount of data to be sent.
  * @param  Timeout  Timeout duration.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMARTCARD_Transmit(const uint8_t *pData, uint16_t Size, uint32_t Timeout){
  if(pData == nullptr || Size == 0){
    return HAL_ERROR;
  }

  unsigned long startTime = millis();
  
  // Configura o pino como saída para transmissão
  digitalWrite(SOFT_SERIAL_PIN, HIGH);  // Garante nível alto antes de trocar o modo
  pinMode(SOFT_SERIAL_PIN, OUTPUT);
  
  // Reconfigura softSerial para modo transmissão (Tx)
  softSerial.begin(PWM_FREQUENCY1 * Di / Fi, SWSERIAL_8E1, -1, SOFT_SERIAL_PIN);
  
  // Envia os dados byte por byte
  for(uint16_t i = 0; i < Size; i++){
    // Verifica timeout
    if((millis() - startTime) > Timeout){
      // Retorna ao modo recepção antes de sair
      softSerial.begin(PWM_FREQUENCY1 * Di / Fi, SWSERIAL_8E1, SOFT_SERIAL_PIN, -1);
      return HAL_TIMEOUT;
    }
    
    softSerial.write(pData[i]);
  }
  
  // Aguarda transmissão completar
  softSerial.flush();
  
  // Reconfigura softSerial de volta para modo recepção (Rx)
  softSerial.begin(PWM_FREQUENCY1 * Di / Fi, SWSERIAL_8E1, SOFT_SERIAL_PIN, -1);
  
  return HAL_OK;
}


/**
  * @brief  Receive an amount of data in blocking mode.
  * @note   Receives data into the buffer byte per byte until the entire amount is received or a timeout occurs. Using the softSerial method
  * @param  pData pointer to data buffer.
  * @param  Size amount of data to be received.
  * @param  Timeout Timeout duration.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SMARTCARD_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout){
  if(pData == nullptr || Size == 0){
    return HAL_ERROR;
  }

  unsigned long startTime = millis();
  uint16_t bytesReceived = 0;
  
  // Garante que está em modo recepção
  softSerial.begin(PWM_FREQUENCY1 * Di / Fi, SWSERIAL_8E1, SOFT_SERIAL_PIN, -1);
  
  // Recebe os dados byte por byte
  while(bytesReceived < Size){
    // Verifica timeout
    if((millis() - startTime) > Timeout){
      return HAL_TIMEOUT;
    }
    
    // Verifica se há dados disponíveis
    if(softSerial.available()){
      pData[bytesReceived] = softSerial.read();
      bytesReceived++;
      
      // Reseta o timer a cada byte recebido (timeout entre bytes)
      startTime = millis();
    }
    
    // Pequeno delay para não sobrecarregar o CPU
    delayMicroseconds(10);
  }
  
  return HAL_OK;
}