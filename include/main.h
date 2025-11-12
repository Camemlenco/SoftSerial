/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MAIN_H
#define _MAIN_H

#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <atr.h>

#endif

/* Exported functions prototypes ---------------------------------------------*/
void enableFreeRunningClock(bool comando);
void IRAM_ATTR intInterrupt();
uint8_t sendSPI(uint8_t dataToSend);
void configureVoltage(int voltage);
void pin_flag();
uint8_t calc_LRC(uint8_t* data, size_t length);
uint8_t* encapsulateIBlock(uint8_t* apdu, size_t length);
String sendAPDU(uint8_t* apdu, size_t length, uint8_t* response_uint, size_t response_uint_size);
void printAPDU(String apdu);
void powerDownSmartCard();
uint32_t resetSmartCard(uint8_t* atr, size_t maxLen);
uint8_t pps_request(ATR_TypeDef* p_atr);
uint8_t ifs_request(ATR_TypeDef* p_atr);

