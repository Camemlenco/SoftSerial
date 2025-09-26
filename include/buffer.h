/**
  ******************************************************************************
  * @brief   This file contains all the functions prototypes for buffer library.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUFFER_H
#define __BUFFER_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct ct_buf
{
  uint8_t * base;
  uint32_t head, tail, size;
  uint32_t overrun;
}
buffer_TypeDef;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Buffer_init(buffer_TypeDef *bp, void *buffer, uint32_t len);
void Buffer_set(buffer_TypeDef *bp, void *buffer, uint32_t len);
int32_t  Buffer_get(buffer_TypeDef *bp, void *buffer, uint32_t len);
int32_t  Buffer_put(buffer_TypeDef *bp, const void *buffer, uint32_t len);
int32_t  Buffer_putc(buffer_TypeDef *bp, uint8_t byte);
uint32_t Buffer_avail(buffer_TypeDef *bp);
void * Buffer_head(buffer_TypeDef *bp);
uint8_t ReverseBits(uint8_t v);
void Buffer_reverse(uint8_t* buffer, uint32_t size);

#endif /* __BUFFER_H */

