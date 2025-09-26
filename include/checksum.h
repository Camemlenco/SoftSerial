/**
  ******************************************************************************
  * @brief   This file contains all the functions prototypes for checksum library.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CHECKSUM_H
#define __CHECKSUM_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t csum_lrc_compute(uint8_t *data, uint32_t len, uint8_t *rc);
uint8_t csum_crc_compute(uint8_t * data, uint32_t len, uint8_t *rc);

#endif /* __CHECKSUM_H */

