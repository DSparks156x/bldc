#ifndef STM32F4XX_CRYP_MANUAL_H
#define STM32F4XX_CRYP_MANUAL_H

#include <stdint.h>
#include "stm32f4xx.h"

#ifndef CRYP_BASE
#define CRYP_BASE             ((uint32_t)0x50060000)
#endif

#ifndef RCC_AHB2Periph_CRYP
#define RCC_AHB2Periph_CRYP   ((uint32_t)0x00000010)
#endif

/* CRYP register structure */
typedef struct {
	__IO uint32_t CR;    /* CRYP control register,       Address offset: 0x00 */
	__IO uint32_t SR;    /* CRYP status register,        Address offset: 0x04 */
	__IO uint32_t DIN;   /* CRYP data input register,    Address offset: 0x08 */
	__IO uint32_t DOUT;  /* CRYP data output register,   Address offset: 0x0C */
	__IO uint32_t DMACR; /* CRYP DMA control register,   Address offset: 0x10 */
	__IO uint32_t IMSCR; /* CRYP interrupt mask set/clear register, Address offset: 0x14 */
	__IO uint32_t RISR;  /* CRYP raw interrupt status register, Address offset: 0x18 */
	__IO uint32_t MISR;  /* CRYP masked interrupt status register, Address offset: 0x1C */
	__IO uint32_t K0LR;  /* CRYP Key 0 Left register,    Address offset: 0x20 */
	__IO uint32_t K0RR;  /* CRYP Key 0 Right register,   Address offset: 0x24 */
	__IO uint32_t K1LR;  /* CRYP Key 1 Left register,    Address offset: 0x28 */
	__IO uint32_t K1RR;  /* CRYP Key 1 Right register,   Address offset: 0x2C */
	__IO uint32_t K2LR;  /* CRYP Key 2 Left register,    Address offset: 0x30 */
	__IO uint32_t K2RR;  /* CRYP Key 2 Right register,   Address offset: 0x34 */
	__IO uint32_t K3LR;  /* CRYP Key 3 Left register,    Address offset: 0x38 */
	__IO uint32_t K3RR;  /* CRYP Key 3 Right register,   Address offset: 0x3C */
	__IO uint32_t IV0LR; /* CRYP Init Vector 0 Left register,  Address offset: 0x40 */
	__IO uint32_t IV0RR; /* CRYP Init Vector 0 Right register, Address offset: 0x44 */
	__IO uint32_t IV1LR; /* CRYP Init Vector 1 Left register,  Address offset: 0x48 */
	__IO uint32_t IV1RR; /* CRYP Init Vector 1 Right register, Address offset: 0x4C */
} CRYP_Manual_TypeDef;

#define CRYP_MANUAL ((CRYP_Manual_TypeDef *) CRYP_BASE)

/* Control Register bit definitions */
#define CRYP_CR_ALGO_AES_ECB       ((uint32_t)0x00000000)
#define CRYP_CR_ALGO_AES_CBC       ((uint32_t)0x00000008)
#define CRYP_CR_ALGO_AES_CTR       ((uint32_t)0x00000010)
#define CRYP_CR_ALGO_DES_ECB       ((uint32_t)0x00000018)
#define CRYP_CR_ALGO_DES_CBC       ((uint32_t)0x00000020)
#define CRYP_CR_ALGO_TDES_ECB      ((uint32_t)0x00000028)
#define CRYP_CR_ALGO_TDES_CBC      ((uint32_t)0x00000030)

#define CRYP_CR_DATATYPE_32B       ((uint32_t)0x00000000)
#define CRYP_CR_DATATYPE_16B       ((uint32_t)0x00000040)
#define CRYP_CR_DATATYPE_8B        ((uint32_t)0x00000080)
#define CRYP_CR_DATATYPE_1B        ((uint32_t)0x000000C0)

#define CRYP_CR_KEYSIZE_128B       ((uint32_t)0x00000000)
#define CRYP_CR_KEYSIZE_192B       ((uint32_t)0x00000100)
#define CRYP_CR_KEYSIZE_256B       ((uint32_t)0x00000200)

#define CRYP_CR_FFLUSH             ((uint32_t)0x00004000)
#define CRYP_CR_CRYPEN             ((uint32_t)0x00008000)

/* Status Register bit definitions */
#define CRYP_SR_IFEM               ((uint32_t)0x00000001) /* Input FIFO empty */
#define CRYP_SR_IFNF               ((uint32_t)0x00000002) /* Input FIFO not full */
#define CRYP_SR_OFNE               ((uint32_t)0x00000004) /* Output FIFO not empty */
#define CRYP_SR_OFFU               ((uint32_t)0x00000008) /* Output FIFO full */
#define CRYP_SR_BUSY               ((uint32_t)0x00000010) /* Busy bit */

void cryp_manual_init(void);
void cryp_manual_aes_ctr_crypt(uint8_t *key, uint8_t *iv, uint8_t *data, uint32_t len);

#endif /* STM32F4XX_CRYP_MANUAL_H */
