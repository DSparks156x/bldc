#include "stm32f4xx_cryp_manual.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

void cryp_manual_init(void) {
	/* Enable CRYP clock */
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_CRYP, ENABLE);
}

void cryp_manual_aes_ctr_crypt(uint8_t *key, uint8_t *iv, uint8_t *data, uint32_t len) {
	/* 1. AES CTR mode selection */
	CRYP_MANUAL->CR &= ~(CRYP_CR_ALGO_AES_ECB | CRYP_CR_ALGO_AES_CBC | CRYP_CR_ALGO_AES_CTR |
						 CRYP_CR_ALGO_DES_ECB | CRYP_CR_ALGO_DES_CBC | CRYP_CR_ALGO_TDES_ECB |
						 CRYP_CR_ALGO_TDES_CBC);
	CRYP_MANUAL->CR |= CRYP_CR_ALGO_AES_CTR;

	/* 2. AES-128 key size and Data type (8-bit for easy handling) */
	CRYP_MANUAL->CR &= ~(0x0300 | 0x00C0);
	CRYP_MANUAL->CR |= CRYP_CR_KEYSIZE_128B | CRYP_CR_DATATYPE_8B;

	/* 3. Flush FIFOs */
	CRYP_MANUAL->CR |= CRYP_CR_FFLUSH;

	/* 4. Write Key (128-bit = 4 words) */
	CRYP_MANUAL->K0LR = ((uint32_t)key[0] << 24) | ((uint32_t)key[1] << 16) | ((uint32_t)key[2] << 8) | key[3];
	CRYP_MANUAL->K0RR = ((uint32_t)key[4] << 24) | ((uint32_t)key[5] << 16) | ((uint32_t)key[6] << 8) | key[7];
	CRYP_MANUAL->K1LR = ((uint32_t)key[8] << 24) | ((uint32_t)key[9] << 16) | ((uint32_t)key[10] << 8) | key[11];
	CRYP_MANUAL->K1RR = ((uint32_t)key[12] << 24) | ((uint32_t)key[13] << 16) | ((uint32_t)key[14] << 8) | key[15];

	/* 5. Write Initialization Vector (128-bit = 4 words) */
	CRYP_MANUAL->IV0LR = ((uint32_t)iv[0] << 24) | ((uint32_t)iv[1] << 16) | ((uint32_t)iv[2] << 8) | iv[3];
	CRYP_MANUAL->IV0RR = ((uint32_t)iv[4] << 24) | ((uint32_t)iv[5] << 16) | ((uint32_t)iv[6] << 8) | iv[7];
	CRYP_MANUAL->IV1LR = ((uint32_t)iv[8] << 24) | ((uint32_t)iv[9] << 16) | ((uint32_t)iv[10] << 8) | iv[11];
	CRYP_MANUAL->IV1RR = ((uint32_t)iv[12] << 24) | ((uint32_t)iv[13] << 16) | ((uint32_t)iv[14] << 8) | iv[15];

	/* 6. Enable CRYP */
	CRYP_MANUAL->CR |= CRYP_CR_CRYPEN;

	/* 7. Process data in 16-byte blocks */
	uint32_t i;
	for (i = 0; i < len; i += 16) {
		/* Wait for Input FIFO Not Full */
		while (!(CRYP_MANUAL->SR & CRYP_SR_IFNF));

		/* Write 4 words (16 bytes) to Input FIFO */
		for (int j = 0; j < 4; j++) {
			uint32_t word = ((uint32_t)data[i + j*4] << 24) |
							((uint32_t)data[i + j*4 + 1] << 16) |
							((uint32_t)data[i + j*4 + 2] << 8) |
							data[i + j*4 + 3];
			CRYP_MANUAL->DIN = word;
		}

		/* Wait for Output FIFO Not Empty */
		while (!(CRYP_MANUAL->SR & CRYP_SR_OFNE));

		/* Read 4 words (16 bytes) from Output FIFO */
		for (int j = 0; j < 4; j++) {
			uint32_t word = CRYP_MANUAL->DOUT;
			data[i + j*4] = (word >> 24) & 0xFF;
			data[i + j*4 + 1] = (word >> 16) & 0xFF;
			data[i + j*4 + 2] = (word >> 8) & 0xFF;
			data[i + j*4 + 3] = word & 0xFF;
		}
	}

	/* 8. Disable CRYP */
	CRYP_MANUAL->CR &= ~CRYP_CR_CRYPEN;
}
