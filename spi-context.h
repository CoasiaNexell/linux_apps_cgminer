#ifndef SPI_CONTEXT_H
#define SPI_CONTEXT_H

#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdbool.h>
#include <stdint.h>
#include "config.h"

#define SPI_DEVICE_TEMPLATE		"/dev/spidev%d.%d"
#define DEFAULT_SPI_BUS			0
#define DEFAULT_SPI_CS_LINE		0
#define DEFAULT_SPI_MODE		SPI_MODE_0
#define DEFAULT_SPI_BITS_PER_WORD	8
#define DEFAULT_SPI_SPEED		1500000
#define DEFAULT_SPI_DELAY_USECS		0
#if defined(USE_BTC08)
#if defined(USE_BTC08_FPGA)
#define MAX_TX_SPI_SPEED         (4*1000*1000)       //  4 MHz
#else
#define MAX_TX_SPI_SPEED         (10*1000*1000)      // 10 MHz
#endif
#endif

struct spi_config {
	int bus;
	int cs_line;
	uint8_t mode;
	uint32_t speed;
	uint8_t bits;
	uint16_t delay;
};

static const struct spi_config default_spi_config = {
	.bus		= DEFAULT_SPI_BUS,
	.cs_line	= DEFAULT_SPI_CS_LINE,
	.mode		= DEFAULT_SPI_MODE,
	.speed		= DEFAULT_SPI_SPEED,
	.bits		= DEFAULT_SPI_BITS_PER_WORD,
	.delay		= DEFAULT_SPI_DELAY_USECS,
};

struct spi_ctx {
	int fd;
	struct spi_config config;
	uint8_t *txb;
};

/* create SPI context with given configuration, returns NULL on failure */
extern struct spi_ctx *spi_init(struct spi_config *config);
/* close descriptor and free resources */
extern void spi_exit(struct spi_ctx *ctx);
/* process RX/TX transfer, ensure buffers are long enough */
extern bool spi_transfer(struct spi_ctx *ctx, uint8_t *txbuf,
			 uint8_t *rxbuf, int len);
extern bool spi_transfer_x20(struct spi_ctx *ctx, uint8_t *txbuf,
			 uint8_t *rxbuf, int len);
extern bool spi_transfer_x20_a(struct spi_ctx *ctx, 
		struct spi_ioc_transfer *xfr, int num);

#endif /* SPI_CONTEXT_H */
