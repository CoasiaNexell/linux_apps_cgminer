/*
 * cgminer SPI driver for BTC08 devices
 *
 * Copyright 2018 Jinyong, Lee <justin@nexell.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>

#include "spi-context.h"
#include "logging.h"
#include "miner.h"
#include "util.h"

#include "btc08-common.h"

#define GPIOA	0
#define GPIOB	32
#define GPIOC	64
#define GPIOD	96
#define GPIOE	128

#define GPIO_HASH0_PLUG         (GPIOA + 24)		// High: Hash0 connected, Low: Hash0 removed
#define GPIO_HASH1_PLUG         (GPIOA + 11)		// High: Hash1 connected, Low: Hash1 removed

#define GPIO_HASH0_BODDET       (GPIOA + 20)		// High: Hash0, Low: VTK
#define GPIO_HASH1_BODDET       (GPIOA +  9)		// High: Hash1, Low: VTK

#define GPIO_HASH0_PWREN        (GPIOA +  0)		// High: FAN ON, Low : FAN OFF
#define GPIO_HASH1_PWREN        (GPIOA + 16)		// High: FAN ON, Low : FAN OFF

#define GPIO_HASH0_OON          (GPIOD + 29)		// ACTIVE_LOW
#define GPIO_HASH0_GLD          (GPIOD + 30)		// ACTIVE_LOW
#define GPIO_HASH0_RST          (GPIOD + 31)		// ACTIVE_LOW

#define GPIO_HASH1_OON          (GPIOE +  2)		// ACTIVE_LOW
#define GPIO_HASH1_GLD          (GPIOE +  3)		// ACTIVE_LOW
#define GPIO_HASH1_RST          (GPIOE +  4)		// ACTIVE_LOW

static struct spi_ctx *spi[MAX_SPI_PORT];
#if defined(USE_BTC08_FPGA)
static int spi_available_bus[MAX_SPI_PORT] = {0};
static int pwren_pin[MAX_SPI_PORT]         = {GPIO_HASH0_PWREN};
static int boddet_pin[MAX_SPI_PORT]        = {GPIO_HASH0_BODDET};
static int plug_pin[MAX_SPI_PORT]          = {GPIO_HASH0_PLUG};
static int reset_pin[MAX_SPI_PORT]         = {GPIO_HASH0_RST};
static int gn_pin[MAX_SPI_PORT]            = {GPIO_HASH0_GLD};
static int oon_pin[MAX_SPI_PORT]           = {GPIO_HASH0_OON};
#else
static int spi_available_bus[MAX_SPI_PORT] = {0, 2};
static int pwren_pin[MAX_SPI_PORT]         = {GPIO_HASH0_PWREN,  GPIO_HASH1_PWREN};
static int boddet_pin[MAX_SPI_PORT]        = {GPIO_HASH0_BODDET, GPIO_HASH1_BODDET};
static int plug_pin[MAX_SPI_PORT]          = {GPIO_HASH0_PLUG,   GPIO_HASH1_PLUG};
static int reset_pin[MAX_SPI_PORT]         = {GPIO_HASH0_RST,    GPIO_HASH1_RST};
static int gn_pin[MAX_SPI_PORT]            = {GPIO_HASH0_GLD,    GPIO_HASH1_GLD};
static int oon_pin[MAX_SPI_PORT]           = {GPIO_HASH0_OON,    GPIO_HASH1_OON};
#endif
static int spi_idx = 0;

/*
 * if not cooled sufficiently, communication fails and chip is temporary
 * disabled. we let it inactive for 30 seconds to cool down
 *
 */
#define COOLDOWN_MS (30 * 1000)
/* if after this number of retries a chip is still inaccessible, disable it */
#define DISABLE_CHIP_FAIL_THRESHOLD	3

struct pll_conf {
	int freq;
	union {
		struct {
			int p        : 6;
			int m        :10;
			int s        : 3;
			int bypass   : 1;
			int div_sel  : 1;
			int afc_enb  : 1;
			int extafc   : 5;
			int feed_en  : 1;
			int fsel     : 1;
			int rsvd     : 3;
		};
		unsigned int val;
	};
};

#define BYPASS      (1)
#define DIV_SEL     (0)

static struct pll_conf pll_sets[] = {
//	{ freq,{p,    m, s, bypass, div_sel, afc_enb, extafc, feed_en, fsel, rsvd}}
	{ 24,  {0,    0, 0, BYPASS, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 50,  {3,  200, 5,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 100, {3,  400, 5,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 150, {2,  200, 4,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 200, {3,  200, 3,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 250, {3,  250, 3,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 300, {2,  200, 3,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 350, {3,  350, 3,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 400, {3,  200, 2,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 450, {2,  150, 2,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 500, {3,  250, 2,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 550, {3,  275, 2,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 600, {2,  200, 2,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 650, {3,  325, 2,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 700, {3,  350, 2,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 750, {3,  375, 2,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 800, {3,  200, 1,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 850, {6,  425, 1,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 900, {2,  150, 1,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{ 950, {6,  475, 1,      0, DIV_SEL, 0, 0, 0, 0, 0}},
	{1000, {6, 1000, 2,      0, DIV_SEL, 0, 0, 0, 0, 0}},
};

#define NUM_PLL_SET (sizeof(pll_sets)/sizeof(struct pll_conf))

/*
 * for now, we have one global config, defaulting values:
 * - ref_clk 16MHz / sys_clk 800MHz
 * - 2000 kHz SPI clock
 */
struct btc08_config_options btc08_config_options = {
	.pll = 550,
	.udiv = (16+1),
	.spi_clk_khz = 2000,             // 2 MHz (minimum spi clock: 1.2 MHz)
	.min_cores = DEFAULT_MIN_CORES,
	.min_chips = DEFAULT_MIN_CHIPS,
	.num_chips = MAX_CHIP_NUM,
	.num_cores = MAX_CORES_PER_CHIP,
};

/* override values with --bitmine-btc08-options ref:sys:spi: - use 0 for default */
static struct btc08_config_options *parsed_config_options;

/********** work queue */
static bool wq_enqueue(struct work_queue *wq, struct work *work)
{
	if (work == NULL)
		return false;
	struct work_ent *we = malloc(sizeof(*we));
	assert(we != NULL);

	we->work = work;
	INIT_LIST_HEAD(&we->head);
	list_add_tail(&we->head, &wq->head);
	wq->num_elems++;
	return true;
}

static struct work *wq_dequeue(struct work_queue *wq)
{
	if (wq == NULL)
		return NULL;
	if (wq->num_elems == 0)
		return NULL;
	struct work_ent *we;
	we = list_entry(wq->head.next, struct work_ent, head);
	struct work *work = we->work;

	list_del(&we->head);
	free(we);
	wq->num_elems--;
	return work;
}

static int32_t get_gpio_value(int pin)
{
	int fd;
	char buf[64];

	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", pin);

	fd = open(buf, O_RDONLY);
	if (0 > fd)
	{
		applog(LOG_ERR, "gpio%d: Failed to open", pin);
		return -1;
	}

	lseek(fd, 0, SEEK_SET);
	if (0 > read(fd, buf, sizeof(buf)))
	{
		close(fd);
		applog(LOG_ERR, "gpio%d: Failed to read", pin);
		return -1;
	}

	close(fd);

	return atoi(buf);
}

static int32_t set_gpio_value(int pin, int val)
{
	int fd, len;
	char buf[64];

	if(val < 0 || val > 1)
	{
		applog(LOG_ERR, "Failed, Check value (%d)\n", val);
		return -1;
	}

	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", pin);

	fd = open(buf, O_WRONLY);
	if (0 > fd)
	{
		applog(LOG_ERR, "gpio%d: Failed to open");
		return -1;
	}

	lseek(fd, 0, SEEK_SET);
	len = snprintf(buf, sizeof(buf), "%d", val);
	if(0 > write(fd, buf, len))
	{
		close(fd);
		applog(LOG_ERR, "gpio%d: Failed to write value %d\n", pin, val);
		return -1;
	}

	close(fd);

	return 0;
}

/* 0x000 : 0V
 * 0xFFF : 1.8V
 * (1.8/4096)xADC = voltage
 *
 * the result must be 0.5V
 * 1.8*adc/4096 = 0.5
 * adc = 0.5*(4096/1.8) ~= 1138
 * min = 0.4*(4096/1.8) ~= 910
 * max = 0.6*(4096/1.8) ~= 1365
 */
#define ad2mV(adc)  ((adc*1800)/4096)

#define HASH_ADC_MIN    910
#define HASH_ADC_MAX    1365

static int get_mvolt(int ch)
{
	int ret = -1;
	int fd, val = 0;
	char adcpath[64];

	sprintf(adcpath, "/sys/bus/iio/devices/iio\\:device0/in_voltage%d_raw", ch);
	fd = open(adcpath, O_RDONLY);

	ret = read(fd, &val, 4);

	ret = ad2mV(val);

	close(fd);

	return ret;
}

static void applog_hexdump(char *prefix, uint8_t *buff, int len, int level)
{
	static char line[512];
	char *pos = line;
	int i;
	if (len < 1)
	{
		return;
	}

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++)
	{
		if (i > 0 && (i % 32) == 0)
		{
			applog(LOG_INFO, "%s", line);
			pos = line;
			pos += sprintf(pos, "\t");
		}
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(level, "%s", line);
}

static void hexdump(char *prefix, uint8_t *buff, int len)
{
	if (opt_btc08_dump) {
		applog_hexdump(prefix, buff, len, LOG_DEBUG);
	}
}

static void hexdump_error(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_ERR);
}

/********** temporary helper for hexdumping SPI traffic */
static void flush_spi(struct btc08_chain *btc08)
{
	int ii;
	memset(btc08->spi_tx, 0, 64);
	bool ret;
	ret = spi_transfer(btc08->spi_ctx, btc08->spi_tx, btc08->spi_rx, 64);
	if(ret == false) {
		btc08->disabled = true;
		applog(LOG_ERR, "%d: %s() error", btc08->chain_id, __func__);
	}
	else
		btc08->disabled = false;
	for(ii=0; ii<64; ii++) btc08->spi_rx[ii] ^= 0xff;
	applog(LOG_DEBUG, "%d: %s()", btc08->chain_id, __func__);
	hexdump("send: TX", btc08->spi_tx, 64);
	hexdump("send: RX", btc08->spi_rx, 64);
}

const char *cmd2str(enum BTC08_command cmd)
{
	switch(cmd)
	{
		case SPI_CMD_READ_ID:           return "SPI_CMD_READ_ID";
		case SPI_CMD_AUTO_ADDRESS:      return "SPI_CMD_AUTO_ADDRESS";
		case SPI_CMD_RUN_BIST:          return "SPI_CMD_RUN_BIST";
		case SPI_CMD_READ_BIST:         return "SPI_CMD_READ_BIST";
		case SPI_CMD_RESET:             return "SPI_CMD_RESET";
		case SPI_CMD_SET_PLL_CONFIG:    return "SPI_CMD_SET_PLL_CONFIG";
		case SPI_CMD_READ_PLL:          return "SPI_CMD_READ_PLL";
		case SPI_CMD_WRITE_PARM:        return "SPI_CMD_WRITE_PARM";
		case SPI_CMD_READ_PARM:         return "SPI_CMD_READ_PARM";
		case SPI_CMD_WRITE_TARGET:      return "SPI_CMD_WRITE_TARGET";
		case SPI_CMD_READ_TARGET:       return "SPI_CMD_READ_TARGET";
		case SPI_CMD_RUN_JOB:           return "SPI_CMD_RUN_JOB";
		case SPI_CMD_READ_JOB_ID:       return "SPI_CMD_READ_JOB_ID";
		case SPI_CMD_READ_RESULT:       return "SPI_CMD_READ_RESULT";
		case SPI_CMD_CLEAR_OON:         return "SPI_CMD_CLEAR_OON";
		case SPI_CMD_SET_DISABLE:       return "SPI_CMD_SET_DISABLE";
		case SPI_CMD_READ_DISABLE:      return "SPI_CMD_READ_DISABLE";
		case SPI_CMD_SET_CONTROL:       return "SPI_CMD_SET_CONTROL";
		case SPI_CMD_DEBUG:             return "SPI_CMD_DEBUG";
		case SPI_CMD_WRITE_NONCE:       return "SPI_CMD_WRITE_NONCE";
		case SPI_CMD_WRITE_CORE_CFG:    return "SPI_CMD_WRITE_CORE_CFG";
		case SPI_CMD_READ_DEBUGCNT:     return "SPI_CMD_READ_DEBUGCNT";
		case SPI_CMD_READ_HASH:         return "SPI_CMD_READ_HASH";
		case SPI_CMD_WRITE_IO_CTRL:     return "SPI_CMD_WRITE_IO_CTRL";
		case SPI_CMD_READ_IO_CTRL:      return "SPI_CMD_READ_IO_CTRL";
		case SPI_CMD_READ_FEATURE:      return "SPI_CMD_READ_FEATURE";
		case SPI_CMD_READ_REVISION:     return "SPI_CMD_READ_REVISION";
		case SPI_CMD_SET_PLL_FOUT_EN:   return "SPI_CMD_SET_PLL_FOUT_EN";
		case SPI_CMD_SET_PLL_RESETB:    return "SPI_CMD_SET_PLL_RESETB";
		case SPI_CMD_WRITE_CORE_DEPTH:  return "SPI_CMD_WRITE_CORE_DEPTH";
		case SPI_CMD_SET_TMODE:         return "SPI_CMD_SET_TMODE";
		default:                        return "UNKNOWN SPI CMD";
	}
}

static bool HWReset(int chain_id)
{
	if ((chain_id < 0) || (chain_id >= MAX_SPI_PORT))
		return false;

	set_gpio_value(reset_pin[chain_id], 0);
	cgsleep_us(1000);
	set_gpio_value(reset_pin[chain_id], 1);

	return true;
}

/********** upper layer SPI functions */
static uint8_t *exec_cmd(struct btc08_chain *btc08,
			  uint8_t cmd, uint8_t chip_id,
			  uint8_t *data, uint8_t parm_len,
			  uint8_t resp_len)
{
	int ii;
	int tx_len = ALIGN((CMD_CHIP_ID_LEN + parm_len + resp_len + DUMMY_BYTES), 4);
	bool ret;
	memset(btc08->spi_tx, 0, tx_len);
	btc08->spi_tx[0] = cmd;
	btc08->spi_tx[1] = chip_id;

	if (data != NULL)
		memcpy(btc08->spi_tx + 2, data, parm_len);

	assert(ret = spi_transfer(btc08->spi_ctx, btc08->spi_tx, btc08->spi_rx, tx_len));
	for(ii=0; ii<tx_len; ii++) btc08->spi_rx[ii] ^= 0xff;
	if(ret == false) {
		btc08->disabled = true;
		applog(LOG_ERR, "%d: %s() error", btc08->chain_id, __func__);
	}
	else
		btc08->disabled = false;

	if (opt_debug) {
		hexdump("send: TX", btc08->spi_tx, tx_len);
		hexdump("send: RX", btc08->spi_rx, tx_len);
	}

	return (btc08->spi_rx + CMD_CHIP_ID_LEN + parm_len);
}

static uint8_t golden_param[WRITE_JOB_LEN] = {
	0x5f, 0x4d, 0x60, 0xa2, 0x53, 0x85, 0xc4, 0x07, 
	0xc2, 0xa8, 0x4e, 0x0c, 0x25, 0x91, 0x69, 0xc4, 
	0x10, 0xa4, 0xa5, 0x4b, 0x93, 0xf7, 0x17, 0x08, 
	0xf1, 0xab, 0xdf, 0xec, 0x6e, 0x8b, 0x81, 0xd2,

	0xf4, 0x2a, 0x1d, 0x6e, 0x5b, 0x30, 0x70, 0x7e, 
	0x17, 0x37, 0x6f, 0x56,

	0x5f, 0x4d, 0x60, 0xa2, 0x53, 0x85, 0xc4, 0x07, 
	0xc2, 0xa8, 0x4e, 0x0c, 0x25, 0x91, 0x69, 0xc4, 
	0x10, 0xa4, 0xa5, 0x4b, 0x93, 0xf7, 0x17, 0x08, 
	0xf1, 0xab, 0xdf, 0xec, 0x6e, 0x8b, 0x81, 0xd2,

	0x5f, 0x4d, 0x60, 0xa2, 0x53, 0x85, 0xc4, 0x07,
	0xc2, 0xa8, 0x4e, 0x0c, 0x25, 0x91, 0x69, 0xc4,
	0x10, 0xa4, 0xa5, 0x4b, 0x93, 0xf7, 0x17, 0x08,
	0xf1, 0xab, 0xdf, 0xec, 0x6e, 0x8b, 0x81, 0xd2,

	0x5f, 0x4d, 0x60, 0xa2, 0x53, 0x85, 0xc4, 0x07, 
	0xc2, 0xa8, 0x4e, 0x0c, 0x25, 0x91, 0x69, 0xc4, 
	0x10, 0xa4, 0xa5, 0x4b, 0x93, 0xf7, 0x17, 0x08, 
	0xf1, 0xab, 0xdf, 0xec, 0x6e, 0x8b, 0x81, 0xd2
};

static uint8_t golden_nonce[NONCE_LEN*2] = {
	0x66, 0xcb, 0x34, 0x26, 0x66, 0xcb, 0x34, 0x26
};

static uint8_t golden_hash[BIST_HASH_LEN] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x22, 0x09, 0x3d, 0xd4, 0x38, 0xed, 0x47, 
	0xfa, 0x28, 0xe7, 0x18, 0x58, 0xb8, 0x22, 0x0d, 
	0x53, 0xe5, 0xcd, 0x83, 0xb8, 0xd0, 0xd4, 0x42,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x22, 0x09, 0x3d, 0xd4, 0x38, 0xed, 0x47, 
	0xfa, 0x28, 0xe7, 0x18, 0x58, 0xb8, 0x22, 0x0d, 
	0x53, 0xe5, 0xcd, 0x83, 0xb8, 0xd0, 0xd4, 0x42,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x22, 0x09, 0x3d, 0xd4, 0x38, 0xed, 0x47, 
	0xfa, 0x28, 0xe7, 0x18, 0x58, 0xb8, 0x22, 0x0d, 
	0x53, 0xe5, 0xcd, 0x83, 0xb8, 0xd0, 0xd4, 0x42,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x22, 0x09, 0x3d, 0xd4, 0x38, 0xed, 0x47, 
	0xfa, 0x28, 0xe7, 0x18, 0x58, 0xb8, 0x22, 0x0d, 
	0x53, 0xe5, 0xcd, 0x83, 0xb8, 0xd0, 0xd4, 0x42
};

static uint8_t golden_target[TARGET_LEN] = {
	0x17, 0x37, 0x6f, 0x56, 0x05, 0x00
};

static uint8_t golden_disable[DISABLE_LEN] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static void disable_cores(struct btc08_chain *btc08, uint8_t chip_id)
{
	uint8_t disable_cores[32] = {0x00,};
	uint8_t disable_core_num = (MAX_CORES_PER_CHIP - btc08_config_options.num_cores);

	applog(LOG_INFO, "btc08_config_options.num_cores:%d disable_core_num:%d",
			btc08_config_options.num_cores, disable_core_num);

	if( disable_core_num > 0 )
	{
		memset(disable_cores, 0xff, 32);
		disable_cores[31] &= ~(1);
		for (int i=1; i<btc08_config_options.num_cores; i++) {
			disable_cores[31-(i/8)] &= ~(1 << (i % 8));
		}
	}
	else
	{
		memset(disable_cores, 0x00, 32);
	}

	exec_cmd(btc08, SPI_CMD_SET_DISABLE, chip_id, disable_cores, DISABLE_LEN, 0);
}

/********** btc08 SPI commands */
static uint8_t *cmd_BIST_BCAST(struct btc08_chain *btc08, uint8_t chip_id)
{
	uint8_t *ret;

	exec_cmd(btc08, SPI_CMD_WRITE_PARM, chip_id, golden_param, WRITE_JOB_LEN, 0);

	exec_cmd(btc08, SPI_CMD_WRITE_TARGET, chip_id, golden_target, TARGET_LEN, 0);

	exec_cmd(btc08, SPI_CMD_WRITE_NONCE, chip_id, golden_nonce, (NONCE_LEN*2), 0);

	disable_cores(btc08, chip_id);

	ret = exec_cmd(btc08, SPI_CMD_RUN_BIST, chip_id, golden_hash, BIST_HASH_LEN, 0);

	return ret;
}

static uint8_t *cmd_RESET_BCAST(struct btc08_chain *btc08)
{
	uint8_t *ret = exec_cmd(btc08, SPI_CMD_RESET, BCAST_CHIP_ID, NULL, 0, 0);
	applog(LOG_INFO, "%d: cmd_RESET_BCAST", btc08->chain_id);

	return ret;
}

static uint8_t *cmd_READ_JOB_ID(struct btc08_chain *btc08, uint8_t chip_id)
{
	int tx_len = ALIGN(CMD_CHIP_ID_LEN + RET_READ_JOB_ID_LEN + DUMMY_BYTES, 4);
	bool ret;
	memset(btc08->spi_tx, 0, tx_len);
	btc08->spi_tx[0] = SPI_CMD_READ_JOB_ID;
	btc08->spi_tx[1] = chip_id;

	assert(ret = spi_transfer(btc08->spi_ctx, btc08->spi_tx, btc08->spi_rx, tx_len));
	for(int ii=0; ii<tx_len; ii++) btc08->spi_rx[ii] ^= 0xff;
	if (opt_debug) {
		hexdump("send: TX", btc08->spi_tx, tx_len);
		hexdump("send: RX", btc08->spi_rx, tx_len);
	}
	if(ret == false) {
		btc08->disabled = true;
		applog(LOG_ERR, "%d: %s() error", btc08->chain_id, __func__);
	}
	else
		btc08->disabled = false;

	return &(btc08->spi_rx[2]);
}

static uint8_t *cmd_READ_RESULT(struct btc08_chain *btc08, uint8_t chip_id)
{
	int tx_len = ALIGN(CMD_CHIP_ID_LEN + RET_READ_RESULT_LEN + DUMMY_BYTES, 4);
	bool ret;
	memset(btc08->spi_tx, 0, tx_len);
	btc08->spi_tx[0] = SPI_CMD_READ_RESULT;
	btc08->spi_tx[1] = chip_id;

	assert(ret = spi_transfer(btc08->spi_ctx, btc08->spi_tx, btc08->spi_rx, tx_len));
	for(int ii=0; ii<tx_len; ii++) btc08->spi_rx[ii] ^= 0xff;
	if(ret == false) {
		btc08->disabled = true;
		applog(LOG_ERR, "%d: %s() error", btc08->chain_id, __func__);
	}
	else
		btc08->disabled = false;

	hexdump("send: TX", btc08->spi_tx, tx_len);
	hexdump("send: RX", btc08->spi_rx, tx_len);

	return &(btc08->spi_rx[2]);
}

static uint8_t *cmd_CLEAR_OON(struct btc08_chain *btc08, uint8_t chip_id)
{
	int tx_len = ALIGN(CMD_CHIP_ID_LEN + DUMMY_BYTES, 4);
	bool ret;
	memset(btc08->spi_tx, 0, tx_len);
	btc08->spi_tx[0] = SPI_CMD_CLEAR_OON;
	btc08->spi_tx[1] = chip_id;

	assert(ret = spi_transfer_x20(btc08->spi_ctx, btc08->spi_tx, btc08->spi_rx, tx_len));
	for(int ii=0; ii<tx_len; ii++) btc08->spi_rx[ii] ^= 0xff;
	hexdump("send: TX", btc08->spi_tx, tx_len);
	hexdump("send: RX", btc08->spi_rx, tx_len);
	if(ret == false) {
		btc08->disabled = true;
		applog(LOG_ERR, "%d: %s() error", btc08->chain_id, __func__);
	}
	else
		btc08->disabled = false;

	return &(btc08->spi_rx[2]);
}

static uint8_t *cmd_READ_HASH(struct btc08_chain *btc08, uint8_t chip_id)
{
	int tx_len = ALIGN(CMD_CHIP_ID_LEN + RET_READ_HASH_LEN + DUMMY_BYTES, 4);
	bool ret;
	memset(btc08->spi_tx, 0, tx_len);
	btc08->spi_tx[0] = SPI_CMD_READ_HASH;
	btc08->spi_tx[1] = chip_id;

	assert(ret = spi_transfer(btc08->spi_ctx, btc08->spi_tx, btc08->spi_rx, tx_len));
	for(int ii=0; ii<tx_len; ii++) btc08->spi_rx[ii] ^= 0xff;
	hexdump("send: TX", btc08->spi_tx, tx_len);
	hexdump("send: RX", btc08->spi_rx, tx_len);
	if(ret == false) {
		btc08->disabled = true;
		applog(LOG_ERR, "%d: %s() error", btc08->chain_id, __func__);
	}
	else
		btc08->disabled = false;

	return &(btc08->spi_rx[2]);
}

static uint8_t *cmd_READ_PARM(struct btc08_chain *btc08, uint8_t chip_id)
{
	int tx_len = ALIGN((CMD_CHIP_ID_LEN + WRITE_JOB_LEN + DUMMY_BYTES), 4);
	bool ret;
	memset(btc08->spi_tx, 0, tx_len);
	btc08->spi_tx[0] = SPI_CMD_READ_PARM;
	btc08->spi_tx[1] = chip_id;

	assert(ret = spi_transfer(btc08->spi_ctx, btc08->spi_tx, btc08->spi_rx, tx_len));
	for(int ii=0; ii<tx_len; ii++) btc08->spi_rx[ii] ^= 0xff;
	if (opt_debug) {
		hexdump("send: TX", btc08->spi_tx, tx_len);
		hexdump("send: RX", btc08->spi_rx, tx_len);
	}
	if(ret == false) {
		btc08->disabled = true;
		applog(LOG_ERR, "%d: %s() error", btc08->chain_id, __func__);
	}
	else
		btc08->disabled = false;

//	for(ii=2; ii<(32+2); ii++)
//		btc08->spi_rx[ii] ^= 0xff;
	return &(btc08->spi_rx[2]);
}

static uint8_t *cmd_READ_ID(struct btc08_chain *btc08, uint8_t chip_id)
{
	uint8_t *ret = exec_cmd(btc08, SPI_CMD_READ_ID, chip_id, NULL, 0, RET_READ_ID_LEN);
	if (ret == NULL || ret[3] != chip_id) {
		applog(LOG_ERR, "%d: cmd_READ_ID chip %d failed",
		       btc08->chain_id, chip_id);
		return NULL;
	}

	return ret;
}

static bool cmd_SET_PLL_RESETB(struct btc08_chain *btc08, uint8_t chip_id, uint8_t reset)
{
	bool ret;
	int tx_len = ALIGN(CMD_CHIP_ID_LEN + PLL_VALUE_LEN + DUMMY_BYTES, 4);

	memset(btc08->spi_tx, 0, tx_len);
	btc08->spi_tx[0] = SPI_CMD_SET_PLL_RESETB;
	btc08->spi_tx[1] = chip_id;
	btc08->spi_tx[2] = 0;
	btc08->spi_tx[3] = (reset&1);       // 0: reset, 1: on

	ret = spi_transfer_x20(btc08->spi_ctx, btc08->spi_tx, btc08->spi_rx, tx_len);
	for(int ii=0; ii<tx_len; ii++) btc08->spi_rx[ii] ^= 0xff;
	if(ret == false) {
		btc08->disabled = true;
		applog(LOG_ERR, "%d: %s() error", btc08->chain_id, __func__);
	}
	else
		btc08->disabled = false;

	return ret;
}

static bool cmd_SET_PLL_FOUT_EN(struct btc08_chain *btc08, uint8_t chip_id, uint8_t fout)
{
	bool ret;
	int tx_len = ALIGN(CMD_CHIP_ID_LEN + PLL_VALUE_LEN + DUMMY_BYTES, 4);

	memset(btc08->spi_tx, 0, tx_len);
	btc08->spi_tx[0] = SPI_CMD_SET_PLL_FOUT_EN;
	btc08->spi_tx[1] = chip_id;
	btc08->spi_tx[2] = 0;
	btc08->spi_tx[3] = (fout&1);		// 0: disable, 1: enable

	ret = spi_transfer_x20(btc08->spi_ctx, btc08->spi_tx, btc08->spi_rx, tx_len);
	for(int ii=0; ii<tx_len; ii++) btc08->spi_rx[ii] ^= 0xff;
	if(ret == false) {
		btc08->disabled = true;
		applog(LOG_ERR, "%d: %s() error", btc08->chain_id, __func__);
	}
	else
		btc08->disabled = false;

	return ret;
}

static bool cmd_SET_PLL_CONFIG(struct btc08_chain *btc08, uint8_t chip_id, uint8_t idx)
{
	bool ret;
	int tx_len = ALIGN(CMD_CHIP_ID_LEN + PLL_VALUE_LEN + DUMMY_BYTES, 4);

	memset(btc08->spi_tx, 0, tx_len);
	btc08->spi_tx[0] = SPI_CMD_SET_PLL_CONFIG;
	btc08->spi_tx[1] = chip_id;
	btc08->spi_tx[2] = (uint8_t)(pll_sets[idx].val>>24)&0xff;
	btc08->spi_tx[3] = (uint8_t)(pll_sets[idx].val>>16)&0xff;
	btc08->spi_tx[4] = (uint8_t)(pll_sets[idx].val>> 8)&0xff;
	btc08->spi_tx[5] = (uint8_t)(pll_sets[idx].val>> 0)&0xff;

	ret = spi_transfer_x20(btc08->spi_ctx, btc08->spi_tx, btc08->spi_rx, tx_len);
	for(int ii=0; ii<tx_len; ii++) btc08->spi_rx[ii] ^= 0xff;
	if(ret == false) {
		btc08->disabled = true;
		applog(LOG_ERR, "%d: %s() error", btc08->chain_id, __func__);
	}
	else
		btc08->disabled = false;

	return ret;
}

static uint8_t *cmd_READ_PLL(struct btc08_chain *btc08, uint8_t chip_id)
{
	uint8_t *ret = exec_cmd(btc08, SPI_CMD_READ_PLL, chip_id, NULL, 0, RET_READ_PLL_LEN);
	if (ret == NULL) {
		applog(LOG_ERR, "%d: SPI_CMD_READ_PLL chip %d failed",
		       btc08->chain_id, chip_id);
		return NULL;
	}

	return ret;
}

static uint32_t nbits_from_target(unsigned char *target)
{
	uint32_t ret = 0;
	int ii= 31;

	while(target[ii--]==0);
	ii++;

	if(target[ii-2] == 0) ii++;

	ret = (ii+1)<<24;
	ret |= target[ii-0]<<16;
	ret |= target[ii-1]<< 8;
	ret |= target[ii-2]<< 0;

	return ret;
}

static uint8_t cmd_WRITE_JOB_test(struct btc08_chain *btc08, uint8_t job_id, uint8_t *job, uint8_t chip_id)
{
	int ii=0;
	bool retb;
	/* ensure we push the SPI command to the last chip in chain */
	uint8_t *spi_tx = job;
	struct spi_ioc_transfer *xfr = btc08->xfr;

	int tx_len;

	tx_len = ALIGN((CMD_CHIP_ID_LEN + WRITE_JOB_LEN + DUMMY_BYTES), 4);
	// WRITE_PARM
	hexdump("send: TX", spi_tx, tx_len);
	xfr[0].tx_buf = (unsigned long)spi_tx;
	xfr[0].rx_buf = (unsigned long)NULL;
	xfr[0].len = tx_len;
	xfr[0].speed_hz = MAX_TX_SPI_SPEED;
	xfr[0].delay_usecs = btc08->spi_ctx->config.delay;
	xfr[0].bits_per_word = btc08->spi_ctx->config.bits;
	xfr[0].tx_nbits = 0;
	xfr[0].rx_nbits = 0;
	xfr[0].pad = 0;
	spi_tx += tx_len;

	// CLEAR_OON
	tx_len = ALIGN((CMD_CHIP_ID_LEN + DUMMY_BYTES), 4);
	spi_tx[0] = SPI_CMD_CLEAR_OON;
	spi_tx[1] = 0;
	hexdump("send: TX", spi_tx, tx_len);
	xfr[1].tx_buf = (unsigned long)spi_tx;
	xfr[1].rx_buf = (unsigned long)NULL;
	xfr[1].len = tx_len;
	xfr[1].speed_hz = MAX_TX_SPI_SPEED;
	xfr[1].delay_usecs = btc08->spi_ctx->config.delay;
	xfr[1].bits_per_word = btc08->spi_ctx->config.bits;
	xfr[1].tx_nbits = 0;
	xfr[1].rx_nbits = 0;
	xfr[1].pad = 0;
	spi_tx += tx_len; 

	ii = 2;

	// WRITE_TARGET
	tx_len = ALIGN((CMD_CHIP_ID_LEN + TARGET_LEN + DUMMY_BYTES), 4);
	spi_tx[0] = SPI_CMD_WRITE_TARGET;
	spi_tx[1] = 0;
	spi_tx[2] = 0x19;
	spi_tx[3] = 0;
	spi_tx[4] = 0x89;
	spi_tx[5] = 0x6c;
	spi_tx[6] = 0x05;
	spi_tx[7] = 0x10;
	spi_tx[8] = 0;
	hexdump("send: TX", spi_tx, tx_len);
	xfr[ii].tx_buf = (unsigned long)spi_tx;
	xfr[ii].rx_buf = (unsigned long)NULL;
	xfr[ii].len = tx_len;
	xfr[ii].speed_hz = MAX_TX_SPI_SPEED;
	xfr[ii].delay_usecs = btc08->spi_ctx->config.delay;
	xfr[ii].bits_per_word = btc08->spi_ctx->config.bits;
	xfr[ii].tx_nbits = 0;
	xfr[ii].rx_nbits = 0;
	xfr[ii].pad = 0;

	ii += 1;
	spi_tx += tx_len;

	// RUN_JOB
	tx_len = ALIGN((CMD_CHIP_ID_LEN + JOB_ID_LEN + DUMMY_BYTES), 4);
	spi_tx[0] = SPI_CMD_RUN_JOB;
	spi_tx[1] = chip_id;
	spi_tx[2] = 0;
	spi_tx[3] = job_id;
	hexdump("send: TX", spi_tx, tx_len);
	xfr[ii].tx_buf = (unsigned long)spi_tx;
	xfr[ii].rx_buf = (unsigned long)NULL;
	xfr[ii].len = tx_len;
	xfr[ii].speed_hz = MAX_TX_SPI_SPEED;
	xfr[ii].delay_usecs = btc08->spi_ctx->config.delay;
	xfr[ii].bits_per_word = btc08->spi_ctx->config.bits;
	xfr[ii].tx_nbits = 0;
	xfr[ii].rx_nbits = 0;
	xfr[ii].pad = 0;

	ii += 1;

	assert(retb = spi_transfer_x20_a(btc08->spi_ctx, btc08->xfr, ii));
	if(retb == false) {
		btc08->disabled = true;
		applog(LOG_ERR, "%d: %s() error", btc08->chain_id, __func__);
	}
	else
		btc08->disabled = false;

	return true;
}

/* calculate btc08 Tartet and Select from nBits */
void calc_btc08_target(uint8_t *dest_target, uint32_t nbits)
{
	uint32_t *nbits_ptr = (uint32_t *)dest_target;
	*nbits_ptr = bswap_32(nbits);
	hexdump("target", dest_target, 4);

	uint8_t select0, select1, shift = 0;
	select0 = (dest_target[0] / 4) - 1;
	select1 = (dest_target[0] % 4) + 1;
	dest_target[4] = select0;
	dest_target[5] = select1<<4 | (shift&0xF);
	hexdump("select", dest_target+4, 2);
}

static uint8_t cmd_WRITE_JOB_fast(struct btc08_chain *btc08,
			      uint8_t job_id, uint8_t *job, struct work *work)
{
	int ii=0;
	bool retb;
	uint32_t nbits;
	uint8_t btc08_target[6] = {0x00,};
	int tx_len;
	/* ensure we push the SPI command to the last chip in chain */
	uint8_t *spi_tx = job;
	struct spi_ioc_transfer *xfr = btc08->xfr;

	tx_len = ALIGN((CMD_CHIP_ID_LEN + WRITE_JOB_LEN + DUMMY_BYTES), 4);

	// WRITE_PARM
	hexdump("[WRITE_PARM]", spi_tx, tx_len);
	xfr[0].tx_buf = (unsigned long)spi_tx;
	xfr[0].rx_buf = (unsigned long)NULL;
	xfr[0].len = tx_len;
	xfr[0].speed_hz = MAX_TX_SPI_SPEED;
	xfr[0].delay_usecs = btc08->spi_ctx->config.delay;
	xfr[0].bits_per_word = btc08->spi_ctx->config.bits;
	xfr[0].cs_change = 1;
	xfr[0].tx_nbits = 0;
	xfr[0].rx_nbits = 0;
	xfr[0].pad = 0;
	spi_tx += tx_len;

	ii++;

	// WRITE_TARGET
	if(btc08->sdiff != work->sdiff)
	{
		tx_len = ALIGN((CMD_CHIP_ID_LEN + TARGET_LEN + DUMMY_BYTES), 4);

		spi_tx[0] = SPI_CMD_WRITE_TARGET;
		spi_tx[1] = BCAST_CHIP_ID;

		/* target */
		btc08->sdiff = work->sdiff;
		nbits = nbits_from_target(work->target);
		calc_btc08_target(btc08_target, nbits);
		memcpy(spi_tx+2, btc08_target, 6);
		spi_tx[8] = 0;
		hexdump("[WRITE_TARGET]", spi_tx, tx_len);
		hexdump("target", work->target, 32);
		applog(LOG_ERR, "diff : %.2f", btc08->sdiff);

		xfr[ii].tx_buf = (unsigned long)spi_tx;
		xfr[ii].rx_buf = (unsigned long)NULL;
		xfr[ii].len = tx_len;
		xfr[ii].speed_hz = MAX_TX_SPI_SPEED;
		xfr[ii].delay_usecs = btc08->spi_ctx->config.delay;
		xfr[ii].bits_per_word = btc08->spi_ctx->config.bits;
		xfr[ii].cs_change = 1;
		xfr[ii].tx_nbits = 0;
		xfr[ii].rx_nbits = 0;
		xfr[ii].pad = 0;

		ii += 1;
		spi_tx += tx_len;
	}

	// RUN_JOB
	tx_len = ALIGN((CMD_CHIP_ID_LEN + JOB_ID_LEN + DUMMY_BYTES), 4);

	spi_tx[0] = SPI_CMD_RUN_JOB;
	spi_tx[1] = BCAST_CHIP_ID;
	spi_tx[2] = 0;
	if (work->pool->vmask)
		spi_tx[2] |= ASIC_BOOST_EN;
	spi_tx[3] = job_id;
	hexdump("[RUN_JOB]", spi_tx, tx_len);
	xfr[ii].tx_buf = (unsigned long)spi_tx;
	xfr[ii].rx_buf = (unsigned long)NULL;
	xfr[ii].len = tx_len;
	xfr[ii].speed_hz = MAX_TX_SPI_SPEED;
	xfr[ii].delay_usecs = btc08->spi_ctx->config.delay;
	xfr[ii].bits_per_word = btc08->spi_ctx->config.bits;
	xfr[ii].cs_change = 1;
	xfr[ii].tx_nbits = 0;
	xfr[ii].rx_nbits = 0;
	xfr[ii].pad = 0;

	ii += 1;

	assert(retb = spi_transfer_x20_a(btc08->spi_ctx, btc08->xfr, ii));
	if(retb == false) {
		btc08->disabled = true;
		applog(LOG_ERR, "%d: %s() error", btc08->chain_id, __func__);
	}
	else
		btc08->disabled = false;

	return true;
}

/********** btc08 low level functions */
#define MAX_PLL_WAIT_CYCLES 25
#define PLL_CYCLE_WAIT_TIME 40
static bool check_chip_pll_lock(struct btc08_chain *btc08, int chip_id)
{
#if !defined(USE_BTC08_FPGA)
	int n;
	int cid = btc08->chain_id;
	uint8_t *ret;

	for (n = 0; n < MAX_PLL_WAIT_CYCLES; n++) {
		/* check for PLL lock status */
		ret = exec_cmd(btc08, SPI_CMD_READ_PLL, chip_id, NULL, 0, RET_READ_PLL_LEN);
		if (ret == NULL) {
			applog(LOG_WARNING, "%d: error in READ_PLL", cid);
			return false;
		}
		if(ret[1]&(1<<7)) {
			applog(LOG_ERR, "%d: succeed to lock PLL on chip %d", cid, chip_id);
			return true;
		}

		cgsleep_ms(PLL_CYCLE_WAIT_TIME);
	}
	applog(LOG_ERR, "%d: failed to lock PLL on chip %d", cid, chip_id);
	return false;
#else
	int cid = btc08->chain_id;
	applog(LOG_WARNING, "%d: PLL locked %d(0x%x)CHIP", cid, chip_id, chip_id);
	return true;
#endif
}

static int get_pll_idx(int pll_freq)
{
	int ret;
	struct pll_conf *plls;

	if(pll_freq < pll_sets[0].freq)
		return -1;
	if(pll_freq > pll_sets[NUM_PLL_SET-1].freq) {
		applog(LOG_WARNING, "set to Max Frequency setting (%d)", pll_sets[NUM_PLL_SET-1].freq);
		return (NUM_PLL_SET-1);
	}

	ret = 0;
	for(plls = &pll_sets[0]; plls; plls++) {
		if(pll_freq <= plls->freq) break;
		ret++;
	}
	return ret;
}

static void set_pll_config_by_idx(struct btc08_chain *btc08, int chipId, int pll_idx)
{
	// seq1. Disable FOUT
	cmd_SET_PLL_FOUT_EN(btc08, chipId, FOUT_EN_DISABLE);

	// seq3. Set PLL(change PMS value)
	cmd_SET_PLL_CONFIG(btc08, chipId, pll_idx);

	// seq2. Down reset
	cmd_SET_PLL_RESETB(btc08, chipId, RESETB_RESET);

	// seq4. Up reset
	cmd_SET_PLL_RESETB(btc08, chipId, RESETB_ON);

	// seq4. wait for 1 ms
	cgsleep_ms(1);

	// seq5. Enable FOUT
	cmd_SET_PLL_FOUT_EN(btc08, chipId, FOUT_EN_ENABLE);
}

/*
 * FPGA: 50MHz
 * ASIC: 50~1000MHz
*/
static bool set_pll_config(struct btc08_chain *btc08, int chip_id, int pll)
{
	int ii;
	uint8_t sbuf[4];
	int cid = btc08->chain_id;

	int chip_index = chip_id -1;
	if(btc08->last_chip)
		chip_index += (btc08->last_chip-1);

	if(((btc08->chips[btc08->num_chips-1].rev >> 8) & 0xf) == FEATURE_FOR_FPGA)
	{
		btc08->timeout_oon = TIME_LIMIT_OF_OON_FPGA;
		if(chip_id) {
			btc08->chips[chip_index].mhz = FPGA_MINER_CORE_CLK;
			applog(LOG_WARNING, "%d: chip%d: skip PLL because FPGA", cid, chip_index);
		}
		else {
			for(ii=btc08->last_chip; ii<btc08->num_chips; ii++)
				btc08->chips[ii].mhz = FPGA_MINER_CORE_CLK;
			applog(LOG_WARNING, "%d: chip%d~%d: skip PLL because FPGA", cid, btc08->last_chip, (btc08->num_chips-1));
		}
	}
	else {
		int pll_idx;
		btc08->timeout_oon = TIME_LIMIT_OF_OON;
		pll_idx = get_pll_idx(pll);
		if(pll_idx < 0) {
			applog(LOG_ERR, "%d: too low frequency (%d), it must be over than %d", pll, pll_sets[0].freq);
			return false;
		}

		set_pll_config_by_idx(btc08, chip_id, pll_idx);
		if(chip_id) {
			if (!check_chip_pll_lock(btc08, chip_id)) {
				applog(LOG_ERR, "%d: chip %d (chip_id:%d) failed PLL lock",
					   cid, chip_index, chip_id);
				btc08->chips[chip_index].mhz = 0;
				return false;
			}
			else
				btc08->chips[chip_index].mhz = pll_sets[pll_idx].freq;
		}
		else {
			int res = 0;
			for(ii=btc08->last_chip; ii<btc08->num_chips; ii++) {
				int chipid = ii +1;
				if(btc08->last_chip)
					chipid += (1-btc08->last_chip);

				if (!check_chip_pll_lock(btc08, chipid)) {
					applog(LOG_ERR, "%d: chip %d (chipid:%d) failed PLL lock",
						   cid, ii, chipid);
					btc08->chips[ii].mhz = 0;
					res++;
				}
				else
					btc08->chips[ii].mhz = pll_sets[pll_idx].freq;
			}
			if(res) return false;
		}
	}
	return true;
}

static bool set_control(struct btc08_chain *btc08, int chip_id, int udiv)
{
	uint8_t sbuf[4];

	sbuf[0] = (uint8_t)(udiv>>24)&0xff;
	sbuf[1] = (uint8_t)(udiv>>16)&0xff;
	sbuf[2] = (uint8_t)(udiv>> 8)&0xff;
	sbuf[3] = (uint8_t)(udiv>> 0)&0xff;

	if (opt_debug)
	{
		if (sbuf[3] & LAST_CHIP)
			applog(LOG_WARNING, "set a last chip (chip_id %d)", chip_id);
		if (sbuf[3] & OON_IRQ_EN)
			applog(LOG_WARNING, "set OON IRQ Enable");
	}

	exec_cmd(btc08, SPI_CMD_SET_CONTROL, chip_id, sbuf, sizeof(sbuf), 0);

	return true;
}

static bool check_chip(struct btc08_chain *btc08, int chip_id)
{
	int cid = btc08->chain_id;
	int chip_index = chip_id - 1;
	uint8_t *ret;

	// READ_BIST to check the number of cores of the active chip
	for (int retry_cnt = 0; retry_cnt < 10; retry_cnt++)
	{
		ret = exec_cmd(btc08, SPI_CMD_READ_BIST, chip_id, NULL, 0, RET_READ_BIST_LEN);
		if ((ret[0] & 1) == BIST_STATUS_IDLE)
			break;

		cgsleep_ms(200);
	}
	ret = exec_cmd(btc08, SPI_CMD_READ_BIST, chip_id, NULL, 0, RET_READ_BIST_LEN);
	if ((ret[0] & 1) == BIST_STATUS_BUSY) {
		applog(LOG_ERR, "%d: error in READ_BIST", cid);
		return false;
	}
	btc08->chips[chip_index].num_cores = ret[1];

	// Calculate the performance of each chip
	if (((btc08->chips[chip_index].rev >> 8) & 0xf) != FEATURE_FOR_FPGA) {
		if(btc08->chips[chip_index].num_cores < btc08_config_options.min_cores) {
			applog(LOG_ERR, "%d: failed. chip %d has not enough cores(%d), it must be over than %d",
					cid, chip_id, btc08->chips[chip_index].num_cores, btc08_config_options.min_cores);
			btc08->chips[chip_index].num_cores = 0;
			btc08->chips[chip_index].perf = 0;
			return false;
		}
	}
	applog(LOG_DEBUG, "%d: Found chip %d(chipid:%d) with %d active cores",
	       cid, chip_index, chip_id, btc08->chips[chip_index].num_cores);

	btc08->chips[chip_index].perf = btc08->chips[chip_index].num_cores*btc08->chips[chip_index].mhz;
	applog(LOG_DEBUG, "%d: chip %d perf = %ld (%ld MHz)\n", cid, chip_id, btc08->chips[chip_index].perf, btc08->chips[chip_index].mhz);

	return true;
}

static bool calc_nonce_range(struct btc08_chain *btc08)
{
	int ii;
	uint8_t *spi_tx = btc08->spi_tx;
	uint32_t *start_nonce_ptr;
	uint32_t *  end_nonce_ptr;
	bool ret;

	if(btc08_config_options.test_mode == 1) {
		for(ii=btc08->last_chip; ii<btc08->num_chips; ii++) {
			btc08->chips[ii].start_nonce = 0;
			btc08->chips[ii].end_nonce = MAX_NONCE_SIZE;
		}
	}
	else {
		btc08->chips[btc08->last_chip].start_nonce = 0;
		for(ii=btc08->last_chip; ii<(btc08->num_chips-1); ii++) {
			btc08->chips[ii].end_nonce = btc08->chips[ii].start_nonce
				+ ((MAX_NONCE_SIZE*btc08->chips[ii].perf)/btc08->perf);
			btc08->chips[ii+1].start_nonce = btc08->chips[ii].end_nonce+1;
		}
		btc08->chips[btc08->num_chips-1].end_nonce = MAX_NONCE_SIZE;
	}

	btc08->disabled = false;

	for(ii=btc08->last_chip; ii<btc08->num_chips; ii++) {
		int tx_len = 0;
		int chip_id = ii +1;
		if(btc08->last_chip)
			chip_id += (1 -btc08->last_chip);
		applog(LOG_DEBUG, "chip %d(chip_index:%d) : %08X ~ %08X",
				chip_id, ii, btc08->chips[ii].start_nonce, btc08->chips[ii].end_nonce);

		start_nonce_ptr = (uint32_t *)(spi_tx+2  );
		end_nonce_ptr   = (uint32_t *)(spi_tx+2+4);
		spi_tx[0] = SPI_CMD_WRITE_NONCE;

		*start_nonce_ptr = bswap_32(btc08->chips[ii].start_nonce);
		  *end_nonce_ptr = bswap_32(btc08->chips[ii].end_nonce);
		spi_tx[1] = ii+1;
		spi_tx[10] = 0;

		tx_len = ALIGN((CMD_CHIP_ID_LEN + (NONCE_LEN*2) + DUMMY_BYTES), 4);
		ret = spi_transfer(btc08->spi_ctx, btc08->spi_tx, btc08->spi_rx, tx_len);
		for(int i=0; i<tx_len; i++) btc08->spi_rx[i] ^= 0xff;
		hexdump("send: TX", btc08->spi_tx, tx_len);
		hexdump("send: RX", btc08->spi_rx, tx_len);
		if(ret == false) {
			btc08->disabled = true;
			applog(LOG_ERR, "%d: %s() error", btc08->chain_id, __func__);
			break;
		}
	}

	return true;
}

static void reset_gpio(struct btc08_chain *btc08, int on)
{
	int fd_gpio;
	char gpio_str[64];
	sprintf(gpio_str, "/sys/class/gpio/gpio%d/value", btc08->pinnum_gpio_reset);

	fd_gpio = open(gpio_str, O_WRONLY);
	if(fd_gpio<0) {
		applog(LOG_ERR, "%d: %s(%d) error, open error (gpio:%d)", btc08->chain_id, __func__, on, btc08->pinnum_gpio_reset);
		return;
	}
	if(on)
		write(fd_gpio, "1", 2);	// reset
	else
		write(fd_gpio, "0", 2);	// reset
	close(fd_gpio);
}

static int test_spi_seq(struct btc08_chain *btc08)
{
	int ii=0;
	bool retb;
	/* ensure we push the SPI command to the last chip in chain */
	uint8_t spi_tx_b[128], spi_rx_b[128];
	uint8_t *spi_tx = spi_tx_b, *spi_rx = spi_rx_b;
	struct spi_ioc_transfer *xfr = btc08->xfr;
	int tx_len;
	int param_len = 0, resp_len = 4;

	tx_len = ALIGN((CMD_CHIP_ID_LEN + param_len + resp_len + DUMMY_BYTES), 4);
	for (ii=0; ii<btc08->num_active_chips; ii++) {
		memset(spi_tx, 0, tx_len);
		memset(spi_rx, 0, tx_len);
		spi_tx[0] = SPI_CMD_READ_REVISION;
		spi_tx[1] = (ii%3) + 1;
		hexdump("send: TX", spi_tx, tx_len);
		xfr[ii].tx_buf = (unsigned long)spi_tx;
		xfr[ii].rx_buf = (unsigned long)spi_rx;
		xfr[ii].len = tx_len;
		xfr[ii].speed_hz = MAX_TX_SPI_SPEED;
		xfr[ii].delay_usecs = btc08->spi_ctx->config.delay;
		xfr[ii].bits_per_word = btc08->spi_ctx->config.bits;
		xfr[ii].cs_change = 1;
		xfr[ii].tx_nbits = 0;
		xfr[ii].rx_nbits = 0;
		xfr[ii].pad = 0;
		spi_tx += tx_len;
		spi_rx += tx_len;
	}

	assert(retb = spi_transfer_x20_a(btc08->spi_ctx, btc08->xfr, ii));
	if(retb == false) {
		btc08->disabled = true;
		applog(LOG_ERR, "%d: %s() error", btc08->chain_id, __func__);
	}
	else
		btc08->disabled = false;

	for(ii=0; ii<btc08->num_active_chips; ii++) {
		int jj;
		spi_rx = (uint8_t *)xfr[ii].rx_buf;
		for(jj=0; jj<tx_len; jj++)
			spi_rx[jj] ^= 0xff;
		hexdump("send: RX", spi_rx, tx_len);
	}

	return true;
}

// Read the number of chips
static int chain_detect(struct btc08_chain *btc08)
{
	uint8_t dummy[32]={0x00,};
	int cid = btc08->chain_id;
	uint8_t *ret;
	int active_chips = 0;

	cmd_RESET_BCAST(btc08);

	// AUTO_ADDRESS to read the number of chips
	ret = exec_cmd(btc08, SPI_CMD_AUTO_ADDRESS, BCAST_CHIP_ID, dummy, sizeof(dummy), RET_AUTO_ADDRESS_LEN);
	if(ret[0] != SPI_CMD_AUTO_ADDRESS) {
		applog(LOG_WARNING, "%d: error in AUTO_ADDRESS(%d)", cid, ret[0]);
		return 0;
	}
	btc08->num_chips = ret[1];

	// READ_ID to check if each chip is active
	for(int chipId = btc08->num_chips; chipId >= 1; chipId--) {
		ret = exec_cmd(btc08, SPI_CMD_READ_ID, chipId, NULL, 0, RET_READ_ID_LEN);
		if(ret[3] == chipId)
			active_chips++;
		else {
			applog(LOG_WARNING, "%d: error in READ_ID(%d;%d)", cid, chipId, ret[3]);
			break;
		}
	}
	btc08->num_active_chips = active_chips;

	/* Don't start mining if even one of the entire chips has a problem */
	if (btc08->num_chips != btc08->num_active_chips)
	{
		btc08->num_chips = 0;
		btc08->num_active_chips = 0;
	}
#if 0
	// SET_LAST_CHIP
	if (btc08->num_chips != btc08->num_active_chips)
	{
		btc08->last_chip = (chipId + 1);
		for(int chipId=1 ; chipId <= btc08->num_active_chips ; chipId++)
		{
			if( chipId == last_chip_id )
				set_control(btc08, last_chipId, (LAST_CHIP | btc08_config_options.udiv));
			else
				set_control(btc08, last_chipId, (btc08_config_options.udiv));
		}
		ret = exec_cmd(btc08, SPI_CMD_AUTO_ADDRESS, BCAST_CHIP_ID, dummy, sizeof(dummy), RET_AUTO_ADDRESS_LEN);
		if(ret[0] != SPI_CMD_AUTO_ADDRESS) {
			applog(LOG_WARNING, "%d: error2 in AUTO_ADDRESS", cid);
			return 0;
		}
		btc08->num_chips = ret[1];
		btc08->num_active_chips = 0;

		// READ_ID to check if each chip is active
		for(chipId = btc08->num_chips; chipId >= 1; chipId--) {
			ret = exec_cmd(btc08, SPI_CMD_READ_ID, chipId, NULL, 0, RET_READ_ID_LEN);
			if(ret[3] == chipId)
				btc08->num_active_chips++;
			else {
				applog(LOG_WARNING, "%d: error2 in READ_ID(%d;%d)", cid, chipId, ret[3]);
				break;
			}
		}
	}
#endif

	applog(LOG_WARNING, "%d: detected %d chips", cid, btc08->num_chips);

	return btc08->num_chips;
}

/********** disable / re-enable related section (temporary for testing) */
static int get_current_ms(void)
{
	cgtimer_t ct;
	cgtimer_time(&ct);
	return cgtimer_to_ms(&ct);
}

static bool is_chip_disabled(struct btc08_chain *btc08, uint8_t chip_index)
{
	struct btc08_chip *chip = &btc08->chips[chip_index];
	return chip->disabled || chip->cooldown_begin != 0;
}

/* check and disable chip, remember time */
static void disable_chip(struct btc08_chain *btc08, uint8_t chip_index)
{
	struct btc08_chip *chip = &btc08->chips[chip_index];
	int cid = btc08->chain_id;
	if (is_chip_disabled(btc08, chip_index)) {
		applog(LOG_WARNING, "%d: chip %d already disabled",
		       cid, chip_index);
		return;
	}
	applog(LOG_WARNING, "%d: temporary disabling chip %d", cid, chip_index);
	chip->cooldown_begin = get_current_ms();
}

static bool set_last_chip(struct btc08_chain *btc08, int last_chip)
{
	uint8_t *ret;
	uint8_t dummy[32];
	int ii, chip_id, abs_last_chip;

	// if btc08->last_chip == 0, new_last_id == 3
	//     index   = 0, 1, 2, 3, 4, 5, 6, ...
	//     chip_id = 1, 2, 3, 4, 5, 6, 7, ...
	// loop          1, 2, 2
	// 
	// if btc08->last_chip == 2, new_last_id == 3
	//     index   = 0, 1, 2, 3, 4, 5, 6, ...
	//     chip_id = 1, 1, 2, 3, 4, 5, 6, ...
	// loop                2, 2, 2
	// btc08->last_chip can be used as chip_index

	for(ii=0; ii<last_chip; ii++) {
		struct btc08_chip *chip = &btc08->chips[btc08->last_chip];
		if(btc08->last_chip)
			chip_id = 1;
		else
			chip_id = 2;
		if(!set_control(btc08, chip_id, (btc08_config_options.udiv|LAST_CHIP))) {
			btc08->disabled = true;
			return false;
		}

		// SPI_CMD_AUTO_ADDRESS
		ret = exec_cmd(btc08, SPI_CMD_AUTO_ADDRESS, BCAST_CHIP_ID,
						dummy, sizeof(dummy), RET_AUTO_ADDRESS_LEN);
		if(ret[0] != SPI_CMD_AUTO_ADDRESS) {
			applog(LOG_WARNING, "%d: error in AUTO_ADDRESS", btc08->chain_id);
			btc08->disabled = true;
			return false;
		}

		chip->disabled = true;
		btc08->last_chip++;
	}

	btc08->num_chips = ret[1];

	return true;
}

static bool reinit_chain(struct btc08_chain *btc08)
{
	uint8_t *ret;
	int ii;
	int chip_id;
	struct btc08_chip *chip;

	// last_chip 0
	// chip_index 0, 1, 2, 3, 4,
	// chip_id    1, 2, 3, 4, 5,
	// loop       0, 1, 2, 3, 4, ...
	//
	// last_chip 1
	// chip_index 0, 1, 2, 3, 4,
	// chip_id    1, 2, 3, 4, 5,
	// loop          2, 3, 4, 5, ...
	//
	// last_chip 2
	// chip_index 0, 1, 2, 3, 4,
	// chip_id    1, 1, 2, 3, 4,
	// loop             2, 3, 4, ...
	//
	// last_chip 4
	// chip_index 0, 1, 2, 3, 4, 5, 6,
	// chip_id    1, 1, 1, 1, 2, 3, 4,
	// loop                   2, 3, 4, ...
	//set_gpio_value(btc08->pinnum_gpio_vctrl, btc08->vctrl);
	if(btc08->last_chip) {
		if(!cmd_SET_PLL_FOUT_EN(btc08, 1, 0)) {
			btc08->disabled = true;
			applog(LOG_ERR, "%d: all chip_id:1 fail to set fout en to 0", btc08->chain_id);
			return false;
		}
	}

	for(ii=btc08->last_chip; ii<btc08->num_chips; ii++) {
		chip = &btc08->chips[ii];
		chip_id = ii +1;
		if(btc08->last_chip)
			chip_id += (1 -btc08->last_chip);
		if (!set_pll_config(btc08, chip_id, chip->mhz)) {
			applog(LOG_ERR, "%d: chip_id:%d(index:%d) fail to set pll(%d)", btc08->chain_id, chip_id, ii, chip->mhz);
			btc08->disabled = true;
			return false;
		}
	}

	ret = cmd_RESET_BCAST(btc08);

	btc08->num_cores = 0;
	btc08->perf = 0;
	btc08->is_processing_job = false;

	cmd_BIST_BCAST(btc08, BCAST_CHIP_ID);
	for (int chipId = 1; chipId <= btc08->num_chips; chipId++) {
		if (check_chip(btc08, chipId))
		{
			btc08->num_cores += btc08->chips[chipId-1].num_cores;
			btc08->perf += btc08->chips[chipId-1].perf;
		}
	}
#if 0
	for (ii = (btc08->num_chips-1); ii>=0; ii--) {
		if (is_chip_disabled(btc08, ii)) {
			applog(LOG_DEBUG, "%d: %d chip disabled", btc08->chain_id, ii);
			continue;
		}
		chip = &btc08->chips[ii];
		chip_id = ii +1;
		if(btc08->last_chip)
			chip_id -=  -(btc08->last_chip-1);
		check_chip(btc08, chip_id);
	}
#endif
	applog(LOG_DEBUG, "perf = %ld\n", btc08->perf);

	calc_nonce_range(btc08);

	return true;
}

/* check if disabled chips can be re-enabled */
static bool check_disabled_chips(struct btc08_chain *btc08)
{
	int i, new_last_chip = 0;
	int cid = btc08->chain_id;
	uint8_t *ret;
	int chip_id;
	int reset_flag = 0;
	for (i = (btc08->num_chips-1); i>=0; i--) {
		struct btc08_chip *chip = &btc08->chips[i];
//		if(chip->disabled) continue;
		chip_id = i +1;
		if(btc08->last_chip) {
			chip_id -=  -(btc08->last_chip-1);
			applog(LOG_WARNING, "%s():loop:chip_id:%d (last_chip:%d)", __FUNCTION__, chip_id, btc08->last_chip);
		}
		else {
			applog(LOG_WARNING, "%s():loop:chip_id:%d", __FUNCTION__, chip_id);
		}

//		if (!is_chip_disabled(btc08, i))
//			continue;
//		if (chip->cooldown_begin + COOLDOWN_MS > get_current_ms())
//			continue;

		// check remain job number
		ret = cmd_READ_ID(btc08, chip_id);
		if(ret == NULL) {
			reset_flag = 1;
			chip->disabled = true;
			btc08->num_cores -= chip->num_cores;
			chip->num_cores = 0;
			chip->mhz = 0;
			chip->perf = 0;
			new_last_chip = chip_id;
			btc08->last_chip += i +1;
			applog(LOG_ERR, "%s():loop:chip_id:%d is disabled, because no reponce", __FUNCTION__, chip_id);
			break;
		}
		if(((btc08->chips[btc08->num_chips-1].rev>>8)&0xf) != FEATURE_FOR_FPGA) {
			if((ret[2]&0x7)>=OON_INT_MAXJOB) {
				reset_flag = 1;
				if(chip->mhz > pll_sets[0].freq) {
					chip->mhz -= 50;
				}
				else {
					chip->disabled = true;
					btc08->num_cores -= chip->num_cores;
					chip->num_cores = 0;
					chip->mhz = 0;
					chip->perf = 0;
					new_last_chip = chip_id;
					btc08->last_chip += i +1;
					/* TODO: set last chip */
					applog(LOG_ERR, "%s():loop:chip_id:%d is disabled, in PLL %dMHz", __FUNCTION__, chip_id, chip->mhz);
					break;
				}
			}
		}
	}
	if(reset_flag == 0) {
		applog(LOG_WARNING, "%d: there is no errors for timeout OON", cid);
		return true;
	}
	if(new_last_chip) {
		if(!set_last_chip(btc08, new_last_chip))
			return false;
	}
	return reinit_chain(btc08);
}

/********** job creation and result evaluation */
uint32_t get_diff(double diff)
{
	uint32_t n_bits;
	int shift = 29;
	double f = (double) 0x0000ffff / diff;
	while (f < (double) 0x00008000) {
		shift--;
		f *= 256.0;
	}
	while (f >= (double) 0x00800000) {
		shift++;
		f /= 256.0;
	}
	n_bits = (int) f + (shift << 24);
	return n_bits;
}

static uint8_t *create_job(uint8_t chip_id, uint8_t *job, struct work *work)
{
	uint8_t *midstate, *wdata, *midstate1, *midstate2, *midstate3;

	job[0] = SPI_CMD_WRITE_PARM;
	job[1] = chip_id;

	midstate  = work->midstate;
	wdata     = work->data + 64;
	midstate1 = work->midstate1;
	midstate2 = work->midstate2;
	midstate3 = work->midstate3;

	memcpy(job+2,              midstate, 32);
	memcpy(job+2+32,              wdata, 12);	// MerkleRoot + TimeStamp + Target
	memcpy(job+2+32+12,       midstate1, 32);
	memcpy(job+2+32+12+32,    midstate2, 32);
	memcpy(job+2+32+12+32+32, midstate3, 32);

	return job;
}

static void dump_work(char* title, struct work *work)
{
	char *header, *prev_blockhash, *merkle_root, *timestamp, *nbits;
	char *midstate, *midstate1, *midstate2, *midstate3, *target;

	header         = bin2hex(work->data,         128);
	prev_blockhash = bin2hex(work->data+4,        32);
	merkle_root    = bin2hex(work->data+4+32,     32);
	timestamp      = bin2hex(work->data+4+32+32,   4);
	nbits          = bin2hex(work->data+4+32+32+4, 4);

	midstate       = bin2hex(work->midstate,      32);
	midstate1      = bin2hex(work->midstate1,     32);
	midstate2      = bin2hex(work->midstate2,     32);
	midstate3      = bin2hex(work->midstate3,     32);
	target         = bin2hex(work->target,        32);

	applog(LOG_DEBUG, "================== %s ==================", title);
	applog(LOG_DEBUG, "header        : %s", header);
	applog(LOG_DEBUG, "prev_blockhash: %s", prev_blockhash);
	applog(LOG_DEBUG, "merkle_root   : %s", merkle_root);
	applog(LOG_DEBUG, "timestamp     : %s", timestamp);
	applog(LOG_DEBUG, "nbits         : %s", nbits);

	applog(LOG_DEBUG, "job_id %s micro_job_id %"PRIu16" nonce2 %"PRIu64" ntime %s",
			work->job_id, work->micro_job_id, work->nonce2, work->ntime);
	applog(LOG_DEBUG, "midstate      : %s", midstate);
	applog(LOG_DEBUG, "midstate1     : %s", midstate1);
	applog(LOG_DEBUG, "midstate2     : %s", midstate2);
	applog(LOG_DEBUG, "midstate3     : %s", midstate3);
	applog(LOG_DEBUG, "target        : %s", target);

	free(header);
	free(prev_blockhash);
	free(merkle_root);
	free(timestamp);
	free(nbits);
	free(midstate);
	free(midstate1);
	free(midstate2);
	free(midstate3);
	free(target);
	applog(LOG_DEBUG, "=======================================================================");
}

static void dump_work_list(struct btc08_chain *btc08)
{
	char s[1024];
	for (int i=0; i<= JOB_ID_NUM_MASK; i++)
	{
		if (btc08->work[i])
		{
			snprintf(s, sizeof(s), "[WORK] btc08->work[%d] job_id:%s "
				"nonce2 %"PRIu64" ntime %s\n", i, btc08->work[i]->job_id,
				btc08->work[i]->nonce2, btc08->work[i]->ntime);
			applog(LOG_DEBUG, "%s", s);
		}
	}
}

/* set work for given chip, returns true if a nonce range was finished */
static bool set_work(struct btc08_chain *btc08, struct work *work)
{
	int cid = btc08->chain_id;
	bool retval = false;
	uint8_t job_id;

	job_id = btc08->last_queued_id + 1;
	applog(LOG_INFO, "%d: queuing job_id %d", cid, job_id);

	if (btc08->work[btc08->last_queued_id] != NULL) {
		if (opt_debug) {
			char s[512];
			snprintf(s, sizeof(s), "[DEL OLD_WORK] btc08->work[%d] job_id: %d",
					btc08->last_queued_id, job_id);
			dump_work(s, btc08->work[btc08->last_queued_id]);
			dump_work_list(btc08);
		}
		// delete already processed work from queued_work of cgpu
		work_completed(btc08->cgpu, btc08->work[btc08->last_queued_id]);
		btc08->work[btc08->last_queued_id] = NULL;
		retval = true;
	}

	// RUN_JOB for a new work
	uint8_t *jobdata = create_job(BCAST_CHIP_ID, btc08->spi_ctx->txb, work);
	if (!cmd_WRITE_JOB_fast(btc08, job_id, jobdata, work)) {
		applog(LOG_ERR, "%d: failed to set work for job_id %d with spi err", cid, job_id);

		// delete a work from queued_work of cgpu
		work_completed(btc08->cgpu, work);
		btc08->disabled = true;
	} else {
		applog(LOG_WARNING, "%d: succeed to set a new job_id:%d for work_job_id:%s", cid, job_id, work->job_id);
		btc08->work[btc08->last_queued_id] = work;
		if (opt_debug) {
			char s[512];
			snprintf(s, sizeof(s), "[NEW WORK] btc08->work[%d] job_id:%d, work_job_id:%s",
					btc08->last_queued_id, job_id, work->job_id);
			dump_work(s, btc08->work[btc08->last_queued_id]);
			dump_work_list(btc08);
		}
		// queue id range: 0~(JOB_ID_NUM_MASK)
		btc08->last_queued_id++;
		if (btc08->last_queued_id > JOB_ID_NUM_MASK)
			btc08->last_queued_id = 0;
	}
	return retval;
}

static bool set_work_test(struct btc08_chain *btc08, uint8_t chip_id, uint8_t job_id)
{
	int cid = btc08->chain_id;
	bool retval = false;
	struct work work;

	uint8_t jobdata[] = {
		0x07, 0x00, 0x4F, 0x40, 0x63, 0xF5, 0x49, 0x63, 
		0x8D, 0x39, 0x6D, 0x6E, 0x8E, 0x43, 0xF6, 0x3F, 
		0x8B, 0xA2, 0x65, 0xB0, 0xBA, 0xA4, 0xE3, 0xAF, 
		0xC3, 0x50, 0x29, 0x36, 0x5A, 0x98, 0x4C, 0xF6, 
		0x9E, 0xB7, 0x91, 0x5C, 0x88, 0x7A, 0x53, 0x6D, 
		0xC8, 0x02, 0x19, 0x00, 0x89, 0x6C, 0x00, 0x00};

	memcpy(btc08->spi_ctx->txb, jobdata, sizeof(jobdata));
	if (!cmd_WRITE_JOB_test(btc08, job_id, btc08->spi_ctx->txb, chip_id)) {
		retval = false;
	} else {
		retval = true;
	}

	return retval;
}

static bool get_nonce(struct btc08_chain *btc08, uint8_t *nonce,
		      uint8_t *chip, uint8_t *micro_job_id)
{
	uint8_t *ret;
	int cid = btc08->chain_id;

	// READ_RESULT to read GN and clear GN IRQ
	ret = cmd_READ_RESULT(btc08, *chip);
	if (ret == NULL)
		return false;

	// [3:0]: lower3/lower2/lower/upper GN
	*micro_job_id = ret[17];
	for (int i=0; i<ASIC_BOOST_CORE_NUM; i++)
	{
		memcpy(&(nonce[i*4]), &(ret[i*4]), 4);
		if((*micro_job_id & (1<<i)) != 0)
		{
			char buf[512];
			snprintf(buf, sizeof(buf),
				"READ_RESULT[%d] on chip#%d Inst_%s", i, *chip,
				(i==0) ? "Upper":(((i==1) ? "Lower": ((i==2) ? "Lower_2":"Lower_3"))));
			applog_hexdump(buf, (ret + i*4), 4, LOG_DEBUG);
		}
	}

	// READ_HASH to read hash
	if (opt_debug)
	{
		int i=0;
		uint8_t *hash_ret = cmd_READ_HASH(btc08, *chip);
		if (hash_ret == NULL)
			return false;

		for (i=0; i<ASIC_BOOST_CORE_NUM; i++)
		{
			char title[512];
			sprintf(title, "READ_HASH[Inst_%s] on chip#%d",
					(i==0) ? "Upper":(((i==1) ? "Lower": ((i==2) ? "Lower_2":"Lower_3"))), *chip);
			applog_hexdump(title, &(hash_ret[i*32]), 32, LOG_DEBUG);
		}
	}

	return true;
}

/* H/W reset of chip chain */
static bool abort_work(int chain_id)
{
	return HWReset(chain_id);
}

/********** driver interface */
void exit_btc08_chain(struct btc08_chain *btc08)
{
	if (btc08 == NULL)
		return;
	free(btc08->chips);
	btc08->chips = NULL;
	btc08->spi_ctx = NULL;
	free(btc08);
}

static int mvolt_array[2] = {400, 420};
#define DEFAULT_HBTEST_MIN_400MV      (400-40);
#define DEFAULT_HBTEST_MAX_400MV      (400+40);
#define DEFAULT_HBTEST_MIN_420MV      (420-42);
#define DEFAULT_HBTEST_MAX_420MV      (420+42);
#define DEFAULT_HBTEST_MIN_CORES      (206-10);
#define DEFAULT_HBTEST_FULLTEST_MSEC (2000)

#define DEFAULT_HBVOLT_SETUPTIME_MSEC (500)

static int hbtest_get_ref_value(char *refstr)
{
	int ret = 0;
	FILE *fwfptr;
	char cmd[128];

	sprintf(cmd, "/sbin/fw_printenv | /bin/grep %s | /usr/bin/awk -F= '{print $2}'", refstr);

	fwfptr = popen(cmd, "r");
	if(fwfptr) {
		char lenstr[16];
		fgets(lenstr, 16, fwfptr);
		ret = strtoul(lenstr, NULL, 0);
		pclose(fwfptr);
	}

	return ret;
}

static int job_weight_idx = 0;
static int hashboard_test(struct btc08_chain *btc08)
{
	uint8_t *ret;
	int res = 0, i, chip_id, ii;
	int mvolt_idx = 0, start_ms;

	applog(LOG_ERR, "----------------------------------------------------------------------");
	applog(LOG_ERR, "----------------------- hash board test mode!! -----------------------");
	applog(LOG_ERR, "----------------------------------------------------------------------");
	int min_mvolt_array[2];
	int max_mvolt_array[2];
	int min_cores, min_chips;
	int fulltest_msec;
	char dummy[32];
	FILE *fwfptr;

	min_mvolt_array[0] = hbtest_get_ref_value("hbtest_min_400mv");
//	if(!min_mvolt_array[0]) min_mvolt_array[0] = DEFAULT_HBTEST_MIN_400MV;

	max_mvolt_array[0] = hbtest_get_ref_value("hbtest_max_400mv");
	if(!max_mvolt_array[0]) max_mvolt_array[0] = DEFAULT_HBTEST_MAX_400MV;
	
	min_mvolt_array[1] = hbtest_get_ref_value("hbtest_min_420mv");
//	if(!min_mvolt_array[1]) min_mvolt_array[1] = DEFAULT_HBTEST_MIN_420MV;

	max_mvolt_array[1] = hbtest_get_ref_value("hbtest_max_420mv");
	if(!max_mvolt_array[1]) max_mvolt_array[1] = DEFAULT_HBTEST_MAX_420MV;
	
	min_cores = hbtest_get_ref_value("hbtest_min_cores");
	if(!min_cores) min_cores = DEFAULT_HBTEST_MIN_CORES;
	
	min_chips = hbtest_get_ref_value("hbtest_min_chips");
	if(!min_chips) min_chips = MAX_CHIP_NUM;
	
	fulltest_msec = hbtest_get_ref_value("hbtest_fulltest_msec");
	if(!fulltest_msec) fulltest_msec = DEFAULT_HBTEST_FULLTEST_MSEC;
	
	applog(LOG_ERR, "--- 0.400V min adc value : %d ---", min_mvolt_array[0]);
	applog(LOG_ERR, "--- 0.400V MAX adc value : %d ---", max_mvolt_array[0]);
	applog(LOG_ERR, "--- 0.420V min adc value : %d ---", min_mvolt_array[1]);
	applog(LOG_ERR, "--- 0.420V MAX adc value : %d ---", max_mvolt_array[1]);
	applog(LOG_ERR, "--- minimum core number : %d ---", min_cores);
	applog(LOG_ERR, "--- full load test time : %d.%03d seconds ---", fulltest_msec/1000, fulltest_msec%1000);

	for(mvolt_idx = 0; mvolt_idx < 2; mvolt_idx++) {
		cmd_RESET_BCAST(btc08);

		//set_gpio_value(btc08->pinnum_gpio_vctrl, mvolt_idx);
		sprintf(dummy, "at %d.%03dV", mvolt_array[mvolt_idx]/1000, mvolt_array[mvolt_idx]%1000);
		cgsleep_ms(DEFAULT_HBVOLT_SETUPTIME_MSEC);

		res = get_mvolt(mvolt_idx);
		if( 
				(res < min_mvolt_array[mvolt_idx]) || 
				(res > max_mvolt_array[mvolt_idx]) ) {
			applog(LOG_ERR, "power error (%dmV detected, it must be %dmV)", res, mvolt_array[mvolt_idx]);
			res = -1;
			return res;
		}
		applog(LOG_ERR, "-- test chip at %d mV --", mvolt_array[mvolt_idx]);

		// SPI_CMD_AUTO_ADDRESS
		ret = exec_cmd(btc08, SPI_CMD_AUTO_ADDRESS, BCAST_CHIP_ID, dummy, sizeof(dummy), RET_AUTO_ADDRESS_LEN);
		if(ret[0] != SPI_CMD_AUTO_ADDRESS) {
			applog(LOG_ERR, "%s: error in AUTO_ADDRESS", dummy);
			res = -1;
			return res;
		}
		btc08->num_chips = ret[1];
		if(btc08->num_chips < min_chips) {
			applog(LOG_ERR, "%s: chip number FAIL!!(total %d, not %d)", dummy, btc08->num_chips, min_chips);
			res = -1;
			return res;
		}

		for (i = 0; i<btc08->num_chips; i++) {
			exec_cmd(btc08, SPI_CMD_READ_ID, i+1, NULL, 0, RET_READ_ID_LEN);
		}

		if (!set_pll_config(btc08, 0, btc08_config_options.pll)) {
			applog(LOG_ERR, "%s: set_pll_config(%d) FAIL!!", dummy, btc08_config_options.pll);
			res = -1;
			return res;
		}
		if (!set_control(btc08, 0, btc08_config_options.udiv)) {
			applog(LOG_ERR, "%s: set_control(%d) FAIL!!", dummy, btc08_config_options.udiv);
			res = -1;
			return res;
		}

		cmd_RESET_BCAST(btc08);
		btc08->num_cores = 0;
		btc08->perf = 0;
		btc08->is_processing_job = false;

		cmd_BIST_BCAST(btc08, BCAST_CHIP_ID);

		for (i = 0; i<btc08->num_chips; i++) {
			struct btc08_chip *chip = &btc08->chips[i];
//			if (is_chip_disabled(btc08, i)) continue;
			chip_id = i +1;
			if(btc08->last_chip)
				chip_id +=  (1-btc08->last_chip);
			if (check_chip(btc08, chip_id))
			{
				btc08->num_cores += btc08->chips[chip_id-1].num_cores;
				btc08->perf += btc08->chips[chip_id-1].perf;
			}

			if(chip->num_cores < min_cores) {
				res = -1;
				applog(LOG_ERR, "%s:\tchip %d has not enough cores (%d, minimum is %d)", dummy, i, chip->num_cores, min_cores);
			}
		}
		if(res == -1) return res;

		applog(LOG_DEBUG, "perf = %ld\n", btc08->perf);
		calc_nonce_range(btc08);

		ii = set_work_test(btc08, 0, job_weight_idx+1);
		job_weight_idx++;
		job_weight_idx&=3;
		if(ii == false) {
			applog(LOG_ERR, "%s:\tchip %d FAIL!!(in the write job)",dummy , i);
			res = -1;
			return res;
		}
		cgsleep_ms(1000);

		for (i = btc08->last_chip; i < btc08->num_chips; i++) {
			uint8_t *ret;

			chip_id = i +1;
			if(btc08->last_chip)
				chip_id +=  (1-btc08->last_chip);

#define	DEFAULT_TEST_TIMEOUT	500
			ii =  get_current_ms();
			do {
				ret = exec_cmd(btc08, SPI_CMD_READ_JOB_ID, chip_id, NULL, 0, RET_READ_JOB_ID_LEN);
				if (ret == NULL || ret[3] != chip_id) {
					applog(LOG_ERR, "%s:\tchip %d  cmd_READ_JOB_ID failed", dummy, i);
					res = -1;
					break;;
				}
				if(get_current_ms() > ( ii + (DEFAULT_TEST_TIMEOUT*100))) {
					break;
				}
			} while((ret[2]&2) == 0);

			if(get_current_ms() > ( ii + (DEFAULT_TEST_TIMEOUT*100))) {
				applog(LOG_ERR, "%s:\tchip %d FAIL!!(gn timeout)", dummy, i);
				res = -1;
				cmd_READ_RESULT(btc08, chip_id);
				continue;
			}

			if((ret[2]&1) == 0) {
				applog(LOG_ERR, "%s:\tchip %d(chip_id:%d) can't find golden nonce, failed", dummy, i, chip_id);
				res = -1;
				cmd_READ_RESULT(btc08, chip_id);
				continue;
			}

			ii = (job_weight_idx+3)&3; // job_id
			if (ret[0] != (ii+1)) {
				applog(LOG_ERR, "%s:\tchip %d FAIL!!(oon job id(%d) in register)", dummy, i, ret[1]);
				res = -1;
				continue;
			}
			if (ret[1] != (ii+1)) {
				applog(LOG_ERR, "%s:\tchip %d FAIL!!(gn job id(%d) in register)", dummy, i, ret[1]);
				res = -1;
				continue;
			}

			ret = cmd_READ_RESULT(btc08, chip_id);

			uint32_t *ret32 = (uint32_t *)ret;
			*ret32 = bswap_32(*ret32);
			*ret32 -= (btc08->chips[i].hash_depth*btc08->chips[i].num_cores);
			if(*ret32 != 0x0d473a59) {
				applog(LOG_ERR, "%s:\tchip %d FAIL!!(nonce:0x%08x is not correct, it must be 0x0d473a59)", dummy, i, *ret32);
				res = -1;
				continue;
			}

			applog(LOG_ERR, "%s:\tchip %d : OK", dummy, i);
		}
	}
	if(res < 0) return res;
	btc08_config_options.test_mode = 0;
	calc_nonce_range(btc08);
	btc08_config_options.test_mode = 1;

	start_ms = get_current_ms();
//	set_control(btc08, 0, 1|(1<<4));	// set OON int
	do {
		if(0 == get_gpio_value(btc08->pinnum_gpio_gn)) {
			ret = cmd_READ_JOB_ID(btc08, BCAST_CHIP_ID);

			if(ret[2]&1) {
				chip_id = ret[3];
				ret = cmd_READ_RESULT(btc08, chip_id);

				uint32_t *ret32 = (uint32_t *)ret;
				*ret32 = bswap_32(*ret32);
				*ret32 -= (btc08->chips[chip_id-1].hash_depth*btc08->chips[chip_id-1].num_cores);
				if(*ret32 != 0x0d473a59) {
					applog(LOG_ERR, "%s:\tchip %d FAIL!!(nonce:0x%08x is not correct, it must be 0x0d473a59)", dummy, i, *ret32);
					res = -1;
					continue;
				}
			}
		}

		if(0 == get_gpio_value(btc08->pinnum_gpio_oon)) {
			cmd_CLEAR_OON(btc08, BCAST_CHIP_ID);
			ii = set_work_test(btc08, 0, job_weight_idx+1);
			job_weight_idx++;
			job_weight_idx&=3;
			if(ii == false) {
				applog(LOG_ERR, "%s:\tfullload: FAIL!!(in the write job)",dummy);
				res = -1;
				break;
			}
		}

		ii =  get_current_ms();
	} while(get_current_ms() < (start_ms + fulltest_msec));

	return res;
}

static void read_feature(struct btc08_chain *btc08, uint8_t chipId)
{
	uint8_t *ret;
	uint32_t *ret32;
	int idx = (chipId > 0) ? (chipId - 1) : 0;

	ret = exec_cmd(btc08, SPI_CMD_READ_FEATURE, chipId, NULL, 0, RET_READ_FEATURE_LEN);
	ret32 = (uint32_t *)ret;
	btc08->chips[idx].hash_depth = ret[3];
	btc08->chips[idx].rev = *ret32;
}

/* Reinit btc08 chip */
static bool reinit_btc08_chip(struct btc08_chain *btc08)
{
	uint8_t *ret;
	int chain_id;

	if (btc08 == NULL)
		return false;

	chain_id = btc08->chain_id;

	if (btc08->chips)
		free(btc08->chips);

	btc08->chips = NULL;
	btc08->num_cores = 0;
	btc08->perf = 0;
	btc08->is_processing_job = false;

	applog(LOG_ERR, "%d: [%s]", chain_id, __FUNCTION__);

	// Check the number of the chips and the active chips via AUTO_ADDRESS & READ_ID
	btc08->num_chips = chain_detect(btc08);
	if (btc08->num_chips == 0)
	{
		applog(LOG_ERR, "%d: Failed to detect chain", chain_id);
		goto failure;
	}

	applog(LOG_ERR, "spidev%d.%d: %d: Found %d BTC08 chips",
	       btc08->spi_ctx->config.bus, btc08->spi_ctx->config.cs_line,
	       btc08->chain_id, btc08->num_chips);

	// Allocate memory for active chips
	btc08->chips = calloc(btc08->num_active_chips, sizeof(struct btc08_chip));
	assert (btc08->chips != NULL);

	// Get feature & revision info
	for(int chip_id = 1; chip_id <= btc08->num_active_chips; chip_id++) {
		read_feature(btc08, chip_id);
		ret = exec_cmd(btc08, SPI_CMD_READ_REVISION, chip_id, NULL, 0, RET_READ_REVISION_LEN);
		applog(LOG_ERR, "%d: chipId %d feature(0x%08x) date(%02x/%02x/%02x), index(%02x)",
					chain_id, chip_id, btc08->chips[chip_id-1].rev, ret[0], ret[1], ret[2], ret[3]);
	}

	// Check if there are enough chips on ASIC for mining
	if (((btc08->chips[btc08->num_chips-1].rev >> 8) & 0xf) != FEATURE_FOR_FPGA) {
		if (btc08->num_chips < btc08_config_options.min_chips) {
			applog(LOG_ERR, "%d: failed to get enough chips(%d; it must be over than %d)",
					chain_id, btc08->num_chips, btc08_config_options.min_chips);
			goto failure;
		}
	}

	// Set PLL config
	if (!set_pll_config(btc08, BCAST_CHIP_ID, btc08_config_options.pll))
		goto failure;

	// RUN_BIST & READ_BIST to check the number of cores passed BIST
	cmd_BIST_BCAST(btc08, BCAST_CHIP_ID);
	for (int chip_id = 1; chip_id <= btc08->num_chips; chip_id++) {
		if (check_chip(btc08, chip_id))
		{
			btc08->num_cores += btc08->chips[chip_id-1].num_cores;
			btc08->perf += btc08->chips[chip_id-1].perf;
		}
	}

	// Enable OON IRQ & Set UART divider
	set_control(btc08, BCAST_CHIP_ID, (OON_IRQ_EN | btc08_config_options.udiv));

	calc_nonce_range(btc08);

	return true;

failure:
	if (btc08->chips)
		free(btc08->chips);

	return false;
}

struct btc08_chain *init_btc08_chain(struct spi_ctx *ctx, int chain_id)
{
	int i, chip_id;
	uint8_t *ret;
	struct btc08_chain *btc08 = malloc(sizeof(*btc08));
	assert(btc08 != NULL);

	applog(LOG_DEBUG, "%d: BTC08 init chain", chain_id);
	memset(btc08, 0, sizeof(*btc08));
	btc08->spi_ctx = ctx;
	btc08->chain_id = chain_id;
	btc08->is_processing_job = false;

	for(i=0; i<MAX_SPI_PORT; i++) {
		if(ctx->config.bus == spi_available_bus[i])
			break;
	}

	btc08->pinnum_gpio_gn    =    gn_pin[i];
	btc08->pinnum_gpio_oon   =   oon_pin[i];
	btc08->pinnum_gpio_reset = reset_pin[i];

	// Check the number of the chips and the active chips via AUTO_ADDRESS & READ_ID
	btc08->num_chips = chain_detect(btc08);
	if (btc08->num_chips == 0) {
		applog(LOG_ERR, "%d: Failed to detect chain", chain_id);
		goto failure;
	}

	applog(LOG_INFO, "spidev%d.%d: %d: Found %d BTC08 chips",
	       btc08->spi_ctx->config.bus, btc08->spi_ctx->config.cs_line,
	       btc08->chain_id, btc08->num_chips);

	// Allocate memory for active chips
	btc08->chips = calloc(btc08->num_active_chips, sizeof(struct btc08_chip));
	assert (btc08->chips != NULL);
	btc08->xfr = calloc(btc08->num_active_chips+4, sizeof(struct spi_ioc_transfer)); // 2 for WRITE_TARGET, RUN_JOB
	assert (btc08->xfr != NULL);

	// Get feature & revision info
	for(chip_id = 1; chip_id <= btc08->num_active_chips; chip_id++) {
		read_feature(btc08, chip_id);
		ret = exec_cmd(btc08, SPI_CMD_READ_REVISION, chip_id, NULL, 0, RET_READ_REVISION_LEN);
		applog(LOG_INFO, "%d: chipId %d feature(0x%08x) date(%02x/%02x/%02x), index(%02x)",
					chain_id, chip_id, btc08->chips[chip_id-1].rev, ret[0], ret[1], ret[2], ret[3]);
	}

	// Check if there are enough chips on ASIC for mining
	if (((btc08->chips[btc08->num_chips-1].rev >> 8) & 0xf) != FEATURE_FOR_FPGA) {
		if (btc08->num_chips < btc08_config_options.min_chips) {
			applog(LOG_ERR, "%d: failed to get enough chips(%d; it must be over than %d)",
					chain_id, btc08->num_chips, btc08_config_options.min_chips);
			goto failure;
		}
	}

	// Set PLL config
	if (!set_pll_config(btc08, BCAST_CHIP_ID, btc08_config_options.pll))
		goto failure;

	// RUN_BIST & READ_BIST to check the number of cores passed BIST
	cmd_BIST_BCAST(btc08, BCAST_CHIP_ID);
	for (chip_id = 1; chip_id <= btc08->num_chips; chip_id++) {
		if (check_chip(btc08, chip_id))
		{
			btc08->num_cores += btc08->chips[chip_id-1].num_cores;
			btc08->perf += btc08->chips[chip_id-1].perf;
		}
	}

	// Enable OON IRQ & Set UART divider (TODO: Need to check if uart divider value is correct!)
	set_control(btc08, BCAST_CHIP_ID, (OON_IRQ_EN | btc08_config_options.udiv));

	/* In order to reduce the number of spi calls that distribute the nonce range to each chip,
	 * it is executed only once after bist */
	calc_nonce_range(btc08);

	applog(LOG_DEBUG, "perf = %ld\n", btc08->perf);

	if(btc08_config_options.test_mode == 1) {
		if(hashboard_test(btc08)<0) {
			applog(LOG_ERR, "TEST FAIL");

			system("echo timer > /sys/class/leds/red/trigger");
			system("echo 300 > /sys/class/leds/red/delay_on");
			system("echo 300 > /sys/class/leds/red/delay_off");
			system("echo timer > /sys/class/leds/green/trigger");
			system("echo 300 > /sys/class/leds/green/delay_on");
			system("echo 0 > /sys/class/leds/green/delay_off");
		}
		else {
			applog(LOG_ERR, "TEST OK");
			system("echo timer > /sys/class/leds/red/trigger");
			system("echo 300 > /sys/class/leds/red/delay_on");
			system("echo 0 > /sys/class/leds/red/delay_off");
			system("echo timer > /sys/class/leds/green/trigger");
			system("echo 300 > /sys/class/leds/green/delay_on");
			system("echo 300 > /sys/class/leds/green/delay_off");
		}
		reset_gpio(btc08, 0);
		while(1) cgsleep_ms(2000);
	}

	applog(LOG_INFO, "%d: found %d chips with total %d active cores",
	       btc08->chain_id, btc08->num_active_chips, btc08->num_cores);

	mutex_init(&btc08->lock);
	INIT_LIST_HEAD(&btc08->active_wq.head);

	return btc08;

failure:
	exit_btc08_chain(btc08);
	return NULL;
}

static bool detect_single_chain(struct spi_ctx *ctx, int idx)
{
	applog(LOG_WARNING, "BTC08: checking single chain");
	struct btc08_chain *btc08 = init_btc08_chain(ctx, idx);
	if (btc08 == NULL) {
		applog(LOG_ERR, "BTC08: Not detected BTC08 chain %d", idx);
		return false;
	}

	struct cgpu_info *cgpu = malloc(sizeof(*cgpu));
	assert(cgpu != NULL);

	memset(cgpu, 0, sizeof(*cgpu));
	cgpu->drv = &btc08_drv;
	cgpu->name = "BTC08.SingleChain";
	cgpu->threads = 1;

	cgpu->device_data = btc08;

	btc08->cgpu = cgpu;
	add_cgpu(cgpu);
	applog(LOG_WARNING, "Detected single BTC08 chain %d with %d chips / %d cores",
	       idx, btc08->num_active_chips, btc08->num_cores);
	return true;
}

static void export_gpios()
{
	int fd;
	char port[8];
	char path[64];
	for( int i=0 ; i<MAX_SPI_PORT ; i++ )
	{
		//	RESET
		fd = open("/sys/class/gpio/export", O_WRONLY);
		if( fd > 0 )
		{
			sprintf(port, "%d", reset_pin[i]);
			write(fd, port, strlen(port));
			close(fd);

			sprintf(path, "/sys/class/gpio/gpio%d/direction", reset_pin[i]);
			fd = open(path, O_WRONLY);
			if( fd>0 )
			{
				write(fd, "out", 3);
				close(fd);
			}
		}
		//	OON
		fd = open("/sys/class/gpio/export", O_WRONLY);
		if( fd > 0 )
		{
			sprintf(port, "%d", oon_pin[i]);
			write(fd, port, strlen(port));
			close(fd);
			sprintf(path, "/sys/class/gpio/gpio%d/direction", oon_pin[i]);
			fd = open(path, O_WRONLY);
			if( fd>0 )
			{
				write(fd, "in", 2);
				close(fd);
			}
		}
		//	GN
		fd = open("/sys/class/gpio/export", O_WRONLY);
		if( fd > 0 )
		{
			sprintf(port, "%d", gn_pin[i]);
			write(fd, port, strlen(port));
			close(fd);
			sprintf(path, "/sys/class/gpio/gpio%d/direction", gn_pin[i]);
			fd = open(path, O_WRONLY);
			if( fd>0 )
			{
				write(fd, "in", 2);
				close(fd);
			}
		}
		//	HASH_PLUG
		fd = open("/sys/class/gpio/export", O_WRONLY);
		if( fd > 0 )
		{
			sprintf(port, "%d", plug_pin[i]);
			write(fd, port, strlen(port));
			close(fd);
			sprintf(path, "/sys/class/gpio/gpio%d/direction", plug_pin[i]);
			fd = open(path, O_WRONLY);
			if( fd>0 )
			{
				write(fd, "in", 2);
				close(fd);
			}
		}
		//	BOD_DET
		fd = open("/sys/class/gpio/export", O_WRONLY);
		if( fd > 0 )
		{
			sprintf(port, "%d", boddet_pin[i]);
			write(fd, port, strlen(port));
			close(fd);
			sprintf(path, "/sys/class/gpio/gpio%d/direction", boddet_pin[i]);
			fd = open(path, O_WRONLY);
			if( fd>0 )
			{
				write(fd, "in", 2);
				close(fd);
			}
		}
		//	PWREN_LDO
		fd = open("/sys/class/gpio/export", O_WRONLY);
		if( fd > 0 )
		{
			sprintf(port, "%d", pwren_pin[i]);
			write(fd, port, strlen(port));
			close(fd);
			sprintf(path, "/sys/class/gpio/gpio%d/direction", pwren_pin[i]);
			fd = open(path, O_WRONLY);
			if( fd>0 )
			{
				write(fd, "out", 3);
				close(fd);
			}
		}
	}
}

void setup_hashboard_gpio(int port_num, int *plug_status, int *board_type)
{
	// Check hash board connection
	*plug_status = get_gpio_value(plug_pin[port_num]);

	// Read board type (HASH/VTK)
	*board_type = get_gpio_value(boddet_pin[port_num]);

	// Enable FN
	set_gpio_value(pwren_pin[port_num], 1);

	applog(LOG_DEBUG, "Hash%d: connection status(%s), board_type(%s)",
			port_num,
			(*plug_status == 1) ? "Connected":"Removed",
			(*board_type == 1) ? "Hash":"VTK");
}

/* Probe SPI channel and register chip chain */
void btc08_detect(bool hotplug)
{
	int ii;
	int plug_status[MAX_SPI_PORT];
	int board_type[MAX_SPI_PORT];

	/* no hotplug support for SPI */
	if (hotplug)
		return;

	export_gpios();

	for (int i=0; i<MAX_SPI_PORT; i++)
	{
		setup_hashboard_gpio(i, &(plug_status[i]), &(board_type[i]));
	}

	// reset
	for (int i=0; i<MAX_SPI_PORT; i++)
	{
		set_gpio_value(reset_pin[i], 0);
		cgsleep_us(1000);
		set_gpio_value(reset_pin[i], 1);
	}

	/* parse btc08-options */
	if (opt_btc08_options != NULL && parsed_config_options == NULL) {
		int spi_clk = 0;
		int sys_clk_mhz = 0;
		int udiv = 0;

		sscanf(opt_btc08_options, "%d:%d:%d",
		       &spi_clk, &sys_clk_mhz, &udiv);
		if (spi_clk != 0)
			btc08_config_options.spi_clk_khz = spi_clk;
		if (sys_clk_mhz != 0)
			btc08_config_options.pll = sys_clk_mhz;
		if (udiv != 0)
			btc08_config_options.udiv = udiv;

		/* config options are global, scan them once */
		parsed_config_options = &btc08_config_options;
	}
	if (opt_btc08_min_cores != NULL) {
		int min_cores;
		sscanf(opt_btc08_min_cores, "%d", &min_cores);
		btc08_config_options.min_cores = min_cores;
	}
	if (opt_btc08_min_chips != NULL) {
		int min_chips;
		sscanf(opt_btc08_min_chips, "%d", &min_chips);
		btc08_config_options.min_chips = min_chips;
	}
	btc08_config_options.test_mode = 0;
	if (opt_btc08_chiptest != NULL)
		btc08_config_options.test_mode = 1;
	if (get_gpio_value(15) == 0)
		btc08_config_options.test_mode = 1;

	if (opt_btc08_test != NULL)
	{
		int num_chips = 0;
		int num_cores = 0;

		sscanf(opt_btc08_test, "%d:%d",
		       &num_chips, &num_cores);

		if (num_chips != 0)
			btc08_config_options.num_chips = num_chips;
		if (num_cores != 0)
			btc08_config_options.num_cores = num_cores;
	}

	applog(LOG_DEBUG, "BTC08 detect");

	/* register global SPI context */
	struct spi_config cfg = default_spi_config;
	for(ii=0; ii<MAX_SPI_PORT; ii++)
	{
		cfg.mode = SPI_MODE_0;
		cfg.speed = btc08_config_options.spi_clk_khz * 1000;
		cfg.bus = spi_available_bus[ii];

		spi[ii] = spi_init(&cfg);
		if (spi[ii] == NULL)
			return;

		if (detect_single_chain(spi[ii], ii) == false)
			/* release SPI context if no BTC08 products found */
			spi_exit(spi[ii]);
	}
}

static int64_t btc08_scanwork(struct thr_info *thr)
{
	int i;
	struct cgpu_info *cgpu = thr->cgpu;
	struct btc08_chain *btc08 = cgpu->device_data;
	int cid = btc08->chain_id;
	int32_t nonce_ranges_processed = 0;
	uint32_t nonce[4];
	uint8_t chip_id, job_id, micro_job_id;
	uint8_t gn_job_id, gn_irq;
	uint8_t *res;

	if ((0 == btc08->num_cores) || (MAX_CORES < btc08->num_cores)) {
		cgpu->deven = DEV_DISABLED;
		applog(LOG_ERR, "%d: wrong num_cores: %d", btc08->chain_id, btc08->num_cores);
		return -1;
	}

	if (btc08->disabled) {
		cgpu->deven = DEV_DISABLED;
		applog(LOG_ERR, "chain%d is disabled", cid);
		return -1;
	}

	mutex_lock(&btc08->lock);

	applog(LOG_INFO, "BTC08 running scanwork",
			(false == btc08->is_processing_job) ? "with the new works":"");

	if (!btc08->is_processing_job)
	{
		// Try to run first 4 works
		for (int i=0; i<MAX_JOB_FIFO ; i++)
		{
			struct work *work = wq_dequeue(&btc08->active_wq);
			if (work == NULL) {
				applog(LOG_WARNING, "%d: work underflow", cid);
				goto failure;
			}

			set_work(btc08, work);
			if (btc08->disabled) {
				applog(LOG_ERR, "chain%d is disabled", cid);
				goto failure;
			} else {
				btc08->is_processing_job = true;
			}
		}
	}

	/* poll queued results */
	while(true) {
		// Check GN GPIO Pin
		if (0 == get_gpio_value(btc08->pinnum_gpio_gn))
		{
			applog(LOG_INFO, "================= GN IRQ !!!! =================");
			for (int i=1; i<=btc08->num_active_chips; i++)
			{
				res = cmd_READ_JOB_ID(btc08, i);
				gn_job_id  = res[1];
				gn_irq 	   = res[2] & (1<<0);
				chip_id    = res[3];

				if (0 != gn_irq)
				{
					job_id = gn_job_id;
					if (get_nonce(btc08, (uint8_t*)&nonce[0], &chip_id, &micro_job_id))
					{
						if (chip_id < 1 || chip_id > btc08->num_active_chips) {
							applog(LOG_WARNING, "%d: wrong chip_id %d", cid, chip_id);
							continue;
						}
						if (job_id < 1 || job_id > (JOB_ID_NUM_MASK+1)) {
							applog(LOG_WARNING, "%d: chip %d: result has wrong job_id %d",
									cid, chip_id, job_id);
							continue;
						}

						struct btc08_chip *chip = &btc08->chips[chip_id - 1];
						struct work *work = btc08->work[job_id - 1];
						if (work == NULL) {
							// already been flushed => stale
							applog(LOG_WARNING, "%d: already been flushed job_id %d chip %d: "
									"stale nonce 0x%08x 0x%08x 0x%08x 0x%08x",
									cid, job_id, chip_id, nonce[0], nonce[1], nonce[2], nonce[3]);
							chip->stales++;
							continue;
						}

						// submit nonce
						for (i=0; i<ASIC_BOOST_CORE_NUM; i++)
						{
							if ((micro_job_id & (1<<i)) != 0)
							{
								work->micro_job_id = (1<<i);
								if (work->pool->vmask) {
									memcpy(work->data, &(work->pool->vmask_001[(1<<i)]), 4);
								}

								if (opt_debug) {
									char s[2048];
									snprintf(s, sizeof(s),
											"[GN WORK] btc08->work[%d] gn_job_id:%d for work_job_id:%s",
											(job_id - 1), job_id, work->job_id);
									dump_work(s, work);
									dump_work_list(btc08);
								}
								if (!submit_nonce(thr, work, nonce[i])) {
									applog(LOG_ERR, "%d: chip %d(job_id:%d, micro_jobid:%d): invalid nonce 0x%08x",
										cid, chip_id, job_id, work->micro_job_id, nonce[i]);
									chip->hw_errors++;
									/* add a penalty of a full nonce range on HW errors */
									nonce_ranges_processed--;
									continue;
								}
								applog(LOG_DEBUG, "YEAH: %d: chip %d (job_id:%d, micro_job_id:%d): nonce 0x%08x",
									cid, chip_id, job_id, work->micro_job_id, nonce[i]);
								chip->nonces_found++;
							}
						}
					}
				}
			}
		}

		// Check OON GPIO Pin
		if (0 == get_gpio_value(btc08->pinnum_gpio_oon))
		{
			applog(LOG_INFO, "================= OON IRQ!!!! =================");
			nonce_ranges_processed += 2;
			applog(LOG_DEBUG, "%d: job done ", cid);

			cmd_CLEAR_OON(btc08, BCAST_CHIP_ID);

			// Fill 2 works into FIFO whenever OON occurs
			for (int i=0; i<2; i++)
			{
				struct work *work = wq_dequeue(&btc08->active_wq);
				if (work == NULL) {
					applog(LOG_INFO, "%d: work underflow", cid);
					break;
				}
				set_work(btc08, work);
				if (btc08->disabled) {
					applog(LOG_ERR, "chain%d is disabled", cid);
					goto failure;
				} else {
					btc08->is_processing_job = true;
				}
			}
			break;
		}

		sched_yield();
	}

	mutex_unlock(&btc08->lock);

	if (nonce_ranges_processed < 0)
		nonce_ranges_processed = 0;

	if (nonce_ranges_processed != 0) {
		applog(LOG_DEBUG, "%d, nonces processed %d",
		       cid, nonce_ranges_processed);
	}

#if defined(USE_BTC08_FPGA)
	return ((uint64_t)MAX_NONCE_SIZE + 1) * ASIC_BOOST_CORE_NUM * 2;
#else
	return ((int64_t)nonce_ranges_processed << 32) * ASIC_BOOST_CORE_NUM;		// nonce range : 4G
#endif

failure:
	mutex_unlock(&btc08->lock);
	return 0;
}


/* queue two work items per chip in chain */
static bool btc08_queue_full(struct cgpu_info *cgpu)
{
	struct btc08_chain *btc08 = cgpu->device_data;
	int queue_full = false;

	mutex_lock(&btc08->lock);
	applog(LOG_DEBUG, "%d, BTC08 running queue_full: %d/%d",
	       btc08->chain_id, btc08->active_wq.num_elems, (MAX_JOB_FIFO*10));

	if (btc08->active_wq.num_elems >= MAX_JOB_FIFO*10)
		queue_full = true;
	else
		wq_enqueue(&btc08->active_wq, get_queued(cgpu));

	mutex_unlock(&btc08->lock);

	return queue_full;
}

static void btc08_flush_work(struct cgpu_info *cgpu)
{
	struct btc08_chain *btc08 = cgpu->device_data;
	int cid = -1;

	if (btc08 == NULL)
		return;

	mutex_lock(&btc08->lock);

	cid = btc08->chain_id;

	applog(LOG_DEBUG, "%d: BTC08 running flushwork", cid);

	/* stop chips hashing current work */
	if (!abort_work(cid)) {
		applog(LOG_ERR, "%d: failed to abort work in chip chain!", cid);
	}

	/* flush the work chips were currently hashing */
	for (int i = 0; i <= JOB_ID_NUM_MASK; i++) {
		struct work *work = btc08->work[i];
		if (work == NULL)
			continue;
		applog(LOG_ERR, "[DELETE WORK] %d: flushing work[%d]: %s",
				cid, i, work->job_id);
		work_completed(cgpu, work);
		btc08->work[i] = NULL;
	}

	/* flush queued work */
	applog(LOG_DEBUG, "%d: flushing queued work...", cid);
	while (btc08->active_wq.num_elems > 0) {
		struct work *work = wq_dequeue(&btc08->active_wq);
		assert(work != NULL);
		work_completed(cgpu, work);
	}
	btc08->sdiff = 0;
	btc08->is_processing_job = false;
	btc08->num_cores = 0;
	btc08->perf = 0;
	btc08->last_queued_id = 0;

	/* reinit btc08 chip */
	if (!reinit_btc08_chip(btc08)) {
		cgpu->deven = DEV_DISABLED;
	}

	mutex_unlock(&btc08->lock);
}

static void btc08_get_statline_before(char *buf, size_t len,
				   struct cgpu_info *cgpu)
{
	struct btc08_chain *btc08 = cgpu->device_data;
	char temp[10];
	if (btc08->temp != 0)
		snprintf(temp, 9, "%2dC", btc08->temp[0]);
	tailsprintf(buf, len, " %2d:%2d/%3d %s",
		    btc08->chain_id, btc08->num_active_chips, btc08->num_cores,
		    btc08->temp[0] == 0 ? "   " : temp);
}

static struct api_data *btc08_api_stats(struct cgpu_info *cgpu)
{
       struct api_data *root = NULL;
       struct btc08_chain *btc08 = cgpu->device_data;

       root = api_add_int(root, "chain_id", &(btc08->chain_id), false);

       root = api_add_int(root, "asic_count", &(btc08->num_chips), false);

       btc08->volt_f = (float)btc08->mvolt/1000.0;
       root = api_add_volts(root, "volt", &(btc08->volt_f), false);

       btc08->high_temp_val_f = (float)btc08->high_temp_val;
       root = api_add_temp(root, "hi_temp", &(btc08->high_temp_val_f), false);

       root = api_add_int(root, "hot_chip", &(btc08->high_temp_id), false);

       root = api_add_int(root, "chain_id_end", &(btc08->chain_id), false);

       return root;
}

struct device_drv btc08_drv = {
	.drv_id = DRIVER_btc08,
	.dname = "BTC08",
	.name = "BTC08",
	.drv_detect = btc08_detect,

	.hash_work = hash_queued_work,
	.scanwork = btc08_scanwork,
	.queue_full = btc08_queue_full,
	.flush_work = btc08_flush_work,
	.get_api_stats = btc08_api_stats,
	.get_statline_before = btc08_get_statline_before,
};
