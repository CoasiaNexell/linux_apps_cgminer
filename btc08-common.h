#ifndef BTC08_COMMON_H
#define BTC08_COMMON_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/********** work queue */
struct work_ent {
	struct work *work;
	struct list_head head;
};

struct work_queue {
	int num_elems;
	struct list_head head;
};

// Used for a bytes align
#define ALIGN(x, a) (((x) + (a) - 1) & ~((a) - 1))

#define CMD_CHIP_ID_LEN		2
#define BCAST_CHIP_ID		0x00

/********** chip and chain context structures */
/* the WRITE_JOB command is the largest (2 bytes command, 56 bytes payload) */
#define MIDSTATE_LEN		32		// 256 bits
// MerkleRoot + timestamp + difficulty
#define DATA_LEN			12		// 96 bits

#define ASIC_BOOST_CORE_NUM	4

#define DISABLE_LEN			32
#define HASH_LEN			32
#define NONCE_LEN			4
#define TARGET_LEN			6
#define PLL_VALUE_LEN 		2
#define JOB_ID_LEN			2

// midstate + data + midstate + midstate + midstate
#define WRITE_JOB_LEN	(((ASIC_BOOST_CORE_NUM * MIDSTATE_LEN) + DATA_LEN))
#define MAX_CHAIN_LEN		256
/*
 * For commands to traverse the chain, we need to issue dummy writes to
 * keep SPI clock running. To reach the last chip in the chain, we need to
 * write the command, followed by chain-length words to pass it through the
 * chain and another chain-length words to get the ACK back to host
 */
#define MAX_CMD_LENGTH			(1024)	// param + command

#define RET_AUTO_ADDRESS_LEN	2		// 16 bits
#define RET_READ_ID_LEN			4		// 32 bits
#define RET_READ_JOB_ID_LEN		4		// 32 bits
#define RET_READ_RESULT_LEN 	18		// 144 bits
#define RET_READ_HASH_LEN 		128		// 1024 bits
#define RET_READ_TEMP_LEN		2		// 16 bits
#define RET_READ_PLL_LEN		4		// 32 bits
#define RET_READ_BIST_LEN		2		// 16 bits
#define RET_READ_FEATURE_LEN	4		// 32 bits
#define RET_READ_REVISION_LEN	4		// 32 bits

#define	TIME_LIMIT_OF_OON	4000	/* mili seconds */
#define	TIME_LIMIT_OF_OON_FPGA	120000	/* mili seconds */

#define OON_INT_MAXJOB	2

#define TEMP_UPDATE_INT_MS	2000

#define MAX_JOB_ID_NUM	64
#define JOB_ID_NUM_MASK	(MAX_JOB_ID_NUM-1)	/* total 4 */

struct btc08_chip {
	int num_cores;
	int last_queued_id;
	struct work *work[MAX_JOB_ID_NUM];
	/* stats */
	int hw_errors;
	int busy_job_id_flag[MAX_JOB_ID_NUM];
	int stales;
	int nonces_found;
	int nonce_ranges_done;
	int hash_depth;
	int rev;
	uint64_t mhz;
	uint64_t perf;
	uint32_t start_nonce;
	uint32_t end_nonce;

	/* systime in ms when chip was disabled */
	int cooldown_begin;
	/* number of consecutive failures to access the chip */
	int fail_count;
	/* mark chip disabled, do not try to re-enable it */
	bool disabled;
};

#define	MAX_CHIP_NUM	66
struct btc08_chain {
	int chain_id;
	struct cgpu_info *cgpu;
//	struct mcp4x *trimpot;
	int num_chips;
	int num_cores;
	uint64_t perf;
	int num_active_chips;
	int chain_skew;
	double		sdiff;
	uint8_t spi_tx[MAX_CMD_LENGTH+2];	// 2 for response
	uint8_t spi_rx[MAX_CMD_LENGTH+2];	// 2 for response
	uint8_t *spi_tx_a;
	uint8_t *spi_rx_a;
	struct spi_ioc_transfer *xfr;
	struct spi_ctx *spi_ctx;
	struct btc08_chip *chips;
	pthread_mutex_t lock;

	struct work_queue active_wq;

	/* mark chain disabled, do not try to re-enable it */
	bool disabled;
	int temp[MAX_CHAIN_LEN];
	int high_temp_val;
	float high_temp_val_f;
	int high_temp_id;
	int last_temp_time;
	int pinnum_gpio_gn;
	int pinnum_gpio_oon;
	int pinnum_gpio_vctrl;
	int pinnum_gpio_reset;
	int vctrl;
	int fd_gpio_gn;
	int fd_gpio_oon;
	int volt_ch;
	int mvolt;
	float volt_f;
	int last_chip;
	int timeout_oon;
	cgtimer_t oon_begin;
};

#define MAX_SPI_PORT	2
struct btc08_board {
	int board_id;
	int num_chains;
	struct btc08_chain *chain[MAX_SPI_PORT];
};

#define	MAX_CORES_PER_CHIP	206
#define DEFAULT_MIN_CORES	(MAX_CORES_PER_CHIP*0.9)

#define	DEFAULT_MIN_CHIPS	(MAX_CHIP_NUM*0.9)

/********** config paramters */
struct btc08_config_options {
	int spi_clk_khz;
	int pll;
	int udiv;
	/* limit chip chain to this number of chips (testing only) */
	int override_chip_num;
	int min_cores;
	int min_chips;
	int test_mode;
};

/* global configuration instance */
extern struct btc08_config_options btc08_config_options;

#endif /* BTC08_COMMON_H */
