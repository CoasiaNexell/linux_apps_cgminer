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

#if defined(USE_BTC08_FPGA)
#define MAX_CHIP_NUM			3
#define MAX_CORES_PER_CHIP		2
#define MAX_SPI_PORT			1
// Hash should be done within 1 second
#define MAX_NONCE_SIZE			(0x07ffffff)
#else
#define MAX_CHIP_NUM			22
#define MAX_CORES_PER_CHIP		30
#define MAX_SPI_PORT			2
#define MAX_NONCE_SIZE			(0xffffffff)
#endif
#define FPGA_MINER_CORE_CLK		50		// 50 MHz

#define MAX_CORES				(MAX_CHIP_NUM * MAX_CORES_PER_CHIP)
#define DEFAULT_MIN_CORES		(MAX_CORES_PER_CHIP*0.9)
#define DEFAULT_MIN_CHIPS		(MAX_CHIP_NUM*0.9)

#define MAX_JOB_FIFO			4

#define CMD_CHIP_ID_LEN			2
#define BCAST_CHIP_ID			0

/********** chip and chain context structures */
/* the WRITE_JOB command is the largest (2 bytes command, 56 bytes payload) */
#define MIDSTATE_LEN            32		// 256 bits
// MerkleRoot + timestamp + difficulty
#define DATA_LEN                12		// 96 bits

#define ASIC_BOOST_CORE_NUM     4

#define DISABLE_LEN             32
#define HASH_LEN                32
#define NONCE_LEN               4
#define TARGET_LEN              6
#define PLL_VALUE_LEN           2
#define JOB_ID_LEN              2
#define BIST_HASH_LEN          (1024/8)
#define DUMMY_BYTES             2

// midstate + data + midstate + midstate + midstate
#define WRITE_JOB_LEN          (((ASIC_BOOST_CORE_NUM * MIDSTATE_LEN) + DATA_LEN))
#define MAX_CHAIN_LEN          256

/*
 * For commands to traverse the chain, we need to issue dummy writes to
 * keep SPI clock running. To reach the last chip in the chain, we need to
 * write the command, followed by chain-length words to pass it through the
 * chain and another chain-length words to get the ACK back to host
 */
#define MAX_CMD_LENGTH         (1024)	// CMD(8)+CHIPID(8)+READ_PARM(1120)

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

#define FEATURE_FOR_FPGA		0x0
#define FEATURE_FOR_ASIC		0x5

#define BIST_STATUS_IDLE		0
#define BIST_STATUS_BUSY		1

#define	TIME_LIMIT_OF_OON		4000	/* mili seconds */
#define	TIME_LIMIT_OF_OON_FPGA	120000	/* mili seconds */

#define OON_INT_MAXJOB	2

#define TEMP_UPDATE_INT_MS	2000

#define MAX_JOB_ID_NUM			256
#define JOB_ID_NUM_MASK         (MAX_JOB_FIFO*2-1)	/* total 7 */

// RUN_JOB Extra Bits
#define ASIC_BOOST_EN           (1<<1)

// SET_CONTROL Extra Bits
#define OON_IRQ_EN				(1<<4)
#define LAST_CHIP		    	(1<<15)
#define MIN_UART_DIVIDER    	(0x03)

enum BTC08_command {
	SPI_CMD_READ_ID          = 0x00,
	SPI_CMD_AUTO_ADDRESS     = 0x01,
	SPI_CMD_RUN_BIST         = 0x02,
	SPI_CMD_READ_BIST        = 0x03,
	SPI_CMD_RESET            = 0x04,
	SPI_CMD_SET_PLL_CONFIG   = 0x05,
	SPI_CMD_READ_PLL         = 0x06,
	SPI_CMD_WRITE_PARM       = 0x07,
	SPI_CMD_READ_PARM        = 0x08,
	SPI_CMD_WRITE_TARGET     = 0x09,
	SPI_CMD_READ_TARGET      = 0x0A,
	SPI_CMD_RUN_JOB          = 0x0B,
	SPI_CMD_READ_JOB_ID      = 0x0C,
	SPI_CMD_READ_RESULT      = 0x0D,
	SPI_CMD_CLEAR_OON        = 0x0E,
	SPI_CMD_SET_DISABLE      = 0x10,
	SPI_CMD_READ_DISABLE     = 0x11,
	SPI_CMD_SET_CONTROL      = 0x12,
	SPI_CMD_DEBUG            = 0x15,
	SPI_CMD_WRITE_NONCE      = 0x16,
	SPI_CMD_WRITE_CORE_CFG   = 0x17,
	SPI_CMD_READ_DEBUGCNT    = 0x19,
	SPI_CMD_READ_HASH        = 0x20,
	SPI_CMD_WRITE_IO_CTRL    = 0x30,
	SPI_CMD_READ_IO_CTRL     = 0x31,
	SPI_CMD_READ_FEATURE     = 0x32,
	SPI_CMD_READ_REVISION    = 0x33,
	SPI_CMD_SET_PLL_FOUT_EN  = 0x34,
	SPI_CMD_SET_PLL_RESETB   = 0x35,
	SPI_CMD_WRITE_CORE_DEPTH = 0x36,
	SPI_CMD_SET_TMODE        = 0x38
};

struct btc08_chip {
	int num_cores;
	/* stats */
	int hw_errors;
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
	struct work *work[JOB_ID_NUM_MASK+1];
	// a flag to prevent sending READ_ID cmd to all chips.
	bool is_processing_job;
	uint8_t last_queued_id;

	/* mark chain disabled, do not try to re-enable it */
	bool disabled;
	int temp[MAX_CHAIN_LEN];
	int high_temp_val;
	float high_temp_val_f;
	int high_temp_id;
	int last_temp_time;
	int pinnum_gpio_gn;
	int pinnum_gpio_oon;
	int pinnum_gpio_reset;
	int fd_gpio_gn;
	int fd_gpio_oon;
	int volt_ch;
	int mvolt;
	float volt_f;
	int last_chip;
	int timeout_oon;
	cgtimer_t oon_begin;
};

struct btc08_board {
	int board_id;
	int num_chains;
	struct btc08_chain *chain[MAX_SPI_PORT];
};

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
const char *cmd2str(enum BTC08_command cmd);
#endif /* BTC08_COMMON_H */
