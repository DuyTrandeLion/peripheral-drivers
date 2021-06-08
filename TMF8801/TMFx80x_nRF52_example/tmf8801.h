/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
*                                                                           *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
*****************************************************************************
*/

#ifndef __TMF8801_H
#define __TMF8801_H

#include <stdint.h>
#include <stdio.h>

#include "mcu_tmf8801_config.h"

/** @file */

///////////////////////////////////////////////////////////////////////////////
//
// ***** All communication TO / FROM ToF device is LITTLE ENDIAN *****
//
///////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
//
// Measurement settings, please modify as required
//
//////////////////////////////////////////////////////////////
#define APP0_DELAY_DEFAULT                 (0x0)
#define APP0_PERIOD_DEFAULT                (33) // 33ms continuous, 0 is single capture
#define APP0_NOISE_THRSHLD_DEFAULT         (0x0)
#define APP0_ALG_DEFAULT                   (0x23)
#define APP0_GPIO_DEFAULT                  (0x0)
#define APP0_ITERATIONS_DEFAULT            (900) // x1000 iterations
//////////////////////////////////////////////////////////////

// MCU driver defines
#define PACKED                             __attribute__((packed))
#define BL_DEFAULT_SALT                    (0x29)
#define CPU_READY_STATUS                   (0x41)
#define CPU_PON0_MASK                      (0x01)
#define MAX_REGS                           (256)
#define IS_CPU_SLEEP(x)                    (!((x) & 0x1))
#define CLK_PLL_LOWPOWER(reg)              (((reg) & 0x40) == 0)
#define APP_ID_BOOTLOADER                  (0x80)
#define BL_IS_CMD_BUSY(x)                  ((x) > 0xF)
#define BL_CMD_WAIT_MSEC                   (1)
#define BL_VALID_CHKSUM                    (0xFF)
#define BL_I2C_CMD_SIZE                    (1)
#define BL_I2C_DATA_LEN_SIZE               (1)
#define BL_I2C_CHKSUM_SIZE                 (1)
#define BL_I2C_NUM_DATA                    (128)
#define BL_I2C_MAX_DATA_SIZE               (BL_I2C_NUM_DATA*sizeof(uint8_t))
#define BL_I2C_HEADER_SIZE                 (BL_I2C_CMD_SIZE + \
                                            BL_I2C_DATA_LEN_SIZE)
#define BL_I2C_FOOTER_SIZE                 (BL_I2C_CHKSUM_SIZE)
#define BL_I2C_CMD_MAX_SIZE                (BL_I2C_HEADER_SIZE   + \
                                            BL_I2C_MAX_DATA_SIZE     + \
                                            BL_I2C_FOOTER_SIZE)
#define BL_CALC_CHKSUM_SIZE(datasize)      ((datasize) + \
                                            BL_I2C_HEADER_SIZE)
#define BL_CALC_BL_CMD_SIZE(datasize)      (BL_CALC_CHKSUM_SIZE(datasize) + \
                                            BL_I2C_FOOTER_SIZE)
#define BL_CALC_BL_RSP_SIZE(datasize)      ((datasize) + \
                                            BL_I2C_HEADER_SIZE + \
                                            BL_I2C_FOOTER_SIZE)
#define APP_ID_APP0                        (0xC0)
#define APP0_VERSION_SIZE                  (5)
#define APP0_ALG_STATE_SIZE                (11)
#define APP0_FAC_CALIB_SIZE                (14)
#define APP0_CFG_DATA_SIZE                 (120)
#define APP0_RESULT_SIZE                   (27)
#define APP0_CMD_NUM_PARAMS                (10)
#define APP0_MAX_CMD_SIZE                  (APP0_CMD_NUM_PARAMS + 1)
#define APP0_CMD_IDX                       (APP0_MAX_CMD_SIZE - 1)
#define APP0_CR_WRAPAROUND                 (0x100000000L)      // 64 bits
#define APP0_CR_WRAPAROUND_DIV2            (0x80000000L )      // 32 bits
#define APP0_RATIO_MINCOUNT                (3)
#define APP0_DEFAULT_CLK_ITERATIONS        (15)
#define APP0_EXPECTEDRATIO                 (5)
#define APP0_EXP_FREQ_RATIO_Q15            (6972)
#define APP0_MAX_FREQ_RATIO_Q15            (7212) // lowest Frequency
#define APP0_MIN_FREQ_RATIO_Q15            (6732) // highest Frequency
#define APP0_FREQ_RATIO_HITH_Q15           (6991) //12.5 KHz band tolerance
#define APP0_FREQ_RATIO_LOTH_Q15           (6953)

#define APP0_MAX_DIST_MM                   (2625) // max dist is 2.5m + 5% error
#define APP0_RESULT_CONF_MASK              (0x3F)
#define APP0_CONF_FROM_RESULT_INFO(x)      ((x) & APP0_RESULT_CONF_MASK)

enum {
    REG_CPU_STATUS = 0xE0,
    REG_INT_STATUS = 0xE1,
    REG_INT_ENABLE = 0xE2,
    REG_CLK_STATUS = 0xEC,
};

enum tof8801_BL_regs {
  REG_BL_CMD_STAT      = 0x08,
  REG_BL_DATA_SIZE     = 0x09,
  REG_BL_DATA_0        = 0x0A,
  REG_BL_DATA_127      = 0x89,
  REG_BL_CHKSUM        = 0x8A,
};

enum tof8801_bootloader_cmd {
  BL_RESET          = 0x10,
  BL_RAMREMAP_RESET = 0x11,
  BL_UPLOAD_INIT    = 0x14,
  BL_R_RAM          = 0x40,
  BL_W_RAM          = 0x41,
  BL_ADDR_RAM       = 0x43,
};

enum tof8801_bootloader_cmd_stat {
  READY          = 0x0,
  ERR_SIZE       = 0x1,
  ERR_CSUM       = 0x2,
  ERR_RES        = 0x3,
  ERR_APP        = 0x4,
  ERR_TIMEOUT    = 0x5,
  ERR_LOCK       = 0x6,
  ERR_RANGE      = 0x7,
  ERR_MORE       = 0x8,
  ERROR1         = 0x9,
  ERROR2         = 0xA,
  ERROR3         = 0xB,
  ERROR4         = 0xC,
  ERROR5         = 0xD,
  ERROR6         = 0xE,
  ERROR7         = 0xF,
  CMD_BUSY       = 0x10,
  MAX_BL_CMD_STAT,
};

enum tof8801_app0_state {
    APP0_IDLE_STATE = 0x01,
};

enum tof8801_app0_cal_flags {
    APP0_NO_FLAG          = 0x0,
    APP0_FAC_CAL_FLAG     = 0x1,
    APP0_ALG_STATE_FLAG   = 0x2,
    APP0_CFG_DATA_FLAG    = 0x4,
    APP0_ALL_FLAG         = 0x7,
};

enum tof8801_app0_cmd {
    APP0_CMD_START_W_CAL = 0x02, // perform measurement with parameters and calibration data
    APP0_CMD_START       = 0x03, // perform measurement with parameters
    APP0_CMD_FAC_CALIB   = 0x0A, // perform factory calibration and return calib data
    APP0_CMD_OSC_TRIM    = 0x29, // switch to clock trim mode
    APP0_CMD_RD_RESULT   = 0x55, // results are automatically published after IRQ_RESULT
    APP0_CMD_STOP        = 0xFF, // stop all measurements and go to IDLE state
};

enum tof8801_app0_regs {
  REG_APP0_MAJ_VER      = 0x01,
  REG_APP0_CMD_START    = 0x06,
  REG_APP0_CMD          = 0x10,
  REG_APP0_MIN_VER      = 0x12,
  REG_APP0_STATE        = 0x1C,
  REG_APP0_STATUS       = 0x1D,
  REG_APP0_CONTENTS     = 0x1E,
  REG_APP0_TID          = 0x1F,
  REG_APP0_DATA_START   = 0x20,
};

enum tof8801_app0_content_id {
    FAC_CALIB_CONTENTS_ID   = APP0_CMD_FAC_CALIB,
    MEAS_RESULT_CONTENTS_ID = APP0_CMD_RD_RESULT,
};

enum tof8801_app0_irq_flags {
    IRQ_RESULTS = 0x01,
    IRQ_DIAG    = 0x02,
    IRQ_ERROR   = 0x04,
};

/*****************************************************************************
 *
 *
 *  START Boot Loader structures
 *
 *
 * ***************************************************************************
 */

/*****************************************************************************/
/***** Bootloader command responses *****/
/*****************************************************************************/
struct tof8801_BL_short_resp {
    uint8_t status;
    uint8_t size;
    uint8_t reserved[BL_I2C_CMD_MAX_SIZE - 3];
    uint8_t chksum;
};

struct tof8801_BL_read_ram_resp {
    uint8_t status;
    uint8_t size;
    uint8_t data[BL_I2C_CMD_MAX_SIZE + 1]; /* chksum at flexible position */
};

struct tof8801_anon_resp {
    uint8_t data[BL_I2C_CMD_MAX_SIZE];
};

union tof8801_BL_response {
    struct tof8801_anon_resp        anon_resp;
    struct tof8801_BL_read_ram_resp read_ram_resp;
    struct tof8801_BL_short_resp    short_resp;
};

/*****************************************************************************/
/***** Bootloader commands *****/
/*****************************************************************************/
struct tof8801_BL_short_cmd {
    uint8_t command;
    uint8_t size;
    uint8_t chksum;
    uint8_t reserved[BL_I2C_MAX_DATA_SIZE];
};

struct tof8801_BL_upload_init_cmd {
    uint8_t command;
    uint8_t size;
    uint8_t seed;
    uint8_t chksum;
    uint8_t reserved[BL_I2C_MAX_DATA_SIZE - 1];
};

struct tof8801_BL_read_ram_cmd {
    uint8_t command;
    uint8_t size;
    uint8_t num_bytes;
    uint8_t chksum;
    uint8_t reserved[BL_I2C_MAX_DATA_SIZE - 1];
};

struct tof8801_BL_write_ram_cmd {
    uint8_t command;
    uint8_t size;
    uint8_t data[BL_I2C_MAX_DATA_SIZE + 1]; /*chksum in flexible position */
};

struct tof8801_BL_addr_ram_cmd {
    uint8_t command;
    uint8_t size;
    uint8_t addr_lsb;
    uint8_t addr_msb;
    uint8_t chksum;
    uint8_t reserved[BL_I2C_MAX_DATA_SIZE - 2];
};

struct tof8801_BL_anon_cmd {
    uint8_t data[BL_I2C_CMD_MAX_SIZE];
};

union tof8801_BL_command {
    struct tof8801_BL_anon_cmd        anon_cmd;
    struct tof8801_BL_addr_ram_cmd    addr_ram_cmd;
    struct tof8801_BL_write_ram_cmd   write_ram_cmd;
    struct tof8801_BL_read_ram_cmd    read_ram_cmd;
    struct tof8801_BL_upload_init_cmd upload_init_cmd;
    struct tof8801_BL_short_cmd       short_cmd;
};

struct tof8801_BL_application {
    uint8_t app_id;
    union tof8801_BL_command  BL_command;
    union tof8801_BL_response BL_response;
};

/*****************************************************************************
 *
 *
 *  END Boot Loader structures
 *
 *
 * ***************************************************************************
 */

/*****************************************************************************
 *
 *
 *  START App0 structures
 *
 *
 * ***************************************************************************
 */
union tof8801_app0_capture_cmd {
  struct {
    uint8_t reserved_arr[APP0_MAX_CMD_SIZE - 9];
    uint8_t data;
    uint8_t alg;
    uint8_t gpio;
    uint8_t delay;
    uint8_t noise_threshold;
    uint8_t period;
    uint8_t iterations[2];
    uint8_t cmd;
  } PACKED;
  uint8_t buf[APP0_MAX_CMD_SIZE];
} PACKED;

union tof8801_app0_status {
  struct {
    uint8_t state;
    uint8_t status;
    uint8_t contents;
    uint8_t tid;
  } PACKED;
  uint8_t buf[4];
} PACKED;

typedef struct
{
    uint32_t first_host;
    uint32_t last_host;
    uint32_t first_i2c;
    uint32_t last_i2c;
    uint32_t ratioQ15;
    uint32_t count;
    uint32_t trim_count;
} TMF8801_cr;

union tof8801_app0_result {
  struct {
    uint8_t res_num;
    uint8_t info;
    uint8_t distance_mm[sizeof(uint16_t)];
    uint8_t sysclk[sizeof(uint32_t)];
    uint8_t alg_state[APP0_ALG_STATE_SIZE];
    uint8_t ref_hits[sizeof(uint32_t)];
    uint8_t obj_hits[sizeof(uint32_t)];
  } PACKED;
  uint8_t buf[APP0_RESULT_SIZE];
} PACKED;

struct tof8801_app0_application {
  uint8_t                         app_id;
  uint8_t                         cal_flags;
  bool                            clk_trim_enable;
  uint8_t                         version[APP0_VERSION_SIZE];
  TMF8801_cr                      clk_cr;
  union tof8801_app0_capture_cmd  capture_cmd;
  union tof8801_app0_status       status;
  union tof8801_app0_result       result;
  uint8_t                         app0_fac_calib[APP0_FAC_CALIB_SIZE];
  uint8_t                         app0_cfg_data[APP0_CFG_DATA_SIZE];
};

/*****************************************************************************
 *
 *
 *  END App0 structures
 *
 *
 * ***************************************************************************
 */

struct tmf8801_ctx {
    uint8_t                         i2c_addr;
    struct tof8801_BL_application   BL_app;
    struct tof8801_app0_application app0_app;
};

/******* Start Generic TMF8801 functions ********/
void tof8801_dump_i2c_regs(struct tmf8801_ctx *tmf8801, uint8_t offset, uint8_t end);
int32_t tof8801_wait_for_cpu_ready(struct tmf8801_ctx *tmf8801);
int32_t tof8801_goto_standby(struct tmf8801_ctx*);
int32_t tof8801_wake_from_standby(struct tmf8801_ctx*);
/******* End Generic TMF8801 functions ********/

/******* Start Bootloader functions ***********/
uint8_t tof8801_calc_chksum(const uint8_t *, uint8_t);
int32_t tof8801_BL_send_rcv_cmd(struct tmf8801_ctx *, struct tof8801_BL_application *);
int32_t tof8801_BL_reset(struct tmf8801_ctx *, struct tof8801_BL_application *);
int32_t tof8801_BL_addr_ram(struct tmf8801_ctx *, struct tof8801_BL_application *, int32_t );
int32_t tof8801_BL_read_ram(struct tmf8801_ctx *, struct tof8801_BL_application *,
                            uint8_t *, int32_t);
int32_t tof8801_BL_write_ram(struct tmf8801_ctx *, struct tof8801_BL_application *,
                             const uint8_t *, int32_t );
int32_t tof8801_BL_ram_remap(struct tmf8801_ctx *client, struct tof8801_BL_application *);
int32_t tof8801_BL_upload_init(struct tmf8801_ctx *client, struct tof8801_BL_application *BL_app,
                               uint8_t salt);
void tof8801_BL_init_app(struct tof8801_BL_application *BL_app);
int32_t tof8801_BL_read_status(struct tmf8801_ctx *client,
                               struct tof8801_BL_application *BL_app,
                               uint32_t num_retries);
int32_t tof8801_BL_bin_download(struct tmf8801_ctx *, uint32_t start_addr,
                                const uint8_t *bin_buf, uint32_t len);
/******* End Bootloader functions ***********/

/******* Start App0 functions ***********/
int32_t tof8801_app0_init_app(struct tmf8801_ctx *tmf8801);
int32_t tof8801_app0_read_version(struct tmf8801_ctx *tmf8801);
int32_t tof8801_app0_enable_int(struct tmf8801_ctx *tmf8801);
int32_t tof8801_app0_start_capture(struct tmf8801_ctx *tmf8801);
int32_t tof8801_app0_stop_capture(struct tmf8801_ctx *tmf8801);
int32_t tof8801_app0_interrupt_status(struct tmf8801_ctx *tmf8801);
int32_t tof8801_app0_read_status(struct tmf8801_ctx *tmf8801);
int32_t tof8801_app0_start_fac_calib(struct tmf8801_ctx *tmf8801);
int32_t tof8801_app0_get_fac_calib(struct tmf8801_ctx *tmf8801, uint8_t *buf, uint32_t len);
int32_t tof8801_app0_set_fac_calib(struct tmf8801_ctx *tmf8801, const uint8_t *buf, uint32_t len);
int32_t tof8801_app0_get_alg_state(struct tmf8801_ctx *tmf8801, uint8_t *buf, uint32_t len);
int32_t tof8801_app0_set_alg_state(struct tmf8801_ctx *tmf8801, const uint8_t *buf, uint32_t len);
int32_t tof8801_app0_get_cfg_data(struct tmf8801_ctx *tmf8801, uint8_t *buf, uint32_t len);
int32_t tof8801_app0_set_cfg_data(struct tmf8801_ctx *tmf8801, const uint8_t *buf, uint32_t len);
int32_t tof8801_app0_get_last_result(struct tmf8801_ctx *tmf8801, uint32_t *dis_mm, uint32_t *confidence);
int32_t tof8801_app0_wait_for_idle(struct tmf8801_ctx *tmf8801, uint32_t usec_timeout);
int32_t tof8801_app0_set_default_capture_settings(struct tmf8801_ctx * tmf8801);
void tof8801_app0_process_irq(struct tmf8801_ctx *tmf8801);
/******* End App0 functions ***********/
#endif
