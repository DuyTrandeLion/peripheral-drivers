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
/**
 * @file tmf8801.c
 * MCU driver functions for Time-of-Flight tmf8801
 */

#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "tmf8801.h"


static int32_t tof_i2c_write(struct tmf8801_ctx *tmf8801, uint8_t reg, uint8_t *buf, uint32_t len)
{
    return write_i2c_block(tmf8801->i2c_addr, reg, buf, len);
}

static int32_t tof_i2c_read(struct tmf8801_ctx *tmf8801, uint8_t reg, uint8_t *buf, uint32_t len)
{
    return read_i2c_block(tmf8801->i2c_addr, reg, buf, len);
}

static int32_t is_BL_cmd_busy(struct tmf8801_ctx *client)
{
    uint8_t status = CMD_BUSY;
    (void)tof_i2c_read(client, REG_BL_CMD_STAT, &status, sizeof(status));
    return BL_IS_CMD_BUSY(status);
}

static uint8_t * get_BL_cmd_buf(struct tof8801_BL_application *app)
{
    return app->BL_command.anon_cmd.data;
}

static uint8_t * get_BL_rsp_buf(struct tof8801_BL_application *app)
{
    return app->BL_response.anon_resp.data;
}

static void TMF8801_cr_recalc(TMF8801_cr * cr)
{
  //
  // start a new recalculation cycle, but don't lose the latest ratioQ15.
  //
  cr->count      = 0;
}

static void TMF8801_cr_addpair(TMF8801_cr * cr, uint32_t host, uint32_t i2c)
{
  uint32_t num = 0;
  uint32_t den = 0;
  uint32_t denQM15 = 0;
  //
  // add a pair of host and I2C times
  //
  if (i2c == 0)
  {
    return;
  }
  if (host < cr->last_host)
  {
    //wraparound case
    cr->count = 0;
  }
  if (i2c < cr->last_i2c)
  {
    //wraparound case
    cr->count = 0;
  }
  if (cr->count == 0)
  {
    cr->first_host = host;
    cr->first_i2c  = i2c;
  }
  cr->last_host  =  host;
  cr->last_i2c   =  i2c;
  cr->count      += 1;
  if (cr->count >= APP0_RATIO_MINCOUNT)
  {
    cr->trim_count += 1;
    num = cr->last_host - cr->first_host;
    den = cr->last_i2c  - cr->first_i2c;
    while ((num < APP0_CR_WRAPAROUND_DIV2) && (den < APP0_CR_WRAPAROUND_DIV2))
    {
        num <<= 1;
        den <<= 1;
    };
    denQM15 = ((den + (1 << 14)) >> 15);   // round up, always positive
    if ((denQM15 == 0) || (num > den)) {
      cr->count = 0;
    } else {
      cr->ratioQ15 = (num + (denQM15 >> 1))/denQM15;
    }
  }
}

static void TMF8801_cr_init(TMF8801_cr * cr)
{
    //
    // start a new recalculation cycle, beginning with the default ratio 0.2
    //
    cr->first_host = 0;
    cr->last_host  = 0;
    cr->first_i2c  = 0;
    cr->last_i2c   = 0;
    cr->ratioQ15   = APP0_EXP_FREQ_RATIO_Q15;
    cr->count      = 0;
    cr->trim_count = 0;
}

static int32_t tof8801_app0_read_fac_calib(struct tmf8801_ctx *tmf8801)
{
    return tof_i2c_read(tmf8801, REG_APP0_DATA_START,
                        tmf8801->app0_app.app0_fac_calib,
                        sizeof(tmf8801->app0_app.app0_fac_calib));
}

static int32_t tof8801_app0_write_cal_data(struct tmf8801_ctx *tmf8801)
{
    int32_t error = 0;
    uint8_t addr = REG_APP0_DATA_START;
    uint8_t cal_flags;

    cal_flags = tmf8801->app0_app.cal_flags;

    if (cal_flags & APP0_FAC_CAL_FLAG) {
        error = tof_i2c_write(tmf8801, addr, tmf8801->app0_app.app0_fac_calib, APP0_FAC_CALIB_SIZE);
        if (error)
            return -1;
        addr += APP0_FAC_CALIB_SIZE;
    }

    // ALG STATE is only valid if also passing in factory calibration data
    if ((cal_flags & APP0_ALG_STATE_FLAG) && (cal_flags & APP0_FAC_CAL_FLAG)) {
        // only first 3 Bytes of algo state should be cached for replay back to ToF
        memset(&tmf8801->app0_app.result.alg_state[3], 0,
                sizeof(tmf8801->app0_app.result.alg_state) - 3);
        error = tof_i2c_write(tmf8801, addr, tmf8801->app0_app.result.alg_state, APP0_ALG_STATE_SIZE);
        if (error)
            return -1;
        addr += APP0_ALG_STATE_SIZE;
    }

    if (cal_flags & APP0_CFG_DATA_FLAG) {
        error = tof_i2c_write(tmf8801, addr, tmf8801->app0_app.app0_cfg_data, APP0_CFG_DATA_SIZE);
        if (error)
            return -1;
        addr += APP0_CFG_DATA_SIZE;
    }
    return 0;
}

static int32_t get_usec_timestamp(uint32_t *usec_stamp)
{
    int32_t error = 0;
    struct timespec time;
    if (!usec_stamp)
        return -1;
    error = clock_gettime(CLOCK_MONOTONIC, &time);
    if (error < 0)
        return -1;
    *usec_stamp = (time.tv_sec * 1000000) + ((time.tv_nsec + 500) / 1000);
    return 0;
}

static uint32_t TMF8801_cr_map(TMF8801_cr * cr, uint32_t distance)
{
  //
  // apply the mapping function to calculate a clock-corrected distance
  //
  return (distance * APP0_EXPECTEDRATIO * cr->ratioQ15 + (1 << 14)) >> 15;
}

static int32_t tof8801_app0_osc_trim(struct tmf8801_ctx *tmf8801, int32_t change)
{
    int32_t error = 0;
    int32_t save_state;
    int32_t trim_val = 0;
    uint8_t fuse3 = 0;
    uint8_t fuse6 = 0;
    uint8_t cmd = 0;

    /*
       Oscillator Trimming procedure:
       1. write 0x06: 0x29
       2. write 0xE0 = 0x00 (PON0)
       3. read/write 0x03
       4. read/write 0x06 bit 6
       5. write 0xE0 = 0x01 (PON1)
       6. wait for CPU_READY, error if still 0x00 after timeout (chip reset)
       */
    cmd = APP0_CMD_OSC_TRIM;
    error = tof_i2c_write(tmf8801, REG_APP0_CMD_START, &cmd, 1);
    if (error)
        return error;

    error = tof8801_goto_standby(tmf8801);
    if (error)
        return error;

    /*** Start osc trimming ***/
    error = tof_i2c_read(tmf8801, 0x03, &fuse3, 1);
    if (error)
        return error;

    error = tof_i2c_read(tmf8801, 0x06, &fuse6, 1);
    if (error)
        return error;

    trim_val = (((int32_t)fuse3) << 1) | ((fuse6 & (0x01 << 6)) >> 6);
    trim_val += change;
    trim_val &= 0x1FF;
    fuse6 &= ~(0x01 << 6);
    fuse6 |= ((trim_val & 0x01) << 6);
    fuse3  = ((trim_val >> 1) & 0xFF);

    error = tof_i2c_write(tmf8801, 0x03, &fuse3, 1);
    if (error)
        return error;

    error = tof_i2c_write(tmf8801, 0x06, &fuse6, 1);
    if (error)
        return error;
    /*** End osc trimming ***/

    error = tof8801_wake_from_standby(tmf8801);
    if (error)
        return error;

    error = tof8801_app0_wait_for_idle(tmf8801, 50000);
    return error;
}


static int32_t tof8801_app0_osc_trim_up(struct tmf8801_ctx *tmf8801)
{
    int32_t error;

    error = tof8801_app0_stop_capture(tmf8801);
    if (error)
        return error;

    error =  tof8801_app0_osc_trim(tmf8801, 1);
    if (error)
        return error;

    error = tof8801_app0_start_capture(tmf8801);
    return error;
}

static int32_t tof8801_app0_osc_trim_down(struct tmf8801_ctx *tmf8801)
{
    int32_t error;

    error = tof8801_app0_stop_capture(tmf8801);
    if (error)
        return error;

    error = tof8801_app0_osc_trim(tmf8801, -1);
    if (error)
        return error;

    error = tof8801_app0_start_capture(tmf8801);
    return error;
}

static int32_t tof8801_app0_check_osc(struct tmf8801_ctx *tmf8801)
{
    int32_t error = 0;
    uint32_t ratioQ15;
    static bool stable = false;

    if (tmf8801->app0_app.clk_cr.trim_count >= APP0_DEFAULT_CLK_ITERATIONS) {

        ratioQ15 = tmf8801->app0_app.clk_cr.ratioQ15;
        ratioQ15 = ratioQ15 < APP0_MIN_FREQ_RATIO_Q15 ? APP0_MIN_FREQ_RATIO_Q15 : ratioQ15;
        ratioQ15 = ratioQ15 > APP0_MAX_FREQ_RATIO_Q15 ? APP0_MAX_FREQ_RATIO_Q15 : ratioQ15;
        tmf8801->app0_app.clk_cr.trim_count = 0;

        if (tmf8801->app0_app.clk_trim_enable) {
            if (ratioQ15 >= APP0_FREQ_RATIO_HITH_Q15) {
                error = tof8801_app0_osc_trim_up(tmf8801);
                stable = false;
            } else if (ratioQ15 <= APP0_FREQ_RATIO_LOTH_Q15) {
                stable = false;
                error = tof8801_app0_osc_trim_down(tmf8801);
            } else if (stable == 0) {
                //printf("%s: clk_ratio (Q15): %u is within tolerance\n",
                //        __func__, tmf8801->app0_app.clk_cr.ratioQ15);
                stable = true;
            }
        }
    }
    return error;
}

static int32_t tof8801_app0_clock_skew_correct_results(struct tmf8801_ctx *tmf8801)
{
    int32_t error = 0;
    static uint32_t last_stamp = 0;
    uint32_t usec_stamp;
    uint32_t old_dist = 0;
    uint32_t cr_dist = 0;
    uint16_t result;
    uint32_t tof_clk_cnt;

    error = get_usec_timestamp(&usec_stamp);
    if (error)
        return error;

    if ( abs((int32_t)(usec_stamp - last_stamp)) >= 60000000 ) {
        //reset the integration once/min
        TMF8801_cr_recalc(&tmf8801->app0_app.clk_cr);
        last_stamp = usec_stamp;
    }

    tof_clk_cnt = tmf8801->app0_app.result.sysclk[0]        |
                 (tmf8801->app0_app.result.sysclk[1] << 8)  |
                 (tmf8801->app0_app.result.sysclk[2] << 16) |
                 (tmf8801->app0_app.result.sysclk[3] << 24);
    old_dist = tmf8801->app0_app.result.distance_mm[0] |
              (tmf8801->app0_app.result.distance_mm[1] << 8);
    TMF8801_cr_addpair(&tmf8801->app0_app.clk_cr, usec_stamp, tof_clk_cnt);
    cr_dist = TMF8801_cr_map(&tmf8801->app0_app.clk_cr, old_dist);
    //printf( "clk_skew: host_ts: %u (us) dev_ts: %u (5MHz cnt) "
    //        "old_dist: %u new_dist: %u\n", usec_stamp, tof_clk_cnt,
    //        old_dist, cr_dist);
    if (cr_dist > APP0_MAX_DIST_MM) {
        // if corrected distance is out of range, remove the object detection
        cr_dist = 0;
        tmf8801->app0_app.result.info &= ~APP0_RESULT_CONF_MASK;
    }

    tmf8801->app0_app.result.distance_mm[0] = cr_dist & 0xFF;
    tmf8801->app0_app.result.distance_mm[1] = (cr_dist >> 8) & 0xFF;

    error = tof8801_app0_check_osc(tmf8801);
    return error;
}

static int32_t tof8801_app0_read_meas_result(struct tmf8801_ctx *tmf8801)
{
    int32_t error = 0;

    error = tof_i2c_read(tmf8801, REG_APP0_DATA_START,
                         tmf8801->app0_app.result.buf,
                         sizeof(tmf8801->app0_app.result.buf));
    if (error)
        return error;

    error = tof8801_app0_clock_skew_correct_results(tmf8801);
    return error;
}

/**
 * Poll CPU_STATUS register until CPU is ready for communication
 *
 * @param tmf8801: tmf8801 context pointer
 * @return 0 is success, negative is an error
 */
int32_t tof8801_wait_for_cpu_ready(struct tmf8801_ctx *tmf8801)
{
    uint8_t cpu_reg = 0;
    int32_t error = 0;
    int32_t retry = 0;

    if (tmf8801 == NULL)
        return -1;

    do {
        error = tof_i2c_read(tmf8801, REG_CPU_STATUS, &cpu_reg, 1);
        if (cpu_reg != CPU_READY_STATUS) {
            if (IS_CPU_SLEEP(cpu_reg)) {
                (void) tof8801_wake_from_standby(tmf8801);
            }
            usleep(200);
            retry++;
        }
    } while ((cpu_reg != CPU_READY_STATUS) && (retry < 3));

    if (cpu_reg != CPU_READY_STATUS || retry >= 3)
        return -1;

    return 0;
}

/**
 * Set PON0 for deep sleep / standby mode (RAM contents remain intact)
 *
 * @param tmf8801: tmf8801 context pointer
 * @return 0 is success, negative is an error
 */
int32_t tof8801_goto_standby(struct tmf8801_ctx * tmf8801)
{
    int32_t error;
    uint8_t cpu_reg;
    uint8_t clk_reg;

    if (!tmf8801)
        return -1;

    error = tof_i2c_read(tmf8801, REG_CPU_STATUS, &cpu_reg, 1);
    if (!error && !IS_CPU_SLEEP(cpu_reg)) {
        // set PON = 0 to go to sleep
        cpu_reg &= ~CPU_PON0_MASK;
        (void) tof_i2c_write(tmf8801, REG_CPU_STATUS, &cpu_reg, 1);
        usleep(1000);
        error = tof_i2c_read(tmf8801, REG_CLK_STATUS, &clk_reg, 1);
    }
    return error ? error : (CLK_PLL_LOWPOWER(clk_reg) ? 0 : -1);
}

/**
 * Set PON1 to wake from deep sleep / standby mode last active application
 * will resume
 * @param tmf8801: tmf8801 context pointer
 * @return 0 is success, negative is an error
 */
int32_t tof8801_wake_from_standby(struct tmf8801_ctx * tmf8801)
{
    int32_t error;
    uint8_t cpu_reg;

    if (!tmf8801)
        return -1;

    error = tof_i2c_read(tmf8801, REG_CPU_STATUS, &cpu_reg, 1);
    if (!error && IS_CPU_SLEEP(cpu_reg)) {
        // set PON = 1 to wake from deep sleep
        cpu_reg |= 0x01;
        error = tof_i2c_write(tmf8801, REG_CPU_STATUS, &cpu_reg, 1);
        usleep(200);
    }
    return error;
}

/**
 * Read registers from offset to end and print to stdout
 * @param tmf8801: tmf8801 context pointer
 * @param offset: start register
 * @param end: end register
 */
void tof8801_dump_i2c_regs(struct tmf8801_ctx * tmf8801, uint8_t offset, uint8_t end)
{
    uint32_t per_line = 4; // per line should always be a power of two
    uint32_t len = 0;
    uint32_t idx, per_line_idx;
    uint8_t regs[MAX_REGS];
    char debug[80];

    if (!tmf8801)
        return;

    offset &= ~(per_line - 1); // Byte boundary for nice printing
    while ((end & (per_line - 1)) != (per_line - 1)) end += 1;
    end = (end < offset) ? (offset+per_line) : end;
    (void) tof_i2c_read(tmf8801, offset, &regs[offset], (end - offset + 1));
    for (idx = offset; idx < end; idx += per_line) {
        memset(debug, 0, sizeof(debug));
        len += snprintf(debug, sizeof(debug) - len, "%02x: ", idx);
        for (per_line_idx = 0; per_line_idx < per_line; per_line_idx++) {
            len += snprintf(debug + len,
                    sizeof(debug) - len,
                    "%02x ", regs[idx+per_line_idx]);
        }
        len = 0;
        printf("%s\n", debug);
    }
}

/**
 * Initialize the Bootloader application structure
 *
 * @param BL_app: Bootloader app context pointer
 */
void tof8801_BL_init_app(struct tof8801_BL_application *BL_app)
{
    //Wipe all local settings for bootloader, (except app_id)
    memset(BL_app, 0, sizeof(struct tof8801_BL_application));
    BL_app->app_id = APP_ID_BOOTLOADER;
}

/**
 * Retry until cmd is executed and status is
 * available in local buffer
 *
 * @param client: pointer to tmf8801_ctx
 * @param BL_app: pointer to BL application struct
 * @param num_retries: how many times to retry waiting on status complete
 * @return 0 is success, negative is an error
 */
int32_t tof8801_BL_read_status(struct tmf8801_ctx *client,
        struct tof8801_BL_application *BL_app,
        uint32_t num_retries)
{
    int32_t error = 0;
    uint8_t *rbuf = get_BL_rsp_buf(BL_app);
    uint8_t *status = &BL_app->BL_response.short_resp.status;
    uint8_t *rdata_size = &BL_app->BL_response.short_resp.size;
    uint8_t chksum;
    do {
        num_retries -= 1;
        error = tof_i2c_read(client, REG_BL_CMD_STAT,
                             rbuf,   BL_I2C_HEADER_SIZE);
        if (error)
            continue;
        if (BL_IS_CMD_BUSY(*status)) {
            /* CMD is still executing, wait and retry */
            usleep(BL_CMD_WAIT_MSEC*1000);
            if (num_retries <= 0) {
                error = -1;
            }
            continue;
        }
        /* if we have reached here, the command has either succeeded or failed */
        /* read in data part and csum */
        error = tof_i2c_read(client, REG_BL_CMD_STAT,
                rbuf,   BL_CALC_BL_RSP_SIZE(*rdata_size));
        if (error)
            continue;
        chksum = (uint8_t) ~tof8801_calc_chksum(rbuf, BL_CALC_BL_RSP_SIZE(*rdata_size));
        if ((chksum != BL_VALID_CHKSUM) && (num_retries <= 0)) {
            return -1;
        }
        /* all done, break and return */
        break;
    } while ((error == 0) && (num_retries > 0));
    return error;
}

/**
 * calculate the chksum and return the result
 *
 * @param[in] data: pointer to data to chksum
 * @param size: number of elements to chksum over in data
 * @return 0 is success, negative is an error
 */
uint8_t tof8801_calc_chksum(const uint8_t *data, uint8_t size)
{
    uint32_t sum = 0;
    int32_t idx = 0;
    for (; idx < size; idx++) {
        sum += data[idx];
    }
    return (uint8_t) ~sum; /* 1's complement of lowest byte */
}

/**
 * Send a command and wait for response
 *
 * @param client: pointer to tmf8801_ctx
 * @param BL_app: pointer to BL application struct, result will be in BL_response
 * @return 0 is success, negative is an error
 */
int32_t tof8801_BL_send_rcv_cmd(struct tmf8801_ctx *client, struct tof8801_BL_application *BL_app)
{
    int32_t error = -1;
    uint8_t *wbuf = get_BL_cmd_buf(BL_app);
    uint8_t wsize = BL_CALC_BL_CMD_SIZE(wbuf[1]);
    if (is_BL_cmd_busy(client))
        return error;

    error = tof_i2c_write(client, REG_BL_CMD_STAT, wbuf, wsize);
    if (error)
        return error;

    return tof8801_BL_read_status(client, BL_app, 5);
}

/**
 * Send a command and do not wait for response.
 * This is primarily used for commands that reset the chip (RESET, RAM REMAP, etc)
 *
 * @param client: pointer to tmf8801_ctx
 * @param BL_app: pointer to BL application struct
 * @return 0 is success, negative is an error
 */
int32_t tof8801_BL_send_cmd(struct tmf8801_ctx *client, struct tof8801_BL_application *BL_app)
{
    int32_t error = -1;
    uint8_t *wbuf = get_BL_cmd_buf(BL_app);
    uint8_t wsize = BL_CALC_BL_CMD_SIZE(wbuf[1]);
    if (is_BL_cmd_busy(client))
        return error;

    error = tof_i2c_write(client, REG_BL_CMD_STAT, wbuf, wsize);
    if (error)
        return error;

    return error;
}

/**
 * Send a generic short cmd (no response expected) to the BL app
 *
 * @param client: pointer to tmf8801_ctx (struct tmf8801_ctx *)
 * @param BL_app: pointer to BL application struct
 * @param cmd_e: bootloader command to send
 * @return 0 is success, negative is an error
 */
int32_t tof8801_BL_short_cmd(struct tmf8801_ctx *client,
        struct tof8801_BL_application *BL_app,
        enum tof8801_bootloader_cmd cmd_e)
{
    struct tof8801_BL_short_cmd *cmd = &(BL_app->BL_command.short_cmd);
    cmd->command = cmd_e;
    cmd->size = 0;
    cmd->chksum = tof8801_calc_chksum(get_BL_cmd_buf(BL_app),
            BL_CALC_CHKSUM_SIZE(cmd->size));

    return tof8801_BL_send_rcv_cmd(client, BL_app);
}

/**
 * Send a soft reset cmd to the bootloader
 *
 * @param client: pointer to tmf8801_ctx (struct tmf8801_ctx *)
 * @param BL_app: pointer to BL application struct
 * @return 0 is success, negative is an error
 */
int32_t tof8801_BL_reset(struct tmf8801_ctx *client,
                         struct tof8801_BL_application *BL_app)
{
    int32_t error;
    struct tof8801_BL_short_cmd *cmd = &(BL_app->BL_command.short_cmd);
    cmd->command = BL_RESET;
    cmd->size = 0;
    cmd->chksum = tof8801_calc_chksum(get_BL_cmd_buf(BL_app), BL_CALC_CHKSUM_SIZE(cmd->size));

    error = tof8801_BL_send_cmd(client, BL_app);
    if (error)
        return error;
    return tof8801_wait_for_cpu_ready(client);
}

/**
 * Send a ram remap cmd to the bootloader
 *
 * @param client: pointer to tmf8801_ctx (struct tmf8801_ctx *)
 * @param BL_app: pointer to BL application struct
 * @return 0 is success, negative is an error
 */
int32_t tof8801_BL_ram_remap(struct tmf8801_ctx *client,
                             struct tof8801_BL_application *BL_app)
{
    int32_t error;
    struct tof8801_BL_short_cmd *cmd = &(BL_app->BL_command.short_cmd);
    cmd->command = BL_RAMREMAP_RESET;
    cmd->size = 0;
    cmd->chksum = tof8801_calc_chksum(get_BL_cmd_buf(BL_app), BL_CALC_CHKSUM_SIZE(cmd->size));

    error = tof8801_BL_send_cmd(client, BL_app);
    if (error)
        return error;
    return tof8801_wait_for_cpu_ready(client);
}

/**
 *  Set RAM pointer in the bootloader
 *
 * @param client: pointer to tmf8801_ctx
 * @param BL_app: pointer to BL application struct
 * @param addr: RAM addr to set
 * @return 0 is success, negative is an error
 */
int32_t tof8801_BL_addr_ram(struct tmf8801_ctx *client,
        struct tof8801_BL_application *BL_app,
        int32_t addr)
{
    struct tof8801_BL_addr_ram_cmd *cmd = &(BL_app->BL_command.addr_ram_cmd);
    cmd->command = BL_ADDR_RAM;
    cmd->size = 2;
    cmd->addr_lsb  = addr & 0xff;
    cmd->addr_msb  = (addr >> 8) & 0xff;
    cmd->chksum = tof8801_calc_chksum(get_BL_cmd_buf(BL_app),
            BL_CALC_CHKSUM_SIZE(cmd->size));

    return tof8801_BL_send_rcv_cmd(client, BL_app);
}

/**
 * Read from RAM a specific number of bytes
 *
 * @param client: pointer to tmf8801_ctx
 * @param BL_app: pointer to BL application struct
 * @param[out] rbuf: location to put read RAM values
 * @param len: length of rbuf
 * @return 0 is success, negative is an error
 */
int32_t tof8801_BL_read_ram(struct tmf8801_ctx *client,
        struct tof8801_BL_application *BL_app,
        uint8_t * rbuf, int32_t len)
{
    int32_t rc;
    struct tof8801_BL_read_ram_cmd *cmd = &(BL_app->BL_command.read_ram_cmd);
    struct tof8801_BL_read_ram_resp *rsp = &(BL_app->BL_response.read_ram_resp);
    int32_t num = 0;
    do {
        cmd->command = BL_R_RAM;
        cmd->size = 1;
        cmd->num_bytes = ((len - num) >= BL_I2C_MAX_DATA_SIZE) ?
            BL_I2C_MAX_DATA_SIZE : (uint8_t) (len - num);
        cmd->chksum = tof8801_calc_chksum(get_BL_cmd_buf(BL_app),
                BL_CALC_CHKSUM_SIZE(cmd->size));
        rc = tof8801_BL_send_rcv_cmd(client, BL_app);
        if (!rc) {
            /* command was successful, lets copy a batch of data over */
            if (rbuf)
                memcpy((rbuf + num), rsp->data, rsp->size);
            num += cmd->num_bytes;
        }
    } while ((num < len) && !rc);
    return rc;
}

/**
 * write to RAM a specific number of bytes
 *
 * @param client: pointer to tmf8801_ctx
 * @param BL_app: pointer to BL application struct
 * @param buf: pointer to buffer of bytes to write
 * @param len: number of bytes to write
 * @return 0 is success, negative is an error
 */
int32_t tof8801_BL_write_ram(struct tmf8801_ctx *client,
        struct tof8801_BL_application *BL_app,
        const uint8_t *buf, int32_t len)
{
    struct tof8801_BL_write_ram_cmd *cmd = &(BL_app->BL_command.write_ram_cmd);
    int32_t idx = 0;
    int32_t num = 0;
    uint8_t chunk_bytes = 0;
    int32_t rc;
    do {
        cmd->command = BL_W_RAM;
        chunk_bytes = ((len - num) > BL_I2C_MAX_DATA_SIZE) ?
            BL_I2C_MAX_DATA_SIZE : (uint8_t) (len - num);
        cmd->size = chunk_bytes;
        for(idx = 0; idx < cmd->size; idx++) {
            cmd->data[idx] = buf[num + idx];
        }
        /* add chksum to end */
        cmd->data[(uint8_t)cmd->size] =
            tof8801_calc_chksum(get_BL_cmd_buf(BL_app),
                    BL_CALC_CHKSUM_SIZE(cmd->size));
        rc = tof8801_BL_send_rcv_cmd(client, BL_app);
        if (!rc)
            num += chunk_bytes;
    } while ((num < len) && !rc);
    return rc;
}

/**
 * Initialize the salt value for downloads
 *
 * @param client: pointer to tmf8801_ctx
 * @param BL_app: pointer to BL application struct
 * @param salt:   salt value to set for upload
 * @return 0 is success, negative is an error
 */
int32_t tof8801_BL_upload_init(struct tmf8801_ctx *client,
                               struct tof8801_BL_application *BL_app,
                               uint8_t salt)
{
    struct tof8801_BL_upload_init_cmd *cmd = &(BL_app->BL_command.upload_init_cmd);
    cmd->command = BL_UPLOAD_INIT;
    cmd->size = 1;
    cmd->seed = salt;
    cmd->chksum = tof8801_calc_chksum(get_BL_cmd_buf(BL_app),
            BL_CALC_CHKSUM_SIZE(cmd->size));

    return tof8801_BL_send_rcv_cmd(client, BL_app);
}

/**
 * Download a binary image to the Bootloader
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @param start_addr: Start address to write the bin image
 * @param bin_buf[in]: Binary data to download
 * @param len: length of bin_buf data buffer
 * @return 0 is success, negative is an error
 */
int32_t tof8801_BL_bin_download(struct tmf8801_ctx *tmf8801, uint32_t start_addr,
                                const uint8_t *bin_buf, uint32_t len)
{
    int32_t error = 0;
    if (tmf8801 == NULL)
        return -1;

    error = tof8801_BL_upload_init(tmf8801, &tmf8801->BL_app, BL_DEFAULT_SALT);
    if (error) {
        return error;
    }

    error = tof8801_BL_addr_ram(tmf8801, &tmf8801->BL_app, start_addr);
    if (error) {
        return error;
    }

    error = tof8801_BL_write_ram(tmf8801, &tmf8801->BL_app, bin_buf, len);
    if (error) {
        return error;
    }

    return tof8801_BL_ram_remap(tmf8801, &tmf8801->BL_app);
}
/**
 * Initialize App0 context structure, read version info and enable interrupts
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_init_app(struct tmf8801_ctx *tmf8801)
{
    int32_t error = 0;
    if (tmf8801 == NULL)
        return -1;
    //Wipe all local settings for app0, (except app_id)
    memset(&tmf8801->app0_app, 0, sizeof(struct tof8801_app0_application));
    tmf8801->app0_app.cal_flags = APP0_NO_FLAG;
    tmf8801->app0_app.app_id = APP_ID_APP0;
    TMF8801_cr_init(&tmf8801->app0_app.clk_cr);
    tmf8801->app0_app.clk_trim_enable = true; // default always trim the clock
    error = tof8801_app0_read_version(tmf8801);
    if (error)
        return error;
    error = tof8801_app0_enable_int(tmf8801);
    if (error)
        return error;
    return error;
}

/**
 * Poll App0 until IDLE state is reached and is ready for more commands
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @param usec_timeout: poll timeout in microseconds
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_wait_for_idle(struct tmf8801_ctx *tmf8801, uint32_t usec_timeout)
{
    int32_t error = 0;
    uint32_t usec_sleep = usec_timeout / 5;
    uint32_t total_sleep = 0;
    uint8_t *state;
    uint8_t cpu_reg = 0;

    if (tmf8801 == NULL)
        return -1;

    state = &tmf8801->app0_app.status.state;

    do {
        usleep(usec_sleep);
        error = tof_i2c_read(tmf8801, REG_CPU_STATUS, &cpu_reg, 1);
        if (error == 0) {
            error = tof8801_app0_read_status(tmf8801);
            if ((error == 0) &&
                (cpu_reg == CPU_READY_STATUS) &&
                (*state == APP0_IDLE_STATE)) {
                return 0;
            }
        }
        total_sleep += usec_sleep;
    } while (total_sleep < usec_timeout);

    return -1;
}

/**
 * Read App0 firmware version information
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_read_version(struct tmf8801_ctx *tmf8801)
{
    int32_t error = 0;
    if (tmf8801 == NULL)
        return -1;

    error = tof_i2c_read(tmf8801, REG_APP0_MAJ_VER, &tmf8801->app0_app.version[0], 1);
    if (error) {
        return -1;
    }
    error = tof_i2c_read(tmf8801, REG_APP0_MIN_VER, &tmf8801->app0_app.version[1], 4);
    if (error) {
        return -1;
    }
    return 0;
}

/**
 * Enable App0 Result and Error interrupts
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_enable_int(struct tmf8801_ctx *tmf8801)
{
    uint8_t intenab = IRQ_RESULTS | IRQ_ERROR;

    if (tmf8801 == NULL)
        return -1;

    return tof_i2c_write(tmf8801, REG_INT_ENABLE, &intenab, 1);
}

/**
 * Issue the start factory calibration command
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_start_fac_calib(struct tmf8801_ctx *tmf8801)
{
    int32_t error = 0;
    if (tmf8801 == NULL)
        return -1;

    memset(tmf8801->app0_app.capture_cmd.buf, 0,
           sizeof(tmf8801->app0_app.capture_cmd.buf));

    tmf8801->app0_app.capture_cmd.cmd = APP0_CMD_FAC_CALIB;

    error = tof_i2c_write(tmf8801, REG_APP0_CMD_START,
                          tmf8801->app0_app.capture_cmd.buf,
                          sizeof(tmf8801->app0_app.capture_cmd.buf));
    return error;
}

/**
 * Initialize the capture parameters with default settings
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_set_default_capture_settings(struct tmf8801_ctx * tmf8801)
{
    if (tmf8801 == NULL)
        return -1;

    tmf8801->app0_app.capture_cmd.iterations[0] =
        (APP0_ITERATIONS_DEFAULT & 0xFF);
    tmf8801->app0_app.capture_cmd.iterations[1] =
        ((APP0_ITERATIONS_DEFAULT >> 8) & 0xFF);
    tmf8801->app0_app.capture_cmd.period = APP0_PERIOD_DEFAULT;
    tmf8801->app0_app.capture_cmd.noise_threshold = APP0_NOISE_THRSHLD_DEFAULT;
    tmf8801->app0_app.capture_cmd.delay = APP0_DELAY_DEFAULT;
    tmf8801->app0_app.capture_cmd.gpio = APP0_GPIO_DEFAULT;
    tmf8801->app0_app.capture_cmd.alg = APP0_ALG_DEFAULT;

    return 0;
}

/**
 * Start the App0 distance capture with current capture settings
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_start_capture(struct tmf8801_ctx *tmf8801)
{
    int32_t error = 0;
    if (tmf8801 == NULL)
        return -1;

    // flags for passing factory calibration, algorithm state, and config data
    tmf8801->app0_app.capture_cmd.data = tmf8801->app0_app.cal_flags & APP0_ALL_FLAG;
    error = tof8801_app0_write_cal_data(tmf8801);
    if (error)
        return -1;

    tmf8801->app0_app.capture_cmd.cmd = tmf8801->app0_app.capture_cmd.data ?
                                            APP0_CMD_START_W_CAL : APP0_CMD_START ;

    error = tof_i2c_write(tmf8801, REG_APP0_CMD_START,
                          tmf8801->app0_app.capture_cmd.buf,
                          sizeof(tmf8801->app0_app.capture_cmd.buf));
    return error;
}

/**
 * Stop all App0 measurements and go to idle
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_stop_capture(struct tmf8801_ctx *tmf8801)
{
    int32_t error = 0;
    uint8_t cmd = APP0_CMD_STOP;
    uint8_t irq_clr = 0;

    if (tmf8801 == NULL)
        return -1;

    error = tof_i2c_write(tmf8801, REG_INT_STATUS, &irq_clr, 1);
    if (error)
        return error;

    error = tof_i2c_write(tmf8801, REG_APP0_CMD, &cmd, 1);
    if (error)
        return error;

    return tof8801_app0_wait_for_idle(tmf8801, 200000);
}

/**
 * Return the current interrupt flags
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @return negative for error, current IRQ flags otherwise
 */
int32_t tof8801_app0_interrupt_status(struct tmf8801_ctx *tmf8801)
{
    int32_t error = 0;
    uint8_t irq_flags = 0;

    if (tmf8801 == NULL)
        return -1;

    error = tof_i2c_read(tmf8801, REG_INT_STATUS, &irq_flags, 1);
    if (error)
        return error;

    return irq_flags;
}

/**
 * Read the current status information from App0
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_read_status(struct tmf8801_ctx *tmf8801)
{
    return tof_i2c_read(tmf8801, REG_APP0_STATE,
                        tmf8801->app0_app.status.buf,
                        sizeof(tmf8801->app0_app.status.buf));
}

/**
 * Pass in the factory calibration to the context structure to enable download
 * before starting the next measurement
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @param[in] buf: factory calibration data
 * @param len: length of buf
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_set_fac_calib(struct tmf8801_ctx *tmf8801, const uint8_t *buf, uint32_t len)
{
    if (!tmf8801 || !buf || len < APP0_FAC_CALIB_SIZE)
        return -1;
    memcpy(tmf8801->app0_app.app0_fac_calib, buf, APP0_FAC_CALIB_SIZE);
    tmf8801->app0_app.cal_flags |= APP0_FAC_CAL_FLAG;
    return 0;
}

/**
 * Retrieve the factory calibration data from the context structure
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @param[out] buf: buffer to put factory cal data
 * @param len: length of buf
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_get_fac_calib(struct tmf8801_ctx *tmf8801, uint8_t *buf, uint32_t len)
{
    if (!tmf8801 || !buf || len < APP0_FAC_CALIB_SIZE)
        return -1;
    memcpy(buf, tmf8801->app0_app.app0_fac_calib, APP0_FAC_CALIB_SIZE);
    return 0;
}

/**
 * Pass in the algorithm state to the context structure to enable download
 * before starting the next measurement
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @param[in] buf: algorithm state data
 * @param len: length of buf
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_set_alg_state(struct tmf8801_ctx *tmf8801, const uint8_t *buf, uint32_t len)
{
    if (!tmf8801 || !buf || len < APP0_ALG_STATE_SIZE)
        return -1;
    memcpy(tmf8801->app0_app.result.alg_state, buf, APP0_ALG_STATE_SIZE);
    tmf8801->app0_app.cal_flags |= APP0_ALG_STATE_FLAG;
    return 0;
}

/**
 * Retrieve the algorithm state data from the context structure
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @param[out] buf: buffer to put algorithm state data
 * @param len: length of buf
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_get_alg_state(struct tmf8801_ctx *tmf8801, uint8_t *buf, uint32_t len)
{
    if (!tmf8801 || !buf || len < APP0_ALG_STATE_SIZE)
        return -1;
    memcpy(buf, tmf8801->app0_app.result.alg_state, APP0_ALG_STATE_SIZE);
    return 0;
}

/**
 * Pass in the configuration data to the context structure to enable download
 * before starting the next measurement
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @param[in] buf: configuration data
 * @param len: length of buf
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_set_cfg_data(struct tmf8801_ctx *tmf8801, const uint8_t *buf, uint32_t len)
{
    if (!tmf8801 || !buf || len < APP0_CFG_DATA_SIZE)
        return -1;
    memcpy(tmf8801->app0_app.app0_cfg_data, buf, APP0_CFG_DATA_SIZE);
    tmf8801->app0_app.cal_flags |= APP0_CFG_DATA_FLAG;
    return 0;
}

/**
 * Retrieve the configuration data from the context structure
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @param[out] buf: buffer to put configuration data
 * @param len: length of buf
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_get_cfg_data(struct tmf8801_ctx *tmf8801, uint8_t *buf, uint32_t len)
{
    if (!tmf8801 || !buf || len < APP0_CFG_DATA_SIZE)
        return -1;
    memcpy(buf, tmf8801->app0_app.app0_cfg_data, APP0_CFG_DATA_SIZE);
    return 0;
}

/**
 * Retrieve the last known result distance and confidence
 *
 * @param tmf8801: pointer to tmf8801_ctx
 * @param[out] dis_mm: distance value in millimeters of last result
 * @param[out] confidence: confidence value of reported distance
 * @return 0 is success, negative is an error
 */
int32_t tof8801_app0_get_last_result(struct tmf8801_ctx *tmf8801, uint32_t *dis_mm, uint32_t *confidence)
{
    if (!tmf8801 || !dis_mm || !confidence)
        return -1;
    *dis_mm = tmf8801->app0_app.result.distance_mm[0] |
             (tmf8801->app0_app.result.distance_mm[1] << 8);
    *confidence = APP0_CONF_FROM_RESULT_INFO(tmf8801->app0_app.result.info);
    return tmf8801->app0_app.result.res_num;
}

/**
 * Read the interrupt status and handle any interrupts this function will read
 * out any results or factory calibration data
 *
 * @param tmf8801: pointer to tmf8801_ctx
 */
void tof8801_app0_process_irq(struct tmf8801_ctx *tmf8801)
{
    uint8_t irq_flags = 0;

    if (tmf8801 == NULL)
        return;

    irq_flags = tof8801_app0_interrupt_status(tmf8801);

    if (irq_flags & IRQ_RESULTS) {
        // clear interrupt status
        (void) tof_i2c_write(tmf8801, REG_INT_STATUS, &irq_flags, sizeof(irq_flags));
        (void) tof8801_app0_read_status(tmf8801);
        switch (tmf8801->app0_app.status.contents) {
            case FAC_CALIB_CONTENTS_ID:
                (void) tof8801_app0_read_fac_calib(tmf8801);
                break;
            case MEAS_RESULT_CONTENTS_ID:
                (void) tof8801_app0_read_meas_result(tmf8801);
                break;
            default:
                break;
        }
    }
    return;
}
