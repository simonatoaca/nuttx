/****************************************************************************
 * drivers/input/drv2605l.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/input/ff.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/timers/timer.h>
#include <nuttx/timers/watchdog.h>
#include <nuttx/lib/lib.h>

#include <nuttx/config.h>
#include <nuttx/nuttx.h>

#include <stdio.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>

#include <nuttx/i2c/i2c_master.h>

#if defined(CONFIG_I2C) && defined(CONFIG_FF_DRV2605L)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define FF_DEVNAME_FMT      "/dev/input_ff%d"
#define FF_DEVNAME_MAX      32

#define FF_EVENT_START      1
#define FF_EVENT_STOP       0

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define DRV2605L_ADDR (0x5A)  /* I2C Slave Address */
#define DRV2605L_FREQ CONFIG_DRV2605L_I2C_FREQUENCY
#define DRV2605L_DEVID (0x7)

/* Register addresses */

#define DRV2605L_STATUS_REG_ADDR (0x0)  /* Device ID, Diag Result, Over Temp, OC Detect */
#define DRV2605L_MODE_REG_ADDR (0x01)   /* Device Reset, Standby, Mode */
#define DRV2605L_RTP_INPUT_REG_ADDR (0x02)

/* Library and waveform-related registers */

#define DRV2605L_LIB_SEL_REG_ADDR (0x03)
#define DRV2605L_WAV_FRM_SEQ_BASE_REG_ADDR (0x04)
#define DRV2605L_WAV_FRM_SEQ1_REG_ADDR (0x04)
#define DRV2605L_WAV_FRM_SEQ2_REG_ADDR (0x05)
#define DRV2605L_WAV_FRM_SEQ3_REG_ADDR (0x06)
#define DRV2605L_WAV_FRM_SEQ4_REG_ADDR (0x07)
#define DRV2605L_WAV_FRM_SEQ5_REG_ADDR (0x08)
#define DRV2605L_WAV_FRM_SEQ6_REG_ADDR (0x09)
#define DRV2605L_WAV_FRM_SEQ7_REG_ADDR (0x0A)
#define DRV2605L_WAV_FRM_SEQ8_REG_ADDR (0x0B)

#define DRV2605L_GO_REG_ADDR (0x0C)   /* Bit 1 is GO bit */

#define DRV2605L_ODT_REG_ADDR (0x0D)  /* Overdrive Time Offset */
#define DRV2605L_SPT_REG_ADDR (0x0E)  /* Sustain Time Offset, Positive Reg*/
#define DRV2605L_SND_REG_ADDR (0x0F)  /* Sustain Time Offset, Negative Reg*/
#define DRV2605L_BRT_REG_ADDR (0x10)  /* Brake Time Offset */

/* Audio-To-Vibe registers*/

#define DRV2605L_ATH_CTRL_REG_ADDR (0x11) /* Control register */
#define DRV2605L_ATH_MIN_INPUT_REG_ADDR (0x12)
#define DRV2605L_ATH_MAX_INPUT_REG_ADDR (0x13)
#define DRV2605L_ATH_MIN_DRIVE_REG_ADDR (0x14)
#define DRV2605L_ATH_MAX_DRIVE_REG_ADDR (0x15)

/* Voltage-related registers */

#define DRV2605L_RATED_VOLTAGE_REG_ADDR (0x16)
#define DRV2605L_OD_CLAMP_VOLTAGE_REG_ADDR (0x17)
#define DRV2605L_VBAT_REG_ADDR (0x21)

/* Auto-Calibration registers */

#define DRV2605L_A_CAL_COMP_REG_ADDR (0x18)
#define DRV2605L_A_CAL_BEMF_REG_ADDR (0x19)

#define DRV2605L_FEEDBACK_CTRL_REG_ADDR (0x1A)
#define DRV2605L_CTRL1_REG_ADDR (0x1B)
#define DRV2605L_CTRL2_REG_ADDR (0x1C)
#define DRV2605L_CTRL3_REG_ADDR (0x1D)
#define DRV2605L_CTRL4_REG_ADDR (0x1E)
#define DRV2605L_CTRL5_REG_ADDR (0x1F)

#define DRV2605L_OL_LRA_PERIOD_REG_ADDR (0x20) /* Open-loop period */
#define DRV2605L_LRA_PERIOD_REG_ADDR (0x22)    /* Resonance period */

/* Bit masks */

#define DRV2605L_DEVID_MASK_MSK (0xE0)
#define DRV2605L_DIAG_RESULT_MSK (1 << 3)
#define DRV2605L_OVERTEMP_MSK (1 << 1)
#define DRV2605L_OC_DETECT_MSK (1)

#define DRV2605L_DEV_RESET_MSK (1 << 7)
#define DRV2605L_STANDBY_MSK (1 << 6)
#define DRV2605L_MODE_MSK (0x7)

#define DRV2605L_HIZ_MSK (1 << 4)
#define DRV2605L_LIB_SEL_MSK (0x7)

#define DRV2605L_WAIT_BIT_MSK (1 << 7)

#define DRV2605L_N_ERM_LRA_MSK (1 << 7)
#define DRV2605L_FB_BRAKE_FACTOR_MSK (0x70)
#define DRV2605L_LOOP_GAIN_MSK (0xC)
#define DRV2605L_BEMF_GAIN_MSK (0x3)

#define DRV2605L_STARTUP_BOOST_MSK (1 << 7)
#define DRV2605L_AC_COUPLE_MSK (1 << 5)
#define DRV2605L_DRIVE_TIME_MSK (0x1F)

#define DRV2605L_BIDIR_INPUT_MSK (1 << 7)
#define DRV2605L_BRAKE_STABILIZER_MSK (1 << 6)
#define DRV2605L_SAMPLE_TIME_MSK ((1 << 4) | (1 <<< 5))
#define DRV2605L_BLANKING_TIME_MSK (0xC)
#define DRV2605L_IDISS_TIME_MSK (0x3)
#define DRV2605L_OL_LRA_PERIOD_MSK (0x7F)
#define DRV2605L_GO_BIT_MSK (1)

/* Result ids */

#define AUTO_CALIB_PASSED (0)
#define AUTO_CALIB_FAILED (1)

#define DIAG_PASSED AUTO_CALIB_PASSED
#define DIAG_FAILED AUTO_CALIB_FAILED /* Actuator missing/shorted/timed out */

#define OVER_TEMP_NORMAL (0)
#define OVER_TEMP_EXCEEDED (1)

#define NO_OVERCURRENT_DETECTED (0)
#define OVERCURRENT_DETECTED (1)

/* Standby bit */

#define DEVICE_READY (0)
#define DEVICE_STANDBY (1)

/* Modes */

#define MODE_INTERNAL_TRIGGER (0)
#define MODE_EXTERNAL_TRIGGER_EDGE (1)
#define MODE_EXTERNAL_TRIGGER_LEVEL (2)
#define MODE_PWM_ANALOG_INPUT (3)
#define MODE_AUDIO_TO_VIBE (4)
#define MODE_RTP (5)
#define MODE_DIAGNOSTICS (6)
#define MODE_AUTO_CALIB (7)

/* Control values */

#define FB_BRAKE_FACTOR_1X (0)
#define FB_BRAKE_FACTOR_2X (1)
#define FB_BRAKE_FACTOR_3X (2)
#define FB_BRAKE_FACTOR_4X (3)
#define FB_BRAKE_FACTOR_6X (4)
#define FB_BRAKE_FACTOR_8X (5)
#define FB_BRAKE_FACTOR_16X (6)
#define FB_BRAKE_FACTOR_DISABLED (7)

#define LOOP_GAIN_LOW (0)
#define LOOP_GAIN_MEDIUM (1)
#define LOOP_GAIN_HIGH (2)
#define LOOP_GAIN_VERY_HIGH (3)



/****************************************************************************
 * Private Types
 ****************************************************************************/

struct drv2605l_calib_s
{
  /* Input */

  uint8_t erm_lra;
  uint8_t fb_brake_factor;
  uint8_t loop_gain;
  uint8_t rated_voltage;
  uint8_t od_clamp;
  uint8_t auto_cal_time;
  uint8_t drive_time;

#ifdef LRA_ACTUATOR
  uint8_t sample_time;
  uint8_t blanking_time;
  uint8_t idiss_time;
  uint8_t zc_det_time;
#endif

  /* Output */

  uint8_t bemf_gain;
  uint8_t a_cal_comp;
  uint8_t a_cal_bemf;
  uint8_t diag_result;
};

struct drv2605l_dev_s
{
  bool enabled;                         /* Enable/Disable DRV2605L */
  struct ff_lowerhalf_s lower;
  FAR struct i2c_master_s *i2c;
  FAR struct ioexpander_dev_s *ioedev;
  FAR struct drv2605l_calib_s *calib;

  int en_pin;
  int int_pin;

  uint16_t max_effects;
  FAR struct ff_effect *effects;
  FAR struct ff_effect *current_effect;

  struct wdog_s wd_timer;
  struct work_s haptic_work;

  mutex_t dev_lock;
  mutex_t rtp_lock;

  float vbat;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void drv2605l_rom_timer_func(wdparm_t arg);


/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drv2605l_getregs
 *
 * Description:
 *   Read <length> bytes starting from a DRV2605L register addr
 *
 ****************************************************************************/

static int drv2605l_getregs(FAR struct drv2605l_dev_s *priv, uint8_t regaddr,
                            uint8_t *rxbuffer, uint8_t length)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = DRV2605L_FREQ;
  msg[0].addr = DRV2605L_ADDR;
  msg[0].flags = 0;
  msg[0].buffer = &regaddr;
  msg[0].length = 1;

  msg[1].frequency = DRV2605L_FREQ;
  msg[1].addr = DRV2605L_ADDR;
  msg[1].flags = I2C_M_READ;
  msg[1].buffer = rxbuffer;
  msg[1].length = length;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      ierr("I2C_TRANSFER failed: %d\n", ret);
      return -1;
    }

  return OK;
}

/****************************************************************************
 * Name: drv2605l_getreg8
 *
 * Description:
 *   Read from an 8-bit DRV2605L register
 *
 ****************************************************************************/

static uint8_t drv2605l_getreg8(FAR struct drv2605l_dev_s *priv,
                                uint8_t regaddr)
{
  struct i2c_msg_s msg[2];
  uint8_t regval = 0;
  int ret;

  msg[0].frequency = DRV2605L_FREQ;
  msg[0].addr = DRV2605L_ADDR;
  msg[0].flags = 0;
  msg[0].buffer = &regaddr;
  msg[0].length = 1;

  msg[1].frequency = DRV2605L_FREQ;
  msg[1].addr = DRV2605L_ADDR;
  msg[1].flags = I2C_M_READ;
  msg[1].buffer = &regval;
  msg[1].length = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      ierr("I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

  return regval;
}

/****************************************************************************
 * Name: drv2605l_putreg8
 *
 * Description:
 *   Write to an 8-bit DRV2605L register
 *
 ****************************************************************************/

static int drv2605l_putreg8(FAR struct drv2605l_dev_s *priv, uint8_t regaddr,
                            uint8_t regval)
{
  struct i2c_msg_s msg[2];
  uint8_t txbuffer[2];
  int ret;

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = DRV2605L_FREQ;
  msg[0].addr = DRV2605L_ADDR;
  msg[0].flags = 0;
  msg[0].buffer = txbuffer;
  msg[0].length = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      ierr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: drv2605l_checkid
 *
 * Description:
 *   Read and verify the DRV2605L chip ID
 *
 ****************************************************************************/

static int drv2605l_checkid(FAR struct drv2605l_dev_s *priv)
{
  uint8_t devid = 0;

  /* Read device ID */

  devid = drv2605l_getreg8(priv, DRV2605L_STATUS_REG_ADDR);
  up_mdelay(10);
  devid >>= 5;

  iinfo("devid: 0x%02x\n", devid);

  if (devid != (uint8_t)DRV2605L_DEVID)
    {
      /* ID is not Correct */

      ierr("Wrong Device ID! %02x\n", devid);
      return -ENODEV;
    }

  return OK;
}

static int drv2605l_haptic_upload_effect(FAR struct ff_lowerhalf_s *lower,
                                         FAR struct ff_effect *effect,
                                         FAR struct ff_effect *old)
{
  iinfo("called: effect_id = %d \n", effect->id);
  FAR struct drv2605l_dev_s *priv;

  priv = container_of(lower, FAR struct drv2605l_dev_s, lower);
  priv->effects[effect->id] = *effect;

  return OK;
}

static int drv2605l_haptic_erase(FAR struct ff_lowerhalf_s *lower,
                                  int effect_id)
{
  iinfo("called: effect_id = %d\n", effect_id);
  return OK;
}

static void drv2605l_rom_standby_work_routine(FAR void *arg)
{
  iinfo("start");
  uint8_t regval = 0;
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)arg;
  regval = drv2605l_getreg8(priv, DRV2605L_GO_REG_ADDR);

  if (regval) {
    wd_start(&priv->wd_timer, MSEC2TICK(300),
          drv2605l_rom_timer_func, (wdparm_t)priv);
    return;
  }

  /* Enter standby */

  drv2605l_putreg8(priv, DRV2605L_MODE_REG_ADDR, (1 << 6));
}

static void drv2605l_force_standby_work_routine(FAR void *arg)
{
  iinfo("start");
  uint8_t regval = 0;
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)arg;

  /* Enter standby */

  drv2605l_putreg8(priv, DRV2605L_MODE_REG_ADDR, (1 << 6));
}

static void drv2605l_rom_timer_func(wdparm_t arg)
{
  iinfo("start\n");
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)arg;

  work_queue(HPWORK, &priv->haptic_work,
             drv2605l_rom_standby_work_routine, priv, 0);
}

static void drv2605l_rtp_timer_func(wdparm_t arg)
{
  iinfo("start\n");
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)arg;

  work_queue(HPWORK, &priv->haptic_work,
             drv2605l_force_standby_work_routine, priv, 0);
}

static void drv2605l_rom_work_routine(FAR void *arg)
{
  iinfo("start\n");
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)arg;
  FAR struct ff_effect *effect = priv->current_effect;
  uint8_t current_wav_seq_reg = DRV2605L_WAV_FRM_SEQ_BASE_REG_ADDR;
  uint8_t data_len = MIN(effect->u.periodic.custom_len, 16);
  uint16_t data = 0;
  uint8_t regval = 0;
  uint8_t i = 0;

  /**
   *  Expected custom_data field:
   *  [effect_id, wait_time,  ...., wait_time, effect_id, wait_time]
   *  with a maximum of 8 non-zero fields
   *  (wait_time = 0 => not written to any registers)
   *  Maximum custom_len is 16 (8 pairs of wait time and id)
   */

  /* TODO: Move this to upload effect */
  for (i = 0; i < data_len; i++) {
    data = effect->u.periodic.custom_data[i];

    if (data) {
      if (i & 1) {
        /* Convert ms to register data: WAV_FRM_SEQ[6:0] * 10ms = wait time */

        regval = data / 10;
        regval |= (1 << 7);
      } else {
        /* Library effect index: [1, 123] */

        regval = MIN(data, 123);
      }

      drv2605l_putreg8(priv, current_wav_seq_reg, regval);

      current_wav_seq_reg++;
    }
  }

  if (current_wav_seq_reg <= DRV2605L_WAV_FRM_SEQ8_REG_ADDR) {
    drv2605l_putreg8(priv, current_wav_seq_reg, 0x0);
  }

  /* Play effects: MODE = Internal Trigger + Remove from standby */

  drv2605l_putreg8(priv, DRV2605L_MODE_REG_ADDR, 0x0);
  drv2605l_putreg8(priv, DRV2605L_GO_REG_ADDR, 0x01);

  /* Check periodically on the GO bit and put the driver in standby */

  wd_start(&priv->wd_timer, MSEC2TICK(250),
          drv2605l_rom_timer_func, (wdparm_t)priv);
}

static void drv2605l_rtp_work_routine(FAR void *arg)
{
  iinfo("start\n");
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)arg;
  FAR struct ff_effect *effect = priv->current_effect;

  drv2605l_putreg8(priv, DRV2605L_RTP_INPUT_REG_ADDR,
                   effect->u.constant.level);

  drv2605l_putreg8(priv, DRV2605L_MODE_REG_ADDR, MODE_RTP);

  wd_start(&priv->wd_timer, MSEC2TICK(effect->replay.length),
          drv2605l_rtp_timer_func, (wdparm_t)priv);
}

static void drv2605l_play_current_effect(FAR struct drv2605l_dev_s *priv)
{
  FAR struct ff_effect *effect;
  sclock_t time_us;

  time_us = wd_gettime(&priv->wd_timer);
  usleep(time_us);

  effect = priv->current_effect;

  if (!effect) {
    ierr("No effect selected!\n");
    return;
  }

  /* RTP for now, TODO: PWM, Analog maybe */

  if (effect->type == FF_CONSTANT) {
    work_queue(HPWORK, &priv->haptic_work,
                drv2605l_rtp_work_routine, priv,
                MSEC2TICK(effect->replay.delay));
    return;
  }

  /* Play effect from ROM or RTP/PWM */

  if (effect->type == FF_PERIODIC) {
    if (effect->u.periodic.waveform == FF_CUSTOM) {
      /* Play effect from ROM library */
#ifndef NO_LIBRARY
      work_queue(HPWORK, &priv->haptic_work,
                drv2605l_rom_work_routine, priv,
                MSEC2TICK(effect->replay.delay));
#else
      ierr("Cannot use FF_CUSTOM when no library is selected\n");
#endif /* !NO_LIBRARY */
      return;
    }
  }
}

static int drv2605l_haptic_playback(struct ff_lowerhalf_s *lower,
                                    int effect_id, int val)
{
  iinfo("called: effect_id = %d val = %d\n", effect_id, val);
  FAR struct drv2605l_dev_s *priv;

  priv = container_of(lower, FAR struct drv2605l_dev_s, lower);
  priv->current_effect = &priv->effects[effect_id];

  if (val == FF_EVENT_START) {
    drv2605l_play_current_effect(priv);
    return OK;
  }

  if (val == FF_EVENT_STOP) {
    work_cancel(HPWORK, &priv->haptic_work);
    return OK;
  }

  ierr("Unsupported event value!\n");
  return OK;
}

static void drv2605l_haptic_set_gain(FAR struct ff_lowerhalf_s *lower,
                                      uint16_t gain)
{
  iinfo("called: gain = %d\n", gain);
}

// static int drv2605l_pin_init(FAR struct drv2605l_dev_s *priv)
// {
//   int ret = 0;

//   ret = IOEXP_SETDIRECTION(priv->ioedev, priv->en_pin, IOEXPANDER_DIRECTION_OUT);
//   if (ret < 0)
//     {
//       ierr("ioexpander set direction error: %d\n", ret);
//       return -EIO;
//     }

//   ret = IOEXP_SETDIRECTION(priv->ioedev, priv->int_pin, IOEXPANDER_DIRECTION_OUT);
//   if (ret < 0)
//     {
//       ierr("ioexpander set direction error: %d\n", ret);
//       return -EIO;
//     }

//   return ret;
// }

static float drv2605l_get_vbat(FAR struct drv2605l_dev_s *priv)
{
  uint8_t regval = 0;
  regval = drv2605l_getreg8(priv, DRV2605L_VBAT_REG_ADDR);

  priv->vbat = regval * 5.6 / 255.0;

  iinfo("Battery level is at %.2f%%\n", priv->vbat * 100);

  return priv->vbat;
}

static int drv2605l_enable(FAR struct drv2605l_dev_s *priv, bool en)
{
  iinfo("enabling drv2605l: %d\n", en);
  int ret = 0;

  priv->enabled = en;

  /* Set / Unset enable pin */

  return ret;
}

static int drv2605l_set_mode(FAR struct drv2605l_dev_s *priv, uint8_t mode)
{
  iinfo("setting drv2605l mode: %d\n", mode);
  int ret = 0;


  return ret;
}

static int drv2605l_select_library(FAR struct drv2605l_dev_s *priv)
{
  uint8_t regval = 0;

  regval = drv2605l_getreg8(priv,  DRV2605L_LIB_SEL_REG_ADDR);
  regval &= ~(DRV2605L_LIB_SEL_MSK);

#ifdef TS2200_LIBRARY_A
  regval |= 1;
#endif

#ifdef TS2200_LIBRARY_B
  regval |= 2;
#endif

#ifdef TS2200_LIBRARY_C
  regval |= 3;
#endif

#ifdef TS2200_LIBRARY_D
  regval |= 4;
#endif

#ifdef TS2200_LIBRARY_E
  regval |= 5;
#endif

#ifdef TS2200_LIBRARY_F
  regval |= 7;
#endif

#ifdef LRA_LIBRARY
  regval |= 6;
#endif

  return drv2605l_putreg8(priv, DRV2605L_LIB_SEL_REG_ADDR, regval);
}

static int drv2605l_auto_calib(FAR struct drv2605l_dev_s *priv,
                               FAR struct drv2605l_calib_s *calib_data)
{
  iinfo("performing auto calibration\n");

  int ret = 0;
  uint8_t regval = 0;

  if (!calib_data) {
    /* Populate with default params */

    calib_data = kmm_zalloc(sizeof(struct drv2605l_calib_s));
    if (calib_data == NULL)
      {
        ierr("failed to allocate drv2605l_calib_s instance\n");
        return -ENOMEM;
      }

    calib_data->erm_lra = 0;

#ifdef LRA_ACTUATOR
    calib_data->erm_lra = 1;
#endif

    calib_data->fb_brake_factor = 3;
    calib_data->loop_gain = 1;
    calib_data->rated_voltage = 0x3e;
    calib_data->od_clamp = 0x8c;
    calib_data->auto_cal_time = 2;
    calib_data->drive_time = 0x13;

#ifdef LRA_ACTUATOR
    calib_data->sample_time = 3;
    calib_data->blanking_time = 1;
    calib_data->idiss_time = 1;
    calib_data->zc_det_time = 0;
#endif
  }

  priv->calib = calib_data;

  /* Place device in auto calibration mode */

  drv2605l_putreg8(priv, DRV2605L_MODE_REG_ADDR, MODE_AUTO_CALIB);

  /* Write auto calib params */

  regval = drv2605l_getreg8(priv, DRV2605L_FEEDBACK_CTRL_REG_ADDR);
  regval &= DRV2605L_BEMF_GAIN_MSK;
  regval |= (priv->calib->erm_lra << 7) |
            (priv->calib->fb_brake_factor << 4) |
            (priv->calib->loop_gain << 2);
  drv2605l_putreg8(priv, DRV2605L_FEEDBACK_CTRL_REG_ADDR, regval);
  drv2605l_putreg8(priv, DRV2605L_RATED_VOLTAGE_REG_ADDR, priv->calib->rated_voltage);
  drv2605l_putreg8(priv, DRV2605L_OD_CLAMP_VOLTAGE_REG_ADDR, priv->calib->od_clamp);

  regval = drv2605l_getreg8(priv, DRV2605L_CTRL1_REG_ADDR);
  regval |= (priv->calib->drive_time & DRV2605L_DRIVE_TIME_MSK);

  regval = drv2605l_getreg8(priv, DRV2605L_CTRL4_REG_ADDR);
  regval |= (priv->calib->auto_cal_time & 0x3) << 4;

#ifdef LRA_ACTUATOR
  regval |= priv->calib->zc_det_time << 6;
#endif

  drv2605l_putreg8(priv, DRV2605L_CTRL4_REG_ADDR, regval);

#ifdef LRA_ACTUATOR
  regval = drv2605l_getreg8(priv, DRV2605L_CTRL2_REG_ADDR);

  regval |= ((priv->calib->sample_time << 4) |
            (priv->calib->blanking_time << 2) |
            (priv->calib->idiss_time)) & ~((1 << 6) | (1 << 7));

  drv2605l_putreg8(priv, DRV2605L_CTRL2_REG_ADDR, regval);
#endif

  /* Trigger auto calibration */

  ret = drv2605l_putreg8(priv, DRV2605L_GO_REG_ADDR, 0x01);

  if (ret < 0)
    {
      return ret;
    }

  /* Store result if calibration was successful */

  do {
    usleep(2000);
    regval = drv2605l_getreg8(priv, DRV2605L_GO_REG_ADDR);
  } while (regval > 0);

  regval = drv2605l_getreg8(priv, DRV2605L_STATUS_REG_ADDR);

  if (regval & DRV2605L_DIAG_RESULT_MSK) {
    ierr("Auto calibration failed\n");
    return -EINVAL;
  }

  regval = drv2605l_getreg8(priv, DRV2605L_FEEDBACK_CTRL_REG_ADDR);
  priv->calib->bemf_gain = regval & 3;

  priv->calib->a_cal_comp = drv2605l_getreg8(priv, DRV2605L_A_CAL_COMP_REG_ADDR);
  priv->calib->a_cal_bemf = drv2605l_getreg8(priv, DRV2605L_A_CAL_BEMF_REG_ADDR);

  regval = drv2605l_getreg8(priv, DRV2605L_STATUS_REG_ADDR);
  priv->calib->diag_result = (regval & DRV2605L_DIAG_RESULT_MSK) >> 3;

  iinfo("diag_result: %d\nbemf_gain: %.2f\na_cal_comp: %.3f\na_cal_bemf: %.3f\n",
        priv->calib->diag_result, priv->calib->bemf_gain,
        (1 + priv->calib->a_cal_comp) / 255.0,
        (priv->calib->a_cal_bemf / 255.0) * 1.22 / priv->calib->bemf_gain);

  return ret;
}

static int drv2605l_init(FAR struct drv2605l_dev_s *priv,
                         FAR struct drv2605l_calib_s *calib_data)
{
  int ret = 0;
  uint8_t regval = 0;

  /* Init EN and IN/TRIG pins */

  // drv2605l_pin_init(priv);

  // usleep(250);

  // ret = drv2605l_enable(priv, true);

  // if (ret < 0)
  //   {
  //     return ret;
  //   }

  ret = drv2605l_checkid(priv);

  if (ret < 0)
    {
      return ret;
    }

  /* Put device in desired mode (Open-Loop, Closed-Loop) */

  regval = drv2605l_getreg8(priv, DRV2605L_CTRL3_REG_ADDR);

#ifdef OPEN_LOOP_MODE
#ifdef LRA_ACTUATOR
  regval |= 1;
#else
  regval |= (1 << 5);
#endif /* LRA_ACTUATOR */
#else
#ifdef LRA_ACTUATOR
  regval &= ~1;
#else
  regval &= ~(1 << 5);
#endif /* LRA_ACTUATOR */
#endif /* OPEN_LOOP_MODE */

  ret = drv2605l_putreg8(priv, DRV2605L_CTRL3_REG_ADDR, regval);

  regval = drv2605l_getreg8(priv, DRV2605L_CTRL2_REG_ADDR);

#ifdef CLOSED_LOOP_UNIDIR_MODE
  regval &= ~(1 << 7);
#else
  regval |= (1 << 7);
#endif

  ret = drv2605l_putreg8(priv, DRV2605L_CTRL2_REG_ADDR, regval);

  /* Start auto calibration */

  ret = drv2605l_auto_calib(priv, calib_data);

  if (ret < 0)
    {
      ierr("auto calibration failed\n");
      return ret;
    }

  /* Select ROM library */

  ret = drv2605l_select_library(priv);

  if (ret < 0)
    {
      return ret;
    }

  /* Standby and switch to internal trigger */

  ret = drv2605l_putreg8(priv, DRV2605L_MODE_REG_ADDR, (1 << 6));

  return ret;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

int drv2605l_register(int devno, FAR struct i2c_master_s *i2c,
                      FAR struct ioexpander_dev_s *ioedev,
                      FAR struct drv2605l_calib_s *calib_data)
{
  FAR struct drv2605l_dev_s *priv;
  char devpath[FF_DEVNAME_MAX];
  int ret = 0;
  int regval = 0;

  // DEBUGASSERT(i2c != NULL && ioedev != NULL);
  DEBUGASSERT(i2c != NULL);

  /* Initialize the drv2605l dev structure */

  priv = kmm_zalloc(sizeof(struct drv2605l_dev_s));
  if (!priv)
    {
      ierr("Failed to allocate drv2605l_dev_s instance\n");
      return -ENOMEM;
    }

  /* Init upper */

  /* FF_MAX_EFFECTS is the maximum number of effects that can be stored */

  priv->max_effects = FF_MAX_EFFECTS;
  priv->i2c = i2c;
  priv->ioedev = ioedev;
  priv->en_pin = CONFIG_DRV2605L_EN_PIN;
  priv->int_pin = CONFIG_DRV2605L_IN_TRIG_PIN;

  priv->effects = kmm_malloc(priv->max_effects * sizeof(*priv->effects));

  if (!priv->effects)
    {
      ierr("Failed to allocate effects array\n");
      kmm_free(priv);
      return -ENOMEM;
    }

  /* Init lowerhalf */

  priv->lower.upload          = drv2605l_haptic_upload_effect;
  priv->lower.erase           = drv2605l_haptic_erase;
  priv->lower.playback        = drv2605l_haptic_playback;
  priv->lower.set_gain        = drv2605l_haptic_set_gain;
  priv->lower.set_autocenter  = NULL;
  priv->lower.destroy         = NULL;

  /* Set up capabilities */

  set_bit(FF_CUSTOM, priv->lower.ffbit);
  set_bit(FF_CONSTANT, priv->lower.ffbit);
  set_bit(FF_PERIODIC, priv->lower.ffbit);

  /* Init device */

  ret = drv2605l_init(priv, calib_data);

  if (ret < 0)
    {
      goto err;
    }

  /* Register driver */

  snprintf(devpath, FF_DEVNAME_MAX, FF_DEVNAME_FMT, devno);

  ret = ff_register(&priv->lower, devpath, priv->max_effects);
  if (ret < 0)
    {
      ierr("Failed to register driver: %d\n", ret);
      goto err;
    }

  iinfo("drv2605l registered at %s\n", devpath);

  return OK;

err:
  /* Free memory */

  kmm_free(priv->effects);
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_FF_DRV2605L */
