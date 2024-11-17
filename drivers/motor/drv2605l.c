/****************************************************************************
 * drivers/motor/drv2605l.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/motor/drv2605l.h>

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

#if defined(CONFIG_I2C) && defined(CONFIG_HAPTIC_DRV2605L)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define DRV2605L_ADDR 0x5A  /* I2C Slave Address */
#define DRV2605L_FREQ CONFIG_DRV2605L_I2C_FREQUENCY
#define DRV2605L_DEVID 0x7

/* Register addresses */

#define DRV2605L_ID_REG_ADDR 0xE0
#define DRV2605L_DEV_RESET_ADDR 0x40

/* TODO: Add rest of registers and bit masks */


/****************************************************************************
 * Private Types
 ****************************************************************************/

struct drv2605l_dev_s
{
  FAR struct drv2605l_ops_s *ops;  /* drv2605l ops */
  FAR struct i2c_master_s *i2c;    /* I2C interface */
  bool enabled;                    /* Enable/Disable DRV2605L */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int drv2605l_setup(FAR struct stepper_lowerhalf_s *dev);
static int drv2605l_shutdown(FAR struct stepper_lowerhalf_s *dev);
static int drv2605l_work(FAR struct stepper_lowerhalf_s *dev,
                      FAR struct stepper_job_s const *param);
static int drv2605l_update_status(FAR struct stepper_lowerhalf_s *dev);
static int drv2605l_clear(FAR struct stepper_lowerhalf_s *dev, uint8_t fault);
static int drv2605l_idle(FAR struct stepper_lowerhalf_s *dev, uint8_t idle);
static int drv2605l_microstepping(FAR struct stepper_lowerhalf_s *dev,
                               uint16_t resolution);
static int drv2605l_ioctl(FAR struct stepper_lowerhalf_s *dev, int cmd,
                       unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct stepper_ops_s g_drv2605l_ops =
{
  drv2605l_setup,          /* setup */
  drv2605l_shutdown,       /* shutdown */
  drv2605l_work,           /* work */
  drv2605l_update_status,  /* update status */
  drv2605l_clear,          /* clear */
  drv2605l_idle,           /* idle */
  drv2605l_microstepping,  /* microstepping */
  drv2605l_ioctl           /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: drv2605l_getreg8
 *
 * Description:
 *   Read from an 8-bit DRV2605L register
 *
 ****************************************************************************/

static uint8_t drv2605l_getreg8(FAR struct drv2605l_dev_s *priv, uint8_t regaddr)
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
      snerr("I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

  return regval;
}

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
      snerr("I2C_TRANSFER failed: %d\n", ret);
      return -1;
    }

  return OK;
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
      snerr("I2C_TRANSFER failed: %d\n", ret);
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

  devid = drv2605l_getreg8(priv, DRV2605L_ID_REG_ADDR);
  up_mdelay(1);
  sninfo("devid: 0x%02x\n", devid);

  if (devid != (uint8_t)DRV2605L_DEVID)
    {
      /* ID is not Correct */

      snerr("Wrong Device ID! %02x\n", devid);
      return -ENODEV;
    }

  return OK;
}


static int drv2605l_setup(FAR struct stepper_lowerhalf_s *dev)
{
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)dev->priv;
  priv->ops->idle(false);
  priv->ops->enable(true);

  return 0;
}

static int drv2605l_shutdown(FAR struct stepper_lowerhalf_s *dev)
{
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)dev->priv;
  priv->ops->idle(true);
  priv->ops->enable(false);

  return 0;
}

static int drv2605l_work(FAR struct stepper_lowerhalf_s *dev,
                      FAR struct stepper_job_s const *job)
{
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)dev->priv;
  int delay;
  int count;

  if (priv->ops->fault())
    {
      /* In fault: do not proceed */

      return -EIO;
    }

  if (job->steps == 0)
    {
      /* Nothing to do */

      return 0;
    }

  /* Compute delay between pulse */

  delay = USEC_PER_SEC / job->speed;
  if (delay < 2)
    {
      delay = 2;
      stpwarn("Delay is clamped to 2 us\n");
    }

  stpinfo("Delay is %d us\n", delay);

  /* Set direction */

  if (job->steps > 0)
    {
      priv->ops->direction(true);
      count = job->steps;
    }
  else
    {
      priv->ops->direction(false);
      count = -job->steps;
    }

  if (priv->auto_idle)
    {
      priv->ops->idle(false);
      nxsig_usleep(USEC_PER_MSEC * 2);
    }

  dev->status.state = STEPPER_STATE_RUN;
  for (int32_t i = 0; i < count; ++i)
    {
      priv->ops->step(true);
      up_udelay(2);
      priv->ops->step(false);
      up_udelay(delay);
    }

  dev->status.state = STEPPER_STATE_READY;

  if (priv->auto_idle)
    {
      priv->ops->idle(true);
    }

  /* Update steps done (drv2605l cannot detect miss steps) */

  dev->status.position += job->steps;

  return 0;
}

static int drv2605l_update_status(FAR struct stepper_lowerhalf_s *dev)
{
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)dev->priv;

  if (priv->ops->fault())
    {
      /* The same pin is used for overtemp and overcurrent fault.
       * Since the current implementation is blocking and fetching
       * is on demand (no interrupt), it is impossible to detect
       * overcurrent.
       */

      dev->status.fault = STEPPER_FAULT_OVERTEMP;
      dev->status.state = STEPPER_STATE_FAULT;
    }

  return 0;
}

static int drv2605l_clear(FAR struct stepper_lowerhalf_s *dev, uint8_t fault)
{
  /* No fault to clear ever */

  dev->status.fault &= ~fault;
  if (dev->status.fault == STEPPER_FAULT_CLEAR)
    {
      dev->status.state = STEPPER_STATE_READY;
    }

  return 0;
}

static int drv2605l_idle(FAR struct stepper_lowerhalf_s *dev, uint8_t idle)
{
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)dev->priv;

  if (idle == STEPPER_AUTO_IDLE)
    {
      priv->auto_idle = true;
      return 0;
    }

  priv->auto_idle = false;

  if (idle == STEPPER_ENABLE_IDLE)
    {
      priv->ops->idle(true);
      dev->status.state = STEPPER_STATE_IDLE;
    }
  else
    {
      priv->ops->idle(false);
      nxsig_usleep(USEC_PER_MSEC * 2);
      dev->status.state = STEPPER_STATE_READY;
    }

  return 0;
}

static int drv2605l_microstepping(FAR struct stepper_lowerhalf_s *dev,
                               uint16_t resolution)
{
  FAR struct drv2605l_dev_s *priv = (FAR struct drv2605l_dev_s *)dev->priv;

  switch (resolution)
    {
      case 1:
        {
          priv->ops->microstepping(false, false, false);
        }
        break;

      case 2:
        {
          priv->ops->microstepping(true, false, false);
        }
        break;

      case 4:
        {
          priv->ops->microstepping(false, true, false);
        }
        break;

      case 8:
        {
          priv->ops->microstepping(true, true, false);
        }
        break;

      case 16:
        {
          priv->ops->microstepping(false, false, true);
        }
        break;

      case 32:
        {
          priv->ops->microstepping(true, true, true);
        }
        break;

      default:
        {
          return -EINVAL;
        }
    }

  return 0;
}

static int drv2605l_ioctl(FAR struct stepper_lowerhalf_s *dev, int cmd,
                       unsigned long arg)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int drv2605l_register(int devno, FAR struct i2c_master_s *i2c)
{
  FAR struct drv2605l_dev_s *priv;
  FAR struct stepper_lowerhalf_s *lower;
  int ret = 0;

  /* Initialize the drv2605l dev structure */

  priv = kmm_malloc(sizeof(struct drv2605l_dev_s));
  if (priv == NULL)
    {
      stperr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->ops = &g_drv2605l_ops;

  lower = kmm_malloc(sizeof(struct stepper_lowerhalf_s));
  if (lower == NULL)
    {
      stperr("Failed to allocate instance\n");
      kmm_free(priv);
      return -ENOMEM;
    }

  lower->priv = priv;
  lower->status.fault = STEPPER_FAULT_CLEAR;
  lower->status.state = STEPPER_STATE_READY;
  lower->status.position = 0;
  lower->ops = &g_drv2605l_ops;

  /* Initialize lower layer (only once) */

  priv->ops->initialize();

  /* Register the character driver */

  ret = stepper_register(devpath, lower);
  if (ret < 0)
    {
      stperr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
      kmm_free(lower);
      return ret;
    }

  stpinfo("drv2605l registered at %s\n", devpath);
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_HAPTIC_DRV2605L */
