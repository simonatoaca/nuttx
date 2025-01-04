/****************************************************************************
 * include/nuttx/input/drv2605l.h
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

#ifndef __INCLUDE_NUTTX_INPUT_DRV2605L_H_
#define __INCLUDE_NUTTX_INPUT_DRV2605L_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdbool.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Types
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: drv2605l_register
 *
 * Description:
 *   drv2605l haptic driver initialize
 *
 * Input Parameters:
 *   devno  - ff device number
 *   master - i2c master param
 *   ioedev - io dev pin set
 *   calib_data - drv2605l_calib_s structure with calibration parameters
 *                or NULL if default values are used
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int drv2605l_register(int devno, FAR struct i2c_master_s *i2c,
                      FAR struct ioexpander_dev_s *ioedev,
                      FAR struct drv2605l_calib_s *calib_data);

#endif /* __INCLUDE_NUTTX_INPUT_DRV2605L_H_ */
