/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_start.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_START_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_START_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Each S32K1xx board must provide the following initialized structure.
 *  This is needed to establish the initial board clocking.
 */

extern const struct clock_configuration_s g_initial_clkconfig;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_board_initialize
 *
 * Description:
 *   All S32K1XX architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after
 *   clocking and memory have been configured but before caches have been
 *   enabled and before any devices have been initialized.
 *
 ****************************************************************************/

void s32k1xx_board_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_S32K1XX_S32K1XX_START_H */
