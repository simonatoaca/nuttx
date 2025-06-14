/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_idle.c
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

#include <nuttx/config.h>

#include <debug.h>
#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/power/pm.h>
#include <nuttx/spinlock.h>

#include "xtensa.h"
#include "esp32s3_pm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Values for the RTC Alarm to wake up from the PM_STANDBY mode
 * (which corresponds to ESP32-C3 stop mode).  If this alarm expires,
 * the logic in this file will wakeup from PM_STANDBY mode and
 * transition to PM_SLEEP mode (ESP32-C3 standby mode).
 */

#ifdef CONFIG_PM
#ifndef CONFIG_PM_ALARM_SEC
#  define CONFIG_PM_ALARM_SEC 60
#endif

#ifndef CONFIG_PM_ALARM_NSEC
#  define CONFIG_PM_ALARM_NSEC 0
#endif

#ifndef CONFIG_PM_SLEEP_WAKEUP_SEC
#  define CONFIG_PM_SLEEP_WAKEUP_SEC 20
#endif

#ifndef CONFIG_PM_SLEEP_WAKEUP_NSEC
#  define CONFIG_PM_SLEEP_WAKEUP_NSEC 0
#endif

#define DEEP_SLEEP_THRESH_MS (10000) // 1000 ms = 1 s

#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PM
static spinlock_t g_esp32s3_idle_lock = SP_UNLOCKED;
static int light_sleep_incr = 0;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_PM
static void esp32s3_pm_handler(enum pm_state_e systemstate)
{
  switch (systemstate)
  {
  case PM_NORMAL:
    reset_light_sleep_ms();
    // esp32s3_pmstandby(200);
#  if XCHAL_HAVE_INTERRUPTS
  __asm__ __volatile__ ("waiti 0");
#  endif
    break;

  case PM_IDLE:
      if (get_light_sleep_ms() < DEEP_SLEEP_THRESH_MS)
        {
          esp32s3_pmstandby(30 * 1000000); // be careful 600ms
        }
      else
        {
          esp32s3_pmsleep(120 * 1000000 +
                          CONFIG_PM_SLEEP_WAKEUP_NSEC / 1000);
        }

    break;
  case PM_STANDBY:
  case PM_SLEEP:
    {
    /* Enter Force-sleep mode */
      esp32s3_pmsleep(CONFIG_PM_SLEEP_WAKEUP_SEC * 1000000 +
                      CONFIG_PM_SLEEP_WAKEUP_NSEC / 1000);
    }
    break;

  default:
    break;
  }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when their is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_idle(void)
{
  /* Report loop explanation here */

#ifdef CONFIG_ESP32S3_SPEED_UP_ISR
  for (; ; )
    {
#endif

#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
      /* If the system is idle and there are no timer interrupts, then
       * process "fake" timer interrupts. Hopefully, something will wake up.
       */

      nxsched_process_timer();
#else

      /* This would be an appropriate place to put some MCU-specific logic
       * to sleep in a reduced power mode until an interrupt occurs to save
       * power.
       */

  /* Perform IDLE mode power management */

#ifdef CONFIG_PM
  pm_idle(esp32s3_pm_handler);
#else
#  if XCHAL_HAVE_INTERRUPTS
  __asm__ __volatile__ ("waiti 0");
#  endif
#endif

#endif /* CONFIG_SUPPRESS_INTERRUPTS || CONFIG_SUPPRESS_TIMER_INTS */

#ifdef CONFIG_ESP32S3_SPEED_UP_ISR
    }
#endif
}
