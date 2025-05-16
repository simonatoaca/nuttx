/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-hacktorwatch/src/esp32s3_buttons.c
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

#include <assert.h>
#include <debug.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

#include "esp32s3_gpio.h"
#include "hardware/esp32s3_gpio_sigmap.h"

#ifdef CONFIG_PM
#define RTCIO_PIN_FUNC                  0

#define RTC_MODE_SHIFT                  0
#define RTC_MODE_MASK                   (7 << RTC_MODE_SHIFT)
#  define RTC_INPUT                     (1 << (RTC_MODE_SHIFT + 0))
#  define RTC_OUTPUT                    (1 << (RTC_MODE_SHIFT + 1))

#define RTC_PULL_SHIFT                  2
#define RTC_PULL_MASK                   (7 << RTC_PULL_SHIFT)
#  define RTC_PULLUP                    (1 << (RTC_PULL_SHIFT + 0))
#  define RTC_PULLDOWN                  (1 << (RTC_PULL_SHIFT + 1))
#  define RTC_OPEN_DRAIN                (1 << (RTC_PULL_SHIFT + 2))

#define RTC_FUNCTION_SHIFT              5
#define RTC_FUNCTION_MASK               (3 << RTC_FUNCTION_SHIFT)
#  define RTC_FUNCTION_RTCIO            (1 << RTC_FUNCTION_SHIFT)
#  define RTC_FUNCTION_DIGITAL          (2 << RTC_FUNCTION_SHIFT)

#define RTC_DRIVE_SHIFT                 7
#define RTC_DRIVE_MASK                  (7 << RTC_DRIVE_SHIFT)
#  define RTC_DRIVE_0                   (1 << RTC_DRIVE_SHIFT)
#  define RTC_DRIVE_1                   (2 << RTC_DRIVE_SHIFT)
#  define RTC_DRIVE_2                   (3 << RTC_DRIVE_SHIFT)
#  define RTC_DRIVE_3                   (4 << RTC_DRIVE_SHIFT)

#define RTC_INPUT_PULLUP                (RTC_INPUT | RTC_PULLUP)
#define RTC_INPUT_PULLDOWN              (RTC_INPUT | RTC_PULLDOWN)
#define RTC_OUTPUT_OPEN_DRAIN           (RTC_OUTPUT | RTC_OPEN_DRAIN)
#define RTC_INPUT_FUNCTION_RTCIO        (RTC_INPUT | RTC_FUNCTION_RTCIO)
#define RTC_INPUT_FUNCTION_DIGITAL      (RTC_INPUT | RTC_FUNCTION_DIGITAL)
#define RTC_OUTPUT_FUNCTION_RTCIO       (RTC_OUTPUT | RTC_FUNCTION_RTCIO)
#define RTC_OUTPUT_FUNCTION_DIGITAL     (RTC_OUTPUT | RTC_FUNCTION_DIGITAL)

typedef uint16_t rtcio_pinattr_t;

extern int esp32s3_configrtcio(int rtcio_num, rtcio_pinattr_t attr);
extern void esp32s3_rtcioirqinitialize(void);
extern void esp32s3_rtcioirqenable(int irq);
extern void esp32s3_rtcioirqdisable(int irq);
extern void rtcio_ext0_set_wakeup_pin(uint32_t rtcio_num, int level);
#endif

#include "esp32s3-hacktorwatch.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct btn_data_s
  {
    uint32_t pin;
    gpio_pinattr_t attr;
#ifdef CONFIG_PM
    rtcio_pinattr_t rtc_attr;
#endif
  };

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct btn_data_s buttons[] =
  {
    {.pin = BUTTON_BOOT, .attr = INPUT_FUNCTION_2 | PULLUP,
#ifdef CONFIG_PM
      .rtc_attr = RTC_FUNCTION_RTCIO | RTC_INPUT_PULLUP
#endif
    },
    {.pin = BUTTON_UP, .attr = INPUT_FUNCTION_2 | PULLUP,
#ifdef CONFIG_PM
      .rtc_attr = RTC_FUNCTION_RTCIO | RTC_INPUT_PULLUP
#endif
    },
    {.pin = BUTTON_DOWN, .attr = INPUT_FUNCTION_2 | PULLUP,
#ifdef CONFIG_PM
      .rtc_attr = RTC_FUNCTION_RTCIO | RTC_INPUT_PULLUP
#endif
    },
  };

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  uint32_t btn_num = sizeof(buttons) / sizeof(struct btn_data_s);

#if defined(CONFIG_PM) && defined(CONFIG_ESP32S3_RTCIO_IRQ)
  esp32s3_rtcioirqinitialize();
#endif

  for (uint32_t i = 0; i < btn_num; i++)
    {
#ifdef CONFIG_PM
      // if (buttons[i].pin == BUTTON_UP)
      //   {
      //     rtcio_ext0_set_wakeup_pin(buttons[i].pin, 1);
      //   }

      if (buttons[i].pin < 22)
        {
          /** Config BOOT and UP buttons as sleep wakeup sources.
           *  The last RTC GPIO is 21 and the DOWN button is using 26.
           */
          btn_num += esp32s3_configrtcio(buttons[i].pin, buttons[i].rtc_attr);
        }
      else
#endif
        {
           btn_num += esp32s3_configgpio(buttons[i].pin, buttons[i].attr);
        }
    }

  return btn_num;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   8-bit bit set with each bit associated with a button.  See the
 *   BUTTON_*_BIT  definitions in board.h for the meaning of each bit.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint8_t ret = 0;
  int i = 0;
  int j = 0;
  int n = 0;
  uint32_t btn_num = sizeof(buttons) / sizeof(struct btn_data_s);

  for (i = 0; i < btn_num; i++)
    {
      bool b0 = esp32s3_gpioread(buttons[i].pin);

      for (j = 0; j < 10; j++)
        {
          up_mdelay(1);

          bool b1 = esp32s3_gpioread(buttons[i].pin);

          if (b0 == b1)
            {
              n++;
            }
          else
            {
              n = 0;
            }

          if (3 == n)
            {
              break;
            }

          b0 = b1;
        }

      iinfo("b=%d n=%d\n", b0, n);

      /* Low value means that the button is pressed */

      if (!b0)
        {
          ret |= (i + 1);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released.  The ID value is
 *   a button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret;
  DEBUGASSERT(id == 0);

  int irq = ESP32S3_PIN2IRQ(buttons[id].pin);

#if defined(CONFIG_PM) && defined(CONFIG_ESP32S3_RTCIO_IRQ)
  int rtc_irq = ESP32S3_IRQ_RTC_SLP_WAKEUP;
#endif

  if (irqhandler != NULL)
    {
      /* Make sure the interrupt is disabled */

      esp32s3_gpioirqdisable(irq);

#if defined(CONFIG_PM) && defined(CONFIG_ESP32S3_RTCIO_IRQ)
      if (buttons[id].pin < 22)
        {
          esp32s3_rtcioirqdisable(rtc_irq);
          ret = irq_attach(rtc_irq, irqhandler, arg);
        }
#endif
      ret = irq_attach(irq, irqhandler, arg);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: irq_attach() failed: %d\n", ret);
          return ret;
        }

      gpioinfo("Attach %p\n", irqhandler);

      gpioinfo("Enabling the interrupt\n");

      /* Configure the interrupt for rising and falling edges */

      esp32s3_gpioirqenable(irq, CHANGE);
#if defined(CONFIG_PM) && defined(CONFIG_ESP32S3_RTCIO_IRQ)
      if (buttons[id].pin < 22)
        {
          esp32s3_rtcioirqenable(rtc_irq);
        }
#endif
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      esp32s3_gpioirqdisable(irq);
#if defined(CONFIG_PM) && defined(CONFIG_ESP32S3_RTCIO_IRQ)
      if (buttons[id].pin < 22)
        {
          esp32s3_rtcioirqdisable(rtc_irq);
        }
#endif
    }

  return OK;
}
#endif
