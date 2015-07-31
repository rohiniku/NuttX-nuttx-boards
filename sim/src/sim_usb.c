/************************************************************************************
 * configs/sim/src/sim_usb.c
 *
 *   Copyright (C) 2012-2013, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "up_internal.h"
#include "sim.h"

#ifdef CONFIG_SIM_USB

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if defined(CONFIG_USBHOST)
#  define HAVE_USB 1
#else
#  warning "CONFIG_SIM_USB is enabled but not CONFIG_USBHOST"
#  undef HAVE_USB
#endif

#ifndef CONFIG_SIM_USBHOST_PRIO
#  define CONFIG_SIM_USBHOST_PRIO 100
#endif

#ifndef CONFIG_SIM_USBHOST_STACKSIZE
#  define CONFIG_SIM_USBHOST_STACKSIZE 1024
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: usbhost_waiter
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ************************************************************************************/

#ifdef CONFIG_USBHOST
static int usbhost_waiter(int argc, char *argv[])
{
  struct usbhost_hubport_s *hport;

  uvdbg("Running\n");
  for (;;)
    {
      /* Wait for the device to change state */
      sim_libusb_handle_events();
      usleep(100000);
    }

  /* Keep the compiler from complaining */

  return 0;
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/***********************************************************************************
 * Name: sim_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ***********************************************************************************/

#ifdef CONFIG_USBHOST
int sim_usbhost_initialize(void)
{
  int ret;
  int pid;
  /* First, register all of the class drivers needed to support the drivers
   * that we care about:
   */

  uvdbg("Register class drivers\n");

#ifdef CONFIG_USBHOST_HUB
  /* Initialize USB hub class support */

  ret = usbhost_hub_initialize();
  if (ret < 0)
    {
      udbg("ERROR: usbhost_hub_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_MSC
  /* Register the USB mass storage class class */

  ret = usbhost_msc_initialize();
  if (ret != OK)
    {
      udbg("ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_CDCACM
  /* Register the CDC/ACM serial class */

  ret = usbhost_cdcacm_initialize();
  if (ret != OK)
    {
      udbg("ERROR: Failed to register the CDC/ACM serial class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD
  /* Initialize the HID keyboard class */

  ret = usbhost_kbdinit();
  if (ret != OK)
    {
      udbg("Failed to register the HID keyboard class\n");
    }
#endif

#ifdef CONFIG_USBHOST_HIDMOUSE
  /* Initialize the HID mouse class */

  ret = usbhost_mouse_init();
  if (ret != OK)
    {
      udbg("Failed to register the HID mouse class\n");
    }
#endif

  /* Then get an instance of the USB host interface */

  uvdbg("Initialize USB host\n");
  ret = sim_usbhost_drvr_initialize();
  if (ret == OK)
    {
      /* Use the libusb hotplug driver instead of a nuttx task */

      uvdbg("Start usbhost_waiter\n");

      ret = sim_usbhost_hotplug_initialize();
      if (ret != OK)
        {
          udbg("Failed to register libusb hotplug detection\n");
          return -ENODEV;
        }
      pid = task_create("usbhost", CONFIG_SIM_USBHOST_PRIO,
                        CONFIG_SIM_USBHOST_STACKSIZE,
                        (main_t)usbhost_waiter, (FAR char * const *)NULL);
      return pid < 0 ? -ENOEXEC : OK;
    }

  return -ENODEV;
}
#endif

#endif /* CONFIG_SIM_USB */
