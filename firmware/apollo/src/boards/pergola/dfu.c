/**
 * DFU Runtime Support
 *
 * This file provides support for automatically rebooting into the DFU bootloader.
 *
 * This file is part of LUNA.
 *
 * Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tusb.h"

/**
 * Handler for DFU_DETACH events, which should cause us to reboot into the bootloader.
 */
void tud_dfu_rt_reboot_to_dfu(void)
{
    //TODO: Enter UF2 bootloader
}
