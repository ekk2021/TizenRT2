/****************************************************************************
 *
 * Copyright 2022 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * configs/stm32f429i-disco/scripts/memory.ld
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/* The STM32F429ZIT has 2048Kb of FLASH beginning at address 0x0800:0000 and
 * 256Kb of SRAM. SRAM is split up into four blocks:
 *
 * 1) 112KB of SRAM beginning at address 0x2000:0000
 * 2)  16KB of SRAM beginning at address 0x2001:c000
 * 3)  64KB of SRAM beginning at address 0x2002:0000
 * 4)  64KB of CCM SRAM beginning at address 0x1000:0000
 *
 * When booting from FLASH, FLASH memory is aliased to address 0x0000:0000
 * where the code expects to begin execution by jumping to the entry point in
 * the 0x0800:0000 address range.
 *
 * For MPU support, the kernel-mode TinyAra section is assumed to be 128Kb of
 * FLASH and 4Kb of SRAM.  That is an excessive amount for the kernel which
 * should fit into 64KB and, of course, can be optimized as needed (See
 * also configs/stm32f429i-disco/scripts/kernel-space.ld).  Allowing the
 * additional does permit addition debug instrumentation to be added to the
 * kernel space without overflowing the partition.
 *
 * Alignment of the user space FLASH partition is also a critical factor:
 * The user space FLASH partition will be spanned with a single region of
 * size 2**n bytes.  The alignment of the user-space region must be the same.
 * As a consequence, as the user-space increases in size, the alignmment
 * requirement also increases.
 *
 * This alignment requirement means that the largest user space FLASH region
 * you can have will be 512KB at it would have to be positioned at
 * 0x08800000.  If you change this address, don't forget to change the
 * CONFIG_NUTTX_USERSPACE configuration setting to match and to modify
 * the check in kernel/userspace.c.
 *
 * For the same reasons, the maximum size of the SRAM mapping is limited to
 * 4KB.  Both of these alignment limitations could be reduced by using
 * multiple regions to map the FLASH/SDRAM range or perhaps with some
 * clever use of subregions.
 *
 * A detailed memory map for the 112KB SRAM region is as follows:
 *
 *   0x20000 0000:  Kernel .data region.  Typical size: 0.1KB
 *   ------- ----   Kernel .bss  region.  Typical size: 1.8KB
 *   0x20000 0800:  Kernel IDLE thread stack (approximate).  Size is
 *                  determined by CONFIG_IDLETHREAD_STACKSIZE and
 *                  adjustments for alignment.  Typical is 1KB.
 *   ------- ----   Padded to 4KB
 *   0x20000 1000:  User .data region.  Size is variable.
 *   ------- ----   User .bss region  Size is variable.
 *   0x20000 2000:  Beginning of kernel heap.  Size determined by
 *                  CONFIG_MM_KERNEL_HEAPSIZE.
 *   ------- ----   Beginning of user heap.  Can vary with other settings.
 *   0x20001 c000:  End+1 of CPU RAM
 */

MEMORY
{
    /* 1024Kb FLASH */

    kflash (rx)      : ORIGIN = 0x08000000, LENGTH = 128K
    uflash (rx)      : ORIGIN = 0x08020000, LENGTH = 128K
    xflash (rx)      : ORIGIN = 0x08040000, LENGTH = 768K

    /* 112Kb of contiguous SRAM */

    ksram (rwx)      : ORIGIN = 0x20000000, LENGTH = 4K
    usram (rwx)      : ORIGIN = 0x20001000, LENGTH = 4K
    xsram (rwx)      : ORIGIN = 0x20002000, LENGTH = 104K
}
