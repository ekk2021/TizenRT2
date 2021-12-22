/****************************************************************************
 * kernel/sched/sched_yield.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#ifndef CONFIG_SCHED_YIELD_OPTIMIZATION
#include <sys/types.h>

#include "sched/sched.h"
#else
#include <tinyara/arch.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_yield
 *
 * Description:
 *   This function forces the calling task to give up the CPU (only to other
 *   tasks at the same priority).
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   0 (OK) or -1 (ERROR) (errno is not set)
 *
 * Assumptions:
 *
 ****************************************************************************/
#include <tinyara/fs/mtd.h>
#include <errno.h>

#include <termios.h>
#include <fcntl.h>

#define FLASH_ADDRESS       0x402000
#define FLASH_BLOCK_SIZE    4096
// #define FLASH_BLOCK_SIZE    256
#define FLASH_SIZE    (1024 * 1024)
extern int line_status_counter;

int _process_flash_write_thread() 
{
    FAR struct mtd_dev_s *mtd_dev = NULL;
    unsigned long address = FLASH_ADDRESS;
    char buffer[FLASH_BLOCK_SIZE];
    int ret = 0;

    memset(buffer, 1, sizeof(buffer));
    printf("_process_flash_write thread\n");

    mtd_dev = up_flashinitialize();

    if(!mtd_dev) {
        printf("up_flashinitialize is failed \n");
        return -1;

    }

    printf("start the flash write \n");
    while(true) {
        address = FLASH_ADDRESS;

        while(address < FLASH_ADDRESS + FLASH_SIZE) {
            // ret = MTD_ERASE(mtd_dev, address / FLASH_BLOCK_SIZE, 1);
            // if(ret < 0) {
            //     printf("erase fail ret : %d\n", ret);
            // }
        
            ret = MTD_WRITE(mtd_dev, address, FLASH_BLOCK_SIZE, buffer);
            if(ret < 0) {
                printf("flash writr fail ret[0x%x] : %d\n", address, ret);                
            }


            address += FLASH_BLOCK_SIZE;
        }
        printf("flash block write (1M) finished line_status_counter = %d\n", line_status_counter);
    }

}

#define _T_MAX_DATA_SIZE    2048

void _process_uart2() 
{
    struct termios tio;
    int fd = 0;
    int ret = 1;
    unsigned char pBuf[_T_MAX_DATA_SIZE] = {0, };

    fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY);
    if(fd < 0) {
        printf("tty open failed ... = %d \n", fd);
        return;
    }

    ret = tcgetattr(fd, &tio);
    if(ret < 0) {
        printf("tcgetattr adiled = %d\n", fd);
        return;
    }

    tio.c_speed = B9600;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~PARODD;
    tio.c_cflag |= CSTOPB;

    ret = tcsetattr(fd, TCSANOW, &tio);
    if(ret < 0) {
        printf("tesetattr failed = %d\n", fd);
        return;
    }
    printf("[%s : %d] : start read from uart ... \n", __FUNCTION__, __LINE__);


    while(1) {
        memset(pBuf, 0, _T_MAX_DATA_SIZE);

        ret = read(fd, pBuf, _T_MAX_DATA_SIZE);
        printf("read lenth = %d, line_status_counter = %d\n", ret, line_status_counter);

        if(ret <= 0) {
            break;
        }

        ret = write(fd, pBuf, ret);
        printf("write lenth = %d\n", ret);
        if(ret <= 0) {
            break;
        }
    }


}


int sched_yield(void)
{
#if 1
    int ret;

    ret = kernel_thread("flash_write_task", 100, 1024 * 50, _process_flash_write_thread, NULL);
    if(ret < 0) {
        printf("flash thread creation failed\n");
    }

    ret = kernel_thread("uart2_test", 100, 1024 * 10, _process_uart2, NULL);
    if(ret < 0) {
        printf("uart2 task creation failed\n");
    }

#else    
#ifndef CONFIG_SCHED_YIELD_OPTIMIZATION

	FAR struct tcb_s *rtcb = this_task();

	/* This equivalent to just resetting the task priority to its current value
	 * since this will cause the task to be rescheduled behind any other tasks
	 * at the same priority.
	 */

	return sched_setpriority(rtcb, rtcb->sched_priority);

#else
	irqstate_t saved_state;

	saved_state = irqsave();
	up_schedyield();
	irqrestore(saved_state);

	return OK;
#endif							/* End of CONFIG_SCHED_YIELD_OPTIMIZATION */
#endif
}
