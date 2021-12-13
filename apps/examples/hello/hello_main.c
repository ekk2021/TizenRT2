/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
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
 * examples/hello/hello_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <stdio.h>

/****************************************************************************
 * hello_main
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
static int _process_flash_write_thread() 
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
        // sleep(1);
    }

}

#if 1
void _process_flash_write() 
{
    pthread_t flash_thread, http_thread;
    pthread_attr_t attr;
    struct sched_param sparam;
    pthread_attr_init(&attr);

    pthread_attr_setstacksize(&attr, 1024*50);
    sparam.sched_priority = 100;
    pthread_attr_setschedparam(&attr, &sparam);

    if(pthread_create(&flash_thread, &attr, _process_flash_write_thread, 0) == 0) {
        pthread_setname_np(flash_thread, "flash thread");
        pthread_detach(flash_thread);
    } else {
        printf("fail to create flash write flash_thread\n");
        return;
    }
}
#endif

#define _T_MAX_DATA_SIZE    2048
void _process_uart2() 
{
    struct termios tio;
    int fd = 0;
    int ret = 1;
    unsigned char *pBuf = NULL;

    pBuf = (unsigned char *)malloc(_T_MAX_DATA_SIZE + 1);
    if(pBuf == NULL) {
        printf("pBuf malloc failed ... \n");
        return;
    }

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
#if 1
        ret = write(fd, pBuf, ret);
        printf("write lenth = %d\n", ret);
        if(ret <= 0) {
            break;
        }
#endif
        if(pBuf != NULL) {
            free(pBuf);
        }
    }

    if(pBuf != NULL) {
        free(NULL);
    }
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	printf("Hello, World!!\n");

    int ret;

#if 1
    ret = task_create("flash_write_task", 100, 1024, (main_t)_process_flash_write, NULL);
    if(ret < 0) {
        printf("flash thread creation failed\n");
    }
#endif

    ret = task_create("uart2_test", 100, 1024, (main_t)_process_uart2, NULL);
    if(ret < 0) {
        printf("uart2 task creation failed\n");
    }

	return 0;
}