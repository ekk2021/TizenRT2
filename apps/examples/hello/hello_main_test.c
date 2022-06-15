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

#include <security/security_common.h>
#include <security/security_api.h>
#include <tinyara/seclink.h>
#include <tinyara/security_hal.h>
#include <security/security_ss.h>
#include <security/security_auth.h>
#include <security/security_keymgr.h>

/****************************************************************************
 * hello_main
 ****************************************************************************/
int se_read(int slot_number, unsigned char *buffer, unsigned int buffer_size, unsigned int *stored_data_size)
{
	security_error ret = 0;
	security_handle hnd;
	ret = security_init(&hnd);
	if (ret != SECURITY_OK) {
		return -1;
	}

	sleep(1);
	dbg("=====================%d==================================================\n",__LINE__);
	sleep(1);
	security_data ss_data = { NULL, buffer_size };
	char ss_path[7] = { 0, };
	snprintf(ss_path, 7, "ss/%d", slot_number);
	dbg("=====================%d==================================================\n",__LINE__);
	sleep(1);
	ret = ss_read_secure_storage(hnd, ss_path, 0, &ss_data);
	if (ret != SECURITY_OK) {
		security_deinit(hnd);
		return -1;
	}
	sleep(1);
	dbg("=====================%d==================================================\n",__LINE__);
	sleep(1);

	dbg("Line: %d, buffer_size: %d \n",__LINE__,buffer_size);
	dbg("Line: %d, lenght: %d \n",__LINE__,ss_data.length);
	dbg("=====================%d==================================================\n",__LINE__);
	if(buffer_size < ss_data.length)
		memcpy(buffer, ss_data.data, buffer_size);
	else
		memcpy(buffer, ss_data.data, ss_data.length);
	dbg("=====================%d==================================================\n",__LINE__);

	free(ss_data.data);
	*stored_data_size = ss_data.length;
	(void)security_deinit(hnd);
	return 0;
}

static unsigned char buff_read[8092];
static int se_read_thread(int argc, char *argv[])
{
	unsigned int read_data_size = 0;
	while(1) {
		memset(buff_read,0xff, 4900);
		dbg("=====================%d==================================================\n",__LINE__);
		se_read(0, buff_read, sizeof(buff_read), &read_data_size);

		sleep(1);
		dbg("=====================%d===============%d===================================\n",__LINE__,read_data_size);
		sleep(1);
		dbg("%s\n", buff_read);
		sleep(1);

		dbg("=====================%d==================================================\n",__LINE__);
		read_data_size = 0;
		sleep(3);
	}
	return 0;
}

/*
int flash_erase(unsigned int dev_addr, unsigned int erasesize)
{
	unsigned int blockcnt = 0;
	int ret = 0;
	if (dev_addr == 0)
		return -1;

	blockcnt = (erasesize + BLOCK_SIZE - 1) / BLOCK_SIZE;
	if ((dev_addr % BLOCK_SIZE) || (erasesize % BLOCK_SIZE))
		return -1;

	ret = MTD_ERASE(dev_mtd , dev_addr / BLOCK_SIZE, blockcnt);
	if(ret < 0)
	{
		printf("MTD_ERASE error: %d\n",ret);
		return -1;
	}
	return 0;
}
*/

#include <tinyara/fs/mtd.h>
#include <errno.h>

#define FLASH_ADDRESS       0x402000
#define FLASH_BLOCK_SIZE    4096
// #define FLASH_BLOCK_SIZE    256
#define FLASH_SIZE    (1024 * 1024)
static int flash_erase_thread(int argc, char *argv[])
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

        printf("[%s] FLASH ERASE \n", __func__);
        ret = MTD_ERASE(mtd_dev, address / FLASH_BLOCK_SIZE, 1);
        if(ret < 0) {
            printf("erase fail ret : %d\n", ret);
		}
        sleep(3);
    }
	

	return 0;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	printf("Hello, World!!\n");

	int tid;
	tid = task_create("se_read_thread", 100, 1024, (main_t)se_read_thread, NULL);
	printf("se_read_thread generated[%d]\n", tid);
	tid = task_create("flash_erase_thread", 100, 1024*50, (main_t)flash_erase_thread, NULL);
	printf("flash_erase_thread generated[%d]\n", tid);

	return 0;
}
