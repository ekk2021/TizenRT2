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
#include <crc32.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>

#include <tinyara/board.h>
#include <tinyara/fs/mtd.h>

struct header_s {
	uint32_t crc_hash;
	uint16_t header_size;
	uint32_t version;
	uint32_t binary_size;
	uint16_t secure_header_size;
} __attribute__((__packed__));
typedef struct header_s header_t;

#define BUF_SIZE 1024
#define OTN_PARTITION_ADDRESS_0		0x82000
#define OTN_PARTITION_ADDRESS_1		0x2C2000

#if 0
int copy_from_otn1(void)
{
	int ret;
	int read_size;
	int total_size;
	int copy_size;
	header_t header;
	off_t offset;
	unsigned long read_addr, write_addr;
	FAR struct mtd_dev_s *dev_mtd = NULL;
	uint32_t crc_value = 0;
	uint8_t buffer[BUF_SIZE];

	dev_mtd = up_flashinitialize();
	if (!dev_mtd) {
		printf("up_flashinitialize Failed\n");
		return ERROR;
	}

	read_addr = OTN_PARTITION_ADDRESS_0;
	write_addr = OTN_PARTITION_ADDRESS_1;

	offset = 0;
	crc_value = 0;

	printf("Erase %x, start block %u block count %d\n", write_addr, write_addr / 4096, 576);

	/* Block count : 2304 * 1024  / 4096 = 576 */
	ret = MTD_ERASE(dev_mtd, write_addr / 4096, 576);
	if (ret < 0) {
		printf("Read header Failed : %d\n", ret);
		return ERROR;
	}

	ret = MTD_READ(dev_mtd, read_addr + offset, sizeof(header_t), &header);
	if (ret <= 0) {
		printf("Read header Failed : %d\n", ret);
		return ERROR;
	}

	offset += sizeof(header_t);

	total_size = header.binary_size;
	copy_size = 0;

	printf("HEADER %d %d %d %d\n", header.binary_size, header.header_size, header.secure_header_size, header.version);

	header.version++;
	crc_value = crc32part((uint8_t *)&header + 4, header.header_size, crc_value);

	while (total_size > copy_size) {
		read_size = ((total_size - copy_size) < BUF_SIZE ? (total_size - copy_size) : BUF_SIZE);

		ret = MTD_READ(dev_mtd, read_addr + offset, read_size, buffer);
		if (ret != read_size) {
			printf("Read data Failed : %d\n", ret);
			return ERROR;
		}

		ret = MTD_WRITE(dev_mtd, write_addr + offset, read_size, buffer);
		if (ret != read_size) {
			printf("Write data Failed : %d\n", ret);
			return ERROR;
		}

		crc_value = crc32part(buffer, read_size, crc_value);
		copy_size += read_size;
		offset += read_size;
		printf("Copy kernel [%d%%]\r", copy_size * 100 / total_size);
	}
	printf("\nCopy SUCCESS\n");

	//update header data
	header.crc_hash = crc_value;

	printf("new CRC : %u\n", crc_value);

	ret = MTD_WRITE(dev_mtd, write_addr, sizeof(header_t), &header);
	if (ret != sizeof(header_t)) {
		printf("Write data Failed : %d\n", ret);
		return ERROR;
	}

	return ERROR;
}
#endif

int powercut_test(void)
{
	int ret;
	int read_size;
	int total_size;
	int copy_size;
	off_t offset;
	unsigned long write_addr;
	FAR struct mtd_dev_s *dev_mtd = NULL;
	uint8_t buffer[BUF_SIZE];
	int fd;

	dev_mtd = up_flashinitialize();
	if (!dev_mtd) {
		printf("up_flashinitialize Failed\n");
		return ERROR;
	}

	fd = open("/mnt/km0_km4_image2_high.bin", O_RDWR);
	if (fd < 0) {
		printf("Open file Failed : %d\n", fd);
		return ERROR;
	}

	write_addr = OTN_PARTITION_ADDRESS_1;

	printf("Erase %x, start block %u block count %d\n", write_addr, write_addr / 4096, 576);

	/* Block count : 2304 * 1024  / 4096 = 576 */
	ret = MTD_ERASE(dev_mtd, write_addr / 4096, 576);
	if (ret < 0) {
		printf("Erase OTA2 Failed : %d\n", ret);
		return ERROR;
	}

	/* Get file size */
	total_size = lseek(fd, 0, SEEK_END);
	if (total_size <= 0) {
		printf("file size %d \n", total_size);
		close(fd);
		return ERROR;
	}

	copy_size = 0;
	offset = 0;

	printf("Copy size %d\n", total_size);

	ret = lseek(fd, 0, SEEK_SET);
	if (ret < 0) {
		printf("Failed to set read fd: %d\n", errno);
		close(fd);
		return ERROR;
	}

	while (total_size > copy_size) {
		read_size = ((total_size - copy_size) < BUF_SIZE ? (total_size - copy_size) : BUF_SIZE);

		ret = read(fd, buffer, read_size);
		if (ret != read_size) {
			printf("Read data Failed : %d, errno %d\n", ret, errno);
			return ERROR;
		}

		ret = MTD_WRITE(dev_mtd, write_addr + offset, read_size, buffer);
		if (ret != read_size) {
			printf("Write data Failed : %d\n", ret);
			return ERROR;
		}

		copy_size += read_size;
		offset += read_size;
		printf("Copy kernel [%d%%]\r", copy_size * 100 / total_size);
	}
	printf("\nCopy SUCCESS\n");

	return ERROR;
}

static void reboot(void)
{
	printf("*REBOOT!*\n");
	printf("*REBOOT!*\n");
#ifdef CONFIG_BOARDCTL_RESET
	board_reset(0);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	printf("Hello World\n");
	if (argc == 2) {
		if (!strncmp(argv[1], "update", strlen("update")+1)) {
			powercut_test();
		} else if (!strncmp(argv[1], "reboot", strlen("reboot")+1)) {
			reboot();
		}
	}
	return 0;
}
