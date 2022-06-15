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

#include <tinyara/config.h>
#include <stdio.h>
#include <security/security_common.h>
#include <security/security_api.h>
#include <tinyara/seclink.h>
#include <tinyara/security_hal.h>
#include <security/security_ss.h>
#define SE_IOTIVITY_SVR_DB 0

static char a[8092];
static char b[8092];
/****************************************************************************
 * hello_main
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	printf("Hello, World!!\n");
	dbg("=====================%d==================================================\n",__LINE__);
	sleep(1);


	memset(a,'a', 4900);
	a[4900] = '\0';

	dbg("=====================%d==================================================\n",__LINE__);
	sleep(1);
	security_handle hnd;
	security_error res = security_init(&hnd);
	if (res != SECURITY_OK) {
		dbg("=====================%d==================================================\n",__LINE__);
		sleep(1);
	}
	dbg("=====================%d==================================================\n",__LINE__);
	sleep(1);
	security_data data = { (void *)a, 4901};
	char ss_path[7];
	snprintf(ss_path, 7, "ss/%d", SE_IOTIVITY_SVR_DB);
	dbg("=====================%d==================================================\n",__LINE__);
	sleep(1);
	res = ss_write_secure_storage(hnd, ss_path, 0, &data);
	if (res != SECURITY_OK) {
		dbg("=====================%d==================================================\n",__LINE__);
		sleep(1);
	}
	dbg("=====================%d==================================================\n",__LINE__);
	sleep(1);
	security_deinit(hnd);
	dbg("=====================%d==================================================\n",__LINE__);
	sleep(1);


	unsigned int read_size = 0;

	security_handle hnd1;
	security_error res1 = security_init(&hnd1);
	if (res1 != SECURITY_OK) {
		dbg("=====================%d==================================================\n",__LINE__);
		sleep(1);
	}
	dbg("=====================%d==================================================\n",__LINE__);
	sleep(1);
	security_data data1 = { NULL, 8092};
	char ss_path1[7] = { 0, };
	snprintf(ss_path1, 7, "ss/%d", SE_IOTIVITY_SVR_DB);
	dbg("=====================%d==================================================\n",__LINE__);
	sleep(1);
	res = ss_read_secure_storage(hnd1, ss_path1, 0, &data1);
	if (res != SECURITY_OK) {
		dbg("=====================%d==================================================\n",__LINE__);
		sleep(1);
	}

	memcpy(b, data1.data, data1.length);

	dbg("=====================%d===============%d===================================\n",__LINE__,data1.length);
	sleep(1);
	dbg("=====================%d===============%s===================================\n",__LINE__,data1.data);
	sleep(1);
	free(data1.data);
	dbg("=====================%d==================================================\n",__LINE__);
	sleep(1);

	(void)security_deinit(hnd1);
	dbg("=====================%d==================================================\n",__LINE__);
	sleep(1);

	return 0;
}
