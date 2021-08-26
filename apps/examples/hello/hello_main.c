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
#include <string.h>
#include <sched.h>
#include <mqueue.h>
#include <errno.h>
#include <stdint.h>
#include <fcntl.h>

static int g_state = 0;
static int g_mq = 0;

static int _process_msg1(int argc, char *argv[])
{
    uint32_t count = 0;
    while(g_state) {
        if (count >= __UINT32_MAX__) count = 0;
        if (count % 10000000 == 0) {
            printf("process1 alive\n");
            fflush(stdout);
        }
        count++;
    }
    return 0;
}

static int _process_msg2(int argc, char *argv[])
{
    uint32_t count = 0;
    while(g_state) {
        if (count >= __UINT32_MAX__) count = 0;
        if (count % 10003000 == 0) {
            printf("process2 alive\n");
            fflush(stdout);
        }
        count++;
    }
    return 0;
}

static int _process_msg3(int argc, char *argv[])
{
    uint32_t count = 0;
    while(g_state) {
        if (count >= __UINT32_MAX__) count = 0;
        if (count % 10010000 == 0) {
            printf("process3 alive\n");
            fflush(stdout);
        }
        count++;
    }
    return 0;
}

#define TEST_MQ_SIZE 1
#define TEST_MQ_NAME "mq_test"

static int _process_mq_send(int argc, char *argv[])
{
    printf("Start MQ Send\n");
    
    struct mq_attr attr;
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = TEST_MQ_SIZE;
    attr.mq_flags = 0;
    int status;
    char buf[TEST_MQ_SIZE] = { 0, };
    int cnt = 0;

    mqd_t mqfd = mq_open(TEST_MQ_NAME, O_WRONLY | O_CREAT, 0666, &attr);
    while(g_mq) {
        buf[0] = (char)cnt;
        status = mq_send(mqfd, buf, TEST_MQ_SIZE, 0);
        if (status < 0) {
            printf("fail to send mqueue[%d]\n", errno);
            return 0;
        }
        printf("--> MQ SEND : %d\n", buf[0]);
        cnt++;
        if (cnt > 100) cnt = 0;
        sleep(1);
    }
    mq_close(mqfd);
    return 0;   
}

static int _process_mq_receive(int argc, char *argv[])
{
    printf("Start MQ Receive\n");

    struct mq_attr attr;
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = TEST_MQ_SIZE;
    attr.mq_flags = 0;
    int nbytes;
    char buf[TEST_MQ_SIZE] = { 0, };

    mqd_t mqfd = mq_open(TEST_MQ_NAME, O_RDONLY | O_CREAT, 0666, &attr);
    while(g_mq) {
        nbytes = mq_receive(mqfd, buf, sizeof(buf), 0);
        if (nbytes <= 0) {
            printf("fail to receive mqueue[%d]\n", errno);
        } else {
            printf("<-- MQ RECV : %d\n", buf[0]);
        }
    }
    mq_close(mqfd);
    return 0;   
}

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

    if (argc <= 1) return 0;

    if (strncmp(argv[1], "stress", 7) == 0 && argv[2][0] == '0') {
        g_state = 0;
    } else if (strncmp(argv[1], "stress", 7) == 0 && argv[2][0] == '1') {

        if (g_state == 1) return 0;

        g_state = 1;

        int tid;
        tid = task_create("test_stress_handle1", 100, 1024, (main_t)_process_msg1, NULL);
        printf("process1 generated[%d]\n", tid);
        tid = task_create("test_stress_handle2", 100, 1024, (main_t)_process_msg2, NULL);
        printf("process2 generated[%d]\n", tid);
        tid = task_create("test_stress_handle3", 100, 1024, (main_t)_process_msg3, NULL);
        printf("process3 generated[%d]\n", tid);

    } else if (strncmp(argv[1], "mq", 3) == 0 && argv[2][0] == '0') {
        g_mq = 0;
    } else if (strncmp(argv[1], "mq", 3) == 0 && argv[2][0] == '1') {
        if (g_mq == 1) return 0;

        g_mq = 1;

        int tid;
        tid = task_create("test_mq_handle1", 100, 1024, (main_t)_process_mq_send, NULL);
        printf("process1 generated[%d]\n", tid);
        
        tid = task_create("test_mq_handle2", 99, 1024, (main_t)_process_mq_receive, NULL);
        printf("process2 generated[%d]\n", tid);        
    }

    return 0;
}
