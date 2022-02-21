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

#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>

#include <tinyara/fs/mtd.h>

#include <sys/socket.h>
#include <wifi_manager/wifi_manager.h>
#include <stress_tool/st_perf.h>

#define __DEBUG__
// #include "spy_debug.h"
/****************************************************************************
 * hello_main
 ****************************************************************************/
#define T_MAX_DATA_SIZE  2048
//#define USE_TERMINAL
//#define USE_DOCKLIGHT

#define TASK_STACKSIZE          1024*50

#define UART_TEST_PRIORITY     100
#define UART_TEST_STACK_SIZE   51200
#define UART_TEST_SCHED_POLICY SCHED_RR

static int wifi_test_task(void);
static int flash_task();
static int uart_task();
static int uart_test_rx( void *param );
static int uart_test_tx( void *param );

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
    printf("Hello, World!!\n");

    // temporary return
    return 0;

    int ret;

    ret = task_create("uart_task", 100, UART_TEST_STACK_SIZE, uart_task, NULL); 
    if(ret < 0) {
        printf("uart_task creation failed\n");
    }

    ret = task_create("flash_task", 100, TASK_STACKSIZE, flash_task, NULL);
    if(ret < 0) {
        printf("flash_task creation failed\n");    
    }

    ret = task_create("wifi_test_task", 100, TASK_STACKSIZE, wifi_test_task, NULL);
    if(ret < 0) {
        printf("wifi_test_task creation failed\n");    
    }    
}


#define TAG "[HELLO_SERVER]"

#define CT_LOG(tag, fmt, args...) \
printf(tag "[T%d] " fmt "\t%s:%d\n", getpid(), ##args, __FUNCTION__, __LINE__)

#define CT_LOGE(tag, fmt, args...) \
printf(tag "[ERR][T%d] " fmt "\t%s:%d\n", getpid(), ##args, __FUNCTION__, __LINE__)

static sem_t g_wm_sem;

#define HELLO_TEST_SIGNAL              \
    do {                            \
        sem_post(&g_wm_sem);        \
        CT_LOG(TAG, "send signal"); \
    } while (0)

#define HELLO_TEST_WAIT                \
    do {                            \
        CT_LOG(TAG, "wait signal"); \
        sem_wait(&g_wm_sem);        \
    } while (0)

/*
 * callbacks
 */
static void wm_cb_sta_connected(wifi_manager_cb_msg_s msg, void *arg);
static void wm_cb_sta_disconnected(wifi_manager_cb_msg_s msg, void *arg);
static void wm_cb_softap_sta_join(wifi_manager_cb_msg_s msg, void *arg);
static void wm_cb_softap_sta_leave(wifi_manager_cb_msg_s msg, void *arg);
static void wm_cb_scan_done(wifi_manager_cb_msg_s msg, void *arg);

static wifi_manager_cb_s g_wifi_callbacks = {
    wm_cb_sta_connected,
    wm_cb_sta_disconnected,
    wm_cb_softap_sta_join,
    wm_cb_softap_sta_leave,
    wm_cb_scan_done,
};


static int wifi_test_task(void)
{
    
    int res = sem_init(&g_wm_sem, 0, 0);
    if (res != 0) {
        return -1;
    }
    
    wifi_manager_result_e wres = WIFI_MANAGER_SUCCESS;

    /* Initialise Wifi */
    CT_LOG(TAG, "init wi-fi");
    wres = wifi_manager_init(&g_wifi_callbacks);
    if (wres != WIFI_MANAGER_SUCCESS) {
        CT_LOGE(TAG, "fail to init %d", wres);
        return -1;
    }
    
    while(1) {
        /*  wait join event */
        //CONTROL_VDRIVER(VWIFI_CMD_GEN_EVT, LWNL_EVT_SOFTAP_STA_JOINED, 0, 3000);
        //CONTROL_VDRIVER(VWIFI_CMD_GEN_EVT, VWIFI_PKT_DHCPS_EVT, 0, 3000);

        /*  scan in STA mode */
        CT_LOG(TAG, "scan in STA mode");
        wres = wifi_manager_scan_ap(NULL);
        if (wres != WIFI_MANAGER_SUCCESS) {
            CT_LOGE(TAG, "fail to scan %d", wres);
            return -1;
        }


        /*  wait scan event */
        CT_LOG(TAG, "wait scan done event in STA mode");
        HELLO_TEST_WAIT; 


        CT_LOG(TAG, "connect to AP");

        wifi_manager_ap_config_s apconfig;
        strncpy(apconfig.ssid, "RTK_AP_TEST", strlen("RTK_AP_TEST") + 1);
        apconfig.ssid_length = strlen("RTK_AP_TEST");
        strncpy(apconfig.passphrase, "1234567890", strlen("1234567890") + 1);   
        apconfig.passphrase_length = strlen("1234567890");
        apconfig.ap_auth_type = WIFI_MANAGER_AUTH_WPA2_PSK;
        apconfig.ap_crypto_type = WIFI_MANAGER_CRYPTO_AES;

        wres = wifi_manager_connect_ap(&apconfig);
        if (wres != WIFI_MANAGER_SUCCESS) {
            CT_LOG(TAG, "connect AP fail %d\n", wres);
            return -1;
        }
        
        /*  wait connect event */
        CT_LOG(TAG, "wait AP connection event in STA mode");
        HELLO_TEST_WAIT; 

        ///////////////////////////////////////////////////////////////
        break;    
        /////////////////////////////////////////////////////////////

        /*  disconnect to AP */
        wres = wifi_manager_disconnect_ap();
        if (wres != WIFI_MANAGER_SUCCESS) {
            CT_LOG(TAG, "disconnect AP fail %d\n", wres);
            return -1;
        }

        /*  wait disconnect event */
        CT_LOG(TAG, "wait AP disconnection event in STA mode");
        HELLO_TEST_WAIT;         
    }

    sem_destroy(&g_wm_sem);
    
    // TEST_0001 : if you run the iperf, it should be returned 
    // return 0;
}

#define FLASH_ADDRESS       0x402000
#define FLASH_BLOCK_SIZE    4096
// #define FLASH_BLOCK_SIZE    256
#define FLASH_SIZE    (1024 * 1024)
static int flash_task()
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
 
        printf("[%s] FLASH WRITE \n", __func__);
        while(address < FLASH_ADDRESS + FLASH_SIZE) {            
            ret = MTD_WRITE(mtd_dev, address, FLASH_BLOCK_SIZE, buffer);
            if(ret < 0) {
                printf("flash writr fail ret[0x%x] : %d\n", address, ret);                
            }


            address += FLASH_BLOCK_SIZE;
        }
        printf("flash block write (1M) finished \n");
        sleep(3);
    }

}

static int uart_task()
{
	
    struct termios tio;
    int fd = 0;
    int ret = -1;
    fd = open( "/dev/ttyS2", O_RDWR | O_NOCTTY );
    if( fd < 0 )
    {
        printf( "tty open failed...%d\n", fd );
        return 0;
    }
    ret = tcgetattr(fd, &tio);
    if (ret < 0) {
        printf( "tcgetattr failed...%d\n", fd );
        return 0;
    }
    tio.c_speed = B9600;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~PARODD;
    tio.c_cflag |= CSTOPB;
    ret = tcsetattr(fd, TCSANOW, &tio);
    if (ret < 0) {
        printf( "tcsetattr failed...%d\n", fd );
        return 0;
    }
    
    printf( "%s(%d) : start threads for uart(%d) test...\n", __func__, __LINE__, fd );

    pthread_t tid_tx;
    pthread_t tid_rx;
    pthread_attr_t attr;
    struct sched_param sparam;
    int r;

    /* Initialize the attribute variable */
    if ((r = pthread_attr_init(&attr)) != 0) {
        printf("%s: pthread_attr_init failed, status=%d\n", __func__, r);
    }

    /* 1. set a priority */
    sparam.sched_priority = UART_TEST_PRIORITY;
    if ((r = pthread_attr_setschedparam(&attr, &sparam)) != 0) {
        printf("%s: pthread_attr_setschedparam failed, status=%d\n", __func__, r);
    }

    if ((r = pthread_attr_setschedpolicy(&attr, UART_TEST_SCHED_POLICY)) != 0) {
        printf("%s: pthread_attr_setschedpolicy failed, status=%d\n", __func__, r);
    }

    /* 2. set a stacksize */
    if ((r = pthread_attr_setstacksize(&attr, UART_TEST_STACK_SIZE)) != 0) {
        printf("%s: pthread_attr_setstacksize failed, status=%d\n", __func__, r);
    }

    if ((r = pthread_create(&tid_rx, &attr, (pthread_startroutine_t)uart_test_rx, (void *)&fd)) != 0) {
        printf("%s: pthread_create(uart_test_rx) failed, status=%d\n", __func__, r);
    }

    if ((r = pthread_create(&tid_tx, &attr, (pthread_startroutine_t)uart_test_tx, (void *)&fd)) != 0) {
        printf("%s: pthread_create(uart_test_tx) failed, status=%d\n", __func__, r);
    }

    /* Wait for the threads to stop */
    pthread_join(tid_tx, NULL);
    pthread_join(tid_rx, NULL);
}

static int uart_test_rx( void *param )
{
    int fd = *(int*)param;

    unsigned char *pBuf = NULL;
    pBuf = (unsigned char *)malloc( T_MAX_DATA_SIZE + 1 );
    if( pBuf == NULL )
    {
        printf( "pBuf malloc failed...\n" );
        goto exit_error0;
    }

    usleep( 2000000 );
    printf( "%s(%d) : start...fd(%d)\n", __func__, __LINE__, fd );
    while( 1 )
    {
        memset( pBuf, 0, T_MAX_DATA_SIZE );
        int ret = read( fd, pBuf, T_MAX_DATA_SIZE );
        printf("input length = %d \n", ret);

        if( ret <= 0 )
        {
            break;
        }
    }

exit_error0:
    if( pBuf != NULL )
    {
        free( pBuf );
    }
	return 0;
}

/*
                                            41,42,43,44,45,46,47,48,49,50,  \
                                            51,52,53,54,55,56,57,58,59,60,  \
                                            61,62,63,64,65,66,67,68,69,70,  \
                                            71,72,73,74,75,76,77,78,79,80,  \
                                            81,82,83,84,85,86,87,88,89,90,  \
                                            91,92,93,94,95,96,97,98,99,100,  \
*/
static unsigned char test_packet[] = {0xAB, 0,2,3,4,5,6,7,8,9,10,    \
                                            11,12,13,14,15,16,17,18,19,20,  \
                                            21,22,23,24,25,26,27,28,29,30,  \
                                            31,32,33,34,35,36,37,38,39,40,  \
                                            0,0xBA};
static int uart_test_tx( void *param )
{
    int fd = *(int*)param;
    unsigned int count = 0;

    usleep( 2000000 );
    printf( "%s(%d) : start...fd(%d)\n", __func__, __LINE__, fd );

    while( 1 )
    {
        unsigned char checksum = 0;
        int len = sizeof(test_packet);
        test_packet[1] = count;
        for( int i = 1; i < len - 2; i ++ )
        {
            checksum += test_packet[i];
        }
        test_packet[len-2] = checksum;
        int ret = write( fd, test_packet, sizeof(test_packet) );
        if( ret <= 0 )
        {
            break;
        }
        usleep( 1*1000*1000);
        printf( "%s(%d) : %d...ret(%d)\n", __func__, __LINE__, count ++, ret );
    }

	return 0;
}


void wm_cb_sta_connected(wifi_manager_cb_msg_s msg, void *arg)
{
    CT_LOG(TAG, "--> res(%d)", msg.res);
    CT_LOG(TAG, "bssid %02x:%02x:%02x:%02x:%02x:%02x",
           msg.bssid[0], msg.bssid[1],
           msg.bssid[2], msg.bssid[3],
           msg.bssid[4], msg.bssid[5]);
    int conn = 0;
    if (WIFI_MANAGER_SUCCESS == msg.res) {
        conn = 0;
    } else {
        conn = 2;
    }
    HELLO_TEST_SIGNAL;
}

void wm_cb_sta_disconnected(wifi_manager_cb_msg_s msg, void *arg)
{
    CT_LOG(TAG, "--> res(%d) reason %d", msg.res, msg.reason);
    HELLO_TEST_SIGNAL;
}

void wm_cb_softap_sta_join(wifi_manager_cb_msg_s msg, void *arg)
{
    CT_LOG(TAG, "--> res(%d)", msg.res);
    CT_LOG(TAG, "bssid %02x:%02x:%02x:%02x:%02x:%02x",
           msg.bssid[0], msg.bssid[1],
           msg.bssid[2], msg.bssid[3],
           msg.bssid[4], msg.bssid[5]);
    HELLO_TEST_SIGNAL;
}

void wm_cb_softap_sta_leave(wifi_manager_cb_msg_s msg, void *arg)
{
    CT_LOG(TAG, "--> res(%d) reason %d", msg.res, msg.reason);
    HELLO_TEST_SIGNAL;
}

void wm_cb_scan_done(wifi_manager_cb_msg_s msg, void *arg)
{
    CT_LOG(TAG, "--> res(%d)", msg.res);
    if (msg.res != WIFI_MANAGER_SUCCESS || msg.scanlist == NULL) {
        HELLO_TEST_SIGNAL;
        return;
    }
    //ct_print_scanlist(msg.scanlist);
    HELLO_TEST_SIGNAL;
}