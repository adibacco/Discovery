/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "lwip/api.h"
#include "lwip/sys.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */
osThreadId udpTaskHandle;
osThreadId tcpTaskHandle;

static struct netconn *udp_conn;
static struct netbuf *udp_buf;
static struct ip_addr *udp_addr;
static unsigned short udp_port;

static struct netconn *tcp_conn;
static struct netbuf *tcp_buf;
static struct ip_addr *tcp_addr;
static unsigned short tcp_port;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void StartUdpTask(void const * argument);
void StartTcpTask(void const * argument);

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	osThreadDef(udpTask, StartUdpTask, osPriorityNormal, 0, 512);
	udpTaskHandle = osThreadCreate(osThread(udpTask), NULL);

	osThreadDef(tcpTask, StartTcpTask, osPriorityNormal, 0, 512);
	tcpTaskHandle = osThreadCreate(osThread(tcpTask), NULL);

	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument) {

	/* init code for LWIP */
	MX_LWIP_Init();

	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */

	for (;;) {
		osDelay(1);
	}

	/* USER CODE END StartDefaultTask */
}

/* StartUdpTask function */
void StartUdpTask(void const * argument) {

	osDelay(1000);

	err_t err, recv_err;

	udp_conn = netconn_new(NETCONN_UDP);
	if (udp_conn != NULL) {
		err = netconn_bind(udp_conn, IP_ADDR_ANY, 7055);
		if (err == ERR_OK) {
			while (1) {
				recv_err = netconn_recv(udp_conn, &udp_buf);

				if (recv_err == ERR_OK) {
					udp_addr = netbuf_fromaddr(udp_buf);
					udp_port = netbuf_fromport(udp_buf);
					netconn_connect(udp_conn, udp_addr, udp_port);
					udp_buf->addr.addr = 0;
					netconn_send(udp_conn, udp_buf);
					netbuf_delete(udp_buf);
				} else {
					printf("recv error\n");
				}
			}
		} else {
			netconn_delete(udp_conn);
			printf("can not bind netconn");
		}
	} else {
		printf("can create new UDP netconn");
	}
}

/* StartTcpTask function */
void StartTcpTask(void const * argument) {
	struct netconn *newconn;

	err_t err, accept_err, recv_err;
	void *data;
	u16_t len;

	osDelay(1000);

	/* Create a new connection identifier. */
	tcp_conn = netconn_new(NETCONN_TCP);

	if (tcp_conn != NULL) {
		/* Bind connection to well known port number 7066. */
		err = netconn_bind(tcp_conn, NULL, 7066);

		if (err == ERR_OK) {
			/* Tell connection to go into listening mode. */
			netconn_listen(tcp_conn);

			while (1) {
				/* Grab new connection. */
				accept_err = netconn_accept(tcp_conn, &newconn);

				/* Process the new connection. */
				if (accept_err == ERR_OK) {
					recv_err = netconn_recv(newconn, &tcp_buf);
					while (recv_err == ERR_OK) {
						do {
							netbuf_data(tcp_buf, &data, &len);
							netconn_write(newconn, data, len, NETCONN_COPY);

						} while (netbuf_next(tcp_buf) >= 0);

						netbuf_delete(tcp_buf);
						recv_err = netconn_recv(newconn, &tcp_buf);
					}

					/* Close connection and discard connection identifier. */
					netconn_close(newconn);
					netconn_delete(newconn);
				}
			}
		} else {
			netconn_delete(newconn);
			printf(" can not bind TCP netconn");
		}
	} else {
		printf("can not create TCP netconn");
	}
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
