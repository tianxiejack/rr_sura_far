/*
 * Cyclops_Message.c
 *
 *  Created on: Dec 8, 2017
 *      Author: xz
 */

/*Cyclops_Message.c*/
#if USE_CAP_SPI
#include <stdio.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <errno.h>
#include <stdlib.h> 	/* add this: exit返回,不会报提示信息 */
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>  /* ture false 有效*/
#include <string.h>
#include <pthread.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include "Cap_Spi_Message.h"
#include "spiH.h"

struct RS422_data RS422_MIRROR_buff = { 0 };
struct RS422_data RS422_VCODE_buff = { 0 };
struct RS422_data RS422_BAK1_buff = { 0 };

typedef struct {
	char serial_tty[64];
	int baud_rate;
} SerialSet;
SerialSet Serial[SERIALNUM];
char IPC_ftok_path[64];

void* Spi2Worker(void* args) {
	Spi2proces(&RS422_MIRROR_buff, &RS422_VCODE_buff, &RS422_BAK1_buff);
	return NULL ;
}

void ReadUartSet() {
	FILE *fp;
	int i;
	char buf[128];
	char temp[128];
	fp = fopen("serialportset", "r");
	if (fp == NULL) {
		fprintf(stderr, "open serialportset fail!!!\n");
		exit(-1);
	}
	for (i = 0; i <= sizeof(Serial) / sizeof(SerialSet); i++) {
		memset(buf, 0, sizeof(buf));
		if (NULL == fgets(buf, 32, fp)) {/*按行读取*/
			break;
		}
		if (sizeof(Serial) / sizeof(SerialSet) == i) {/*第三项为用户名*/
			sprintf(temp, "/home/%s", buf);
			strncpy(IPC_ftok_path, temp, strlen(temp) - 1);
			printf("%s\n", IPC_ftok_path);
			break;
		}
		sprintf(temp, "/dev/%s", buf);/*串口名*/
		strncpy(Serial[i].serial_tty, temp, strlen(temp) - 1);
		memset(buf, 0, sizeof(buf));
		if (NULL == fgets(buf, 32, fp)) {
			break;
		}
		Serial[i].baud_rate = atoi(buf);/*波特率*/
		printf("%s\n", Serial[i].serial_tty);
		printf("%d\n", Serial[i].baud_rate);
	}
}

void UartSet() {
	int err, ret;
	pthread_t t;

	RS422_MIRROR_buff.receiveData = (uint8_t*) malloc(1024);
	RS422_MIRROR_buff.length = 0;
	pthread_mutex_init(&RS422_MIRROR_buff.mutex, NULL);
	if (ret = open_device((char *) Serial[0].serial_tty)) {   //open spi
		printf("spi0.0 open failed\n");
	}
	transfer_init(3, 1, Serial[0].baud_rate);
	pthread_create(&t, NULL, Spi2Worker, NULL);
}

/*消息队列模块：包括初始化和开接收线程*/
void InitIPCModule() {
	ReadUartSet();
	UartSet();
}

void WriteMessage(MSG_TYPE n, char buf) {

	unsigned char data[1024];
	int CameraNumber;
	memset(data, 0, sizeof(data));
	switch (n) {
	case MSG_TYPE_YUAN_JIASHI:
		data[0] = 0xaa;
		data[1] = 0x55;
		data[3] = 0x5a;
		data[4] = 0xa5;
		if (buf <= '9' & buf >= '0') {
			CameraNumber = atoi(&buf) + 1;
			data[2] = CameraNumber & 0xf;
			sendDataToSpi(3, data, 5);
		} else {
			printf("远景相机请输入0-9的相机编号！\n");
			break;
		}
		break;
	case MSG_TYPE_YUAN_CHEZHANG:
		data[0] = 0xaa;
		data[1] = 0x55;
		data[3] = 0x5a;
		data[4] = 0xa5;
		if (buf <= '9' & buf >= '0') {
			CameraNumber = atoi(&buf) + 1;
			data[2] = (CameraNumber & 0xf) << 4;
			sendDataToSpi(3, data, 5);
		} else {
			printf("远景相机请输入0-9的相机编号！\n");
			break;
		}
		break;
	case MSG_TYPE_JIN_JIASHI:
		data[0] = 0xaa;
		data[1] = 0x55;
		data[4] = 0x5a;
		data[5] = 0xa5;
		if (buf <= '9' & buf >= '0') {
			CameraNumber = atoi(&buf) + 1;
			data[2] = CameraNumber & 0xf;
			data[3] = 0x0;
			sendDataToSpi(3, data, 6);
		} else if (buf <= 'd' & buf >= 'a') {
			CameraNumber = buf - 'a' + 1;
			data[2] = 0x0;
			data[3] = CameraNumber;
			sendDataToSpi(3, data, 6);
		} else {
			printf("近景相机请输入0-9或者a-d的相机编号！\n");
			break;
		}
		break;
	case MSG_TYPE_JIN_CHEZHANG:
		data[0] = 0xaa;
		data[1] = 0x55;
		data[4] = 0x5a;
		data[5] = 0xa5;
		if (buf <= '9' & buf >= '0') {
			CameraNumber = atoi(&buf) + 1;
			data[2] = (CameraNumber & 0xf) << 4;
			data[3] = 0x0;
			sendDataToSpi(3, data, 6);
		} else if (buf <= 'd' & buf >= 'a') {
			CameraNumber = buf - 'a' + 1;
			data[2] = 0x0;
			data[3] = CameraNumber;
			sendDataToSpi(3, data, 6);
		} else {
			printf("近景相机请输入0-9或者a-d的相机编号！\n");
			break;
		}
		break;
	default:
		printf("不存在的选项！\n");
	}
}
#endif
