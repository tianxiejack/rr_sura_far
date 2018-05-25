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
#include "Zodiac_GPIO_Message.h"

#define Z2G_FTOK_PATH "/home/ubuntu"

int  z2g_ret_value;
int  z2g_ipc_port = 145;
key_t z2g_ipc_key;
int  z2g_ipc_qid;

void set_gpioNum_Value(int gpio_num,int gpio_value)
{
	SET_Gpio set_gpio;
	memset(&set_gpio,-1,sizeof(SET_Gpio));
	set_gpio.msgtype=NUM_VALUE;
	set_gpio.Data.gpioNum=gpio_num;
	set_gpio.Data.value=gpio_value;
	SendMessage(&set_gpio);
}


void delete_GPIO_IPCMessage()
{
	if(msgctl(z2g_ipc_qid,IPC_RMID,0)==-1)
	{
		printf("msgctl (IPC_RMID) failed\n");
		exit(-1);
	}
}


/*
void set_gpioDirection(int gpio_direction)
{
	SET_GPIO set_gpio;
	memset(&set_gpio,-1,sizeof(SET_GPIO));
	set_gpio.msgtype=DIRECTION;
	set_gpio.direction=gpio_direction;
	SendMessage(&set_gpio);
}
void set_gpioValue(int gpio_value)
{
	SET_GPIO set_gpio;
	memset(&set_gpio,-1,sizeof(SET_GPIO));
	set_gpio.msgtype=DIRECTION;
	set_gpio.value=VALUE;
	SendMessage(&set_gpio);
}
*/
void init_GPIO_IPCMessage()
{
	 z2g_ipc_key = ftok(Z2G_FTOK_PATH,  z2g_ipc_port);
		if ( z2g_ipc_key == -1) {
			printf(" port %d ftok key failed erro:  \n", z2g_ipc_port);
			perror(Z2G_FTOK_PATH);
			exit(-1);
		}
		 z2g_ipc_qid = msgget( z2g_ipc_key, IPC_EXCL); /*检查消息队列是否存在*/
		if ( z2g_ipc_qid < 0) {
			 z2g_ipc_qid = msgget( z2g_ipc_key, IPC_CREAT | 0666);/*创建消息队列*/
			if ( z2g_ipc_qid < 0) {
				printf("failed to create msq | errno=%d \n", errno);
				perror(" ");
				exit(-1);
			}
		} else {
			printf("port %d already exist! \n",  z2g_ipc_port);
		}
}
void SendMessage(Set_Gpio* value)
{
	if(msgsnd(z2g_ipc_qid,value,sizeof(value->Data),0)==-1)
	{
		perror("msgsnd error\n");
		exit(1);
	}
}
