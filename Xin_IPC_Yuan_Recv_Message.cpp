/*
 * Xin_IPC_Message.c
 *
 *  Created on: 2018年4月23日
 *      Author: xz
 */
#include <stdio.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <errno.h>
#include <stdlib.h> 	/* add this: exit返回,不会报提示信息 */
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>  /* ture false 有效*/
#include <string.h>
#include <sys/time.h>
#include "Xin_IPC_Yuan_Recv_Message.h"

#define IPC_PORT_NUM 125
int ipc_port[IPC_NUM];
key_t ipc_key[IPC_NUM];
int ipc_qid[IPC_NUM];
struct timeval cap_lasttime, pas_lasttime, near_lasttime, cur_time;

typedef enum {
	IPC_MSG_TYPE_ORIGIN,

	IPC_MSG_TYPE_DRIVER_CLICK = 0x01,
	IPC_MSG_TYPE_ETHOR_CLICK,
	IPC_MSG_TYPE_DRIVER_KEY = 0x07,
	IPC_MSG_TYPE_TURN_STATE,
	IPC_MSG_TYPE_CAPTURE_STATE1,
	IPC_MSG_TYPE_CAPTURE_STATE2,
	IPC_MSG_TYPE_CAPTURE_STATE3,
	IPC_MSG_TYPE_PASSENGER_STATE,
	IPC_MSG_TYPE_NEARBOARD_STATE,
	IPC_MSG_TYPE_PERISCOPIC_ANGLE,
	IPC_MSG_TYPE_TURRET,
	IPC_MSG_TYPE_CAPTURE_CONTROL,
	IPC_MSG_TYPE_ETHOR_KEY,
	IPC_MSG_TYPE_TRIMMING,

	IPC_MSG_TYPE_RESERVE,
} IPC_MSG_TYPE;

typedef enum {
	KEY_TYPE_ORIGIN,

	KEY_TYPE_SINGLE_WINDOWS = 0x01,
	KEY_TYPE_TARGET_DETECTION,
	KEY_TYPE_IMAGE_ENHANCEMENT,
	KEY_TYPE_MOVEUP,
	KEY_TYPE_MOVEDOWN,
	KEY_TYPE_MOVELEFT,
	KEY_TYPE_MOVERIGHT,
	KEY_TYPE_DEBUGMODE,

	KEY_TYPE_F9,
	KEY_TYPE_F10,

	KEY_TYPE_RESERVE,
} KEY_TYPE;

typedef enum {
	DEBUG_ORDER_ORIGIN,

	DEBUG_ORDER_PERISCOPIC_MODE = 0x01,
	DEBUG_ORDER_CROSS_MOVEUP,
	DEBUG_ORDER_CROSS_MOVEDOWN,
	DEBUG_ORDER_CROSS_MOVELEFT,
	DEBUG_ORDER_CROSS_MOVERIGHT,
	DEBUG_ORDER_TARGETDETECTION_ON,
	DEBUG_ORDER_TARGETDETECTION_OFF,
	DEBUG_ORDER_TRIMMING_ON,
	DEBUG_ORDER_CHOOSECAMERA_LEFT,
	DEBUG_ORDER_CHOOSECAMERA_RIGHT,
	DEBUG_ORDER_CHECKEDCAMERA_MOVEUP,
	DEBUG_ORDER_CHECKEDCAMERA_MOVEDOWN,
	DEBUG_ORDER_CHECKEDCAMERA_MOVELEFT,
	DEBUG_ORDER_CHECKEDCAMERA_MOVERIGHT,
	DEBUG_ORDER_SAVE_TRIMMING_RESULT,
	DEBUG_ORDER_CLEAN_CHECKEDCAMERA_RESULT,
	DEBUG_ORDER_CLEAN_ALLCAMERA_RESULT,
	DEBUG_ORDER_TRIMMING_OFF,
	DEBUG_ORDER_SINGLECAMERA_MODE,
	DEBUG_ORDER_SINGLECAMERA_0,
	DEBUG_ORDER_SINGLECAMERA_1,
	DEBUG_ORDER_SINGLECAMERA_2,
	DEBUG_ORDER_SINGLECAMERA_3,
	DEBUG_ORDER_SINGLECAMERA_4,
	DEBUG_ORDER_SINGLECAMERA_5,
	DEBUG_ORDER_SINGLECAMERA_6,
	DEBUG_ORDER_SINGLECAMERA_7,
	DEBUG_ORDER_SINGLECAMERA_8,
	DEBUG_ORDER_SINGLECAMERA_9,

	DEBUG_ORDER_RESERVE,
} DEBUG_ORDER;

typedef struct {
	long msg_type;
	union Xx_far_payload {
		coor_p click_coord;
		ANGLE_GROUP angle_msg;
		unsigned int Stete;
		unsigned int Cap_State;
		KEY_TYPE key_value;
		DEBUG_ORDER debugorder;
		unsigned char Cap_timeover;
		unsigned char Passenger_timeover;
		unsigned char NearBoard_timeover;
	} payload;
} IPC_msg;
/*函数返回值*/
coor_p Ephor_CoorPoint[2] = { { -1, -1 }, { -1, -1 } };
Mode_Type Key_SwitchMode[2] = { Mode_Type_START, Mode_Type_START };
int Key_TargetDetectionState[2] = { -1, -1 };
int Key_ImageEnhancementState[2] = { -1, -1 };
MOVE_TYPE Key_MoveDirection[2] = { MOVETYPE_START, MOVETYPE_START };
ANGLE_GROUP AngleFar_PeriscopicLens = { 0, 0 };
ANGLE_GROUP AngleFar_GunAngle = { 0, 0 };
ANGLE_GROUP AngleFar_CanonAngle = { 0, 0 };
Cap_Msg CaptureMessage = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1 };
DEBUG_MODE Debug_SwitchMode[2] = { DEBUG_MODE_START, DEBUG_MODE_START };
MOVE_TYPE Debug_CrossMoveDirectiom[2] = { MOVETYPE_START, MOVETYPE_START };
int Debug_TargetDetectionState[2] = { -1, -1 };
DEBUG_STATE Debug_State[2] = { DEBUG_STATE_ORIGIN, DEBUG_STATE_ORIGIN };
DEBUG_CHOOSECAMERA Debug_ChooseCamera[2] = { DEBUG_CHOOSEVIDEO_ORIGIN,
		DEBUG_CHOOSEVIDEO_ORIGIN };
MOVE_TYPE Debug_CameraMoveDirection[2] = { MOVETYPE_START, MOVETYPE_START };

#define  IPC_ftok_path "/home/"
pthread_mutex_t Mutex[2] = PTHREAD_MUTEX_INITIALIZER;

void *Recv_ipc_Ephor(void *arg);
void *Recv_ipc_Driver(void *arg);

void IPC_Init(int num) {
	int ret;
	pthread_t thread;
	ipc_port[num] = IPC_PORT_NUM + num;
	ipc_key[num] = ftok(IPC_ftok_path, ipc_port[num]);
	if (ipc_key[num] == -1) {
		printf(" port %d ftok key failed erro:  \n", ipc_port[num]);
		perror(IPC_ftok_path);
		exit(-1);
	}

	ipc_qid[num] = msgget(ipc_key[num], IPC_EXCL); /*检查消息队列是否存在*/
	if (ipc_qid[num] < 0) {
		ipc_qid[num] = msgget(ipc_key[num], IPC_CREAT | 0666);/*创建消息队列*/
		if (ipc_qid[num] < 0) {
			printf("failed to create msq | errno=%d \n", errno);
			perror(" ");
			exit(-1);
		}
	} else {
		printf("port %d already exist! \n", ipc_port[num]);
	}
}

void IPC_Init_Ephor() {
	int ret;
	pthread_t thread1;
	IPC_Init(TRANSFER_TO_APP_ETHOR);
	ret = pthread_create(&thread1, NULL, Recv_ipc_Ephor, NULL);
	if (ret != 0) {
		printf("pthread1_create failed!\n");
		exit(ret);
	}
}

void IPC_Init_Driver() {
	int ret;
	pthread_t thread2;
	IPC_Init(TRANSFER_TO_APP_DRIVER);
	ret = pthread_create(&thread2, NULL, Recv_ipc_Driver, NULL);
	if (ret != 0) {
		printf("pthread2_create failed!\n");
		exit(ret);
	}
}

void *Over_Time(void *arg) {
	double cap_time, pas_time, near_time;
	sleep(1);
	gettimeofday(&cur_time, NULL);
	cap_time = (cur_time.tv_sec - cap_lasttime.tv_sec)
			+ (cur_time.tv_usec - cap_lasttime.tv_usec) / 1000000.0;
	pas_time = (cur_time.tv_sec - pas_lasttime.tv_sec)
			+ (cur_time.tv_usec - pas_lasttime.tv_usec) / 1000000.0;
	near_time = (cur_time.tv_sec - near_lasttime.tv_sec)
			+ (cur_time.tv_usec - near_lasttime.tv_usec) / 1000000.0;
	if (cap_time > 60.0) {
		CaptureMessage.Cap_FAULT_Colour = POINT_RED;
		CaptureMessage.Cap_BoxState = POINT_RED;
	} else {
		CaptureMessage.Cap_FAULT_Colour = POINT_GREEN;
	}
	if (pas_time > 60.0) {
		CaptureMessage.passenger_FAULT_Colour = POINT_RED;
		CaptureMessage.passengerState = POINT_RED;
	} else {
		CaptureMessage.passenger_FAULT_Colour = POINT_GREEN;
	}

	if (near_time > 60.0) {
		CaptureMessage.nearBoard_FAULT_Colour = POINT_RED;
		CaptureMessage.nearBoardState = POINT_RED;
	} else {
		CaptureMessage.nearBoard_FAULT_Colour = POINT_GREEN;
	}
}

void IPC_Init_All() {
	int ret;
	pthread_t thread3;
	IPC_Init_Ephor();
	IPC_Init_Driver();
	ret = pthread_create(&thread3, NULL, Over_Time, NULL);
	if (ret != 0) {
		printf("pthread3_create failed!\n");
		exit(ret);
	}
}

/*接收线程*/
void *Recv_ipc_Ephor(void *arg) {
	int ret_value;
	IPC_msg msg;
	printf("\n%x\n", ipc_qid[TRANSFER_TO_APP_ETHOR]);
	while (1) {
		memset(&msg, 0, sizeof(IPC_msg));
		ret_value = msgrcv(ipc_qid[TRANSFER_TO_APP_ETHOR], &msg,
				sizeof(msg.payload), 0, 0);
		if (ret_value < 0) {
			printf("%s Receive  IPC msg failed,errno=%d !!!\n", __FUNCTION__,
			errno);
		}
		switch (msg.msg_type) {
		case IPC_MSG_TYPE_DRIVER_CLICK:
			break;
		case IPC_MSG_TYPE_ETHOR_CLICK:
			pthread_mutex_lock(&Mutex[0]);
			Ephor_CoorPoint[0].point_x = msg.payload.click_coord.point_x;
			Ephor_CoorPoint[0].point_y = msg.payload.click_coord.point_y;
			pthread_mutex_unlock(&Mutex[0]);
			break;
		case IPC_MSG_TYPE_DRIVER_KEY:
			break;
		case IPC_MSG_TYPE_TURN_STATE:
			break;
		case IPC_MSG_TYPE_CAPTURE_STATE1:
			pthread_mutex_lock(&Mutex[0]);
			CaptureMessage.cameraFrontState = (msg.payload.Stete >> 24) & 0xff;
			CaptureMessage.cameraLeft1State = (msg.payload.Stete >> 16) & 0xff;
			CaptureMessage.cameraRight1State = (msg.payload.Stete >> 8) & 0xff;
			CaptureMessage.cameraLeft2State = msg.payload.Stete & 0xff;
			gettimeofday(&cap_lasttime, NULL);
			pthread_mutex_unlock(&Mutex[0]);
			break;
		case IPC_MSG_TYPE_CAPTURE_STATE2:
			pthread_mutex_lock(&Mutex[0]);
			CaptureMessage.cameraRight2State = (msg.payload.Stete >> 24) & 0xff;
			CaptureMessage.cameraLeft3State = (msg.payload.Stete >> 16) & 0xff;
			CaptureMessage.cameraRight3State = (msg.payload.Stete >> 8) & 0xff;
			CaptureMessage.cameraBackState = msg.payload.Stete & 0xff;
			gettimeofday(&cap_lasttime, NULL);
			pthread_mutex_unlock(&Mutex[0]);
			break;
		case IPC_MSG_TYPE_CAPTURE_STATE3:
			pthread_mutex_lock(&Mutex[0]);
			CaptureMessage.Cap_BoxState = (msg.payload.Cap_State >> 16) & 0xff;
			CaptureMessage.cameraFrontTest = !(msg.payload.Cap_State >> 8)
					& 0x1;
			CaptureMessage.cameraLeft1Test = !(msg.payload.Cap_State >> 8)
					& 0x2;
			CaptureMessage.cameraRight1Test = !(msg.payload.Cap_State >> 8)
					& 0x4;
			CaptureMessage.cameraLeft2Test = !(msg.payload.Cap_State >> 8)
					& 0x8;
			CaptureMessage.cameraRight2Test = !(msg.payload.Cap_State >> 8)
					& 0x10;
			CaptureMessage.cameraLeft3Test = !(msg.payload.Cap_State >> 8)
					& 0x20;
			CaptureMessage.cameraRight3Test = !(msg.payload.Cap_State >> 8)
					& 0x40;
			CaptureMessage.cameraBackTest = !(msg.payload.Cap_State >> 8)
					& 0x80;
			CaptureMessage.Cap_BoxTest = msg.payload.Cap_State & 0xff;
			gettimeofday(&cap_lasttime, NULL);
			pthread_mutex_unlock(&Mutex[0]);
			break;
		case IPC_MSG_TYPE_PASSENGER_STATE:
			pthread_mutex_lock(&Mutex[0]);
			CaptureMessage.passengerTest = (msg.payload.Cap_State >> 8) & 0xff;
			CaptureMessage.passengerState = msg.payload.Cap_State & 0xff;
			gettimeofday(&pas_lasttime, NULL);
			pthread_mutex_unlock(&Mutex[0]);
			break;
		case IPC_MSG_TYPE_NEARBOARD_STATE:
			pthread_mutex_lock(&Mutex[0]);
			CaptureMessage.nearBoardTest = (msg.payload.Cap_State >> 8) & 0xff;
			CaptureMessage.nearBoardState = msg.payload.Cap_State & 0xff;
			gettimeofday(&near_lasttime, NULL);
			pthread_mutex_unlock(&Mutex[0]);
			break;
		case IPC_MSG_TYPE_PERISCOPIC_ANGLE:
			pthread_mutex_lock(&Mutex[0]);
			AngleFar_PeriscopicLens.hor_angle = msg.payload.angle_msg.hor_angle;
			AngleFar_PeriscopicLens.ver_angle = msg.payload.angle_msg.ver_angle;
			pthread_mutex_unlock(&Mutex[0]);
			break;
		case IPC_MSG_TYPE_TURRET:
			if (msg.payload.angle_msg.type == 0) {
				pthread_mutex_lock(&Mutex[0]);
				AngleFar_GunAngle.hor_angle = msg.payload.angle_msg.hor_angle;
				AngleFar_GunAngle.ver_angle = msg.payload.angle_msg.ver_angle;
				pthread_mutex_unlock(&Mutex[0]);
			} else if (msg.payload.angle_msg.type == 1) {
				pthread_mutex_lock(&Mutex[0]);
				AngleFar_CanonAngle.hor_angle = msg.payload.angle_msg.hor_angle;
				AngleFar_CanonAngle.ver_angle = msg.payload.angle_msg.ver_angle;
				pthread_mutex_unlock(&Mutex[0]);
			}
			break;
		case IPC_MSG_TYPE_CAPTURE_CONTROL:
			break;
		case IPC_MSG_TYPE_ETHOR_KEY:
			pthread_mutex_lock(&Mutex[0]);
			switch (msg.payload.key_value) {
			case KEY_TYPE_SINGLE_WINDOWS:
				Key_SwitchMode[0] = Mode_Type_SINGLE_POPUP_WINDOWS;
				break;
			case KEY_TYPE_DEBUGMODE:
				Key_SwitchMode[0] = Mode_Type_DEBUG;
				break;
			case KEY_TYPE_TARGET_DETECTION:
				Key_TargetDetectionState[0] = 1;
				break;
			case KEY_TYPE_IMAGE_ENHANCEMENT:
				Key_ImageEnhancementState[0] = 1;
				break;
			case KEY_TYPE_MOVEUP:
				Key_MoveDirection[0] = MOVE_TYPE_MOVEUP;
				break;
			case KEY_TYPE_MOVEDOWN:
				Key_MoveDirection[0] = MOVE_TYPE_MOVEDOWN;
				break;
			case KEY_TYPE_MOVELEFT:
				Key_MoveDirection[0] = MOVE_TYPE_MOVELEFT;
				break;
			case KEY_TYPE_MOVERIGHT:
				Key_MoveDirection[0] = MOVE_TYPE_MOVERIGHT;
				break;
			default:
				printf("No KEY_TYPE!\n");
			}
			pthread_mutex_unlock(&Mutex[0]);
			break;
		case IPC_MSG_TYPE_TRIMMING:
			pthread_mutex_lock(&Mutex[0]);
			switch (msg.payload.key_value) {
			case DEBUG_ORDER_PERISCOPIC_MODE:
				Debug_SwitchMode[0] = DEBUG_MODE_PERISCOPIC;
				break;
			case DEBUG_ORDER_CROSS_MOVEUP:
				Debug_CrossMoveDirectiom[0] = MOVE_TYPE_MOVEUP;
				break;
			case DEBUG_ORDER_CROSS_MOVEDOWN:
				Debug_CrossMoveDirectiom[0] = MOVE_TYPE_MOVEDOWN;
				break;
			case DEBUG_ORDER_CROSS_MOVELEFT:
				Debug_CrossMoveDirectiom[0] = MOVE_TYPE_MOVELEFT;
				break;
			case DEBUG_ORDER_CROSS_MOVERIGHT:
				Debug_CrossMoveDirectiom[0] = MOVE_TYPE_MOVERIGHT;
				break;
			case DEBUG_ORDER_TARGETDETECTION_ON:
				Debug_TargetDetectionState[0] = 1;
				break;
			case DEBUG_ORDER_TARGETDETECTION_OFF:
				Debug_TargetDetectionState[0] = 0;
				break;
			case DEBUG_ORDER_TRIMMING_ON:
				Debug_State[0] = DEBUG_STATE_ON;
				break;
			case DEBUG_ORDER_CHOOSECAMERA_LEFT:
				Debug_ChooseCamera[0] = DEBUG_CHOOSEVIDEO_LEFT;
				break;
			case DEBUG_ORDER_CHOOSECAMERA_RIGHT:
				Debug_ChooseCamera[0] = DEBUG_CHOOSEVIDEO_RIGHT;
				break;
			case DEBUG_ORDER_CHECKEDCAMERA_MOVEUP:
				Debug_CameraMoveDirection[0] = MOVE_TYPE_MOVEUP;
				break;
			case DEBUG_ORDER_CHECKEDCAMERA_MOVEDOWN:
				Debug_CameraMoveDirection[0] = MOVE_TYPE_MOVEDOWN;
				break;
			case DEBUG_ORDER_CHECKEDCAMERA_MOVELEFT:
				Debug_CameraMoveDirection[0] = MOVE_TYPE_MOVELEFT;
				break;
			case DEBUG_ORDER_CHECKEDCAMERA_MOVERIGHT:
				Debug_CameraMoveDirection[0] = MOVE_TYPE_MOVERIGHT;
				break;
			case DEBUG_ORDER_SAVE_TRIMMING_RESULT:
				Debug_State[0] = DEBUG_STATE_SAVE;
				break;
			case DEBUG_ORDER_CLEAN_CHECKEDCAMERA_RESULT:
				Debug_State[0] = DEBUG_STATE_CLEAN_CHECKEDCAMERA;
				break;
			case DEBUG_ORDER_CLEAN_ALLCAMERA_RESULT:
				Debug_State[0] = DEBUG_STATE_CLEAN_ALLCAMERA;
				break;
			case DEBUG_ORDER_TRIMMING_OFF:
				Debug_State[0] = DEBUG_STATE_OFF;
				break;
			case DEBUG_ORDER_SINGLECAMERA_MODE:
				Debug_SwitchMode[0] = DEBUG_MODE_SINGLE_VIDEO;
				break;
			case DEBUG_ORDER_SINGLECAMERA_0:
				Debug_SwitchMode[0] = DEBUG_MODE_SINGLE_VIDEO0;
				break;
			case DEBUG_ORDER_SINGLECAMERA_1:
				Debug_SwitchMode[0] = DEBUG_MODE_SINGLE_VIDEO1;
				break;
			case DEBUG_ORDER_SINGLECAMERA_2:
				Debug_SwitchMode[0] = DEBUG_MODE_SINGLE_VIDEO2;
				break;
			case DEBUG_ORDER_SINGLECAMERA_3:
				Debug_SwitchMode[0] = DEBUG_MODE_SINGLE_VIDEO3;
				break;
			case DEBUG_ORDER_SINGLECAMERA_4:
				Debug_SwitchMode[0] = DEBUG_MODE_SINGLE_VIDEO4;
				break;
			case DEBUG_ORDER_SINGLECAMERA_5:
				Debug_SwitchMode[0] = DEBUG_MODE_SINGLE_VIDEO5;
				break;
			case DEBUG_ORDER_SINGLECAMERA_6:
				Debug_SwitchMode[0] = DEBUG_MODE_SINGLE_VIDEO6;
				break;
			case DEBUG_ORDER_SINGLECAMERA_7:
				Debug_SwitchMode[0] = DEBUG_MODE_SINGLE_VIDEO7;
				break;
			case DEBUG_ORDER_SINGLECAMERA_8:
				Debug_SwitchMode[0] = DEBUG_MODE_SINGLE_VIDEO8;
				break;
			case DEBUG_ORDER_SINGLECAMERA_9:
				Debug_SwitchMode[0] = DEBUG_MODE_SINGLE_VIDEO9;
				break;
			default:
				printf("No DEBUG_ORDER!\n");
			}
			pthread_mutex_unlock(&Mutex[0]);
			break;
		default:
			printf("%s no msg.msg_type ", __FUNCTION__);
		}
	}
	printf("++++++++++++Recv_ipc_Ephor msg.msg_type:%0.2ld+++++++++++++++++\n",
			msg.msg_type);
}

void *Recv_ipc_Driver(void *arg) {
	int ret_value;
	IPC_msg msg;
	printf("\n%x\n", ipc_qid[TRANSFER_TO_APP_DRIVER]);
	while (1) {
		memset(&msg, 0, sizeof(IPC_msg));
		ret_value = msgrcv(ipc_qid[TRANSFER_TO_APP_DRIVER], &msg, sizeof(msg),
				0, 0);
		if (ret_value < 0) {
			printf("%s Receive  IPC msg failed,errno=%d !!!\n", __FUNCTION__,
			errno);
		}
		switch (msg.msg_type) {
		case IPC_MSG_TYPE_DRIVER_CLICK:
			break;
		case IPC_MSG_TYPE_ETHOR_CLICK:
			pthread_mutex_lock(&Mutex[1]);
			Ephor_CoorPoint[1].point_x = msg.payload.click_coord.point_x;
			Ephor_CoorPoint[1].point_y = msg.payload.click_coord.point_y;
			pthread_mutex_unlock(&Mutex[1]);
			break;
		case IPC_MSG_TYPE_DRIVER_KEY:
			break;
		case IPC_MSG_TYPE_TURN_STATE:
			break;
		case IPC_MSG_TYPE_CAPTURE_STATE1:
			pthread_mutex_lock(&Mutex[1]);
			CaptureMessage.cameraFrontState = (msg.payload.Stete >> 24) & 0xff;
			CaptureMessage.cameraLeft1State = (msg.payload.Stete >> 16) & 0xff;
			CaptureMessage.cameraRight1State = (msg.payload.Stete >> 8) & 0xff;
			CaptureMessage.cameraLeft2State = msg.payload.Stete & 0xff;
			pthread_mutex_unlock(&Mutex[1]);
			break;
		case IPC_MSG_TYPE_CAPTURE_STATE2:
			pthread_mutex_lock(&Mutex[1]);
			CaptureMessage.cameraRight2State = (msg.payload.Stete >> 24) & 0xff;
			CaptureMessage.cameraLeft3State = (msg.payload.Stete >> 16) & 0xff;
			CaptureMessage.cameraRight3State = (msg.payload.Stete >> 8) & 0xff;
			CaptureMessage.cameraBackState = msg.payload.Stete & 0xff;
			pthread_mutex_unlock(&Mutex[1]);
			break;
		case IPC_MSG_TYPE_CAPTURE_STATE3:
			pthread_mutex_lock(&Mutex[1]);
			CaptureMessage.Cap_BoxState = (msg.payload.Cap_State >> 16) & 0xff;
			CaptureMessage.cameraFrontTest = !(msg.payload.Cap_State >> 8)
					& 0x1;
			CaptureMessage.cameraLeft1Test = !(msg.payload.Cap_State >> 8)
					& 0x2;
			CaptureMessage.cameraRight1Test = !(msg.payload.Cap_State >> 8)
					& 0x4;
			CaptureMessage.cameraLeft2Test = !(msg.payload.Cap_State >> 8)
					& 0x8;
			CaptureMessage.cameraRight2Test = !(msg.payload.Cap_State >> 8)
					& 0x10;
			CaptureMessage.cameraLeft3Test = !(msg.payload.Cap_State >> 8)
					& 0x20;
			CaptureMessage.cameraRight3Test = !(msg.payload.Cap_State >> 8)
					& 0x40;
			CaptureMessage.cameraBackTest = !(msg.payload.Cap_State >> 8)
					& 0x80;
			CaptureMessage.Cap_BoxTest = msg.payload.Cap_State & 0xff;
			pthread_mutex_unlock(&Mutex[1]);
			break;
		case IPC_MSG_TYPE_PASSENGER_STATE:
			pthread_mutex_lock(&Mutex[1]);
			CaptureMessage.passengerTest = (msg.payload.Cap_State >> 8) & 0xff;
			CaptureMessage.passengerState = msg.payload.Cap_State & 0xff;
			pthread_mutex_unlock(&Mutex[1]);
			break;
		case IPC_MSG_TYPE_NEARBOARD_STATE:
			pthread_mutex_lock(&Mutex[1]);
			CaptureMessage.nearBoardTest = (msg.payload.Cap_State >> 8) & 0xff;
			CaptureMessage.nearBoardState = msg.payload.Cap_State & 0xff;
			pthread_mutex_unlock(&Mutex[1]);
			break;
		case IPC_MSG_TYPE_PERISCOPIC_ANGLE:
			pthread_mutex_lock(&Mutex[1]);
			AngleFar_PeriscopicLens.hor_angle = msg.payload.angle_msg.hor_angle;
			AngleFar_PeriscopicLens.ver_angle = msg.payload.angle_msg.ver_angle;
			pthread_mutex_unlock(&Mutex[1]);
			break;
		case IPC_MSG_TYPE_TURRET:
			if (msg.payload.angle_msg.type == 0) {
				pthread_mutex_lock(&Mutex[1]);
				AngleFar_GunAngle.hor_angle = msg.payload.angle_msg.hor_angle;
				AngleFar_GunAngle.ver_angle = msg.payload.angle_msg.ver_angle;
				pthread_mutex_unlock(&Mutex[1]);
			} else if (msg.payload.angle_msg.type == 1) {
				pthread_mutex_lock(&Mutex[1]);
				AngleFar_CanonAngle.hor_angle = msg.payload.angle_msg.hor_angle;
				AngleFar_CanonAngle.ver_angle = msg.payload.angle_msg.ver_angle;
				pthread_mutex_unlock(&Mutex[1]);
			}
			break;
		case IPC_MSG_TYPE_CAPTURE_CONTROL:
			break;
		case IPC_MSG_TYPE_ETHOR_KEY:
			pthread_mutex_lock(&Mutex[1]);
			switch (msg.payload.key_value) {
			case KEY_TYPE_SINGLE_WINDOWS:
				Key_SwitchMode[1] = Mode_Type_SINGLE_POPUP_WINDOWS;
				break;
			case KEY_TYPE_DEBUGMODE:
				Key_SwitchMode[1] = Mode_Type_DEBUG;
				break;
			case KEY_TYPE_TARGET_DETECTION:
				Key_TargetDetectionState[1] = 1;
				break;
			case KEY_TYPE_IMAGE_ENHANCEMENT:
				Key_ImageEnhancementState[1] = 1;
				break;
			case KEY_TYPE_MOVEUP:
				Key_MoveDirection[1] = MOVE_TYPE_MOVEUP;
				break;
			case KEY_TYPE_MOVEDOWN:
				Key_MoveDirection[1] = MOVE_TYPE_MOVEDOWN;
				break;
			case KEY_TYPE_MOVELEFT:
				Key_MoveDirection[1] = MOVE_TYPE_MOVELEFT;
				break;
			case KEY_TYPE_MOVERIGHT:
				Key_MoveDirection[1] = MOVE_TYPE_MOVERIGHT;
				break;
			default:
				printf("No KEY_TYPE!\n");
			}
			pthread_mutex_unlock(&Mutex[1]);
			break;
		case IPC_MSG_TYPE_TRIMMING:
			pthread_mutex_lock(&Mutex[1]);
			switch (msg.payload.key_value) {
			case DEBUG_ORDER_PERISCOPIC_MODE:
				Debug_SwitchMode[1] = DEBUG_MODE_PERISCOPIC;
				break;
			case DEBUG_ORDER_CROSS_MOVEUP:
				Debug_CrossMoveDirectiom[1] = MOVE_TYPE_MOVEUP;
				break;
			case DEBUG_ORDER_CROSS_MOVEDOWN:
				Debug_CrossMoveDirectiom[1] = MOVE_TYPE_MOVEDOWN;
				break;
			case DEBUG_ORDER_CROSS_MOVELEFT:
				Debug_CrossMoveDirectiom[1] = MOVE_TYPE_MOVELEFT;
				break;
			case DEBUG_ORDER_CROSS_MOVERIGHT:
				Debug_CrossMoveDirectiom[1] = MOVE_TYPE_MOVERIGHT;
				break;
			case DEBUG_ORDER_TARGETDETECTION_ON:
				Debug_TargetDetectionState[1] = 1;
				break;
			case DEBUG_ORDER_TARGETDETECTION_OFF:
				Debug_TargetDetectionState[1] = 0;
				break;
			case DEBUG_ORDER_TRIMMING_ON:
				Debug_State[1] = DEBUG_STATE_ON;
				break;
			case DEBUG_ORDER_CHOOSECAMERA_LEFT:
				Debug_ChooseCamera[1] = DEBUG_CHOOSEVIDEO_LEFT;
				break;
			case DEBUG_ORDER_CHOOSECAMERA_RIGHT:
				Debug_ChooseCamera[1] = DEBUG_CHOOSEVIDEO_RIGHT;
				break;
			case DEBUG_ORDER_CHECKEDCAMERA_MOVEUP:
				Debug_CameraMoveDirection[1] = MOVE_TYPE_MOVEUP;
				break;
			case DEBUG_ORDER_CHECKEDCAMERA_MOVEDOWN:
				Debug_CameraMoveDirection[1] = MOVE_TYPE_MOVEDOWN;
				break;
			case DEBUG_ORDER_CHECKEDCAMERA_MOVELEFT:
				Debug_CameraMoveDirection[1] = MOVE_TYPE_MOVELEFT;
				break;
			case DEBUG_ORDER_CHECKEDCAMERA_MOVERIGHT:
				Debug_CameraMoveDirection[1] = MOVE_TYPE_MOVERIGHT;
				break;
			case DEBUG_ORDER_SAVE_TRIMMING_RESULT:
				Debug_State[1] = DEBUG_STATE_SAVE;
				break;
			case DEBUG_ORDER_CLEAN_CHECKEDCAMERA_RESULT:
				Debug_State[1] = DEBUG_STATE_CLEAN_CHECKEDCAMERA;
				break;
			case DEBUG_ORDER_CLEAN_ALLCAMERA_RESULT:
				Debug_State[1] = DEBUG_STATE_CLEAN_ALLCAMERA;
				break;
			case DEBUG_ORDER_TRIMMING_OFF:
				Debug_State[1] = DEBUG_STATE_OFF;
				break;
			case DEBUG_ORDER_SINGLECAMERA_MODE:
				Debug_SwitchMode[1] = DEBUG_MODE_SINGLE_VIDEO;
				break;
			case DEBUG_ORDER_SINGLECAMERA_0:
				Debug_SwitchMode[1] = DEBUG_MODE_SINGLE_VIDEO0;
				break;
			case DEBUG_ORDER_SINGLECAMERA_1:
				Debug_SwitchMode[1] = DEBUG_MODE_SINGLE_VIDEO1;
				break;
			case DEBUG_ORDER_SINGLECAMERA_2:
				Debug_SwitchMode[1] = DEBUG_MODE_SINGLE_VIDEO2;
				break;
			case DEBUG_ORDER_SINGLECAMERA_3:
				Debug_SwitchMode[1] = DEBUG_MODE_SINGLE_VIDEO3;
				break;
			case DEBUG_ORDER_SINGLECAMERA_4:
				Debug_SwitchMode[1] = DEBUG_MODE_SINGLE_VIDEO4;
				break;
			case DEBUG_ORDER_SINGLECAMERA_5:
				Debug_SwitchMode[1] = DEBUG_MODE_SINGLE_VIDEO5;
				break;
			case DEBUG_ORDER_SINGLECAMERA_6:
				Debug_SwitchMode[1] = DEBUG_MODE_SINGLE_VIDEO6;
				break;
			case DEBUG_ORDER_SINGLECAMERA_7:
				Debug_SwitchMode[1] = DEBUG_MODE_SINGLE_VIDEO7;
				break;
			case DEBUG_ORDER_SINGLECAMERA_8:
				Debug_SwitchMode[1] = DEBUG_MODE_SINGLE_VIDEO8;
				break;
			case DEBUG_ORDER_SINGLECAMERA_9:
				Debug_SwitchMode[1] = DEBUG_MODE_SINGLE_VIDEO9;
				break;
			default:
				printf("No DEBUG_ORDER!\n");
			}
			pthread_mutex_unlock(&Mutex[0]);
			break;
		default:
			printf("%s no msg.msg_type!\n ", __FUNCTION__);
		}
	}
}

void IPC_Destroy(int num) {
	msgctl(ipc_qid[num], IPC_RMID, 0); //删除消息队列
}

void IPC_DestroyEphor() {
	IPC_Destroy(TRANSFER_TO_APP_ETHOR);
}
void IPC_DestroyDriver() {
	IPC_Destroy(TRANSFER_TO_APP_DRIVER);
}

void IPC_Destroy_All() {
	IPC_DestroyEphor();
	IPC_DestroyDriver();
}

void err_IPC_NUM_TYPE(IPC_NUM_TYPE n) {
	if (n != TRANSFER_TO_APP_ETHOR && n != TRANSFER_TO_APP_DRIVER) {
		exit(-2);
	}
}

coor_p getEphor_CoorPoint(IPC_NUM_TYPE n) {
	err_IPC_NUM_TYPE(n);
	coor_p temp = Ephor_CoorPoint[n];
	Ephor_CoorPoint[n].point_x = -1;
	Ephor_CoorPoint[n].point_y = -1;
	return temp;
} //得到车长触摸屏坐标,只能调用一次，之后数据会清除
Mode_Type getKey_SwitchMode(IPC_NUM_TYPE n) {
	err_IPC_NUM_TYPE(n);
	Mode_Type temp = Key_SwitchMode[n];
	Key_SwitchMode[n] = Mode_Type_START;
	return temp;
} //得到车长显示器按键切换的模式,只能调用一次，之后数据会清除
int getKey_TargetDetectionState(IPC_NUM_TYPE n) {
	err_IPC_NUM_TYPE(n);
	int temp = Key_TargetDetectionState[n];
	Key_TargetDetectionState[n] = 0;
	return temp;
} //得到按键切换的目标检测状态，,只能调用一次，之后数据会清除
int getKey_ImageEnhancementState(IPC_NUM_TYPE n) {
	err_IPC_NUM_TYPE(n);
	int temp = Key_ImageEnhancementState[n];
	Key_ImageEnhancementState[n] = 0;
	return temp;
} //得到按键切换的图像增强状态，,只能调用一次，之后数据会清除
MOVE_TYPE getKey_MoveDirection(IPC_NUM_TYPE n) {
	err_IPC_NUM_TYPE(n);
	MOVE_TYPE temp = Key_MoveDirection[n];
	Key_MoveDirection[n] = MOVETYPE_START;
	return temp;
} //得到按键切换的移动方向,只能调用一次，之后数据会清除,没有移动返回零
ANGLE_GROUP getAngleFar_PeriscopicLens(IPC_NUM_TYPE n) {
	err_IPC_NUM_TYPE(n);
	ANGLE_GROUP temp = AngleFar_PeriscopicLens;
	return temp;
} //得到周视镜方位信息，数据会一直保存当前角度，并根据信息更新
ANGLE_GROUP getAngleFar_GunAngle(IPC_NUM_TYPE n) {
	err_IPC_NUM_TYPE(n);
	ANGLE_GROUP temp = AngleFar_GunAngle;
	return temp;
} //得到枪塔方位信息，数据会一直保存当前角度，并根据信息更新
ANGLE_GROUP getAngleFar_CanonAngle(IPC_NUM_TYPE n) {
	err_IPC_NUM_TYPE(n);
	ANGLE_GROUP temp = AngleFar_CanonAngle;
	return temp;
} //得到炮塔方位信息，数据会一直保存当前角度，并根据信息更新
Cap_Msg getCaptureMessage() {
	Cap_Msg temp = CaptureMessage;
	return temp;
} //采集信息，数据会一直保存，并根据新信息更新
/*微调*/
DEBUG_MODE getDebug_SwitchMode(IPC_NUM_TYPE n) {
	err_IPC_NUM_TYPE(n);
	DEBUG_MODE temp = Debug_SwitchMode[n];
	Debug_SwitchMode[n] = DEBUG_MODE_START;
	return temp;
} //切换模式，数据调用后清除
MOVE_TYPE getDebug_CrossMoveDirectiom(IPC_NUM_TYPE n) {
	err_IPC_NUM_TYPE(n);
	MOVE_TYPE temp = Debug_CrossMoveDirectiom[n];
	Debug_CrossMoveDirectiom[n] = MOVETYPE_START;
	return temp;
} //十字标移动方向，数据调用后清除
int getDebug_TargetDetectionState(IPC_NUM_TYPE n) {
	err_IPC_NUM_TYPE(n);
	int temp = Debug_TargetDetectionState[n];
	Debug_TargetDetectionState[n] = 0;
	return temp;
} //目标检测状态，数据调用后清除
DEBUG_STATE getDebug_State(IPC_NUM_TYPE n) {
	err_IPC_NUM_TYPE(n);
	DEBUG_STATE temp = Debug_State[n];
	Debug_State[n] = DEBUG_STATE_ORIGIN;
	return temp;
} //微调状态，数据调用后清除
DEBUG_CHOOSECAMERA getDebug_ChooseCamera(IPC_NUM_TYPE n) {
	err_IPC_NUM_TYPE(n);
	DEBUG_CHOOSECAMERA temp;
	return temp;
} //选择相机，数据调用后清除
MOVE_TYPE getDebug_CameraMoveDirection(IPC_NUM_TYPE n) {
	err_IPC_NUM_TYPE(n);
	MOVE_TYPE temp;
	return temp;
} //相机画面移动方向，数据调用后清除
