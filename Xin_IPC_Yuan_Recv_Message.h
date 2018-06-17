#ifndef XIN_IPC_YUAN_RECV_MESSAGE_H
#define XIN_IPC_YUAN_RECV_MESSAGE_H

typedef enum {
	TRANSFER_TO_APP_ETHOR = 0x00,
	TRANSFER_TO_APP_DRIVER = 0x01,
	TRANSFER_TO_COMM = 0x02,
	IPC_NUM,
} IPC_NUM_TYPE;

typedef enum {
	Mode_Type_START = 0x00,

	Mode_Type_SINGLE_POPUP_WINDOWS, Mode_Type_DEBUG,

	Mode_Type_RESERVE,
} Mode_Type; //物理按键模式切换

typedef enum {
	MOVETYPE_START = 0x00,

	MOVE_TYPE_MOVEUP,
	MOVE_TYPE_MOVEDOWN,
	MOVE_TYPE_MOVELEFT,
	MOVE_TYPE_MOVERIGHT,

	MOVE_TYPE_COUNT,
} MOVE_TYPE; //物理按键移动方向

typedef enum {
	POINT_GREEN = 0x00, POINT_RED = 0x01, POINT_GRAY = 0x02,
} PIONT_COLOUR; //圆点颜色用这个表示

typedef struct {
	char cameraFrontState;
	char cameraFrontTest;
	char cameraLeft1State;
	char cameraLeft1Test;
	char cameraRight1State;
	char cameraRight1Test;
	char cameraLeft2State;
	char cameraLeft2Test;
	char cameraRight2State;
	char cameraRight2Test;
	char cameraLeft3State;
	char cameraLeft3Test;
	char cameraRight3State;
	char cameraRight3Test;
	char cameraBackState;
	char cameraBackTest;
	char Cap_BoxState;
	char Cap_BoxTest;
	char passengerState;
	char passengerTest;
	char nearBoardState;
	char nearBoardTest;
	char Cap_FAULT_Colour;
	char passenger_FAULT_Colour;
	char nearBoard_FAULT_Colour;
} Cap_Msg; //预留的采集信息

typedef struct coordinate_point {
	int point_x;
	int point_y;
} coor_p; //点击的坐标

typedef struct {
	char type;
	float hor_angle;
	float ver_angle;
} ANGLE_GROUP;

typedef enum {
	DEBUG_MODE_START = 0x00,

	DEBUG_MODE_PERISCOPIC,
	DEBUG_MODE_SINGLE_VIDEO,
	DEBUG_MODE_SINGLE_VIDEO0,
	DEBUG_MODE_SINGLE_VIDEO1,
	DEBUG_MODE_SINGLE_VIDEO2,
	DEBUG_MODE_SINGLE_VIDEO3,
	DEBUG_MODE_SINGLE_VIDEO4,
	DEBUG_MODE_SINGLE_VIDEO5,
	DEBUG_MODE_SINGLE_VIDEO6,
	DEBUG_MODE_SINGLE_VIDEO7,
	DEBUG_MODE_SINGLE_VIDEO8,
	DEBUG_MODE_SINGLE_VIDEO9,

	DEBUG_MODE_RESERVE,
} DEBUG_MODE; //调试显示模式切换

typedef enum {
	DEBUG_STATE_ORIGIN = 0x0,

	DEBUG_STATE_ON,
	DEBUG_STATE_SAVE,
	DEBUG_STATE_CLEAN_CHECKEDCAMERA,
	DEBUG_STATE_CLEAN_ALLCAMERA,
	DEBUG_STATE_OFF,

	DEBUG_STATE_RESERVE,
} DEBUG_STATE; //调试状态

typedef enum {
	DEBUG_CHOOSEVIDEO_ORIGIN = 0x0,

	DEBUG_CHOOSEVIDEO_LEFT, DEBUG_CHOOSEVIDEO_RIGHT,

	DEBUG_CHOOSEVIDEO_RESERVE,
} DEBUG_CHOOSECAMERA; //选择相机

void IPC_Init_All(); //初始化所有消息队列
void IPC_Destroy_All(); //删除所有消息队列

coor_p getEphor_CoorPoint(IPC_NUM_TYPE n); //得到车长触摸屏坐标,只能调用一次，之后数据会清除
Mode_Type getKey_SwitchMode(IPC_NUM_TYPE n); //得到车长显示器按键切换的模式,只能调用一次，之后数据会清除
int getKey_TargetDetectionState(IPC_NUM_TYPE n); //得到按键切换的目标检测状态，只能调用一次，之后数据会清除
int getKey_ImageEnhancementState(IPC_NUM_TYPE n); //得到按键切换的图像增强状态，只能调用一次，之后数据会清除
MOVE_TYPE getKey_MoveDirection(IPC_NUM_TYPE n); //得到按键切换的移动方向,只能调用一次，之后数据会清除,没有移动返回零
ANGLE_GROUP getAngleFar_PeriscopicLens(IPC_NUM_TYPE n); //得到周视镜方位信息，数据会一直保存当前角度，并根据信息更新
ANGLE_GROUP getAngleFar_GunAngle(IPC_NUM_TYPE n); //得到枪塔方位信息，数据会一直保存当前角度，并根据信息更新
ANGLE_GROUP getAngleFar_CanonAngle(IPC_NUM_TYPE n); //得到炮塔方位信息，数据会一直保存当前角度，并根据信息更新
Cap_Msg getCaptureMessage(); //采集信息，数据会一直保存，并根据新信息更新
/*微调*/
DEBUG_MODE getDebug_SwitchMode(IPC_NUM_TYPE n); //切换模式，数据调用后清除
MOVE_TYPE getDebug_CrossMoveDirectiom(IPC_NUM_TYPE n); //十字标移动方向，数据调用后清除
int getDebug_TargetDetectionState(IPC_NUM_TYPE n); //目标检测状态，数据调用后清除
DEBUG_STATE getDebug_State(IPC_NUM_TYPE n); //微调状态，数据调用后清除
DEBUG_CHOOSECAMERA getDebug_ChooseCamera(IPC_NUM_TYPE n); //选择相机，数据调用后清除
MOVE_TYPE getDebug_CameraMoveDirection(IPC_NUM_TYPE n); //相机画面移动方向，数据调用后清除

#endif
