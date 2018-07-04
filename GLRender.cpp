/*
 * GLRender.cpp
 *
 *  Created on: Nov 3, 2016
 *      Author: hoover
 */
 //=====setting time headers======
 #include <time.h>
#include <stdio.h>    
#include <fcntl.h>    
#include <sys/types.h>    
#include <sys/stat.h>    
#include <sys/ioctl.h>    
#include <sys/time.h>    
#include <linux/rtc.h>    
#include <linux/capability.h> 
#include <unistd.h>
#include <ctype.h>
 //===========
#include "GLRender.h"
#include "CaptureGroup.h"
#include "RenderMain.h"
#include "common.h"
#include "main.h"
#ifdef CAPTURE_SCREEN
#include "cabinCapture.h"
#endif
#include "scanner.h"
#if USE_GAIN
#include"ExposureCompensationThread.h"
#include"overLapRegion.h"
#include "thread.h"
#endif
#if TRACK_MODE
#include "VideoProcessTrack.hpp"
#endif

#if MVDETECTOR_MODE
#include "mvDetector.hpp"
#endif

#include"signal.h"
#include"unistd.h"

#if USE_UART
#include"Zodiac_Message.h"
#endif
#include"Zodiac_GPIO_Message.h"
#include"ForeSight.h"
#include"CheckMyself.h"
#include"SelfCheckThread.h"
#include"StlGlDefines.h"
#include"ProcessIPCMsg.h"
#if 0
#include"GetScreenBuffer.h"
#endif
#include "gst_capture.h"
#include"GLEnv.h"
#if USE_CAP_SPI
#include"Cap_Spi_Message.h"
#endif
#include"Thread_Priority.h"
#include"MvDetect.hpp"
#include "thread_idle.h"

#include "Xin_IPC_Yuan_Recv_Message.h"


extern thread_idle tIdle;
extern unsigned char * target_data[CAM_COUNT];

char chosenCam[2]={3,3};

#if MVDECT
 extern MvDetect mv_detect;
#endif
int m_cam_pos=-1;

extern GLEnv env1;
extern GLEnv env2;
bool enable_hance=false;
bool saveSinglePic[CAM_COUNT]={false};
bool isTracking=false;

extern bool 	IsMvDetect;
bool IsgstCap=true;


PanoCamOnForeSight  panocamonforesight[2];
TelCamOnForeSight	     telcamonforesight[2];

//Process_Zodiac_Message  zodiac_msg;
extern AlarmTarget mainAlarmTarget;
extern unsigned char *sdi_data;
extern unsigned char *vga_data;
extern SelfCheck selfcheck;
extern ForeSightPos foresightPos[MS_COUNT];
#if USE_UART
extern IPC_msg  g_MSG[2];
#endif
static float xdelta=0;
static time_t time1,time2;
unsigned int last_gpio_sdi=999;
bool isinSDI=false;
using namespace std;
using namespace cv;

/* ASCII code for the various keys used */
#define ESCAPE 27     /* esc */
#define ROTyp  105    /* i   */
#define ROTym  109    /* m   */
#define ROTxp  107    /* k   */
#define ROTxm  106    /* j   */
#define SCAp   43     /* +   */
#define SCAm   45     /* -   */

#define SPECIAL_KEY_UP			101
#define SPECIAL_KEY_DOWN 		103
#define SPECIAL_KEY_LEFT 		100
#define SPECIAL_KEY_RIGHT 		102

#define SPECIAL_KEY_INSERT 		108
#define SPECIAL_KEY_HOME 		106
#define SPECIAL_KEY_END 		107
#define SPECIAL_KEY_PAGEUP 		104
#define SPECIAL_KEY_PAGEDOWN 	105

#define SPECIAL_KEY_LEFT_SHIFT	112
#define SPECIAL_KEY_RIGHT_SHIFT	113
#define SPECIAL_KEY_LEFT_CTRL	114
#define SPECIAL_KEY_RIGHT_CTRL	115
#define SPECIAL_KEY_LEFT_ALT	116
#define SPECIAL_KEY_RIGHT_ALT	117
#define SCAN_REGION_ANGLE 50.0f//10~180
#define RULER_START_ANGLE 0.0f

#define DELTA_OF_PANOFLOAT 0.5
#define PANO_FLOAT_DATA_FILENAME "panofloatdata.yml"
#define PANO_ROTATE_ANGLE_FILENAME "rotateangledata.yml"
#define DELTA_OF_PANO_SCALE 0.001
#define DELTA_OF_PANO_HOR 1
#define DELTA_OF_ROTATE_ANGLE 0.2
/* Stuff for the frame rate calculation */

int window; /* The number of our GLUT window */
extern Point3f bar[CAM_COUNT*2];
//extern RecordHandle * screen_handle;
#define SHOW_DIRECTION_DYNAMIC 1
#define HIDE_DIRECTION_DYNAMIC 0
//#define ALPHA_ZOOM_SCALE 0.50f
//#define SET_POINT_SCALE 512.0/256.0
#define SET_POINT_SCALE 480.0/240.0
#define RULER_ANGLE_MOVE_STEP 	0.80f    // move_step==360/450
#define SMALL_PANO_VIEW_SCALE   1.45/3.0

#define UP_DOWN_SCALE    1.4
#define TEL_XSCALE 13.6/18.4//1.0/8.0*7.38
#define TEL_XTRAS   12.83//3.182
#define TEL_YSCALE 1/5.8*10//5.62
#define TEL_YTRAS  1/2.7*2.8
//vector<cv::Point2f> LeftpixleList,RightpixleList;
#define COLOR_NORMAL 20
#define COLOR_LINE 21
#define PIXELS_ADD_ON_ALARM 64
#define ALARM_MIN_X  100
#define ALARM_MIN_Y  458
#define ALARM_MAX_X  1820
#define ALARM_MAX_Y  962
int alarm_period=200000;
float forward_data=-30.0f;
float x_set_angle=0.0;
float temp_math[2]={0};
int center_cam[2]={0};
int Twotimescenter_cam[2]={0};
float Twotimestemp_math[2]={0};
int TelTwotimescenter_cam[2]={0};
float TelTwotimestemp_math[2]={0};
int Fourtimescenter_cam[2]={0};
float Fourtimestemp_math[2]={0};
int Telscenter_cam[2]={0};
float Teltemp_math[2]={0};
extern float track_pos[4];
float  canshu[8]={0,0,0,0,0,0,0,0};
float  menu_tpic[8]={0,0,0,0,0,0,0,0};

extern bool enable_hance;

#if TRACK_MODE
CVideoProcess* trackMode=CVideoProcess::getInstance();
#endif
#if MVDETECTOR_MODE
mvDetector* pSingleMvDetector=mvDetector::getInstance();
#endif
void readcanshu()
{

	FILE * fp;
		int i=0;
		float read_data=0.0;
		fp=fopen("./data/AAAreadfile.txt","r");
		if(fp!=NULL)
		{
			for(i=0;i<8;i++)
			{
				fscanf(fp,"%f\n",&canshu[i]);
				printf("%f\n",canshu[i]);
			}
			fclose(fp);
		}
}

void readmenu_tpic()
{

	FILE * fp;
		int i=0;
		fp=fopen("./data/menu_tpic.txt","r");
		if(fp!=NULL)
		{
			for(i=0;i<8;i++)
			{
				fscanf(fp,"%f\n",&menu_tpic[i]);
				printf("%f\n",menu_tpic[i]);
			}
			fclose(fp);
		}
}



int getMaxData(int * data,int count)
{
	int max=data[0];
	int i=0;
	for(i=0;i<count;i++)
	{
		if(max<data[i])
		{
			max=data[i];
		}
	}
	return max;
}

int getMinData(int * data,int count)
{
	int min=data[0];
	int i=0;
	for(i=0;i<count;i++)
	{
		if(min>data[i])
		{
			min=data[i];
		}
	}
	return min;
}

void getoutline(int * src,int * dst,int count)
{
	int x[count/2],y[count/2];
	int i=0;
	for(i=0;i<count/2;i++)
	{
		x[i]=src[2*i];
		y[i]=src[2*i+1];
	}
	dst[0]=getMinData(x,count/2);
	dst[1]=getMinData(y,count/2);
	dst[2]=getMaxData(x,count/2);
	dst[3]=getMaxData(y,count/2);

}

void Render::SendtoTrack()
{
#if USE_UART
	int x=-1,y=-1,width=-1,height=-1;
	if(zodiac_msg.GetdispalyMode()==RECV_ENABLE_TRACK)
	{
		isTracking=true;
		 if(displayMode==VGA_WHITE_VIEW_MODE
					||displayMode==VGA_HOT_BIG_VIEW_MODE
					||displayMode==VGA_HOT_SMALL_VIEW_MODE
					||displayMode==VGA_FUSE_WOOD_LAND_VIEW_MODE
					||displayMode==VGA_FUSE_GRASS_LAND_VIEW_MODE
					||displayMode==VGA_FUSE_SNOW_FIELD_VIEW_MODE
					||displayMode==VGA_FUSE_DESERT_VIEW_MODE
					||displayMode==VGA_FUSE_CITY_VIEW_MODE)
			{
					foresightPos.ChangeEnlarge2Ori(g_windowWidth*1024.0/1920.0, g_windowHeight*768.0/1080.0,
					g_windowWidth*1434/1920, g_windowHeight,
					foresightPos.Change2TrackPosX(PanoLen/37.685200*15.505) ,
					foresightPos.Change2TrackPosY(PanoHeight/6.0000*11.600),
					TRACKFRAMEX, TRACKFRAMEY,
					PanoLen/37.685200*15.505,PanoHeight/6.0000*11.600);
					x=foresightPos.Getsendtotrack()[SEND_TRACK_START_X];
					y=foresightPos.Getsendtotrack()[SEND_TRACK_START_Y];
					width=foresightPos.Getsendtotrack()[SEND_TRACK_FRAME_X];
					height=foresightPos.Getsendtotrack()[SEND_TRACK_FRAME_Y];
#if TRACK_MODE
	trackMode->enableTrackAcq( x, y,width,height,VGA_TRACK);
#endif
			}
			else if(displayMode==SDI1_WHITE_BIG_VIEW_MODE
					||displayMode==SDI1_WHITE_SMALL_VIEW_MODE
					||displayMode==SDI2_HOT_BIG_VIEW_MODE
					||displayMode==SDI2_HOT_SMALL_VIEW_MODE)
			{
				foresightPos.ChangeEnlarge2Ori(g_windowWidth*1920.0/1920.0, g_windowHeight*1080.0/1080.0,
						g_windowWidth*1434/1920, g_windowHeight,
						foresightPos.Change2TrackPosX(PanoLen/37.685200*15.505) ,
						foresightPos.Change2TrackPosY(PanoHeight/6.0000*11.600),
						TRACKFRAMEX, TRACKFRAMEY,
						PanoLen/37.685200*15.505,PanoHeight/6.0000*11.600);
				x=foresightPos.Getsendtotrack()[SEND_TRACK_START_X];
										y=foresightPos.Getsendtotrack()[SEND_TRACK_START_Y];
												width=foresightPos.Getsendtotrack()[SEND_TRACK_FRAME_X];
														height=foresightPos.Getsendtotrack()[SEND_TRACK_FRAME_Y];
#if TRACK_MODE

														trackMode->enableTrackAcq( x, y,width,height,SDI_TRACK);
#endif
			}
			else if(displayMode==PAL1_WHITE_BIG_VIEW_MODE
					||displayMode==PAL1_WHITE_SMALL_VIEW_MODE
					||displayMode==PAL2_HOT_BIG_VIEW_MODE
					||displayMode==PAL2_HOT_SMALL_VIEW_MODE)
			{
				foresightPos.ChangeEnlarge2Ori(g_windowWidth*720.0/1920.0, g_windowHeight*576.0/1080.0,
						g_windowWidth*1346/1920, g_windowHeight,
						foresightPos.Change2TrackPosX(PanoLen/37.685200*14.524) ,
						foresightPos.Change2TrackPosY(PanoHeight/6.0000*11.600),
						TRACKFRAMEX, TRACKFRAMEY,
						PanoLen/37.685200*14.524,PanoHeight/6.0000*11.600);
				x=foresightPos.Getsendtotrack()[SEND_TRACK_START_X];
										y=foresightPos.Getsendtotrack()[SEND_TRACK_START_Y];
												width=foresightPos.Getsendtotrack()[SEND_TRACK_FRAME_X];
														height=foresightPos.Getsendtotrack()[SEND_TRACK_FRAME_Y];
#if TRACK_MODE
		 if(displayMode==PAL1_WHITE_BIG_VIEW_MODE
							||displayMode==PAL1_WHITE_SMALL_VIEW_MODE)
				 {
			 	 	 trackMode->enableTrackAcq( x, y,width,height,PAL_TRACK);
				 }
		 else if(displayMode==PAL2_HOT_BIG_VIEW_MODE
					||displayMode==PAL2_HOT_SMALL_VIEW_MODE)
		 {
			 trackMode->enableTrackAcq( x, y,width,height,PAL2_TRACK);
		 }

#endif
			}
		 g_MSG[1].payload.ipc_settings.display_mode=100;
	}
else
	{

	}
#endif

}

#if TRACK_MODE

int Render::getTrkId(int displayMode,int nextMode)
{
	
	int trackId;  ///0:sdi,1:vga 2:pal
	switch(displayMode)
	{
		case VGA_WHITE_VIEW_MODE:
		case VGA_HOT_BIG_VIEW_MODE:
		case VGA_HOT_SMALL_VIEW_MODE:
		case VGA_FUSE_WOOD_LAND_VIEW_MODE:
		case VGA_FUSE_GRASS_LAND_VIEW_MODE:
		case  VGA_FUSE_SNOW_FIELD_VIEW_MODE:
		case VGA_FUSE_DESERT_VIEW_MODE:
		case VGA_FUSE_CITY_VIEW_MODE:
			trackId=1;
			break;
		case SDI1_WHITE_BIG_VIEW_MODE:
		case SDI1_WHITE_SMALL_VIEW_MODE:
		case SDI2_HOT_BIG_VIEW_MODE:
		case SDI2_HOT_SMALL_VIEW_MODE:
			trackId=0;
			break;
		case PAL1_WHITE_BIG_VIEW_MODE:
		case PAL1_WHITE_SMALL_VIEW_MODE:
			trackId=2;
			break;
		case PAL2_HOT_BIG_VIEW_MODE:
		case	PAL2_HOT_SMALL_VIEW_MODE:
			trackId=3;
			break;
		default:
			return -1;
	}
	if(trackId==1)
	{
		switch(nextMode)
		{
		case VGA_WHITE_VIEW_MODE:
		case VGA_HOT_BIG_VIEW_MODE:
		case VGA_HOT_SMALL_VIEW_MODE:
		case VGA_FUSE_WOOD_LAND_VIEW_MODE:
		case VGA_FUSE_GRASS_LAND_VIEW_MODE:
		case  VGA_FUSE_SNOW_FIELD_VIEW_MODE:
		case VGA_FUSE_DESERT_VIEW_MODE:
		case VGA_FUSE_CITY_VIEW_MODE:
			return 1;
		default:
			return -1;
		}
	}else if(trackId==0){
		switch(nextMode)
		{
		case SDI1_WHITE_BIG_VIEW_MODE:
		case SDI1_WHITE_SMALL_VIEW_MODE:
		case SDI2_HOT_BIG_VIEW_MODE:
		case SDI2_HOT_SMALL_VIEW_MODE:
			return 0;
		default :
			return -1;
		}
	}else if(trackId==2){
		switch(nextMode)
		{
		case PAL1_WHITE_BIG_VIEW_MODE:
		case PAL1_WHITE_SMALL_VIEW_MODE:
			return 2;
		default :
			return -1;
		}
	}else{
		switch(nextMode)
		{
		case PAL2_HOT_BIG_VIEW_MODE:
		case	PAL2_HOT_SMALL_VIEW_MODE:
			return 3;
		default :
			return -1;
		}
	}
}
void Render::clearTrackParams()
{
	track_pos[0]=0;
	track_pos[1]=0;
	track_pos[2]=0;
	track_pos[3]=0;
}
#endif
void Render::writeFirstMode(int Modenum)
{
	static int LastMode=-1;
	if(Modenum!=LastMode)
	{
		char buf[12];
		FILE * fp=fopen("./data/firstMode.txt","w");
		if(fp==NULL)
		{
			cout<<"firstMode open failed"<<endl;
		}
		sprintf(buf,"%d\n",Modenum);
		fwrite(buf,sizeof(buf),1,fp);
		fclose(fp);
		LastMode=Modenum;
	}
}

int Render::readFirstMode()
{
	char buf[12];
	int firstMode;
	FILE * fp=fopen("./data/firstMode.txt","r");
	if(fp==NULL)
	{
		cout<<"firstMode open failed"<<endl;
	}
	fread(buf,sizeof(buf),1,fp);
	fclose(fp);
	firstMode=atoi(buf);
//	printf("firstMode==%d\n",firstMode);
	if(firstMode==0)
		firstMode=2;//VGA_WHITE_VIEW_MODE;
//	printf("firstMode==%d\n",firstMode);
	return firstMode;
}

void Render::Show_first_mode(int read_mode)
{
//	printf("read_mode=%d\n",read_mode);
	static bool once=true;
	if(once)
	{
		time(&time2);
		time2+=5;
		once=false;
	}
	time(&time1);

	if(time(&time1)>=time2)
	{
		if(read_mode==2)
			displayMode=VGA_WHITE_VIEW_MODE;
	else if(read_mode==3)
			displayMode=VGA_HOT_BIG_VIEW_MODE;
		else if(read_mode==4)
			displayMode=VGA_HOT_SMALL_VIEW_MODE;
		else if(read_mode==5)
			displayMode=VGA_FUSE_WOOD_LAND_VIEW_MODE;
		else if(read_mode==6)
			displayMode=VGA_FUSE_GRASS_LAND_VIEW_MODE;
		else if(read_mode==7)
			displayMode=VGA_FUSE_SNOW_FIELD_VIEW_MODE;
		else if(read_mode==8)
			displayMode=VGA_FUSE_DESERT_VIEW_MODE;
		else if(read_mode==9)
			displayMode=VGA_FUSE_CITY_VIEW_MODE;
		else if(read_mode==10)
		{
		}
		else if(read_mode==11)
		{
		}
		else if(read_mode==12)
		{

		}
		else if(read_mode==13)
		{

		}
		else if(read_mode==14)
		{

		}
		else if(read_mode==15)
			displayMode=SDI1_WHITE_BIG_VIEW_MODE;
		else if(read_mode==16)
				displayMode=SDI1_WHITE_SMALL_VIEW_MODE;
		else if(read_mode==17)
				displayMode=SDI2_HOT_BIG_VIEW_MODE;
		else if(read_mode==18)
				displayMode=SDI2_HOT_SMALL_VIEW_MODE;
		else if(read_mode==19)
				displayMode=PAL1_WHITE_BIG_VIEW_MODE;
		else if(read_mode==20)
				displayMode=PAL1_WHITE_SMALL_VIEW_MODE;
		else if(read_mode==21)
				displayMode=PAL2_HOT_BIG_VIEW_MODE;
		else if(read_mode==22)
				displayMode=PAL2_HOT_SMALL_VIEW_MODE;
		}
}


void Render::ReadPanoFloatDataFromFile(char * filename)
{
	FILE * fp;
	fp=fopen(filename,"r");
	int i=0;
	if(fp!=NULL)
	{
		for(i=0;i<CAM_COUNT;i++)
		{
			fscanf(fp,"%f\n",&PanoFloatData[i]);
		}
		fclose(fp);
	}
	else
	{
		for(i=0;i<CAM_COUNT;i++)
		{
			PanoFloatData[i]=0.0f;
		}
	}
}

void Render::WritePanoFloatDataFromFile(char * filename,float * panofloatdata)
{
	FILE * fp;
	fp=fopen(filename,"w");
	char data[20];
	int i=0;
	for(i=0;i<CAM_COUNT;i++)
	{
		sprintf(data,"%f\n",panofloatdata[i]);
		fwrite(data,strlen(data),1,fp);
	}
	fclose(fp);
}

void Render::ReadRotateAngleDataFromFile(char * filename)
{
	FILE * fp;
	fp=fopen(filename,"r");
	int i=0;
	if(fp!=NULL)
	{
		for(i=0;i<CAM_COUNT;i++)
		{
			fscanf(fp,"%f\n",&rotate_angle[i]);
		}
		fclose(fp);
	}
	else
	{
		for(i=0;i<CAM_COUNT;i++)
		{
			rotate_angle[i]=0.0f;
		}
		WritePanoFloatDataFromFile(filename,rotate_angle);
	}
}

void Render::WriteRotateAngleDataToFile(char * filename,float * rotateangledata)
{
	FILE * fp;
	fp=fopen(filename,"w");
	char data[20];
	int i=0;
	for(i=0;i<CAM_COUNT;i++)
	{
		sprintf(data,"%f\n",rotateangledata[i]);
		fwrite(data,strlen(data),1,fp);
	}
	fclose(fp);
}







static void set10camsOverlapArea(int count,int & direction,bool &AppOverlap);
static void setOverlapArea(int count,int & direction,bool &AppOverlap);
static void math_scale_pos(int direction,int count,int & scale_count,int & this_channel_max_count);
bool stop_scan=false;


#define USE_ICON 1



void Render::initPixle(void)
{
	const char* file = "./cylinder_pixelCoord";
	for(int i=0; i<CAM_COUNT; i++)
		readPixleFile(file, i);
}
void Render::readPixleFile(const char* file, int index)
{
	char filename[64];
	memset(filename,0,sizeof(filename));
	sprintf(filename,"%s_%02d.ini",file, index);

	FILE *fp = fopen(filename,"r");
	char buf[256];
	float fx,fy;
	if(fp==NULL)
	{
		printf("open %s error !\n",filename);
		return ;
	}

	pixleList[index].clear();

	char * retp=NULL;
	do
	{
		retp = fgets(buf,sizeof(buf),fp);
		//		sscanf(buf,"outer loop \n");

		retp = fgets(buf,sizeof(buf),fp);
		if(retp == NULL)
			break;
		sscanf(buf,"\tvertexpixel\t%f\t%f",&fx,&fy);
		pixleList[index].push_back(cv::Point2f(fx,fy));

		retp = fgets(buf,sizeof(buf),fp);
		sscanf(buf,"\tvertexpixel\t%f\t%f",&fx,&fy);
		pixleList[index].push_back(cv::Point2f(fx,fy));

		fgets(buf,sizeof(buf),fp);
		sscanf(buf,"\tvertexpixel\t%f\t%f",&fx,&fy);
		pixleList[index].push_back(cv::Point2f(fx,fy));

		fgets(buf,sizeof(buf),fp);
		//		sscanf(buf,"endloop \n");
	}while(!feof(fp));

	fclose(fp);
	//printf("file:%s size: %d\n",filename, (int)pixleList[index].size());
}

//-------------------------GL-related function---------------
Render::Render():g_subwindowWidth(0),g_subwindowHeight(0),g_windowWidth(0),g_windowHeight(0),isFullscreen(FALSE),
		g_nonFullwindowWidth(0),g_nonFullwindowHeight(0),bRotTimerStart(FALSE),
		bControlViewCamera(FALSE),pVehicle(NULL),
		isCalibTimeOn(FALSE),isDirectionOn(TRUE),p_BillBoard(NULL),p_BillBoardExt(NULL),p_FixedBBD_2M(NULL),
		p_FixedBBD_5M(NULL),p_FixedBBD_8M(NULL),p_FixedBBD_1M(NULL),
		m_presetCameraRotateCounter(0),m_ExtVideoId(EXT_CAM_0),
		fboMode(FBO_ALL_VIEW_MODE),
		displayMode(CHECK_MYSELF),
		SecondDisplayMode(SECOND_ALL_VIEW_MODE),
		p_DynamicTrack(NULL),m_DynamicWheelAngle(0.0f),
		stopcenterviewrotate(FALSE),rotateangle_per_second(10),set_scan_region_angle(SCAN_REGION_ANGLE),
		send_follow_angle_enable(false),p_CompassBillBoard(NULL),p_LineofRuler(NULL),refresh_ruler(true),
		EnterSinglePictureSaveMode(false),enterNumberofCam(0),EnablePanoFloat(false),testPanoNumber(0),PanoDirectionLeft(false),
		TrackSpeed(0.0),
		RulerAngle(0.0),
		PanoLen(0),
		SightWide(0),
		m_VGAVideoId(VGA_CAM_0),
		m_SDIVideoId(SDI_CAM_0),
		p_CornerMarkerGroup(NULL),
		psy_button_f1(true),psy_button_f2(true),psy_button_f3(true),psy_button_f8(true),
		canon_hor_angle(0.0),canon_ver_angle(0.0),gun_hor_angle(0.0),gun_ver_angle(0.0),calc_hor_data(0.0),calc_ver_data(0.0),
		touch_pos_x(0),touch_pos_y(0),
		shaderManager2(GLShaderManager(CAM_COUNT)),
		shaderManager(GLShaderManager(CAM_COUNT)),pPano(NULL)
{
	MOUSEx = 0, MOUSEy = 0, BUTTON = 0;
	ROTx = ROTy = ROTz =0;
	PANx = 0, PANy = 0;
	scale = 0;
	oScale = 1.0f;

	GenerateGLTextureIds();
	for(int i = 0 ; i < CORNER_COUNT; i++){
		pConerMarkerColors[i] = NULL;
	}
	track_control_params[0]=1920/2-30;
	track_control_params[1]=1080/2-30;
	track_control_params[2]=60;
	track_control_params[3]=60;

	for(int i=0; i<CAM_COUNT; i++)
		{
			OverLap[i] = new GLBatch;
			Petal_OverLap[i] = OverLap[i];
			env1.Setp_Panel_OverLap(i,new GLBatch);
			env2.Setp_Panel_OverLap(i,new GLBatch);
			env1.Setp_Panel_Petal_OverLap(i,env1.Getp_Panel_OverLap(i));
			env2.Setp_Panel_Petal_OverLap(i,env2.Getp_Panel_OverLap(i));
		}
	for(int i=0;i<12;i++)
	{
		for(int j=0;j<3;j++)
		{
			state_label_data[i][j]=1;
		}
	}
}

Render::~Render()
{
	destroyPixList();
	glDeleteTextures(PETAL_TEXTURE_COUNT,textures);
	glDeleteTextures(EXTENSION_TEXTURE_COUNT,extensionTextures);
#if USE_COMPASS_ICON
	glDeleteTextures(1,iconTextures);
#endif
#if	USE_ICON
	glDeleteTextures(1,iconRuler45Textures);
	glDeleteTextures(1,iconRuler90Textures);
	glDeleteTextures(1,iconRuler180Textures);
#endif
	delete p_ForeSightFacade;
	delete p_ForeSightFacade2;
	delete p_ForeSightFacade_Track;
	delete p_BillBoard;
	delete p_CompassBillBoard;
	delete p_BillBoardExt;
	delete p_FixedBBD_1M;
	delete p_FixedBBD_2M;
	delete p_FixedBBD_5M;
	delete p_FixedBBD_8M;
	delete p_DynamicTrack;
	delete p_CornerMarkerGroup;
	delete p_LineofRuler;
	for(int i=0; i<CAM_COUNT; i++)
	{
		if(Petal_OverLap[i])
			delete Petal_OverLap[i];
	}
}





void Render::destroyPixList()
{
	for(int i=0; i<CAM_COUNT; i++)
		pixleList[i].clear();
}


static void captureVGACam(GLubyte *ptr, int index,GLEnv &env)
{
//	CaptureGroup::GetVGACaptureGroup()->captureCam(ptr,index);
	#if TRACK_MODE
	Point p1,p2;
	p1.x=track_pos[0];
	p1.y=track_pos[1];
	p2.x=p1.x+track_pos[2];
	p2.y=p1.y+track_pos[3];
	if(track_pos[2]>0&&track_pos[3]>0)
	{
		if(isTracking)
		{
			Mat frame(768,1024,CV_8UC4,ptr);
			cv::rectangle( frame,p1,p2,Scalar(0,0,0, 0),2);
		}
	}
	#endif
}

static void captureSDICam(GLubyte *ptr, int index,GLEnv &env)
{
//	CaptureGroup::GetSDICaptureGroup()->captureCam(ptr,index);
	#if TRACK_MODE
	Point p1,p2;
	p1.x=track_pos[0];
	p1.y=track_pos[1];
	p2.x=p1.x+track_pos[2];
	p2.y=p1.y+track_pos[3];
	if(track_pos[2]>0&&track_pos[3]>0)
	{
		if(isTracking)
		{
			Mat frame(1080,1920,CV_8UC4,ptr);
			cv::rectangle( frame,p1,p2,Scalar(0,0,0, 0),2);
		}
	}
	#endif
}

static void capturePanoCam(GLubyte *ptr, int index,GLEnv &env)
{
	env.GetPanoCaptureGroup()->captureCam(ptr,index);
}

static void mainTarget0(GLubyte *ptr, int index,GLEnv &env)
{
#if MVDECT
	mv_detect.selectFrame(ptr,target_data[index],MAIN_TARGET_0,index);
#endif
}
static void mainTarget1(GLubyte *ptr, int index,GLEnv &env)
{
#if MVDECT
	mv_detect.selectFrame(ptr,target_data[index],MAIN_TARGET_1,index);
#endif
}
static void subTarget0(GLubyte *ptr, int index,GLEnv &env)
{
#if MVDECT
	mv_detect.selectFrame(ptr,target_data[index],SUB_TARGET_0,index);
#endif
}
static void subTarget1(GLubyte *ptr, int index,GLEnv &env)
{
#if MVDECT
	mv_detect.selectFrame(ptr,target_data[index],SUB_TARGET_1,index);
#endif
}

static void captureChosenCam(GLubyte *ptr, int index,GLEnv &env)
{
#	if USE_BMPCAP
#else
	index-=MAGICAL_NUM;
#endif
	env.GetChosenCaptureGroup()->captureCam(ptr,index);
}
//Fish calibrated
static void captureCamFish(GLubyte *ptr, int index,GLEnv &env)
{
	env.GetPanoCaptureGroup()->captureCamFish(ptr,index);
}

#if 1
static void captureRuler45Cam(GLubyte *ptr, int index,GLEnv &env)
{
#if USE_ICON
	env.GetMiscCaptureGroup()->captureCam(ptr,ICON_45DEGREESCALE);
#endif
}
static void captureRuler90Cam(GLubyte *ptr, int index,GLEnv &env)
{
#if USE_ICON
	env.GetMiscCaptureGroup()->captureCam(ptr,ICON_90DEGREESCALE);
#endif
}
static void captureRuler180Cam(GLubyte *ptr, int index,GLEnv &env)
{
#if USE_ICON
	env.GetMiscCaptureGroup()->captureCam(ptr,ICON_180DEGREESCALE);
#endif
}
#endif
/* Sets up Projection matrix according to command switch -o or -p */
/* called from initgl and the window resize function */
void Render::SetView(int Width, int Height)
{
	float aspect = (float)Width / (float)Height;
	float Z_Depth = BowlLoader.GetZ_Depth();
	float Big_Extent = BowlLoader.GetBig_Extent();
	float extent_neg_x = BowlLoader.Getextent_neg_x(), extent_neg_y = BowlLoader.Getextent_neg_y();
	float extent_pos_x = BowlLoader.Getextent_pos_x(), extent_pos_y = BowlLoader.Getextent_pos_y();
	g_windowWidth = Width;
	g_windowHeight = Height;
//	cout<<"Bowl neg_x, y, z = "<<extent_neg_x<<", "<<extent_neg_y<<", "<<BowlLoader.Getextent_neg_z()<<endl;
//	cout<<"Bolw pos_x, y, z = "<<extent_pos_x<<", "<<extent_pos_y<<", "<<BowlLoader.Getextent_pos_z()<<endl;
	if (common.isVerbose())
		printf("Window Aspect is: %f\n", aspect);


	if (common.isPerspective())
	{
		/* Calculate The Aspect Ratio Of The Window*/
		gluPerspective(FOV,(GLfloat)Width/(GLfloat)Height,0.1f,(Z_Depth + Big_Extent));
	}

	if (common.isOrtho())
	{
		/* glOrtho(left, right, bottom, top, near, far) */
		glOrtho((extent_neg_x*1.2f), (extent_pos_x*1.2f), (extent_neg_y*aspect), (extent_pos_y*aspect), -1.0f, 10.0f);
	}

}

/* Frame rate counter.  Based off of the Oreilly OGL demo !  */
/* updates the global variables FrameCount & FrameRate each time it is called. */
/* called from the main drawing function */
void Render::GetFPS()
{
	/* Number of samples for frame rate */
#define FR_SAMPLES 10

	static struct timeval last={0,0};
	struct timeval now;
	float delta;
	if (common.plusAndGetFrameCount() >= FR_SAMPLES) {
		gettimeofday(&now, NULL);
		delta= (now.tv_sec - last.tv_sec +(now.tv_usec - last.tv_usec)/1000000.0);
		last = now;
		common.setFrameRate(FR_SAMPLES / delta);
		common.setFrameCount(0);
	}
}


///////////////////////////////////////////////////////////////////////////////
// This function does any needed initialization on the rendering context.
// This is the first opportunity to do any OpenGL related tasks.
void Render::SetupRC(int windowWidth, int windowHeight)
{
	IPC_Init_All();

	readmenu_tpic();
#if 1
		ChangeMainChosenCamidx(3);
		ChangeSubChosenCamidx(3);
#endif
	GLEnv & env=env1;
	GLubyte *pBytes;
#if 1
	GLint nWidth=DEFAULT_IMAGE_WIDTH, nHeight=DEFAULT_IMAGE_HEIGHT, nComponents=GL_RGB8;
	GLenum format= GL_BGR;
#else
	GLint nWidth=DEFAULT_IMAGE_WIDTH, nHeight=DEFAULT_IMAGE_HEIGHT, nComponents=GL_RGBA8;
	GLenum format= GL_BGRA;
#endif

#if MVDECT
 mv_detect.ReadConfig();
#endif

	if(!shaderManager.InitializeStockShaders()){
		cout<<"failed to intialize shaders"<<endl;
		exit(1);
	}

	if(!env.Getp_PBOMgr()->Init()
				|| !env.Getp_PBOExtMgr()->Init()
				||!env.Getp_PBORcr()->Init()
				|| !env.Getp_PBOVGAMgr()->Init()
				|| !env.Getp_PBOSDIMgr()->Init()
				||!env.Getp_PBOChosenMgr()->Init()){
		cout<<"Failed to init PBO manager"<<endl;
			exit(1);
		}
		if(!env.Getp_FBOmgr()->Init())
		{
			printf("FBO init failed\n");
			exit(-1);
		}

	// midNight blue background
#if 1
	glClearColor(0.0f, 0/255.0f, 0.0f, 1.0f);//25/255.0f, 25/255.0f, 112/255.0f, 0.0f);
	glLineWidth(1.5f);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);     //enable blending
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glClearDepth(1.0);				/* Enables Clearing Of The Depth Buffer*/
	glDepthFunc(GL_LESS);			        /* The Type Of Depth Test To Do*/
	glEnable(GL_DEPTH_TEST);		        /* Enables Depth Testing*/
	glShadeModel(GL_SMOOTH);			/* Enables Smooth Color Shading*/
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	SetView(windowWidth, windowHeight);
#endif
	{//setting up models and their textures
#if 0
		overLapRegion::GetoverLapRegion()->SetSingleHightLightState(false);
			ReadPanoFloatDataFromFile(PANO_FLOAT_DATA_FILENAME);
#endif
		ReadPanoFloatDataFromFile(PANO_FLOAT_DATA_FILENAME);
		ReadRotateAngleDataFromFile(PANO_ROTATE_ANGLE_FILENAME);

		env.GettransformPipeline()->SetMatrixStacks(*(env.GetmodelViewMatrix()), *(env.GetprojectionMatrix()));
		InitLineofRuler(env);


#if 1
		GenerateCenterView();
		GenerateCompassView();
		GenerateScanPanelView();
		GeneratePanoView();

		GenerateTriangleView();
		GeneratePanoTelView(MAIN);
		GeneratePanoTelView(SUB);
		GenerateTrack();

		GenerateLeftPanoView();
		GenerateRightPanoView();
#endif
		GenerateLeftSmallPanoView();
		GenerateRightSmallPanoView();
#if 1
		float x;
		x=(p_LineofRuler->Load())/360.0*(render.get_PanelLoader().Getextent_pos_x()-render.get_PanelLoader().Getextent_neg_x());
		RulerAngle=p_LineofRuler->Load();
		for(int i=0;i<2;i++)
		{
		GenerateOnetimeView(i);
		GenerateOnetimeView2(i);

		GenerateTwotimesView(i);
		GenerateTwotimesView2(i);

		GenerateTwotimesTelView(i);
		GenerateFourtimesTelView(i);
		}
		GenerateCheckView();

		GenerateBirdView();
		GenerateFrontView();
		GenerateRearTopView();
		GenerateExtentView();
		GenerateChosenView();
		GenerateSDIView();
		GenerateVGAView();
		GenerateRender2FrontView();
		GenerateTargetFrameView();
#endif

		PanoLen=(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
		PanoHeight=(PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z());
		for(int i=0;i<MS_COUNT;i++)
		{
			foresightPos[i].SetPanoLen_Height(PanoLen,PanoHeight);
//		zodiac_msg.setPanoHeight_Length(PanoHeight,PanoLen);

	//	camonforesight.setPanoheight(PanoHeight);
		panocamonforesight[i].setPanoheight(PanoHeight);
		panocamonforesight[i].setPanolen(PanoLen);
		telcamonforesight[i].setPanoheight(PanoHeight);
		telcamonforesight[i].setPanolen(PanoLen);
		}
	//camonforesight.setPanolen(PanoLen);

		InitALPHA_ZOOM_SCALE();
	//	InitBowl();
		InitScanAngle();

		InitPanoScaleArrayData();
		InitPanel(env);
		InitFollowCross();
		InitRuler(env);
		InitCalibrate();
	//	InitOitVehicle(env);
	//	    glmDelete(VehicleLoader);
	//	pVehicle->initFBOs(windowWidth, windowHeight);

		InitShadow(env);
		InitBillBoard(env);
//		InitFrontTracks();
	//	InitWheelTracks();
	//	InitCrossLines();
		InitWealTrack();
		InitDynamicTrack(env);
		InitCornerMarkerGroup(env);
		initAlphaMask();


		initLabelBatch();
//		InitDataofAlarmarea();

				FILE *fp;
				char read_data[20];
				fp=fopen("forward.yml","r");
				if(fp!=NULL)
				{
					fscanf(fp,"%f\n",&forward_data);
					fclose(fp);
					printf("forward:%f\n",forward_data);
				}
		InitForesightGroupTrack(env);
		DrawNeedleonCompass(env);
		DrawTriangle(env);

		pthread_t th_rec;
	   	int arg_rec = 10;
	 	env.Set_FboPboFacade(*(env.Getp_FBOmgr()),*(env.Getp_PBORcr()));
	   	mPresetCamGroup.LoadCameras();
		// Load up CAM_COUNT textures
		glGenTextures(PETAL_TEXTURE_COUNT, textures);

#if WHOLE_PIC
		for(int i = 0; i < 1; i++){
				glBindTexture(GL_TEXTURE_2D, textures[i]);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
				glTexImage2D(GL_TEXTURE_2D,0,nComponents,PANO_TEXTURE_WIDTH, PANO_TEXTURE_HEIGHT, 0,
						format, GL_UNSIGNED_BYTE, 0);
			}
		for(int i = 0; i < TARGET_CAM_COUNT; i++){
				glBindTexture(GL_TEXTURE_2D, GL_TargetTextureIDs[i]);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
				glTexImage2D(GL_TEXTURE_2D,0,nComponents,FPGA_SINGLE_PIC_W, FPGA_SINGLE_PIC_H, 0,
						format, GL_UNSIGNED_BYTE, 0);
			}
#else
		for(int i = 0; i < CAM_COUNT; i++){
			glBindTexture(GL_TEXTURE_2D, textures[i]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
			if(i==1)
			{
			glTexImage2D(GL_TEXTURE_2D,0,nComponents,1280, 1080, 0,
										format, GL_UNSIGNED_BYTE, 0);
			}
			else
			glTexImage2D(GL_TEXTURE_2D,0,nComponents,PANO_TEXTURE_WIDTH, PANO_TEXTURE_HEIGHT, 0,
								format, GL_UNSIGNED_BYTE, 0);
		}
#endif
		//nComponents=GL_RGBA8;
		// format= GL_RGBA;
		// Alpha mask: 1/16 size of 1920x1080
		glBindTexture(GL_TEXTURE_2D, textures[ALPHA_TEXTURE_IDX]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,ALPHA_MASK_WIDTH, ALPHA_MASK_HEIGHT, 0,
				GL_RGBA, GL_UNSIGNED_BYTE, alphaMask);

		glBindTexture(GL_TEXTURE_2D, textures[ALPHA_TEXTURE_IDX0]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,ALPHA_MASK_WIDTH, ALPHA_MASK_HEIGHT, 0,
				GL_RGBA, GL_UNSIGNED_BYTE, alphaMask0);

		glBindTexture(GL_TEXTURE_2D, textures[ALPHA_TEXTURE_IDX1]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,ALPHA_MASK_WIDTH, ALPHA_MASK_HEIGHT, 0,
				GL_RGBA, GL_UNSIGNED_BYTE, alphaMask1);
	}
	//setting up extension textures etc.
	{
		glGenTextures(EXTENSION_TEXTURE_COUNT, extensionTextures);
		for(int i = 0; i < EXTENSION_TEXTURE_COUNT; i++){
			glBindTexture(GL_TEXTURE_2D, extensionTextures[i]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
			glTexImage2D(GL_TEXTURE_2D,0,nComponents,1920, 1080, 0,
					format, GL_UNSIGNED_BYTE, 0);
		}

		glGenTextures(VGA_TEXTURE_COUNT, VGATextures);
				for(int i = 0; i < VGA_TEXTURE_COUNT; i++){
					glBindTexture(GL_TEXTURE_2D, VGATextures[i]);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
					glTexImage2D(GL_TEXTURE_2D,0,nComponents,VGA_WIDTH, VGA_HEIGHT, 0,
							format, GL_UNSIGNED_BYTE, 0);
				}

		glGenTextures(SDI_TEXTURE_COUNT, SDITextures);
				for(int i = 0; i < SDI_TEXTURE_COUNT; i++){
					glBindTexture(GL_TEXTURE_2D, SDITextures[i]);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
					glTexImage2D(GL_TEXTURE_2D,0,nComponents,SDI_WIDTH, SDI_HEIGHT, 0,
							format, GL_UNSIGNED_BYTE, 0);
				}

		glGenTextures(CHOSEN_TEXTURE_COUNT, GL_ChosenTextures);
						for(int i = 0; i < CHOSEN_TEXTURE_COUNT; i++){
							glBindTexture(GL_TEXTURE_2D, GL_ChosenTextures[i]);
							glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
							glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
							glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
							glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
							glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
							glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
							glTexImage2D(GL_TEXTURE_2D,0,nComponents,SDI_WIDTH, SDI_HEIGHT, 0,
									format, GL_UNSIGNED_BYTE, 0);
						}

#if USE_COMPASS_ICON
		glGenTextures(1, iconTextures);
		for(int i = 0; i < 1; i++){
			glBindTexture(GL_TEXTURE_2D, iconTextures[i]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
			glTexImage2D(GL_TEXTURE_2D,0,nComponents,nWidth, nHeight, 0,
					format, GL_UNSIGNED_BYTE, 0);
		}
#endif
#if USE_ICON
		glGenTextures(1, iconRuler45Textures);
		for(int i = 0; i < 1; i++){
			glBindTexture(GL_TEXTURE_2D, iconRuler45Textures[i]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
			glTexImage2D(GL_TEXTURE_2D,0,nComponents,720, 576, 0,
					format, GL_UNSIGNED_BYTE, 0);
		}

		glGenTextures(1, iconRuler90Textures);
		for(int i = 0; i < 1; i++){
			glBindTexture(GL_TEXTURE_2D, iconRuler90Textures[i]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
			glTexImage2D(GL_TEXTURE_2D,0,nComponents,720, 576, 0,
					format, GL_UNSIGNED_BYTE, 0);
		}

		glGenTextures(1, iconRuler180Textures);
		for(int i = 0; i < 1; i++){
			glBindTexture(GL_TEXTURE_2D, iconRuler180Textures[i]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
			glTexImage2D(GL_TEXTURE_2D,0,nComponents,720, 576, 0,
					format, GL_UNSIGNED_BYTE, 0);
		}
#endif
	}
	glMatrixMode(GL_MODELVIEW);
}

///////////////////////////////////////////////////////////////////////////////
// Window has changed size, or has just been created. In either case, we need
// to use the window dimensions to set the viewport and the projection matrix.
void Render::ChangeSize(int w, int h)
{
	//pVehicle->ChangeSize(w, h);
	g_windowWidth = w;
	g_windowHeight = h;
	if(!isFullscreen){
		g_nonFullwindowWidth = w;
		g_nonFullwindowHeight = h;
	}
}
void Render::ChangeSizeDS(int w, int h)
{
	//pVehicle->ChangeSize(w, h);
	g_subwindowWidth = w;
	g_subwindowHeight = h;
	//if(!isFullscreen){
//		g_nonFullwindowWidth = w;
	//	g_nonFullwindowHeight = h;
//	}
}

/* The function called when our window is resized  */
void Render::ReSizeGLScene(int Width, int Height)
{
	if (Height==0)	/* Prevent A Divide By Zero If The Window Is Too Small*/
		Height=1;

	ChangeSize(Width, Height);
	common.setUpdate(GL_YES);
}

void Render::GenerateGLTextureIds()
{
	unsigned int textureCount = sizeof(GL_TextureIDs)/sizeof(GL_TextureIDs[0]);
	GL_TextureIDs[0] = GL_TEXTURE0;
	for(int i = 1; i < textureCount; i++){
	    GL_TextureIDs[i] = GL_TextureIDs[i-1] + 1;
	}

	textureCount = sizeof(GL_ExtensionTextureIDs)/sizeof(GL_ExtensionTextureIDs[0]);
	GL_ExtensionTextureIDs[0] = GL_TEXTURE25;
	for(int i = 1; i < textureCount; i++){
		GL_ExtensionTextureIDs[i] = GL_ExtensionTextureIDs[i-1] + 1;
	}

/*
	textureCount = sizeof(GL_VGATextureIDs)/sizeof(GL_VGATextureIDs[0]);
	GL_VGATextureIDs[0] = GL_TEXTURE22;
	for(int i = 1; i < textureCount; i++){
		GL_VGATextureIDs[i] = GL_VGATextureIDs[i-1] + 1;
	}

	textureCount = sizeof(GL_SDITextureIDs)/sizeof(GL_SDITextureIDs[0]);
	GL_SDITextureIDs[0] = GL_TEXTURE23;
	for(int i = 1; i < textureCount; i++){
		GL_SDITextureIDs[i] = GL_SDITextureIDs[i-1] + 1;
	}
*/
	textureCount = sizeof(GL_TargetTextureIDs)/sizeof(GL_TargetTextureIDs[0]);
	GL_TargetTextureIDs[0] = GL_TEXTURE26;
	for(int i = 1; i < textureCount; i++){
		GL_TargetTextureIDs[i] = GL_TargetTextureIDs[i-1] + 1;
	}

	textureCount = sizeof(GL_ChosenTextureIDs)/sizeof(GL_ChosenTextureIDs[0]);
	GL_ChosenTextureIDs[0] = GL_TEXTURE22;
	for(int i = 1; i < textureCount; i++){
		GL_ChosenTextureIDs[i] = GL_ChosenTextureIDs[i-1] + 1;
	}



	textureCount = sizeof(GL_FBOTextureIDs)/sizeof(GL_FBOTextureIDs[0]);
	GL_FBOTextureIDs[0] = GL_TEXTURE31;
	for(int i = 1; i < textureCount; i++){
		GL_FBOTextureIDs[i] = GL_FBOTextureIDs[i-1] + 1;
	}

#if USE_COMPASS_ICON
	textureCount = sizeof(GL_IconTextureIDs)/sizeof(GL_IconTextureIDs[0]);
	GL_IconTextureIDs[0] = GL_TEXTURE16;
	for(int i = 1; i < textureCount; i++){
		GL_IconTextureIDs[i] = GL_IconTextureIDs[i-1] + 1;
	}
#endif
#if USE_ICON
	textureCount = sizeof(GL_IconRuler45TextureIDs)/sizeof(GL_IconRuler45TextureIDs[0]);
	GL_IconRuler45TextureIDs[0] = GL_TEXTURE17;
	for(int i = 1; i < textureCount; i++){
		GL_IconRuler45TextureIDs[i] = GL_IconRuler45TextureIDs[i-1] + 1;
	}

	textureCount = sizeof(GL_IconRuler90TextureIDs)/sizeof(GL_IconRuler90TextureIDs[0]);
	GL_IconRuler90TextureIDs[0] = GL_TEXTURE18;
	for(int i = 1; i < textureCount; i++){
		GL_IconRuler90TextureIDs[i] = GL_IconRuler90TextureIDs[i-1] + 1;
	}

	textureCount = sizeof(GL_IconRuler180TextureIDs)/sizeof(GL_IconRuler180TextureIDs[0]);
	GL_IconRuler180TextureIDs[0] = GL_TEXTURE19;
	for(int i = 1; i < textureCount; i++){
		GL_IconRuler180TextureIDs[i] = GL_IconRuler180TextureIDs[i-1] + 1;
	}
#endif
}

void Render::DrawStringsWithHighLight(GLEnv &m_env,int w, int h, const char* s, int idx_HLt)
{
	static int flicker = 0;
    char quote[1][80];
    int i,l,lenghOfQuote;
    int numberOfQuotes=1;
    const GLfloat *PDefaultColor = DEFAULT_TEXT_COLOR;
    const GLfloat *pColor = PDefaultColor;
    const GLfloat HighLightColor[]={1.0f-pColor[0], 1.0f -pColor[1], 1.0f - pColor[2], pColor[3]};
    strcpy(quote[0],"2010-01-20 13:50:11");
    if(s)
    	strcpy(quote[0],s);
    GLfloat UpwardsScrollVelocity = -10.0;

    if(flicker++ > 10)
    	flicker = 0;
    m_env.GetmodelViewMatrix()->LoadIdentity();
    m_env.GetmodelViewMatrix()->Translate(0.0, 0.0, UpwardsScrollVelocity);
    //m_env.GetmodelViewMatrix()->Rotate(-20, 1.0, 0.0, 0.0);
    m_env.GetmodelViewMatrix()->Scale(0.08, 0.08, 0.08);
    glLineWidth(2);
    for(l=0;l<numberOfQuotes;l++)
    {
        lenghOfQuote = (int)strlen(quote[l]);
        m_env.GetmodelViewMatrix()->PushMatrix();
        m_env.GetmodelViewMatrix()->Translate(-(lenghOfQuote-5)*90.0f, -(l*200.0f), 0.0);
        for (i = 0; i < lenghOfQuote; i++)
        {
        	m_env.GetmodelViewMatrix()->Translate((90.0f), 0.0, 0.0);
			if(i == idx_HLt && flicker<5){
				if(PDefaultColor == vGrey)
					pColor = vWhite;
				else
				    pColor = HighLightColor;
			}
			else{
				pColor = PDefaultColor;
			}
            shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), pColor);
            glutStrokeCharacter(GLUT_STROKE_ROMAN, quote[l][i]);
        }
        m_env.GetmodelViewMatrix()->PopMatrix();
    }
}

void Render::DrawStrings(GLEnv &m_env,int w, int h, const char* s)
{
	DrawStringsWithHighLight(m_env,w,h,s);
}

bool Render::IsOverlay(bool AppDirection[CAM_COUNT], int *direction)
{
	for(int i=0; i<CAM_COUNT; i++)
	{
		if(AppDirection[i]&&AppDirection[(i+1)%CAM_COUNT])
		{
					*direction = i;
					return true;
		}
	}

	for(int i=0; i<CAM_COUNT; i++)
	{
		if(AppDirection[i])
		{
			*direction = i;
			return false;
		}
	}
//	printf("sth wrong in direction !\n");
}

void Render::generateAlphaList(Point2f AlphaList[], float alpha_index_x, float alpha_index_y, int id)
{
#define GET_X(num)         ((num) * alpha_index_x)// ((num) * alpha_x_step)
#define GET_Y(num)        ((num) * alpha_index_y) // ((num) * alpha_y_step)

#define GET_VALUE_Y(data, y)   ( GET_Y( (data)/2 + y) )

#define GET_VALUE_X_DOWN(data, x)   (GET_X( (data)%2) )

//	0         1
//	  --------
//	 |        |
//	 |        |
//	  --------
//	2          3
//  according to the points sequence of bowl.stl

//	printf("step_x=%f,step_y=%f,alhpa_x=%f,alpha_y=%f\n",alpha_x_step,alpha_y_step,alpha_index_x,alpha_index_y);
	int up[6]={2, 0, 1,  1, 3, 2};
//	int up[6]={3, 2, 0, 1, 0 , 3 };
	int *p = up;
	for(int k=0; k<3; k++)
	{
		AlphaList[k].x = GET_X(p[id%2*3+k]%2 + id/2);
		AlphaList[k].y = GET_VALUE_Y(p[id%2*3 + k], alpha_index_y);
//		printf("alhpa: k=%d, x=%f,y=%f\n",k, AlphaList[k].x, AlphaList[k].y);
	}
}
void Render::getOffsetValue(int direction, int x, int* offset_L, int* offset_R)
{
//	*offset_R = *offset_L = 0;
//	return;
	int offset_mod=BLEND_HEAD;
	int offset_rear=BLEND_REAR;
//	int line = x/PER_CIRCLE;

//	if(line > 60)
//	{
//		offset_mod = MAX(offset_mod,offset_mod+10);
//		offset_rear= MAX(offset_rear,offset_rear+5);
//	}
#if 0
	if(direction == CAM_0)
	{
		*offset_R = *offset_L = offset_mod;
	}else if(direction == CAM_1)
	{
		*offset_L = -offset_mod;
		*offset_R = -offset_rear;
	}else if(direction == CAM_COUNT-1)
	{
		*offset_R = -offset_mod;
		*offset_L = -offset_rear;
	}else if(direction == CAM_2)
	{
		*offset_R = *offset_L = offset_rear;
	}
#else
	if(direction == CAM_0)
	{
		*offset_R = *offset_L = offset_mod;
	}else if(direction == CAM_1)
	{
		*offset_L = -offset_mod;
		*offset_R = 0;
	}else if(direction == CAM_2)
	{
		*offset_L = -offset_mod;
		*offset_R = 0;
	}else 	if(direction == CAM_3)
	{
		*offset_L = -offset_mod;
		*offset_R =0;

	}else if(direction == CAM_4)
	{
		*offset_L = 0;
		*offset_R = 0;
	}
	else if(direction==CAM_5)
	{
		*offset_R = -offset_mod;
		*offset_L =0;
	}
	else if(direction==CAM_6)
	{
		*offset_R =-offset_mod;
		*offset_L = 0;
	}else if(direction == CAM_COUNT-1)
	{
		*offset_R = -offset_mod;
		*offset_L = 0;
	}
	*offset_R =0;
	*offset_L = 0;
#endif
}
//set direction false when this triangle has a vertex == (-1,-1)
void Render::checkDirection(bool AppDirection[],int x)
{
	cv::Point2f tmpPoint[3];
	for(int i = CAM_0 ; i<CAM_COUNT; i++ ){
		AppDirection[i] = true;
	          getPointsValue( i , x, tmpPoint);
	          for(int idx = 0; idx < 3; idx++){
				  if(tmpPoint[idx].x<0 ||tmpPoint[idx].y < 0){
					  AppDirection[i]= false;
					  break;
				  }
	          }
	}
}
void Render::markDirection(bool AppDirection[],int x)
{
#define EVEN(num) (((num)%2) ? ((num)+1) : (num))
#define ODD(num)  (((num)%2) ? (num) : ((num)-1))

	int zone = PER_CIRCLE/CAM_COUNT;//200/4
	int num = x%PER_CIRCLE; //横向
//	int num = x/MODLE_CIRCLE; //竖向
	int offset = zone/2;//2;//
	int offset_R, offset_L, offset_repeat = BLEND_OFFSET;//repeat is polygon = twice of triangles
	int max, min;
	int dir;
	for(int direction=0; direction<CAM_COUNT; direction++)
	{
		offset_R = offset_L = 0;
		getOffsetValue(direction,x,&offset_L,&offset_R);

		max = zone*(direction+1) - (offset+offset_L) + offset_repeat ;
		min = zone*(direction)   - (offset-offset_R) - offset_repeat;

		max = EVEN(max);
		min = ODD(min);

		if(LOOP_RIGHT==1)
			//dir = direction;
			dir = (CAM_COUNT-direction+2)%CAM_COUNT;
		else
			dir = (2*CAM_COUNT-direction-2)%CAM_COUNT;

		if(direction == 0)
		{
			AppDirection[dir] = (num>(min+PER_CIRCLE-2))||(num<max);
		}else
		{
			AppDirection[dir] = (num>min)&&(num<max);
		}
	}
}

void Render::fillDataList(vector<cv::Point3f> *list,int x)
{
	vector<cv::Point3f> *poly_list = BowlLoader.Getpoly_vector();
	list->clear();
	for(int i=0; i<4; i++)
	{
		list->push_back(poly_list->at(x*4+i));
	}
}

void Render::panel_fillDataList(vector<cv::Point3f> *list,int x,int idx)
{
	vector<cv::Point3f>::iterator iter;
	if(idx==0)
	{
		vector<cv::Point3f> *poly_list = PanelLoader.Getpoly_vector();
		list->clear();
		for(int i=0; i<4; i++)
		{
			list->push_back(poly_list->at(x*4+i));
		}
	}
	else if(idx==1)
	{
//		vector<cv::Point3f> *poly_list = PanelLoader.Getpoly_vector2();
//		list->clear();
//		for(int i=0; i<4; i++)
//		{
//			list->push_back(poly_list->at(x*4+i));
//		}
	}
#if 0
	if(idx==1)
	{
		static bool Once1=true;
	//	printf("idx=%d\n",idx);
		//sleep(5);
		if(Once1==true)
		{
			printf("dian1=%d\n",poly_list->at(0));
		printf("11111111111111list[0].x=%d list[0].y=%d list[0].z=%d\n",(*list)[0].x,(*list)[0].y,(*list)[0].z);
		Once1=false;
		}
	}
	else
	{
		static bool Once2=true;
	//	printf("idx=%d\n",idx);
	//	sleep(5);
		if(Once2==true)
		{
			printf("dian2=%d\n",poly_list->at(0));
			printf("333322222222222list[0].x=%d list[0].y=%d list[0].z=%d\n",(*list)[0].x,(*list)[0].y,(*list)[0].z);
			Once2=false;
	}
#endif
}

int Render::getOverlapIndex(int direction,int idx)
{
	if(idx==0)
	{
	static int count=0,dir=-1;
	if(dir != direction)
	{
		dir = direction;
		count = 0;
	}else
	{
		count ++;
	}
	return count;
	}
	else if(idx==1)
	{
		static int count2=0,dir2=-1;
		if(dir2 != direction)
		{
			dir2 = direction;
			count2 = 0;
		}else
		{
			count2 ++;
		}
		return count2;
		}
}

bool Render::getPointsValue(int direction, int x, Point2f *Point)
{
	for(int i=0; i<3; i++)
	{
		Point[i] = pixleList[ (direction)  %CAM_COUNT ][ (x)*3+i ];
	}
	return true;
}

bool Render::getOverLapPointsValue(int direction, int x, Point2f *Point1, Point2f *Point2)
{
	bool ret=true;
	ret &= getPointsValue(direction,   x, Point1);
	ret &= getPointsValue(direction+1, x, Point2);
	return ret;
}


void Render::InitBowl()
{
	if ((!common.isUpdate()) && (!common.isIdleDraw()))
		return;

	common.setUpdate(GL_NO);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);	/* Clear The Screen And The Depth Buffer*/
	calcCommonZone();

	cv::Point2f Point[3], Point1[3], Point2[3];
	cv::Point2f Alpha[3];
	vector<cv::Point3f> list;
	GLBatch *pBatch = &Petal[0];

	int poly_count = BowlLoader.Getpoly_count();
	bool AppOverlap = false, App = false;
	int direction=0, count=0;
	bool AppDirection[CAM_COUNT]={false};

	int extra_count=0;
	for(int petal_idx = 0; petal_idx < CAM_COUNT; petal_idx++){
		if(petal_idx==CAM_0)
		{
			extra_count=PER_CIRCLE;
		}
		else
		{
			extra_count=0;;
		}
		Petal[petal_idx].Begin(GL_TRIANGLES/* */,(poly_count+extra_count)*2*3,1);// each petal has 1 texture unit
		Petal_OverLap[petal_idx]->Begin(GL_TRIANGLES,1*(poly_count+extra_count)*2*3,3);// petal_overLap has 3 textures, left, right and alpha mask
	}
	float triangle_array[6][3]={{0.0, 0.0, 0.0},{0.0, 1.0, 0.0},{0.0,1.0,1.0},
			{0.0, 0.0, 0.0},{0.0, 1.0, 1.0},{0.0,0.0,1.0}};
	static int tri_dir=0;

	int temp_data=0,temp_data2=0;
	int even_data=0;
	int y=0;

	cv::Point2f set_alpha[6];
	static int alpha_dir=0;

	float temp_length=0.99;
	float x_data[6]={  temp_length,  1-temp_length,  1-temp_length,  temp_length,  1-temp_length,  temp_length};
	float y_data[6]={1-temp_length,1-temp_length,   temp_length,     temp_length,     temp_length,   1-temp_length};

	for(y=0;y<6;y++)
	{
		set_alpha[y].x=x_data[y];
		set_alpha[y].y=y_data[y];
	}
	for(int x = 0 ; x < (poly_count); x++)//loop through all vertex in triangles
	{
/*		if(x>=(poly_count/2-poly_count*1.4/8)&&(x<poly_count/2+poly_count*1.4/8))
		{
			continue;
		}
*/
	if(x>=(poly_count*7/40) && (x<poly_count*33/40))  //poly_count/512==40
	{
			continue;
	}
	fillDataList(&list, x);

		checkDirection(AppDirection, x);

		direction = INVALID_DIRECTION;
		AppOverlap = IsOverlay(AppDirection,&direction);

		if(direction==INVALID_DIRECTION)//out of show range ,the point is invalid
		{
			continue;
		}
#if  USE_GAIN
               if(AppOverlap)
               {
                       getPointsValue(direction, x, Point1);
                   	   overLapRegion::GetoverLapRegion()->push_overLap_triangle(direction,x);
               }
#endif

		AppOverlap=false;

		setOverlapArea(x,direction,AppOverlap);

		if(AppOverlap)
		{
//			overlappoint[direction].push_back(vector<int>(x));
		}

		if(AppOverlap)
		{
			App = false;
			pBatch = Petal_OverLap[direction%CAM_COUNT];

			count = getOverlapIndex(direction,0);
			generateAlphaList(Alpha, 1.0/BLEND_OFFSET,1.0*x/PER_CIRCLE, count);
			getOverLapPointsValue(direction, x, Point1, Point2);
		}else if(!pixleList[direction].empty())
		{
			App = true;
			pBatch = &Petal[direction%CAM_COUNT];
			getPointsValue(direction,x,Point);
		}else
		{
			printf("sth was wrong here x:%d!!\n",x);
			continue;
		}

DRAW:

		pBatch->Normal3f(list[0].x,list[0].y,list[0].z);
		int index=0;

		for (index=0; index<3; index++)
		{
			if(AppOverlap)
			{
				//continue;
				pBatch->MultiTexCoord2f(0, Point1[index].x/DEFAULT_IMAGE_WIDTH,  Point1[index].y/DEFAULT_IMAGE_HEIGHT);
				pBatch->MultiTexCoord2f(1, Point2[index].x/DEFAULT_IMAGE_WIDTH,  Point2[index].y/DEFAULT_IMAGE_HEIGHT);
				pBatch->MultiTexCoord2f(2, /*1 -*/ set_alpha[alpha_dir*3+index].x, set_alpha[alpha_dir*3+index].y);
			}else if(App)
			{
					pBatch->MultiTexCoord2f(0, Point[index].x/DEFAULT_IMAGE_WIDTH,  Point[index].y/DEFAULT_IMAGE_HEIGHT);
			}

			pBatch->Vertex3f(list[index+1].x, list[index+1].y, list[index+1].z);
			if(x==0||x==1||x==510||x==511||x==509||x==508)
			{
		//		printf("\ni:%d,index:%d,x:%f,y:%f\n",x,index,Point[index].x,Point[index].y);
			}
		}
		alpha_dir=1-alpha_dir;
		tri_dir=1-tri_dir;
	}
#if USE_GAIN
if( overLapRegion::GetoverLapRegion()->beExist())
       start_exposure_thread();
#endif
	// end petals
    for(int petal_idx = 0; petal_idx < CAM_COUNT; petal_idx ++){
		Petal[petal_idx].End();
		Petal_OverLap[petal_idx]->End();
    }
}

cv::Point2f RotatePoint(cv::Point2f Point,cv::Point2f rotate_center,float rotate_angle,float max_panel_length,int cam_count)
{
	cv::Point2f rotate_point_pre;
	cv::Point2f result_point;
	float rotate_math_a=0.0,rotate_math_b=0.0;
	float dec_length=0.0;
	float rotate_radius=0.0;
	rotate_point_pre=Point;
	rotate_math_a=rotate_point_pre.x-rotate_center.x;
	if(rotate_math_a>max_panel_length/cam_count)
	{
		rotate_math_a-=max_panel_length;
		dec_length=max_panel_length;
	}
	rotate_math_b=rotate_point_pre.y-rotate_center.y;
	rotate_radius=sqrt(rotate_math_a*rotate_math_a+rotate_math_b*rotate_math_b);
	if(rotate_radius!=0.0)
		rotate_math_b=180.0*asin(rotate_math_b/rotate_radius)/PI;
	else
		rotate_math_b=0.0;

	if(rotate_math_a<0.0&&rotate_radius!=0.0)
	{
		rotate_math_b=180.0-rotate_math_b;
	}
	rotate_math_b+=rotate_angle;
	if(rotate_math_b>360.0)
	{
		rotate_math_b-=360.0;
	}
	result_point.x=dec_length+rotate_radius*cos(rotate_math_b*PI/180.0)+rotate_center.x;
	result_point.y=rotate_radius*sin(rotate_math_b*PI/180.0)+rotate_center.y;

	return result_point;
}

void Render::InitPanel(GLEnv &m_env,int idx,bool reset)
{
//	printf("%d\n",idx);
//	sleep(3);
	if ((!common.isUpdate()) && (!common.isIdleDraw()))
		return;

	float pano_float_delta=0.0f;
	common.setUpdate(GL_NO);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);	/* Clear The Screen And The Depth Buffer*/
	calcCommonZone();

	cv::Point2f Point[3], Point1[3], Point2[3],PointZero[3],Point_temp[3];
	//cv::Point2f Alpha[3];
	vector<cv::Point3f> list;
	GLBatch *pBatch = m_env.GetPanel_Petal(0);
	int poly_count = PanelLoader.Getpoly_count();
	bool AppOverlap = false, App = false;
	int direction=0, count=0;
	bool AppDirection[CAM_COUNT]={false};

	int extra_count=0;
//每次初始化，对每个通道进行重置;只有第一次，使用Begin 初始化内存
		for(int petal_idx = 0; petal_idx < CAM_COUNT; petal_idx++){
			//pBatch = &Panel_Petal[petal_idx];
			pBatch=m_env.GetPanel_Petal(petal_idx);
			pBatch->Reset();//reset before init
			pBatch = m_env.Getp_Panel_Petal_OverLap(petal_idx);
			pBatch->Reset();//reset before init
			if(petal_idx==CAM_0)
			{
				extra_count=PER_CIRCLE;
			}
			else
			{
				extra_count=0;
			}
			if(!reset)
			{
				(*m_env.GetPanel_Petal(petal_idx)).Begin(GL_TRIANGLES/* */,(poly_count+extra_count)*2*3,1);// each petal has 1 texture unit
				m_env.Getp_Panel_Petal_OverLap(petal_idx)->Begin(GL_TRIANGLES,1*(poly_count+extra_count)*2*3,3);// petal_overLap has 3 textures, left, right and alpha mask
			}
	}
	int y=0;

	cv::Point2f set_alpha[6];
	static int alpha_dir=0;

	float temp_length=0.99;
	float x_data[6]={  temp_length,  1-temp_length,  1-temp_length,  temp_length,  1-temp_length,  temp_length};
	float y_data[6]={1-temp_length,1-temp_length,   temp_length,     temp_length,     temp_length,   1-temp_length};

	float base_x_scale=0.0,base_y_scale=0.0;
	int scale_count=0;
	int thechannel_max_count=0;

	getPointsValue(0,0,Point);
	base_x_scale=Point[0].x;
	base_y_scale=Point[0].y;

	static cv::Point2f rotate_center[CAM_COUNT];
	cv::Point2f rotate_point,rotate_point_pre;
	static bool rotate_init=true;
	float rotate_radius=0.0,rotate_math_a=0.0,rotate_math_b=0.0;
	if(rotate_init)
	{
		rotate_init=false;
		for(int x=0;x<CAM_COUNT;x++)
		{
			//getPointsValue(x,int(512.0*x/CAM_COUNT+0.6*512/CAM_COUNT),Point);
			getPointsValue(x,int(480.0*x/CAM_COUNT+0.6*480/CAM_COUNT),Point);
			rotate_center[x]=Point[0];
		}
	}

	float max_panel_length=PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x();
	float dec_length=0.0;


	for(y=0;y<6;y++)
	{
		set_alpha[y].x=x_data[y];
		set_alpha[y].y=y_data[y];
	}


	for(int x = 0 ; x <poly_count ; x++)//loop through all vertex in triangles
	{
		//if(x>=(poly_count*11/42+24) && (x<poly_count*22/42))  //poly_count/512==40
		//	{																								//	p_c/480=42
		//	continue;poly_count*33/42-416
		//	}
		if(x>=(poly_count/80*19) && (x<poly_count/80*62))
				{																								//	38400
				continue;
				}

		panel_fillDataList(&list, x,idx); //将STL文件值导入到list中


		//每个x在cam_count上，true则对于在该相机上，false则不在
		checkDirection(AppDirection, x);

		direction = INVALID_DIRECTION;
		//如果有两个true,即表明同时在两个相机上都存在，即为重合区
		AppOverlap = IsOverlay(AppDirection,&direction);

		if(direction==INVALID_DIRECTION)//out of show range ,the point is invalid
		{
			continue;
		}

		AppOverlap=false;
		 set10camsOverlapArea(x,direction,AppOverlap);
		math_scale_pos(direction,x,scale_count,thechannel_max_count);

		if(AppOverlap)
		{
			App = false;

			pBatch = m_env.Getp_Panel_Petal_OverLap(direction%CAM_COUNT);
			count = getOverlapIndex(direction,idx);
		//	generateAlphaList(Alpha, 1.0/BLEND_OFFSET,1.0*x/PER_CIRCLE, count);
			getOverLapPointsValue(direction, x, Point1, Point2);
			{
				for(int k=0;k<3;k++)
				{

			//point1图０左边，point2图１右边


			Point1[k].x = Point1[k].x/1920.0*640.0;
			Point1[k].y = Point1[k].y/1080.0*540.0+(direction%CAM_COUNT)*540.0;

			Point2[k].x = Point2[k].x/1920.0*640.0;
			Point2[k].y = Point2[k].y/1080.0*540.0+((direction+1)%CAM_COUNT)*540.0;


					Point1[k].x=Point1[k].x+move_hor[(direction)%CAM_COUNT];
					Point1[k].y=(Point1[k].y-base_y_scale)*(channel_left_scale[direction])+base_y_scale+PanoFloatData[direction];
					Point1[k]=RotatePoint( Point1[k],rotate_center[direction],rotate_angle[direction],max_panel_length,CAM_COUNT);

					Point2[k].x=Point2[k].x+move_hor[(direction+1)%CAM_COUNT];
					Point2[k].y=(Point2[k].y-base_y_scale)*(channel_right_scale[(direction+1)%CAM_COUNT])+base_y_scale+PanoFloatData[(direction+1)%CAM_COUNT];
					Point2[k]=RotatePoint( Point2[k],rotate_center[direction+1],rotate_angle[direction+1],max_panel_length,CAM_COUNT);

				}
			}


		}else if(!pixleList[direction].empty())
		{
			App = true;
			pBatch = 	m_env.GetPanel_Petal(direction%CAM_COUNT);
			getPointsValue(direction,x,Point);
			{
				for(int k=0;k<3;k++)
				{

					Point[k].x = Point[k].x/1920.0*640.0;
					Point[k].y= Point[k].y/1080.0*540.0+(direction%CAM_COUNT)*540.0;

					Point[k].x=Point[k].x+move_hor[direction];
					Point[k].y=(Point[k].y-base_y_scale)*(channel_right_scale[direction]+(channel_left_scale[direction]-channel_right_scale[direction])*scale_count/thechannel_max_count)+base_y_scale+PanoFloatData[direction];
					Point[k]=RotatePoint( Point[k],rotate_center[direction],rotate_angle[direction],max_panel_length,CAM_COUNT);
				}
			}
		}else
		{
			continue;
		}
DRAW:

	static bool Once2=true;
	static bool Once1=true;
	if(idx==0)
	{
			if(Once2==true)
			{
				printf("list[0].x=%f list[0].y=%f, list[0].z=%f\n",list[0].x,list[0].y,list[0].z);
				Once2=false;
		}
	}
	else
	{
		if(Once1==true)
		{
			printf("list[0].x=%f list[0].y=%f, list[0].z=%f\n",list[0].x,list[0].y,list[0].z);
			Once1=false;
		}
	}
//
		pBatch->Normal3f(list[0].x,list[0].y,list[0].z);
		int index=0;

		for (index=0; index<3; index++)
		{
			if(AppOverlap)
			{
				pBatch->MultiTexCoord2f(0, Point1[index].x/DEFAULT_IMAGE_WIDTH,  Point1[index].y/DEFAULT_IMAGE_HEIGHT);
				pBatch->MultiTexCoord2f(1, Point2[index].x/DEFAULT_IMAGE_WIDTH,  Point2[index].y/DEFAULT_IMAGE_HEIGHT);
				pBatch->MultiTexCoord2f(2, /*1 -*/ set_alpha[alpha_dir*3+index].x, set_alpha[alpha_dir*3+index].y);
			}else if(App)
			{
					pBatch->MultiTexCoord2f(0, Point[index].x/(float)DEFAULT_IMAGE_WIDTH,  ((Point[index].y)/(float)DEFAULT_IMAGE_HEIGHT));
			}
			pBatch->Vertex3f(list[index+1].x, list[index+1].y, list[index+1].z);
		}
		alpha_dir=1-alpha_dir;
	}
	// end petals
    for(int petal_idx = 0; petal_idx < CAM_COUNT; petal_idx ++){

    	m_env.GetPanel_Petal(petal_idx)->End();
		m_env.Getp_Panel_Petal_OverLap(petal_idx)->End();
    }
}


void Render::DrawFrontBackTracks(GLEnv &m_env)
{
	const GLfloat* pVehicleDimension = pVehicle->GetDimensions();
	const GLfloat* pVehicleYMaxMin = pVehicle->GetYMaxMins();
	m_env.GetmodelViewMatrix()->PushMatrix();

	glDisable(GL_BLEND);
	m_env.GetmodelViewMatrix()->Translate(0.0f, pVehicle->GetScale() *(pVehicleYMaxMin[0]),0.0f);
	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vLtYellow);
	glLineWidth(0.2f);
	WheelTrackBatch.Draw();

	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vLtYellow);
	glLineWidth(1.0f);
	WheelTrackBatch2.Draw();

	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vLtGreen);
	WheelTrackBatch5.Draw();

	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vRed);
	WheelTrackBatch1.Draw();

	//draw front tracks in symmetric ways
	m_env.GetmodelViewMatrix()->Translate( 0.0f,-( pVehicle->GetScale() *(pVehicleDimension[1])-1*(DEFAULT_VEHICLE_TRANSLATION_1)),	0.0f);
	m_env.GetmodelViewMatrix()->Rotate(180.0f,0.0f,0.0f,1.0f);

	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vGreen);
	glLineWidth(1.5f);
	FrontTrackBatch.Draw();

	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vGreen);
	glLineWidth(2.0f);
	FrontTrackBatch2.Draw();

	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vLtGreen);
	FrontTrackBatch5.Draw();

	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vRed);
	FrontTrackBatch1.Draw();

	m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::Draw4CrossLines(GLEnv &m_env)
{
	m_env.GetmodelViewMatrix()->PushMatrix();

	glDisable(GL_BLEND);

	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vBlack);
	glLineWidth(0.5);
	CrossLinesBatch.Draw();
	m_env.GetmodelViewMatrix()->PopMatrix();
}
void Render::DrawTrackHead(GLEnv &m_env)
{
	m_env.GetmodelViewMatrix()->PushMatrix();
	glDisable(GL_BLEND);
	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vRed);
	glLineWidth(1);
	WheelTrackBatchHead.Draw();
	m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::DrawTrackRear(GLEnv &m_env)
{
	m_env.GetmodelViewMatrix()->PushMatrix();
	glDisable(GL_BLEND);
	glLineWidth(5);
	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vRed);
	WheelTrackBatchRear1.Draw();	
	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vLtYellow);
	WheelTrackBatchRear2.Draw();
	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vLtGreen);
	WheelTrackBatchRear5.Draw();
	glEnable(GL_BLEND);
	glLineWidth(10);
	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vLtYellow);
	WheelTrackBatchRear.Draw();
	m_env.GetmodelViewMatrix()->PopMatrix();
}
void Render::DrawShadow(GLEnv &m_env)
{ 
	m_env.GetmodelViewMatrix()->PushMatrix();
	glDisable(GL_BLEND);
	const GLfloat* pVehicleDimension = pVehicle->GetDimensions();
	m_env.GetmodelViewMatrix()->Scale(DEFAULT_SHADOW_TO_VEHICLE_RATE_WIDTH*pVehicleDimension[2]/2, DEFAULT_SHADOW_TO_VEHICLE_RATE_LENGTH*pVehicleDimension[1]/2, 1.0f);	//since the plate is [-1,1], divide the factors by 2. And make it a rect
	m_env.GetmodelViewMatrix()->Translate(0.0f, DEFAULT_SHADOW_TRANSLATE_LENGTH_METER, 0.01f);
	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vBlack);
	m_env.Getp_shadowBatch()->Draw();
	glEnable(GL_BLEND);
	m_env.GetmodelViewMatrix()->PopMatrix();
}
// draw the individual video on shadow rect
void Render::DrawIndividualVideo(GLEnv &m_env,bool needSendData)
{
	int idx = p_BillBoard->m_Direction;
	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->Rotate(180.0f, 0.0f, 0.0f, 1.0f);
	m_env.GetmodelViewMatrix()->Rotate(180.0f,0.0f, 1.0f, 0.0f);
	glActiveTexture(GL_TextureIDs[idx]);
	
	if(needSendData){
		m_env.Getp_PBOMgr()->sendData(m_env,textures[idx], (PFN_PBOFILLBUFFER)captureCamFish /* captureCam */,idx);
	}
	else{
		glBindTexture(GL_TEXTURE_2D, textures[idx]);
	}

	shaderManager.UseStockShader(GLT_SHADER_TEXTURE_REPLACE,m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), idx);
	m_env.Getp_shadowBatch()->Draw();
	m_env.GetmodelViewMatrix()->PopMatrix();
	if((SPLIT_VIEW_MODE == displayMode))
	{
		if(CAM_0 == idx)
			DrawTrackHead(m_env);
		else if(CAM_3 == idx)
			DrawTrackRear(m_env);
	}
}

void Render::DrawTargetVideo(GLEnv &m_env,int  targetIdx,int camIdx,bool needSendData)
{
			m_env.GetmodelViewMatrix()->PushMatrix();
			m_env.GetmodelViewMatrix()->Rotate(180.0f, 0.0f, 0.0f, 1.0f);
			m_env.GetmodelViewMatrix()->Rotate(180.0f,0.0f, 1.0f, 0.0f);
			glActiveTexture(GL_TargetTextureIDs[targetIdx]);
			if(needSendData){
				switch (targetIdx)
				{
				case MAIN_TARGET_0:
					m_env.Getp_PBOTargetMgr()->sendData(m_env,GL_TargetTextureIDs[targetIdx], (PFN_PBOFILLBUFFER)mainTarget0,camIdx);
					break;
				case MAIN_TARGET_1:
					m_env.Getp_PBOTargetMgr()->sendData(m_env,GL_TargetTextureIDs[targetIdx], (PFN_PBOFILLBUFFER)mainTarget1,camIdx);

					break;
				case SUB_TARGET_0:
					m_env.Getp_PBOTargetMgr()->sendData(m_env,GL_TargetTextureIDs[targetIdx], (PFN_PBOFILLBUFFER)subTarget0,camIdx);

					break;
				case SUB_TARGET_1:
					m_env.Getp_PBOTargetMgr()->sendData(m_env,GL_TargetTextureIDs[targetIdx], (PFN_PBOFILLBUFFER)subTarget1,camIdx);

					break;
							}
			}
			else{
				glBindTexture(GL_TEXTURE_2D, GL_TargetTextureIDs[targetIdx]);
			}
			shaderManager.UseStockShader(GLT_SHADER_TEXTURE_REPLACE,m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), targetIdx+28);
			m_env.Getp_shadowBatch()->Draw();
			m_env.GetmodelViewMatrix()->PopMatrix();
	//todo
}
void Render::DrawVGAVideo(GLEnv &m_env,bool needSendData)
{
	if(1)//vga_data!=NULL)
	{
		int idx = GetCurrentVGAVideoId();
		m_env.GetmodelViewMatrix()->PushMatrix();
		m_env.GetmodelViewMatrix()->Rotate(180.0f, 0.0f, 0.0f, 1.0f);
		m_env.GetmodelViewMatrix()->Rotate(180.0f,0.0f, 1.0f, 0.0f);
		glActiveTexture(GL_VGATextureIDs[idx]);
		if(needSendData){
			m_env.Getp_PBOVGAMgr()->sendData(m_env,VGATextures[idx], (PFN_PBOFILLBUFFER)captureVGACam,idx);
		}
		else{
			glBindTexture(GL_TEXTURE_2D, VGATextures[idx]);
		}
		shaderManager.UseStockShader(GLT_SHADER_TEXTURE_REPLACE,m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), idx+22);// VGA texture start from 15
		m_env.Getp_shadowBatch()->Draw();
		m_env.GetmodelViewMatrix()->PopMatrix();
	}
}


void Render::DrawSDIVideo(GLEnv &m_env,bool needSendData)
{
	if(1)
	{
		int idx = GetCurrentSDIVideoId();
		m_env.GetmodelViewMatrix()->PushMatrix();
		m_env.GetmodelViewMatrix()->Rotate(180.0f, 0.0f, 0.0f, 1.0f);
		m_env.GetmodelViewMatrix()->Rotate(180.0f,0.0f, 1.0f, 0.0f);
		glActiveTexture(GL_SDITextureIDs[idx]);
			if(needSendData){
				m_env.Getp_PBOSDIMgr()->sendData(m_env,SDITextures[idx], (PFN_PBOFILLBUFFER)captureSDICam,idx+MAGICAL_NUM);
			}
			else{
				glBindTexture(GL_TEXTURE_2D, SDITextures[idx]);
			}
			shaderManager.UseStockShader(GLT_SHADER_TEXTURE_REPLACE,m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), idx+23);// VGA texture start from 15
			m_env.Getp_shadowBatch()->Draw();
		m_env.GetmodelViewMatrix()->PopMatrix();
	}
}


void Render::DrawChosenVideo(GLEnv &m_env,bool needSendData,int mainorsub)
{
		int idx = mainorsub;//GetCurrentChosenVideoId();
		m_env.GetmodelViewMatrix()->PushMatrix();
		m_env.GetmodelViewMatrix()->Rotate(180.0f, 0.0f, 0.0f, 1.0f);
		m_env.GetmodelViewMatrix()->Rotate(180.0f,0.0f, 1.0f, 0.0f);
		glActiveTexture(GL_ChosenTextureIDs[idx]);
			if(needSendData){
				m_env.Getp_PBOChosenMgr()->sendData(m_env,GL_ChosenTextures[idx], (PFN_PBOFILLBUFFER)captureChosenCam,idx+MAGICAL_NUM);
			}
			else{
				glBindTexture(GL_TEXTURE_2D, GL_ChosenTextures[idx]);
			}
#if USE_CPU
			shaderManager.UseStockShader(GLT_SHADER_ORI,m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), idx+22);// VGA texture start from 15
#else
			shaderManager.UseStockShader(GLT_SHADER_TEXTURE_REPLACE,m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), idx+22);// VGA texture start from 15
#endif
			m_env.Getp_shadowBatch()->Draw();
		m_env.GetmodelViewMatrix()->PopMatrix();
}


int alpha[12]={1,1,1,1,1,1,1,1,1,1,1,1};

/*PBOMgr.sendData(textures[0], (PFN_PBOFILLBUFFER)captureCam,i);\*/

#define SEND_TEXTURE_TO_PETAL(i,m_env) 		{\
											if(needSendData)\
											m_env.Getp_PBOMgr()->sendData(m_env,textures[0], (PFN_PBOFILLBUFFER)capturePanoCam,i);\
											else{\
												glBindTexture(GL_TEXTURE_2D, textures[0]);\
											}\
										}


#if USE_GAIN
#if WHOLE_PIC
#define USE_ENHANCE_TEXTURE_ON_PETAL_OVERLAP(m_env,i)        {\
        shaderManager.UseStockShader(GLT_SHADER_TEXTURE_ENHANCE_BLENDING, \
            m_env.GettransformPipeline()->GetModelViewProjectionMatrix(),0,\
           0,ALPHA_TEXTURE_IDX0+alpha[i],i);\
                }


#define USE_TEXTURE_ON_PETAL_OVERLAP(m_env,i)        {\
                                               shaderManager.UseStockShader(GLT_SHADER_TEXTURE_BLENDING, \
                                                   m_env.GettransformPipeline()->GetModelViewProjectionMatrix(),0,\
                                                  0,ALPHA_TEXTURE_IDX0+alpha[i],i);\
                                                       }
/*#define USE_ENHANCE_TEXTURE_ON_PETAL_OVERLAP(m_env,i)        {\
                                               shaderManager.UseStockShader(GLT_SHADER_TEXTURE_BLENDING_ORI, \
                                                   m_env.GettransformPipeline()->GetModelViewProjectionMatrix(),0,\
                                                  0,ALPHA_TEXTURE_IDX0+alpha[i],i);\
                                                       }*/
#else
#define USE_TEXTURE_ON_PETAL_OVERLAP(m_env,i)        {\
                                               shaderManager.UseStockShader(GLT_SHADER_TEXTURE_BLENDING, \
                                                   m_env.GettransformPipeline()->GetModelViewProjectionMatrix(),(i)%CAM_COUNT,\
                                                   (i+1)%CAM_COUNT,ALPHA_TEXTURE_IDX0+alpha[i],i);\
                                                       }
#endif
#else
#define USE_TEXTURE_ON_PETAL_OVERLAP(i)	{\
						shaderManager.UseStockShader(GLT_SHADER_TEXTURE_BLENDING, \
						    m_env.GettransformPipeline()->GetModelViewProjectionMatrix(),(i)%CAM_COUNT,\
						    (i+1)%CAM_COUNT,ALPHA_TEXTURE_IDX0+alpha[i]);\
							}
#endif
void Render::updateTexture(int id, bool needSendData)
{
//	SEND_TEXTURE_TO_PETAL(id);
//	USE_TEXTURE_ON_PETAL_OVERLAP(id);
}

void Render::prepareTexture(int t_id){
	glActiveTexture(GL_TextureIDs[t_id]);
	glBindTexture(GL_TEXTURE_2D, textures[t_id]);
}
void Render::drawDynamicTracks(GLEnv &m_env)
{
	prepareTexture(ALPHA_TEXTURE_IDX);
	prepareTexture(ALPHA_TEXTURE_IDX0);
	p_DynamicTrack->DrawTracks(m_env);
}
void Render::DrawBowl(GLEnv &m_env,bool needSendData,int mainOrsub)
{
	glDisable(GL_BLEND);

	m_env.GetmodelViewMatrix()->PushMatrix();
	//#pragma omp parallel sections
	{
		//bind alpha mask to texture6, render the imagei, imagei+1 and alphaMask on petal_overlap[i]
		glActiveTexture(GL_TextureIDs[ALPHA_TEXTURE_IDX]);
		glBindTexture(GL_TEXTURE_2D, textures[ALPHA_TEXTURE_IDX]);
		glActiveTexture(GL_TextureIDs[ALPHA_TEXTURE_IDX0]);
		glBindTexture(GL_TEXTURE_2D, textures[ALPHA_TEXTURE_IDX0]);
		glActiveTexture(GL_TextureIDs[ALPHA_TEXTURE_IDX1]);
		glBindTexture(GL_TEXTURE_2D, textures[ALPHA_TEXTURE_IDX1]);

		for(int i = 0; i < CAM_COUNT; i++){
			glActiveTexture(GL_TextureIDs[i]);
		    SEND_TEXTURE_TO_PETAL(i,m_env);
		}
		
		for(int i = 0; i < CAM_COUNT; i++){
#if USE_GAIN
                       shaderManager.UseStockShader(GLT_SHADER_TEXTURE_BRIGHT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), (i)%CAM_COUNT,i);
#else

			shaderManager.UseStockShader(GLT_SHADER_TEXTURE_REPLACE, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), (i)%CAM_COUNT);
#endif

			Petal[i].Draw();

			USE_TEXTURE_ON_PETAL_OVERLAP(m_env,i);
			Petal_OverLap[i]->Draw();
		}

		m_env.GetmodelViewMatrix()->PopMatrix();
	}
}

void Render::DrawPanel(GLEnv &m_env,bool needSendData,int *p_petalNum,int mainOrsub)
{
#ifdef GET_ALARM_AERA
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, pVehicle->msFBO);
#endif
	glDisable(GL_BLEND);
	m_env.GetmodelViewMatrix()->PushMatrix();
	//#pragma omp parallel sections
	{
		//bind alpha mask to texture6, render the imagei, imagei+1 and alphaMask on petal_overlap[i]
		glActiveTexture(GL_TextureIDs[ALPHA_TEXTURE_IDX]);
		glBindTexture(GL_TEXTURE_2D, textures[ALPHA_TEXTURE_IDX]);
		glActiveTexture(GL_TextureIDs[ALPHA_TEXTURE_IDX0]);
		glBindTexture(GL_TEXTURE_2D, textures[ALPHA_TEXTURE_IDX0]);
		glActiveTexture(GL_TextureIDs[ALPHA_TEXTURE_IDX1]);
		glBindTexture(GL_TEXTURE_2D, textures[ALPHA_TEXTURE_IDX1]);

		if(p_petalNum==NULL)
		{
				glActiveTexture(GL_TextureIDs[0]);
				for(int i = 0; i < 2; i++){
						SEND_TEXTURE_TO_PETAL(i,m_env);
			}
			for(int i = 0; i < CAM_COUNT; i++){
				if(	enable_hance)
				{
#if ENABLE_ENHANCE_FUNCTION
					shaderManager.UseStockShader(GLT_SHADER_ENHANCE, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), 0,i);
#endif
				}else
					shaderManager.UseStockShader(GLT_SHADER_TEXTURE_BRIGHT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), 0,i);
			//	shaderManager.UseStockShader(GLT_SHADER_ORI, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), (i)%CAM_COUNT);

				 (*m_env.GetPanel_Petal(i)).Draw();


				if(	enable_hance)
				{
#if ENABLE_ENHANCE_FUNCTION
				USE_ENHANCE_TEXTURE_ON_PETAL_OVERLAP(m_env,i);
#endif
				}
				else
				{
					USE_TEXTURE_ON_PETAL_OVERLAP(m_env,i);
				}
				m_env.Getp_Panel_Petal_OverLap(i)->Draw();
			}
		}
		else
		{
			glActiveTexture(GL_TextureIDs[0]);
			for(int i = 0; i < 2; i++){
				    SEND_TEXTURE_TO_PETAL(i,m_env);
		}
		for(int i=0;i<CAM_COUNT;i++)
				{
					if(p_petalNum[i]!=-1)
					{
						if(	enable_hance)
						{
#if ENABLE_ENHANCE_FUNCTION
							shaderManager.UseStockShader(GLT_SHADER_ENHANCE, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), 0,i);
#endif
						}
						else
						shaderManager.UseStockShader(GLT_SHADER_TEXTURE_BRIGHT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), 0,i);
				//shaderManager.UseStockShader(GLT_SHADER_ORI, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), (i)%CAM_COUNT);
				(*m_env.GetPanel_Petal(p_petalNum[i])).Draw();
				{
					if(enable_hance)
					{
						#if ENABLE_ENHANCE_FUNCTION
					USE_ENHANCE_TEXTURE_ON_PETAL_OVERLAP(m_env,p_petalNum[i]);
#endif
					}
					else
					{
					USE_TEXTURE_ON_PETAL_OVERLAP(m_env,p_petalNum[i]);
					}
				}
				m_env.Getp_Panel_Petal_OverLap(p_petalNum[i])->Draw();
					}
				}
		}
		m_env.GetmodelViewMatrix()->PopMatrix();
	}




#ifdef GET_ALARM_AERA
	glReadBuffer(GL_FRONT);

	glReadPixels(0,0,720,576,GL_BGRA_EXT,GL_UNSIGNED_BYTE,screen_data_1920X1080);

#endif
}

void Render::initAlphaMask()
{
	alpha_x_count = BLEND_OFFSET;
	alpha_y_count = MODLE_CIRCLE;
	alpha_x_step=1.0/alpha_x_count;
	alpha_y_step=1.0/alpha_y_count;
	float alpha_zoom_scale_data = getALPHA_ZOOM_SCALE();

	GLuint* pPixel = alphaMask;
	GLubyte alpha;
	for(int y = 0 ; y < ALPHA_MASK_HEIGHT; y ++)
	{
		for(int x = 0; x <ALPHA_MASK_WIDTH; x++)
		{
			if(x<((1-alpha_zoom_scale_data)*ALPHA_MASK_WIDTH))
			{
				alpha=255;
			}
			else
			{
				alpha =254*((ALPHA_MASK_WIDTH-x)/((alpha_zoom_scale_data)*ALPHA_MASK_WIDTH));//255;//255*x/(ALPHA_MASK_WIDTH-1);
			}
/*			if((x>(0.5+ALPHA_ZOOM_SCALE/2)*ALPHA_MASK_WIDTH))
			{
				alpha=0;
			}
			else if((x<(0.5-ALPHA_ZOOM_SCALE/2)*ALPHA_MASK_WIDTH))
			{
				alpha=255;
			}
			else
			{
			alpha =254*(1-(x-(0.5-ALPHA_ZOOM_SCALE/2))/(ALPHA_ZOOM_SCALE));//255;//255*x/(ALPHA_MASK_WIDTH-1);
			}*/
			GLuint pix = (alpha<<24)|(alpha<<16)|(alpha<<8)|alpha;
			*(pPixel+y*ALPHA_MASK_WIDTH+x) = pix;
		}
	}

	pPixel = alphaMask0;
	for(int y = 0 ; y < ALPHA_MASK_HEIGHT; y ++)
	{
		for(int x = 0; x <ALPHA_MASK_WIDTH; x++)
		{
			GLubyte alpha =0;// 255;//255*x/(ALPHA_MASK_WIDTH-1);
			GLuint pix = *(pPixel+y*ALPHA_MASK_WIDTH+x);
			*(pPixel+y*ALPHA_MASK_WIDTH+x) = (pix & 0x00FFFFFF)| (alpha<<24);
		}
	}

	pPixel = alphaMask1;
	for(int y = 0 ; y < ALPHA_MASK_HEIGHT; y ++)
	{
		for(int x = 0; x <ALPHA_MASK_WIDTH; x++)
		{
			GLubyte alpha = 0;//255;//255*x/(ALPHA_MASK_WIDTH-1);
			GLuint pix = *(pPixel+y*ALPHA_MASK_WIDTH+x);
			*(pPixel+y*ALPHA_MASK_WIDTH+x) = (pix & 0x00FFFFFF)| (alpha<<24);
		}
	}
}

void Render::InitForesightGroupTrack(GLEnv &m_env)
{
	float pano_length=PanoLen;//(render.get_PanelLoader().Getextent_pos_x()-render.get_PanelLoader().Getextent_neg_x());
	float pano_height=PanoHeight;//(render.get_PanelLoader().Getextent_pos_z()-render.get_PanelLoader().Getextent_neg_z());
		int pcindex=0;


		GLfloat pano_cross[48][3];
		pano_cross[pcindex][0] =-pano_length/144;
		pano_cross[pcindex][1] =0;
		pano_cross[pcindex++][2] = pano_height/2;
		pano_cross[pcindex][0] =pano_length/144;
		pano_cross[pcindex][1] =0;
		pano_cross[pcindex++][2] = pano_height/2;

		pano_cross[pcindex][0] =0;
		pano_cross[pcindex][1] =0;
		pano_cross[pcindex++][2] = pano_height/2-pano_length/144;
		pano_cross[pcindex][0] =0;
		pano_cross[pcindex][1] =0;
		pano_cross[pcindex++][2] =  pano_height/2+pano_length/144;

	int pirindex=0;
	GLfloat pano_inner_rect[48][3];
	pano_inner_rect[pirindex][0] =-pano_length/32/2;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] = pano_height/insideDH;
	pano_inner_rect[pirindex][0] =-pano_length/32/2;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height/insideDH+pano_length/insideLen;

	pano_inner_rect[pirindex][0] =-pano_length/32/2;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height/insideDH;
	pano_inner_rect[pirindex][0] =-pano_length/32/2+pano_length/insideLen;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height/insideDH;

	pano_inner_rect[pirindex][0] =pano_length/32/2;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height/insideDH;
	pano_inner_rect[pirindex][0] =pano_length/32/2-pano_length/insideLen;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height/insideDH;

	pano_inner_rect[pirindex][0] =pano_length/32/2;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height/insideDH;
	pano_inner_rect[pirindex][0] =pano_length/32/2;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height/insideDH+pano_length/insideLen;

	pano_inner_rect[pirindex][0] =pano_length/32/2;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height*insideUH;
	pano_inner_rect[pirindex][0] =pano_length/32/2-pano_length/insideLen;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height*insideUH;

	pano_inner_rect[pirindex][0] =pano_length/32/2;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height*insideUH;
	pano_inner_rect[pirindex][0] =pano_length/32/2;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height*insideUH-pano_length/insideLen;

	pano_inner_rect[pirindex][0] =-pano_length/32/2;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height*insideUH;
	pano_inner_rect[pirindex][0] =-pano_length/32/2+pano_length/insideLen;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height*insideUH;

	pano_inner_rect[pirindex][0] =-pano_length/32/2;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height*insideUH;
	pano_inner_rect[pirindex][0] =-pano_length/32/2;
	pano_inner_rect[pirindex][1] =0;
	pano_inner_rect[pirindex++][2] =  pano_height*insideUH-pano_length/insideLen;


	 int porindex=0;
	GLfloat pano_outer_rect[48][3];
	float ttdelta=pano_length/9/25;
	pano_outer_rect[porindex][0] =-pano_length/9/2+ttdelta;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height/outsideDH;
	pano_outer_rect[porindex][0] =-pano_length/9/2+ttdelta+pano_length/outsideLen;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height/outsideDH;

	pano_outer_rect[porindex][0] =-pano_length/9/2+ttdelta;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height/outsideDH;
	pano_outer_rect[porindex][0] =-pano_length/9/2+ttdelta;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height/outsideDH+pano_length/outsideLen;

	pano_outer_rect[porindex][0] =pano_length/9/2-ttdelta;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height/outsideDH;
	pano_outer_rect[porindex][0] =pano_length/9/2-ttdelta-pano_length/outsideLen;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height/outsideDH;

	pano_outer_rect[porindex][0] =pano_length/9/2-ttdelta;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height/outsideDH;
	pano_outer_rect[porindex][0] =pano_length/9/2-ttdelta;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height/outsideDH+pano_length/outsideLen;

	pano_outer_rect[porindex][0] =pano_length/9/2-ttdelta;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height*outsideUH;
	pano_outer_rect[porindex][0] =pano_length/9/2-ttdelta-pano_length/outsideLen;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height*outsideUH;

	pano_outer_rect[porindex][0] =pano_length/9/2-ttdelta;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height*outsideUH;
	pano_outer_rect[porindex][0] =pano_length/9/2-ttdelta;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height*outsideUH-pano_length/outsideLen;


	pano_outer_rect[porindex][0] =-pano_length/9/2+ttdelta;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height*outsideUH;
	pano_outer_rect[porindex][0] =-pano_length/9/2+ttdelta+pano_length/outsideLen;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height*outsideUH;

	pano_outer_rect[porindex][0] =-pano_length/9/2+ttdelta;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height*outsideUH;
	pano_outer_rect[porindex][0] =-pano_length/9/2+ttdelta;
	pano_outer_rect[porindex][1] =0;
	pano_outer_rect[porindex++][2] =  pano_height*outsideUH-pano_length/outsideLen;


/*******2********/
	int  tcindex=0;
	GLfloat tel_cross[48][3];
	tel_cross[tcindex][0] =-pano_length/288;
	tel_cross[tcindex][1] =0;
	tel_cross[tcindex++][2] = pano_height/2;
	tel_cross[tcindex][0] =pano_length/288;
	tel_cross[tcindex][1] =0;
	tel_cross[tcindex++][2] = pano_height/2;

	tel_cross[tcindex][0] =0;
	tel_cross[tcindex][1] =0;
	tel_cross[tcindex++][2] = pano_height/2-pano_length/250;
	tel_cross[tcindex][0] =0;
	tel_cross[tcindex][1] =0;
	tel_cross[tcindex++][2] =  pano_height/2+pano_length/250;


	 int tirindex=0;
	GLfloat tel_inner_rect[48][3];
	tel_inner_rect[tirindex][0] =-pano_length/64/2;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] = pano_height/inside4DH;
	tel_inner_rect[tirindex][0] =-pano_length/64/2;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4DH+pano_length/insideLen;

	tel_inner_rect[tirindex][0] =-pano_length/64/2;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4DH;
	tel_inner_rect[tirindex][0] =-pano_length/64/2+pano_length/insideLen;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4DH;

	tel_inner_rect[tirindex][0] =pano_length/64/2;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4DH;
	tel_inner_rect[tirindex][0] =pano_length/64/2-pano_length/insideLen;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4DH;

	tel_inner_rect[tirindex][0] =pano_length/64/2;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4DH;
	tel_inner_rect[tirindex][0] =pano_length/64/2;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4DH+pano_length/insideLen;

	tel_inner_rect[tirindex][0] =pano_length/64/2;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4UH;
	tel_inner_rect[tirindex][0] =pano_length/64/2-pano_length/insideLen;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4UH;

	tel_inner_rect[tirindex][0] =pano_length/64/2;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4UH;
	tel_inner_rect[tirindex][0] =pano_length/64/2;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4UH-pano_length/insideLen;

	tel_inner_rect[tirindex][0] =-pano_length/64/2;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4UH;
	tel_inner_rect[tirindex][0] =-pano_length/64/2+pano_length/insideLen;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4UH;

	tel_inner_rect[tirindex][0] =-pano_length/64/2;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4UH;
	tel_inner_rect[tirindex][0] =-pano_length/64/2;
	tel_inner_rect[tirindex][1] =0;
	tel_inner_rect[tirindex++][2] =  pano_height/inside4UH-pano_length/insideLen;


	 int torindex=0;
	GLfloat tel_outer_rect [48][3];   //outer_rect

	tel_outer_rect[torindex][0] =-pano_length/16/2;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] = pano_height/insideDH;
	tel_outer_rect[torindex][0] =-pano_length/16/2;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height/insideDH+pano_length/insideLen;

	tel_outer_rect[torindex][0] =-pano_length/16/2;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height/insideDH;
	tel_outer_rect[torindex][0] =-pano_length/16/2+pano_length/insideLen;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height/insideDH;

	tel_outer_rect[torindex][0] =pano_length/16/2;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height/insideDH;
	tel_outer_rect[torindex][0] =pano_length/16/2-pano_length/insideLen;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height/insideDH;

	tel_outer_rect[torindex][0] =pano_length/16/2;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height/insideDH;
	tel_outer_rect[torindex][0] =pano_length/16/2;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height/insideDH+pano_length/insideLen;

	tel_outer_rect[torindex][0] =pano_length/16/2;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height*insideUH;
	tel_outer_rect[torindex][0] =pano_length/16/2-pano_length/insideLen;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height*insideUH;

	tel_outer_rect[torindex][0] =pano_length/16/2;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height*insideUH;
	tel_outer_rect[torindex][0] =pano_length/16/2;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height*insideUH-pano_length/insideLen;

	tel_outer_rect[torindex][0] =-pano_length/16/2;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height*insideUH;
	tel_outer_rect[torindex][0] =-pano_length/16/2+pano_length/insideLen;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height*insideUH;

	tel_outer_rect[torindex][0] =-pano_length/16/2;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height*insideUH;
	tel_outer_rect[torindex][0] =-pano_length/16/2;
	tel_outer_rect[torindex][1] =0;
	tel_outer_rect[torindex++][2] =  pano_height*insideUH-pano_length/insideLen;

	/*******2********/

	//TRACK  g_windowWidth  g_windowHei
	float bei=5.0;
	int  trackindex=0;
	GLfloat track_cross[48][3];
	track_cross[trackindex][0] =-pano_length/288.0*bei;
	track_cross[trackindex][1] =0;
	track_cross[trackindex++][2] = pano_height/2.0;
	track_cross[trackindex][0] =pano_length/288.0*bei;
	track_cross[trackindex][1] =0;
	track_cross[trackindex++][2] = pano_height/2.0;

	track_cross[trackindex][0] =0;
	track_cross[trackindex][1] =0;
	track_cross[trackindex++][2] = pano_height/2-pano_length/250.0*bei;
	track_cross[trackindex][0] =0;
	track_cross[trackindex][1] =0;
	track_cross[trackindex++][2] =  pano_height/2+pano_length/250.0*bei;

	//TRACK

	float inidelta=(p_LineofRuler->Load())/360.0*pano_length;
	  if(inidelta>pano_length*3/4)
	 {
		 inidelta-=pano_length;      //ORI POS IS LEN LEFT OF STANDARD POS ,SO MOVE RIGHT
		for(int i=0;i<MS_COUNT;i++)
		 foresightPos[i].SetxDelta(inidelta);
	 }
	 else
	 {
		 for(int i=0;i<MS_COUNT;i++)
			 foresightPos[i].SetxDelta(inidelta);
	 }
	  float deltaY_core=1.2;
	  float deltaY=11.5-5.7+1.0+10.0;
	  float deltaY1=11.5-5.7+1.0-3.0;
	  float deltaY2=11.5-5.7+1.0+4.0;
	  for(int i=0;i<2;i++)
	  {
	 	  p_ForeSightFacade[i] =new ForeSightFacade(
	 			  new ForeSight_decorator(*(m_env.GetmodelViewMatrix()),*(m_env.GetprojectionMatrix()),&shaderManager, auto_ptr<BaseForeSight>(
	 										  new PseudoForeSight_core()),
	 					  	  	  	  porindex,pano_outer_rect,pano_length*100.0,pano_height/(OUTER_RECT_AND_PANO_TWO_TIMES_CAM_LIMIT))
	 	  	  	  	  	  	  	  	  	  ,foresightPos[i],&panocamonforesight[i]);
		  assert(p_ForeSightFacade[i]);

		  p_ForeSightFacade2[i] =new ForeSightFacade(
		 	 			  new ForeSight_decorator(*(m_env.GetmodelViewMatrix()),*(m_env.GetprojectionMatrix()),&shaderManager, auto_ptr<BaseForeSight>(
		 	 					  new ForeSight_decorator(*(m_env.GetmodelViewMatrix()),*(m_env.GetprojectionMatrix()),&shaderManager,auto_ptr<BaseForeSight>(
		 	 							  new ForeSight_decorator(*(m_env.GetmodelViewMatrix()),*(m_env.GetprojectionMatrix()), &shaderManager,auto_ptr<BaseForeSight>(
		 	 									  new PseudoForeSight_core()),tcindex,tel_cross,pano_length/TELXLIMIT,pano_height/5.7)) ,tirindex,tel_inner_rect,pano_length/TELXLIMIT-((1.0/14.0-1.0/15.75)*pano_length),pano_height/8.0)),
		 	 					  	  	  	  	  torindex,tel_outer_rect,(pano_length/TELXLIMIT-(1/14.0-1/25.0)*pano_length),pano_height/12.0)
		  	  	  	  	  	  	  	  	  	  	  	  ,  foresightPos[i],&telcamonforesight[i]);   //14.0  15.75   25
		 		  assert(p_ForeSightFacade2[i]);

	  }







		 			  p_ForeSightFacade_Track=new ForeSightFacade(
		 					 		 														new ForeSight_decorator(*(m_env.GetmodelViewMatrix()),*(m_env.GetprojectionMatrix()), &shaderManager,auto_ptr<BaseForeSight>(
		 					 		 				 	 	 	 	 	 	 	 	 	 	 	new PseudoForeSight_core()),trackindex,track_cross,g_windowWidth*1434.0/1920.0/2.0*100.0,g_windowHeight/2.0*100.0),
		 					 		 				 	 	 	 	 	 	 	 	 	 	 	foresightPos[0],
		 					 		 				 	 	 	 	 	 	 	 	 	 	 	new PseudoForeSight_cam());
		 			  assert(p_ForeSightFacade_Track);
}

// Trick: put the indivial video on the shadow rect texture
void Render::InitShadow(GLEnv &m_env)
{
	GLuint texture = 0;
	m_env.Getp_shadowBatch()->Begin(GL_TRIANGLE_FAN, 4, 1);
	m_env.Getp_shadowBatch()->MultiTexCoord2f(texture, 0.0f, 0.0f);
	m_env.Getp_shadowBatch()->Vertex3f(-1.0f,-1.0f, 0.0f );
	m_env.Getp_shadowBatch()->MultiTexCoord2f(texture, 0.0f, 1.0f);
	m_env.Getp_shadowBatch()->Vertex3f(-1.0f, 1.0f, 0.0f);
	m_env.Getp_shadowBatch()->MultiTexCoord2f(texture, 1.0f, 1.0f);
	m_env.Getp_shadowBatch()->Vertex3f( 1.0f,1.0f,  0.0f);
	m_env.Getp_shadowBatch()->MultiTexCoord2f(texture, 1.0f, 0.0f);
	m_env.Getp_shadowBatch()->Vertex3f( 1.0f,-1.0f,  0.0f);
	m_env.Getp_shadowBatch()->End();
}

// __   __ 8m
//|         |
//|         |
//|__  __|5m
//|         |
//|__  __|2m
//|         |
//|         |
void Render::InitWheelTracks()
{
	// Load as a bunch of line segments
	GLfloat vTracks[16][3], vTracks2[2][3], vTracks5[2][3], vTracks1[2][3];
	GLfloat fixBBDPos[3];
	GLfloat Track_to_Vehicle_width_rate = DEFAULT_TRACK2_VEHICLE_WIDTH_RATE;
	const GLfloat* pVehicleDimension = pVehicle->GetDimensions();
	const GLfloat* pVehicleYMaxMin = pVehicle->GetYMaxMins();
	GLfloat   TrackLength = DEFAULT_TRACK_LENGTH_METER;
	GLfloat   TrackWidth = Track_to_Vehicle_width_rate*pVehicleDimension[0]; 
	cout<<"Vehicle x = "<<pVehicleDimension[0]<<", y = "<<pVehicleDimension[1]<<", z = "<<pVehicleDimension[2]<<endl;
	int i = 0;
	float line_y_amplify=1.0f;
	vTracks[i][0] = -TrackWidth/2;//-TrackWidth/30;
	vTracks[i][1] =0.0f*line_y_amplify;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = -TrackWidth/2;//-TrackWidth/30;
	vTracks[i][1] = TrackLength*5*line_y_amplify;
	vTracks[i++][2] = 0.0f;
	
	vTracks[i][0] = TrackWidth/2;
	vTracks[i][1] =0.0f*line_y_amplify;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = TrackWidth/2;
	vTracks[i][1] = TrackLength*5*line_y_amplify;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = -TrackWidth/2;//-TrackWidth/30;
	vTracks[i][1] = TrackLength*5*line_y_amplify;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = -TrackWidth/4;
	vTracks[i][1] = TrackLength*5*line_y_amplify;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = TrackWidth/2;
	vTracks[i][1] = TrackLength*5*line_y_amplify;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = TrackWidth/4;
	vTracks[i][1] = TrackLength*5*line_y_amplify;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = -TrackWidth/2;//-TrackWidth/30;
	vTracks[i][1] = TrackLength*5*line_y_amplify;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = TrackWidth/2;
	vTracks[i][1] = TrackLength*5*line_y_amplify;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = -TrackWidth/2;//-TrackWidth/30;
	vTracks[i][1] = TrackLength*2*line_y_amplify;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = TrackWidth/2;
	vTracks[i][1] = TrackLength*2*line_y_amplify;
	vTracks[i++][2] = 0.0f;

	WheelTrackBatch.Begin(GL_LINES, 10);
	WheelTrackBatch.CopyVertexData3f(vTracks);
	WheelTrackBatch.End();

		i = 0;
		vTracks[i][0] = -TrackWidth/2;
		vTracks[i][1] =-TrackLength/4*line_y_amplify;
		vTracks[i++][2] =0.0f;

		vTracks[i][0] = -TrackWidth/2;
		vTracks[i][1] = TrackLength*5*line_y_amplify;
		vTracks[i++][2] = 0.0f;

		vTracks[i][0] = TrackWidth/2;//+TrackWidth/30;
		vTracks[i][1] =-TrackLength/4*line_y_amplify;
		vTracks[i++][2] = 0.0f;

		vTracks[i][0] = TrackWidth/2;//+TrackWidth/30;
		vTracks[i][1] = TrackLength*5*line_y_amplify;
		vTracks[i++][2] = 0.0f;

		vTracks[i][0] = -TrackWidth/2;
		vTracks[i][1] = TrackLength*5*line_y_amplify;
		vTracks[i++][2] = 0.0f;

		vTracks[i][0] = -TrackWidth/4;
		vTracks[i][1] = TrackLength*5*line_y_amplify;
		vTracks[i++][2] = 0.0f;

		vTracks[i][0] = TrackWidth/2;//+TrackWidth/30;
		vTracks[i][1] = TrackLength*5*line_y_amplify;
		vTracks[i++][2] = 0.0f;

		vTracks[i][0] = TrackWidth/4;
		vTracks[i][1] = TrackLength*5*line_y_amplify;
		vTracks[i++][2] = 0.0f;

		vTracks[i][0] = -TrackWidth/2;
		vTracks[i][1] = TrackLength*5*line_y_amplify;
		vTracks[i++][2] = 0.0f;

		vTracks[i][0] = TrackWidth/2;//+TrackWidth/30;
		vTracks[i][1] = TrackLength*5*line_y_amplify;
		vTracks[i++][2] = 0.0f;

		vTracks[i][0] = -TrackWidth/2;
		vTracks[i][1] = TrackLength*2*line_y_amplify;
		vTracks[i++][2] = 0.0f;

		vTracks[i][0] = TrackWidth/2;//+TrackWidth/30;
		vTracks[i][1] = TrackLength*2*line_y_amplify;
		vTracks[i++][2] = 0.0f;

		FrontTrackBatch.Begin(GL_LINES, 10);
		FrontTrackBatch.CopyVertexData3f(vTracks);
		FrontTrackBatch.End();

	i=0;
	vTracks1[i][0] = -TrackWidth/2;//-TrackWidth/30;
	vTracks1[i][1] = TrackLength*1*line_y_amplify;
	vTracks1[i++][2] = 0.0f;

	vTracks1[i][0] = TrackWidth/2;
	vTracks1[i][1] = TrackLength*1*line_y_amplify;
	vTracks1[i++][2] = 0.0f;

	WheelTrackBatch1.Begin(GL_LINES, 2);
	WheelTrackBatch1.CopyVertexData3f(vTracks1);
	WheelTrackBatch1.End();

	i=0;
	vTracks1[i][0] = -TrackWidth/2;
	vTracks1[i][1] = TrackLength*1*line_y_amplify;
	vTracks1[i++][2] = 0.0f;

	vTracks1[i][0] = TrackWidth/2;//+TrackWidth/30;
	vTracks1[i][1] = TrackLength*1*line_y_amplify;
	vTracks1[i++][2] = 0.0f;

	FrontTrackBatch1.Begin(GL_LINES, 2);
	FrontTrackBatch1.CopyVertexData3f(vTracks1);
	FrontTrackBatch1.End();

	memcpy(fixBBDPos, &vTracks1[i-1][0], sizeof(GLfloat)*3);
	fixBBDPos[0]=fixBBDPos[0]+TrackWidth/6;
	fixBBDPos[1] = -fixBBDPos[1] - pVehicleYMaxMin[0]+TrackLength/3;
	p_FixedBBD_1M->SetBackLocation(fixBBDPos);
	fixBBDPos[1] =  fixBBDPos[1]+ pVehicleDimension[1]+ 2*vTracks1[i-1][1]+TrackLength/3;
	p_FixedBBD_1M->SetHeadLocation(fixBBDPos);

	i=0;
	vTracks2[i][0] = -TrackWidth/2;//-TrackWidth/30;
	vTracks2[i][1] = TrackLength*2*line_y_amplify;
	vTracks2[i++][2] = 0.0f;

	vTracks2[i][0] = TrackWidth/2;
	vTracks2[i][1] = TrackLength*2*line_y_amplify;
	vTracks2[i++][2] = 0.0f;

	WheelTrackBatch2.Begin(GL_LINES, 2);
	WheelTrackBatch2.CopyVertexData3f(vTracks2);
	WheelTrackBatch2.End();

	i=0;
	vTracks2[i][0] = -TrackWidth/2;
	vTracks2[i][1] = TrackLength*2*line_y_amplify;
	vTracks2[i++][2] = 0.0f;

	vTracks2[i][0] = TrackWidth/2;//+TrackWidth/30;
	vTracks2[i][1] = TrackLength*2*line_y_amplify;
	vTracks2[i++][2] = 0.0f;

	FrontTrackBatch2.Begin(GL_LINES, 2);
	FrontTrackBatch2.CopyVertexData3f(vTracks2);
	FrontTrackBatch2.End();

	memcpy(fixBBDPos, &vTracks2[i-1][0], sizeof(GLfloat)*3);
	fixBBDPos[0]=fixBBDPos[0]+TrackWidth/6;
	fixBBDPos[1] = -fixBBDPos[1] - pVehicleYMaxMin[0]+TrackLength/3;
	p_FixedBBD_2M->SetBackLocation(fixBBDPos);
	fixBBDPos[1] =  fixBBDPos[1]+ pVehicleDimension[1]+ 2*vTracks2[i-1][1]+TrackLength/3;
	p_FixedBBD_2M->SetHeadLocation(fixBBDPos);

	i=0;
	vTracks5[i][0] = -TrackWidth/2;//-TrackWidth/30;
	vTracks5[i][1] = TrackLength*5*line_y_amplify;
	vTracks5[i++][2] = 0.0f;

	vTracks5[i][0] = TrackWidth/2;
	vTracks5[i][1] = TrackLength*5*line_y_amplify;
	vTracks5[i++][2] = 0.0f;
	WheelTrackBatch5.Begin(GL_LINES, 2);
	WheelTrackBatch5.CopyVertexData3f(vTracks5);
	WheelTrackBatch5.End();

	i=0;
	vTracks5[i][0] = -TrackWidth/2;
	vTracks5[i][1] = TrackLength*5*line_y_amplify;
	vTracks5[i++][2] = 0.0f;

	vTracks5[i][0] = TrackWidth/2;//+TrackWidth/30;
	vTracks5[i][1] = TrackLength*5*line_y_amplify;
	vTracks5[i++][2] = 0.0f;
	FrontTrackBatch5.Begin(GL_LINES, 2);
	FrontTrackBatch5.CopyVertexData3f(vTracks5);
	FrontTrackBatch5.End();

	memcpy(fixBBDPos, &vTracks5[i-1][0], sizeof(GLfloat)*3);
	fixBBDPos[0]=fixBBDPos[0]+TrackWidth/6;
	fixBBDPos[1] = -fixBBDPos[1] - pVehicleYMaxMin[0]+TrackLength/6;
	p_FixedBBD_5M->SetBackLocation(fixBBDPos);
	fixBBDPos[1] =  fixBBDPos[1]+ pVehicleDimension[1]+ 2*vTracks5[i-1][1]+TrackLength/6;
	p_FixedBBD_5M->SetHeadLocation(fixBBDPos);

}

void Render::InitFrontTracks()
{
	GLfloat vTracks[8][3];
	GLfloat Track_to_Vehicle_width_rate = DEFAULT_TRACK2_VEHICLE_WIDTH_RATE;
	const GLfloat* pVehicleDimension = pVehicle->GetDimensions();
	GLfloat   TrackLength = DEFAULT_FRONT_TRACK_LENGTH_METER;
	GLfloat   TrackWidth = Track_to_Vehicle_width_rate*pVehicleDimension[0]; // workaround, the car would rotate 90. so the y is actually its width
	int i = 0;
	vTracks[i][0] = -TrackWidth/2;
	vTracks[i][1] = TrackLength*DEFAULT_VEHICLE_TRANSLATION_1;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = -TrackWidth/2;
	vTracks[i][1] = -TrackLength;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = TrackWidth/2;
	vTracks[i][1] = TrackLength*DEFAULT_VEHICLE_TRANSLATION_1;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = TrackWidth/2;
	vTracks[i][1] = -TrackLength;
	vTracks[i++][2] = 0.0f;

	FrontTrackBatch.Begin(GL_LINES, 4);
	FrontTrackBatch.CopyVertexData3f(vTracks);
	FrontTrackBatch.End();

	vTracks[i][0] = -TrackWidth/2;
	vTracks[i][1] = -TrackLength/3;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = TrackWidth/2;
	vTracks[i][1] = -TrackLength/3;
	vTracks[i++][2] = 0.0f;

	FrontTrackBatch1.Begin(GL_LINES, 2);
	FrontTrackBatch1.CopyVertexData3f(&vTracks[4]);
	FrontTrackBatch1.End();

	vTracks[i][0] = -TrackWidth/2;
	vTracks[i][1] = -TrackLength;
	vTracks[i++][2] = 0.0f;

	vTracks[i][0] = TrackWidth/2;
	vTracks[i][1] = -TrackLength;
	vTracks[i++][2] = 0.0f;

	FrontTrackBatch3.Begin(GL_LINES, 2);
	FrontTrackBatch3.CopyVertexData3f(&vTracks[6]);
	FrontTrackBatch3.End();
}

void Render::InitCrossLines()
{
	GLfloat vTracks[CAM_COUNT*2][3];
	int i = 0;
	for(i=0; i<CAM_COUNT; i++)
	{
		vTracks[i*2][0] = bar[i].x; //0.0f;
		vTracks[i*2][1] = bar[i].y; //0.0f;
		vTracks[i*2][2] = bar[i].z; //0.0f;

		vTracks[i*2+1][0] = bar[i+CAM_COUNT].x; //0.0f;
		vTracks[i*2+1][1] = bar[i+CAM_COUNT].y; //0.0f;
		vTracks[i*2+1][2] = bar[i+CAM_COUNT].z; //0.0f;
	}

	CrossLinesBatch.Begin(GL_LINES, CAM_COUNT*2);
	CrossLinesBatch.CopyVertexData3f(vTracks);
	CrossLinesBatch.End();
}
void Render::InitWealTrack()
{
	GLfloat vTracksH[4][3], vTracksR[16][3];
	GLfloat fixBBDPos[3];
	int i=0;
	//1M
	vTracksR[i][0] = -0.50f;
	vTracksR[i][1] = -0.40;
	vTracksR[i++][2] = 0.0f;

    memcpy(fixBBDPos, &vTracksR[i-1][0], sizeof(GLfloat)*3);
    p_FixedBBD_1M->SetFishEyeLocation(fixBBDPos);

	vTracksR[i][0] = 0.53f;
	vTracksR[i][1] = -0.40;
	vTracksR[i++][2] = 0.0f;
	
	WheelTrackBatchRear1.Begin(GL_LINES, 2);
	WheelTrackBatchRear1.CopyVertexData3f(&vTracksR[0]);
	WheelTrackBatchRear1.End();
//2M
	vTracksR[i][0] = -0.41f;
	vTracksR[i][1] = 0.07;
	vTracksR[i++][2] = 0.0f;

    memcpy(fixBBDPos, &vTracksR[i-1][0], sizeof(GLfloat)*3);

    p_FixedBBD_2M->SetFishEyeLocation(fixBBDPos);

	vTracksR[i][0] = 0.40f;
	vTracksR[i][1] = 0.07;
	vTracksR[i++][2] = 0.0f;

	WheelTrackBatchRear2.Begin(GL_LINES, 2);
	WheelTrackBatchRear2.CopyVertexData3f(&vTracksR[2]);
	WheelTrackBatchRear2.End();
//5M
	vTracksR[i][0] = -0.22f;
	vTracksR[i][1] = 0.58f;
	vTracksR[i++][2] = 0.0f;
    memcpy(fixBBDPos, &vTracksR[i-1][0], sizeof(GLfloat)*3);
    p_FixedBBD_5M->SetFishEyeLocation(fixBBDPos);

	vTracksR[i][0] = 0.27f;
	vTracksR[i][1] = 0.58f;
	vTracksR[i++][2] = 0.0f;
	
	WheelTrackBatchRear5.Begin(GL_LINES, 2);
	WheelTrackBatchRear5.CopyVertexData3f(&vTracksR[4]);
	WheelTrackBatchRear5.End();
// Side lines
	vTracksR[i][0] = -0.23f;
	vTracksR[i][1] = 0.58f;
	vTracksR[i++][2] = 0.0f;

	vTracksR[i][0] = -0.61f;
	vTracksR[i][1] = -0.58f;
	vTracksR[i++][2] = 0.0f;

	vTracksR[i][0] = 0.27f;
	vTracksR[i][1] = 0.60f;
	vTracksR[i++][2] = 0.0f;

	vTracksR[i][0] = 0.62f;
	vTracksR[i][1] = -0.60f;
	vTracksR[i++][2] = 0.0f;
	WheelTrackBatchRear.Begin(GL_LINES, 4);
	WheelTrackBatchRear.CopyVertexData3f(&vTracksR[6]);
	WheelTrackBatchRear.End();

	i = 0;
	vTracksH[i][0] = -0.32f;
	vTracksH[i][1] = 0.31f;
	vTracksH[i++][2] = 0.0f;

	vTracksH[i][0] = -0.75f;
	vTracksH[i][1] = -0.83f;
	vTracksH[i++][2] = 0.0f;

	vTracksH[i][0] = 0.35f;
	vTracksH[i][1] = 0.27f;
	vTracksH[i++][2] = 0.0f;

	vTracksH[i][0] = 0.83f;
	vTracksH[i][1] = -0.83f;
	vTracksH[i++][2] = 0.0f;

	WheelTrackBatchHead.Begin(GL_LINES, 4);
	WheelTrackBatchHead.CopyVertexData3f(vTracksH);
	WheelTrackBatchHead.End();
}

void Render::InitDynamicTrack(GLEnv &m_env)
{
	p_DynamicTrack = new DynamicTrack(*(m_env.GetmodelViewMatrix()), *(m_env.GetprojectionMatrix()), &shaderManager);
	assert(p_DynamicTrack);
}

void Render::InitLineofRuler(GLEnv &m_env)
{
	p_LineofRuler=new Calibrate(	*(m_env.GetmodelViewMatrix()), *(m_env.GetprojectionMatrix()), &shaderManager);
	assert(p_LineofRuler);
	float the_angle=0.0;
	the_angle=p_LineofRuler->Load();
	p_LineofRuler->SetAngle(the_angle);
	setrulerreferenceangle(the_angle);
}

void Render::InitCornerMarkerGroup(GLEnv &m_env)
{
	p_CornerMarkerGroup = new CornerMarkerGroup(*(m_env.GetmodelViewMatrix()), *(m_env.GetprojectionMatrix()), &shaderManager);
	assert(p_DynamicTrack);
}
void Render::InitOitVehicle(GLEnv &m_env)
{
	pVehicle = new OitVehicle(*(m_env.GetmodelViewMatrix()), *(m_env.GetprojectionMatrix()), &shaderManager);
	if(pVehicle)
	{
		pVehicle->SetLoader(VehicleLoader);
		pVehicle->InitVehicle();
		pVehicle->InitShaders();
	}
	else
	{
		cerr<<"OitVehicle Failed"<<endl;
	}
}

void Render::InitAlarmAeraonPano(GLEnv &m_env)
{
	pPano = new OitVehicle(*(m_env.GetmodelViewMatrix()), *(m_env.GetprojectionMatrix()), &shaderManager);
	if(pPano)
	{
		pPano->SetLoader(VehicleLoader);
		pPano->InitVehicle();
		pPano->InitShaders();
	}
	else
	{
		cerr<<"Alarm aera on pano Failed"<<endl;
	}
}

void  Render::InitBillBoard(GLEnv &m_env)
{
    p_BillBoard = new BillBoard(*(m_env.GetmodelViewMatrix()), *(m_env.GetprojectionMatrix()), &shaderManager);
    if(p_BillBoard)
    {
    	p_BillBoard->Init();
    }
    else
    {
	cerr<<"BillBoard failed"<<endl;
    }
   	p_ChineseCBillBoard=new ChineseCharacterBillBoard(*(m_env.GetmodelViewMatrix()), *(m_env.GetprojectionMatrix()), &shaderManager);
	   if(p_ChineseCBillBoard)
	    {
		   p_ChineseCBillBoard->Init();
	    }
	    else
	    {
		cerr<<"p_ChineseCBillBoard failed"<<endl;
	    }
	   p_ChineseCBillBoard_bottem_pos=new ChineseCharacterBillBoard(*(m_env.GetmodelViewMatrix()), *(m_env.GetprojectionMatrix()), &shaderManager);

	   if(p_ChineseCBillBoard_bottem_pos)
	  	    {
		   p_ChineseCBillBoard_bottem_pos->Init(CBB_X,CBB_Y,CBB_WIDTH,CBB_HEIGHT);
	  	    }
	  	    else
	  	    {
	  		cerr<<"p_ChineseCBillBoard_bottem_pos failed"<<endl;
	  	    }

    p_CompassBillBoard = new CompassBillBoard(*(m_env.GetmodelViewMatrix()), *(m_env.GetprojectionMatrix()),  &shaderManager);
    if(p_CompassBillBoard)
    {
    	p_CompassBillBoard->Init();
    }
    else
    {
	cerr<<"CompassBillBoard failed"<<endl;
    }

    p_BillBoardExt = new ExtBillBoard(*(m_env.GetmodelViewMatrix()), *(m_env.GetprojectionMatrix()),  &shaderManager);
    if(p_BillBoardExt)
    {
    	p_BillBoardExt->Init();
    }
    else
    {
    	cerr<<"BillBoard Ext failed"<<endl;
    }
    p_FixedBBD_2M = new FixedBillBoard(DDS_FILE_2M,*(m_env.GetmodelViewMatrix()), *(m_env.GetprojectionMatrix()),  &shaderManager);

     if(NULL == p_FixedBBD_2M){
	   	cerr<<"Failed to load" <<DDS_FILE_2M<<endl;
    }	
    p_FixedBBD_5M = new FixedBillBoard(DDS_FILE_5M,*(m_env.GetmodelViewMatrix()), *(m_env.GetprojectionMatrix()),  &shaderManager);

	if(NULL == p_FixedBBD_5M){
		cerr<<"Failed to load" <<DDS_FILE_5M<<endl;
	}
    p_FixedBBD_8M = new FixedBillBoard(DDS_FILE_8M,*(m_env.GetmodelViewMatrix()), *(m_env.GetprojectionMatrix()), &shaderManager);
	if(NULL == p_FixedBBD_8M){
		cerr<<"Failed to load" <<DDS_FILE_8M<<endl;
	}
	p_FixedBBD_1M = new FixedBillBoard(DDS_FILE_1M,*(m_env.GetmodelViewMatrix()), *(m_env.GetprojectionMatrix()), &shaderManager);
	if(NULL == p_FixedBBD_1M){
		cerr<<"Failed to load" <<DDS_FILE_1M<<endl;
	}
}
void Render::DrawOitVehicle(GLEnv &m_env)
{
	pVehicle->DrawVehicle(m_env);
	//pVehicle->DrawVehicle_second();
	//pVehicle->DrawVehicle_third();
}

void Render::DrawVehiclesEtcWithFixedBBD(GLEnv &m_env,M3DMatrix44f camera )
{
	//DrawVehiclesEtc(m_env,camera);
	
}
void Render::DrawVehiclesEtc(GLEnv &m_env,M3DMatrix44f camera)
{
	m_env.GetmodelViewMatrix()->PushMatrix();
//	m_env.GetmodelViewMatrix()->Rotate(-180.0f, 0.0f, 0.0f, 1.0f);
	DrawShadow(m_env);
	DrawOitVehicle(m_env);
	DrawFrontBackTracks(m_env);

	m_env.GetmodelViewMatrix()->PopMatrix();
	if(camera)
		DrawTrackFixBBDs(m_env,camera);
	glFlush();
}

void Render::DrawTrackFixBBDs(GLEnv &m_env,M3DMatrix44f camera)
{
	static FixedBillBoard *pBBD[3] = {p_FixedBBD_2M,p_FixedBBD_5M,p_FixedBBD_1M };
	FixedBillBoard::DrawGroup(m_env,camera, pBBD, 3);
}
void Render::RenderBirdView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h, bool needSendData)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(35.0f, /*1.0/10*/float(w) / float(h), 1.0f, 500.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();

	M3DMatrix44f mCamera;
	birdViewCameraFrame.GetCameraMatrix(mCamera);
	m_env.GetmodelViewMatrix()->PushMatrix(mCamera);

	m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
	m_env.GetmodelViewMatrix()->Rotate(p_LineofRuler->GetAngle(),0.0,0.0,1.0);
	DrawBowl(m_env,needSendData);
	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->Translate(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x(),0.0,0.0);
	DrawBowl(m_env,false);
	m_env.GetmodelViewMatrix()->PopMatrix();
	m_env.GetmodelViewMatrix()->PopMatrix();

	if(BLEND_OFFSET == 0)
		Draw4CrossLines(m_env);
	DrawVehiclesEtcWithFixedBBD(m_env,mCamera);
	UpdateWheelAngle();
	drawDynamicTracks(m_env);
	m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix

	m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::RenderAnimationToBirdView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h, bool needSendData)
{
 	static enum {
		ANIM_ROTATE,
		ANIM_RISE,
		ANIM_END
 	} animation_state = ANIM_ROTATE;

 	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(45.0f, float(w) / float(h), 1.0f, 500.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();
	M3DMatrix44f mCamera;
	frontCameraFrame.GetCameraMatrix(mCamera);	

	switch (animation_state){

	case ANIM_ROTATE:	//bowl rotate
		{
			static CStopWatch animRotTimer;
			static const unsigned rotation_seconds = 7;
			static const float	rotation_angle = 360.0f;
			float zRot = rotTimer.GetElapsedSeconds()*(rotation_angle/rotation_seconds );
			m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
			if(zRot >= rotation_angle){
				zRot = rotation_angle;
				animation_state = ANIM_RISE;
			}
			m_env.GetmodelViewMatrix()->Rotate(zRot, 0.0f, 0.0f, 1.0f);
		}
		break;
	case ANIM_RISE:	// camera animation front to birdview
		{
			m_env.GetviewFrustum()->SetPerspective(35.0f, float(w) / float(h), 1.0f, 500.0f);
			m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());
			static const unsigned int DEFAULT_RISE_COUNT=14;
			static unsigned int riseCount = DEFAULT_RISE_COUNT;
			M3DMatrix44f nextCam;
			if(riseCount-- >0){
				birdViewCameraFrame.GetCameraMatrix(nextCam);
				for(int i = 0; i< 16; i++){
				   mCamera[i] = (DEFAULT_RISE_COUNT-riseCount)*
				 	(nextCam[i]-mCamera[i])/DEFAULT_RISE_COUNT +mCamera[i];
				}
			}else{
				birdViewCameraFrame.GetCameraMatrix(mCamera);
				animation_state = ANIM_END;
			}
			m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
		}
		break;
	case ANIM_END:
	default:
		{
			m_env.GetviewFrustum()->SetPerspective(35.0f, float(w) / float(h), 1.0f, 500.0f);
			m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());
			birdViewCameraFrame.GetCameraMatrix(mCamera);
			m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
		}
		break;

	}

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->Rotate(p_LineofRuler->GetAngle(),0.0,0.0,1.0);
	DrawBowl(m_env,needSendData);
	m_env.GetmodelViewMatrix()->PopMatrix();

	if(BLEND_OFFSET == 0)
		Draw4CrossLines(m_env);
	DrawVehiclesEtcWithFixedBBD(m_env,mCamera);
	UpdateWheelAngle();
	drawDynamicTracks(m_env);
	m_env.GetmodelViewMatrix()->PopMatrix(); //pop front camera

	m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::RenderFreeView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h, bool needSendData)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(35.0f, float(w) / float(h), 1.0f, 500.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();

	M3DMatrix44f mCamera;
	m_freeCamera.GetCameraMatrix(mCamera);
	m_env.GetmodelViewMatrix()->PushMatrix(mCamera);

	m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
	m_env.GetmodelViewMatrix()->Rotate(p_LineofRuler->GetAngle(),0.0,0.0,1.0);
	DrawBowl(m_env,needSendData);
	m_env.GetmodelViewMatrix()->PopMatrix();


	DrawVehiclesEtcWithFixedBBD(m_env,mCamera);
	UpdateWheelAngle();
	drawDynamicTracks(m_env);
	m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix

	m_env.GetmodelViewMatrix()->PopMatrix();
}
void Render::RenderRearTopView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,bool needSendData)
{
	RenderPreSetView(m_env,x,y, w,h,needSendData, true);
}
void Render::RenderPreSetView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,bool needSendData, bool isRearTop)
{
	M3DMatrix44f mCamera;
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(35.0f, float(w) / float(h), 1.0f, 500.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();
	if(isRearTop){
		rearTopCameraFrame.GetCameraMatrix(mCamera);
	}
	else{
	    GLFrame currentCamera=mPresetCamGroup.GetCameraFrame(p_BillBoard->m_Direction);
	    currentCamera.GetCameraMatrix(mCamera);
	}
	m_env.GetmodelViewMatrix()->PushMatrix(mCamera);

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->Rotate(p_LineofRuler->GetAngle(),0.0,0.0,1.0);
	DrawBowl(m_env,needSendData);
	m_env.GetmodelViewMatrix()->PopMatrix();

	DrawVehiclesEtcWithFixedBBD(m_env,mCamera);
	UpdateWheelAngle();
	drawDynamicTracks(m_env);
	m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
	m_env.GetmodelViewMatrix()->PopMatrix();
}
void Render::RenderPresetViewByRotating(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,bool needSendData)
{
			glViewport(x,y,w,h);
			m_env.GetviewFrustum()->SetPerspective(35.0f, float(w) / float(h), 1.0f, 500.0f);
			m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

			m_env.GetmodelViewMatrix()->PushMatrix();
			M3DMatrix44f prevCam, nextCam;
			GLFrame nextCamera;
			GLFrame prevCamera;
			nextCamera=mPresetCamGroup.GetCameraFrame(p_BillBoard->m_Direction);
			nextCamera.GetCameraMatrix(nextCam);

			if(m_presetCameraRotateCounter-- >0){
					prevCamera= mPresetCamGroup.GetCameraFrame((p_BillBoard->m_lastDirection));
				prevCamera.GetCameraMatrix(prevCam);
				for(int i = 0; i< 16; i++){
				   nextCam[i] = (PRESET_CAMERA_ROTATE_MAX-m_presetCameraRotateCounter)*
				 	(nextCam[i]-prevCam[i])/PRESET_CAMERA_ROTATE_MAX +prevCam[i];
				}
			}
			else{
				m_presetCameraRotateCounter = 0;
			}
			m_env.GetmodelViewMatrix()->PushMatrix(nextCam);

			m_env.GetmodelViewMatrix()->PushMatrix();
			m_env.GetmodelViewMatrix()->Rotate(p_LineofRuler->GetAngle(),0.0,0.0,1.0);
			DrawBowl(m_env,needSendData);
			m_env.GetmodelViewMatrix()->PopMatrix();

			DrawVehiclesEtcWithFixedBBD(m_env,nextCam);
			m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix

			m_env.GetmodelViewMatrix()->PopMatrix();

}
// camera look down from vehicle front
void Render::RenderRotatingView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,bool needSendData)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(35.0f, float(w) / float(h), 1.0f, 500.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();
	M3DMatrix44f mCamera;
	frontCameraFrame.GetCameraMatrix(mCamera);
	m_env.GetmodelViewMatrix()->PushMatrix(mCamera);

	
	static float zRotFirst = 0.0f;
	float zRot = rotTimer.GetElapsedSeconds()*15.0f;

	//alway let the rotation start from 0 degree
	if(bRotTimerStart){
		rotTimer.Reset();
		zRot = rotTimer.GetElapsedSeconds()*15.0f;
		zRotFirst = zRot+180.0f;
		bRotTimerStart = false;
	}

	zRot -= zRotFirst;

	m_env.GetmodelViewMatrix()->Rotate(zRot, 0.0f, 0.0f, 1.0f);
	DrawBowl(m_env,needSendData);
	DrawVehiclesEtcWithFixedBBD(m_env,mCamera);
	m_env.GetmodelViewMatrix()->PopMatrix(); //pop front camera

	m_env.GetmodelViewMatrix()->PopMatrix();
}
void Render::RenderSingleView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,int mainOrsub)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(35.0f, float(w) / float(h), 1.0f, 500.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

//	pVehicle->PrepareBlendMode();
	m_env.GetmodelViewMatrix()->PushMatrix();
	//	glDisable(GL_CULL_FACE);
	if(bControlViewCamera){
		M3DMatrix44f mCamera;
		birdViewCameraFrame.GetCameraMatrix(mCamera);
		m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
	}
	else{
		m_env.GetmodelViewMatrix()->LoadIdentity();
		m_env.GetmodelViewMatrix()->Translate(PANx, PANy, BowlLoader.GetZ_Depth()+scale);
		m_env.GetmodelViewMatrix()->Rotate(ROTx, 0.0f, 0.0f, 1.0f);
		m_env.GetmodelViewMatrix()->Rotate(ROTy, 1.0f, 0.0f, 0.0f);
		m_env.GetmodelViewMatrix()->Rotate(ROTz, 0.0f, 0.1f, 1.0f);
	}

	//m_env.GetmodelViewMatrix()->PushMatrix();
													//	m_env.GetmodelViewMatrix()->Rotate(p_LineofRuler->GetAngle(),0.0,0.0,1.0);
//	DrawBowl(true);
	//m_env.GetmodelViewMatrix()->PopMatrix();


	int array[10]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
	for(int i=1;i<2;i++)
		{
	//	array[i]=i;
		}
	array[0]=0;
	array[3]=3;
	array[2]=2;
	DrawPanel(m_env,true,NULL,mainOrsub);
		m_env.GetmodelViewMatrix()->PushMatrix();
		if(RulerAngle<180.0)
		{
			m_env.GetmodelViewMatrix()->Translate((PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x()),0.0,0.0);
		}
		else
		{
			m_env.GetmodelViewMatrix()->Translate(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x()),0.0,0.0);
		}
	//	DrawPanel(false,NULL);
		m_env.GetmodelViewMatrix()->PopMatrix();




/*
	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->Translate(BowlLoader.Getextent_pos_x()-BowlLoader.Getextent_neg_x(),0.0,0.0);
	DrawBowl(false);
	m_env.GetmodelViewMatrix()->PopMatrix();
*/
//	DrawVehiclesEtc();
//	UpdateWheelAngle();
//	drawDynamicTracks();
	if(bControlViewCamera){
		m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
	}
//	p_ForeSightFacade_Track->SetAlign(1,TRACK_VGA);
//	p_ForeSightFacade_Track->Draw(0);

	m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::RenderCenterView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(27.0f, float(w) / float(h), 1.0f, 1000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

//	pVehicle->PrepareBlendMode();
	m_env.GetmodelViewMatrix()->PushMatrix();
	//rotate camera view
	float rotate_speed=rotateangle_per_second;
	static float last_rotate_angle=0.0f;
	static bool last_stop_order=false;
	float dec_angle=0.0f;

	float rotate_angle=0;//
	if(stopcenterviewrotate)
	{
		rotate_angle=last_rotate_angle;
	}
	else
	{
		if(last_stop_order!=stopcenterviewrotate)
		{
			dec_angle=last_rotate_angle;
		}
		rotate_angle=rotTimer.GetElapsedSeconds()*rotate_speed-dec_angle;
		last_rotate_angle=rotate_angle;
	}
	last_stop_order=stopcenterviewrotate;

	//get center camera matrix apply to the modelviewmatrix
{
		M3DMatrix44f mCamera;
		CenterViewCameraFrame.GetCameraMatrix(mCamera);
		m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
	}

	m_env.GetmodelViewMatrix()->Rotate(rotate_angle, 0.0f, 0.0f, 1.0f);

	DrawBowl(m_env,true);

//	DrawVehiclesEtc();
//	UpdateWheelAngle();
//	drawDynamicTracks();
	{
		m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
	}
	m_env.GetmodelViewMatrix()->PopMatrix();


}

void Render::RenderRegionPanelView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,int mainOrsub)
{
	static float last_scan_distance=0;
	glViewport(x,y,w,h);
//	m_env.GetviewFrustum()->SetPerspective(27.0f, float(w) / float(h), 1.0f, 100.0f);
	m_env.GetviewFrustum()->SetPerspective(27.0f,  float(w) / float(h), 1.0f, 500.0f);

	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	pVehicle->PrepareBlendMode();
	m_env.GetmodelViewMatrix()->PushMatrix();

	//get center camera matrix apply to the modelviewmatrix
{
		M3DMatrix44f mCamera;
		float get_second=(rotTimer.GetElapsedSeconds());
		int now_scan_angle=0;
		float now_scan_distance=0;
		float now_region_forward_distance=0;
		if(!stop_scan)
		{
			now_scan_angle=get_second*100;
			now_scan_angle=now_scan_angle%3600;
			now_scan_distance=(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())*now_scan_angle/3600;
			PanelLoader.SetScan_pos(now_scan_distance);
		}
		else
		{
			now_scan_distance=PanelLoader.GetScan_pos();
		}
			ScanPanelViewCameraFrame.MoveRight(last_scan_distance);
			ScanPanelViewCameraFrame.MoveRight(now_scan_distance*-1.0);
			last_scan_distance=now_scan_distance;

			ScanPanelViewCameraFrame.MoveForward(0.0-getlastregionforwarddistance());


			now_scan_angle=getScanRegionAngle();
			if(now_scan_angle>90.0)
			{
				now_region_forward_distance=-(10+10.85*(now_scan_angle-90.0)/90.0);
			}
			else
			{
				now_region_forward_distance=-10.0;
			}
			setlastregionforwarddistance(now_region_forward_distance);
			ScanPanelViewCameraFrame.MoveForward(now_region_forward_distance);

			ScanPanelViewCameraFrame.GetCameraMatrix(mCamera);
			m_env.GetmodelViewMatrix()->PushMatrix(mCamera);

	}
float math_data=0.0;
math_data=1.0-0.17*(180.0-getScanRegionAngle())/45.0;

m_env.GetmodelViewMatrix()->Scale(1.0,1.0,1.0);//*SCAN_REGION_ANGLE/60.0f);
m_env.GetmodelViewMatrix()->Translate(0.0,0.0,0.0);


	DrawPanel(m_env,false,NULL,mainOrsub);

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->Translate(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x(),0.0,0.0);
	DrawPanel(m_env,false,NULL,mainOrsub);
	m_env.GetmodelViewMatrix()->PopMatrix();


	{
		m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
	}

	m_env.GetmodelViewMatrix()->PopMatrix();
	m_env.GetmodelViewMatrix()->PushMatrix();

	m_env.GetmodelViewMatrix()->Rotate(90.0,0.0,1.0,0.0);
//	p_CompassBillBoard->DrawBillBoard(200,200);

	m_env.GetmodelViewMatrix()->PopMatrix();
//	RenderCompassView(x,y,w,h);
}
/*
void Render::RenderCompassView(GLint x, GLint y, GLint w, GLint h)
{
	static float last_scan_distance=0;
	glViewport(x,y,w,h);
//	m_env.GetviewFrustum()->SetPerspective(27.0f, float(w) / float(h), 1.0f, 100.0f);
	m_env.GetviewFrustum()->SetPerspective(27.0f,  float(w) / float(h), 1.0f, 500.0f);

	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	pVehicle->PrepareBlendMode();
	m_env.GetmodelViewMatrix()->PushMatrix();

	//get center camera matrix apply to the modelviewmatrix
{
		M3DMatrix44f mCamera;
		float get_second=(rotTimer.GetElapsedSeconds());

			CompassCameraFrame.RotateLocalZ(-PI/2);
			p_CompassBillBoard->DrawBillBoard(200,200);

			CompassCameraFrame.GetCameraMatrix(mCamera);

			m_env.GetmodelViewMatrix()->PushMatrix(mCamera);

	}


	{
		m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
	}

	m_env.GetmodelViewMatrix()->PopMatrix();


}
*/

void Render::RenderTrackForeSightView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(20.6f, float(w) / float(h), 1.0f, 100.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());
	//pVehicle->PrepareBlendMode();
	m_env.GetmodelViewMatrix()->PushMatrix();
{
		M3DMatrix44f mCamera;
		TrackCameraFrame.GetCameraMatrix(mCamera);
		m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
}
	//m_env.GetmodelViewMatrix()->Scale(8*TEL_XSCALE,1.0,5.8*TEL_YSCALE);
		p_ForeSightFacade_Track->SetAlign(1,7);
	 	p_ForeSightFacade_Track->Draw(m_env,0);
{
	{
		m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
	}
	m_env.GetmodelViewMatrix()->PopMatrix();
}
}

void Render::RenderPanoTelView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,int direction,int mainOrsub)
{
	int petal1[CAM_COUNT];
	memset(petal1,-1,sizeof(petal1));
	int petal2[CAM_COUNT];
	memset(petal2,-1,sizeof(petal2));
	int petaltest[12]={0,1,2,-1,-1,-1,-1,-1,-1,-1,-1,-1};
	int petal3[CAM_COUNT];
		memset(petal3,-1,sizeof(petal3));
		int petal4[CAM_COUNT];
		memset(petal4,-1,sizeof(petal4));
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(20.6f, float(w) / float(h), 1.0f, 100.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());
//	pVehicle->PrepareBlendMode();
	m_env.GetmodelViewMatrix()->PushMatrix();
	float delta_dis=0.0;
	int i=0;
{
		M3DMatrix44f mCamera;
		repositioncamera();
			PanoTelViewCameraFrame[mainOrsub].GetCameraMatrix(mCamera);
		m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
}
if(direction==LEFT)
{
	m_env.GetmodelViewMatrix()->Scale(8*TEL_XSCALE,1.0,5.8*TEL_YSCALE);
	m_env.GetmodelViewMatrix()->Translate(37.68+TEL_XTRAS+0.5,0.0,-2.7*TEL_YTRAS);
}
else if(direction==BACK)
{
	m_env.GetmodelViewMatrix()->Scale(8*TEL_XSCALE,1.0,5.8*TEL_YSCALE);
	m_env.GetmodelViewMatrix()->Translate(47.1+13+0.3,0.0,-2.7*TEL_YTRAS);
}
else if(direction==FRONT)
{
	m_env.GetmodelViewMatrix()->Scale(8*TEL_XSCALE,1.0,5.8*TEL_YSCALE);
	m_env.GetmodelViewMatrix()->Translate(28.26+13.34,0.0,-2.7*TEL_YTRAS);
}
else if(direction==RIGHT)
{
	m_env.GetmodelViewMatrix()->Scale(8*TEL_XSCALE,1.0,5.8*TEL_YSCALE);
	m_env.GetmodelViewMatrix()->Translate(15+17+0.2,0.0,-2.7*TEL_YTRAS);
}
			float Angle=RulerAngle;
			if(direction==FRONT)
			{
				 i=0;
				 for(int j=1;j<=3;j++)
					 petal3[j]=j;
			}
			else if(direction==RIGHT)
			{
				i=3;
					 petal3[1]=1;
					 petal3[0]=0;
					 petal3[9]=9;
					 petal4[9]=9;
					 petal4[8]=8;
			}
			else if(direction==BACK)
			{
				 petal3[8]=8;
				 petal3[7]=7;
				 petal3[6]=6;
			}
			else if(direction==LEFT)
			{
				i=9;
				 petal3[6]=6;
				 petal3[5]=5;
				 petal3[4]=4;
				 petal3[3]=3;
			}
			/*
			int Cam_num[12]={3,2,1,0,11,10,9,8,7,6,5,4};
			if((Angle<15.0)||(Angle>=345.0))
			{
				Telscenter_cam[mainOrsub]=0;
			}
			else
			{
				Teltemp_math[mainOrsub]=(Angle-15.0)/30.0;
				Telscenter_cam[mainOrsub]=(int)Teltemp_math[mainOrsub];
				Telscenter_cam[mainOrsub]++;
			}
			Telscenter_cam[mainOrsub]+=i;
			while(Telscenter_cam[mainOrsub]>11)
			{
				Telscenter_cam[mainOrsub]-=12;
			}
				if(Cam_num[Telscenter_cam[mainOrsub]]+1==12)
				{
					petal1[1]=1;
					petal1[0]=0;
					petal1[11]=11;
					petal1[10]=10;
					petal1[9]=9;

					petal2[1]=1;
					petal2[0]=0;
					petal2[11]=11;
					petal2[10]=10;
					petal2[9]=9;
				}
				else if(Cam_num[Telscenter_cam[mainOrsub]]-1==-1)
				{
					petal1[0]=0;
					petal1[1]=1;
					petal1[2]=2;
					petal1[11]=11;
					petal1[10]=10;
					petal2[0]=0;
					petal2[1]=1;
					petal2[2]=2;
					petal2[11]=11;
					petal2[10]=10;
				}
				else if(Cam_num[Telscenter_cam[mainOrsub]]==10)
				{
					petal1[0]=0;
					petal1[9]=9;
					petal1[8]=8;
					petal1[11]=11;
					petal1[10]=10;
					petal2[0]=0;
					petal2[9]=9;
					petal2[8]=8;
					petal2[11]=11;
					petal2[10]=10;
				}
				else if(Cam_num[Telscenter_cam[mainOrsub]]==1)
				{
					petal1[0]=0;
					petal1[1]=1;
					petal1[2]=2;
					petal1[11]=11;
					petal1[3]=3;
					petal2[0]=0;
					petal2[1]=1;
					petal2[2]=2;
					petal2[11]=11;
					petal2[3]=3;
				}
				else
				{
					petal1[Cam_num[Telscenter_cam[mainOrsub]]+2]=Cam_num[Telscenter_cam[mainOrsub]]+2;
					petal1[Cam_num[Telscenter_cam[mainOrsub]]-2]=Cam_num[Telscenter_cam[mainOrsub]]-2;
					petal1[Cam_num[Telscenter_cam[mainOrsub]]]=Cam_num[Telscenter_cam[mainOrsub]];
					petal1[Cam_num[Telscenter_cam[mainOrsub]]+1]=Cam_num[Telscenter_cam[mainOrsub]]+1;
					petal1[Cam_num[Telscenter_cam[mainOrsub]]-1]=Cam_num[Telscenter_cam[mainOrsub]]-1;

					petal2[Cam_num[Telscenter_cam[mainOrsub]]+2]=Cam_num[Telscenter_cam[mainOrsub]]+2;
					petal2[Cam_num[Telscenter_cam[mainOrsub]]-2]=Cam_num[Telscenter_cam[mainOrsub]]-2;
					petal2[Cam_num[Telscenter_cam[mainOrsub]]]=Cam_num[Telscenter_cam[mainOrsub]];
					petal2[Cam_num[Telscenter_cam[mainOrsub]]+1]=Cam_num[Telscenter_cam[mainOrsub]]+1;
					petal2[Cam_num[Telscenter_cam[mainOrsub]]-1]=Cam_num[Telscenter_cam[mainOrsub]]-1;
				}*/
#if 0
				if(RulerAngle<=270 ||RulerAngle>300)
				{
					DrawPanel(m_env,false,petal2,mainOrsub);
					m_env.GetmodelViewMatrix()->PushMatrix();
					m_env.GetmodelViewMatrix()->Translate(PanoLen,0.0,0.0);
					DrawPanel(m_env,false,petal1,mainOrsub);
					m_env.GetmodelViewMatrix()->PopMatrix();
				}
				else if(RulerAngle<=300 && RulerAngle>270)
				{
					DrawPanel(m_env,false,petal1,mainOrsub);
					m_env.GetmodelViewMatrix()->PushMatrix();
					m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0);
					DrawPanel(m_env,false,petal1,mainOrsub);
					m_env.GetmodelViewMatrix()->PopMatrix();
				}

#else
	/*			m_env.GetmodelViewMatrix()->PushMatrix();
				m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0); //1
				DrawPanel(m_env,false,NULL);
				m_env.GetmodelViewMatrix()->PopMatrix();
				m_env.GetmodelViewMatrix()->PushMatrix();
				m_env.GetmodelViewMatrix()->Translate(PanoLen,0.0,0.0);//2
				DrawPanel(m_env,false,NULL);
				m_env.GetmodelViewMatrix()->PopMatrix();
				m_env.GetmodelViewMatrix()->PushMatrix();
				m_env.GetmodelViewMatrix()->Translate(0.0,0.0,0.0);//3
				DrawPanel(m_env,false,NULL);
				m_env.GetmodelViewMatrix()->PopMatrix();*/
				DrawPanel(m_env,false,petal3,mainOrsub);
				m_env.GetmodelViewMatrix()->PushMatrix();
				m_env.GetmodelViewMatrix()->Translate(PanoLen,0.0,0.0);
				DrawPanel(m_env,false,petal4,mainOrsub);
				m_env.GetmodelViewMatrix()->PopMatrix();
#endif

for(int i=0;i<0;i++)
{
	if(displayMode==TELESCOPE_FRONT_MODE)
	{
		p_ForeSightFacade2[i]->SetAlign(1,TEL_FORESIGHT_POS_FRONT);
		p_ForeSightFacade2[i]->Draw(m_env,render.getRulerAngle()->Load(),mainOrsub);
	}
	else if(displayMode==TELESCOPE_RIGHT_MODE)
		{
		p_ForeSightFacade2[i]->SetAlign(1,TEL_FORESIGHT_POS_RIGHT);
		p_ForeSightFacade2[i]->Draw(m_env,render.getRulerAngle()->Load(),mainOrsub);
		}

	else if(displayMode==TELESCOPE_BACK_MODE)
		{
		p_ForeSightFacade2[i]->SetAlign(1,TEL_FORESIGHT_POS_BACK);
		p_ForeSightFacade2[i]->Draw(m_env,render.getRulerAngle()->Load(),mainOrsub);
		}

	else if(displayMode==TELESCOPE_LEFT_MODE)
	{
		p_ForeSightFacade2[i]->SetAlign(1,TEL_FORESIGHT_POS_LEFT);
		p_ForeSightFacade2[i]->Draw(m_env,render.getRulerAngle()->Load(),mainOrsub);
	}
}
	{
		{
			m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
		}
		m_env.GetmodelViewMatrix()->PopMatrix();
	}
	return;
}


void Render::RenderPanoView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,int mainOrsub)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(37.6f, float(w) / float(h), 1.0f, 100.0f);
	//m_env.GetviewFrustum()->SetPerspective(27.0f,330.0/100.0, 20.0f, 2000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	//pVehicle->PrepareBlendMode();
	m_env.GetmodelViewMatrix()->PushMatrix();

	float delta_dis=0.0;

	int i=0;

	//get center camera matrix apply to the modelviewmatrix
{
		M3DMatrix44f mCamera;
		repositioncamera();
		PanoViewCameraFrame.GetCameraMatrix(mCamera);
		m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
	}

m_env.GetmodelViewMatrix()->Scale(4.0,1.0,4.5);
m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-3.0);

	DrawPanel(m_env,true,NULL,mainOrsub);
	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->Translate(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x(),0.0,0.0);
	DrawPanel(m_env,false,NULL,mainOrsub);
	m_env.GetmodelViewMatrix()->PopMatrix();
	m_env.GetmodelViewMatrix()->PushMatrix();
	if(RulerAngle<180.0)
	{
		m_env.GetmodelViewMatrix()->Translate(2.0*(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x()),0.0,0.0);
	}
	else
	{
		m_env.GetmodelViewMatrix()->Translate(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x()),0.0,0.0);
	}

	DrawPanel(m_env,false,NULL,mainOrsub);
	m_env.GetmodelViewMatrix()->PopMatrix();
	DrawSlideonPanel(m_env);
	if(getFollowValue())
	{
		DrawCrossonPanel(m_env);
	}

	if(getenableshowruler())
	{
		DrawRuleronPanel(m_env);
	}

	{
		m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
	}
	m_env.GetmodelViewMatrix()->PopMatrix();
	Rect *rect1;
	char text[16];
	int j=0;
	for(i=0;i<4;i++)
	{
		strcpy(text,"");
		sprintf(text,"%3d",i*90);
		rect1=new Rect(i*g_windowWidth/4.0-g_windowWidth/20.0, g_windowHeight*0.7/4.0,g_windowWidth/8,g_windowHeight/12);
		DrawAngleCordsView(m_env,rect1,text,1);
		delete(rect1);
	}
	strcpy(text,"");
	sprintf(text,"%3d",i*90);
	rect1=new Rect(i*g_windowWidth/4.0-g_windowWidth/15.0, g_windowHeight*0.7/4.0,g_windowWidth/8,g_windowHeight/12);
	DrawAngleCordsView(m_env,rect1,text,1);
	delete(rect1);
}

void Render::RenderOnetimeView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,int mainOrsub)
{
			int petal1[CAM_COUNT];
			memset(petal1,-1,sizeof(petal1));
			int petal2[CAM_COUNT];
			memset(petal2,-1,sizeof(petal2));
			int petal3[CAM_COUNT];
			memset(petal3,-1,sizeof(petal3));
			int petal4[CAM_COUNT];
						memset(petal4,-1,sizeof(petal4));
						int petal5[CAM_COUNT];
									memset(petal5,-1,sizeof(petal5));
			int petaltest[12]={0,1,2,3,4,-1,-1,-1,-1,-1,-1,-1};
		glViewport(x,y,w,h);
		m_env.GetviewFrustum()->SetPerspective(40.0f, float(w) / float(h), 1.0f, 100.0f);
		m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());
		m_env.GetmodelViewMatrix()->PushMatrix();
		static bool once = true;
	{
			M3DMatrix44f mCamera;
			M3DMatrix44f tempCamera;
			if(once)
			{
				once = false;
			}
			if(panocamonforesight[mainOrsub].GetFront())
				panocamonforesight[mainOrsub].getOneTimeCam().GetCameraMatrix(mCamera);
			else
					panocamonforesight[mainOrsub].getOneTimeCam2().GetCameraMatrix(mCamera);
			m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
		}

	float FXangle=foresightPos[mainOrsub].GetAngle()[0];//todo
	float Angle=FXangle+RulerAngle;
		while(Angle>360)
		{
			Angle-=360.0;
		}
			int Cam_num[10]={2,1,0,9,8,7,6,5,4,3};

//		if((Angle<18.0)||(Angle>=342.0))
			if((Angle>=342.0-18))
		{
			center_cam[mainOrsub]=0;
		}
		else
		{
			temp_math[mainOrsub]=(Angle)/36.0;
			center_cam[mainOrsub]=(int)temp_math[mainOrsub];
			center_cam[mainOrsub]++;
		}
		petal1[Cam_num[center_cam[mainOrsub]]]=Cam_num[center_cam[mainOrsub]];
		petal2[Cam_num[center_cam[mainOrsub]]+1]=Cam_num[center_cam[mainOrsub]]+1;

		if(Cam_num[center_cam[mainOrsub]]==9)
		{
			petal3[0]=0;
			petal4[9]=9;
			m_env.GetmodelViewMatrix()->PushMatrix();
			DrawPanel(m_env,false,petal4,mainOrsub);
			m_env.GetmodelViewMatrix()->PopMatrix();

			m_env.GetmodelViewMatrix()->PushMatrix();
			m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0); //1
			DrawPanel(m_env,false,petal3,mainOrsub);
			DrawPanel(m_env,false,petal4,mainOrsub);
			m_env.GetmodelViewMatrix()->PopMatrix();
		}
		else if (Cam_num[center_cam[mainOrsub]]==4)
		{
			m_env.GetmodelViewMatrix()->PushMatrix();
			if(panocamonforesight[mainOrsub].GetFront())
			{
				m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0);
			}
				DrawPanel(m_env,false,petal1,mainOrsub);
				DrawPanel(m_env,false,petal2,mainOrsub);

			m_env.GetmodelViewMatrix()->PopMatrix();
		}
		else if(Cam_num[center_cam[mainOrsub]]>=0&&Cam_num[center_cam[mainOrsub]]<4)
		{
			m_env.GetmodelViewMatrix()->PushMatrix();
			m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0); //1
			DrawPanel(m_env,false,petal1,mainOrsub);
			DrawPanel(m_env,false,petal2,mainOrsub);
			m_env.GetmodelViewMatrix()->PopMatrix();
		}
		else
		{
		m_env.GetmodelViewMatrix()->PushMatrix();
		DrawPanel(m_env,false,petal1,mainOrsub);
		DrawPanel(m_env,false,petal2,mainOrsub);
		m_env.GetmodelViewMatrix()->PopMatrix();
		}


#if 0
	if(RulerAngle<160.0 ||RulerAngle>=270.0)
		{
			DrawPanel(false,petal2);
			m_env.GetmodelViewMatrix()->PushMatrix();
			m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0);
			DrawPanel(false,petal1);
			m_env.GetmodelViewMatrix()->PopMatrix();
		}
		else if(RulerAngle>=160.0 && RulerAngle<=200.0)
		{
			DrawPanel(false,petal2);
			if(FXangle>=270)
			{
					m_env.GetmodelViewMatrix()->PushMatrix();
					m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0);
					DrawPanel(false,petal1);
					m_env.GetmodelViewMatrix()->PopMatrix();
			}
			else{
					m_env.GetmodelViewMatrix()->PushMatrix();
					m_env.GetmodelViewMatrix()->Translate(PanoLen,0.0,0.0);
					DrawPanel(false,petal1);
					m_env.GetmodelViewMatrix()->PopMatrix();
			}
		}
		else// if(p_LineofRuler->Load()<270.0)
		{
			DrawPanel(false,petal1);
			m_env.GetmodelViewMatrix()->PushMatrix();
			m_env.GetmodelViewMatrix()->Translate(PanoLen,0.0,0.0);
			DrawPanel(false,petal1);
			m_env.GetmodelViewMatrix()->PopMatrix();
		}
				m_env.GetmodelViewMatrix()->PushMatrix();
							m_env.GetmodelViewMatrix()->Translate(0,0.0,0.0); //1
							DrawPanel(m_env,false,NULL,mainOrsub);
							m_env.GetmodelViewMatrix()->PopMatrix();
							m_env.GetmodelViewMatrix()->PushMatrix();
										m_env.GetmodelViewMatrix()->Translate(-2*PanoLen,0.0,0.0); //1
										DrawPanel(m_env,false,NULL,mainOrsub);
										m_env.GetmodelViewMatrix()->PopMatrix();
#endif


		{
			m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
		}
		m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::RenderMilView(GLEnv &m_env,GLint x, GLint y,GLint w, GLint h)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());
	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->LoadIdentity();

	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h);
	m_env.GetmodelViewMatrix()->Scale(w, h, 1.0f);
	{
		char text[8];
		char text2[8];
		char text3[100][10];
		strcpy(text," FANG:");
		strcpy(text2," YANG:");
		int j=0;
		for(int i=0;i<CAM_COUNT;i++)
		{
		if(selfcheck.GetBrokenCam()[i]==0)
		{
			sprintf(text3[j],"Q_%d",i);
			  j++;
		}
		}
		if(displayMode!=TRIM_MODE)
		{
		Rect2i rect1(g_windowWidth*2450.0/1920.0,g_windowHeight*420.0/1920.0, g_windowWidth/5, g_windowHeight/5);
		DrawAngleCordsView(m_env,&rect1,text,0.65);
		Rect2i rect2(g_windowWidth*2450.0/1920.0,g_windowHeight*780.0/1920.0, g_windowWidth/5, g_windowHeight/5);
		DrawAngleCordsView(m_env,&rect2,text2,0.65);
		}
		for(int k=0;k<j;k++)
		{
			if(k<5)
			{
				Rect2i rect1(g_windowWidth*(2450.0+k*100)/1920.0,g_windowHeight*(1200.0)/1920.0, g_windowWidth/5, g_windowHeight/5);
				DrawAngleCordsView(m_env,&rect1,text3[k],0.65);
			}
			else
			{
				Rect2i rect1(g_windowWidth*(2450.0+(k-5)*100)/1920.0,g_windowHeight*(1100.0)/1920.0, g_windowWidth/5, g_windowHeight/5);
				DrawAngleCordsView(m_env,&rect1,text3[k],0.65);
			}
		}
	}
	m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::RenderPositionView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h)
{
	Rect *rect1;
	Rect *rect2;
	glViewport(x,y,w,h);
		m_env.GetviewFrustum()->SetPerspective(37.6f, float(w) / float(h), 1.0f, 100.0f);
		//m_env.GetviewFrustum()->SetPerspective(27.0f,330.0/100.0, 20.0f, 2000.0f);
		m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());
	//	pVehicle->PrepareBlendMode();
		m_env.GetmodelViewMatrix()->PushMatrix();
		float delta_dis=0.0;
		int i=0;
		{
			M3DMatrix44f mCamera;
			repositioncamera();
			PanoViewCameraFrame.GetCameraMatrix(mCamera);
			m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
		}

	m_env.GetmodelViewMatrix()->Scale(1.0,1.0,4.5);
	m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-3.0);
//	}
		{
			m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
		}
		m_env.GetmodelViewMatrix()->PopMatrix();
		    char text[8];
			char text2[8];
			char text3[8];
			char temp[10];
			char temp2[10];
			int n;
			char text4[100][10];
			char text5[100][10];
			bzero(text4,1000);
			bzero(text5,1000);
#if USE_UART
			int recv_n=0;
			n=ReadMessage(IPC_MSG_TYPE_SELF_CHECK).payload.ipc_s_faultcode.fault_code_number;
			if(n!=0)
			{
				for(i=0;i<n;i++)
				{
					recv_n=ReadMessage(IPC_MSG_TYPE_SELF_CHECK).payload.ipc_s_faultcode.start_selfcheak[i];
					if(recv_n>0&&recv_n<100)
						sprintf(text4[i],"E%d",recv_n);
				}
			}
			bzero(text,8);
			bzero(text2,8);
			bzero(text3,8);

			if(ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.orientation_angle>=0
					&&ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.orientation_angle<=6000)
			{
				sprintf(text,"%d",ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.orientation_angle);
			}
			if(ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.pitch_angle>=-167
					&&ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.pitch_angle<=1000)
			{
				sprintf(text2,"%d",ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.pitch_angle);
			}
			if(ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.distance>=0
					&&ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.distance<=9999)
			{
				sprintf(text3,"%d",ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.distance);
			}

			int *BrokenCam=selfcheck.getBrokenCam();
					int BrokenSum=0;
					for(int i=0;i<CAM_COUNT;i++)
					{
						if(BrokenCam[i]!=-1)
						{
							sprintf(text5[i],"QE%d",BrokenCam[i]);
							BrokenSum++;
						}
					}
	#else

		   n=100;
		   for(int i=0;i<100;i++)
			{
				sprintf(text4[i]," ");
			}
			for(int i=0;i<n;i++)
			{
				sprintf(temp,"E%d",i);
				strcpy(text4[i],temp);
			}
			for(int i=0;i<n;i++)
			{
				sprintf(temp2,"QE%d",i);
				strcpy(text5[i],temp2);
			}
			strcpy(text," FANG:");
			strcpy(text2," YANG:");
			strcpy(text3," M IL");
		//	strcpy(text3,"    CEJU:");
#endif

			float pos1[50]={3.42,3.10,2.825,2.6,2.4,2.22,2.065,1.94,1.83,1.73,1.645,1.57,1.495,1.435,1.382,1.334,1.286,1.237,1.19
			,1.15,1.112,1.074};

			float pos2[20]={1.615,1.54,1.47,1.408,1.35,1.298,1.25,1.2055,1.16,1.12,1.08};
			float pos3[5]={1.7,1.615,1.53,1.46,1.39};

	//		if(fboMode==FBO_ALL_VIEW_MODE
	//				||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE)
			if(0)
			{
				if(1)
				{
			/*		Rect2i rect1(g_windowWidth*730/1024,g_windowHeight*320/768, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect1,text,0.8);
					Rect2i rect2(g_windowWidth*730/1024,g_windowHeight*280.0/768.0, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect2,text2,0.8);
					Rect2i rect3(g_windowWidth*900/1024,g_windowHeight*320.0/768.0, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect3,text3,0.8);
					Rect2i rect4(g_windowWidth*900/1024,g_windowHeight*280.0/768.0, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect4,text3,0.8);*/
					Rect2i rect1(g_windowWidth*2600.0/1920.0,g_windowHeight/4, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect1,text,0.65);
					Rect2i rect2(g_windowWidth*89.5/100,g_windowHeight*400.0/1920.0, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect2,text2,0.65);
				}
				else
				{
					Rect2i rect1(g_windowWidth*89.5/100,g_windowHeight/4, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect1,text,0.65);
					Rect2i rect2(g_windowWidth*89.5/100,g_windowHeight*400.0/1920.0, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect2,text2,0.65);
					Rect2i rect3(g_windowWidth*89.5/100,g_windowHeight*400.0/1920.0, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect2,text2,0.65);
					Rect2i rect4(g_windowWidth*89.5/100,g_windowHeight*400.0/1920.0, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect2,text2,0.65);
				}
			}




			if(displayMode==	ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
					||displayMode==TELESCOPE_FRONT_MODE
					||displayMode==TELESCOPE_RIGHT_MODE
					||displayMode==TELESCOPE_BACK_MODE
					||displayMode==TELESCOPE_LEFT_MODE
					)
			{
					Rect2i rect1(g_windowWidth*89.5/100,g_windowHeight/4, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect1,text,0.65);
					Rect2i rect2(g_windowWidth*89.5/100,g_windowHeight*400.0/1920.0, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect2,text2,0.65);
								static int textnum2=0;
									for(int i=0;i<12;i++)
									{
										for(int j=0;j<3;j++)
										{
											Rect2i temprect(g_windowWidth*(90+3.5*j)/100,g_windowHeight/pos1[i], g_windowWidth/20, g_windowHeight*1/20);
											DrawAngleCordsView(m_env,&temprect,text4[textnum2],0.65);
											textnum2++;
										}
									}
									if(n>33)
									{
										for(int i=0;i<11;i++)
										{
											for(int j=0;j<3;j++)
											{
												Rect2i temprect(g_windowWidth*(90+3.5*j-3.5*3)/100,g_windowHeight/pos1[i+1], g_windowWidth/20, g_windowHeight*1/20);
												DrawAngleCordsView(m_env,&temprect,text4[textnum2],0.65);
												textnum2++;
											}
										}
									}
									if(n>66)
									for(int i=0;i<11;i++)
									{
										for(int j=0;j<3;j++)
										{
											if(textnum2>100)
											{
												textnum2=100;
											}
											Rect2i temprect(g_windowWidth*(90+3.5*j-3.5*3*2)/100,g_windowHeight/pos1[i+1], g_windowWidth/20, g_windowHeight*1/20);
											DrawAngleCordsView(m_env,&temprect,text4[textnum2],0.65);
											textnum2++;
										}
									}
									textnum2=0;
			}
			else if(displayMode==	VGA_WHITE_VIEW_MODE
					||displayMode==VGA_HOT_BIG_VIEW_MODE
					||displayMode==VGA_HOT_SMALL_VIEW_MODE
					||displayMode==VGA_FUSE_WOOD_LAND_VIEW_MODE
					||displayMode==VGA_FUSE_GRASS_LAND_VIEW_MODE
					||displayMode==VGA_FUSE_SNOW_FIELD_VIEW_MODE
					||displayMode==VGA_FUSE_DESERT_VIEW_MODE
					||displayMode==VGA_FUSE_CITY_VIEW_MODE
					||displayMode==SDI1_WHITE_BIG_VIEW_MODE
					||displayMode==SDI1_WHITE_SMALL_VIEW_MODE
					||displayMode==SDI2_HOT_BIG_VIEW_MODE
					||displayMode==SDI2_HOT_SMALL_VIEW_MODE
					||displayMode==PAL1_WHITE_BIG_VIEW_MODE
					||displayMode==PAL1_WHITE_SMALL_VIEW_MODE
					||displayMode==PAL2_HOT_BIG_VIEW_MODE
					||displayMode==PAL2_HOT_SMALL_VIEW_MODE
					)
				{
				if(displayMode==	VGA_WHITE_VIEW_MODE
					||displayMode==SDI1_WHITE_BIG_VIEW_MODE
					||displayMode==SDI1_WHITE_SMALL_VIEW_MODE
					||displayMode==PAL1_WHITE_BIG_VIEW_MODE
					||displayMode==PAL1_WHITE_SMALL_VIEW_MODE
						)
				{
					Rect2i rect1(g_windowWidth*85/100,g_windowHeight/2.02, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect1,text,0.65);
					Rect2i rect2(g_windowWidth*85/100,g_windowHeight/2.15, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect2,text2,0.65);
					Rect2i rect3(g_windowWidth*85/100,g_windowHeight/2.3, g_windowWidth/20, g_windowHeight*1/20);
					DrawAngleCordsView(m_env,&rect3,text3,0.65);
				}

					else
					{
						Rect2i rect1(g_windowWidth*85/100,g_windowHeight/1.95, g_windowWidth/20, g_windowHeight*1/20);
						DrawAngleCordsView(m_env,&rect1,text,0.65);
						Rect2i rect2(g_windowWidth*85/100,g_windowHeight/2.08, g_windowWidth/20, g_windowHeight*1/20);
						DrawAngleCordsView(m_env,&rect2,text2,0.65);
						Rect2i rect3(g_windowWidth*85/100,g_windowHeight/2.22, g_windowWidth/20, g_windowHeight*1/20);
						DrawAngleCordsView(m_env,&rect3,text3,0.65);
					}
					static int textnum=0;
					for(int i=0;i<12;i++)
					{
						for(int j=0;j<3;j++)
						{
							Rect2i temprect(g_windowWidth*(85+5*j)/100,g_windowHeight/pos2[i], g_windowWidth/20, g_windowHeight*1/20);
							DrawAngleCordsView(m_env,&temprect,text4[textnum],0.65);
							textnum++;
						}
					}
					if(n>33)
					{
						for(int i=0;i<11;i++)
						{
							for(int j=0;j<3;j++)
							{
								Rect2i temprect(g_windowWidth*(85+5*j-15)/100,g_windowHeight/pos2[i+1], g_windowWidth/20, g_windowHeight*1/20);
								DrawAngleCordsView(m_env,&temprect,text4[textnum],0.65);
								textnum++;
							}
						}
					}
					if(n>66)
					for(int i=0;i<11;i++)
					{
						for(int j=0;j<3;j++)
						{
							if(textnum>100)
							{
								textnum=100;
							}
							Rect2i temprect(g_windowWidth*(85+5*j-15*2)/100,g_windowHeight/pos2[i+1], g_windowWidth/20, g_windowHeight*1/20);
							DrawAngleCordsView(m_env,&temprect,text4[textnum],0.65);
							textnum++;
						}
					} //98
					textnum=0;
			}
}
void Render::RenderCheckMyselfView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h)
{
	glViewport(x,y,w,h);
		m_env.GetviewFrustum()->SetPerspective(37.6f, float(w) / float(h), 1.0f, 100.0f);
		m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());
	//	pVehicle->PrepareBlendMode();
		m_env.GetmodelViewMatrix()->PushMatrix();
		float delta_dis=0.0;
		int i=0;
	{
			M3DMatrix44f mCamera;
			repositioncamera();
			CheckViewCameraFrame.GetCameraMatrix(mCamera);
			m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
		}
	m_env.GetmodelViewMatrix()->Scale(1.0,1.0,4.5);
	m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-3.0);

		{
			m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
		}
		m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::RenderTwotimesView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,int mainOrsub)
{
				int petal1[CAM_COUNT];
				memset(petal1,-1,sizeof(petal1));
				int petal2[CAM_COUNT];
				memset(petal2,-1,sizeof(petal2));
				int petal3[CAM_COUNT];
				memset(petal3,-1,sizeof(petal3));
				int petal4[CAM_COUNT];
				memset(petal4,-1,sizeof(petal4));
		glViewport(x,y,w,h);
		m_env.GetviewFrustum()->SetPerspective(40.0f, float(w) / float(h), 1.0f, 100.0f);
		m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());
		m_env.GetmodelViewMatrix()->PushMatrix();
			static bool once = true;
	{
			M3DMatrix44f mCamera;
			if(once)
			{
				once = false;
			}
#if 1
	/*		if(displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
					||displayMode==ALL_VIEW_MODE
					||SecondDisplayMode==SECOND_TOTAL_MODE_COUNT)*/
			{
				if(panocamonforesight[mainOrsub].GetFront())
					panocamonforesight[mainOrsub].getTwoTimesCam().GetCameraMatrix(mCamera);
				else
					panocamonforesight[mainOrsub].getTwoTimesCam2().GetCameraMatrix(mCamera);
				}
		/*	else if(displayMode==TELESCOPE_FRONT_MODE)
			{
				telcamonforesight[mainOrsub].getTwoTimesCamTelF().GetCameraMatrix(mCamera);
			}
			else if(displayMode==TELESCOPE_RIGHT_MODE)
				telcamonforesight[mainOrsub].getTwoTimesCamTelR().GetCameraMatrix(mCamera);
			else if(displayMode==TELESCOPE_BACK_MODE)
				telcamonforesight[mainOrsub].getTwoTimesCamTelB().GetCameraMatrix(mCamera);
			else if(displayMode==TELESCOPE_LEFT_MODE)
				telcamonforesight[mainOrsub].getTwoTimesCamTelL().GetCameraMatrix(mCamera);
*/
#endif
			m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
	}

//if(fboMode==FBO_ALL_VIEW_559_MODE)
{
	float FXangle=foresightPos[mainOrsub].GetAngle()[0];//todo
	float Angle=FXangle+RulerAngle;
		while(Angle>360)
		{
			Angle-=360.0;
		}
		int Cam_num[10]={2,1,0,9,8,7,6,5,4,3};
		if((Angle>=342.0-18))
	{
		center_cam[mainOrsub]=0;
	}
	else
	{
		temp_math[mainOrsub]=(Angle)/36.0;
		center_cam[mainOrsub]=(int)temp_math[mainOrsub];
		center_cam[mainOrsub]++;
	}
	petal1[Cam_num[center_cam[mainOrsub]]]=Cam_num[center_cam[mainOrsub]];
	petal2[Cam_num[center_cam[mainOrsub]]+1]=Cam_num[center_cam[mainOrsub]]+1;

	if(Cam_num[center_cam[mainOrsub]]==9)
	{
		petal3[0]=0;
		petal4[9]=9;
		m_env.GetmodelViewMatrix()->PushMatrix();
		DrawPanel(m_env,false,petal4,mainOrsub);
		m_env.GetmodelViewMatrix()->PopMatrix();

		m_env.GetmodelViewMatrix()->PushMatrix();
		m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0); //1
		DrawPanel(m_env,false,petal3,mainOrsub);
		DrawPanel(m_env,false,petal4,mainOrsub);
		m_env.GetmodelViewMatrix()->PopMatrix();
	}
	else if (Cam_num[center_cam[mainOrsub]]==4)
	{
		m_env.GetmodelViewMatrix()->PushMatrix();
		if(panocamonforesight[mainOrsub].GetFront())
		{
			m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0);
		}
			DrawPanel(m_env,false,petal1,mainOrsub);
			DrawPanel(m_env,false,petal2,mainOrsub);

		m_env.GetmodelViewMatrix()->PopMatrix();
	}
	else if(Cam_num[center_cam[mainOrsub]]>=0&&Cam_num[center_cam[mainOrsub]]<4)
	{
		m_env.GetmodelViewMatrix()->PushMatrix();
		m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0); //1
		DrawPanel(m_env,false,petal1,mainOrsub);
		DrawPanel(m_env,false,petal2,mainOrsub);
		m_env.GetmodelViewMatrix()->PopMatrix();
	}
	else
	{
	m_env.GetmodelViewMatrix()->PushMatrix();
	DrawPanel(m_env,false,petal1,mainOrsub);
	DrawPanel(m_env,false,petal2,mainOrsub);
	m_env.GetmodelViewMatrix()->PopMatrix();
	}
}
#if 0
else if(displayMode==TELESCOPE_FRONT_MODE
		||displayMode==TELESCOPE_RIGHT_MODE
		||displayMode==TELESCOPE_BACK_MODE
		||displayMode==TELESCOPE_LEFT_MODE)
{
	float FXangle=foresightPos[MAIN].GetAngle()[0];//todo
	float Angle=RulerAngle+FXangle;
	int i=0;
	if(displayMode==TELESCOPE_FRONT_MODE)
	{
		 i=0;
	}
	else if(displayMode==TELESCOPE_RIGHT_MODE)
	{
		i=3;
	}
	else if(displayMode==TELESCOPE_BACK_MODE)
	{
		i=6;
	}
	else if(displayMode==TELESCOPE_LEFT_MODE)
	{
		i=9;
	}
		while(Angle>360)
		{
			Angle-=360.0;
		}
	//	printf("F:%f\tA:%f\n",FXangle,Angle);
		int Cam_num[12]={3,2,1,0,11,10,9,8,7,6,5,4};
		if((Angle<15.0-8.0)||(Angle>=345.0-8.0))
		{
			TelTwotimescenter_cam[mainOrsub]=0;
		}
		else
		{
			TelTwotimestemp_math[mainOrsub]=(Angle-15.0+0.0)/30.0;
			TelTwotimescenter_cam[mainOrsub]=(int)TelTwotimestemp_math[mainOrsub];
			TelTwotimescenter_cam[mainOrsub]++;
		}

		//printf("now Center_cam= %d\n",Cam_num[Twotimescenter_cam]);
		TelTwotimescenter_cam[mainOrsub]+=i;
		while(TelTwotimescenter_cam[mainOrsub]>11)
		{
			TelTwotimescenter_cam[mainOrsub]-=12;
		}
		//printf("now TelTwotimescenter_cam= %d\n",Cam_num[TelTwotimescenter_cam]);
			if(Cam_num[TelTwotimescenter_cam[mainOrsub]]==11)
			{
				petal1[0]=0;
				petal1[11]=11;
				petal1[10]=10;
				petal2[0]=0;
				petal2[11]=11;
				petal2[10]=10;
			}
			else if(Cam_num[TelTwotimescenter_cam[mainOrsub]]==0)
			{
				petal1[0]=0;
				petal1[1]=1;
				petal1[11]=11;
				petal2[0]=0;
				petal2[1]=1;
				petal2[11]=11;
			}
			else
			{
				petal1[Cam_num[TelTwotimescenter_cam[mainOrsub]]]=Cam_num[TelTwotimescenter_cam[mainOrsub]];
				petal1[Cam_num[TelTwotimescenter_cam[mainOrsub]]+1]=Cam_num[TelTwotimescenter_cam[mainOrsub]]+1;
				petal1[Cam_num[TelTwotimescenter_cam[mainOrsub]]-1]=Cam_num[TelTwotimescenter_cam[mainOrsub]]-1;

				petal2[Cam_num[TelTwotimescenter_cam[mainOrsub]]]=Cam_num[TelTwotimescenter_cam[mainOrsub]];
				petal2[Cam_num[TelTwotimescenter_cam[mainOrsub]]+1]=Cam_num[TelTwotimescenter_cam[mainOrsub]]+1;
				petal2[Cam_num[TelTwotimescenter_cam[mainOrsub]]-1]=Cam_num[TelTwotimescenter_cam[mainOrsub]]-1;
			}
}

 if(displayMode==TELESCOPE_FRONT_MODE
		||displayMode==TELESCOPE_RIGHT_MODE
		||displayMode==TELESCOPE_BACK_MODE
		||displayMode==TELESCOPE_LEFT_MODE
		||displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE)
 {
		float FXangle=foresightPos[MAIN].GetAngle()[0]; //todo
		if(RulerAngle<=135.0 ||RulerAngle>=270.0)
			{
				DrawPanel(m_env,false,petal2,mainOrsub);
				m_env.GetmodelViewMatrix()->PushMatrix();
				m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0);
				DrawPanel(m_env,false,petal1,mainOrsub);
				m_env.GetmodelViewMatrix()->PopMatrix();
			}
			else if(RulerAngle>135.0 && RulerAngle<175.0)
			{
				DrawPanel(m_env,false,petal2,mainOrsub);
				if(FXangle>=270)
				{
						m_env.GetmodelViewMatrix()->PushMatrix();
						m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0);
						DrawPanel(m_env,false,petal1,mainOrsub);
						m_env.GetmodelViewMatrix()->PopMatrix();
				}
				else{
						m_env.GetmodelViewMatrix()->PushMatrix();
						m_env.GetmodelViewMatrix()->Translate(PanoLen,0.0,0.0);
						DrawPanel(m_env,false,petal1,mainOrsub);
						m_env.GetmodelViewMatrix()->PopMatrix();
				}

			}
			else// if(p_LineofRuler->Load()<270.0)
			{
				DrawPanel(m_env,false,petal1,mainOrsub);
				m_env.GetmodelViewMatrix()->PushMatrix();
				m_env.GetmodelViewMatrix()->Translate(PanoLen,0.0,0.0);
				DrawPanel(m_env,false,petal1,mainOrsub);
				m_env.GetmodelViewMatrix()->PopMatrix();
			}
 }
	else
	{
					m_env.GetmodelViewMatrix()->PushMatrix();
					m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0); //1
					DrawPanel(m_env,false,NULL,mainOrsub);
					m_env.GetmodelViewMatrix()->PopMatrix();
					m_env.GetmodelViewMatrix()->PushMatrix();
					m_env.GetmodelViewMatrix()->Translate(PanoLen,0.0,0.0);//2
					DrawPanel(m_env,false,NULL,mainOrsub);
					m_env.GetmodelViewMatrix()->PopMatrix();
					m_env.GetmodelViewMatrix()->PushMatrix();
					m_env.GetmodelViewMatrix()->Translate(0.0,0.0,0.0);//3
					DrawPanel(m_env,false,NULL,mainOrsub);
					m_env.GetmodelViewMatrix()->PopMatrix();
	}
#endif
		{
			m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
		}
		m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::TargectTelView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,int targetIdx,int camIdx,int enlarge,int mainOrsub)
{
#if MVDECT
		glViewport(x,y,w,h);
		glClear(GL_DEPTH_BUFFER_BIT);
		m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);

		m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

		m_env.GetmodelViewMatrix()->PushMatrix();
		m_env.GetmodelViewMatrix()->LoadIdentity();
		// move h since the shadow dimension is [-1,1], use h/2 if it is [0,1]
		M3DMatrix44f camera ;

		targetFrame[targetIdx].GetCameraMatrix(camera);
		m_env.GetmodelViewMatrix()->PushMatrix(targetFrame[targetIdx]);
		m_env.GetmodelViewMatrix()->PopMatrix();
		m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h);//-h
		m_env.GetmodelViewMatrix()->Scale(w, h, 1.0f);
		DrawTargetVideo(m_env,targetIdx,camIdx,true);
		m_env.GetmodelViewMatrix()->PopMatrix();
#else
		RenderSDIView(m_env,x,y,w,h,true);
#endif
}



void Render::RenderFourtimesTelView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,int mainOrsub)
{
	//get_delta;
	int petal1[CAM_COUNT];
					memset(petal1,-1,sizeof(petal1));
					int petal2[CAM_COUNT];
					memset(petal2,-1,sizeof(petal2));

		glViewport(x,y,w,h);
		m_env.GetviewFrustum()->SetPerspective(20.0f, float(w) / float(h), 1.0f, 100.0f);
		m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

//		pVehicle->PrepareBlendMode();
		m_env.GetmodelViewMatrix()->PushMatrix();
		//get center camera matrix apply to the modelviewmatrix
		static bool once = true;
	{
			M3DMatrix44f mCamera;
			if(once)
			{
				once = false;
				//camonforesight.getOneTimeCam().MoveRight(-36.0f);
			}
		if(displayMode==TELESCOPE_FRONT_MODE)
			telcamonforesight[mainOrsub].getFourTimesCamTelF().GetCameraMatrix(mCamera);
		else if(displayMode==TELESCOPE_RIGHT_MODE)
			telcamonforesight[mainOrsub].getFourTimesCamTelR().GetCameraMatrix(mCamera);
		else if(displayMode==TELESCOPE_BACK_MODE)
			telcamonforesight[mainOrsub].getFourTimesCamTelB().GetCameraMatrix(mCamera);
		else if(displayMode==TELESCOPE_LEFT_MODE)
			telcamonforesight[mainOrsub].getFourTimesCamTelL().GetCameraMatrix(mCamera);

			m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
		}
	float FXangle=foresightPos[MAIN].GetAngle()[0];//todo
	float Angle=RulerAngle+FXangle;
	while(Angle>360)
	{
		Angle-=360.0;
	}
//	printf("FXangle=%f\n",Angle);
		int i=0;
			if(displayMode==TELESCOPE_FRONT_MODE)
			{
				 i=0;
			}
			else if(displayMode==TELESCOPE_RIGHT_MODE)
			{
				i=3;
			}
			else if(displayMode==TELESCOPE_BACK_MODE)
			{
				i=6;
			}
			else if(displayMode==TELESCOPE_LEFT_MODE)
			{
				i=9;
			}
			int Cam_num[12]={3,2,1,0,11,10,9,8,7,6,5,4};
			if((Angle<15.0)||(Angle>=345.0))
			{
				Fourtimescenter_cam[mainOrsub]=0;
			}
			else
			{
				Fourtimestemp_math[mainOrsub]=(Angle-15.0)/30.0;
				Fourtimescenter_cam[mainOrsub]=(int)Fourtimestemp_math[mainOrsub];
				Fourtimescenter_cam[mainOrsub]++;
			}
			Fourtimescenter_cam[mainOrsub]+=i;
			while(Fourtimescenter_cam[mainOrsub]>11)
			{
				Fourtimescenter_cam[mainOrsub]-=12;
			}
		//	printf("now 4Center_cam= %d\n",Cam_num[Fourtimescenter_cam]);
				if(Cam_num[Fourtimescenter_cam[mainOrsub]]+1==12)
				{
					petal1[0]=0;
					petal1[11]=11;
					petal1[10]=10;
					petal2[0]=0;
					petal2[11]=11;
					petal2[10]=10;
				}
				else if(Cam_num[Fourtimescenter_cam[mainOrsub]]-1==-1)
				{
					petal1[0]=0;
					petal1[1]=1;
					petal1[11]=11;
					petal2[0]=0;
					petal2[1]=1;
					petal2[11]=11;
				}
				else
				{
					petal1[Cam_num[Fourtimescenter_cam[mainOrsub]]]=Cam_num[Fourtimescenter_cam[mainOrsub]];
					petal1[Cam_num[Fourtimescenter_cam[mainOrsub]]+1]=Cam_num[Fourtimescenter_cam[mainOrsub]]+1;
					petal1[Cam_num[Fourtimescenter_cam[mainOrsub]]-1]=Cam_num[Fourtimescenter_cam[mainOrsub]]-1;

					petal2[Cam_num[Fourtimescenter_cam[mainOrsub]]]=Cam_num[Fourtimescenter_cam[mainOrsub]];
					petal2[Cam_num[Fourtimescenter_cam[mainOrsub]]+1]=Cam_num[Fourtimescenter_cam[mainOrsub]]+1;
					petal2[Cam_num[Fourtimescenter_cam[mainOrsub]]-1]=Cam_num[Fourtimescenter_cam[mainOrsub]]-1;
				}
				if(RulerAngle<=135.0 ||RulerAngle>=270.0)
					{
						DrawPanel(m_env,false,petal2,mainOrsub);
						m_env.GetmodelViewMatrix()->PushMatrix();
						m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0);
						DrawPanel(m_env,false,petal1,mainOrsub);
						m_env.GetmodelViewMatrix()->PopMatrix();
					}
					else if(RulerAngle>135.0 && RulerAngle<175.0)
					{
						DrawPanel(m_env,false,petal2,mainOrsub);
						if(FXangle>=270)
						{
								m_env.GetmodelViewMatrix()->PushMatrix();
								m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0);
								DrawPanel(m_env,false,petal1,mainOrsub);
								m_env.GetmodelViewMatrix()->PopMatrix();
						}
						else{
								m_env.GetmodelViewMatrix()->PushMatrix();
								m_env.GetmodelViewMatrix()->Translate(PanoLen,0.0,0.0);
								DrawPanel(m_env,false,petal1,mainOrsub);
								m_env.GetmodelViewMatrix()->PopMatrix();
						}

					}
					else// if(p_LineofRuler->Load()<270.0)
					{
						DrawPanel(m_env,false,petal1);
						m_env.GetmodelViewMatrix()->PushMatrix();
						m_env.GetmodelViewMatrix()->Translate(PanoLen,0.0,0.0);
						DrawPanel(m_env,false,petal1);
						m_env.GetmodelViewMatrix()->PopMatrix();
					}
		{
			m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
		}
		m_env.GetmodelViewMatrix()->PopMatrix();
}
void Render::RenderMyLeftPanoView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,int mainOrsub,bool needSendData)
{
		glViewport(x,y,w,h);
		m_env.GetviewFrustum()->SetPerspective(40.0f, float(w) / float(h), 1.0f, 100.0f);
		m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());
	//	pVehicle->PrepareBlendMode();
		m_env.GetmodelViewMatrix()->PushMatrix();

		//get center camera matrix apply to the modelviewmatrix

			M3DMatrix44f mCamera;
			repositioncamera();
			LeftSmallPanoViewCameraFrame.GetCameraMatrix(mCamera);
			m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
			m_env.GetmodelViewMatrix()->Scale(2.50,1.0,3.3);
				m_env.GetmodelViewMatrix()->Translate(9.50,0.0,0.0);
				m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-2.0);
				if(RulerAngle<180.0)
					{
					m_env.GetmodelViewMatrix()->Translate((PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x()),0.0,0.0);
					}
				m_env.GetmodelViewMatrix()->PushMatrix();
				m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0);
				DrawPanel(m_env,true,NULL,mainOrsub);
				m_env.GetmodelViewMatrix()->PopMatrix();
				m_env.GetmodelViewMatrix()->PopMatrix();
				m_env.GetmodelViewMatrix()->PopMatrix();
}
void Render::RenderLeftPanoView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,int mainOrsub,bool needSendData)
{
	int petal1[CAM_COUNT];
	memset(petal1,-1,sizeof(petal1));
	int petal2[CAM_COUNT];
	memset(petal2,-1,sizeof(petal2));
	int petal3[CAM_COUNT];
	memset(petal1,-1,sizeof(petal3));
	int petal4[CAM_COUNT];
	memset(petal2,-1,sizeof(petal4));

	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(40.0f, float(w) / float(h), 1.0f, 100.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	//pVehicle->PrepareBlendMode();
	m_env.GetmodelViewMatrix()->PushMatrix();

	//get center camera matrix apply to the modelviewmatrix
{
		M3DMatrix44f mCamera;
		repositioncamera();
		if(displayMode==FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE
				||displayMode==FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE
				||displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
				||fboMode==FBO_ALL_VIEW_MODE
				||fboMode==FBO_ALL_VIEW_559_MODE
				||displayMode==ALL_VIEW_MODE
				||displayMode==TRIM_MODE
				||SecondDisplayMode==SECOND_ALL_VIEW_MODE
				||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE)
		{
			LeftSmallPanoViewCameraFrame.GetCameraMatrix(mCamera);
		}
		else if(displayMode==TWO_HALF_PANO_VIEW_MODE)
		{
			LeftPanoViewCameraFrame.GetCameraMatrix(mCamera);
		}
		m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
	}
if(displayMode==FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE
		||displayMode==FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE
	)

{
	m_env.GetmodelViewMatrix()->Scale(2.50,1.0,3.3);
	m_env.GetmodelViewMatrix()->Translate(9.50,0.0,0.0);
}

else if(displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE)
{
//	m_env.GetmodelViewMatrix()->Scale(4.0,1.0,3.3*UP_DOWN_SCALE);
	m_env.GetmodelViewMatrix()->Scale(2.50,1.0,3.3);
	m_env.GetmodelViewMatrix()->Translate(9.50,0.0,0.0);
}
else if(displayMode==TWO_HALF_PANO_VIEW_MODE)
{
	m_env.GetmodelViewMatrix()->Scale(4.0,1.0,4.5);
}
else if(displayMode==ALL_VIEW_MODE
		||SecondDisplayMode==SECOND_ALL_VIEW_MODE
		||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE
		||fboMode==FBO_ALL_VIEW_MODE
		||fboMode==FBO_ALL_VIEW_559_MODE
		||displayMode==TRIM_MODE)
{
	m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-1.2);//-17.6
	m_env.GetmodelViewMatrix()->Scale(6.0,1.0,4.58);//6.0 4.58
	m_env.GetmodelViewMatrix()->Translate(-15.8,0.0,-0.1);//-17.6
}
m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-2.0);



if(RulerAngle<180.0)
	{
		m_env.GetmodelViewMatrix()->Translate((PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x()),0.0,0.0);
	}
 if(displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE)
{
	 m_env.GetmodelViewMatrix()->Translate(0.0,0.0,2.0);
	 m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-2.6);
}
if(displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE)
{
	if(RulerAngle>=0&&RulerAngle<15)
	{
		petal2[0]=0;
		petal2[11]=11;
		for(int i=0;i<7;i++)
			petal1[i]=i;
	}
	else if(RulerAngle>=15&&RulerAngle<45)
		{
			for(int i=0;i<6;i++)
				petal1[i]=i;
			petal2[0]=0;
			for(int i=11;i<12;i++)
				petal2[i]=i;
		}

	else if(RulerAngle>=45&&RulerAngle<75)
		{
		for(int i=0;i<5;i++)
					petal1[i]=i;
				petal2[0]=0;
				for(int i=10;i<12;i++)
					petal2[i]=i;
		}
	else if(RulerAngle>=75&&RulerAngle<105)
		{
		for(int i=0;i<4;i++)
							petal1[i]=i;
						petal2[0]=0;
						for(int i=9;i<12;i++)
							petal2[i]=i;
		}
	else if(RulerAngle>=105&&RulerAngle<135)
		{
		for(int i=0;i<3;i++)
							petal1[i]=i;
						petal2[0]=0;
						for(int i=8;i<12;i++)
							petal2[i]=i;
		}
	else if(RulerAngle>=135&&RulerAngle<165)
			{
		for(int i=0;i<2;i++)
							petal1[i]=i;
						petal2[0]=0;
						for(int i=7;i<12;i++)
							petal2[i]=i;
			}

	else if(RulerAngle>=165&&RulerAngle<195)
			{
		for(int i=0;i<1;i++)
							petal1[i]=i;
						petal2[0]=0;
						for(int i=6;i<12;i++)
							petal2[i]=i;
			}
	else if(RulerAngle>=195&&RulerAngle<225)
				{
		for(int i=0;i<0;i++)
							petal1[i]=i;
						for(int i=5;i<12;i++)
							petal2[i]=i;
			}
	else if(RulerAngle>=225&&RulerAngle<255)
				{
		for(int i=0;i<0;i++)
								petal1[i]=i;
							for(int i=4;i<11;i++)
								petal2[i]=i;
				}
	else if(RulerAngle>=255&&RulerAngle<285)
					{
		for(int i=0;i<0;i++)
								petal1[i]=i;
							for(int i=3;i<10;i++)
								petal2[i]=i;
					}
	else if(RulerAngle>=285&&RulerAngle<315)
					{
		for(int i=0;i<0;i++)
								petal1[i]=i;
							for(int i=2;i<9;i++)
								petal2[i]=i;
					}
	else if(RulerAngle>=315&&RulerAngle<345)
					{
		for(int i=0;i<0;i++)
								petal1[i]=i;
							for(int i=1;i<8;i++)
								petal2[i]=i;
					}
	else if(RulerAngle>=345)
						{
			for(int i=0;i<0;i++)
									petal1[i]=i;
								for(int i=0;i<7;i++)
									petal2[i]=i;
						}
	if(RulerAngle<180.0)
		{
		DrawPanel(m_env,false,NULL,mainOrsub);
		//DrawPanel(false,petal1);
		}
	else
	{
		DrawPanel(m_env,false,NULL,mainOrsub);
	//	DrawPanel(false,petal2);
	}
}
else if(displayMode==ALL_VIEW_MODE
		||SecondDisplayMode==SECOND_ALL_VIEW_MODE
		||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE
		||fboMode==FBO_ALL_VIEW_MODE
		||fboMode==FBO_ALL_VIEW_559_MODE
		||displayMode==TRIM_MODE)
{
		for(int i=0;i<5;i++)
		{
			petal2[i]=i;
		}
		petal1[9]=9;
		m_env.GetmodelViewMatrix()->PushMatrix();
		DrawPanel(m_env,false,petal2,mainOrsub);
		m_env.GetmodelViewMatrix()->PopMatrix();
		m_env.GetmodelViewMatrix()->PushMatrix();
			m_env.GetmodelViewMatrix()->Translate(0,0.0,0.0);
			DrawPanel(m_env,false,petal1,mainOrsub);
			m_env.GetmodelViewMatrix()->PopMatrix();
}
else
{
	petal3[0]=0;
	DrawPanel(m_env,false,petal3,mainOrsub);
	petal3[0]=0;
	petal3[1]=1;
	petal3[2]=2;
	petal3[3]=3;
	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0);
	DrawPanel(m_env,false,petal3,mainOrsub);
	m_env.GetmodelViewMatrix()->PopMatrix();
}
/*
	m_env.GetmodelViewMatrix()->PushMatrix();
	if(RulerAngle<180.0)
	{
		m_env.GetmodelViewMatrix()->Translate(PanoLen,0.0,0.0);
		 if(displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE)
			{
			//	DrawPanel(false,petal2);
				DrawPanel(false,NULL);
			}
		 else
		 		 		DrawPanel(false,NULL);
	}
	else	if(RulerAngle>=180.0)
	{
		m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0);
		 if(displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE)
		{
		//	DrawPanel(false,petal1);
			 DrawPanel(false,NULL);
		}
		 else
		 		DrawPanel(false,NULL);
	}
	m_env.GetmodelViewMatrix()->PopMatrix();

	if(EnablePanoFloat==true)
	{
		GLShaderManager * my_shaderm=(GLShaderManager *)getDefaultShaderMgr();
		for(int i=0;i<CAM_COUNT;i++)
		{
			if(i==testPanoNumber)
			{
				my_shaderm->set_gain_(i,1.0,1.0,0.5);
			}
			else
			{
				my_shaderm->set_gain_(i,1.0,1.0,1.0);
			}
		}

	}

	if(displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE)
	{
		p_ForeSightFacade->SetAlign(3,FORESIGHT_POS_LEFT);
		p_ForeSightFacade->Draw(render.getRulerAngle()->Load());
	}
*/
/*
for(int i=0;i<2;i++)
{
p_ForeSightFacade[i]->SetAlign(3,FORESIGHT_POS_LEFT);
p_ForeSightFacade[i]->Draw(m_env,render.getRulerAngle()->Load());
}*/
	{
		m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
	}
	m_env.GetmodelViewMatrix()->PopMatrix();


	Rect * temp_rect;
	char text[16];
	if((EnablePanoFloat==true))
	{
		if((ALL_VIEW_MODE==displayMode))
		{
			if(PanoDirectionLeft==true)
			{
				strcpy(text,"left");
			}
			else
			{
				strcpy(text,"right");
			}
			temp_rect=new Rect(w*1/5, h*1/5,w/8,h/8);
			DrawAngleCordsView(m_env,temp_rect,text,1);
			strcpy(text,"");
			sprintf(text,"ch:%.2d",testPanoNumber);
			temp_rect=new Rect(w*1/5,h*2/5,w/8,h/8);
			DrawAngleCordsView(m_env,temp_rect,text,1);
		}
	}
}
void Render::RenderLeftForeSightView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,int mainOrsub)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(40.0f, float(w) / float(h), 1.0f, 100.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();

{
		M3DMatrix44f mCamera;
		repositioncamera();
		if(displayMode==FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE
				||displayMode==FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE
				||displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
				||fboMode==FBO_ALL_VIEW_MODE
				||fboMode==FBO_ALL_VIEW_559_MODE
				||displayMode==TRIM_MODE
				||displayMode==ALL_VIEW_MODE
				||SecondDisplayMode==SECOND_ALL_VIEW_MODE
				||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE)
		{
			LeftSmallPanoViewCameraFrame.GetCameraMatrix(mCamera);
		}
		m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
	}
 if(displayMode==ALL_VIEW_MODE
		||SecondDisplayMode==SECOND_ALL_VIEW_MODE
		||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE
		||fboMode==FBO_ALL_VIEW_MODE
		||fboMode==FBO_ALL_VIEW_559_MODE
		||displayMode==TRIM_MODE)
{
	m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-1.2);//-17.6
	m_env.GetmodelViewMatrix()->Scale(6.0,1.0,4.58);//6.0
	m_env.GetmodelViewMatrix()->Translate(-15.8,0.0,-0.1);//-17.6
}
m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-2.0);

if(RulerAngle<180.0)
	{
		m_env.GetmodelViewMatrix()->Translate((PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x()),0.0,0.0);
	}
p_ForeSightFacade[mainOrsub]->SetAlign(3,FORESIGHT_POS_LEFT);
p_ForeSightFacade[mainOrsub]->Draw(m_env,render.getRulerAngle()->Load(),mainOrsub);
	{
		m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
	}
	m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::RenderRightForeSightView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,int mainOrsub)
{
	glViewport(x,y,w,h);
	if(displayMode==FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE||
			displayMode==FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE
			||displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
			||displayMode==ALL_VIEW_MODE
			||fboMode==FBO_ALL_VIEW_MODE
			||fboMode==FBO_ALL_VIEW_559_MODE
			||displayMode==TRIM_MODE
			||SecondDisplayMode==SECOND_ALL_VIEW_MODE
			||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE
			)
	{
		m_env.GetviewFrustum()->SetPerspective(40.0, float(w) / float(h), 1.0f, 100.0f);
	}
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	//pVehicle->PrepareBlendMode();
	m_env.GetmodelViewMatrix()->PushMatrix();
		M3DMatrix44f mCamera;
		if(displayMode==FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE||
				displayMode==FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE ||
				displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
				||displayMode==ALL_VIEW_MODE
				||fboMode==FBO_ALL_VIEW_MODE
				||fboMode==FBO_ALL_VIEW_559_MODE
				||displayMode==TRIM_MODE
				||SecondDisplayMode==SECOND_ALL_VIEW_MODE
				||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE)
		{
			RightSmallPanoViewCameraFrame.GetCameraMatrix(mCamera);
		}
		m_env.GetmodelViewMatrix()->PushMatrix(mCamera);


 if(displayMode==ALL_VIEW_MODE
		||SecondDisplayMode==SECOND_ALL_VIEW_MODE
		||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE
		||fboMode==FBO_ALL_VIEW_MODE
		||fboMode==FBO_ALL_VIEW_559_MODE
		||displayMode==TRIM_MODE)
{
	m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-1.18);//26.2 +1.8
	m_env.GetmodelViewMatrix()->Scale(6.0,1.0,4.58);//6.0
	m_env.GetmodelViewMatrix()->Translate(26.2+2.0,0.0,0);//26.2 +1.8
}
m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-2.0);

p_ForeSightFacade[mainOrsub]->SetAlign(3,FORESIGHT_POS_LEFT);
p_ForeSightFacade[mainOrsub]->Draw(m_env,render.getRulerAngle()->Load(),mainOrsub);

	{
		m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
	}
	m_env.GetmodelViewMatrix()->PopMatrix();
}


void Render::RenderRightPanoView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,int mainOrsub,GLint scissor_x, GLint scissor_y, GLint scissor_w, GLint scissor_h,bool needSendData)
{
	int petal1[CAM_COUNT];
	memset(petal1,-1,sizeof(petal1));
	int petal2[CAM_COUNT];
	memset(petal2,-1,sizeof(petal2));
	int petal3[CAM_COUNT];
	memset(petal3,-1,sizeof(petal3));
	int petal4[CAM_COUNT];
	memset(petal4,-1,sizeof(petal4));

	glViewport(x,y,w,h);
	if(displayMode==FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE||
			displayMode==FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE
			||displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
			||displayMode==ALL_VIEW_MODE
			||fboMode==FBO_ALL_VIEW_MODE
			||fboMode==FBO_ALL_VIEW_559_MODE
			||displayMode==TRIM_MODE
			||SecondDisplayMode==SECOND_ALL_VIEW_MODE
			||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE
			)
	{
		m_env.GetviewFrustum()->SetPerspective(40.0, float(w) / float(h), 1.0f, 100.0f);
	}
	else if(displayMode==TWO_HALF_PANO_VIEW_MODE)
	{
		m_env.GetviewFrustum()->SetPerspective(40.0, float(w) / float(h), 1.0f, 100.0f);
	}
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	//pVehicle->PrepareBlendMode();
	m_env.GetmodelViewMatrix()->PushMatrix();
{

		M3DMatrix44f mCamera;
		if(displayMode==FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE||
				displayMode==FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE ||
				displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
				||displayMode==ALL_VIEW_MODE
				||fboMode==FBO_ALL_VIEW_MODE
				||fboMode==FBO_ALL_VIEW_559_MODE
				||displayMode==TRIM_MODE
				||SecondDisplayMode==SECOND_ALL_VIEW_MODE
				||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE)
		{
			RightSmallPanoViewCameraFrame.GetCameraMatrix(mCamera);
		}
		else if(displayMode==TWO_HALF_PANO_VIEW_MODE)
		{
			RightPanoViewCameraFrame.GetCameraMatrix(mCamera);
		}
		m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
	}

if(displayMode==FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE
		||displayMode==FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE)
{
	m_env.GetmodelViewMatrix()->Scale(2.50,1.0,3.3);
	m_env.GetmodelViewMatrix()->Translate(-2.2,0.0,0.0);
}
else if(displayMode==ALL_VIEW_MODE
		||SecondDisplayMode==SECOND_ALL_VIEW_MODE
		||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE
		||fboMode==FBO_ALL_VIEW_MODE
		||fboMode==FBO_ALL_VIEW_559_MODE
		||displayMode==TRIM_MODE)
{
	m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-1.18);//26.2 +1.8
	m_env.GetmodelViewMatrix()->Scale(6.0,1.0,4.58);//6.0
	m_env.GetmodelViewMatrix()->Translate(26.2+2.0,0.0,0);//26.2 +1.8
}

else if(displayMode==TWO_HALF_PANO_VIEW_MODE)
{
	m_env.GetmodelViewMatrix()->Scale(4.0,1.0,4.5);
}

m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-2.0);

 if(displayMode==ALL_VIEW_MODE
		||SecondDisplayMode==SECOND_ALL_VIEW_MODE
		||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE
		||fboMode==FBO_ALL_VIEW_MODE
		||fboMode==FBO_ALL_VIEW_559_MODE
		||displayMode==TRIM_MODE)
{
	 m_env.GetmodelViewMatrix()->Translate(0.0,0.0,2.0);
	 m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-2.1);
}

 if(displayMode==ALL_VIEW_MODE
		||SecondDisplayMode==SECOND_ALL_VIEW_MODE
		||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE
	||fboMode==FBO_ALL_VIEW_MODE
	||fboMode==FBO_ALL_VIEW_559_MODE
	||displayMode==TRIM_MODE)
{
	for(int i=5;i<10;i++)
		{
			petal3[i]=i;
		}
		m_env.GetmodelViewMatrix()->PushMatrix();
		DrawPanel(m_env,needSendData,petal3,mainOrsub);
		m_env.GetmodelViewMatrix()->PopMatrix();
}
else
{
	for(int i=0;i<12;i++)
			{
				petal2[i]=i;
			}
			m_env.GetmodelViewMatrix()->PushMatrix();
			m_env.GetmodelViewMatrix()->Translate(-PanoLen,0.0,0.0);
			DrawPanel(m_env,true,NULL,mainOrsub);
			m_env.GetmodelViewMatrix()->PopMatrix();
}
 /*
 for(int i=0;i<2;i++)
 {
p_ForeSightFacade[i]->SetAlign(3,FORESIGHT_POS_LEFT);
p_ForeSightFacade[i]->Draw(m_env,render.getRulerAngle()->Load());
 }
 */
	{
		m_env.GetmodelViewMatrix()->PopMatrix();//pop camera matrix
	}
	m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::RenderIndividualView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h, bool needSendData)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->LoadIdentity();
	// move h since the shadow dimension is [-1,1], use h/2 if it is [0,1]
	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h);
	m_env.GetmodelViewMatrix()->Scale(w, h, 1.0f);
	DrawIndividualVideo(m_env,needSendData);

	if((CAM_3 == p_BillBoard->m_Direction)&&(SPLIT_VIEW_MODE == displayMode))
	{
		static GLFrame cameraFrame;
		static bool once = true;
		if(once){
			cameraFrame.MoveForward(-40.0f);
			once = false;
			}
		M3DMatrix44f camera ;
		cameraFrame.GetCameraMatrix(camera);
		p_FixedBBD_2M->DrawSingle(m_env,camera,p_FixedBBD_2M);
		p_FixedBBD_5M->DrawSingle(m_env,camera,p_FixedBBD_5M);
		p_FixedBBD_1M->DrawSingle(m_env,camera,p_FixedBBD_1M);
	}
	if( (BillBoard::BBD_FRONT == p_BillBoard->m_Direction) && (SINGLE_PORT_MODE == displayMode)){
			//Draw front left-right corner markers
		p_CornerMarkerGroup->DrawCorner(m_env,CORNER_FRONT_LEFT, pConerMarkerColors[CORNER_FRONT_LEFT]);
		p_CornerMarkerGroup->DrawCorner(m_env,CORNER_FRONT_RIGHT, pConerMarkerColors[CORNER_FRONT_RIGHT]);
	}
	else if( (BillBoard::BBD_REAR == p_BillBoard->m_Direction) && (SINGLE_PORT_MODE == displayMode)){
		//Draw front left-right corner markers
	p_CornerMarkerGroup->DrawCorner(m_env,CORNER_REAR_LEFT, pConerMarkerColors[CORNER_REAR_LEFT]);
	p_CornerMarkerGroup->DrawCorner(m_env,CORNER_REAR_RIGHT, pConerMarkerColors[CORNER_REAR_RIGHT]);
	}

	m_env.GetmodelViewMatrix()->PopMatrix();



}
// get current time, format it into "1997-01-01 13:01:01" and display it
void Render::RenderTimeView(GLEnv &m_env,GLint x, GLint y,GLint w, GLint h)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->LoadIdentity();
	// move h since the shadow dimension is [-1,1], use h/2 if it is [0,1]
	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h);
	m_env.GetmodelViewMatrix()->Scale(w, h, 1.0f);
	{
		const char* pBuf = m_Timebar.GetFTime();
		DrawStringsWithHighLight(m_env,w,h, pBuf, m_Timebar.GetIndicator());
	}
	m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::RenderBillBoardAt(GLEnv &m_env,GLint x, GLint y,GLint w, GLint h)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->LoadIdentity();
	// move h since the shadow dimension is [-1,1], use h/2 if it is [0,1]
	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h);
	m_env.GetmodelViewMatrix()->Scale(w, h, 1.0f);
	{
		p_BillBoard->DrawBillBoard(w,h);
	}
	m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::RenderChineseCharacterBillBoardAt(GLEnv &m_env,GLint x, GLint y,GLint w, GLint h,bool isbottem)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->LoadIdentity();
	// move h since the shadow dimension is [-1,1], use h/2 if it is [0,1]
	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h);
	m_env.GetmodelViewMatrix()->Scale(w, h, 1.0f);
	{
	//	p_ChineseCBillBoard->DoTextureBinding(choice);
		if(isbottem)
		{
			p_ChineseCBillBoard_bottem_pos->DrawBillBoard(w,h);
		}
		else
		p_ChineseCBillBoard->DrawBillBoard(w,h);
	}
	m_env.GetmodelViewMatrix()->PopMatrix();
}


void Render::RenderCompassBillBoardAt(GLEnv &m_env,GLint x, GLint y,GLint w, GLint h)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();

	m_env.GetmodelViewMatrix()->LoadIdentity();
	// move h since the shadow dimension is [-1,1], use h/2 if it is [0,1]

	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, 0.0f);
	m_env.GetmodelViewMatrix()->Scale(w, h, 1.0f);

	{
	//	p_CompassBillBoard->DrawBillBoard(w,h);
	}
	//m_env.GetmodelViewMatrix()->Rotate(10.0, 0.0f, 0.0f, 1.0f);
	m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::RenderVGAView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h, bool needSendData)
{
		glViewport(x,y,w,h);
		glClear(GL_DEPTH_BUFFER_BIT);
		m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);

		m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

		m_env.GetmodelViewMatrix()->PushMatrix();
		m_env.GetmodelViewMatrix()->LoadIdentity();
		// move h since the shadow dimension is [-1,1], use h/2 if it is [0,1]
		M3DMatrix44f camera ;

		VGACameraFrame.GetCameraMatrix(camera);
		m_env.GetmodelViewMatrix()->PushMatrix(VGACameraFrame);
		m_env.GetmodelViewMatrix()->PopMatrix();
		m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h);//-h
		m_env.GetmodelViewMatrix()->Scale(w, h, 1.0f);
		DrawVGAVideo(m_env,needSendData);
		m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::RenderSDIView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h, bool needSendData)
{
		glViewport(x,y,w,h);
		glClear(GL_DEPTH_BUFFER_BIT);
		m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
		m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());
		m_env.GetmodelViewMatrix()->PushMatrix();
		m_env.GetmodelViewMatrix()->LoadIdentity();
			M3DMatrix44f camera ;
		SDICameraFrame.GetCameraMatrix(camera);
		m_env.GetmodelViewMatrix()->PushMatrix(SDICameraFrame);
		m_env.GetmodelViewMatrix()->PopMatrix();
		m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h);//-h
		m_env.GetmodelViewMatrix()->Scale(w, h, 1.0f);
		DrawSDIVideo(m_env,needSendData);
		m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::ChangeMainChosenCamidx(char idx)
{
#if USE_CAP_SPI
	WriteMessage(MSG_TYPE_YUANJING_DATA2,idx );//0~9
#endif
}
void Render::ChangeSubChosenCamidx(char idx)
{
#if USE_CAP_SPI
	WriteMessage(MSG_TYPE_YUANJING_DATA1, idx);//0~9
#endif
}


void Render::RenderChosenView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h, int mainorsub,bool needSendData)
{
		glViewport(x,y,w,h);
		glClear(GL_DEPTH_BUFFER_BIT);
		m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
		m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());
		m_env.GetmodelViewMatrix()->PushMatrix();
		m_env.GetmodelViewMatrix()->LoadIdentity();
		M3DMatrix44f camera ;
		ChosenCameraFrame.GetCameraMatrix(camera);
		m_env.GetmodelViewMatrix()->PushMatrix(ChosenCameraFrame);
		m_env.GetmodelViewMatrix()->PopMatrix();
		m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h);//-h
		m_env.GetmodelViewMatrix()->Scale(w, h, 1.0f);
		DrawChosenVideo(m_env,needSendData,mainorsub);
		m_env.GetmodelViewMatrix()->PopMatrix();
}


void PrintGLText(GLint x, GLint y, const char *string)
{
    if (string == NULL)
    {
    	printf("error in text! \n");
        return;
    }

    GLuint ListBase = glGenLists(96);
    glPushAttrib(GL_LIST_BIT | GL_CURRENT_BIT |  GL_ENABLE_BIT | GL_LIGHTING_BIT);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glColor4f(1.0, 0.0, 0.0, 0.5);

    glRasterPos2i(x, y);

    glListBase(ListBase - 32);
    glCallLists(strlen(string), GL_UNSIGNED_BYTE, string);
    glPopAttrib();
}
void glText(double x, double y, char *str)
{
    glColor3f(1.0, 0.0, 0.0);
    glLineWidth(2);
	glRasterPos2f(x,y);
	while(*str){
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,*str);
		str++;
	}
}
void Render::DrawInitView(GLEnv &m_env,Rect* rec, bool needSendData)
{

	int pic_count_hor=CAM_COUNT/2;
	int wid = rec->width/pic_count_hor, hei = rec->height/2;
	int startx,starty;
	for(int i=0; i<CAM_COUNT; i++)
	{
		startx = rec->x + wid*(i%pic_count_hor);
		starty = rec->y + hei*(1 - i/pic_count_hor);
		DrawSigleScale(m_env,new Rect(startx,starty,wid,hei), i, needSendData);
	}

	if(common.Scaned())
	{
		char text[]="O K     ";
		DrawCordsView(m_env,new Rect(rec->x+rec->width/2, rec->y+rec->height*2/5,
								rec->width/16,rec->height/10),
							text);
	}

	for(int i=0; i<CAM_COUNT; i++)
	{
		if(common.getStateChannel(i))
		{
			char text[]="Pass     ";
			startx = wid*(i%4) + (wid/pic_count_hor);
			starty = hei*(1 - i/pic_count_hor) ;
			DrawCordsView(m_env,new Rect(startx, starty,rec->width/24,rec->height/10),text);
		}

		if(picSaveState[i]==true)
		{
			char text_save[]="Save     ";
			startx = wid*(i%(CAM_COUNT/2)) + (wid/pic_count_hor);
			if(i<CAM_COUNT/2)
			{
				starty = hei*1.5 ;
			}
			else
			{
				starty = hei*0.5 ;
			}
			DrawCordsView(m_env,new Rect(startx, starty,rec->width/24,rec->height/10),text_save);
		}
	}

	if(EnterSinglePictureSaveMode==true)
	{
		char text_show[]="Enter    ";
		startx = 0;//wid/24;
		starty = hei*8/10 ;
		DrawCordsView(m_env,new Rect(startx, starty,rec->width/12,rec->height/10),text_show);
		char text_number[20];
		sprintf(text_number,"n:%.2d  ",enterNumberofCam);
		DrawCordsView(m_env,new Rect(startx, starty*0.8,rec->width/12,rec->height/10),text_number);


		for(int j=0;j<CAM_COUNT/2;j++)
		{

			startx = wid*(j%(CAM_COUNT/2)) + (wid/pic_count_hor);
			starty=hei*0.5;
			sprintf(text_number,"%.2d      ",j+CAM_COUNT/2);
			DrawCordsView(m_env,new Rect(startx, starty*0.3,rec->width/24,rec->height/10),text_number);
			sprintf(text_number,"%.2d      ",j);
			DrawCordsView(m_env,new Rect(startx, starty*2.3,rec->width/24,rec->height/10),text_number);
		}
	}
}
void Render::DrawCordsView(GLEnv &m_env,Rect* rec, char* text)
{
	glViewport(rec->x, rec->y, rec->width, rec->height);
	DrawCords(m_env,rec->width, rec->height, text);
}

void Render::DrawAngleCordsView(GLEnv &m_env,Rect* rec, char* text,float toScale)
{
	glViewport(rec->x, rec->y, rec->width, rec->height);
	DrawAngleCords(m_env,rec->width, rec->height, text,toScale);
}

void Render::DrawSigleScale(GLEnv &m_env,Rect* rec, GLint idx, bool needSendData)
{
	glViewport(rec->x, rec->y, rec->width, rec->height);
	m_env.GetviewFrustum()->SetPerspective(90.0f, float(rec->width) / float(rec->height), 1.0f, 4000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->LoadIdentity();
	// move h since the shadow dimension is [-1,1], use h/2 if it is [0,1]
	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -rec->height);
	m_env.GetmodelViewMatrix()->Scale(rec->width, rec->height, 1.0f);

	DrawSigleVideo(m_env,idx, needSendData);
	m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::DrawSigleVideo(GLEnv &m_env,GLint idx, bool needSendData)
{
	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->Rotate(180.0f, 0.0f, 0.0f, 1.0f);
	m_env.GetmodelViewMatrix()->Rotate(180.0f,0.0f, 1.0f, 0.0f);
	glActiveTexture(GL_TextureIDs[idx]);

	if(needSendData){
		m_env.Getp_PBOMgr()->sendData(m_env,textures[idx], (PFN_PBOFILLBUFFER)capturePanoCam,idx);
	}
	else{
		glBindTexture(GL_TEXTURE_2D, textures[idx]);
	}

	shaderManager.UseStockShader(GLT_SHADER_TEXTURE_REPLACE,m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), idx);
	m_env.Getp_shadowBatch()->Draw();
	m_env.GetmodelViewMatrix()->PopMatrix();
}
void Render::RenderExtensionBillBoardAt(GLEnv &m_env,GLint x, GLint y,GLint w, GLint h)
{
	glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->LoadIdentity();
	// move h since the shadow dimension is [-1,1], use h/2 if it is [0,1]
	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h);
	m_env.GetmodelViewMatrix()->Scale(w, h, 1.0f);
	{
		p_BillBoardExt->DrawBillBoard(w,h);
	}
	m_env.GetmodelViewMatrix()->PopMatrix();
}
///////////////////////////////////////////////////////////////////////////////
// Called to draw scene

void Render::ChangeTelMode()
{
	static int  lastMode=-1;
	lastMode=displayMode;
	if(lastMode!=TELESCOPE_FRONT_MODE
		&&lastMode!=TELESCOPE_RIGHT_MODE
		&&lastMode!=TELESCOPE_BACK_MODE
		&&lastMode!=TELESCOPE_LEFT_MODE)
	{
		displayMode=TELESCOPE_FRONT_MODE;
	}
#if USE_UART
		if(zodiac_msg.IsToChangeTelMode())
		{
			DISPLAYMODE nextMode;
			if(zodiac_msg.GetTelBreak()==TEL_LEFT_BREAK)
			{
				nextMode = DISPLAYMODE(((int)displayMode-1) % TOTAL_MODE_COUNT);
				if(nextMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE)
				{
					 nextMode = TELESCOPE_LEFT_MODE;
				}
			}
			else if(zodiac_msg.GetTelBreak()==TEL_RIGHT_BREAK)
			{
				nextMode = DISPLAYMODE(((int)displayMode+1) % TOTAL_MODE_COUNT);
				if(nextMode==SDI1_WHITE_BIG_VIEW_MODE)
				{
					 nextMode = TELESCOPE_FRONT_MODE;
				}
			}
			zodiac_msg.DisableChangeTelMode();
			displayMode=nextMode;
		}
#endif
}

void Render::sendBack()
{
#if USE_UART
	if(ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.turn==1)
	{
		if(displayMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE)
		{
			SendBackXY(p_ForeSightFacade->GetMil());
		}
		else if(displayMode==TELESCOPE_FRONT_MODE)
		{
			SendBackXY(p_ForeSightFacade2->GetTelMil(0));
		}
		else if(displayMode==TELESCOPE_RIGHT_MODE)
		{
			SendBackXY(p_ForeSightFacade2->GetTelMil(1));
		}
		else if(displayMode==TELESCOPE_BACK_MODE)
		{
			SendBackXY(p_ForeSightFacade2->GetTelMil(2));
		}
		else if(displayMode==TELESCOPE_LEFT_MODE)
		{
			SendBackXY(p_ForeSightFacade2->GetTelMil(3));
		}

		else if(displayMode==	VGA_WHITE_VIEW_MODE
								||displayMode==VGA_HOT_BIG_VIEW_MODE
								||displayMode==VGA_HOT_SMALL_VIEW_MODE
								||displayMode==VGA_FUSE_WOOD_LAND_VIEW_MODE
								||displayMode==VGA_FUSE_GRASS_LAND_VIEW_MODE
								||displayMode==VGA_FUSE_SNOW_FIELD_VIEW_MODE
								||displayMode==VGA_FUSE_DESERT_VIEW_MODE
								||displayMode==VGA_FUSE_CITY_VIEW_MODE)
		{
			sendTrackSpeed(1024,768);
		}
		else if(displayMode==SDI1_WHITE_BIG_VIEW_MODE
								||displayMode==SDI1_WHITE_SMALL_VIEW_MODE
								||displayMode==SDI2_HOT_BIG_VIEW_MODE
								||displayMode==SDI2_HOT_SMALL_VIEW_MODE)
		{
			sendTrackSpeed(1920,1080);
		}
		else if(displayMode==PAL1_WHITE_BIG_VIEW_MODE
								||displayMode==PAL1_WHITE_SMALL_VIEW_MODE
								||displayMode==PAL2_HOT_BIG_VIEW_MODE
								||displayMode==PAL2_HOT_SMALL_VIEW_MODE)
		{
			sendTrackSpeed(720,576);
		}
	}
#endif
}
void Render::SetdisplayMode( )
{
#if 0

	switch(zodiac_msg.GetdispalyMode())
		{
			case RECV_VGA_WHITE_MODE :
		{
				displayMode=VGA_WHITE_VIEW_MODE;
				break;
		}
			case RECV_VGA_HOT_BIG_MODE :
			{
					displayMode=VGA_HOT_BIG_VIEW_MODE;
					break;
			}
			case RECV_VGA_HOT_SMALL_MODE :
			{
					displayMode=VGA_HOT_SMALL_VIEW_MODE;
					break;
			}
			case RECV_VGA_FUSE_WOOD_MODE :
			{
					displayMode=VGA_FUSE_WOOD_LAND_VIEW_MODE;
					break;
			}
			case RECV_VGA_FUSE_GRASS_MODE :
			{
					displayMode=VGA_FUSE_GRASS_LAND_VIEW_MODE;
					break;
			}
			case RECV_VGA_FUSE_SNOW_MODE :
			{
					displayMode=VGA_FUSE_SNOW_FIELD_VIEW_MODE;
					break;
			}
			case RECV_VGA_FUSE_DESERT_MODE:
			{
					displayMode=VGA_FUSE_DESERT_VIEW_MODE;
					break;
			}
			case RECV_VGA_FUSE_CITY_MODE:
			{
					displayMode=VGA_FUSE_CITY_VIEW_MODE;
					break;
			}
			case RECV_WHOLE_MODE:
			{
				displayMode=ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE;
				break;
			}
			case RECV_TELESCOPE_MODE:
			{
				ChangeTelMode();
				break;
			}
			case RECV_SDI1_WHITE_BIG_MODE:
			{
					displayMode=SDI1_WHITE_BIG_VIEW_MODE;
					break;
			}
			case 	RECV_SDI1_WHITE_SMALL_MODE:
			{
				displayMode=SDI1_WHITE_SMALL_VIEW_MODE;
				break;
			}
			case 	RECV_SDI2_HOT_BIG_MODE:
			{
					displayMode=SDI2_HOT_BIG_VIEW_MODE;
					break;
				}
			case 	RECV_SDI2_HOT_SMALL_MODE:
			{
				displayMode=SDI2_HOT_SMALL_VIEW_MODE;
				break;
			}
			case	RECV_PAL1_WHITE_BIG_MODE:
			{
					displayMode=PAL1_WHITE_BIG_VIEW_MODE;
					break;
				}
			case RECV_PAL1_WHITE_SMALL_MODE:
			{
				displayMode=PAL1_WHITE_SMALL_VIEW_MODE;
				break;
			}
			case RECV_PAL2_HOT_BIG_MODE:
			{
					displayMode=PAL2_HOT_BIG_VIEW_MODE;
					break;
				}
			case RECV_PAL2_HOT_SMALL_MODE:
			{
				displayMode=PAL2_HOT_SMALL_VIEW_MODE;
					break;
			}
			case RECV_ENABLE_TRACK:
			{
					break;
			}
		}
#endif
}


void Render::RenderScene(void)
{
static bool setpriorityOnce=true;
if(setpriorityOnce)
{
	setCurrentThreadHighPriority(THREAD_L_M_RENDER);
	setpriorityOnce=false;
}
	GLEnv &env=env1;
	bool bShowDirection = false, isBillBoardExtOn = false;
	bool needSendData = true;
	int billBoardx = 0, billBoardy = g_windowHeight*15/16;//7/8;
	int extBillBoardx = 0, extBillBoardy = g_windowHeight*15/16;//*7/8;
	static int last_mode=0;

#ifdef GL_TIME_STAMP
	GLuint queries[4];
	GLuint startTime, rearTime, birdTime, fboTime;
	glGenQueries(3, queries);

	glBeginQuery(GL_TIME_ELAPSED, queries[0]);
#endif

#ifdef GET_ALARM_AERA
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, pVehicle->msFBO);
#endif
	// Clear the window with current clearing color
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

#if 1
	if(env.Getp_FboPboFacade()->IsFboUsed())
	{
		env.Getp_FBOmgr()->SetDrawBehaviour(&render);
		env.Getp_FboPboFacade()->DrawAndGet();
	}

#endif
	#if 1
#if USE_UART
	zodiac_msg.setXspeedandMove();
	zodiac_msg.setYspeedandMove();
	zodiac_msg.save0pos();
	SetdisplayMode( );
	sendBack();
#endif
	if(displayMode==VGA_WHITE_VIEW_MODE
		||displayMode==VGA_HOT_BIG_VIEW_MODE
		||displayMode==VGA_HOT_SMALL_VIEW_MODE
		||displayMode==VGA_FUSE_WOOD_LAND_VIEW_MODE
		||displayMode==VGA_FUSE_GRASS_LAND_VIEW_MODE
		||displayMode==VGA_FUSE_SNOW_FIELD_VIEW_MODE
		||displayMode==VGA_FUSE_DESERT_VIEW_MODE
		||displayMode==VGA_FUSE_CITY_VIEW_MODE
		||displayMode==SDI1_WHITE_BIG_VIEW_MODE
		||displayMode==SDI1_WHITE_SMALL_VIEW_MODE
		||displayMode==SDI2_HOT_BIG_VIEW_MODE
		||displayMode==SDI2_HOT_SMALL_VIEW_MODE
		||displayMode==PAL1_WHITE_BIG_VIEW_MODE
		||displayMode==PAL1_WHITE_SMALL_VIEW_MODE
		||displayMode==PAL2_HOT_BIG_VIEW_MODE
		||displayMode==PAL2_HOT_SMALL_VIEW_MODE
)
	{
		writeFirstMode(displayMode);
	}

	if(displayMode==	SDI1_WHITE_BIG_VIEW_MODE
			||displayMode==SDI1_WHITE_SMALL_VIEW_MODE
			||displayMode==SDI2_HOT_BIG_VIEW_MODE
			||displayMode==SDI2_HOT_SMALL_VIEW_MODE)
	{
		isinSDI=true;
	}
	else
	{
		isinSDI=false;
	}
	switch(displayMode)
	{
	case SPLIT_VIEW_MODE:
	{
		RenderBirdView(env,0,0, g_windowWidth*BIRD_VIEW_WIDTH_RATE, g_windowHeight*BIRD_VIEW_HEIGHT_RATE, needSendData);
#ifdef GL_TIME_STAMP
		glEndQuery(GL_TIME_ELAPSED);

		glBeginQuery(GL_TIME_ELAPSED, queries[1]);
#endif

		RenderIndividualView(env,g_windowWidth*BIRD_VIEW_WIDTH_RATE, g_windowHeight*OTHRE_VIEW_HEIGHT_RATE, g_windowWidth*OTHER_VIEW_WIDTH_RATE, g_windowHeight/2, false);
#ifdef GL_TIME_STAMP
		glEndQuery(GL_TIME_ELAPSED);

		glBeginQuery(GL_TIME_ELAPSED, queries[2]);
#endif
        bShowDirection = true;
		billBoardx = g_windowWidth*BIRD_VIEW_WIDTH_RATE*9/10;
		billBoardy = g_windowHeight*11/16;// g_windowHeight*7/8 -  g_windowHeight/4;
	}
	break;
	case SINGLE_PORT_MODE:
	{
#ifdef GL_TIME_STAMP
		glEndQuery(GL_TIME_ELAPSED);
		glBeginQuery(GL_TIME_ELAPSED, queries[1]);
#endif
		bShowDirection = true;
		RenderIndividualView(env,0,0,g_windowWidth, g_windowHeight, needSendData);

#ifdef GL_TIME_STAMP
		glEndQuery(GL_TIME_ELAPSED);
		glBeginQuery(GL_TIME_ELAPSED, queries[2]);
#endif
	}
	break;
	case PREVIEW_MODE:
	{
	//	RenderDectorView(env,0,0,g_windowWidth,g_windowHeight);
	    RenderSingleView(env,0,0,g_windowWidth, g_windowHeight);
	}
	break;
	case TRIPLE_VIEW_MODE:
	{		
		bShowDirection = isBillBoardExtOn = true;
		billBoardx = extBillBoardx = g_windowWidth*BIRD_VIEW_WIDTH_RATE*9/10;
		extBillBoardy  = g_windowHeight*15/16;//7/8;
		billBoardy = extBillBoardy - g_windowHeight/2;
		RenderAnimationToBirdView(env,0,0, g_windowWidth*BIRD_VIEW_WIDTH_RATE, g_windowHeight, needSendData);
		RenderPresetViewByRotating(env,g_windowWidth*BIRD_VIEW_WIDTH_RATE,0,g_windowWidth*OTHER_VIEW_WIDTH_RATE, g_windowHeight/2, false);
//		RenderExtensionView(g_windowWidth*BIRD_VIEW_WIDTH_RATE, g_windowHeight/2, g_windowWidth*OTHER_VIEW_WIDTH_RATE, g_windowHeight/2, needSendData);
	}
	break;
	case FREE_VIEW_MODE:
	{
		bShowDirection = true;
		RenderFreeView(env,0,0,g_windowWidth, g_windowHeight, needSendData);
		RenderCordsView(env,0,0, g_windowWidth, g_windowHeight*1/8);

	}
	break;
	case BACK_VIEW_MODE:
	{
		bShowDirection = true;
		p_BillBoard->m_Direction = BillBoard::BBD_REAR;
		RenderRearTopView(env,0, 0, g_windowWidth, g_windowHeight, needSendData);

	}
	break;
	case EXTENSION_VIEW_MODE:
	{
//		RenderExtensionView(env,0, 0, g_windowWidth, g_windowHeight, needSendData);
		isBillBoardExtOn = true;
	}
	break;
	case ALL_ADD_712_MODE:
		RenderBirdView(env,0,0, g_windowWidth*BIRD_VIEW_WIDTH_RATE, g_windowHeight*BIRD_VIEW_HEIGHT_RATE, needSendData);
//		RenderExtensionView(env,g_windowWidth*BIRD_VIEW_WIDTH_RATE, g_windowHeight*OTHRE_VIEW_HEIGHT_RATE, g_windowWidth*OTHER_VIEW_WIDTH_RATE, g_windowHeight/2, needSendData);

		break;
	case CENTER_VIEW_MODE:
	{
		RenderCenterView(env,0,0,g_windowWidth, g_windowHeight);

	}
		break;
	case PANO_VIEW_MODE:
		if(last_mode!=PANO_VIEW_MODE)
		{
			InitScanAngle();//each time enter pano_view_mode read the angle file again

		}
		RenderRegionPanelView(env,g_windowWidth/2-(g_windowWidth*getScanRegionAngle()/360.0f)*4.0/2,g_windowHeight*1.0/4.0,
				g_windowWidth*getScanRegionAngle()*4.0/360.0f,g_windowHeight*3.0/4.0);
		RenderRulerView(env,0,g_windowHeight*3.5/20.0,g_windowWidth,g_windowHeight*0.9/10.0,RULER_180);
		RenderPanoView(env,0,g_windowHeight*0.0/4.0,g_windowWidth, g_windowHeight*1.0/4.0);


		RenderCompassView(env,0,g_windowHeight/4.0,g_windowWidth/10.0,g_windowWidth/10.0);
		//RenderCompassView(env,0,g_windowHeight/4.0,g_windowWidth,g_windowHeight/10.0);
		break;
	case TWO_HALF_PANO_VIEW_MODE:
		if(last_mode!=PANO_VIEW_MODE)
		{
			InitScanAngle();//each time enter pano_view_mode read the angle file again
		}
		RenderRightPanoView(env,0,0,g_windowWidth, g_windowHeight/2.0);
		RenderLeftPanoView(env,0,g_windowHeight/2.0,g_windowWidth, g_windowHeight/2.0);
		RenderRulerView(env,0,g_windowHeight-g_windowHeight*0.6/10.0,g_windowWidth,g_windowHeight*0.9/10.0,RULER_90);
		RenderRulerView(env,0,g_windowHeight/2.0-g_windowHeight*0.6/10.0,g_windowWidth,g_windowHeight*0.9/10.0,RULER_90);

		break;
	case FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE:
		if(last_mode!=FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE)
		{
			InitScanAngle();//each time enter pano_view_mode read the angle file again
		}

		RenderRightPanoView(env,0,g_windowHeight*200.0/1080.0,g_windowWidth*1920.0/1920.0, g_windowHeight*360.0/1080.0);
		RenderLeftPanoView(env,0,g_windowHeight*(200.0+360.0)/1080.0,g_windowWidth*1920.0/1920.0, g_windowHeight*360.0/1080.0);
		//glEnable(GL_SCISSOR_TEST);
		//glScissor(env,g_windowWidth*(1920.0-704.0)/1920.0, g_windowHeight*(360.0)/1080.0, g_windowWidth*704.0/1080.0, g_windowHeight*(576.0-360.0)/1080.0);
		//glClear(GL_DEPTH_BUFFER_BIT);
		//glDisable(GL_SCISSOR_TEST);
//		RenderExtensionView(env,g_windowWidth*(1920.0-704.0)/1920.0, g_windowHeight*0.0/1080.0, g_windowWidth*704.0/1920.0,  g_windowHeight*576.0/1080.0, needSendData);
	//	RenderCompassView(env,0,0,500,500);
		//		RenderCompassView(env,0,0,g_windowWidth/3.0,g_windowHeight/3.0);
		break;
	case FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE:
		if(last_mode!=FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE)
		{
			InitScanAngle();//each time enter pano_view_mode read the angle file again
		}
		RenderRightPanoView(env,0,g_windowHeight*0.0/1080.0,g_windowWidth*1920.0/1920.0, g_windowHeight*540.0/1080.0);
		RenderLeftPanoView(env,0,g_windowHeight*540.0/1080.0,g_windowWidth*1920.0/1920.0, g_windowHeight*540.0/1080.0);

		//		RenderExtensionView(env,g_windowWidth*(1920.0-704.0)/1920.0, g_windowHeight*0.0/1080.0, g_windowWidth*704.0/1920.0, g_windowHeight*360.0/1080.0, needSendData);
		break;
	case INIT_VIEW_MODE:
		DrawInitView(env,new Rect(0, 0, g_windowWidth, g_windowHeight), needSendData);
		break;
	case ALL_VIEW_MODE:
	{
#if		MVDECT
		if(mv_detect.CanUseMD(MAIN))
		{
		//	mv_detect.SetoutRect();
		}
#endif
		tIdle.threadIdle(MAIN_CN);
		env.Getp_FboPboFacade()->Render2Front(MAIN,g_windowWidth,g_windowHeight);
//if(g_windowHeight==768)
{
	//	RenderRightForeSightView(env,0,g_windowHeight*538.0/768.0,g_windowWidth, g_windowHeight*116.0/768.0,MAIN);
		//RenderLeftForeSightView(env,0,g_windowHeight*655.0/768.0,g_windowWidth, g_windowHeight*115.0/768.0,MAIN);
}
//else
{
	RenderRightForeSightView(env,0,g_windowHeight*643.0/1080.0,g_windowWidth, g_windowHeight*216.0/1080.0,MAIN);
	RenderLeftForeSightView(env,0,g_windowHeight*864.0/1080.0,g_windowWidth, g_windowHeight*216.0/1080.0,MAIN);
}



			{

				int x=g_windowWidth/2,y=g_windowHeight/4,w=g_windowWidth/2,h=g_windowHeight/2;

				x=0;
				y=0;
				w=g_windowWidth;
				h=g_windowHeight;

				glViewport(x,y,w,h);
			//	m_env.GetviewFrustum()->SetPerspective(27.0f, float(w) / float(h), 1.0f, 100.0f);
				env.GetviewFrustum()->SetPerspective(27.0f,  float(w) / float(h), 1.0f, 500.0f);

				env.GetprojectionMatrix()->LoadMatrix(env.GetviewFrustum()->GetProjectionMatrix());

				env.GetmodelViewMatrix()->PushMatrix();
				
				{
		 			   
		    			
/*					
					for(int i=0;i<36;i++)
					{
						shaderManager.UseStockShader(GLT_SHADER_FLAT, env.GettransformPipeline()->GetModelViewProjectionMatrix(), vRed);
						array_round_point[i].Draw();
					}
*/
				}
				
				env.GetmodelViewMatrix()->PopMatrix();



				x=g_windowWidth/2,y=g_windowHeight/4,w=g_windowWidth/2,h=g_windowHeight/2;
				glViewport(x,y,w,h);
			//	m_env.GetviewFrustum()->SetPerspective(27.0f, float(w) / float(h), 1.0f, 100.0f);
				env.GetviewFrustum()->SetPerspective(27.0f,  float(w) / float(h), 1.0f, 500.0f);

				env.GetprojectionMatrix()->LoadMatrix(env.GetviewFrustum()->GetProjectionMatrix());

				env.GetmodelViewMatrix()->PushMatrix();

				p_ChineseCBillBoard->ChooseTga=STATE_LABEL2_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth/2, g_windowHeight/4, g_windowWidth/2, g_windowHeight*1.2/2);
				p_ChineseCBillBoard->ChooseTga=STATE_LABEL_T;
RenderChineseCharacterBillBoardAt(env,g_windowWidth/2-g_windowWidth/5, g_windowHeight/4, g_windowWidth/2, g_windowHeight*1.2/2);
				
				int point_hor_delta=g_windowWidth/4+g_windowWidth/36;
				int point_ver_delta=0;

				
				for(int cx_i=0;cx_i<6;cx_i++)
				{
					for(int cx_j=0;cx_j<3;cx_j++)
					{
						if(state_label_data[cx_i][cx_j]==1)
						{

							p_ChineseCBillBoard->ChooseTga=POINT_RED_T;


						}
						else if(state_label_data[cx_i][cx_j]==0)
						{
							p_ChineseCBillBoard->ChooseTga=POINT_GREEN_T;
						}
						else
						{
							p_ChineseCBillBoard->ChooseTga=POINT_GREY_T;
						}
						
RenderChineseCharacterBillBoardAt(env,g_windowWidth/2-g_windowWidth/12+(cx_j)*2*g_windowWidth/60+point_hor_delta-0.8*2*g_windowWidth/60,
					g_windowHeight/4+g_windowHeight/10-cx_i*1.2*g_windowHeight/60+5, g_windowWidth/12, g_windowHeight/8);
					}
									
				}


				point_hor_delta+=8*g_windowWidth/48;

				for(int cx_i=0;cx_i<6;cx_i++)
				{
					for(int cx_j=0;cx_j<3;cx_j++)
					{

						if(state_label_data[cx_i+6][cx_j]==1)
						{

							p_ChineseCBillBoard->ChooseTga=POINT_RED_T;


						}
						else if(state_label_data[cx_i+6][cx_j]==0)
						{
							p_ChineseCBillBoard->ChooseTga=POINT_GREEN_T;
						}
						else
						{
							p_ChineseCBillBoard->ChooseTga=POINT_GREY_T;
						}
RenderChineseCharacterBillBoardAt(env,g_windowWidth/2-g_windowWidth/12+(cx_j)*2*g_windowWidth/60+point_hor_delta,
					g_windowHeight/4+g_windowHeight/10-cx_i*1.2*g_windowHeight/60+5, g_windowWidth/12, g_windowHeight/8);
					}
									
				}


				env.GetmodelViewMatrix()->PopMatrix();

			}


/*	p_ChineseCBillBoard->ChooseTga=TURRET_T;
	RenderChineseCharacterBillBoardAt(env,g_windowWidth*160.0/1920.0, g_windowHeight*250.0/1080.0, g_windowWidth*800.0/1920.0,g_windowHeight*1000.0/1920.0);
	p_ChineseCBillBoard->ChooseTga=PANORAMIC_MIRROR_T;
	RenderChineseCharacterBillBoardAt(env,g_windowWidth*600.0/1920.0, g_windowHeight*250.0/1080.0, g_windowWidth*800.0/1920.0,g_windowHeight*1000.0/1920.0);
*/
	//RenderRightPanoView(env,0,g_windowHeight*864.0/1080.0,g_windowWidth, g_windowHeight*216.0/1080.0,MAIN);
	//	RenderLeftPanoView(env,0,g_windowHeight*648.0/1080.0,g_windowWidth, g_windowHeight*216.0/1080.0,MAIN);
			break;
	}
	case TRIM_MODE:
		env.Getp_FboPboFacade()->Render2Front(MAIN,g_windowWidth,g_windowHeight);
		p_ChineseCBillBoard->ChooseTga=DEBUG_T;
		RenderChineseCharacterBillBoardAt(env,g_windowWidth-g_windowWidth/0.9, g_windowHeight*1/8, g_windowHeight, g_windowHeight);

		break;
	case CHOSEN_VIEW_MODE:
		tIdle.threadRun(MAIN_CN);
		RenderChosenView(env,0,0,g_windowWidth, g_windowHeight,MAIN,true);
		break;





	case ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE:
	{
			RenderRightPanoView(env,0,g_windowHeight*440.0/1080.0,g_windowWidth, g_windowHeight*320.0/1080.0);
			RenderLeftPanoView(env,0,g_windowHeight*760.0/1080.0,g_windowWidth, g_windowHeight*320.0/1080.0);
	//	RenderRightPanoView(env,0,g_windowHeight*0.0/1080.0,g_windowWidth*1920.0/1920.0, g_windowHeight*540.0/1080.0);
		//	RenderLeftPanoView(env,0,g_windowHeight*540.0/1080.0,g_windowWidth*1920.0/1920.0, g_windowHeight*540.0/1080.0);
			PrepareAlarmAera(env,0,0,g_windowWidth,g_windowHeight);
			p_ForeSightFacade[MAIN]->Reset(ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE);
		    RenderRulerView(env,g_windowWidth*0/1920.0,g_windowHeight*0.0/1080.0,g_windowWidth,g_windowHeight*140.0/1080,RULER_90);
		    RenderRulerView(env,g_windowWidth*0/1920.0,g_windowHeight*540/1080.0,g_windowWidth,g_windowHeight*140.0/1080,RULER_180);
		    RenderOnetimeView(env,g_windowWidth*60.0/1920.0,g_windowHeight*20.0/1080.0,g_windowWidth*1000.0/1920.0, g_windowHeight*400.0/1080.0);
	//		RenderTwotimesView(env,g_windowWidth*1120.0/1920.0,g_windowHeight*20.0/1080.0,g_windowWidth*500.0/1920.0, g_windowHeight*400.0/1080.0);
//		RenderCompassView(env,g_windowWidth*1615.0/1920.0,g_windowHeight*-15/1080.0,g_windowWidth*290.0/1920.0,g_windowWidth*290.0/1920.0);
//			RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
			break;
	}

	case	TELESCOPE_FRONT_MODE:
		tIdle.threadIdle(MAIN_CN);
			p_ForeSightFacade2[MAIN]->Reset(TELESCOPE_FRONT_MODE);
		    RenderRulerView(env,-g_windowWidth*3.0/1920.0,g_windowHeight*980.0/1080.0,g_windowWidth,g_windowHeight*140.0/1080.0,RULER_45);
			RenderPanoTelView(env,0,g_windowHeight*478.0/1080,g_windowWidth, g_windowHeight*592.0/1080.0,FRONT);
#if			MVDECT
			if(mv_detect.CanUseMD(MAIN))
			{
			//		mv_detect.SetoutRect();
		//		TargectTelView(env,g_windowWidth*60/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*480.0/1920.0, g_windowHeight*400.0/1080.0,0,0);
		//		TargectTelView(env,g_windowWidth*560/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*480.0/1920.0, g_windowHeight*400.0/1080.0,1,1);
			}
#endif
			//	RenderTwotimesView(env,g_windowWidth*60/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*1000.0/1920.0, g_windowHeight*400.0/1080.0);
	//		RenderFourtimesTelView(env,g_windowWidth*1120.0/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*500.0/1920.0, g_windowHeight*400.0/1080.0);
	//		RenderCompassView(env,g_windowWidth*1615.0/1920.0,g_windowHeight*-15/1080.0,g_windowWidth*290.0/1920.0,g_windowWidth*290.0/1920.0);
			RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
			break;
		case	TELESCOPE_RIGHT_MODE:
			tIdle.threadIdle(MAIN_CN);
			p_ForeSightFacade2[MAIN]->Reset(TELESCOPE_RIGHT_MODE);
			   RenderRulerView(env,-g_windowWidth*3.0/1920.0,g_windowHeight*980.0/1080.0,g_windowWidth,g_windowHeight*140.0/1080.0,RULER_45);
				RenderPanoTelView(env,0,g_windowHeight*478.0/1080,g_windowWidth, g_windowHeight*592.0/1080.0,RIGHT);
#if MVDECT
				if(mv_detect.CanUseMD(MAIN))
						{
			//			mv_detect.SetoutRect();
			//				TargectTelView(env,g_windowWidth*60/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*480.0/1920.0, g_windowHeight*400.0/1080.0,0,0);
			//				TargectTelView(env,g_windowWidth*560/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*480.0/1920.0, g_windowHeight*400.0/1080.0,1,1);
						}
#endif
				//		RenderTwotimesView(env,g_windowWidth*60/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*1000.0/1920.0, g_windowHeight*400.0/1080.0);
		//		RenderFourtimesTelView(env,g_windowWidth*1120.0/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*500.0/1920.0, g_windowHeight*400.0/1080.0);
		//		RenderCompassView(env,g_windowWidth*1615.0/1920.0,g_windowHeight*-15/1080.0,g_windowWidth*290.0/1920.0,g_windowWidth*290.0/1920.0);
				RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
				break;
		case	TELESCOPE_BACK_MODE:
			tIdle.threadIdle(MAIN_CN);
			p_ForeSightFacade2[MAIN]->Reset(TELESCOPE_BACK_MODE);
			   RenderRulerView(env,-g_windowWidth*3.0/1920.0,g_windowHeight*980.0/1080.0,g_windowWidth,g_windowHeight*140.0/1080.0,RULER_45);
			   RenderPanoTelView(env,0,g_windowHeight*478.0/1080,g_windowWidth, g_windowHeight*592.0/1080.0,BACK);
#if MVDECT
			   if(mv_detect.CanUseMD(MAIN))
						{
			//			mv_detect.SetoutRect();
			//				TargectTelView(env,g_windowWidth*60/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*480.0/1920.0, g_windowHeight*400.0/1080.0,0,0);
			//				TargectTelView(env,g_windowWidth*560/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*480.0/1920.0, g_windowHeight*400.0/1080.0,1,1);
						}
#endif
			   //		RenderTwotimesView(env,g_windowWidth*60/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*1000.0/1920.0, g_windowHeight*400.0/1080.0);
		//		RenderFourtimesTelView(env,g_windowWidth*1120.0/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*500.0/1920.0, g_windowHeight*400.0/1080.0);
		//		RenderCompassView(env,g_windowWidth*1615.0/1920.0,g_windowHeight*-15/1080.0,g_windowWidth*290.0/1920.0,g_windowWidth*290.0/1920.0);
				RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
				break;
		case	TELESCOPE_LEFT_MODE:
			tIdle.threadIdle(MAIN_CN);
			p_ForeSightFacade2[MAIN]->Reset(TELESCOPE_LEFT_MODE);
			  RenderRulerView(env,-g_windowWidth*3.0/1920.0,g_windowHeight*980.0/1080.0,g_windowWidth,g_windowHeight*140.0/1080.0,RULER_45);
			RenderPanoTelView(env,0,g_windowHeight*478.0/1080,g_windowWidth, g_windowHeight*592.0/1080.0,LEFT);
#if MVDECT
				if(mv_detect.CanUseMD(MAIN))
						{
			//		mv_detect.SetoutRect();
			//				TargectTelView(env,g_windowWidth*60/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*480.0/1920.0, g_windowHeight*400.0/1080.0,0,0);
				//			TargectTelView(env,g_windowWidth*560/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*480.0/1920.0, g_windowHeight*400.0/1080.0,1,1);
						}
#endif
				//		RenderTwotimesView(env,g_windowWidth*60/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*1000.0/1920.0, g_windowHeight*400.0/1080.0);
				//		RenderFourtimesTelView(env,g_windowWidth*1120.0/1920.0,g_windowHeight*39.0/1080.0,g_windowWidth*500.0/1920.0, g_windowHeight*400.0/1080.0);
		//		RenderCompassView(env,g_windowWidth*1615.0/1920.0,g_windowHeight*-15/1080.0,g_windowWidth*290.0/1920.0,g_windowWidth*290.0/1920.0);
				RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
				break;



	case CHECK_MYSELF:
	{
		static bool Once=true;
		if(Once)
		{
			start_SelfCheck_thread();
			Once=false;
		}
		//Show_first_mode(readFirstMode());
		break;
	}
/*	case	THERMAL_IMAGERY_VIEW_MODE:
		RenderIndividualView(0,0,g_windowWidth*1400/1920, g_windowHeight, needSendData);
		break;
	case		WHITE_LIGHT_VIEW_MODE:
		RenderIndividualView(0,0,g_windowWidth*1400/1920, g_windowHeight, needSendData);
		break;
	case		FUSE_VIEW_MODE:
		break;*/
	case	VGA_WHITE_VIEW_MODE:
			p_ForeSightFacade_Track->Reset(VGA_WHITE_VIEW_MODE);
			RenderTriangleView(env,g_windowWidth*1100.0/1920.0,g_windowHeight*415.0/1080.0,g_windowWidth*800.0/1920.0,g_windowHeight*800.0/1920.0);
			RenderVGAView(env,0,0,g_windowWidth*1434/1920, g_windowHeight, needSendData);
			RenderTrackForeSightView(env,0,0,g_windowWidth*1434/1920, g_windowHeight);
			render.SendtoTrack();
			RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
			RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
			break;
		case	VGA_HOT_BIG_VIEW_MODE:
				p_ForeSightFacade_Track->Reset(VGA_HOT_BIG_VIEW_MODE);
				RenderVGAView(env,0,0,g_windowWidth*1434/1920, g_windowHeight, needSendData);
				RenderTrackForeSightView(env,0,0,g_windowWidth*1434/1920, g_windowHeight);
				render.SendtoTrack();
				RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
				RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
				break;
		case		VGA_HOT_SMALL_VIEW_MODE:
			p_ForeSightFacade_Track->Reset(VGA_HOT_SMALL_VIEW_MODE);
			RenderVGAView(env,0,0,g_windowWidth*1434/1920, g_windowHeight, needSendData);
				RenderTrackForeSightView(env,0,0,g_windowWidth*1434/1920, g_windowHeight);
				render.SendtoTrack();
				RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
				RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
				break;
		case		VGA_FUSE_WOOD_LAND_VIEW_MODE:
			p_ForeSightFacade_Track->Reset(VGA_FUSE_WOOD_LAND_VIEW_MODE);
				RenderVGAView(env,0,0,g_windowWidth*1434/1920, g_windowHeight, needSendData);
				RenderTrackForeSightView(env,0,0,g_windowWidth*1434/1920, g_windowHeight);
				render.SendtoTrack();
				RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
				RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
				break;
		case		VGA_FUSE_GRASS_LAND_VIEW_MODE:
			p_ForeSightFacade_Track->Reset(VGA_FUSE_GRASS_LAND_VIEW_MODE);
				RenderVGAView(env,0,0,g_windowWidth*1434/1920, g_windowHeight, needSendData);
				RenderTrackForeSightView(env,0,0,g_windowWidth*1434/1920, g_windowHeight);
				render.SendtoTrack();
				RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
				RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
				break;
		case			VGA_FUSE_SNOW_FIELD_VIEW_MODE:
			p_ForeSightFacade_Track->Reset(VGA_FUSE_SNOW_FIELD_VIEW_MODE);
				RenderVGAView(env,0,0,g_windowWidth*1434/1920, g_windowHeight, needSendData);
				RenderTrackForeSightView(env,0,0,g_windowWidth*1434/1920, g_windowHeight);
				render.SendtoTrack();
				RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
				RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
				break;
		case		VGA_FUSE_DESERT_VIEW_MODE:
			p_ForeSightFacade_Track->Reset(VGA_FUSE_DESERT_VIEW_MODE);
			RenderVGAView(env,0,0,g_windowWidth*1434/1920, g_windowHeight, needSendData);
				RenderTrackForeSightView(env,0,0,g_windowWidth*1434/1920, g_windowHeight);
				render.SendtoTrack();
				RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
				RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
				break;
		case			VGA_FUSE_CITY_VIEW_MODE:
		{
			p_ForeSightFacade_Track->Reset(VGA_FUSE_CITY_VIEW_MODE);
			RenderVGAView(env,0,0,g_windowWidth*1434/1920, g_windowHeight, needSendData);
			RenderTrackForeSightView(env,0,0,g_windowWidth*1434/1920, g_windowHeight);
			render.SendtoTrack();
			RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
			RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
			break;
		}

		case	SDI1_WHITE_BIG_VIEW_MODE:
	#if USE_GPIO
			if( last_gpio_sdi!=ENABLE_SDI1)
			{
				set_gpioNum_Value(GPIO_NUM152,ENABLE_SDI1);
				last_gpio_sdi=ENABLE_SDI1;
			}
	#endif
			p_ForeSightFacade_Track->Reset(SDI1_WHITE_BIG_VIEW_MODE);
			RenderSDIView(env,0,0,g_windowWidth*1434/1920, g_windowHeight, true);
			RenderTrackForeSightView(env,0,0,g_windowWidth*1434/1920, g_windowHeight);
			render.SendtoTrack();
			RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
			RenderTriangleView(env,g_windowWidth*1100.0/1920.0,g_windowHeight*415.0/1080.0,g_windowWidth*800.0/1920.0,g_windowHeight*800.0/1920.0);
			RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
				break;
		case			SDI1_WHITE_SMALL_VIEW_MODE:
		{
	#if USE_GPIO
			if( last_gpio_sdi!=ENABLE_SDI1)
			{
				set_gpioNum_Value(GPIO_NUM152,ENABLE_SDI1);
				last_gpio_sdi=ENABLE_SDI1;
			}
	#endif
			p_ForeSightFacade_Track->Reset(SDI1_WHITE_SMALL_VIEW_MODE);
			RenderSDIView(env,0,0,g_windowWidth*1434/1920, g_windowHeight, true);
			RenderTrackForeSightView(env,0,0,g_windowWidth*1434/1920, g_windowHeight);
			render.SendtoTrack();
			RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
			RenderTriangleView(env,g_windowWidth*1100.0/1920.0,g_windowHeight*415.0/1080.0,g_windowWidth*800.0/1920.0,g_windowHeight*800.0/1920.0);
			RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
				break;
		}
		case		SDI2_HOT_BIG_VIEW_MODE:
	#if USE_GPIO
			if( last_gpio_sdi!=ENABLE_SDI2)
			{
				set_gpioNum_Value(GPIO_NUM152,ENABLE_SDI2);
				last_gpio_sdi=ENABLE_SDI2;
			}
	#endif
			p_ForeSightFacade_Track->Reset(SDI2_HOT_BIG_VIEW_MODE);
			RenderSDIView(env,0,0,g_windowWidth*1434/1920, g_windowHeight, true);
			RenderTrackForeSightView(env,0,0,g_windowWidth*1434/1920, g_windowHeight);
			render.SendtoTrack();
			RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
					RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
			break;
		case		SDI2_HOT_SMALL_VIEW_MODE:
		{
	#if USE_GPIO
			if( last_gpio_sdi!=ENABLE_SDI2)
			{
				set_gpioNum_Value(GPIO_NUM152,ENABLE_SDI2);
				last_gpio_sdi=ENABLE_SDI2;
			}
	#endif
			p_ForeSightFacade_Track->Reset(SDI2_HOT_SMALL_VIEW_MODE);
			RenderSDIView(env,0,0,g_windowWidth*1434/1920, g_windowHeight, true);
			RenderTrackForeSightView(env,0,0,g_windowWidth*1434/1920, g_windowHeight);
			render.SendtoTrack();
			RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
					RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
			break;
		}
		case	PAL1_WHITE_BIG_VIEW_MODE:
			p_ForeSightFacade_Track->Reset(PAL1_WHITE_BIG_VIEW_MODE);
				SetCurrentExtesionVideoId(EXT_CAM_0);
				RenderTrackForeSightView(env,0,0,g_windowWidth*1346/1920, g_windowHeight);
				render.SendtoTrack();
				RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
				RenderTriangleView(env,g_windowWidth*1100.0/1920.0,g_windowHeight*415.0/1080.0,g_windowWidth*800.0/1920.0,g_windowHeight*800.0/1920.0);
					RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
				break;
		case	PAL1_WHITE_SMALL_VIEW_MODE:
		{
			p_ForeSightFacade_Track->Reset(SDI2_HOT_BIG_VIEW_MODE);
			SetCurrentExtesionVideoId(EXT_CAM_0);
			RenderTrackForeSightView(env,0,0,g_windowWidth*1346/1920, g_windowHeight);
			render.SendtoTrack();
			RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
			RenderTriangleView(env,g_windowWidth*1100.0/1920.0,g_windowHeight*415.0/1080.0,g_windowWidth*800.0/1920.0,g_windowHeight*800.0/1920.0);
				RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
			break;
		}
		case		PAL2_HOT_BIG_VIEW_MODE:
			p_ForeSightFacade_Track->Reset(PAL2_HOT_BIG_VIEW_MODE);
					SetCurrentExtesionVideoId(EXT_CAM_1);
					RenderTrackForeSightView(env,0,0,g_windowWidth*1346/1920, g_windowHeight);
					render.SendtoTrack();
					RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
					RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
					break;
		case		PAL2_HOT_SMALL_VIEW_MODE:
		{
			p_ForeSightFacade_Track->Reset(PAL2_HOT_SMALL_VIEW_MODE);
			SetCurrentExtesionVideoId(EXT_CAM_1);
			RenderTrackForeSightView(env,0,0,g_windowWidth*1346/1920, g_windowHeight);
			render.SendtoTrack();
			RenderCompassView(env,g_windowWidth*1495/1920,g_windowHeight*140/1080.0, g_windowWidth*290.0/1920.0, g_windowWidth*290.0/1920.0);
			RenderPositionView(env,g_windowWidth*0,g_windowHeight*0,g_windowWidth, g_windowHeight);
			break;
		}
		default:
			break;
		}

	if(isCalibTimeOn){
		RenderTimeView(env,0,g_windowHeight*7/8,g_windowWidth, g_windowHeight/8);
	}
	if((isDirectionOn&& bShowDirection)){
		RenderBillBoardAt(env,billBoardx, billBoardy, g_windowHeight*1/8, g_windowHeight*1/8);
	}
	if(displayMode==PANO_VIEW_MODE)
	{
		RenderCompassBillBoardAt(env,0,g_windowHeight*1.0/4.0,g_windowHeight, g_windowHeight);

	}
	float scaleofsmallpano=2.0/3.0;

	if(displayMode==FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE)
	{
		billBoardx=0;
		billBoardy=g_windowHeight;
		int last_direction_second=p_BillBoard->m_Direction;
		for(int i=0;i<2;i++)
		{
			p_BillBoard->m_Direction=(i+5)%6;
			RenderBillBoardAt(env,billBoardx+i*g_windowWidth/2-i*g_windowWidth/16, billBoardy-scaleofsmallpano*g_windowHeight*1/20,
					scaleofsmallpano*g_windowHeight*1/8, scaleofsmallpano*g_windowHeight*1/8);
			p_BillBoard->m_Direction=(4-i)%6;
			RenderBillBoardAt(env,billBoardx+i*g_windowWidth/2-i*g_windowWidth/16, billBoardy-g_windowHeight/3-scaleofsmallpano*g_windowHeight*1/20,
					scaleofsmallpano*g_windowHeight*1/8, scaleofsmallpano*g_windowHeight*1/8);

		}
		p_BillBoard->m_Direction=1;
		RenderBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/10, billBoardy-scaleofsmallpano*g_windowHeight*1/20,
				scaleofsmallpano*g_windowHeight*1/8, scaleofsmallpano*g_windowHeight*1/8);

		p_BillBoard->m_Direction=2;
		RenderBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/10, billBoardy-g_windowHeight/3-scaleofsmallpano*g_windowHeight*1/20,
				scaleofsmallpano*g_windowHeight*1/8, scaleofsmallpano*g_windowHeight*1/8);

		p_BillBoard->m_Direction=last_direction_second;
	}





	if(displayMode==TWO_HALF_PANO_VIEW_MODE)
	{
		billBoardx=0;
		billBoardy=g_windowHeight;
		int last_direction=p_BillBoard->m_Direction;
		for(int i=0;i<2;i++)
		{
			p_BillBoard->m_Direction=(i+5)%6;
			RenderBillBoardAt(env,billBoardx+i*g_windowWidth/2-i*g_windowWidth/16, billBoardy-g_windowHeight*1/20, g_windowHeight*1/8, g_windowHeight*1/8);
			p_BillBoard->m_Direction=(4-i)%6;
			RenderBillBoardAt(env,billBoardx+i*g_windowWidth/2-i*g_windowWidth/16, billBoardy-g_windowHeight/2-g_windowHeight*1/20, g_windowHeight*1/8, g_windowHeight*1/8);

		}
		p_BillBoard->m_Direction=1;
		RenderBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/10, billBoardy-g_windowHeight*1/20, g_windowHeight*1/8, g_windowHeight*1/8);

		p_BillBoard->m_Direction=2;
		RenderBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/10, billBoardy-g_windowHeight/2-g_windowHeight*1/20, g_windowHeight*1/8, g_windowHeight*1/8);

		p_BillBoard->m_Direction=last_direction;
	}

	if(displayMode==CHECK_MYSELF)
	{
	//	Show_first_mode(readFirstMode());
		billBoardx=0;
		billBoardy=g_windowHeight;
	//p_ChineseCBillBoard->ChooseTga=CHECK_SELF_T;
	//RenderChineseCharacterBillBoardAt(billBoardx+g_windowWidth-g_windowWidth/1.1, billBoardy-g_windowHeight*1/3, g_windowHeight*1/2, g_windowHeight*1/2);


		selfcheck.CalculateTime(1);
		#if USE_UART
		if(zodiac_msg.CheckFine()==SELFCHECK_IDLE)
		{
			if(selfcheck.IsOnesec())
			{
				p_ChineseCBillBoard->ChooseTga=IDLE_T;
				RenderChineseCharacterBillBoardAt(g_windowWidth*360.0/1920.0, billBoardy-g_windowHeight*2/3.3, g_windowWidth*1.0/2.5, g_windowHeight*1/2*1.5);
		}
		}
		else if(zodiac_msg.CheckFine()==SELFCHECK_PASS)
		{
				p_ChineseCBillBoard->ChooseTga=FINE_T;
				RenderChineseCharacterBillBoardAt(g_windowWidth*360.0/1920.0, billBoardy-g_windowHeight*2/3.3, g_windowWidth*1.0/2.5, g_windowHeight*1/2*1.5);
				Show_first_mode(readFirstMode());
		}
		else if(zodiac_msg.CheckFine()==SELFCHECK_FAIL)
		{
			p_ChineseCBillBoard->ChooseTga=WRONG_T;
			RenderChineseCharacterBillBoardAt(g_windowWidth*360.0/1920.0, billBoardy-g_windowHeight*2/3.3, g_windowWidth*1.0/2.5, g_windowHeight*1/2*1.5);
			Show_first_mode(readFirstMode());
		}
#else
		selfcheck.CalculateTime(1);
		if(selfcheck.IsIDLE()==SELFCHECK_IDLE)
		{
			if(selfcheck.IsOnesec())
			{
				p_ChineseCBillBoard->ChooseTga=IDLE_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*360.0/1920.0, billBoardy-g_windowHeight*2/3.3, g_windowWidth*1.0/2.5, g_windowHeight*1/2*1.5);
			}
		}
		else 	if(selfcheck.IsIDLE()==SELFCHECK_PASS)
		{
			p_ChineseCBillBoard->ChooseTga=FINE_T;
			RenderChineseCharacterBillBoardAt(env,g_windowWidth*360.0/1920.0, billBoardy-g_windowHeight*2/3.3, g_windowWidth*1.0/2.5, g_windowHeight*1/2*1.5);
			static int a=0;
			if(a++==50)
			{
			displayMode=ALL_VIEW_MODE;
			a=0;
			}
		}
		else if(selfcheck.IsIDLE()==SELFCHECK_FAIL)
		{
				p_ChineseCBillBoard->ChooseTga=WRONG_T;
		RenderChineseCharacterBillBoardAt(env,g_windowWidth*360.0/1920.0, billBoardy-g_windowHeight*2/3.3, g_windowWidth*1.0/2.5, g_windowHeight*1/2*1.5);
		static int b=0;
				if(b++==50)
				{
					displayMode=ALL_VIEW_MODE;
				b=0;
				}

		}
		#endif
	}

	else if(displayMode==	ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE)
	{
		p_ChineseCBillBoard->ChooseTga=ONEX_REALTIME_T;
		RenderChineseCharacterBillBoardAt(env,-g_windowWidth*1050.0/1920.0, g_windowHeight*120.0/1080.0, g_windowWidth*1344.0/1920.0,g_windowHeight*1536.0/1920.0);
			p_ChineseCBillBoard->ChooseTga=TWOX_REALTIME_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.0/1920.0,g_windowHeight*120.0/1080.0, g_windowWidth*1344.0/1920.0,g_windowHeight*1536.0/1920.0);

			p_ChineseCBillBoard->ChooseTga=ANGLE_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*999.0/1920.0, g_windowHeight*174.0/1080.0, g_windowWidth*900.0/1920.0,g_windowHeight*980.0/1080.0);

			p_ChineseCBillBoard->ChooseTga=LOCATION_T;
	//		RenderChineseCharacterBillBoardAt(g_windowWidth*1100.0/1920.0, g_windowHeight*121.5/1920.0, g_windowWidth*812.5/1920.0,g_windowWidth*650.0/1920.0);
			RenderChineseCharacterBillBoardAt(env,g_windowWidth*950.0/1920.0, g_windowHeight*50/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);

	}
	else if(displayMode==	ALL_VIEW_MODE)
	{
/*		p_ChineseCBillBoard->ChooseTga=ONEX_REALTIME_T;
		RenderChineseCharacterBillBoardAt(env,-g_windowWidth*1050.0/1920.0, g_windowHeight*120.0/1080.0, g_windowWidth*1344.0/1920.0,g_windowHeight*1536.0/1920.0);
			p_ChineseCBillBoard->ChooseTga=TWOX_REALTIME_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.0/1920.0,g_windowHeight*120.0/1080.0, g_windowWidth*1344.0/1920.0,g_windowHeight*1536.0/1920.0);

			p_ChineseCBillBoard->ChooseTga=ANGLE_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*999.0/1920.0, g_windowHeight*174.0/1080.0, g_windowWidth*900.0/1920.0,g_windowHeight*980.0/1080.0);
		//	p_ChineseCBillBoard->ChooseTga=LOCATION_T;
		//		RenderChineseCharacterBillBoardAt(env,g_windowWidth*950.0/1920.0, g_windowHeight*50/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
*/
	}

	else if(displayMode==TELESCOPE_FRONT_MODE
			||displayMode==TELESCOPE_RIGHT_MODE
			||displayMode==TELESCOPE_BACK_MODE
			||displayMode==TELESCOPE_LEFT_MODE)
	{
		/*
	p_ChineseCBillBoard->ChooseTga=TWOX_REALTIME_T;
	RenderChineseCharacterBillBoardAt(env,-g_windowWidth*1050.0/1920.0, g_windowHeight*120.0/1080.0, g_windowWidth*1344.0/1920.0,g_windowHeight*1536.0/1920.0);

		p_ChineseCBillBoard->ChooseTga=FOURX_REALTIME_T;
		RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.0/1920.0,g_windowHeight*120.0/1080.0, g_windowWidth*1344.0/1920.0,g_windowHeight*1536.0/1920.0);
*/
	p_ChineseCBillBoard->ChooseTga=ANGLE_T;
	RenderChineseCharacterBillBoardAt(env,g_windowWidth*999.0/1920.0, g_windowHeight*174.0/1080.0, g_windowWidth*900.0/1920.0,g_windowHeight*980.0/1080.0);

		 if(displayMode==TELESCOPE_FRONT_MODE)
		{
			p_ChineseCBillBoard->ChooseTga=RADAR_FRONT_T;
		//	if(mv_detect.CanUseMD())
		//		RenderChineseCharacterBillBoardAt(g_windowWidth*750.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
		//	else
			RenderChineseCharacterBillBoardAt(env,g_windowWidth*200.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
		}
		else if(displayMode==TELESCOPE_RIGHT_MODE)
		{
				p_ChineseCBillBoard->ChooseTga=RADAR_RIGHT_T;
				//	if(mv_detect.CanUseMD())
						//		RenderChineseCharacterBillBoardAt(g_windowWidth*750.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
						//	else
							RenderChineseCharacterBillBoardAt(env,g_windowWidth*200.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
		}
		else if(displayMode==TELESCOPE_BACK_MODE)
		{
			p_ChineseCBillBoard->ChooseTga=RADAR_BACK_T;
			//	if(mv_detect.CanUseMD())
					//		RenderChineseCharacterBillBoardAt(g_windowWidth*750.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
					//	else
						RenderChineseCharacterBillBoardAt(env,g_windowWidth*200.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
		}
		else if(displayMode==TELESCOPE_LEFT_MODE)
		{
		p_ChineseCBillBoard->ChooseTga=RADAR_LEFT_T;
		//	if(mv_detect.CanUseMD())
				//		RenderChineseCharacterBillBoardAt(g_windowWidth*750.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
				//	else
					RenderChineseCharacterBillBoardAt(env,g_windowWidth*200.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
		}
	}

	else if(displayMode==VGA_WHITE_VIEW_MODE
			||displayMode==VGA_HOT_BIG_VIEW_MODE
			||displayMode==VGA_HOT_SMALL_VIEW_MODE
			||displayMode==VGA_FUSE_WOOD_LAND_VIEW_MODE
			||displayMode==VGA_FUSE_GRASS_LAND_VIEW_MODE
			||displayMode==VGA_FUSE_SNOW_FIELD_VIEW_MODE
			||displayMode==VGA_FUSE_DESERT_VIEW_MODE
			||displayMode==VGA_FUSE_CITY_VIEW_MODE
			||displayMode==SDI1_WHITE_BIG_VIEW_MODE
			||displayMode==SDI1_WHITE_SMALL_VIEW_MODE
			||displayMode==SDI2_HOT_BIG_VIEW_MODE
			||displayMode==SDI2_HOT_SMALL_VIEW_MODE
			||displayMode==PAL1_WHITE_BIG_VIEW_MODE
			||displayMode==PAL1_WHITE_SMALL_VIEW_MODE
			||displayMode==PAL2_HOT_BIG_VIEW_MODE
			||displayMode==PAL2_HOT_SMALL_VIEW_MODE
			)
	{
		p_ChineseCBillBoard->ChooseTga=LOCATION_T;
		RenderChineseCharacterBillBoardAt(env,g_windowWidth*830.0/1920.0, g_windowHeight*322.0/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);

		switch(displayMode)
		{
			case VGA_WHITE_VIEW_MODE:
				{
					p_ChineseCBillBoard->ChooseTga=VGA_WHITE_BIG_T;

				RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.46/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);
					break;
				}
				case VGA_HOT_BIG_VIEW_MODE :
				{
					p_ChineseCBillBoard->ChooseTga=VGA_HOT_BIG_T;
					RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.40/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);

					break;
				}
				case VGA_HOT_SMALL_VIEW_MODE :
				{
					p_ChineseCBillBoard->ChooseTga=VGA_HOT_SMALL_T;
					RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.4/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);
							break;
				}

				case VGA_FUSE_WOOD_LAND_VIEW_MODE :
				{
					p_ChineseCBillBoard->ChooseTga=VGA_FUSE_WOOD_T;
					RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.4/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);
						break;
				}
				case VGA_FUSE_GRASS_LAND_VIEW_MODE :
				{
					p_ChineseCBillBoard->ChooseTga=VGA_FUSE_GRASS_T;
					RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.4/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);
						break;

				}
				case VGA_FUSE_SNOW_FIELD_VIEW_MODE :
				{
					p_ChineseCBillBoard->ChooseTga=VGA_FUSE_SNOW_T;
					RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.4/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);
						break;
				}
				case VGA_FUSE_DESERT_VIEW_MODE:
				{
					p_ChineseCBillBoard->ChooseTga=VGA_FUSE_DESERT_T;
					RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.4/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);
						break;
				}
				case VGA_FUSE_CITY_VIEW_MODE:
				{
					p_ChineseCBillBoard->ChooseTga=VGA_FUSE_CITY_T;
					RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.4/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);
						break;
				}



				case SDI1_WHITE_BIG_VIEW_MODE:
				{
					p_ChineseCBillBoard->ChooseTga=SDI1_WHITE_BIG_T;
					RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.46/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);
							break;
				}
				case SDI1_WHITE_SMALL_VIEW_MODE:
				{
					p_ChineseCBillBoard->ChooseTga=SDI1_WHITE_SMALL_T;
					RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.46/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);
						break;
				}
				case SDI2_HOT_BIG_VIEW_MODE:
				{
					p_ChineseCBillBoard->ChooseTga=SDI2_HOT_BIG_T;
					RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.4/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);
						break;
				}
				case	SDI2_HOT_SMALL_VIEW_MODE:
				{
					p_ChineseCBillBoard->ChooseTga=SDI2_HOT_SMALL_T;
					RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.4/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);
							break;
				}
				case	PAL1_WHITE_BIG_VIEW_MODE:
				{
					p_ChineseCBillBoard->ChooseTga=PAL1_WHITE_BIG_T;
					RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.46/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);
						break;

				}
				case	PAL1_WHITE_SMALL_VIEW_MODE:
				{
					p_ChineseCBillBoard->ChooseTga=PAL1_WHITE_SMALL_T;
					RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.46/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);
							break;
				}
				case	PAL2_HOT_BIG_VIEW_MODE:
				{
					p_ChineseCBillBoard->ChooseTga=PAL2_HOT_BIG_T;
					RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.4/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);

					//RenderChineseCharacterBillBoardAt(billBoardx+g_windowWidth-g_windowWidth/2.5, billBoardy-g_windowHeight*2/3, g_windowHeight*1/2, g_windowHeight*1/2);
										break;
				}
				case	PAL2_HOT_SMALL_VIEW_MODE:
				{
					 p_ChineseCBillBoard->ChooseTga=PAL2_HOT_SMALL_T;
						RenderChineseCharacterBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/1.2, billBoardy-g_windowHeight*1.4/3, g_windowWidth*1/1.2, g_windowHeight*1/1.2);

					 //RenderChineseCharacterBillBoardAt(billBoardx+g_windowWidth-g_windowWidth/2.5, billBoardy-g_windowHeight*2/3, g_windowHeight*1/2, g_windowHeight*1/2);
					break;
				}
		}
	}


	if(displayMode==FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE
			||displayMode==FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE)
	{
		billBoardx=0;
		billBoardy=g_windowHeight;
		int last_direction_second=p_BillBoard->m_Direction;
		for(int i=0;i<2;i++)
		{
			p_BillBoard->m_Direction=(i+5)%6;
			RenderBillBoardAt(env,billBoardx+i*g_windowWidth/2-i*g_windowWidth/16, billBoardy-scaleofsmallpano*g_windowHeight*1/20,
					scaleofsmallpano*g_windowHeight*1/8, scaleofsmallpano*g_windowHeight*1/8);
			p_BillBoard->m_Direction=(4-i)%6;
			RenderBillBoardAt(env,billBoardx+i*g_windowWidth/2-i*g_windowWidth/16, billBoardy-g_windowHeight/3-scaleofsmallpano*g_windowHeight*1/20,
					scaleofsmallpano*g_windowHeight*1/8, scaleofsmallpano*g_windowHeight*1/8);

		}
		p_BillBoard->m_Direction=1;
		RenderBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/10, billBoardy-scaleofsmallpano*g_windowHeight*1/20,
				scaleofsmallpano*g_windowHeight*1/8, scaleofsmallpano*g_windowHeight*1/8);

		p_BillBoard->m_Direction=2;
		RenderBillBoardAt(env,billBoardx+g_windowWidth-g_windowWidth/10, billBoardy-g_windowHeight/3-scaleofsmallpano*g_windowHeight*1/20,
				scaleofsmallpano*g_windowHeight*1/8, scaleofsmallpano*g_windowHeight*1/8);

		p_BillBoard->m_Direction=last_direction_second;
	}

	if(isBillBoardExtOn){
		RenderExtensionBillBoardAt(env,extBillBoardx, extBillBoardy, g_windowHeight*1/8, g_windowHeight*1/8 );
	}
#ifdef GET_ALARM_AERA
	pVehicle->updateFBOs();

#endif
	last_mode=displayMode;
#ifdef GL_TIME_STAMP
	glEndQuery(GL_TIME_ELAPSED);
	glGetQueryObjectuiv( queries[0],GL_QUERY_RESULT, &rearTime);
	glGetQueryObjectuiv(queries[1], GL_QUERY_RESULT, &birdTime);
	glGetQueryObjectuiv( queries[2],GL_QUERY_RESULT, &fboTime);
	//	 glGetQueryObjectuiv( queries[3],GL_QUERY_RESULT, &fboTime);
	glDeleteQueries(3, queries);
	cout<<"mode="<<displayMode<<":rearTime="<<rearTime/1000000<<",birdTime="<<birdTime/1000000<<",fboTime="<<fboTime/1000000<<endl;
#endif

#endif// RENDER2FRONT
	p_ChineseCBillBoard_bottem_pos->ChooseTga=MENU_T;
			RenderChineseCharacterBillBoardAt(env,menu_tpic[0], menu_tpic[1], menu_tpic[2],menu_tpic[3],true);

			{

				int x=0,y=0,w=g_windowWidth,h=g_windowHeight;
				glViewport(x,y,w,h);
			//	m_env.GetviewFrustum()->SetPerspective(27.0f, float(w) / float(h), 1.0f, 100.0f);
				env.GetviewFrustum()->SetPerspective(27.0f,  float(w) / float(h), 1.0f, 500.0f);

				env.GetprojectionMatrix()->LoadMatrix(env.GetviewFrustum()->GetProjectionMatrix());

				env.GetmodelViewMatrix()->PushMatrix();





				int net_show_mode=getKey_SwitchMode(TRANSFER_TO_APP_ETHOR);
				static bool net_enable_move_cursor=false;
				if(net_show_mode==Mode_Type_DEBUG)
				{
					
					net_enable_move_cursor=!net_enable_move_cursor;
					SetPSYButtonF8(!net_enable_move_cursor);
					
				}
				else if(net_show_mode==Mode_Type_SINGLE_POPUP_WINDOWS)
				{
					if(displayMode==ALL_VIEW_MODE)
					{
						displayMode=CHOSEN_VIEW_MODE;
						SetPSYButtonF1(false);
						net_enable_move_cursor=false;
						SetPSYButtonF8(!net_enable_move_cursor);
					}
					else
					{
						displayMode=ALL_VIEW_MODE;
						SetPSYButtonF1(true);
						net_enable_move_cursor=false;
						SetPSYButtonF8(!net_enable_move_cursor);
						
					}
				}
				int net_open_mvdetect=getKey_TargetDetectionState(TRANSFER_TO_APP_ETHOR);
				if(net_open_mvdetect==1)
				{
					IsMvDetect=!IsMvDetect;
					SetPSYButtonF2(!IsMvDetect);
				}

				int net_open_enhance=getKey_ImageEnhancementState(TRANSFER_TO_APP_ETHOR);
				static bool enhance_state=false;
				if(net_open_enhance==1)
				{
					enhance_state=!enhance_state;
					SetPSYButtonF3(!enhance_state);
				}

				int net_dirction=getKey_MoveDirection(TRANSFER_TO_APP_ETHOR);
				if(net_enable_move_cursor)
				{
					if(net_dirction==MOVE_TYPE_MOVELEFT)
					{
						p_ForeSightFacade[MAIN]->MoveLeft(-PanoLen*100.0);
					}
					if(net_dirction==MOVE_TYPE_MOVERIGHT)
					{
						p_ForeSightFacade[MAIN]->MoveRight(PanoLen*100.0);
					}
					if(net_dirction==MOVE_TYPE_MOVEUP)
					{
						p_ForeSightFacade[MAIN]->MoveUp(PanoHeight/(OUTER_RECT_AND_PANO_TWO_TIMES_CAM_LIMIT));
					}
					if(net_dirction==MOVE_TYPE_MOVEDOWN)
					{
						p_ForeSightFacade[MAIN]->MoveDown(-PanoHeight/(20));
					}
					
				}
				else
				{
					if(displayMode==CHOSEN_VIEW_MODE)
					{
						int camera_dir=chosenCam[MAIN]-1;
						if((net_dirction==MOVE_TYPE_MOVELEFT)||(net_dirction==MOVE_TYPE_MOVEUP))
						{
							camera_dir=(camera_dir+9)%10;
						}
						else if((net_dirction==MOVE_TYPE_MOVERIGHT)||(net_dirction==MOVE_TYPE_MOVEDOWN))
						{
							camera_dir=(camera_dir+1)%10;
						}
						chosenCam[MAIN]=camera_dir+1;
					}
				}

				calc_hor_data=getAngleFar_PeriscopicLens(TRANSFER_TO_APP_ETHOR).hor_angle;
				calc_ver_data=getAngleFar_PeriscopicLens(TRANSFER_TO_APP_ETHOR).ver_angle;

				gun_hor_angle=getAngleFar_GunAngle(TRANSFER_TO_APP_ETHOR).hor_angle;
				gun_ver_angle=getAngleFar_GunAngle(TRANSFER_TO_APP_ETHOR).ver_angle;

				canon_hor_angle=getAngleFar_CanonAngle(TRANSFER_TO_APP_ETHOR).hor_angle;
				canon_ver_angle=getAngleFar_CanonAngle(TRANSFER_TO_APP_ETHOR).ver_angle;

				state_label_data[0][0]=getCaptureMessage().cameraFrontTest;
				state_label_data[0][1]=getCaptureMessage().cameraFrontState;

				
				state_label_data[1][0]=getCaptureMessage().cameraLeft1Test;
				state_label_data[1][1]=getCaptureMessage().cameraLeft1State;

				state_label_data[2][0]=getCaptureMessage().cameraLeft2Test;
				state_label_data[2][1]=getCaptureMessage().cameraLeft2State;

				state_label_data[3][0]=getCaptureMessage().cameraLeft3Test;
				state_label_data[3][1]=getCaptureMessage().cameraLeft3State;

				state_label_data[4][0]=getCaptureMessage().passengerTest;
				state_label_data[4][1]=getCaptureMessage().passengerState;

				state_label_data[5][0]=0;
				state_label_data[5][1]=0;



				
				state_label_data[6][0]=getCaptureMessage().cameraBackTest;
				state_label_data[6][1]=getCaptureMessage().cameraBackState;


				state_label_data[7][0]=getCaptureMessage().cameraRight1Test;
				state_label_data[7][1]=getCaptureMessage().cameraRight1State;


				state_label_data[8][0]=getCaptureMessage().cameraRight2Test;
				state_label_data[8][1]=getCaptureMessage().cameraRight2State;


				state_label_data[9][0]=getCaptureMessage().cameraRight3Test;
				state_label_data[9][1]=getCaptureMessage().cameraRight3State;


				state_label_data[10][0]=getCaptureMessage().Cap_BoxTest;
				state_label_data[10][1]=getCaptureMessage().Cap_BoxState;


				state_label_data[11][0]=getCaptureMessage().nearBoardTest;
				state_label_data[11][1]=getCaptureMessage().nearBoardState;



				for(int x_i=0;x_i<12;x_i++)
				{
					state_label_data[x_i][2]=getCaptureMessage().Cap_FAULT_Colour;
				}

state_label_data[5][2]=0;
				int index_i=-1;
				int width_delta=100;
				int w_y=0;
				int t_width=600;
				int t_height=700;
				if(GetPSYButtonF1())
				{
					p_ChineseCBillBoard->ChooseTga=F1_ON_T;
				}
				else
				{
					p_ChineseCBillBoard->ChooseTga=F1_OFF_T;
				}
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y, t_width, t_height);
				p_ChineseCBillBoard->ChooseTga=CANON_HOR_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y+50, t_width, t_height);
				p_ChineseCBillBoard->ChooseTga=CALC_HOR_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y+100, t_width, t_height);


				index_i++;

				if(GetPSYButtonF2())
				{
					p_ChineseCBillBoard->ChooseTga=F2_ON_T;
				}
				else
				{
					p_ChineseCBillBoard->ChooseTga=F2_OFF_T;
				}
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y, t_width, t_height);
				index_i++;

				if(GetPSYButtonF3())
				{
					p_ChineseCBillBoard->ChooseTga=F3_ON_T;
				}
				else
				{
					p_ChineseCBillBoard->ChooseTga=F3_OFF_T;
				}
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y, t_width, t_height);
				p_ChineseCBillBoard->ChooseTga=CANON_VER_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y+50, t_width, t_height);
				p_ChineseCBillBoard->ChooseTga=CALC_VER_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y+100, t_width, t_height);



				index_i++;

				p_ChineseCBillBoard->ChooseTga=F4_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y, t_width, t_height);
				index_i++;

				p_ChineseCBillBoard->ChooseTga=F5_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y, t_width, t_height);
				p_ChineseCBillBoard->ChooseTga=GUN_HOR_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y+50, t_width, t_height);

				index_i++;

				p_ChineseCBillBoard->ChooseTga=F6_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y, t_width, t_height);
				index_i++;

				p_ChineseCBillBoard->ChooseTga=F7_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y, t_width, t_height);
				p_ChineseCBillBoard->ChooseTga=GUN_VER_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y+50, t_width, t_height);

				index_i++;

				if(GetPSYButtonF8())
				{
					p_ChineseCBillBoard->ChooseTga=F8_ON_T;
				}
				else
				{
					p_ChineseCBillBoard->ChooseTga=F8_OFF_T;
				}
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y, t_width, t_height);
				index_i++;

				p_ChineseCBillBoard->ChooseTga=F9_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.1*index_i-width_delta, w_y, t_width, t_height);
				index_i++;

				env.GetmodelViewMatrix()->PopMatrix();

				int text_x=g_windowWidth/15;
				int text_y=30;

				int text_width=g_windowWidth/20;
				int text_height=80;
				Rect * rect;
				char text_data[20];
				rect=new Rect(g_windowWidth/10+text_x,text_y,text_width,text_height);
				strcpy(text_data,"");
				sprintf(text_data,"    %.4f",canon_hor_angle);
				DrawCordsView(env,rect,text_data);

				Rect * rect_1;
				rect_1=new Rect(1*g_windowWidth/10+text_x,text_y+50,text_width,text_height);
				strcpy(text_data,"");
				sprintf(text_data,"    %.4f",calc_hor_data);
				DrawCordsView(env,rect_1,text_data);

				Rect * rect_touch;
				rect_touch=new Rect(0*g_windowWidth/10+text_x,text_y+100,text_width,text_height);
				strcpy(text_data,"");
				sprintf(text_data,"%d,%d    ",touch_pos_x,touch_pos_y);
				DrawCordsView(env,rect_touch,text_data);

				Rect * rect2;
				rect2=new Rect(3*g_windowWidth/10+text_x,text_y,text_width,text_height);
				strcpy(text_data,"");
				sprintf(text_data,"    %.4f",canon_ver_angle);
				DrawCordsView(env,rect2,text_data);


				Rect * rect_2;
				rect_2=new Rect(3*g_windowWidth/10+text_x,text_y+50,text_width,text_height);
				strcpy(text_data,"");
				sprintf(text_data,"    %.4f",calc_ver_data);
				DrawCordsView(env,rect_2,text_data);

				Rect * rect3;
				rect3=new Rect(5*g_windowWidth/10+text_x,text_y,text_width,text_height);
				strcpy(text_data,"");
				sprintf(text_data,"    %.4f",gun_hor_angle);
				DrawCordsView(env,rect3,text_data);

				Rect * rect4;
				rect4=new Rect(7*g_windowWidth/10+text_x,text_y,text_width,text_height);
				strcpy(text_data,"");
				sprintf(text_data,"    %.4f",gun_ver_angle);
				DrawCordsView(env,rect4,text_data);

			}

}



void Render::setOverlapPeta(int chId, float alphaId)
{
//	setPetaUpdate(chId, true);
	alpha[chId] = alphaId;
//	printf("chId:%d,alpha:%f\n",chId,la);
}
/* The main drawing function. */
void Render::DrawGLScene()
{
	char arg1[128],arg2[128];
	unsigned char  screen_data[720*576*4];
	unsigned char full_screen_data[1920*1080*4];
#if USE_TRACK
	glReadBuffer(GL_FRONT);
	glReadPixels(0,0,1920,1080,GL_BGRA_EXT,GL_UNSIGNED_BYTE,full_screen_data);
#endif
	RenderScene();
	/* swap the buffers to display, since double buffering is used.*/
#if ENABLE_SCREEN_CAPTURE
{
	static timeval lasttime,nowtime;
	static int frametime=-1;
	gettimeofday(&nowtime,0);
	if(frametime>=0)
	{
		frametime=frametime+(nowtime.tv_sec-lasttime.tv_sec)*1000000+(nowtime.tv_usec-lasttime.tv_usec);
		if(frametime>40000)
		{
			glReadBuffer(GL_FRONT);

			glReadPixels(0,0,720,576,GL_BGRA_EXT,GL_UNSIGNED_BYTE,screen_data);

			//CabinPushData(screen_handle,(char *)screen_data,720*576*4);//record screen

			frametime=frametime-40000;
		}
	}
	else
	{
		frametime=0;
	}
	lasttime=nowtime;
}
#endif

	glutSwapBuffers();
	glFinish();
	GetFPS();  /* Get frame rate stats */

	/* Copy saved window name into temp string arg1 so that we can add stats */
	strcpy (arg1, common.getWindowName());

	if (sprintf(arg2, "%.2f FPS", common.getFrameRate()))
	{
		strcat (arg1, arg2);
	}

	/* cut down on the number of redraws on window title.  Only draw once per sample*/
	if (common.isCountUpdate())
	{
		glutSetWindowTitle(arg1);
	}
}

/* The function called whenever a mouse button event occurs */
void Render::mouseButtonPress(int button, int state, int x, int y)
{
	if (common.isVerbose())
		printf(" mouse--> %i %i %i %i\n", button, state, x, y);
	setMouseCor(x,y);
	setMouseButton(button);
}

void Render::GenerateBirdView()
{
	static M3DVector3f camBirdView;
	static bool once =true;
	if(once){
		once = false;
		birdViewCameraFrame.MoveForward(-34.5f);
		birdViewCameraFrame.MoveRight(0.0f);
		birdViewCameraFrame.MoveUp(0.5f);
		birdViewCameraFrame.GetOrigin(camBirdView);
	}
	birdViewCameraFrame.SetOrigin(camBirdView);
}

void Render::GenerateCenterView()
{
	static M3DVector3f camCenterView;
	static bool once =true;
	if(once){
		once = false;
		CenterViewCameraFrame.RotateLocalX(-PI/2);
		CenterViewCameraFrame.MoveUp(3.0f);//4.5
		CenterViewCameraFrame.MoveForward(-6.0f);
		CenterViewCameraFrame.MoveRight(0.0f);
		CenterViewCameraFrame.GetOrigin(camCenterView);
	}
	CenterViewCameraFrame.SetOrigin(camCenterView);
}
/*
void Render::GenerateCompassView()
{
	static M3DVector3f camCompassView;
	static bool once =true;
	if(once){
		once = false;
		CompassCameraFrame.RotateLocalX(-PI/4);
		CompassCameraFrame.MoveUp(0.0f);//4.5
		CompassCameraFrame.MoveForward(0.0f);
		CompassCameraFrame.MoveRight(0.0f);
		CompassCameraFrame.GetOrigin(camCompassView);
	}
	CompassCameraFrame.SetOrigin(camCompassView);
}
*/
void Render::GenerateScanPanelView()
{
	static M3DVector3f camScanPanelView;
	static bool once =true;
	if(once){
		once = false;
		ScanPanelViewCameraFrame.RotateLocalX(-PI/2);
		ScanPanelViewCameraFrame.MoveForward(-(10.0));//(-10.0f);//(-36.5*4.0/7.0);//(-PanelLoader.Getextent_pos_z()*2);//+PanelLoader.Getextent_neg_y());
		setlastregionforwarddistance(-10.0f);
		//		ScanPanelViewCameraFrame.MoveForward(-33.3f*SCAN_REGION_ANGLE/360.0f);
		ScanPanelViewCameraFrame.MoveUp((PanelLoader.Getextent_pos_z()+PanelLoader.Getextent_neg_z())/2);
		ScanPanelViewCameraFrame.GetOrigin(camScanPanelView);
	}
	ScanPanelViewCameraFrame.SetOrigin(camScanPanelView);
}

void Render::GenerateTrack()
{
//	float inidelta=(p_LineofRuler->Load())/360.0*(render.get_PanelLoader().Getextent_pos_x()-render.get_PanelLoader().Getextent_neg_x());
	static M3DVector3f camTrackView;
	static bool once =true;
	if(once){
		once = false;
		TrackCameraFrame.RotateLocalX(-PI/2);
		TrackCameraFrame.MoveForward(-39.0f);
		TrackCameraFrame.MoveForward(-25.0f);
		TrackCameraFrame.MoveUp((BowlLoader.Getextent_pos_z()-BowlLoader.Getextent_neg_z())/2);
		TrackCameraFrame.MoveRight(0.0f);
		TrackCameraFrame.GetOrigin(camTrackView);
	}
	TrackCameraFrame.SetOrigin(camTrackView);
}

void Render::GeneratePanoTelView(int mainOrsub)
{
//	float inidelta=(p_LineofRuler->Load())/360.0*(render.get_PanelLoader().Getextent_pos_x()-render.get_PanelLoader().Getextent_neg_x());
	static M3DVector3f camPanoTelView[2];
	static bool once[2] ={true,true};
	if(once[mainOrsub]){
		once[mainOrsub] = false;
		PanoTelViewCameraFrame[mainOrsub].RotateLocalX(-PI/2);
		PanoTelViewCameraFrame[mainOrsub].MoveForward(-39.0f);
		PanoTelViewCameraFrame[mainOrsub].MoveForward(-25.0f);
		PanoTelViewCameraFrame[mainOrsub].MoveUp((BowlLoader.Getextent_pos_z()-BowlLoader.Getextent_neg_z())/2);
		PanoTelViewCameraFrame[mainOrsub].MoveRight(0.0f);
		PanoTelViewCameraFrame[mainOrsub].GetOrigin(camPanoTelView[mainOrsub]);
	}
	PanoTelViewCameraFrame[mainOrsub].SetOrigin(camPanoTelView[mainOrsub]);
}

void Render::GeneratePanoView()
{
	static M3DVector3f camPanoView;
	static bool once =true;
	if(once){
		once = false;
		PanoViewCameraFrame.RotateLocalX(-PI/2);
		PanoViewCameraFrame.MoveForward(-39.0f);
		PanoViewCameraFrame.MoveUp((BowlLoader.Getextent_pos_z()-BowlLoader.Getextent_neg_z())/2);
		PanoViewCameraFrame.MoveRight(0.0f);
		PanoViewCameraFrame.GetOrigin(camPanoView);
	}
	PanoViewCameraFrame.SetOrigin(camPanoView);
}
float leftandrightdis=35.7;
void Render::GenerateLeftPanoView()
{
	static M3DVector3f camPanoView;
	static bool once =true;
	if(once){
		once = false;
		LeftPanoViewCameraFrame.RotateLocalX(-PI/2);
		LeftPanoViewCameraFrame.MoveForward(-leftandrightdis);
		LeftPanoViewCameraFrame.MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);
		LeftPanoViewCameraFrame.MoveRight(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x()-0.4));
		LeftPanoViewCameraFrame.GetOrigin(camPanoView);
	}
	LeftPanoViewCameraFrame.SetOrigin(camPanoView);
}

void Render::GenerateRightPanoView()
{
	static M3DVector3f camPanoView;
	static bool once =true;
	if(once){
		once = false;

		RightPanoViewCameraFrame.RotateLocalX(-PI/2);
		RightPanoViewCameraFrame.MoveForward(leftandrightdis);
		RightPanoViewCameraFrame.MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);

		RightPanoViewCameraFrame.RotateLocalY(PI);
		RightPanoViewCameraFrame.MoveRight((PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())*(3.0-0.025)+0.4);
		RightPanoViewCameraFrame.GetOrigin(camPanoView);
	}
	RightPanoViewCameraFrame.SetOrigin(camPanoView);
}

void  Render::GenerateOnetimeView(int mainOrsub)
{
//	float inidelta=p_LineofRuler->Load();
	float Len=(render.get_PanelLoader().Getextent_pos_x()-render.get_PanelLoader().Getextent_neg_x());
	float inidelta=(RulerAngle)/360.0*Len;
	if(inidelta>270.0/360.0*Len)
	{
		inidelta-=Len;
		inidelta-=Len;
	}
	else
		inidelta-=Len;
	static M3DVector3f OnetimeCamView;
	static bool once[2] ={true,true};
	if(once[mainOrsub]){
		once[mainOrsub] = false;
			panocamonforesight[mainOrsub].getOneTimeCam().RotateLocalX(-PI/2);
		panocamonforesight[mainOrsub].getOneTimeCam().MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);
		panocamonforesight[mainOrsub].getOneTimeCam().MoveRight(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())*(1/4.0));
		panocamonforesight[mainOrsub].getOneTimeCam().MoveRight(-inidelta);
		panocamonforesight[mainOrsub].getOneTimeCam().MoveForward(-0.16*leftandrightdis/2);
			panocamonforesight[mainOrsub].getOneTimeCam().GetOrigin(OnetimeCamView);
	}
	panocamonforesight[mainOrsub].getOneTimeCam().SetOrigin(OnetimeCamView);
}

void  Render::GenerateOnetimeView2(int mainOrsub)
{
	float Len=(render.get_PanelLoader().Getextent_pos_x()-render.get_PanelLoader().Getextent_neg_x());
	float inidelta=(RulerAngle)/360.0*Len;
	if(inidelta>270.0/360.0*Len)
	{
		inidelta-=Len;
		inidelta-=Len;
	}
	else
		inidelta-=Len;
	static M3DVector3f OnetimeCamView2;
	static bool once[2] ={true,true};

	if(once[mainOrsub]){
		once[mainOrsub] = false;
		panocamonforesight[mainOrsub].getOneTimeCam2().RotateLocalX(-PI/2);
		panocamonforesight[mainOrsub].getOneTimeCam2().MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);
		panocamonforesight[mainOrsub].getOneTimeCam2().MoveRight(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())*(1/4.0));
			panocamonforesight[mainOrsub].getOneTimeCam2().MoveRight(-inidelta);
			panocamonforesight[mainOrsub].getOneTimeCam2().MoveForward(0.16*leftandrightdis/2);
			panocamonforesight[mainOrsub].getOneTimeCam2().RotateLocalY(-PI);
		panocamonforesight[mainOrsub].getOneTimeCam2().GetOrigin(OnetimeCamView2);
	}
	panocamonforesight[mainOrsub].getOneTimeCam2().SetOrigin(OnetimeCamView2);
}

void  Render::GenerateTwotimesView(int mainOrsub)
{
	float Len=(render.get_PanelLoader().Getextent_pos_x()-render.get_PanelLoader().Getextent_neg_x());
	float inidelta=(RulerAngle)/360.0*Len;
	if(inidelta>270.0/360.0*Len)
	{
		inidelta-=Len;
		inidelta-=Len;
	}
	else
		inidelta-=Len;
	static M3DVector3f TwotimesCamView;
	static bool once[2] ={true,true};
	if(once[mainOrsub]){
	once[mainOrsub] = false;
	panocamonforesight[mainOrsub].getTwoTimesCam().RotateLocalX(-PI/2);
	panocamonforesight[mainOrsub].getTwoTimesCam().MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);
	panocamonforesight[mainOrsub].getTwoTimesCam().MoveRight(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())/4);
	panocamonforesight[mainOrsub].getTwoTimesCam().MoveRight(-inidelta);
	panocamonforesight[mainOrsub].getTwoTimesCam().MoveForward(-0.08*leftandrightdis/2);
	panocamonforesight[mainOrsub].getTwoTimesCam().GetOrigin(TwotimesCamView);
	}
	panocamonforesight[mainOrsub].getTwoTimesCam().SetOrigin(TwotimesCamView);
}

void  Render::GenerateTwotimesView2(int mainOrsub)
{
	float Len=(render.get_PanelLoader().Getextent_pos_x()-render.get_PanelLoader().Getextent_neg_x());
	float inidelta=(RulerAngle)/360.0*Len;
	if(inidelta>270.0/360.0*Len)
	{
		inidelta-=Len;
		inidelta-=Len;
	}
	else
		inidelta-=Len;
	static M3DVector3f TwotimesCamView2;
	static bool once[2] ={true,true};
	if(once[mainOrsub]){
	once[mainOrsub] = false;
	panocamonforesight[mainOrsub].getTwoTimesCam2().RotateLocalX(-PI/2);
	panocamonforesight[mainOrsub].getTwoTimesCam2().MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);
	panocamonforesight[mainOrsub].getTwoTimesCam2().MoveRight(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())/4);
	panocamonforesight[mainOrsub].getTwoTimesCam2().MoveRight(-inidelta);
	panocamonforesight[mainOrsub].getTwoTimesCam2().MoveForward(0.08*leftandrightdis/2);
	panocamonforesight[mainOrsub].getTwoTimesCam2().RotateLocalY(-PI);
	panocamonforesight[mainOrsub].getTwoTimesCam2().GetOrigin(TwotimesCamView2);
	}
	panocamonforesight[mainOrsub].getTwoTimesCam2().SetOrigin(TwotimesCamView2);
}

void  Render::GenerateTwotimesTelView(int mainOrsub)
{
	float Len=(render.get_PanelLoader().Getextent_pos_x()-render.get_PanelLoader().Getextent_neg_x());
	float inidelta=(RulerAngle)/360.0*Len;
	if(inidelta>270.0/360.0*Len)
	{
		inidelta-=Len;
		inidelta-=Len;
	}
	else
		inidelta-=Len;
	static M3DVector3f TwotimesCamTelViewF;
	static bool once[2] ={true,true};
	if(once[mainOrsub]){
	once[mainOrsub] = false;
	telcamonforesight[mainOrsub].getTwoTimesCamTelF().RotateLocalX(-PI/2);
	telcamonforesight[mainOrsub].getTwoTimesCamTelF().MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);
	telcamonforesight[mainOrsub].getTwoTimesCamTelF().MoveRight(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())/4);
	telcamonforesight[mainOrsub].getTwoTimesCamTelF().MoveRight(-inidelta);
	telcamonforesight[mainOrsub].getTwoTimesCamTelF().MoveForward(-0.08*leftandrightdis/2);
	telcamonforesight[mainOrsub].getTwoTimesCamTelF().GetOrigin(TwotimesCamTelViewF);
	}
	telcamonforesight[mainOrsub].getTwoTimesCamTelF().SetOrigin(TwotimesCamTelViewF);

	static M3DVector3f TwotimesCamTelViewR;
	static bool onceR[2] ={true,true};
	if(onceR[mainOrsub]){
	onceR[mainOrsub] = false;
	telcamonforesight[mainOrsub].getTwoTimesCamTelR().RotateLocalX(-PI/2);
	telcamonforesight[mainOrsub].getTwoTimesCamTelR().MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);
	telcamonforesight[mainOrsub].getTwoTimesCamTelR().MoveRight(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())/2.0);
	telcamonforesight[mainOrsub].getTwoTimesCamTelR().MoveRight(-inidelta);
	telcamonforesight[mainOrsub].getTwoTimesCamTelR().MoveForward(-0.08*leftandrightdis/2);
	telcamonforesight[mainOrsub].getTwoTimesCamTelR().GetOrigin(TwotimesCamTelViewR);
	}
	telcamonforesight[mainOrsub].getTwoTimesCamTelR().SetOrigin(TwotimesCamTelViewR);
	static M3DVector3f TwotimesCamTelViewB;
	static bool onceB[2] ={true,true};
	if(onceB[mainOrsub]){
	onceR[mainOrsub] = false;
	telcamonforesight[mainOrsub].getTwoTimesCamTelB().RotateLocalX(-PI/2);
	telcamonforesight[mainOrsub].getTwoTimesCamTelB().MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);
	telcamonforesight[mainOrsub].getTwoTimesCamTelB().MoveRight(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())*3.0/4.0);
	telcamonforesight[mainOrsub].getTwoTimesCamTelB().MoveRight(-inidelta);
	telcamonforesight[mainOrsub].getTwoTimesCamTelB().MoveForward(-0.08*leftandrightdis/2);
	telcamonforesight[mainOrsub].getTwoTimesCamTelB().GetOrigin(TwotimesCamTelViewB);
	}
	telcamonforesight[mainOrsub].getTwoTimesCamTelB().SetOrigin(TwotimesCamTelViewB);
	static M3DVector3f TwotimesCamTelViewL;
	static bool onceL[2] ={true,true};
	if(onceL[mainOrsub]){
	onceL[mainOrsub] = false;
	telcamonforesight[mainOrsub].getTwoTimesCamTelL().RotateLocalX(-PI/2);
	telcamonforesight[mainOrsub].getTwoTimesCamTelL().MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);
	telcamonforesight[mainOrsub].getTwoTimesCamTelL().MoveRight(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())/1);
	telcamonforesight[mainOrsub].getTwoTimesCamTelL().MoveRight(-inidelta);
	telcamonforesight[mainOrsub].getTwoTimesCamTelL().MoveForward(-0.08*leftandrightdis/2);
	telcamonforesight[mainOrsub].getTwoTimesCamTelL().GetOrigin(TwotimesCamTelViewL);
	}
	telcamonforesight[mainOrsub].getTwoTimesCamTelL().SetOrigin(TwotimesCamTelViewL);
}

void  Render::GenerateFourtimesTelView(int mainOrsub)
{
	float Len=(render.get_PanelLoader().Getextent_pos_x()-render.get_PanelLoader().Getextent_neg_x());
	float inidelta=(RulerAngle)/360.0*Len;
	if(inidelta>270.0/360.0*Len)
	{
		inidelta-=Len;
		inidelta-=Len;
	}
	else
		inidelta-=Len;
	static M3DVector3f FourtimesCamTelViewF;
	static bool onceF[2] ={true,true};
	if(onceF[mainOrsub]){
	onceF[mainOrsub] = false;
	telcamonforesight[mainOrsub].getFourTimesCamTelF().RotateLocalX(-PI/2);
	telcamonforesight[mainOrsub].getFourTimesCamTelF().MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);
	telcamonforesight[mainOrsub].getFourTimesCamTelF().MoveRight(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())/4);
	telcamonforesight[mainOrsub].getFourTimesCamTelF().MoveRight(-inidelta);
	telcamonforesight[mainOrsub].getFourTimesCamTelF().MoveForward(-0.04*leftandrightdis);
	telcamonforesight[mainOrsub].getFourTimesCamTelF().GetOrigin(FourtimesCamTelViewF);
	}
	telcamonforesight[mainOrsub].getFourTimesCamTelF().SetOrigin(FourtimesCamTelViewF);

		static M3DVector3f FourtimesCamTelViewR;
		static bool onceR[2] ={true,true};
		if(onceR[mainOrsub]){
		onceR[mainOrsub] = false;
		telcamonforesight[mainOrsub].getFourTimesCamTelR().RotateLocalX(-PI/2);
		telcamonforesight[mainOrsub].getFourTimesCamTelR().MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);
		telcamonforesight[mainOrsub].getFourTimesCamTelR().MoveRight(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())/2);
		telcamonforesight[mainOrsub].getFourTimesCamTelR().MoveRight(-inidelta);
		telcamonforesight[mainOrsub].getFourTimesCamTelR().MoveForward(-0.04*leftandrightdis);
		telcamonforesight[mainOrsub].getFourTimesCamTelR().GetOrigin(FourtimesCamTelViewR);
		}
		telcamonforesight[mainOrsub].getFourTimesCamTelR().SetOrigin(FourtimesCamTelViewR);
		static M3DVector3f FourtimesCamTelViewB;
		static bool onceB[2]={true,true};
		if(onceB[mainOrsub]){
		onceR[mainOrsub] = false;
		telcamonforesight[mainOrsub].getFourTimesCamTelB().RotateLocalX(-PI/2);
		telcamonforesight[mainOrsub].getFourTimesCamTelB().MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);
		telcamonforesight[mainOrsub].getFourTimesCamTelB().MoveRight(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())*3.0/4.0);
		telcamonforesight[mainOrsub].getFourTimesCamTelB().MoveRight(-inidelta);
		telcamonforesight[mainOrsub].getFourTimesCamTelB().MoveForward(-0.04*leftandrightdis);
		telcamonforesight[mainOrsub].getFourTimesCamTelB().GetOrigin(FourtimesCamTelViewB);
		}
		telcamonforesight[mainOrsub].getFourTimesCamTelB().SetOrigin(FourtimesCamTelViewB);

		static M3DVector3f FourtimesCamTelViewL;
		static bool onceL[2] ={true,true};
		if(onceL[mainOrsub]){
		onceL[mainOrsub] = false;
		telcamonforesight[mainOrsub].getFourTimesCamTelL().RotateLocalX(-PI/2);
		telcamonforesight[mainOrsub].getFourTimesCamTelL().MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);
		telcamonforesight[mainOrsub].getFourTimesCamTelL().MoveRight(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())/1);
		telcamonforesight[mainOrsub].getFourTimesCamTelL().MoveRight(-inidelta);
		telcamonforesight[mainOrsub].getFourTimesCamTelL().MoveForward(-0.04*leftandrightdis);
		telcamonforesight[mainOrsub].getFourTimesCamTelL().GetOrigin(FourtimesCamTelViewL);
		}
		telcamonforesight[mainOrsub].getFourTimesCamTelL().SetOrigin(FourtimesCamTelViewL);
}

void  Render::GenerateCheckView()
{
	static M3DVector3f camCheckView;
	static bool once =true;
	if(once){
		once = false;
		CheckViewCameraFrame.RotateLocalX(-PI/2);
		CheckViewCameraFrame.MoveForward(-39.0f);
		CheckViewCameraFrame.MoveUp((BowlLoader.Getextent_pos_z()-BowlLoader.Getextent_neg_z())/2);
		CheckViewCameraFrame.MoveRight(0.0f);
		CheckViewCameraFrame.GetOrigin(camCheckView);
	}
	CheckViewCameraFrame.SetOrigin(camCheckView);
}

void Render::GenerateLeftSmallPanoView()
{
	static M3DVector3f camPanoView;
	static bool once =true;
	if(once){
		once = false;

		LeftSmallPanoViewCameraFrame.RotateLocalX(-PI/2);
		LeftSmallPanoViewCameraFrame.MoveForward(-leftandrightdis*SMALL_PANO_VIEW_SCALE);
		LeftSmallPanoViewCameraFrame.MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);
		LeftSmallPanoViewCameraFrame.MoveRight(-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x()-0.4));
		LeftSmallPanoViewCameraFrame.GetOrigin(camPanoView);
	}
	LeftSmallPanoViewCameraFrame.SetOrigin(camPanoView);
}

void Render::GenerateRightSmallPanoView()
{
	static M3DVector3f camPanoView;
	static bool once =true;
	if(once){
		once = false;

		RightSmallPanoViewCameraFrame.RotateLocalX(-PI/2);
		RightSmallPanoViewCameraFrame.MoveForward(leftandrightdis*SMALL_PANO_VIEW_SCALE);
		RightSmallPanoViewCameraFrame.MoveUp((PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2);

		RightSmallPanoViewCameraFrame.RotateLocalY(PI);
		RightSmallPanoViewCameraFrame.MoveRight((PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())*(3.0-0.025)+0.4);
		RightSmallPanoViewCameraFrame.GetOrigin(camPanoView);
	}
	RightSmallPanoViewCameraFrame.SetOrigin(camPanoView);
}

void Render::GenerateFrontView()
{
	static M3DVector3f camFrontView;
	static bool once =true;
	if(once){
		once = false;
		frontCameraFrame.MoveForward(-7.5f);
		frontCameraFrame.MoveUp(-14.0f);
		frontCameraFrame.RotateLocalX(float(m3dDegToRad(-63.0f)));

		frontCameraFrame.GetOrigin(camFrontView);
	}
	frontCameraFrame.SetOrigin(camFrontView);
}

void Render::GenerateRearTopView()
{
	static M3DVector3f camRearView;
		static bool once =true;
		if(once){
			once = false;
			rearTopCameraFrame.MoveForward(-14.5f);
			rearTopCameraFrame.MoveUp(-17.0f);
			rearTopCameraFrame.RotateLocalX(float(m3dDegToRad(-42.0f)));

			rearTopCameraFrame.GetOrigin(camRearView);
		}
		rearTopCameraFrame.SetOrigin(camRearView);
	rearTopCameraFrame.SetOrigin(camRearView);
}
bool Render::isDirectionMode()
{// when we can change direction using up/down/left/right
	return  (TRIPLE_VIEW_MODE == displayMode) ||
			(FREE_VIEW_MODE == displayMode) ||
			( SINGLE_PORT_MODE == displayMode)||
			(PREVIEW_MODE == displayMode)||
			(SPLIT_VIEW_MODE == displayMode);
}
/* The function called whenever a mouse motion event occurs */
void Render::mouseMotionPress(int x, int y)
{
	/* The mouse buttons */
#define LMB 0	/* Left */
#define MMB 1   /* Middle */
#define RMB 2   /* Right */
	float 	Z_Depth = BowlLoader.GetZ_Depth();
	static M3DVector3f camOrigin;
	float delta_x, delta_y, delta_z;

	if (common.isVerbose())
		printf("You did this with the mouse--> %i %i\n", x, y);
	 if( SINGLE_PORT_MODE == displayMode && p_CornerMarkerGroup->isAdjusting()){
		 if(BillBoard::BBD_FRONT == p_BillBoard->m_Direction){
			 	if(x < g_windowWidth/2){
			 		p_CornerMarkerGroup->Adjust(CORNER_FRONT_LEFT  );
			 	}else{
			 		p_CornerMarkerGroup->Adjust(CORNER_FRONT_RIGHT );
			 	}
		 }
		 else if(BillBoard::BBD_REAR == p_BillBoard->m_Direction){
			 if(x < g_windowWidth/2){
			 	p_CornerMarkerGroup->Adjust(CORNER_REAR_LEFT);
			 }else{
			 	p_CornerMarkerGroup->Adjust(CORNER_REAR_RIGHT);
			 }
		 }
	 }
	if (BUTTON == LMB)
	{
	     if( SINGLE_PORT_MODE == displayMode && p_CornerMarkerGroup->isAdjusting()){
			p_CornerMarkerGroup->Move(-0.0027*(MOUSEx -x), 0.0035*(MOUSEy-y));
		}
		delta_x = ((MOUSEx - x)*(tanf(PI/180*15)*(Z_Depth+scale)))*.005;
		delta_y = ((MOUSEy - y)*(tanf(PI/180*15)*(Z_Depth+scale)))*.005;
		setMouseCor(x,y);
		if(bControlViewCamera){
			birdViewCameraFrame.SetOrigin(camOrigin);
			birdViewCameraFrame.MoveRight(delta_x);
			birdViewCameraFrame.MoveUp(delta_y);
			birdViewCameraFrame.GetOrigin(camOrigin);
		}else{
			updatePano(delta_x, -delta_y);
		}

		if(FREE_VIEW_MODE == displayMode){
			m_freeCamera.Translate(delta_x, delta_y, 0.0f);
		}
		
	}
	if (BUTTON == MMB)
	{
		delta_x = ((MOUSEx - x)*0.5);
		delta_y = ((MOUSEy - y)*0.5);
		setMouseCor(x,y);

		if(bControlViewCamera){
			birdViewCameraFrame.SetOrigin(camOrigin);
			birdViewCameraFrame.RotateLocal((float)m3dDegToRad(delta_y),1.0f,0.0f,0.0f);
			birdViewCameraFrame.RotateLocal((float)m3dDegToRad(delta_x),0.0f,1.0f,0.0f);
			birdViewCameraFrame.GetOrigin(camOrigin);
		}else{
			updateRotate(-delta_x, -delta_y);
		}
		if(FREE_VIEW_MODE == displayMode){
			m_freeCamera.Rotate((float)m3dDegToRad(delta_y), 1.0f, 0.0f,0.0f);
			m_freeCamera.Rotate((float)m3dDegToRad(delta_x), 0.0f,0.0f,1.0f);
		}
	}
	if (BUTTON == RMB)
	{
		delta_z = ((MOUSEy - y)*(tanf(PI/180*15)*(Z_Depth+scale)))*.01;
		delta_x = ((MOUSEx - x)*0.5);
		setMouseCor(x,y);
		if(bControlViewCamera){
			birdViewCameraFrame.SetOrigin(camOrigin);
			birdViewCameraFrame.MoveForward(-delta_z);
			birdViewCameraFrame.GetOrigin(camOrigin);
		}else{
			updateScale(delta_z, delta_z);
			updateRotateZ(delta_x);
			/* scale = scale - ((MOUSEy - y)*0.05);
      		oScale = oScale - ((MOUSEy - y)*0.05);  */
		}
		if(FREE_VIEW_MODE == displayMode){
                    m_freeCamera.MoveForward(-delta_z);
		      m_freeCamera.Rotate((float)m3dDegToRad(delta_x), 0.0f, 1.0f,0.0f);
		}
	}
	if(common.isVerbose())
		cout<<"ROTx, ROTy, ROTz, scale = "<<ROTx<<", "<<ROTy<<", "<<ROTz<<","<<scale<<endl;
}

void Render::SetShowDirection(int dir,bool show_mobile)
{

		p_BillBoard->m_lastDirection=p_BillBoard->m_Direction;
		p_BillBoard->m_Direction=dir;
		if((TRIPLE_VIEW_MODE == displayMode)&&(show_mobile)){
			m_presetCameraRotateCounter = PRESET_CAMERA_ROTATE_MAX;
		}

}

void Render::ProcessOitKeys(GLEnv &m_env,unsigned char key, int x, int y)
{
GLEnv & env=env1;

	int Now_Window_Width,Now_Window_Height;
	Now_Window_Width=glutGet(GLUT_WINDOW_WIDTH);
	Now_Window_Width=glutGet(GLUT_WINDOW_HEIGHT);
	int Milx=0;
	bool istochangeTelMode=false;
	float pano_pos2angle=0.0;

	float mydelta=0.0;
	float ori_ruler_angle=0.0;
	float now_ruler_angle=0.0;

#define DISPLAYMODE_SWITCH_TO(mode) 		{\
			DISPLAYMODE nextMode = mode;\
			if(BACK_VIEW_MODE == displayMode){\
				p_BillBoard->m_Direction = lastSingleViewDirection;\
			}else{\
			       lastSingleViewDirection = p_BillBoard->m_Direction;\
			}\
			displayMode = nextMode;\
		}
	char cmdBuf[128];
	static int mode = OitVehicle::USER_BLEND;
	static int blendMode = 6;
	float cross_pos[2];
	float set_follow_angle[2];//0:hor;1:ver;
	float panel_pos_hor_start=0.0;
	float panel_pos_ver_start=0.0;
	float math_ruler_angle=0.0;


        float zoom_data = 0.0;  //getALPHA_ZOOM_SCALE();
        GLint nWidth = DEFAULT_IMAGE_WIDTH, nHeight = DEFAULT_IMAGE_HEIGHT,
                        nComponents = GL_RGBA8;
        GLenum format = GL_BGRA;

	static unsigned int lastSingleViewDirection = BillBoard::BBD_FRONT;
	do{
		switch(key)
		{
		case ';':
			stop_scan=!stop_scan;
			break;
		//case 'o':
		case 'O':
#if MVDECT
			if(IsMvDetect==false)
				IsMvDetect=true;
			else if(IsMvDetect==true)
				IsMvDetect=false;
			//mv_detect.OpenMD(MAIN);
#endif
		//	mode = OitVehicle::USER_OIT;
			break;
		//case 'b':
		case 'B':
			mode = OitVehicle::USER_BLEND;
			break;
		case '1':
			displayMode=ALL_VIEW_MODE;
			break;
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
			blendMode = key-'0';
						if((INIT_VIEW_MODE==displayMode)&&(EnterSinglePictureSaveMode==true))
						{
							enterNumberofCam=enterNumberofCam*10+blendMode;
						}
						else
						{
							if(isDirectionOn)
								{
								    p_BillBoard->setBlendMode(blendMode);
								    FixedBillBoard::setBlendMode(blendMode);
								}
						}
						break;
		case '8':
					blendMode = key-'0';
					if((INIT_VIEW_MODE==displayMode)&&(EnterSinglePictureSaveMode==true))
					{
						enterNumberofCam=enterNumberofCam*10+blendMode;
					}
					break;
		//case 's' :
		case 'S':
		{
			DISPLAYMODE nextMode = DISPLAYMODE(((int)displayMode+1) % TOTAL_MODE_COUNT);
			//cout<<"NEXTMODE=="<<nextMode<<endl;

			if(nextMode==TELESCOPE_FRONT_MODE)
			{
				nextMode=ALL_VIEW_MODE;
			//	 nextMode = DISPLAYMODE(((int)displayMode+15) % TOTAL_MODE_COUNT);
			}
			bool isEnterringBackView = (nextMode == BACK_VIEW_MODE);
			if(isEnterringBackView)
			{
				lastSingleViewDirection = p_BillBoard->m_Direction;
			}else if(BACK_VIEW_MODE == displayMode){
				p_BillBoard->m_Direction = lastSingleViewDirection;
			}
			#if TRACK_MODE
				if(nextMode==VGA_WHITE_VIEW_MODE)
				{
					trackMode->SwitchTrackChId(vga_Track);
				}else if(nextMode==SDI1_WHITE_BIG_VIEW_MODE)
				{
					trackMode->SwitchTrackChId(sdi_Track);
				}else if(nextMode==PAL1_WHITE_BIG_VIEW_MODE)
				{
					trackMode->SwitchTrackChId(pal_Track);
				}else if(nextMode==PAL2_HOT_BIG_VIEW_MODE)
				{
					trackMode->SwitchTrackChId(pal2_Track);
				}
				if(isTracking)
				{
					int set_track_params[4];
					memcpy(set_track_params,gettrackcontrolparams(),sizeof(set_track_params));
					int trkId=getTrkId(displayMode,nextMode);		
					if(trkId==-1)
					{
						isTracking=false;
						trackMode->disableTrack(sdi_Track);
						trackMode->disableTrack(pal_Track);
						trackMode->disableTrack(vga_Track);
						trackMode->disableTrack(pal2_Track);

					}else{
					if(trkId==sdi_Track){
						trackMode->disableTrack(trkId);
						clearTrackParams();
				trackMode->enableTrackAcq(set_track_params[0],set_track_params[1],
						set_track_params[2],set_track_params[3],trkId);
					}else if(trkId==vga_Track){
					clearTrackParams();
					trackMode->enableTrackAcq(482,354,
						60,60,trkId);
						}else if(trkId==pal_Track){
						clearTrackParams();
						trackMode->enableTrackAcq(330,258,
						60,60,trkId);
							}else if(trkId==pal2_Track){
							clearTrackParams();
							trackMode->enableTrackAcq(330,258,
							60,60,trkId);}
						}
				}
				
			#endif
			displayMode = nextMode;
			if( PREVIEW_MODE  == displayMode ){
				bRotTimerStart = true;
			}
		}
			break;
		case	'n':
		{
			chosenCam[MAIN]=chosenCam[MAIN]+1;
			if(chosenCam[MAIN]==11)
				chosenCam[MAIN]=1;
			ChangeMainChosenCamidx(chosenCam[MAIN]);
	//		FBO_MODE nextMode=FBO_MODE(((int)fboMode+1)%FBO_MODE_COUNT);
	//		fboMode = nextMode;
		}
		break;
		case 'L':
		//case 'l':
			DISPLAYMODE_SWITCH_TO(TRIPLE_VIEW_MODE);
			break;
		case 'P':
		//case 'p':
			DISPLAYMODE_SWITCH_TO(SPLIT_VIEW_MODE);
			break;
		case 'M':
		//case 'm':
			DISPLAYMODE_SWITCH_TO(EXTENSION_VIEW_MODE);
			break;
		//case 'f':
		case 'F':
			//full screen on/off
			if(isFullscreen){
				isFullscreen = false;
				glutReshapeWindow(g_nonFullwindowWidth, g_nonFullwindowHeight);
				glutPostRedisplay();
			}else{
				isFullscreen = true;
				glutFullScreen();
			}
			break;
		//case 'c':
		case 'C':
	//		if(isCalibTimeOn){
		//		RememberTime();
	//		}
	//		isCalibTimeOn = !isCalibTimeOn;
			break;
		//case 'd':
		case 'D':
			isDirectionOn = !isDirectionOn;
			break;
		//case 'q':
		case 'Q':
			DISPLAYMODE_SWITCH_TO(SINGLE_PORT_MODE);
			break;
		case 'A':
		//case 'a':
			{
			DISPLAYMODE nextMode = BACK_VIEW_MODE;
			if(BACK_VIEW_MODE != displayMode){
				 lastSingleViewDirection=p_BillBoard->m_Direction;
			}
			displayMode = nextMode;
		       }
			break;
		//case 'i':
		case 'I':
		{
			DISPLAYMODE nextMode = INIT_VIEW_MODE;
			if(BACK_VIEW_MODE != displayMode){
				p_BillBoard->m_Direction = lastSingleViewDirection;
			}
			displayMode = nextMode;
		}
			break;
		//case 'u'://forward
		case 'U':
			SetShowDirection(BillBoard::BBD_FRONT, SHOW_DIRECTION_DYNAMIC);
			break;
		//case 'v'://right forward
		case 'V':
			SetShowDirection(BillBoard::BBD_FRONT_RIGHT, SHOW_DIRECTION_DYNAMIC);
			break;
		//case 'w'://right back
		case 'W':
			SetShowDirection(BillBoard::BBD_REAR_RIGHT, SHOW_DIRECTION_DYNAMIC);
			break;
		//case 'x'://back
		case 'X':
			SetShowDirection(BillBoard::BBD_REAR, SHOW_DIRECTION_DYNAMIC);
			break;
		//case 'y'://left back
		case 'Y':
			SetShowDirection(BillBoard::BBD_REAR_LEFT, SHOW_DIRECTION_DYNAMIC);
			break;
		//case 'z'://left forward
		case 'Z':
			SetShowDirection(BillBoard::BBD_FRONT_LEFT, SHOW_DIRECTION_DYNAMIC);
			break;
		//case 't'://all + 712
		case 'T':
			DISPLAYMODE_SWITCH_TO(ALL_ADD_712_MODE);
			break;
		//case 'e'://view from center
		case 'E':
			DISPLAYMODE_SWITCH_TO(CENTER_VIEW_MODE);
			break;
		case 'R':
			stopcenterviewrotate=!stopcenterviewrotate;
			break;
		//case 'j':
		case 'J':
			if(rotateangle_per_second<72)
			{
				rotateangle_per_second=rotateangle_per_second+2;
			}
			break;
		//case 'k':
		case 'K':
			if(rotateangle_per_second>4)
			{
				rotateangle_per_second=rotateangle_per_second-2;
			}
			break;
		case 'H':
		//case 'h':
		{
					common.setScanned(true);
					DISPLAYMODE nextMode = INIT_VIEW_MODE;
					if(BACK_VIEW_MODE != displayMode){
						p_BillBoard->m_Direction = lastSingleViewDirection;
					}
					displayMode = nextMode;

					if((INIT_VIEW_MODE==displayMode)&&(EnterSinglePictureSaveMode==true))
					{
						SetThreadStitchState(STITCH_ONLY);
					}
					else
					{
						SetThreadStitchState(SAVEPIC_STITCH);
					}
				}
					break;
		case 'r':
			if(EnablePanoFloat==true&&TRIM_MODE==displayMode)
			{
					shaderManager.ResetTrimColor();
					testPanoNumber=(CAM_COUNT-testPanoNumber-1)%CAM_COUNT;
					shaderManager.SetTrimColor(testPanoNumber);
			}
			break;
		case 't'://PANO add PTZ view
			break;
		case 'y'://two half PANO view
			DISPLAYMODE_SWITCH_TO(TWO_HALF_PANO_VIEW_MODE);
			break;
		case 'u'://single high quality view
			break;
		case 'i'://pano view mode
			DISPLAYMODE_SWITCH_TO(PANO_VIEW_MODE);
			break;
		case 'o'://high definition pano view
#if MVDECT
	//		mv_detect.CloseMD(MAIN);
#endif
			break;
		case 'p'://high definition pano add pano view
			break;
		case 'w'://up
			if(getFollowValue())
			{
				memcpy(cross_pos,getCrossCenterPos(),sizeof(cross_pos));
				if(cross_pos[1]>(PanelLoader.Getextent_pos_z())*CROSS_VER_UP_SCALE)
				{
					;
				}
				else
				{
					cross_pos[1]=cross_pos[1]+CROSS_VER_STEP;
					setCrossCenterPos(cross_pos);
				}
			}
			break;
		case 's'://down
			if(getFollowValue())
						{
							memcpy(cross_pos,getCrossCenterPos(),sizeof(cross_pos));
							if(cross_pos[1]<(PanelLoader.Getextent_pos_z())*CROSS_VER_DOWN_SCALE)
							{
								;
							}
							else
							{
								cross_pos[1]=cross_pos[1]-CROSS_VER_STEP;
								setCrossCenterPos(cross_pos);
							}
						}

						if((INIT_VIEW_MODE==displayMode)&&(EnterSinglePictureSaveMode==true))
						{

							if(enterNumberofCam>=0&&enterNumberofCam<CAM_COUNT)
							{
								env.GetPanoCaptureGroup()->saveSingleCapImg(enterNumberofCam);
								picSaveState[enterNumberofCam]=true;
							}
							enterNumberofCam=0;
						}
						break;
		case 'a'://left
			if(getFollowValue())
			{
				memcpy(cross_pos,getCrossCenterPos(),sizeof(cross_pos));
				cross_pos[0]=cross_pos[0]-CROSS_HOR_STEP;
				if(cross_pos[0]<(PanelLoader.Getextent_neg_x()))
				{
					cross_pos[0]=-cross_pos[0]+PanelLoader.Getextent_pos_x()+PanelLoader.Getextent_neg_x();
				}
				else
				{
					;
				}
				setCrossCenterPos(cross_pos);
			}

           	if(EnablePanoFloat==true)
            	{
    				if((TRIM_MODE==displayMode))
    				{
 						move_hor[testPanoNumber]=move_hor[testPanoNumber]+DELTA_OF_PANO_HOR;
						InitPanel(m_env,0,true);
    				}
            	}

			break;
		case 'd'://right
			if(getFollowValue())
			{
				memcpy(cross_pos,getCrossCenterPos(),sizeof(cross_pos));
				cross_pos[0]=cross_pos[0]+CROSS_HOR_STEP;
				if(cross_pos[0]>(PanelLoader.Getextent_pos_x()))
				{
					cross_pos[0]=-cross_pos[0]+PanelLoader.Getextent_pos_x()+PanelLoader.Getextent_neg_x();
				}
				else
				{
					;
				}
				setCrossCenterPos(cross_pos);
			}
           	if(EnablePanoFloat==true)
            	{
    				if((TRIM_MODE==displayMode))
    				{
 						move_hor[testPanoNumber]=move_hor[testPanoNumber]-DELTA_OF_PANO_HOR;
						InitPanel(m_env,0,true);
    				}
            	}

			break;
		case 'q'://enter
			if(getFollowValue())
			{
				panel_pos_hor_start=(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())/2;
				panel_pos_ver_start=(PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2;
				memcpy(cross_pos,getCrossCenterPos(),sizeof(cross_pos));
				set_follow_angle[0]=(cross_pos[0]-panel_pos_hor_start)*360.0/(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
				set_follow_angle[1]=180.0*atan((cross_pos[1]-panel_pos_ver_start)/PanelLoader.Getextent_pos_y())/PI;

				setFollowAngle(set_follow_angle);
				setSendFollowAngleEnable(true);
				printf("\nstart to send\n");

			}

	     	if(EnablePanoFloat==true)
	            	{
	    				if((TRIM_MODE==displayMode))
	    				{
	 						rotate_angle[testPanoNumber]+=DELTA_OF_ROTATE_ANGLE;
	 						if(rotate_angle[testPanoNumber]>360.0)
	 						{
	 							rotate_angle[testPanoNumber]-=360.0;
	 						}
							InitPanel(m_env,0,true);
	    				}
	            	}
			break;
		case 'e':
	     	if(EnablePanoFloat==true)
	            	{
	    				if((TRIM_MODE==displayMode))
	    				{
	 						rotate_angle[testPanoNumber]-=DELTA_OF_ROTATE_ANGLE;
	 						if(rotate_angle[testPanoNumber]<0.0)
	 						{
	 							rotate_angle[testPanoNumber]+=360.0;
	 						}
							InitPanel(m_env,0,true);
							
	    				}
	            	}
			break;
		case 'G'://PTZ--CCD
			break;
		case 'g'://PTZ--FIR
			if(IsgstCap==false)
			{
				IsgstCap=true;
			}
			else
			{
				IsgstCap=false;
			}
			break;
		case 'z'://steady on
			break;
		case 'x'://steady off
			break;
		case 'c'://follow on
			if(!getFollowValue())
						{
							setFollowVaule(true);
						}

						if((INIT_VIEW_MODE==displayMode)&&(EnterSinglePictureSaveMode==true))
						{
							for(int cam_num=0;cam_num<MAX_PANO_CAMERA_COUNT;cam_num++)
							{
								picSaveState[cam_num]=false;
							}
							enterNumberofCam=0;
						}
						break;
		case 'v'://follow off
			setFollowVaule(false);
			break;
		case '['://show ruler
			if(displayMode==PANO_VIEW_MODE||displayMode==TWO_HALF_PANO_VIEW_MODE
					||displayMode==FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE
					||displayMode==FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE
					||displayMode==		ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
					)
			{
				setenableshowruler(true);
			}
			break;
		case ']'://hide ruler
			if(displayMode==PANO_VIEW_MODE||displayMode==TWO_HALF_PANO_VIEW_MODE
					||displayMode==FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE
					||displayMode==FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE
					||displayMode==	ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
					)
    		{
				setenableshowruler(false);
    		}
			break;
		case ','://move ruler to left
			if(getenableshowruler())
			{
				math_ruler_angle=p_LineofRuler->GetAngle();//getrulerangle();
				math_ruler_angle=math_ruler_angle-RULER_ANGLE_MOVE_STEP;
				if(math_ruler_angle>0.0)
				{
					p_LineofRuler->SetAngle(math_ruler_angle);//setrulerangle(math_ruler_angle);
				}
				else
				{
					p_LineofRuler->SetAngle(math_ruler_angle+360.0);//setrulerangle(math_ruler_angle+360.0);
				}

			}
		//	camonforesight.MoveLimit(MOVE_LEFT);
			if((EnablePanoFloat==true))
			{
				if((displayMode==TRIM_MODE))
				{
					PanoDirectionLeft=true;
				}
			}


			break;
		case '.'://move ruler to right
			if(getenableshowruler())
			{
				math_ruler_angle=p_LineofRuler->GetAngle();//getrulerangle();
				math_ruler_angle=math_ruler_angle+RULER_ANGLE_MOVE_STEP;
				if(math_ruler_angle<360.0)
				{
					p_LineofRuler->SetAngle(math_ruler_angle);//setrulerangle(math_ruler_angle);
				}
				else
				{
					p_LineofRuler->SetAngle(math_ruler_angle-360.0);//setrulerangle(math_ruler_angle-360.0);
				}
			}
			if(EnablePanoFloat==true)
			{
				if(displayMode==TRIM_MODE)
				{
					PanoDirectionLeft=false;
				}
			}
//			camonforesight.MoveLimit(MOVE_RIGHT);
			break;
		case '9':
			blendMode = key-'0';
						if((INIT_VIEW_MODE==displayMode)&&(EnterSinglePictureSaveMode==true))
						{
							enterNumberofCam=enterNumberofCam*10+blendMode;
						}
						else
						{
							DISPLAYMODE_SWITCH_TO(FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE);
						}
						break;
		case '0':
			blendMode = key-'0';
						if((INIT_VIEW_MODE==displayMode)&&(EnterSinglePictureSaveMode==true))
						{
							enterNumberofCam=enterNumberofCam*10+blendMode;
						}
						else
						{
							DISPLAYMODE_SWITCH_TO(FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE);
						}
						break;
                case '+':       //add alpha
                	if(EnablePanoFloat==true)
                	{
        				if((TRIM_MODE==displayMode))
        				{
							if(PanoDirectionLeft==true)
							{
								channel_left_scale[testPanoNumber]=channel_left_scale[testPanoNumber]-DELTA_OF_PANO_SCALE;
							}
							else
							{
								channel_right_scale[testPanoNumber]=channel_right_scale[testPanoNumber]-DELTA_OF_PANO_SCALE;
							}
							InitPanel(m_env,0,true);
        				}
                	}
                	else
                	{
                        zoom_data = getALPHA_ZOOM_SCALE();
                        zoom_data += 0.1;
                        if (zoom_data >= 1.1) {
                                ;
                        } else {
                                setALPHA_ZOOM_SCALE(zoom_data);

                                initAlphaMask();
                                glBindTexture(GL_TEXTURE_2D, textures[ALPHA_TEXTURE_IDX]);
                                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                                                GL_LINEAR);
                                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                                                GL_LINEAR);
                                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
                                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
                                glTexImage2D(GL_TEXTURE_2D, 0, nComponents, ALPHA_MASK_WIDTH,
                                                ALPHA_MASK_HEIGHT, 0, format, GL_UNSIGNED_BYTE,
                                                alphaMask);
                        }
                	}
			break;
                case '-':       //dec alpha
                 	if(EnablePanoFloat==true)
                	{
        				if((TRIM_MODE==displayMode))
        				{
							if(PanoDirectionLeft==true)
							{
								channel_left_scale[testPanoNumber]=channel_left_scale[testPanoNumber]+DELTA_OF_PANO_SCALE;
							}
							else
							{
								channel_right_scale[testPanoNumber]=channel_right_scale[testPanoNumber]+DELTA_OF_PANO_SCALE;
							}
							InitPanel(m_env,0,true);
        				}
                	}
                	else
                	{
                        zoom_data = getALPHA_ZOOM_SCALE();
                        zoom_data -= 0.1;
                        if (zoom_data < 0.0) {
                                ;
                        } else {
                                setALPHA_ZOOM_SCALE(zoom_data);

                                initAlphaMask();
                                glBindTexture(GL_TEXTURE_2D, textures[ALPHA_TEXTURE_IDX]);
                                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                                                GL_LINEAR);
                                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                                                GL_LINEAR);
                                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
                                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
                                glTexImage2D(GL_TEXTURE_2D, 0, nComponents, ALPHA_MASK_WIDTH,
                                                ALPHA_MASK_HEIGHT, 0, format, GL_UNSIGNED_BYTE,
                                                alphaMask);
                        }
                	}
			break;
                case '=':       //save alpha data
                        zoom_data = getALPHA_ZOOM_SCALE();
                        render.writeALPHA_ZOOM_SCALE("ALPHA_ZOOM_SCALE.txt", zoom_data);
			break;

			case '!':
				if(displayMode==ALL_VIEW_MODE)
				{
							p_ForeSightFacade[MAIN]->MoveLeft(-PanoLen*100.0);
					//		printf("m_cam_pos=%d\n",m_cam_pos);
						//	foresightPos.GetAngle()[0];
					//		foresightPos.ShowPosX();
					//		pano_pos2angle=p_ForeSightFacade->GetForeSightPosX()/PanoLen*360.0;
					//		printf("POS_angle=%f\n",pano_pos2angle);
				}
				else	if(displayMode==	ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
													||displayMode==PREVIEW_MODE)
				{
//							p_ForeSightFacade[MAIN]->MoveLeft(-PanoLen*100.0);
					//		pano_pos2angle=p_ForeSightFacade->GetForeSightPosX()/PanoLen*360.0;
				//			printf("POS_angle=%f\n",pano_pos2angle);
				}

				else if(displayMode==TELESCOPE_FRONT_MODE
										||displayMode==TELESCOPE_RIGHT_MODE
										||displayMode==TELESCOPE_BACK_MODE
										||displayMode==TELESCOPE_LEFT_MODE)
				{
					istochangeTelMode=p_ForeSightFacade2[MAIN]->MoveLeft(-PanoLen/TELXLIMIT);
					if(displayMode==TELESCOPE_FRONT_MODE)
						p_ForeSightFacade[MAIN]->GetTelMil(0);
					else if(displayMode==TELESCOPE_RIGHT_MODE)
						p_ForeSightFacade[MAIN]->GetTelMil(1);
					else if(displayMode==TELESCOPE_BACK_MODE)
						p_ForeSightFacade[MAIN]->GetTelMil(2);
					else if(displayMode==TELESCOPE_LEFT_MODE)
						p_ForeSightFacade[MAIN]->GetTelMil(3);
					if(istochangeTelMode)
					{
						DISPLAYMODE nextMode = DISPLAYMODE(((int)displayMode-1) % TOTAL_MODE_COUNT);
						if(nextMode==ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE)
						{
							 nextMode = TELESCOPE_LEFT_MODE;//DISPLAYMODE(((int)displayMode+15) % TOTAL_MODE_COUNT);
						}
						displayMode=nextMode;
		/*				if(displayMode==TELESCOPE_FRONT_MODE)
							displayMode=TELESCOPE_LEFT_MODE;
						else if(displayMode==TELESCOPE_RIGHT_MODE)
							displayMode=TELESCOPE_FRONT_MODE;
						else if(displayMode==TELESCOPE_BACK_MODE)
							displayMode=TELESCOPE_RIGHT_MODE;
						else if(displayMode==TELESCOPE_LEFT_MODE)
							displayMode=TELESCOPE_BACK_MODE;*/
					}
				}
				else if(displayMode==	VGA_WHITE_VIEW_MODE
						||displayMode==VGA_HOT_BIG_VIEW_MODE
						||displayMode==VGA_HOT_SMALL_VIEW_MODE
						||displayMode==VGA_FUSE_WOOD_LAND_VIEW_MODE
						||displayMode==VGA_FUSE_GRASS_LAND_VIEW_MODE
						||displayMode==VGA_FUSE_SNOW_FIELD_VIEW_MODE
						||displayMode==VGA_FUSE_DESERT_VIEW_MODE
						||displayMode==VGA_FUSE_CITY_VIEW_MODE)
				{
					p_ForeSightFacade_Track->TrackMoveLeft(-PanoLen/37.685200*15.505);
		//			printf("panolen=%f\n",PanoLen);
		//			foresightPos.ShowPosX();
				}
				else if(displayMode==SDI1_WHITE_BIG_VIEW_MODE
						||displayMode==SDI1_WHITE_SMALL_VIEW_MODE
						||displayMode==SDI2_HOT_BIG_VIEW_MODE
						||displayMode==SDI2_HOT_SMALL_VIEW_MODE)
				{
					p_ForeSightFacade_Track->TrackMoveLeft(-PanoLen/37.685200*15.505);
		//			printf("panolen=%f\n",PanoLen);
		//			foresightPos.ShowPosX();
				}
				else if(displayMode==PAL1_WHITE_BIG_VIEW_MODE
						||displayMode==PAL1_WHITE_SMALL_VIEW_MODE
						||displayMode==PAL2_HOT_BIG_VIEW_MODE
						||displayMode==PAL2_HOT_SMALL_VIEW_MODE)
				{
					p_ForeSightFacade_Track->TrackMoveLeft(-PanoLen/37.685200*14.524);
			//		printf("panolen=%f\n",PanoLen);
			//		foresightPos.ShowPosX();
				}
			break;
			case '@':
				if(displayMode==	ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
													||displayMode==PREVIEW_MODE
													)
							{
		//							p_ForeSightFacade[MAIN]->MoveRight(PanoLen*100.0);
		//							pano_pos2angle=p_ForeSightFacade[MAIN]->GetForeSightPosX()/PanoLen*360.0;

									//			printf("POS_angle=%f\n",pano_pos2angle);
							}
				if(displayMode==ALL_VIEW_MODE)
				{
					p_ForeSightFacade[MAIN]->MoveRight(PanoLen*100.0);
			//		foresightPos.GetAngle()[0];
		//			printf("m_cam_pos=%d\n",m_cam_pos);
				//	foresightPos.ShowPosX();
				}
							else if(displayMode==TELESCOPE_FRONT_MODE
													||displayMode==TELESCOPE_RIGHT_MODE
													||displayMode==TELESCOPE_BACK_MODE
													||displayMode==TELESCOPE_LEFT_MODE)
							{
								istochangeTelMode=p_ForeSightFacade2[MAIN]->MoveRight(PanoLen/TELXLIMIT);
								if(displayMode==TELESCOPE_FRONT_MODE)
													p_ForeSightFacade2[MAIN]->GetTelMil(0);
												else if(displayMode==TELESCOPE_RIGHT_MODE)
													p_ForeSightFacade[MAIN]->GetTelMil(1);
												else if(displayMode==TELESCOPE_BACK_MODE)
													p_ForeSightFacade[MAIN]->GetTelMil(2);
												else if(displayMode==TELESCOPE_LEFT_MODE)
													p_ForeSightFacade[MAIN]->GetTelMil(3);


								if(istochangeTelMode)
								{
									DISPLAYMODE nextMode = DISPLAYMODE(((int)displayMode+1) % TOTAL_MODE_COUNT);
									if(nextMode==SDI1_WHITE_BIG_VIEW_MODE)
									{
										 nextMode = TELESCOPE_FRONT_MODE;//DISPLAYMODE(((int)displayMode+15) % TOTAL_MODE_COUNT);
									}
									displayMode=nextMode;
							/*		if(displayMode==TELESCOPE_FRONT_MODE)
									{
										displayMode=TELESCOPE_RIGHT_MODE;
									}
									else if(displayMode==TELESCOPE_RIGHT_MODE)
									{
										displayMode=TELESCOPE_BACK_MODE;
									}
									else if(displayMode==TELESCOPE_BACK_MODE)
									{
										displayMode=TELESCOPE_LEFT_MODE;
									}
									else if(displayMode==TELESCOPE_LEFT_MODE)
									{
										displayMode=TELESCOPE_FRONT_MODE;
									}*/
								}
							}
							else if(displayMode==	VGA_WHITE_VIEW_MODE
									||displayMode==VGA_HOT_BIG_VIEW_MODE
									||displayMode==VGA_HOT_SMALL_VIEW_MODE
									||displayMode==VGA_FUSE_WOOD_LAND_VIEW_MODE
									||displayMode==VGA_FUSE_GRASS_LAND_VIEW_MODE
									||displayMode==VGA_FUSE_SNOW_FIELD_VIEW_MODE
									||displayMode==VGA_FUSE_DESERT_VIEW_MODE
									||displayMode==VGA_FUSE_CITY_VIEW_MODE)
							{
								p_ForeSightFacade_Track->TrackMoveRight(PanoLen/37.685200*15.505);
					//			printf("panolen=%f\n",PanoLen);
					//			foresightPos.ShowPosX();
							}
							else if(displayMode==SDI1_WHITE_BIG_VIEW_MODE
									||displayMode==SDI1_WHITE_SMALL_VIEW_MODE
									||displayMode==SDI2_HOT_BIG_VIEW_MODE
									||displayMode==SDI2_HOT_SMALL_VIEW_MODE)
							{
								p_ForeSightFacade_Track->TrackMoveRight(PanoLen/37.685200*15.505);
						//		printf("panolen=%f\n",PanoLen);
						//		foresightPos.ShowPosX();
							}
							else if(displayMode==PAL1_WHITE_BIG_VIEW_MODE
									||displayMode==PAL1_WHITE_SMALL_VIEW_MODE
									||displayMode==PAL2_HOT_BIG_VIEW_MODE
									||displayMode==PAL2_HOT_SMALL_VIEW_MODE)
							{
								p_ForeSightFacade_Track->TrackMoveRight(PanoLen/37.685200*14.524);
						//		printf("panolen=%f\n",PanoLen);
						//		foresightPos.ShowPosX();
							}
			break;
			case '#':
			{

				if(displayMode==	ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
													||displayMode==PREVIEW_MODE)
							{
											p_ForeSightFacade[MAIN]->MoveUp(PanoHeight/5.7);
							}
				else if (displayMode==ALL_VIEW_MODE)
				{
					p_ForeSightFacade[MAIN]->MoveUp(PanoHeight/(OUTER_RECT_AND_PANO_TWO_TIMES_CAM_LIMIT));
				}

							else if(displayMode==TELESCOPE_FRONT_MODE
													||displayMode==TELESCOPE_RIGHT_MODE
													||displayMode==TELESCOPE_BACK_MODE
													||displayMode==TELESCOPE_LEFT_MODE)
							{
											p_ForeSightFacade2[MAIN]->MoveUp(PanoHeight/5.7);
							}
							else if(displayMode==	VGA_WHITE_VIEW_MODE
													||displayMode==VGA_HOT_BIG_VIEW_MODE
													||displayMode==VGA_HOT_SMALL_VIEW_MODE
													||displayMode==VGA_FUSE_WOOD_LAND_VIEW_MODE
													||displayMode==VGA_FUSE_GRASS_LAND_VIEW_MODE
													||displayMode==VGA_FUSE_SNOW_FIELD_VIEW_MODE
													||displayMode==VGA_FUSE_DESERT_VIEW_MODE
													||displayMode==VGA_FUSE_CITY_VIEW_MODE)
											{
												p_ForeSightFacade_Track->TrackMoveUp(PanoHeight/6.0000*11.600);
								//				printf("panoheight=%f\n",PanoHeight);
								//				foresightPos.ShowPosY();
											}
											else if(displayMode==SDI1_WHITE_BIG_VIEW_MODE
													||displayMode==SDI1_WHITE_SMALL_VIEW_MODE
													||displayMode==SDI2_HOT_BIG_VIEW_MODE
													||displayMode==SDI2_HOT_SMALL_VIEW_MODE)
											{
												p_ForeSightFacade_Track->TrackMoveUp(PanoHeight/6.0000*11.600);
									//			printf("panoheight=%f\n",PanoHeight);
								//				foresightPos.ShowPosY();
											}
											else if(displayMode==PAL1_WHITE_BIG_VIEW_MODE
													||displayMode==PAL1_WHITE_SMALL_VIEW_MODE
													||displayMode==PAL2_HOT_BIG_VIEW_MODE
													||displayMode==PAL2_HOT_SMALL_VIEW_MODE)
											{
												p_ForeSightFacade_Track->TrackMoveUp(PanoHeight/6.0000*11.600);
									//			printf("panoheight=%f\n",PanoHeight);
									//			foresightPos.ShowPosY();
											}
			}
			break;
			case '$':
			{
				if(displayMode==	ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
													||displayMode==PREVIEW_MODE)
				{
								p_ForeSightFacade[MAIN]->MoveDown(-PanoHeight/5.7);
				}
				else if (displayMode==ALL_VIEW_MODE)
				{
					p_ForeSightFacade[MAIN]->MoveDown(-PanoHeight/(20));
				}
				else if(displayMode==TELESCOPE_FRONT_MODE
										||displayMode==TELESCOPE_RIGHT_MODE
										||displayMode==TELESCOPE_BACK_MODE
										||displayMode==TELESCOPE_LEFT_MODE)
				{
								p_ForeSightFacade2[MAIN]->MoveDown(-PanoHeight/OUTER_RECT_AND_PANO_TWO_TIMES_CAM_LIMIT);
				}
				else if(displayMode==	VGA_WHITE_VIEW_MODE
										||displayMode==VGA_HOT_BIG_VIEW_MODE
										||displayMode==VGA_HOT_SMALL_VIEW_MODE
										||displayMode==VGA_FUSE_WOOD_LAND_VIEW_MODE
										||displayMode==VGA_FUSE_GRASS_LAND_VIEW_MODE
										||displayMode==VGA_FUSE_SNOW_FIELD_VIEW_MODE
										||displayMode==VGA_FUSE_DESERT_VIEW_MODE
										||displayMode==VGA_FUSE_CITY_VIEW_MODE)
								{
									p_ForeSightFacade_Track->TrackMoveDown(-PanoHeight/6.0000*11.600);
							//		printf("panolen=%f\n",PanoLen);
								//	foresightPos.ShowPosY();
								}
								else if(displayMode==SDI1_WHITE_BIG_VIEW_MODE
										||displayMode==SDI1_WHITE_SMALL_VIEW_MODE
										||displayMode==SDI2_HOT_BIG_VIEW_MODE
										||displayMode==SDI2_HOT_SMALL_VIEW_MODE)
								{
									p_ForeSightFacade_Track->TrackMoveDown(-PanoHeight/6.0000*11.600);
						//			printf("panoheight=%f\n",PanoHeight);
						//			foresightPos.ShowPosY();
								}
								else if(displayMode==PAL1_WHITE_BIG_VIEW_MODE
										||displayMode==PAL1_WHITE_SMALL_VIEW_MODE
										||displayMode==PAL2_HOT_BIG_VIEW_MODE
										||displayMode==PAL2_HOT_SMALL_VIEW_MODE)
								{
									p_ForeSightFacade_Track->TrackMoveDown(-PanoHeight/6.0000*11.600);
						//			printf("panoheight=%f\n",PanoHeight);
						//			foresightPos.ShowPosY();
								}
			}
			break;
			case '~':
			{
				if(CHOSEN_VIEW_MODE==displayMode)
				{
				#if 1//USE_CAP_SPI
					saveSinglePic[chosenCam[MAIN]-1]=true;
				//		printf("chosenCam %d\n",chosenCam[MAIN]);
				//	printf("savePic=%d\n",chosenCam[MAIN]-1);
				#endif
				}
				if(displayMode==	ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE)
				{
					SendBackXY(p_ForeSightFacade[MAIN]->GetMil());
			//		Milx=p_ForeSightFacade->GetMil()[0];
			//		printf("PANO_MilX=%d\n",Milx);
				}
				 else if(displayMode==TELESCOPE_FRONT_MODE)
				 {
					 SendBackXY(p_ForeSightFacade2[MAIN]->GetTelMil(0));
			//			Milx=p_ForeSightFacade2->GetTelMil(0)[0];
			//			printf("TEL_MilX=%d\n",Milx);
				 }
				 else if(displayMode==TELESCOPE_RIGHT_MODE)
				 {
					 SendBackXY(p_ForeSightFacade2[MAIN]->GetTelMil(1));
				//		Milx=p_ForeSightFacade2->GetTelMil(1)[0];
			//			printf("TEL_MilX=%d\n",Milx);
				 }
				 else if(displayMode==TELESCOPE_BACK_MODE)
				 {
					 SendBackXY(p_ForeSightFacade2[MAIN]->GetTelMil(2));
				//		Milx=p_ForeSightFacade2->GetTelMil(2)[0];
			//			printf("TEL_MilX=%d\n",Milx);
				 }
				 else if(displayMode==TELESCOPE_LEFT_MODE)
				 {
					 SendBackXY(p_ForeSightFacade2[MAIN]->GetTelMil(3));
			//			Milx=p_ForeSightFacade2->GetTelMil(3)[0];
			//			printf("TEL_MilX=%d\n",Milx);
				 }
#if 0
						else if(displayMode==	VGA_WHITE_VIEW_MODE
									||displayMode==VGA_HOT_BIG_VIEW_MODE
									||displayMode==VGA_HOT_SMALL_VIEW_MODE
									||displayMode==VGA_FUSE_WOOD_LAND_VIEW_MODE
									||displayMode==VGA_FUSE_GRASS_LAND_VIEW_MODE
									||displayMode==VGA_FUSE_SNOW_FIELD_VIEW_MODE
									||displayMode==VGA_FUSE_DESERT_VIEW_MODE
									||displayMode==VGA_FUSE_CITY_VIEW_MODE)
							{
							foresightPos.ChangeEnlarge2Ori(g_windowWidth*1024.0/1920.0, g_windowHeight*768.0/1080.0,
									g_windowWidth*1434/1920, g_windowHeight,
									foresightPos.Change2TrackPosX(PanoLen/37.685200*15.505) ,
									foresightPos.Change2TrackPosY(PanoHeight/6.0000*11.600),
									TRACKFRAMEX, TRACKFRAMEY,
									PanoLen/37.685200*15.505,PanoHeight/6.0000*11.600);

							}
							else if(displayMode==SDI1_WHITE_BIG_VIEW_MODE
									||displayMode==SDI1_WHITE_SMALL_VIEW_MODE
									||displayMode==SDI2_HOT_BIG_VIEW_MODE
									||displayMode==SDI2_HOT_SMALL_VIEW_MODE)
							{
								foresightPos.ChangeEnlarge2Ori(g_windowWidth*1920.0/1920.0, g_windowHeight*1080.0/1080.0,
										g_windowWidth*1434/1920, g_windowHeight,
										foresightPos.Change2TrackPosX(PanoLen/37.685200*15.505) ,
										foresightPos.Change2TrackPosY(PanoHeight/6.0000*11.600),
										TRACKFRAMEX, TRACKFRAMEY,
										PanoLen/37.685200*15.505,PanoHeight/6.0000*11.600);
							}

							else if(displayMode==PAL1_WHITE_BIG_VIEW_MODE
									||displayMode==PAL1_WHITE_SMALL_VIEW_MODE
									||displayMode==PAL2_HOT_BIG_VIEW_MODE
									||displayMode==PAL2_HOT_SMALL_VIEW_MODE)
							{
								foresightPos.ChangeEnlarge2Ori(g_windowWidth*720.0/1920.0, g_windowHeight*576.0/1080.0,
										g_windowWidth*1346/1920, g_windowHeight,
										foresightPos.Change2TrackPosX(PanoLen/37.685200*14.524) ,
										foresightPos.Change2TrackPosY(PanoHeight/6.0000*11.600),
										TRACKFRAMEX, TRACKFRAMEY,
										PanoLen/37.685200*14.524,PanoHeight/6.0000*11.600);
							}
#endif
			}
				break;
			case '%':

			break;
			case '^':
			{
							pano_pos2angle=p_ForeSightFacade[MAIN]->GetForeSightPosX()/PanoLen*360.0;
			//				printf("POS_angle=%f\n",pano_pos2angle);
							math_ruler_angle=p_LineofRuler->GetAngle();//getrulerangle();
							math_ruler_angle=math_ruler_angle+pano_pos2angle;
							if(math_ruler_angle>0.0 &&math_ruler_angle<360.0)
							{
								p_LineofRuler->SetAngle(math_ruler_angle);//setrulerangle(math_ruler_angle);
							}
							else if(math_ruler_angle<0.0)
							{
								p_LineofRuler->SetAngle(math_ruler_angle+360.0);//setrulerangle(math_ruler_angle+360.0);
							}
							else if(math_ruler_angle>360.0)
							{
								p_LineofRuler->SetAngle(math_ruler_angle-360.0);//setrulerangle(math_ruler_angle-360.0);
							}
							ori_ruler_angle=p_LineofRuler->Load();
							mydelta=(p_LineofRuler->GetAngle())/360.0*(render.get_PanelLoader().Getextent_pos_x()-render.get_PanelLoader().Getextent_neg_x());
							now_ruler_angle=p_LineofRuler->GetAngle();
							p_LineofRuler->Save(now_ruler_angle);
							setenablerefrushruler(true);
							break;
			}
			case '(':
				foresightPos[MAIN].SetSpeedX((render.get_PanelLoader().Getextent_pos_x()-render.get_PanelLoader().Getextent_neg_x())/1920.0*10.0);
					break;
			case ')':
				foresightPos[MAIN].SetSpeedY((render.get_PanelLoader().Getextent_pos_z()-render.get_PanelLoader().Getextent_neg_z())/1920.0*20.0);
				break;
			case '?':
			{
				enable_hance=!enable_hance;
			//	int Vanw,Vanh;
			//	Vanw=glutGet(GLUT_WINDOW_WIDTH);
			//	Vanh=glutGet(GLUT_WINDOW_HEIGHT);
			//	cout<<"W===="<<Vanw<<"H===="<<Vanh<<endl;
			}
				break;

			case '{':
			{
				int set_track_params[4];
				#if TRACK_MODE
				int trkId=getTrkId(displayMode,displayMode);
				memcpy(set_track_params,gettrackcontrolparams(),sizeof(set_track_params));
				if(trkId!=-1)
				{
				if(trkId==sdi_Track){
					clearTrackParams();
				trackMode->enableTrackAcq(set_track_params[0],set_track_params[1],
						set_track_params[2],set_track_params[3],trkId);
					}else if(trkId==vga_Track){
					clearTrackParams();
					trackMode->enableTrackAcq(482,354,
						60,60,trkId);
						}else if(trkId==pal_Track){
						clearTrackParams();
						trackMode->enableTrackAcq(330,258,
						60,60,trkId);
							}else if(trkId==pal2_Track){
						clearTrackParams();
						trackMode->enableTrackAcq(330,258,
						60,60,trkId);
								}
				isTracking=true;
				}
				#endif
			}
			break;
			case '}':
			{
				#if TRACK_MODE
					trackMode->disableTrack(sdi_Track);
					trackMode->disableTrack(pal_Track);
					trackMode->disableTrack(vga_Track);
					trackMode->disableTrack(pal2_Track);
					isTracking=false;
				#endif
			}
			break;
			case '`':
			{
				bool button_f1=GetPSYButtonF1();
				SetPSYButtonF1(!button_f1);
			}
			break;
			case ':':
			{
				bool button_f2=GetPSYButtonF2();
				SetPSYButtonF2(!button_f2);
			}
			break;
			case '|':
			{
				bool button_f3=GetPSYButtonF3();
				SetPSYButtonF3(!button_f3);
			}
			break;
			case '*':
			{
				bool button_f8=GetPSYButtonF8();
				SetPSYButtonF8(!button_f8);
			}
			break;



		default:
			break;
		}
	//	pVehicle->SetBlendMode(mode, blendMode);
	}while(0);

	if( (SINGLE_PORT_MODE != displayMode)||(BillBoard::BBD_REAR != p_BillBoard->m_Direction && BillBoard::BBD_FRONT != p_BillBoard->m_Direction ) )
	{
	  //      p_CornerMarkerGroup->StopAdjust();
	}

}


/* The function called whenever a key is pressed. */
void Render::keyPressed(GLEnv &m_env,unsigned char key, int x, int y)
{
	/* Keyboard debounce */
	/* I don't know which lib has this in win32 */
	usleep(100);

	/* Pressing escape kills everything --Have a nice day! */
	if (key == ESCAPE)
	{
		/* shut down our window */
		glutDestroyWindow(window);

		/* exit the program...normal termination. */
		exit(0);
	}
	ProcessOitKeys(m_env,key, x, y);
}

/* This function is for the special keys.  */
/* The dynamic viewing keys need to be time based */
void Render::specialkeyPressed (GLEnv &m_env,int key, int x, int y)
{
	/* keep track of time between calls, if it exceeds a certian value, then */
	/* assume the user has released the key and wants to start fresh */
	static int first = GL_YES;
	static clock_t last=0;
	clock_t now;
	float delta;
	float delta_x, delta_y;
	float Z_Depth = BowlLoader.GetZ_Depth();
	float scan_angle=10.0;
	float now_ruler_angle=0.0;
	float ori_ruler_angle=0.0;
	/* Properly initialize the MOUSE vars on first time through this function */
	if (first)
	{
		first = GL_NO;
		setMouseCor(x,y);
	}

	/* If the clock exceeds a reasonable value, assume user has released F key */
	now  = clock();
	delta= (now - last) / (float) CLOCKS_PER_SEC;
	last = now;
	if (delta > 0.1)
	{
		setMouseCor(x,y);
	}

	switch(key)
	{
	case 1:
           	if((TRIM_MODE==displayMode)&&(EnablePanoFloat==true))
           	{
           		WritePanoScaleArrayData(PANO_SCALE_ARRAY_FILE,channel_left_scale,channel_right_scale,move_hor);
           		WritePanoFloatDataFromFile(PANO_FLOAT_DATA_FILENAME,PanoFloatData);
           		WriteRotateAngleDataToFile(PANO_ROTATE_ANGLE_FILENAME,rotate_angle);
           	}
		else if(FREE_VIEW_MODE == displayMode){//save current position to YML file
			mPresetCamGroup.SetCameraFrame(p_BillBoard->m_Direction,  m_freeCamera.GetFrame());
			mPresetCamGroup.SaveCameraTo(p_BillBoard->m_Direction );
		}
		else if((displayMode==PANO_VIEW_MODE)

						)

		{

			writeScanAngle("./scanangle.yml",getScanRegionAngle(),getrulerangle());
			if(getenableshowruler())
			{
				ori_ruler_angle=p_LineofRuler->Load();
				now_ruler_angle=p_LineofRuler->GetAngle();
				p_LineofRuler->Save(now_ruler_angle);
				setenablerefrushruler(true);
			}
		}
		else if((FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE==displayMode)||
				(PANO_VIEW_MODE==displayMode)||(FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE)||
				(FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE==displayMode)
				||displayMode==	ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE
				)
		{
			if(getenableshowruler())
			{
				ori_ruler_angle=p_LineofRuler->Load();
				now_ruler_angle=p_LineofRuler->GetAngle();
				cout<<"now_ruler_angle="<<now_ruler_angle<<endl;
				p_LineofRuler->Save(now_ruler_angle);
				setenablerefrushruler(true);

			}
			if((FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE==displayMode)||(EnterSinglePictureSaveMode==true))
					{
           					WritePanoScaleArrayData(PANO_SCALE_ARRAY_FILE,channel_left_scale,channel_right_scale,move_hor);
           					WritePanoFloatDataFromFile(PANO_FLOAT_DATA_FILENAME,PanoFloatData);
					}

		}
		else if(SINGLE_PORT_MODE == displayMode ){
			if( BillBoard::BBD_FRONT== p_BillBoard->m_Direction){
				if(p_CornerMarkerGroup->isAdjusting()){
					p_CornerMarkerGroup->StopAdjust();
				}
				else{
				    if(x < g_windowWidth/2) {p_CornerMarkerGroup->Adjust(CORNER_FRONT_LEFT);}
				    else{p_CornerMarkerGroup->Adjust(CORNER_FRONT_RIGHT);}
				}
			}
			else if( BillBoard::BBD_REAR== p_BillBoard->m_Direction){
				if(p_CornerMarkerGroup->isAdjusting()){
					p_CornerMarkerGroup->StopAdjust();
				}
				else{
				    if(x < g_windowWidth/2) {p_CornerMarkerGroup->Adjust(CORNER_REAR_LEFT);}
				    else{p_CornerMarkerGroup->Adjust(CORNER_REAR_RIGHT);}
				}
			}
		}
		break;
	case 2:
			if(FREE_VIEW_MODE == displayMode){//reset to default position
				GLFrame	frame = mPresetCamGroup.GetCameraFrame(p_BillBoard->m_Direction );
				m_freeCamera.SetFrame(frame);
			}
			else if(SINGLE_PORT_MODE == displayMode && ( BillBoard::BBD_FRONT== p_BillBoard->m_Direction || BillBoard::BBD_REAR== p_BillBoard->m_Direction )){
			 p_CornerMarkerGroup->SaveCorners();
		}
		break;
	case 3:
	{
		static bool b_set_arcWidth = true;
		if(b_set_arcWidth)
		{
			float set_arcWidth = DEFAULT_ARC_WIDTH_SET;
			p_DynamicTrack->SetArcWidth(set_arcWidth);
			b_set_arcWidth = false;
		}
		float &angle = m_DynamicWheelAngle;
		static float sign = -1.0f;
		printf("Dynamic Track angle = %f\n", angle);
		angle = angle +  sign * 1.57f;
		if(angle < 0.0f){
			angle = 180.0f + angle;
		}
		if(angle > 360.0f){
			angle -= 360.0f;
		}
		if((angle>73 && angle <107)||(angle>253 && angle <287)){
			sign = -1.0*sign;// change direction
			angle = angle + 2* sign * 1.57f;
		}
	}
		break;
	case 4:
	{
		static bool bRevMode = true;
		p_DynamicTrack->SetReverseMode(bRevMode);
		p_DynamicTrack->RefreshAngle();
		bRevMode = !bRevMode;
	}

		           	if((TRIM_MODE==displayMode)&&(EnablePanoFloat==true))
		           	{
		           			channel_left_scale[testPanoNumber]=1.0;
		           			channel_right_scale[testPanoNumber]=1.0;
		           			move_hor[testPanoNumber]=0.0;
		           			PanoFloatData[testPanoNumber]=0.0;
		           			rotate_angle[testPanoNumber]=0.0;
		           			InitPanel(m_env,0,true);

		           		
		           	}
	break;
	case 5:
#if USE_GAIN
               {
                       if( overLapRegion::GetoverLapRegion()->get_change_gain()==false)
                               overLapRegion::GetoverLapRegion()->set_change_gain(true);
                       else
                               overLapRegion::GetoverLapRegion()->set_change_gain(false);
               }
#endif
       break;

	case 6:
		           	if((TRIM_MODE==displayMode)&&(EnablePanoFloat==true))
		           	{
					for(int set_i=0;set_i<CAM_COUNT;set_i++)
					{
		           			channel_left_scale[set_i]=1.0;
		           			channel_right_scale[set_i]=1.0;
		           			move_hor[set_i]=0.0;
		           			PanoFloatData[set_i]=0.0;
		           			rotate_angle[set_i]=0.0;
					}
					InitPanel(m_env,0,true);
		           	}

		break;
	case 7:
	case 8:
		break;
	case 9:
		scan_angle=getScanRegionAngle();
		scan_angle=scan_angle+0.2;
		if(scan_angle>180.0)
		{
			scan_angle=180.0;
		}
		setScanRegionAngle(scan_angle);
		break;
	case 10:
		scan_angle=getScanRegionAngle();
		scan_angle=scan_angle-0.2;
		if(scan_angle<10.0)
		{
			scan_angle=10.0;
		}
		setScanRegionAngle(scan_angle);
		break;
	case 11:
		if((TRIM_MODE==displayMode))
		{
			EnablePanoFloat=false;
			shaderManager.ResetTrimColor();
		}
				if(INIT_VIEW_MODE==displayMode)
				{
					EnterSinglePictureSaveMode=false;
					for(int cam_num=0;cam_num<MAX_PANO_CAMERA_COUNT;cam_num++)
					{
						picSaveState[cam_num]=false;
					}
				}
				if(FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE==displayMode)
				{
					EnablePanoFloat=false;
#if USE_GAIN
					overLapRegion::GetoverLapRegion()->SetSingleHightLightState(false);
#endif
				}
				break;
	case 12:
		displayMode=TRIM_MODE;
		if((TRIM_MODE==displayMode))
		{
			EnablePanoFloat=true;
		}

		if(INIT_VIEW_MODE==displayMode)
		{
			EnterSinglePictureSaveMode=true;
		}

		if(FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE==displayMode)
		{
			EnablePanoFloat=true;
#if USE_GAIN
			overLapRegion::GetoverLapRegion()->SetSingleHightLightState(true);
#endif
		}
		break;
	case SPECIAL_KEY_UP:
	case SPECIAL_KEY_DOWN:
	case SPECIAL_KEY_LEFT:
	case SPECIAL_KEY_RIGHT:
		if(isCalibTimeOn){
				m_Timebar.CalibTime(key);
			}
			else if(EnablePanoFloat==true&&TRIM_MODE==displayMode)
			{
				if(SPECIAL_KEY_RIGHT == key)
				{
					shaderManager.ResetTrimColor();
					testPanoNumber=(testPanoNumber+CAM_COUNT-1)%CAM_COUNT;
					shaderManager.SetTrimColor(testPanoNumber);
				}
				else if(key==SPECIAL_KEY_LEFT)
				{
					shaderManager.ResetTrimColor();
					testPanoNumber=(testPanoNumber+1)%CAM_COUNT;
					shaderManager.SetTrimColor(testPanoNumber);
				}
				else if(SPECIAL_KEY_UP == key)
				{
					PanoFloatData[testPanoNumber]=PanoFloatData[testPanoNumber]+DELTA_OF_PANOFLOAT;
					InitPanel(m_env,0,true);
				}
				else if(SPECIAL_KEY_DOWN == key)
				{
					PanoFloatData[testPanoNumber]=PanoFloatData[testPanoNumber]-DELTA_OF_PANOFLOAT;
					InitPanel(m_env,0,true);
				}
			}
			else if(isDirectionOn && isDirectionMode()&&(key==SPECIAL_KEY_RIGHT)){
				int nextdir=(p_BillBoard->m_Direction+1)%BillBoard::BBD_COUNT;
				ProcessOitKeys(m_env,'U'+nextdir,x,y);
				if(TRIPLE_VIEW_MODE == displayMode){
					m_presetCameraRotateCounter = PRESET_CAMERA_ROTATE_MAX;
				}
			}
			else if(SPECIAL_KEY_LEFT == key){
				ProcessOitKeys(m_env,'L', x,y);
			}
			else if(SPECIAL_KEY_UP == key){
				ProcessOitKeys(m_env,'P', x,y);
			}
			else if(SPECIAL_KEY_DOWN == key){
				ProcessOitKeys(m_env,'M', x,y);
			}
			else{
				//do nothing
			}
			break;
	default:
		break;
	}

	if (common.isVerbose())
		printf("Special Key--> %i at %i, %i screen location\n", key, x, y);
}

void Render::BowlParseSTLAscii(const char* filename)
{
	BowlLoader.ParseSTLAscii(filename);
}

void Render::PanelParseSTLAscii(const char* filename)
{
	PanelLoader.ParseSTLAscii(filename);
	PanelLoader.cpyl1l2();
}

void Render::VehicleParseObj(const char* filename)
{
	VehicleLoader = glmReadOBJ(filename);
}

int Render::VehicleGetMemSize()
{
	return VehicleLoader->GetMemSize();
}

int Render::VehicleGetpoly_count()
{
	return VehicleLoader->Getpoly_count();
}

void Render::RememberTime()
{
	m_Timebar.SetFTime(m_Timebar.GetFTime());
	m_Timebar.ResetDeltas();
	bRotTimerStart = false;
}
void Render::RenderOriginCords(GLEnv &m_env,GLint x, GLint y,GLint w, GLint h)
{
    glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->LoadIdentity();
	// move h since the shadow dimension is [-1,1], use h/2 if it is [0,1]
	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h/2);
	m_env.GetmodelViewMatrix()->Scale(w, h, 1.0f);
	{
		const char* pCords = m_freeCamera.GetOriginCords();
		DrawCords(m_env,w,h, pCords);
		
	}
	m_env.GetmodelViewMatrix()->PopMatrix();
}
void Render::RenderUpCords(GLEnv &m_env,GLint x, GLint y,GLint w, GLint h)
{
    glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->LoadIdentity();
	// move h since the shadow dimension is [-1,1], use h/2 if it is [0,1]
	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h/2);
	m_env.GetmodelViewMatrix()->Scale(w, h, 1.0f);
	{
		const char* pCords = m_freeCamera.GetUpCords();
		DrawCords(m_env,w,h, pCords);
	}
	m_env.GetmodelViewMatrix()->PopMatrix();
}
void Render::RenderFwdCords(GLEnv &m_env,GLint x, GLint y,GLint w, GLint h)
{
    glViewport(x,y,w,h);
	m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->LoadIdentity();
	// move h since the shadow dimension is [-1,1], use h/2 if it is [0,1]
	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h/2);
	m_env.GetmodelViewMatrix()->Scale(w, h, 1.0f);
	{
		const char* pCords = m_freeCamera.GetFwdCords();
		DrawCords(m_env,w,h, pCords);
	}
	m_env.GetmodelViewMatrix()->PopMatrix();
}
void Render::RenderCordsView(GLEnv &m_env,GLint x, GLint y,GLint w, GLint h)
{
	RenderOriginCords(m_env,x, y, w, h);
    RenderUpCords(m_env,x, y+h/3, w, h);
	RenderFwdCords(m_env,x,y+h*2/3,w, h);
}

void Render::DrawCords(GLEnv &m_env,int w, int h, const char* s)
{
    char quote[1][160];
    bzero(quote[0],160);
    int i,l,lenghOfQuote;
     int numberOfQuotes=1;
    const GLfloat *PDefaultColor = vWhite;
    const GLfloat *pColor = PDefaultColor;
    strcpy(quote[0],"FreeView:");
    int min=MIN(160,strlen(s));
    if(s)
    	strncpy(quote[0],s,min);

    GLfloat UpwardsScrollVelocity = -10.0;

    m_env.GetmodelViewMatrix()->LoadIdentity();
    m_env.GetmodelViewMatrix()->Translate(0.0, 0.0, UpwardsScrollVelocity);
    m_env.GetmodelViewMatrix()->Scale(0.03, 0.03, 0.03);
    glLineWidth(2);
    for(  l=0;l<numberOfQuotes;l++)
    {
        lenghOfQuote = (int)strlen(quote[l]);
        m_env.GetmodelViewMatrix()->PushMatrix();
        m_env.GetmodelViewMatrix()->Translate(-(lenghOfQuote-5)*90.0f, -(l*200.0f), 0.0);
        for (i = 0; i < lenghOfQuote; i++)
        {
        	m_env.GetmodelViewMatrix()->Translate((90.0f), 0.0, 0.0);
        	pColor = PDefaultColor;
            shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), pColor);
            glutStrokeCharacter(GLUT_STROKE_ROMAN, quote[l][i]);
        }
        m_env.GetmodelViewMatrix()->PopMatrix();
    }
}


void Render::DrawAngleCords(GLEnv &m_env,int w, int h, const char* s,float toScale)
{
    char quote[1][160];
    bzero(quote[0],160);
    int i,l,lenghOfQuote;
     int numberOfQuotes=1;
    const GLfloat *PDefaultColor = vWhite;
    const GLfloat *pColor = PDefaultColor;
    strcpy(quote[0],"         ");
    int min=MIN(160,strlen(s));
    if(s)
    	strncpy(quote[0],s,min);

    GLfloat UpwardsScrollVelocity = -10.0;

    m_env.GetmodelViewMatrix()->LoadIdentity();
    m_env.GetmodelViewMatrix()->Translate(0.0, 0.0, UpwardsScrollVelocity);
    m_env.GetmodelViewMatrix()->Scale(0.03*toScale, 0.03*toScale, 0.03*toScale);
    glLineWidth(2);
    for(  l=0;l<numberOfQuotes;l++)
    {
        lenghOfQuote = (int)strlen(quote[l]);
        m_env.GetmodelViewMatrix()->PushMatrix();
        m_env.GetmodelViewMatrix()->Translate(-(lenghOfQuote-5)*90.0f, -(l*200.0f), 0.0f);
        for (i = 0; i < lenghOfQuote; i++)
        {
        	m_env.GetmodelViewMatrix()->Translate((90.0f), 0.0, 0.0);
        	pColor = PDefaultColor;
            shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), pColor);
            glutStrokeCharacter(GLUT_STROKE_ROMAN, quote[l][i]);
        }
        m_env.GetmodelViewMatrix()->PopMatrix();
    }


}

void Render::SwitchBlendMode(int blendmode)
{
	switch (blendmode)
		{
		case 1:
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			break;
		case 2:
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_DST_ALPHA);
			break;
		case 3:
			glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
			break;
		case 4:
			glBlendFunc(GL_SRC_ALPHA, GL_ONE);
			break;
		case 5:
			glBlendFunc(GL_SRC_ALPHA, GL_DST_COLOR);
			break;
		case 6:
			glBlendFuncSeparate(GL_SRC_ALPHA, GL_DST_ALPHA, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			break;
		case 7:
			glBlendFuncSeparate(GL_SRC_COLOR, GL_DST_COLOR, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			break;
		default:
			glDisable(GL_BLEND);
		}
}
//Embedded time bar class implementation=======================================

void Render::TimeBar::CalibTime( int key)
{

      unsigned int length = strlen(buf);
      unsigned int counter = 0; //avoid deadloop in a non-digit string
      int value = 0;
	switch (key){
	case SPECIAL_KEY_UP:
		if(isdigit(buf[indicator]))
		{
		    value = 1;
		}
		else{
		    cout<<"unknown character "<<buf[indicator]<<endl;
		}
		break;
	case SPECIAL_KEY_DOWN:
		if(isdigit(buf[indicator]))
		{
		     value = -1;
		}
		else{
		    cout<<"unknown character "<<buf[indicator]<<endl;
		}
		break;
	case SPECIAL_KEY_LEFT:
		indicator = (indicator+length-1)%length;
		while(!isdigit(buf[indicator]) && counter++ < length*2)
		{
		    indicator = (indicator+length -1)%length;
		}
		break;
	case SPECIAL_KEY_RIGHT:
		indicator = (indicator+1)%length;
		while(!isdigit(buf[indicator]) && counter++ < length*2)
		{
		    indicator = (indicator+1)%length;
		}
		break;
	default:
		break;
	}
	// buf:
	//        1998-01-01 11:11:22
	// calculate the deltas
	if(value != 0){
		if(indicator == 0){	delta_year += value*1000;}
		else if(indicator == 1){delta_year += value*100;}
		else if(indicator == 2){delta_year += value*10;}
		else if(indicator == 3){delta_year += value;}
		else if(indicator == 5){delta_month += value*10;}
		else if(indicator == 6){delta_month += value;}
		else if(indicator == 8){delta_day += value*10;}
		else if(indicator == 9){delta_day += value;}
		else if(indicator == 11){delta_hour += value*10;}
		else if(indicator == 12){delta_hour += value;}
		else if(indicator == 14){delta_minute += value*10;}
		else if(indicator == 15){delta_minute += value;}
		else if(indicator == 17){delta_second += value*10;}
		else if(indicator == 18){delta_second += value;}
		else{
			//do nothing
		}
	}
}
const char* Render::TimeBar::GetFTime()
{
	time_t now;
	struct tm *timenow;
	struct tm destTime;
	time(&now);
	bool leapYear = false;
	int monthDays=31;

	timenow = localtime(&now);
	destTime = *timenow;
	//validate year
	do{
		if (destTime.tm_year+delta_year +1900<1000)
		{
			delta_year++;
		}
		else if(destTime.tm_year+delta_year +1900>=9999)
		{
			if(indicator == 0)
				delta_year -= 1000;
			else if(indicator == 1)
				delta_year -= 100;
			else if(indicator == 2)
				delta_year -= 10;
			else if(indicator == 3)
			    delta_year --;
		}
		else
		{
			destTime.tm_year = destTime.tm_year+delta_year;

			int year = destTime.tm_year+1900;
			if( ((year%4 == 0) && (year%100 != 0)) ||(year%400 == 0) )
			{
				leapYear = true;
			}
			
			break;
		}
	}while(1);
	//validate month
	do{
		if (destTime.tm_mon+delta_month <0)
		{
			if(destTime.tm_mon+delta_month < -1){
				delta_month += 10;
			}
			else{
				delta_month ++;
			}						
		}
		else if(destTime.tm_mon+delta_month>=12)
		{
			if(indicator == 5)
				delta_month -= 10;
			else if(indicator == 6)
			    delta_month --;
		}
		else
		{
			destTime.tm_mon = destTime.tm_mon+delta_month;

			if (destTime.tm_mon == 3 || destTime.tm_mon == 5 || 
				destTime.tm_mon == 8 || destTime.tm_mon == 10){
				monthDays = 30;
			}else if(destTime.tm_mon == 1){
				if(leapYear){
					monthDays = 29;
				}else{
					monthDays = 28;
				}
			}else{
				monthDays = 31;	
			}
			
			break;
		}
	}while(1);
	//validate day
	do{
		if (destTime.tm_mday+delta_day <= 0)
		{
			if(destTime.tm_mday+delta_day < -2){
				delta_day += 10;
			}
			else{
				delta_day ++;
			}	
		}
		else if(destTime.tm_mday+delta_day>monthDays)
		{
			if(indicator == 8)
				delta_day -= 10;
			else if(indicator == 9)
				delta_day --;
			else
				delta_day --;
		}
		else
		{
			destTime.tm_mday = destTime.tm_mday+delta_day;
			break;
		}
	}while(1);
	//validate hour
	do{
		if (destTime.tm_hour+delta_hour <0)
		{
			delta_hour ++;
		}
		else if(destTime.tm_hour+delta_hour>23)
		{
			if(indicator == 11)
				delta_hour -= 10;
			else if(indicator == 12)
				delta_hour --;
		}
		else{
			destTime.tm_hour = destTime.tm_hour+delta_hour;
			break;
		}
	}while(1);
	//validate minute
	do{
		if (destTime.tm_min+delta_minute <0)
		{
			delta_minute ++;
		}
		else if(destTime.tm_min+delta_minute>59){
			if(indicator == 14)
				delta_minute -= 10;
			else if(indicator == 15)
				delta_minute --;
		}
		else{
			destTime.tm_min = destTime.tm_min+delta_minute;
			break;
		}

	}while(1);
	//validate second
	do{
		if (destTime.tm_sec+delta_second <0)
		{
			delta_second ++;
		}
		else if(destTime.tm_sec+delta_second>59)
		{
			if(indicator == 17)
				delta_second -= 10;
			else if(indicator == 18)
				delta_second --;
		}
		else
		{
			destTime.tm_sec = destTime.tm_sec+delta_second;
			break;
		}

	}while(1);
	strftime(buf, 64, "%Y-%m-%d %H:%M:%S", &destTime);
	return buf;
}
void Render::TimeBar::SetFTime(const char* time_str)
{
     SetSysDateAndTime(time_str);
	 SetHWClockFromSysClock(0);
}

int  Render::TimeBar::SetSysDateAndTime(const char *time_str)
{
	struct tm       time_tm;
	struct timeval  time_tv;       
	time_t          timep;
	int             ret;

	if(time_str == NULL)            
	{                   
	    fprintf(stderr, "time string args invalid!\n");                    
	    return -1;           
	}        

	sscanf(time_str, "%d-%d-%d %d:%d:%d", 
		&time_tm.tm_year, &time_tm.tm_mon, &time_tm.tm_mday, &time_tm.tm_hour, &time_tm.tm_min, &time_tm.tm_sec);       
	time_tm.tm_year     -= 1900;        
	time_tm.tm_mon      -= 1;        
	time_tm.tm_wday     = 0;        
	time_tm.tm_yday     = 0;        
	time_tm.tm_isdst    = 0;        
	timep = mktime(&time_tm);        
	time_tv.tv_sec  = timep;        
	time_tv.tv_usec = 0;        
	
	ret = settimeofday(&time_tv, NULL);        
	if(ret != 0)        
	{            
	    fprintf(stderr, "settimeofday failed. Maybe try run in root or sudo mode?\n");
		return -2;        
	}        

	return 0;
}

void Render::TimeBar::SetHWClockFromSysClock(int utc)
{
	system("/sbin/hwclock -w ");
}


//==============end of embedded timebar implementation================
//===============embedded BaseBillBoard implemntation===================

Render::BaseBillBoard::BaseBillBoard(GLMatrixStack &modelViewMat,GLMatrixStack	&projectionMat,GLShaderManager* mgr):
	m_pShaderManager(mgr),modelViewMatrix(modelViewMat),projectionMatrix(projectionMat),	blendmode(6)
{
	if(NULL == m_pShaderManager){
		m_pShaderManager = (GLShaderManager*)getDefaultShaderMgr();
	}
}
Render::BaseBillBoard::~BaseBillBoard()
{

}
bool Render::BaseBillBoard::LoadTGATextureRect(const char *szFileName, GLenum minFilter, GLenum magFilter, GLenum wrapMode)
{
	GLbyte *pBits;
	int nWidth, nHeight, nComponents;
	GLenum eFormat;
	
	// Read the texture bits
	pBits = gltReadTGABits(szFileName, &nWidth, &nHeight, &nComponents, &eFormat);
	if(pBits == NULL) 
		return false;
	
	glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, wrapMode);
	glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, wrapMode);
	
	glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, minFilter);
	glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, magFilter);
		
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexImage2D(GL_TEXTURE_RECTANGLE, 0, nComponents, nWidth, nHeight, 0,
				 eFormat, GL_UNSIGNED_BYTE, pBits);
	
    free(pBits);
	//cout<<"Load TGA Texture "<<szFileName<<" ok"<<endl;
	return true;
}

void Render::BaseBillBoard::Init(int x,int y,int width,int height)
{

	HZbatch.Begin(GL_TRIANGLE_FAN, 4, 1);

	// Upper left hand corner
	HZbatch.MultiTexCoord2f(0, 0.0f, height);
	HZbatch.Vertex3f(x, y, 0.0);

	// Lower left hand corner
	HZbatch.MultiTexCoord2f(0, 0.0f, 0.0f);
	HZbatch.Vertex3f(x, y - height, 0.0f);

	// Lower right hand corner
	HZbatch.MultiTexCoord2f(0, width, 0.0f);
	HZbatch.Vertex3f(x + width, y - height, 0.0f);

	// Upper righ hand corner
	HZbatch.MultiTexCoord2f(0, width, height);
	HZbatch.Vertex3f(x + width, y, 0.0f);

	HZbatch.End();
	InitTextures();
	
}
void Render::BaseBillBoard::DrawBillBoard(int w, int h)
{
	M3DMatrix44f mScreenSpace;
	m3dMakeOrthographicMatrix(mScreenSpace, 0.0f,800.0f, 0.0f, 600.0f, -1.0f, 1.0f);
	// Turn blending on, and depth testing off
	glEnable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	glActiveTexture(GL_TEXTURE0);
	DoTextureBinding();

	Render::SwitchBlendMode(blendmode);
	m_pShaderManager->UseStockShader(GLT_SHADER_TEXTURE_RECT_REPLACE,mScreenSpace,0);
	HZbatch.Draw();
	    // Restore no blending and depth test
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
}


//----------------------------inherited compassbillboard class implementation----------------------
Render::CompassBillBoard::CompassBillBoard(GLMatrixStack &modelViewMat,GLMatrixStack	&projectionMat,GLShaderManager* mgr, COMPASS_BBD_DIRECTION direction)
	:BaseBillBoard((modelViewMat),(projectionMat),(mgr)),m_CompassDirection(direction)
{
	strcpy(m_CompassBBDTextureFileName[COMPASS__PIC], DEFAULT_COMPASS);

}
Render::CompassBillBoard::~CompassBillBoard()
{
	glDeleteTextures(COMPASS_COUNT, m_CompassBBDTextures);
}
void Render::CompassBillBoard::CompassLoadTGATextureRects()
{
	for(int i = 0; i < COMPASS_COUNT; i++){
           glBindTexture(GL_TEXTURE_RECTANGLE, m_CompassBBDTextures[i]);
	    LoadTGATextureRect(m_CompassBBDTextureFileName[i], GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE);
	}
}

void Render::CompassBillBoard::InitTextures(){
    glGenTextures(COMPASS_COUNT, m_CompassBBDTextures);
	CompassLoadTGATextureRects();
}
void Render::CompassBillBoard::processKeyDirection(int key)
{
/*	switch(key){
	case SPECIAL_KEY_UP:
		m_Direction = BBD_FRONT;
		break;
	case SPECIAL_KEY_DOWN:
		m_Direction = BBD_REAR;
		break;
	case SPECIAL_KEY_LEFT:
		m_Direction = (m_Direction + BBD_COUNT-1)%BBD_COUNT;//anti-clock_wise
		break;
	case SPECIAL_KEY_RIGHT:
		m_Direction = (m_Direction+1)%BBD_COUNT;//clock_wise
		break;
	default:
		break;
	}*/
}
void Render::CompassBillBoard::DoTextureBinding()
{
	unsigned int direction = m_CompassDirection;
	glBindTexture(GL_TEXTURE_RECTANGLE, m_CompassBBDTextures[direction]);
}

Render::ChineseCharacterBillBoard::ChineseCharacterBillBoard(GLMatrixStack &modelViewMat,GLMatrixStack	&projectionMat,GLShaderManager* mgr)
:BaseBillBoard((modelViewMat),(projectionMat),(mgr))
{
	strcpy( ChineseC_TextureFileName[MENU_T], MENU_TGA);
	strcpy( ChineseC_TextureFileName[TURRET_T], TURRET_TGA);
	strcpy( ChineseC_TextureFileName[PANORAMIC_MIRROR_T], PANORAMIC_MIRROR_TGA);
	strcpy( ChineseC_TextureFileName[DEBUG_T], DEBUG_TGA);
	strcpy( ChineseC_TextureFileName[ONEX_REALTIME_T], ONEX_REALTIME_TGA);
		strcpy( ChineseC_TextureFileName[TWOX_REALTIME_T], TWOX_REALTIME_TGA);
		strcpy( ChineseC_TextureFileName[FOURX_REALTIME_T], FOURX_REALTIME_TGA);
		strcpy( ChineseC_TextureFileName[ANGLE_T], ANGLE_TGA);
		strcpy( ChineseC_TextureFileName[LOCATION_T], LOCATION_TGA);
		strcpy( ChineseC_TextureFileName[RADAR_FRONT_T], RADAR_FRONT_TGA);
		strcpy( ChineseC_TextureFileName[RADAR_LEFT_T], RADAR_LEFT_TGA);
		strcpy( ChineseC_TextureFileName[RADAR_RIGHT_T], RADAR_RIGHT_TGA);
		strcpy( ChineseC_TextureFileName[RADAR_BACK_T], RADAR_BACK_TGA);

		strcpy( ChineseC_TextureFileName[SDI1_WHITE_BIG_T], SDI1_WHITE_BIG_TGA);
		strcpy( ChineseC_TextureFileName[SDI1_WHITE_SMALL_T], SDI1_WHITE_SMALL_TGA);
		strcpy( ChineseC_TextureFileName[SDI2_HOT_BIG_T], SDI2_HOT_BIG_TGA);
		strcpy( ChineseC_TextureFileName[SDI2_HOT_SMALL_T], SDI2_HOT_SMALL_TGA);


		strcpy( ChineseC_TextureFileName[PAL1_WHITE_BIG_T], PAL1_WHITE_BIG_TGA);
		strcpy( ChineseC_TextureFileName[PAL1_WHITE_SMALL_T], PAL1_WHITE_SMALL_TGA);
		strcpy( ChineseC_TextureFileName[PAL2_HOT_BIG_T], PAL2_HOT_BIG_TGA);
		strcpy( ChineseC_TextureFileName[PAL2_HOT_SMALL_T], PAL2_HOT_SMALL_TGA);

		strcpy( ChineseC_TextureFileName[VGA_WHITE_BIG_T], VGA_WHITE_BIG_TGA);
		strcpy( ChineseC_TextureFileName[VGA_WHITE_SMALL_T], VGA_WHITE_SMALL_TGA);
		strcpy( ChineseC_TextureFileName[VGA_HOT_BIG_T], VGA_HOT_BIG_TGA);
		strcpy( ChineseC_TextureFileName[VGA_HOT_SMALL_T], VGA_HOT_SMALL_TGA);

		strcpy( ChineseC_TextureFileName[VGA_FUSE_WOOD_T], VGA_FUSE_WOOD_TGA);
		strcpy( ChineseC_TextureFileName[VGA_FUSE_GRASS_T], VGA_FUSE_GRASS_TGA);
		strcpy( ChineseC_TextureFileName[VGA_FUSE_SNOW_T], VGA_FUSE_SNOW_TGA);
		strcpy( ChineseC_TextureFileName[VGA_FUSE_DESERT_T], VGA_FUSE_DESERT_TGA);
		strcpy( ChineseC_TextureFileName[VGA_FUSE_CITY_T], VGA_FUSE_CITY_TGA);

		strcpy( ChineseC_TextureFileName[FINE_T], FINE_TGA);
		strcpy( ChineseC_TextureFileName[WRONG_T], WRONG_TGA);
		strcpy( ChineseC_TextureFileName[IDLE_T], IDLE_TGA);

		strcpy( ChineseC_TextureFileName[F1_ON_T], FAR_START_UP_WINDOW);
		strcpy( ChineseC_TextureFileName[F1_OFF_T], FAR_STOP_UP_WINDOW);
		strcpy( ChineseC_TextureFileName[F2_ON_T], FAR_START_MOVE_DETECT);
		strcpy( ChineseC_TextureFileName[F2_OFF_T], FAR_STOP_MOVE_DETECT);
		strcpy( ChineseC_TextureFileName[F3_ON_T], FAR_START_ENHANCE);
		strcpy( ChineseC_TextureFileName[F3_OFF_T], FAR_STOP_ENHANCE);
		strcpy( ChineseC_TextureFileName[F4_T], FAR_UP);
		strcpy( ChineseC_TextureFileName[F5_T], FAR_DOWN);
		strcpy( ChineseC_TextureFileName[F6_T], FAR_LEFT);
		strcpy( ChineseC_TextureFileName[F7_T], FAR_RIGHT);
		strcpy( ChineseC_TextureFileName[F8_ON_T], FAR_START_TEST);
		strcpy( ChineseC_TextureFileName[F8_OFF_T], FAR_EXIT_TEST);
		strcpy( ChineseC_TextureFileName[F9_T], FAR_NEXT);
		strcpy( ChineseC_TextureFileName[STATE_LABEL_T], STATE_LABEL);
		strcpy( ChineseC_TextureFileName[STATE_LABEL2_T], STATE_LABEL2);

		strcpy( ChineseC_TextureFileName[POINT_RED_T], POINT_RED_F);
		strcpy( ChineseC_TextureFileName[POINT_GREEN_T], POINT_GREEN_F);
		strcpy( ChineseC_TextureFileName[POINT_GREY_T], POINT_GREY_F);

		strcpy( ChineseC_TextureFileName[CANON_HOR_T], CANON_HOR);
		strcpy( ChineseC_TextureFileName[CANON_VER_T], CANON_VER);
		strcpy( ChineseC_TextureFileName[GUN_HOR_T], GUN_HOR);
		strcpy( ChineseC_TextureFileName[GUN_VER_T], GUN_VER);

		strcpy( ChineseC_TextureFileName[CALC_HOR_T], CALC_HOR);
		strcpy( ChineseC_TextureFileName[CALC_VER_T], CALC_VER);

}

void Render::ChineseCharacterBillBoard::InitTextures()
{
    glGenTextures(CCT_COUNT, ChineseC_Textures);
    LoadChineseCTGATexture();
}
void Render::ChineseCharacterBillBoard::DoTextureBinding()
{
	glBindTexture(GL_TEXTURE_RECTANGLE, ChineseC_Textures[ChooseTga]);
}


void Render::ChineseCharacterBillBoard::LoadChineseCTGATexture()
{
	for(int i = 0; i < CCT_COUNT; i++){
		   glBindTexture(GL_TEXTURE_RECTANGLE, ChineseC_Textures[i]);
		LoadTGATextureRect(ChineseC_TextureFileName[i], GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE);
	}
}


//----------------------------inherited billboard class implementation----------------------
Render::BillBoard::BillBoard(GLMatrixStack &modelViewMat,GLMatrixStack	&projectionMat,GLShaderManager* mgr, BBD_DIRECTION direction)
	:BaseBillBoard((modelViewMat),(projectionMat),(mgr)),m_Direction(direction)
{
	strcpy(m_BBDTextureFileName[BBD_FRONT], DEFAULT_FRONT_TGA);
	strcpy(m_BBDTextureFileName[BBD_REAR], DEFAULT_REAR_TGA);
	strcpy(m_BBDTextureFileName[BBD_FRONT_LEFT], DEFAULT_FRONT_LEFT_TGA);
	strcpy(m_BBDTextureFileName[BBD_FRONT_RIGHT], DEFAULT_FRONT_RIGHT_TGA);
	strcpy(m_BBDTextureFileName[BBD_REAR_LEFT], DEFAULT_REAR_LEFT_TGA);
	strcpy(m_BBDTextureFileName[BBD_REAR_RIGHT], DEFAULT_REAR_RIGHT_TGA);
}
Render::BillBoard::~BillBoard()
{
	glDeleteTextures(BBD_COUNT, m_BBDTextures);
}
void Render::BillBoard::LoadTGATextureRects()
{
	for(int i = 0; i < BBD_COUNT; i++){
  //         glBindTexture(GL_TEXTURE_RECTANGLE, m_BBDTextures[i]);
	//    LoadTGATextureRect(m_BBDTextureFileName[i], GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE);
	}
}

void Render::BillBoard::InitTextures(){
    glGenTextures(BBD_COUNT, m_BBDTextures);
	LoadTGATextureRects();
}



void Render::BillBoard::processKeyDirection(int key)
{
	switch(key){
	case SPECIAL_KEY_UP:
		m_Direction = BBD_FRONT;
		break;
	case SPECIAL_KEY_DOWN:
		m_Direction = BBD_REAR;
		break;
	case SPECIAL_KEY_LEFT:
		m_Direction = (m_Direction + BBD_COUNT-1)%BBD_COUNT;//anti-clock_wise
		break;
	case SPECIAL_KEY_RIGHT:
		m_Direction = (m_Direction+1)%BBD_COUNT;//clock_wise
		break;
	default:
		break;
	}
}
void Render::BillBoard::DoTextureBinding()
{
	unsigned int direction = m_Direction;
	glBindTexture(GL_TEXTURE_RECTANGLE, m_BBDTextures[direction]);
}

//------------------------Ext BillBoard implementation-------------

Render::ExtBillBoard::ExtBillBoard(GLMatrixStack &modelViewMat,GLMatrixStack	&projectionMat,GLShaderManager* mgr)
:BaseBillBoard((modelViewMat),(projectionMat),(mgr))
{
	strcpy(m_BBDTextureFileName[0], DEFAULT_EXT_WG712_TGA);
}
Render::ExtBillBoard::~ExtBillBoard()
{
	glDeleteTextures(EXT_COUNT, m_BBDTextures);
}
void Render::ExtBillBoard::InitTextures()
{
    glGenTextures(EXT_COUNT, m_BBDTextures);
	LoadTGATextureRectExt();
}
void Render::ExtBillBoard::LoadTGATextureRectExt()
{
	for(int i = 0; i < EXT_COUNT; i++){
           glBindTexture(GL_TEXTURE_RECTANGLE, m_BBDTextures[i]);
	    LoadTGATextureRect(m_BBDTextureFileName[i], GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE);
	}
}
void Render::ExtBillBoard::DoTextureBinding()
{
	glBindTexture(GL_TEXTURE_RECTANGLE, m_BBDTextures[0]);
}
//================end of embedded BillBoard implementation=============

//================embedded FreeCamera implementation===============
const char* Render::FreeCamera::GetCords()
{
	stringstream stream;
	M3DVector3f tmp;
	
	stream<<"Orgn:(x=" << fixed << setprecision(4) << \
			m_CameraFrame.GetOriginX()<<",y="<<m_CameraFrame.GetOriginY()\
			<<",z="<<m_CameraFrame.GetOriginZ()<<")";
	m_CameraFrame.GetUpVector(tmp);
	stream<<",Up:(x="<<tmp[0]<<",y="<<tmp[1]<<",z="<<tmp[2]<<")";
	m_CameraFrame.GetForwardVector(tmp);
	stream<<",Fwd(x="<<tmp[0]<<",y="<<tmp[1]<<",z="<<tmp[2]<<")";
	Cords = stream.str();
	return Cords.c_str();
}
const char* Render::FreeCamera::GetOriginCords()
{
	stringstream stream;
	stream<<"Orgn:(x=" << fixed << setprecision(4) << \
			m_CameraFrame.GetOriginX()<<",y="<<m_CameraFrame.GetOriginY()\
			<<",z="<<m_CameraFrame.GetOriginZ()<<")";
	Cords = stream.str();
	return Cords.c_str();
}
const char* Render::FreeCamera::GetUpCords()
{
	stringstream stream;
	M3DVector3f tmp;
	m_CameraFrame.GetUpVector(tmp);
	stream<<"Up:(x="<< fixed << setprecision(4)<<tmp[0]<<",y="<<tmp[1]<<",z="<<tmp[2]<<")";
	Cords = stream.str();
	return Cords.c_str();
}
const char* Render::FreeCamera::GetFwdCords()
{
	stringstream stream;
	M3DVector3f tmp;
	m_CameraFrame.GetForwardVector(tmp);
	stream<<"Fwd(x="<< fixed << setprecision(4)<<tmp[0]<<",y="<<tmp[1]<<",z="<<tmp[2]<<")";
	Cords = stream.str();
	return Cords.c_str();
}
//================end of embedded FreeCamera implementation==========
//=================embedded fixed billboard implementation============
int Render::FixedBillBoard::blendmode;
GLuint Render::FixedBillBoard::billboard_vertex_buffer;
GLuint Render::FixedBillBoard::programID;
GLuint Render::FixedBillBoard::TextureID;
Render::FixedBillBoard::FixedBillBoard(const char* fileName,
					GLMatrixStack &modelViewMat,GLMatrixStack	&projectionMat,GLShaderManager* mgr):
					modelViewMatrix(modelViewMat),projectionMatrix(projectionMat),m_pShaderManager(mgr)
{
	if(NULL == m_pShaderManager){
		m_pShaderManager = (GLShaderManager*)getDefaultShaderMgr();
	}
	Texture = LoadDDS(fileName);
	assert(Texture);
	static bool once = true;
		// The VBO containing the 4 vertices of the particles.
	static const GLfloat g_vertex_buffer_data[] = {
		 -0.5f, -0.5f, 0.0f,//0
		  0.5f, -0.5f, 0.0f,//1
		 -0.5f,  0.5f, 0.0f,//2
		  0.5f,  0.5f, 0.0f,//3
	};
	if(once){
		once = false;
		blendmode = 1;
		programID = m_pShaderManager->LoadShaderPair( "Billboard.vertexshader", "Billboard.fragmentshader" );
		TextureID  = glGetUniformLocation(programID, "myTextureSampler");
		glGenBuffers(1, &billboard_vertex_buffer);
	      glBindBuffer(GL_ARRAY_BUFFER, billboard_vertex_buffer);
	      glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_DYNAMIC_DRAW);
	
		}
	// Vertex shader
	CameraRight_worldspace_ID  = glGetUniformLocation(FixedBillBoard::programID, "CameraRight_worldspace");
	CameraUp_worldspace_ID  = glGetUniformLocation(FixedBillBoard::programID, "CameraUp_worldspace");
	ViewProjMatrixID = glGetUniformLocation(FixedBillBoard::programID, "VP");
	BillboardPosID = glGetUniformLocation(FixedBillBoard::programID, "BillboardPos");
	BillboardSizeID = glGetUniformLocation(FixedBillBoard::programID, "BillboardSize");


	// Get a handle for our buffers
	squareVerticesID = glGetAttribLocation(FixedBillBoard::programID, "squareVertices");


	bbdsize[0] = DEFAULT_FIXED_BBD_WIDTH;
	bbdsize[1] = DEFAULT_FIXED_BBD_HEIGHT;

}
Render::FixedBillBoard::~FixedBillBoard()
{
      static bool once =true;
	if(once){
		once = false;
		glDeleteBuffers(1, &billboard_vertex_buffer);
		glDeleteProgram(programID);
		glDeleteTextures(1, &TextureID);
	}
}
void Render::FixedBillBoard::SetBackLocation(float location[3])
{
	position[0] = location[0];
	position[1] = location[1];
	position[2] = location[2];
}
void Render::FixedBillBoard::SetHeadLocation(float location[3])
{
	head_position[0] = location[0];
	head_position[1] = location[1];
	head_position[2] = location[2];
}
void Render::FixedBillBoard::SetFishEyeLocation(float location[3])
{
	fisheye_position[0] = location[0];
	fisheye_position[1] = location[1];
	fisheye_position[2] = location[2];
}
void Render::FixedBillBoard::SetSize(float size[2])
{
	bbdsize[0] = size[0];
	bbdsize[1] = size[1];
}

void Render::FixedBillBoard::Draw(GLEnv &m_env,M3DMatrix44f ViewMatrix, LOCATION_BBD loc)
{
	GLGeometryTransform * pTransformPipeline = (GLGeometryTransform *)getDefaultTransformPipeline(m_env);

	// Bind our texture in Texture Unit 0
	glActiveTexture(GL_TEXTURE21);
	glBindTexture(GL_TEXTURE_2D, Texture);
	// Set our "myTextureSampler" sampler to user Texture Unit 0
	glUniform1i(TextureID, 21);

	// This is the only interesting part of the tutorial.
	// This is equivalent to mlutiplying (1,0,0) and (0,1,0) by inverse(ViewMatrix).
	// ViewMatrix is orthogonal (it was made this way), 
	// so its inverse is also its transpose, 
	// and transposing a matrix is "free" (inversing is slooow)
	glUniform3f(CameraRight_worldspace_ID, ViewMatrix[0], ViewMatrix[4], ViewMatrix[8]);
	glUniform3f(CameraUp_worldspace_ID   , ViewMatrix[1], ViewMatrix[5], ViewMatrix[9]);
    if( LOCATION_BBD_FISHEYE ==loc){
    	 glUniform3f(BillboardPosID, fisheye_position[0], fisheye_position[1], fisheye_position[2]); // The billboard will be just above the cube
    	 glUniform2f(BillboardSizeID, bbdsize[0]/5, bbdsize[1]/5);     // and 1m*12cm, because it matches its 256*32 resolution =)
    }
    else if(LOCATION_BBD_BACK == loc){
	    glUniform3f(BillboardPosID, position[0], position[1], position[2]); // The billboard will be just above the cube
	    glUniform2f(BillboardSizeID, bbdsize[0], bbdsize[1]);     // and 1m*12cm, because it matches its 256*32 resolution =)
    }
    else{//location_bbd_head
	    glUniform3f(BillboardPosID, head_position[0], head_position[1]-bbdsize[1], head_position[2]); // The billboard will be just above the cube
	    glUniform2f(BillboardSizeID, bbdsize[0], bbdsize[1]);     // and 1m*12cm, because it matches its 256*32 resolution =)
    }
    

	glUniformMatrix4fv(ViewProjMatrixID, 1, GL_FALSE, pTransformPipeline->GetModelViewProjectionMatrix());

	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(squareVerticesID);
	glBindBuffer(GL_ARRAY_BUFFER, billboard_vertex_buffer);
	glVertexAttribPointer(
		squareVerticesID,                  // attribute. 
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
	);


	// Draw the billboard !
	// This draws a triangle_strip which looks like a quad.
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	glDisableVertexAttribArray(squareVerticesID);
}
void Render::FixedBillBoard::DrawGroup(GLEnv &m_env,M3DMatrix44f camera, FixedBillBoard *bbd[], unsigned int count)
{
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	Render::SwitchBlendMode(blendmode);
	// Use our shader
	glUseProgram(programID);

	for(int i = 0; i < count; i ++){
		bbd[i]->Draw(m_env,camera,FixedBillBoard::LOCATION_BBD_BACK);
		bbd[i]->Draw(m_env,camera,FixedBillBoard::LOCATION_BBD_HEAD);
	}
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);	
}
void Render::FixedBillBoard::DrawSingle(GLEnv &m_env,M3DMatrix44f camera, FixedBillBoard *bbd)
{
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	Render::SwitchBlendMode(blendmode);
	// Use our shader
	glUseProgram(programID);
	bbd->Draw(m_env,camera, FixedBillBoard::LOCATION_BBD_FISHEYE);

	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
}
#define FOURCC_DXT1 0x31545844 // Equivalent to "DXT1" in ASCII
#define FOURCC_DXT3 0x33545844 // Equivalent to "DXT3" in ASCII
#define FOURCC_DXT5 0x35545844 // Equivalent to "DXT5" in ASCII
GLuint Render::FixedBillBoard::LoadDDS(const char* imagepath)
{

	unsigned char header[124];

	FILE *fp; 
 
	/* try to open the file */ 
	fp = fopen(imagepath, "rb"); 
	if (fp == NULL){
		printf("%s could not be opened. Are you in the right directory ?!\n", imagepath);
		return 0;
	}
   
	/* verify the type of file */ 
	char filecode[4]; 
	fread(filecode, 1, 4, fp); 
	if (strncmp(filecode, "DDS ", 4) != 0) { 
		fclose(fp); 
		return 0; 
	}
	
	/* get the surface desc */ 
	fread(&header, 124, 1, fp); 

	unsigned int height      = *(unsigned int*)&(header[8 ]);
	unsigned int width	     = *(unsigned int*)&(header[12]);
	unsigned int linearSize	 = *(unsigned int*)&(header[16]);
	unsigned int mipMapCount = *(unsigned int*)&(header[24]);
	unsigned int fourCC      = *(unsigned int*)&(header[80]);

 
	unsigned char * buffer;
	unsigned int bufsize;
	/* how big is it going to be including all mipmaps? */ 
	bufsize = mipMapCount > 1 ? linearSize * 2 : linearSize; 
	buffer = (unsigned char*)malloc(bufsize * sizeof(unsigned char)); 
	fread(buffer, 1, bufsize, fp); 
	/* close the file pointer */ 
	fclose(fp);

	unsigned int components  = (fourCC == FOURCC_DXT1) ? 3 : 4; 
	unsigned int format;
	switch(fourCC) 
	{ 
	case FOURCC_DXT1: 
		format = GL_COMPRESSED_RGBA_S3TC_DXT1_EXT; 
		break; 
	case FOURCC_DXT3: 
		format = GL_COMPRESSED_RGBA_S3TC_DXT3_EXT; 
		break; 
	case FOURCC_DXT5: 
		format = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT; 
		break; 
	default: 
		free(buffer); 
		return 0; 
	}

	// Create one OpenGL texture
	GLuint textureID;
	glGenTextures(1, &textureID);

	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, textureID);
	glPixelStorei(GL_UNPACK_ALIGNMENT,1);	
	
	unsigned int blockSize = (format == GL_COMPRESSED_RGBA_S3TC_DXT1_EXT) ? 8 : 16; 
	unsigned int offset = 0;

	/* load the mipmaps */ 
	for (unsigned int level = 0; level < mipMapCount && (width || height); ++level) 
	{ 
		unsigned int size = ((width+3)/4)*((height+3)/4)*blockSize; 
		glCompressedTexImage2D(GL_TEXTURE_2D, level, format, width, height,  
			0, size, buffer + offset); 
	 
		offset += size; 
		width  /= 2; 
		height /= 2; 

		// Deal with Non-Power-Of-Two textures. This code is not included in the webpage to reduce clutter.
		if(width < 1) width = 1;
		if(height < 1) height = 1;

	} 

	free(buffer); 

	return textureID;


}
void Render::UpdateWheelAngle()
{
	p_DynamicTrack->SetAngle(m_DynamicWheelAngle);
}

int Render::SetWheelArcWidth(float arcWidth)
{
	if(p_DynamicTrack == NULL)
		return -1;
	p_DynamicTrack->SetArcWidth(arcWidth);
		return 0;
}

void Render::DrawSlideonPanel(GLEnv &m_env)
{
	// Load as a bunch of line segments
	GLfloat vTracks[30][3];
	GLfloat fixBBDPos[3];
	GLfloat Track_to_Vehicle_width_rate = DEFAULT_TRACK2_VEHICLE_WIDTH_RATE;
	const GLfloat* pVehicleDimension = pVehicle->GetDimensions();
	const GLfloat* pVehicleYMaxMin = pVehicle->GetYMaxMins();
	GLfloat   TrackLength = DEFAULT_TRACK_LENGTH_METER;
	GLfloat   TrackWidth = Track_to_Vehicle_width_rate*pVehicleDimension[0];
	int i = 0;

	float pano_length=0.0,pano_height=0.0,scan_view_length=0.0;
	float now_scan_pos=0.0;
	float height_delta=0.8f;

	pano_length=PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x();
	pano_height=PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z();
	scan_view_length=getScanRegionAngle()*pano_length/360.0;
	now_scan_pos=PanelLoader.GetScan_pos();

	//first
	vTracks[i][0] =now_scan_pos-scan_view_length/2;//-TrackWidth/30;
	vTracks[i][1] =-0.1f;
	vTracks[i++][2] = height_delta;

	vTracks[i][0] = now_scan_pos-scan_view_length/2;//-TrackWidth/30;
	vTracks[i][1] = -0.1f;
	vTracks[i++][2] = pano_height-height_delta;

	vTracks[i][0] = now_scan_pos+scan_view_length/2;
	vTracks[i][1] =-0.1f;
	vTracks[i++][2] =pano_height-height_delta;

	vTracks[i][0] = now_scan_pos+scan_view_length/2;
	vTracks[i][1] = -0.1f;
	vTracks[i++][2] =height_delta;

	vTracks[i][0] = now_scan_pos-scan_view_length/2;//-TrackWidth/30;
	vTracks[i][1] =-0.1f;
	vTracks[i++][2] =height_delta;

	vTracks[i][0] = now_scan_pos+scan_view_length/2;
	vTracks[i][1] = -0.1f;
	vTracks[i++][2] =height_delta;

	vTracks[i][0] = now_scan_pos-scan_view_length/2;//-TrackWidth/30;
	vTracks[i][1] =-0.1f;
	vTracks[i++][2] =pano_height-height_delta;

	vTracks[i][0] = now_scan_pos+scan_view_length/2;
	vTracks[i][1] = -0.1f;
	vTracks[i++][2] =pano_height-height_delta;

	//second
	vTracks[i][0] =now_scan_pos-scan_view_length/2+pano_length;//-TrackWidth/30;
	vTracks[i][1] =-0.1f;
	vTracks[i++][2] =height_delta;

	vTracks[i][0] = now_scan_pos-scan_view_length/2+pano_length;//-TrackWidth/30;
	vTracks[i][1] = -0.1f;
	vTracks[i++][2] = pano_height-height_delta;

	vTracks[i][0] = now_scan_pos+scan_view_length/2+pano_length;
	vTracks[i][1] =-0.1f;
	vTracks[i++][2] =pano_height-height_delta;

	vTracks[i][0] = now_scan_pos+scan_view_length/2+pano_length;
	vTracks[i][1] = -0.1f;
	vTracks[i++][2] =height_delta;

	vTracks[i][0] = now_scan_pos-scan_view_length/2+pano_length;//-TrackWidth/30;
	vTracks[i][1] =-0.1f;
	vTracks[i++][2] =height_delta;

	vTracks[i][0] = now_scan_pos+scan_view_length/2+pano_length;
	vTracks[i][1] = -0.1f;
	vTracks[i++][2] =height_delta;

	vTracks[i][0] = now_scan_pos-scan_view_length/2+pano_length;//-TrackWidth/30;
	vTracks[i][1] =-0.1f;
	vTracks[i++][2] =pano_height-height_delta;

	vTracks[i][0] = now_scan_pos+scan_view_length/2+pano_length;
	vTracks[i][1] = -0.1f;
	vTracks[i++][2] =pano_height-height_delta;

	//third
	vTracks[i][0] =now_scan_pos-scan_view_length/2-pano_length;//-TrackWidth/30;
	vTracks[i][1] =-0.1f;
	vTracks[i++][2] =height_delta;

	vTracks[i][0] = now_scan_pos-scan_view_length/2-pano_length;//-TrackWidth/30;
	vTracks[i][1] = -0.1f;
	vTracks[i++][2] = pano_height-height_delta;

	vTracks[i][0] = now_scan_pos+scan_view_length/2-pano_length;
	vTracks[i][1] =-0.1f;
	vTracks[i++][2] =pano_height-height_delta;

	vTracks[i][0] = now_scan_pos+scan_view_length/2-pano_length;
	vTracks[i][1] = -0.1f;
	vTracks[i++][2] =height_delta;

	vTracks[i][0] = now_scan_pos-scan_view_length/2-pano_length;//-TrackWidth/30;
	vTracks[i][1] =-0.1f;
	vTracks[i++][2] =height_delta;

	vTracks[i][0] = now_scan_pos+scan_view_length/2-pano_length;
	vTracks[i][1] = -0.1f;
	vTracks[i++][2] =height_delta;

	vTracks[i][0] = now_scan_pos-scan_view_length/2-pano_length;//-TrackWidth/30;
	vTracks[i][1] =-0.1f;
	vTracks[i++][2] =pano_height-height_delta;

	vTracks[i][0] = now_scan_pos+scan_view_length/2-pano_length;
	vTracks[i][1] = -0.1f;
	vTracks[i++][2] =pano_height-height_delta;

	SlideFrameBatch.Begin(GL_LINES, i);
	SlideFrameBatch.CopyVertexData3f(vTracks);
	SlideFrameBatch.End();

	m_env.GetmodelViewMatrix()->PushMatrix();

//	glDisable(GL_BLEND);
//	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vLtYellow);
	glLineWidth(2.0f);
	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vRed);
	SlideFrameBatch.Draw();
	m_env.GetmodelViewMatrix()->PopMatrix();

}

void Render::DrawCrossonPanel(GLEnv &m_env)
{
	// Load as a bunch of line segments
	GLfloat vTracks[50][3];
	GLfloat fixBBDPos[3];
	GLfloat Track_to_Vehicle_width_rate = DEFAULT_TRACK2_VEHICLE_WIDTH_RATE;
	const GLfloat* pVehicleDimension = pVehicle->GetDimensions();
	const GLfloat* pVehicleYMaxMin = pVehicle->GetYMaxMins();
	GLfloat   TrackLength = DEFAULT_TRACK_LENGTH_METER;
	GLfloat   TrackWidth = Track_to_Vehicle_width_rate*pVehicleDimension[0];
	int i = 0;

	float cross_width=0.0,cross_height=0.0,margin_width=0.0,margin_height=0.0;

	float cross_center_x=cross_center_pos[0],cross_center_y=cross_center_pos[1];

	float left_delta=-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());

	cross_width=CROSS_WIDTH;
	cross_height=CROSS_HEIGHT;
	margin_width=CROSS_MARGIN_WIDTH;
	margin_height=CROSS_MARGIN_HEIGHT;
//right front
	vTracks[i][0] =cross_center_x-cross_width;
	vTracks[i][1] =-0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] = cross_center_x-margin_width;
	vTracks[i][1] = -0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] =cross_center_x+cross_width;
	vTracks[i][1] =-0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] = cross_center_x+margin_width;
	vTracks[i][1] = -0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] =cross_center_x;
	vTracks[i][1] =-0.01f;
	vTracks[i++][2] = cross_center_y+cross_height;

	vTracks[i][0] = cross_center_x;
	vTracks[i][1] = -0.01f;
	vTracks[i++][2] = cross_center_y+margin_height;

	vTracks[i][0] =cross_center_x;
	vTracks[i][1] =-0.01f;
	vTracks[i++][2] = cross_center_y-cross_height;

	vTracks[i][0] = cross_center_x;
	vTracks[i][1] = -0.01f;
	vTracks[i++][2] = cross_center_y-margin_height;



//right rear
	vTracks[i][0] =cross_center_x-cross_width;
	vTracks[i][1] =0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] = cross_center_x-margin_width;
	vTracks[i][1] = 0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] =cross_center_x+cross_width;
	vTracks[i][1] =0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] = cross_center_x+margin_width;
	vTracks[i][1] = 0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] =cross_center_x;
	vTracks[i][1] =0.01f;
	vTracks[i++][2] = cross_center_y+cross_height;

	vTracks[i][0] = cross_center_x;
	vTracks[i][1] = 0.01f;
	vTracks[i++][2] = cross_center_y+margin_height;

	vTracks[i][0] =cross_center_x;
	vTracks[i][1] =0.01f;
	vTracks[i++][2] = cross_center_y-cross_height;

	vTracks[i][0] = cross_center_x;
	vTracks[i][1] = 0.01f;
	vTracks[i++][2] = cross_center_y-margin_height;



//left_front
	vTracks[i][0] =cross_center_x-cross_width+left_delta;
	vTracks[i][1] =-0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] = cross_center_x-margin_width+left_delta;
	vTracks[i][1] = -0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] =cross_center_x+cross_width+left_delta;
	vTracks[i][1] =-0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] = cross_center_x+margin_width+left_delta;
	vTracks[i][1] = -0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] =cross_center_x+left_delta;
	vTracks[i][1] =-0.01f;
	vTracks[i++][2] = cross_center_y+cross_height;

	vTracks[i][0] = cross_center_x+left_delta;
	vTracks[i][1] = -0.01f;
	vTracks[i++][2] = cross_center_y+margin_height;

	vTracks[i][0] =cross_center_x+left_delta;
	vTracks[i][1] =-0.01f;
	vTracks[i++][2] = cross_center_y-cross_height;

	vTracks[i][0] = cross_center_x+left_delta;
	vTracks[i][1] = -0.01f;
	vTracks[i++][2] = cross_center_y-margin_height;



//left rear
	vTracks[i][0] =cross_center_x-cross_width+left_delta;
	vTracks[i][1] =0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] = cross_center_x-margin_width+left_delta;
	vTracks[i][1] = 0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] =cross_center_x+cross_width+left_delta;
	vTracks[i][1] =0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] = cross_center_x+margin_width+left_delta;
	vTracks[i][1] = 0.01f;
	vTracks[i++][2] = cross_center_y;

	vTracks[i][0] =cross_center_x+left_delta;
	vTracks[i][1] =0.01f;
	vTracks[i++][2] = cross_center_y+cross_height;

	vTracks[i][0] = cross_center_x+left_delta;
	vTracks[i][1] = 0.01f;
	vTracks[i++][2] = cross_center_y+margin_height;

	vTracks[i][0] =cross_center_x+left_delta;
	vTracks[i][1] =0.01f;
	vTracks[i++][2] = cross_center_y-cross_height;

	vTracks[i][0] = cross_center_x+left_delta;
	vTracks[i][1] = 0.01f;
	vTracks[i++][2] = cross_center_y-margin_height;

	CrossFrameBatch.Begin(GL_LINES, i);
	CrossFrameBatch.CopyVertexData3f(vTracks);
	CrossFrameBatch.End();

	m_env.GetmodelViewMatrix()->PushMatrix();

//	glDisable(GL_BLEND);
//	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vLtYellow);
	glLineWidth(2.0f);
	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vYellow);
	CrossFrameBatch.Draw();
	m_env.GetmodelViewMatrix()->PopMatrix();

}

void Render::DrawRuleronPanel(GLEnv &m_env)
{
	// Load as a bunch of line segments
	float get_lineofruler=0.0;
	get_lineofruler=p_LineofRuler->GetAngle();
	if(displayMode==TWO_HALF_PANO_VIEW_MODE||displayMode==FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE||
			displayMode==FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE)
	{
		get_lineofruler=get_lineofruler+90.0;
		if(get_lineofruler>=360.0)
		{
			get_lineofruler=get_lineofruler-360.0;
		}
	}
	if(displayMode==TWO_HALF_PANO_VIEW_MODE)
	{
		get_lineofruler=get_lineofruler+90.0;
		if(get_lineofruler>=360.0)
		{
			get_lineofruler=get_lineofruler-360.0;
		}
	}
	if(displayMode==PANO_VIEW_MODE)
	{
		get_lineofruler=get_lineofruler+0.0;
		if(get_lineofruler>=360.0)
		{
			get_lineofruler=get_lineofruler-360.0;
		}
	}

	float the_ruler_pos=PanelLoader.Getextent_neg_x()+(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())*get_lineofruler/360.0;

	float the_low_pos=PanelLoader.Getextent_neg_z();
	float the_high_pos=PanelLoader.Getextent_pos_z();

	float temp_y=0.01f;

	int i=0;
	float math_ruler_pos[36];
	for(i=0;i<2;i++)
	{
		if(i%2==0)
		{
			temp_y=0.01f;
		}
		else
		{
			temp_y=-0.01f;
		}
		math_ruler_pos[6*i]=the_ruler_pos;
		math_ruler_pos[6*i+1]=temp_y;
		math_ruler_pos[6*i+2]=the_low_pos;

		math_ruler_pos[6*i+3]=the_ruler_pos;
		math_ruler_pos[6*i+4]=temp_y;
		math_ruler_pos[6*i+5]=the_high_pos;
	}
	for(;i<4;i++)
	{
		if(i%2==0)
		{
			temp_y=0.01f;
		}
		else
		{
			temp_y=-0.01f;
		}
		math_ruler_pos[6*i]=the_ruler_pos-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
		math_ruler_pos[6*i+1]=temp_y;
		math_ruler_pos[6*i+2]=the_low_pos;

		math_ruler_pos[6*i+3]=the_ruler_pos-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
		math_ruler_pos[6*i+4]=temp_y;
		math_ruler_pos[6*i+5]=the_high_pos;
	}
	for(;i<6;i++)
	{
		if(i%2==0)
		{
			temp_y=0.01f;
		}
		else
		{
			temp_y=-0.01f;
		}
		math_ruler_pos[6*i]=the_ruler_pos+(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
		math_ruler_pos[6*i+1]=temp_y;
		math_ruler_pos[6*i+2]=the_low_pos;

		math_ruler_pos[6*i+3]=the_ruler_pos+(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
		math_ruler_pos[6*i+4]=temp_y;
		math_ruler_pos[6*i+5]=the_high_pos;
	}
	p_LineofRuler->DrawRuler(m_env,math_ruler_pos);
//reference line
	float reference_pos[24];
	float reference_angle=getrulerreferenceangle();
	float the_reference_pos=0.0;

	if(displayMode==TWO_HALF_PANO_VIEW_MODE||displayMode==FRONT_BACK_PANO_ADD_MONITOR_VIEW_MODE||
			displayMode==FRONT_BACK_PANO_ADD_SMALLMONITOR_VIEW_MODE)
	{
		reference_angle=reference_angle+90.0;
		if(reference_angle>360.0)
		{
			reference_angle=reference_angle-360.0;
		}
	}
	if(displayMode==TWO_HALF_PANO_VIEW_MODE)
	{
		reference_angle=reference_angle+90.0;
		if(reference_angle>360.0)
		{
			reference_angle=reference_angle-360.0;
		}
	}
	if(displayMode==PANO_VIEW_MODE)
	{
		reference_angle=reference_angle+0.0;
		if(reference_angle>360.0)
		{
			reference_angle=reference_angle-360.0;
		}
	}
	the_reference_pos=PanelLoader.Getextent_neg_x()+(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())*reference_angle/360.0;
	i=0;
	/*
	for(i=0;i<8;i++)
	{
		reference_pos[3*i]=math_ruler_pos[3*i]+2.0;
		reference_pos[3*i+1]=math_ruler_pos[3*i+1];
		reference_pos[3*i+2]=math_ruler_pos[3*i+2];
	}*/

	reference_pos[i++]=the_reference_pos;
	reference_pos[i++]=0.01f;
	reference_pos[i++]=the_low_pos;

	reference_pos[i++]=the_reference_pos;
	reference_pos[i++]=0.01f;
	reference_pos[i++]=the_high_pos;

	reference_pos[i++]=the_reference_pos;
	reference_pos[i++]=-0.01f;
	reference_pos[i++]=the_low_pos;

	reference_pos[i++]=the_reference_pos;
	reference_pos[i++]=-0.01f;
	reference_pos[i++]=the_high_pos;



	reference_pos[i++]=the_reference_pos+(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
	reference_pos[i++]=0.01f;
	reference_pos[i++]=the_low_pos;

	reference_pos[i++]=the_reference_pos+(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
	reference_pos[i++]=0.01f;
	reference_pos[i++]=the_high_pos;

	reference_pos[i++]=the_reference_pos+(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
	reference_pos[i++]=-0.01f;
	reference_pos[i++]=the_low_pos;

	reference_pos[i++]=the_reference_pos+(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
	reference_pos[i++]=-0.01f;
	reference_pos[i++]=the_high_pos;



	reference_pos[i++]=the_reference_pos-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
	reference_pos[i++]=0.01f;
	reference_pos[i++]=the_low_pos;

	reference_pos[i++]=the_reference_pos-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
	reference_pos[i++]=0.01f;
	reference_pos[i++]=the_high_pos;

	reference_pos[i++]=the_reference_pos-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
	reference_pos[i++]=-0.01f;
	reference_pos[i++]=the_low_pos;

	reference_pos[i++]=the_reference_pos-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
	reference_pos[i++]=-0.01f;
	reference_pos[i++]=the_high_pos;

	p_LineofRuler->DrawReferenceLine(m_env,reference_pos);
}

void Render::InitFollowCross()
{
	float center_pos[2];
	center_pos[0]=(PanelLoader.Getextent_pos_x()+PanelLoader.Getextent_neg_x())/2;
	center_pos[1]=(PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z())/2;
	setCrossCenterPos(center_pos);
	setFollowVaule(false);
}

void Render::InitCalibrate()
{
	setenableshowruler(false);
}


void Render::InitScanAngle(void )
{
	const char * filename="./scanangle.yml";
	readScanAngle(filename);
}

void Render::readScanAngle(const char * filename)
{
	char buf[256];
	float get_scan_angle=SCAN_REGION_ANGLE;
	float get_ruler_angle=RULER_START_ANGLE;
	FILE * fp=fopen(filename,"r");
	static bool once=true;
	if(fp!=NULL)
	{
		fscanf(fp,"%f\n%f\n",&get_scan_angle,&get_ruler_angle);
		fclose(fp);
	}
	setScanRegionAngle(get_scan_angle);
	setrulerangle(get_ruler_angle);
	if(once)
	{
		once=false;
		setpanodeltaangle(get_ruler_angle);
	}
}

void Render::writeScanAngle(const char * filename,float angle,float angleofruler)
{
	char buf[256];

	float get_scan_angle=SCAN_REGION_ANGLE;
	float get_ruler_angle=RULER_START_ANGLE;
	FILE * fp=fopen(filename,"r");
	if(fp!=NULL)
	{
		fscanf(fp,"%f\n%f\n",&get_scan_angle,&get_ruler_angle);
		fclose(fp);
	}


	fp=fopen(filename,"w");

	float write_angle=0.0,write_angleofruler=0.0;
	if(getSendFollowAngleEnable())
	{
		write_angle=getSendFollowAngleEnable();
	}
	else
	{
		write_angle=get_scan_angle;
	}

	if(getenableshowruler())
	{
		write_angleofruler=getrulerangle();
	}
	else
	{
		write_angleofruler=get_ruler_angle;
	}
	sprintf(buf,"%f\n%f\n",write_angle,write_angleofruler);
	fwrite(buf,sizeof(buf),1,fp);
	fclose(fp);
}

void Render::repositioncamera()
{
	float tel_pano_dis=0.0;
	static float last_tel_pano_dis=0.0;
	float Len=(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
	float move_dis=0.0;
	float right_move_dis=0.0;
	float pano_dis=0.0;
	static float last_move_dis=0.0;
	static float last_pano_dis=0.0;
	static float last_right_move_dis=0.0;//((PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())*(3.0-0.025)+0.4)/4.0;
	if(getenablerefrushruler())
	{
		setrulerreferenceangle(p_LineofRuler->GetAngle());
		move_dis=(p_LineofRuler->GetAngle())*(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())/360.0;

		if(move_dis<(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())/2.0)
		{
			move_dis=(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())+move_dis;
		}
		else if(move_dis<(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())*3.0/2.0)
		{
			move_dis=-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())+move_dis;;
		}

		tel_pano_dis=(p_LineofRuler->GetAngle())/360*Len;
		cout<<"tel_pano_dis="<<tel_pano_dis<<endl;
		if(tel_pano_dis>Len*3/4)
		{
			tel_pano_dis-=Len;
			tel_pano_dis+=Len;
		}
		else
		tel_pano_dis+=Len;   //ORI POS is LEN left  so move right




		//pano_dis=move_dis+(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x())/2.0;
		pano_dis=move_dis;


		PanoViewCameraFrame.MoveRight(4.0*last_pano_dis);
		PanoViewCameraFrame.MoveRight(-4.0*pano_dis);

		for(int i=0;i<2;i++)
		{
		PanoTelViewCameraFrame[i].MoveRight(8.0*last_tel_pano_dis);
			PanoTelViewCameraFrame[i].MoveRight(-8.0*tel_pano_dis);
		}
		last_tel_pano_dis=tel_pano_dis;

		LeftSmallPanoViewCameraFrame.MoveRight(4.0*last_move_dis);
		LeftSmallPanoViewCameraFrame.MoveRight(-4.0*move_dis);

		LeftPanoViewCameraFrame.MoveRight(4.0*last_move_dis);
		LeftPanoViewCameraFrame.MoveRight(-4.0*move_dis);

		right_move_dis=move_dis-(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());

		RightSmallPanoViewCameraFrame.MoveRight(-4.0*last_right_move_dis);
		RightSmallPanoViewCameraFrame.MoveRight(4.0*right_move_dis);

		RightPanoViewCameraFrame.MoveRight(-4.0*last_right_move_dis);
		RightPanoViewCameraFrame.MoveRight(4.0*right_move_dis);

		last_move_dis=move_dis;
		last_pano_dis=pano_dis;
		last_right_move_dis=right_move_dis;
		setenablerefrushruler(false);
	}
}

void Render::InitALPHA_ZOOM_SCALE(void) {
       const char * filename = "./ALPHA_ZOOM_SCALE.txt";
       readALPHA_ZOOM_SCALE(filename);
}

void Render::readALPHA_ZOOM_SCALE(const char * filename) {
       char buf[256];
       float ALPHA_ZOOM_SCALE_3 = 0.5;
       FILE * fp = fopen(filename, "r");
       if (fp != NULL) {
               fscanf(fp, "%f\n", &ALPHA_ZOOM_SCALE_3);
               fclose(fp);
	}
	setALPHA_ZOOM_SCALE(ALPHA_ZOOM_SCALE_3);
}

void Render::writeALPHA_ZOOM_SCALE(char * filename, float ALPHA_ZOOM_SCALE_4) {
       char buf[16];
       FILE * fp = fopen(filename, "w");
       sprintf(buf, "%f\n", ALPHA_ZOOM_SCALE_4);
       fwrite(buf, sizeof(float), 1, fp);
       fclose(fp);
}



void Render::PrepareAlarmAera(GLEnv &m_env,int x,int y,int w,int h)
{
	//readcanshu();
	static int Once=true;
	if(Once)
	{
#if test
				readcanshu();
				int set_pos[8]={
						canshu[0],canshu[1],
						canshu[2],canshu[3],
						canshu[4],canshu[5],
						canshu[6],canshu[7]
#else
		int set_pos[8]={
				ALARM_MIN_X,ALARM_MIN_Y,
				ALARM_MAX_X,ALARM_MIN_Y,
				ALARM_MAX_X,ALARM_MAX_Y,
				ALARM_MIN_X,ALARM_MAX_Y
#endif
		};
		InitAlarmArea(set_pos,TYPE_MOVE);
		Once=false;
	}
//	if(zodiac_msg.GetdispalyMode()==RECV_ENABLE_TRACK)
	{
	static timeval starttime,startalarm_time,read_pic_time,send_pic_time,write_pic_time,draw_target_time;
	gettimeofday(&starttime,0);
	static timeval lasttime;
	static timeval timebeforereadpixel;
	static timeval preparation;
	static int frametime=-1;

	static int last_alarm_time=-1;

	static unsigned char alarm_area_data[1920*1080*4];
	int math_alarm_pos[ALARM_MAX_COUNT][4];
	int read_alarm_pos[8];
	float draw_pos[8];
	int i=0,j=0,k=0;
	float draw_data[8];
	int alarm_offset[2];
	Rect temp_rect;
	char img_filename[50];
	float width_scale=1.0;
	int width4mode=0;
	int height4mode=0;
static int readpixel_count=0;
static int count=0;
static float  sum_wait=0,sum_read=0,sum_write=0,sum_send=0;
	if(last_alarm_time<0)
	{
		last_alarm_time=0;
	}
	else
	{
		last_alarm_time=(starttime.tv_sec-lasttime.tv_sec)*1000000+(starttime.tv_usec-lasttime.tv_usec)+last_alarm_time;

		{
					for(i=0;i<ALARM_MAX_COUNT;i++)
					{
						for(j=0;j<4;j++)
						{
							math_alarm_pos[i][j]=100;
						}
					}
					for(i=0;i<p_dataofalarmarea->GetAlarmAreaCount();i++)
					{
						gettimeofday(&startalarm_time,0);
						memcpy(read_alarm_pos,p_dataofalarmarea->GetAlarmPos(i),sizeof(read_alarm_pos));
						for(k=0;k<8;k++)
						{
							draw_data[k]=read_alarm_pos[k]*1.0;
						}

						{
							memcpy(alarm_offset,p_dataofalarmarea->GetOffsetData(i),sizeof(alarm_offset));
							getoutline(read_alarm_pos,math_alarm_pos[i],8);

							draw_pos[0]=math_alarm_pos[i][0];
							draw_pos[1]=math_alarm_pos[i][1];
							draw_pos[2]=math_alarm_pos[i][2];
							draw_pos[3]=math_alarm_pos[i][1];
							draw_pos[4]=math_alarm_pos[i][2];
							draw_pos[5]=math_alarm_pos[i][3];
							draw_pos[6]=math_alarm_pos[i][0];
							draw_pos[7]=math_alarm_pos[i][3];
							/*
							if(mainAlarmTarget.GetTargetType(i)!=TYPE_CROSS_BORDER)
							{
								DrawAlarmArea(draw_data,x,y,w,h,COLOR_NORMAL);//alarm area
							}
							else
							{
								DrawAlarmArea(draw_data,x,y,w,h,COLOR_LINE);//alarm area
							}
							*/
							width4mode=(math_alarm_pos[i][2]-math_alarm_pos[i][0])%4;
							height4mode=(math_alarm_pos[i][3]-math_alarm_pos[i][1])%4;

							gettimeofday(&preparation,0);
							if(last_alarm_time>alarm_period)
							{
								readpixel_count++;
								gettimeofday(&timebeforereadpixel,0);
								glReadPixels(math_alarm_pos[i][0]-PIXELS_ADD_ON_ALARM,
										math_alarm_pos[i][1]-PIXELS_ADD_ON_ALARM,
										(math_alarm_pos[i][2]-math_alarm_pos[i][0]-width4mode)*width_scale+2*PIXELS_ADD_ON_ALARM,
										math_alarm_pos[i][3]-math_alarm_pos[i][1]-height4mode+2*PIXELS_ADD_ON_ALARM,
										GL_BGRA_EXT,GL_UNSIGNED_BYTE,alarm_area_data);
								gettimeofday(&read_pic_time,0);
								sum_write+=(read_pic_time.tv_sec-timebeforereadpixel.tv_sec)*1000000+(read_pic_time.tv_usec-timebeforereadpixel.tv_usec);
								Mat frame(math_alarm_pos[i][3]-math_alarm_pos[i][1]-height4mode+2*PIXELS_ADD_ON_ALARM,
										(math_alarm_pos[i][2]-math_alarm_pos[i][0]-width4mode)*width_scale+2*PIXELS_ADD_ON_ALARM,
										CV_8UC4,alarm_area_data);

								sprintf(img_filename,"alarm%.2d.bmp",i);
								gettimeofday(&write_pic_time,0);
#ifdef MVDETECTOR_MODE
					//			pSingleMvDetector->process_frame(frame,i);
								gettimeofday(&send_pic_time,0);
#endif
							}
							for(j=0;j<mainAlarmTarget.GetTargetCount(i);j++)
							{
								temp_rect=mainAlarmTarget.GetSingleRectangle(i,j);
								draw_pos[0]=temp_rect.x+alarm_offset[0]-PIXELS_ADD_ON_ALARM;
								draw_pos[1]=temp_rect.y+alarm_offset[1]-PIXELS_ADD_ON_ALARM;
								draw_pos[2]=temp_rect.x+alarm_offset[0]-PIXELS_ADD_ON_ALARM+temp_rect.width;
								draw_pos[3]=temp_rect.y+alarm_offset[1]-PIXELS_ADD_ON_ALARM;
								draw_pos[4]=temp_rect.x+alarm_offset[0]-PIXELS_ADD_ON_ALARM+temp_rect.width;
								draw_pos[5]=temp_rect.y+alarm_offset[1]-PIXELS_ADD_ON_ALARM+temp_rect.height;
								draw_pos[6]=temp_rect.x+alarm_offset[0]-PIXELS_ADD_ON_ALARM;
								draw_pos[7]=temp_rect.y+alarm_offset[1]-PIXELS_ADD_ON_ALARM+temp_rect.height;
								DrawAlarmArea(m_env,draw_pos,0,0,w,h,mainAlarmTarget.GetTargetType(i));//target
							}
							gettimeofday(&draw_target_time,0);

							count++;
							if(count>=50)
							{
								count=0;
								sum_wait=0;
								sum_read=0;
								sum_write=0;
								sum_send=0;
								readpixel_count=0;
							}
							else
							{
								sum_wait+=(preparation.tv_sec-starttime.tv_sec)*1000000+(preparation.tv_usec-starttime.tv_usec);
								sum_read+=(starttime.tv_sec-lasttime.tv_sec)*1000000+(starttime.tv_usec-lasttime.tv_usec);

								sum_send+=(send_pic_time.tv_sec-write_pic_time.tv_sec)*1000000+(send_pic_time.tv_usec-write_pic_time.tv_usec);
							}
							mainAlarmTarget.SetTargetCount(i,j);
						}
					}
					if(last_alarm_time>alarm_period)
					{
						last_alarm_time=last_alarm_time-alarm_period;
					}
			}
	}
	lasttime=starttime;
	}
}

void Render::InitDataofAlarmarea()
{
	p_dataofalarmarea=new BaseAlarmObject(TYPE_ALARM_AREA);
	p_dataofalarmline=new BaseAlarmObject(TYPE_ALARM_LINE);
}

void Render::DrawAlarmArea(GLEnv &m_env,float get_pos[8],int x,int y,int w,int h,int color_type)
{
	// Load as a bunch of line segments
	GLfloat vTracks[50][3];
	float pos[8];
	int i = 0,j=0;
	float scale=SMALL_PANO_VIEW_SCALE;
	int window_w=MAX_SCREEN_WIDTH,window_h=MAX_SCREEN_HEIGHT;

	for(i=0;i<4;i++)
	{
		pos[2*i]=(get_pos[2*i]*1.0-w/2.0);
		pos[2*i+1]=(get_pos[2*i+1]*1.0-h/2.0);
	}

	i=0;
	float z_data=forward_data;
	// front
	vTracks[i][0] =pos[j];
	vTracks[i][1] =pos[j+1];
	vTracks[i++][2] =-z_data;

	j=(j+2)%8;

	vTracks[i][0] =pos[j];
	vTracks[i][1] =pos[j+1];
	vTracks[i++][2] =-z_data;

	vTracks[i][0] =pos[j];
	vTracks[i][1] =pos[j+1];
	vTracks[i++][2] =-z_data;

	j=(j+2)%8;

	vTracks[i][0] =pos[j];
	vTracks[i][1] =pos[j+1];
	vTracks[i++][2] =-z_data;

	vTracks[i][0] =pos[j];
	vTracks[i][1] =pos[j+1];
	vTracks[i++][2] =-z_data;

	j=(j+2)%8;

	vTracks[i][0] =pos[j];
	vTracks[i][1] =pos[j+1];
	vTracks[i++][2] =-z_data;

	vTracks[i][0] =pos[j];
	vTracks[i][1] =pos[j+1];
	vTracks[i++][2] =-z_data;

	j=(j+2)%8;

	vTracks[i][0] =pos[j];
	vTracks[i][1] =pos[j+1];
	vTracks[i++][2] =-z_data;

	AlarmAreaBatch.Begin(GL_LINES, i);
	AlarmAreaBatch.CopyVertexData3f(vTracks);
	AlarmAreaBatch.End();

	glViewport(0,0,g_windowWidth,g_windowHeight);
	m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());
	m_env.GetmodelViewMatrix()->PushMatrix();

	glLineWidth(2.0f);
	float draw_color[4];
	switch(color_type)
	{
	case COLOR_NORMAL:
		memcpy(draw_color,vYellow,sizeof(draw_color));
		break;
	case COLOR_LINE:
		memcpy(draw_color,vYellow,sizeof(draw_color));
		break;
	case TYPE_CROSS_BORDER:
		memcpy(draw_color,vGreen,sizeof(draw_color));
		break;
	case TYPE_MOVE:
		memcpy(draw_color,vGreen,sizeof(draw_color));
		break;
	case TYPE_INVADE:
		memcpy(draw_color,vBlue,sizeof(draw_color));
		break;
	case TYPE_LOST:
		memcpy(draw_color,vRed,sizeof(draw_color));
		break;
	default:
		memcpy(draw_color,vGreen,sizeof(draw_color));
		break;
	}
	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), draw_color);
	AlarmAreaBatch.Draw();
	m_env.GetmodelViewMatrix()->PopMatrix();

}

void Render::DrawAlarmAreaonScreen(GLEnv &m_env)
{
	// Load as a bunch of line segments
	GLfloat vTracks[50][3];

	int i = 0,j=0;

	vTracks[i][0] =0;
	vTracks[i][1] =0;
	vTracks[i++][2] = 0;

	j=(j+2)%8;

	vTracks[i][0] =-360.0*3.0/2.0;
	vTracks[i][1] =0;
	vTracks[i++][2] =0;

	vTracks[i][0] =0;
	vTracks[i][1] =0;
	vTracks[i++][2] =0;

	j=(j+2)%8;

	vTracks[i][0] =0.0;
	vTracks[i][1] =-144.0;
	vTracks[i++][2] = 0;

	vTracks[i][0] =0;
	vTracks[i][1] =-72*2.0/3.0;
	vTracks[i++][2] = 0;

	j=(j+2)%8;

	vTracks[i][0] =360.0;
	vTracks[i][1] =-72.0*2.0/3.0;
	vTracks[i++][2] = 0;


	AlarmAreaBatch.Begin(GL_LINES, i);
	AlarmAreaBatch.CopyVertexData3f(vTracks);
	AlarmAreaBatch.End();

	glViewport(0,0,g_windowWidth,g_windowHeight);
	m_env.GetmodelViewMatrix()->PushMatrix();

	glLineWidth(2.0f);
	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vYellow);
	AlarmAreaBatch.Draw();
	m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::DrawAlarmLine(GLEnv &m_env,float pos[4])
{
	// Load as a bunch of line segments
	GLfloat vTracks[50][3];

	int i = 0,j=0;

	float y_data=0.01;

	//front
	vTracks[i][0] =pos[j*2];
	vTracks[i][1] =y_data;
	vTracks[i++][2] = pos[j*2+1];

	j++;

	vTracks[i][0] =pos[j*2];
	vTracks[i][1] =y_data;
	vTracks[i++][2] =pos[j*2+1];

	j++;

	vTracks[i][0] =pos[j*2];
	vTracks[i][1] =y_data;
	vTracks[i++][2] = pos[j*2+1];

	j++;

	vTracks[i][0] =pos[j*2];
	vTracks[i][1] =y_data;
	vTracks[i++][2] = pos[j*2+1];

	j++;

	//back
	j=0;
	vTracks[i][0] =pos[j*2];
	vTracks[i][1] =-y_data;
	vTracks[i++][2] = pos[j*2+1];

	j++;

	vTracks[i][0] =pos[j*2];
	vTracks[i][1] =-y_data;
	vTracks[i++][2] =pos[j*2+1];

	j++;

	vTracks[i][0] =pos[j*2];
	vTracks[i][1] =-y_data;
	vTracks[i++][2] = pos[j*2+1];

	j++;

	vTracks[i][0] =pos[j*2];
	vTracks[i][1] =-y_data;
	vTracks[i++][2] = pos[j*2+1];

	j++;

	AlarmAreaBatch.Begin(GL_LINES, i);
	AlarmAreaBatch.CopyVertexData3f(vTracks);
	AlarmAreaBatch.End();

	m_env.GetmodelViewMatrix()->PushMatrix();

	glLineWidth(2.0f);
	shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vYellow);
	AlarmAreaBatch.Draw();
	m_env.GetmodelViewMatrix()->PopMatrix();
}

void  Render::InitAlarmArea(int positionofalarm[8],int type)
{
	int channel_id=p_dataofalarmarea->GetAlarmIndex();
	p_dataofalarmarea->AppendAlarmArea(positionofalarm);
	int i=0;
	int pos_x[4],pos_y[4],pos_offset[2];
	for(i=0;i<4;i++)
	{
		pos_x[i]=positionofalarm[2*i];
		pos_y[i]=positionofalarm[2*i+1];
	}
	pos_offset[0]=getMinData(pos_x,4);
	pos_offset[1]=getMinData(pos_y,4);
	p_dataofalarmarea->SetOffsetData(channel_id,pos_offset);
#if MVDETECTOR_MODE
	pSingleMvDetector=mvDetector::getInstance();

	std::vector<cv::Point> polyWarnRoi ;
	polyWarnRoi.resize(4);
	polyWarnRoi[0]	= cv::Point(positionofalarm[0]-pos_offset[0]+PIXELS_ADD_ON_ALARM,positionofalarm[1]-pos_offset[1]+PIXELS_ADD_ON_ALARM);
	polyWarnRoi[1]	= cv::Point(positionofalarm[2]-pos_offset[0]+PIXELS_ADD_ON_ALARM,positionofalarm[3]-pos_offset[1]+PIXELS_ADD_ON_ALARM);
	polyWarnRoi[2]	= cv::Point(positionofalarm[4]-pos_offset[0]+PIXELS_ADD_ON_ALARM,positionofalarm[5]-pos_offset[1]+PIXELS_ADD_ON_ALARM);
	polyWarnRoi[3]	= cv::Point(positionofalarm[6]-pos_offset[0]+PIXELS_ADD_ON_ALARM,positionofalarm[7]-pos_offset[1]+PIXELS_ADD_ON_ALARM);
	pSingleMvDetector->setWarningRoi(polyWarnRoi,channel_id);
	WARN_MODE warn_type=WARN_MOVEDETECT_MODE;
	switch(type)
	{
	case 	TYPE_CROSS_BORDER:
		warn_type=WARN_BOUNDARY_MODE;
		break;
	case TYPE_MOVE:
		warn_type=WARN_MOVEDETECT_MODE;
		break;
	case TYPE_INVADE:
		warn_type=WARN_INVADE_MODE;
		break;
	case TYPE_LOST:
		warn_type=WARN_LOST_MODE;
		break;
	default:
		break;
	}
	pSingleMvDetector->setWarnMode(warn_type,channel_id);
#endif
}

void Render::InitAlarmAreaType(int type)
{
	p_dataofalarmarea->SetAlarmType(type);
}

void Render::CancelAlarmArea(){
	mainAlarmTarget.Reset();
	p_dataofalarmarea->Reset();
	p_dataofalarmline->Reset();
}


void Render::RenderTriangleView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h)
{
		glViewport(x,y,w,h);
		glClear(GL_DEPTH_BUFFER_BIT);
		m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
		m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());
		m_env.GetmodelViewMatrix()->PushMatrix();
		m_env.GetmodelViewMatrix()->LoadIdentity();
		M3DMatrix44f mCamera;
		CompassCameraFrame.GetCameraMatrix(mCamera);
		m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
		m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h);//-h
		m_env.GetmodelViewMatrix()->Scale(w*1.0, h*1.0, 1.0f);
		m_env.GetmodelViewMatrix()->PopMatrix();
		m_env.GetmodelViewMatrix()->PopMatrix();
		float recv_focal_length=0;
#if USE_UART
		if(ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.focal_length>=0
				&&ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.focal_length<=999)
		{
			 recv_focal_length=(float)ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.focal_length;
			 recv_focal_length=recv_focal_length/1000.0;
			 recv_focal_length=recv_focal_length*g_windowWidth*646.0/1920.0;
		}
#else
	  recv_focal_length=g_windowWidth*646.0/1920.0;
	#endif
		m_env.GetmodelViewMatrix()->PushMatrix();
		m_env.GetmodelViewMatrix()->Translate(recv_focal_length,0,0);
		glLineWidth(2.0f);
		shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vYellow);
		triangleBatch.Draw();
		m_env.GetmodelViewMatrix()->PopMatrix();
}

void Render::RenderCompassView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h)
{
//RenderRotatingView(x,y,w,h, needSendData);
	glViewport(x,y,w,h);
	glClear(GL_DEPTH_BUFFER_BIT);
	m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->LoadIdentity();

	M3DMatrix44f mCamera;

static bool once=true;
if(once)
{
	once=false;
	//CompassCameraFrame.RotateLocal(-PI/2,0.0,0.0,1.0);
}


		CompassCameraFrame.GetCameraMatrix(mCamera);
		m_env.GetmodelViewMatrix()->PushMatrix(mCamera);
	// move h since the shadow dimension is [-1,1], use h/2 if it is [0,1]
	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h);//-h
	m_env.GetmodelViewMatrix()->Scale(w*1.0, h*1.0, 1.0f);
//	if(displayMode!=ALL_VIEW_FRONT_BACK_ONE_DOUBLE_MODE	)
	//	DrawCompassVideo(true);
	m_env.GetmodelViewMatrix()->PopMatrix();
	m_env.GetmodelViewMatrix()->PopMatrix();

#if USE_UART
	float recv_angle=0;
	if(ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.orientation_angle>=0
			&&ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.orientation_angle<=6000)
	{
		recv_angle=(float)ReadMessage(IPC_MSG_TYPE_40MS_HEARTBEAT).payload.ipc_settings.orientation_angle;
	}
	float needle_angle=recv_angle/6000.0*360.0;
#else
	static float needle_angle=0.0f;
	needle_angle+=1;
#endif

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->Rotate(-needle_angle,0,0,1);
//	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h+50.0);//-h

	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h+180.0);//-h
	//m_env.GetmodelViewMatrix()->Translate(0.0f,0.0f,100.0f);

		glLineWidth(2.0f);
		shaderManager.UseStockShader(GLT_SHADER_FLAT, m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), vYellow);
		NeedleFrameBatch.Draw();
		m_env.GetmodelViewMatrix()->PopMatrix();

//	DrawNeedleonCompass();
}

void Render::DrawCompassVideo(GLEnv &m_env,bool needSendData)
{
	int idx =0;// GetCurrentExtesionVideoId();
#if USE_COMPASS_ICON
	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->Rotate(180.0f, 0.0f, 0.0f, 1.0f);
	m_env.GetmodelViewMatrix()->Rotate(180.0f,0.0f, 1.0f, 0.0f);
	glActiveTexture(GL_IconTextureIDs[idx]);

	if(needSendData){
		PBOExtMgr.sendData(iconTextures[idx], (PFN_PBOFILLBUFFER)captureIconCam,idx);
	}
	else{
		glBindTexture(GL_TEXTURE_2D, iconTextures[idx]);
	}
	shaderManager.UseStockShader(GLT_SHADER_TEXTURE_REPLACE,m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), idx+16);//ICON texture start from 16
	shadowBatch.Draw();
	m_env.GetmodelViewMatrix()->PopMatrix();

#endif
}

void Render::GenerateCompassView()
{
	static M3DVector3f camPanoView;
	static bool once =true;
	if(once){
		once = false;

		CompassCameraFrame.RotateLocalX(0.0);
		CompassCameraFrame.MoveForward(0.0);
		CompassCameraFrame.MoveUp(0.0);
		CompassCameraFrame.MoveRight(0.0);
		//CompassCameraFrame.RotateLocalZ(45.0);
		CompassCameraFrame.GetOrigin(camPanoView);
	}
	CompassCameraFrame.SetOrigin(camPanoView);
}


void Render::GenerateTriangleView()
{
	static M3DVector3f camPanoView;
	static bool once =true;
	if(once){
		once = false;
		TriangleCameraFrame.RotateLocalX(0.0);
		TriangleCameraFrame.MoveForward(0.0);
		TriangleCameraFrame.MoveUp(0.0);
		TriangleCameraFrame.MoveRight(0.0);
		//CompassCameraFrame.RotateLocalZ(45.0);
		TriangleCameraFrame.GetOrigin(camPanoView);
	}
	TriangleCameraFrame.SetOrigin(camPanoView);
}

void Render::set_SightWide(int recvWide)
{
	SightWide=recvWide/6000;
}




void Render::DrawTriangle(GLEnv &m_env)
{
	GLfloat vTracks[50][3];
	GLfloat fixBBDPos[3];
	int i = 0;
	float needle_radius=0;
	float  xmove=0;
	xmove+=SightWide*400;
	float recv_focal_length=0.0;
	xmove+=recv_focal_length;

	vTracks[i][0] =needle_radius+xmove;
	vTracks[i][1] =needle_radius;
	vTracks[i++][2] = 0.0;

	vTracks[i][0] =needle_radius+10+xmove;
	vTracks[i][1] =needle_radius+25;
	vTracks[i++][2] = 0.0;

	vTracks[i][0] =10+needle_radius+xmove;
	vTracks[i][1] =25+needle_radius;
	vTracks[i++][2] = 0.0;

	vTracks[i][0] =20+needle_radius+xmove;
	vTracks[i][1] =needle_radius;
	vTracks[i++][2] = 0.0;

	vTracks[i][0] =20+needle_radius+xmove;
	vTracks[i][1] =needle_radius;
	vTracks[i++][2] = 0.0;

	vTracks[i][0] =needle_radius+xmove;
	vTracks[i][1] =needle_radius;
	vTracks[i++][2] = 0.0;

	triangleBatch.Begin(GL_LINES, i);
	triangleBatch.CopyVertexData3f(vTracks);
	triangleBatch.End();
}

void Render::DrawNeedleonCompass(GLEnv &m_env)
{
	GLfloat vTracks[50][3];
	GLfloat fixBBDPos[3];
	int i = 0;
	float dis=PanelLoader.GetScan_pos();
	float needle_angle=0.0;
	float needle_radius=g_windowWidth/6;
	float recv_angle=0.0;

	vTracks[i][0] =needle_radius*sin(needle_angle*PI/180.0);
	vTracks[i][1] =needle_radius*cos(needle_angle*PI/180.0);
	vTracks[i++][2] = 0.0;

	vTracks[i][0] =1.2*needle_radius*sin((needle_angle+5)*PI/180.0);
	vTracks[i][1] =1.2*needle_radius*cos((needle_angle+5)*PI/180.0);
	vTracks[i++][2] = 0.0;

	vTracks[i][0] =1.2*needle_radius*sin((needle_angle-5)*PI/180.0);
	vTracks[i][1] =1.2*needle_radius*cos((needle_angle-5)*PI/180.0);
	vTracks[i++][2] = 0.0;

	vTracks[i][0] =vTracks[i-3][0];
	vTracks[i][1] =vTracks[i-3][1];
	vTracks[i++][2] =0.0;

	vTracks[i][0] =1.2*needle_radius*sin((needle_angle+5)*PI/180.0);
	vTracks[i][1] =1.2*needle_radius*cos((needle_angle+5)*PI/180.0);
	vTracks[i++][2] = 0.0;

	vTracks[i][0] =1.2*needle_radius*sin((needle_angle-5)*PI/180.0);
	vTracks[i][1] =1.2*needle_radius*cos((needle_angle-5)*PI/180.0);
	vTracks[i++][2] = 0.0;

	NeedleFrameBatch.Begin(GL_LINES, i);
	NeedleFrameBatch.CopyVertexData3f(vTracks);
	NeedleFrameBatch.End();
}


void Render::GenerateVGAView()
{
	static M3DVector3f camPanoView;
	static bool once =true;
	if(once){
		once = false;

		VGACameraFrame.RotateLocalX(0.0);
		VGACameraFrame.MoveForward(0.0);
		VGACameraFrame.MoveUp(0.0);

		VGACameraFrame.RotateLocalY(0.0);
		VGACameraFrame.MoveRight(0.0);
		VGACameraFrame.GetOrigin(camPanoView);
	}
	VGACameraFrame.SetOrigin(camPanoView);
}

void Render::GenerateTargetFrameView()
{
	static M3DVector3f camPanoView;
	static bool once =true;
	if(once){
		once = false;
for(int i=0;i<2;i++)
{
		targetFrame[i].RotateLocalX(0.0);
		targetFrame[i].MoveForward(0.0);
		targetFrame[i].MoveUp(0.0);

		targetFrame[i].RotateLocalY(0.0);
		targetFrame[i].MoveRight(0.0);
		targetFrame[i].GetOrigin(camPanoView);
		targetFrame[i].SetOrigin(camPanoView);
}
	}

}

void Render::GenerateRender2FrontView()
{
	static M3DVector3f camPanoView;
	static bool once =true;
	if(once){
		once = false;

		Render2FrontCameraFrame.RotateLocalX(0.0);
		Render2FrontCameraFrame.MoveForward(0.0);
		Render2FrontCameraFrame.MoveUp(0.0);

		Render2FrontCameraFrame.RotateLocalY(0.0);
		Render2FrontCameraFrame.MoveRight(0.0);
		Render2FrontCameraFrame.GetOrigin(camPanoView);
	}
	Render2FrontCameraFrame.SetOrigin(camPanoView);
}



void Render::GenerateSDIView()
{
	static M3DVector3f camPanoView;
	static bool once =true;
	if(once){
		once = false;

		SDICameraFrame.RotateLocalX(0.0);
		SDICameraFrame.MoveForward(0.0);
		SDICameraFrame.MoveUp(0.0);

		SDICameraFrame.RotateLocalY(0.0);
		SDICameraFrame.MoveRight(0.0);
		SDICameraFrame.GetOrigin(camPanoView);
	}
	SDICameraFrame.SetOrigin(camPanoView);
}

void Render::GenerateChosenView()
{
	static M3DVector3f camPanoView;
	static bool once =true;
	if(once){
		once = false;

		ChosenCameraFrame.RotateLocalX(0.0);
		ChosenCameraFrame.MoveForward(0.0);
		ChosenCameraFrame.MoveUp(0.0);

		ChosenCameraFrame.RotateLocalY(0.0);
		ChosenCameraFrame.MoveRight(0.0);
		ChosenCameraFrame.GetOrigin(camPanoView);
	}
	ChosenCameraFrame.SetOrigin(camPanoView);
}

void Render::GenerateExtentView()
{
	static M3DVector3f camPanoView;
	static bool once =true;
	if(once){
		once = false;

		ExtCameraFrame.RotateLocalX(0.0);
		ExtCameraFrame.MoveForward(0.0);
		ExtCameraFrame.MoveUp(0.0);

		ExtCameraFrame.RotateLocalY(0.0);
		ExtCameraFrame.MoveRight(0.0);
		ExtCameraFrame.GetOrigin(camPanoView);
	}
	ExtCameraFrame.SetOrigin(camPanoView);
}

void Render::InitRuler(GLEnv &m_env)
{
	GLuint texture = 0;
	float panel_width=0,panel_height=0;
	panel_width=PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x();
	panel_height=PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z();
	float scale=1.0,hight_scale=0.1;
	scale=2.0;
	m_env.Getdegreescale45Batch()->Begin(GL_TRIANGLE_FAN, 4, 1);
	m_env.Getdegreescale45Batch()->MultiTexCoord2f(texture, 0.0f, 0.0f);
	//m_env.Getdegreescale45Batch()->Vertex3f(-1.0f,-1.0f, 0.0f );//(-panel_width/scale,0.0,-panel_height*hight_scale);//(-10,0,0);//
	m_env.Getdegreescale45Batch()->Vertex3f(-panel_width/scale,-panel_height/2.0,0.0f);//(-10,0,0);//
	m_env.Getdegreescale45Batch()->MultiTexCoord2f(texture, 0.0f, 1.0f);
	//m_env.Getdegreescale45Batch()->Vertex3f(-1.0f, 1.0f, 0.0f);//(-panel_width/scale,0.0,panel_height*hight_scale);//(-10,10,0);//
	m_env.Getdegreescale45Batch()->Vertex3f(-panel_width/scale,panel_height/2.0,0.0f);
	m_env.Getdegreescale45Batch()->MultiTexCoord2f(texture, 1.0f, 1.0f);
	//m_env.Getdegreescale45Batch()->Vertex3f( 1.0f,1.0f,  0.0f);//( panel_width/scale,0.0,panel_height*hight_scale);//(10,10,0);//
	m_env.Getdegreescale45Batch()->Vertex3f(panel_width/scale,panel_height/2.0,0.0f);
	m_env.Getdegreescale45Batch()->MultiTexCoord2f(texture, 1.0f, 0.0f);
	//m_env.Getdegreescale45Batch()->Vertex3f( 1.0f,-1.0f,  0.0f);	//( panel_width/scale,0.0,-panel_height*hight_scale);//(10,0,0);//
	m_env.Getdegreescale45Batch()->Vertex3f(panel_width/scale,-panel_height/2.0,0.0f);
	m_env.Getdegreescale45Batch()->End();

	scale=2.0;

	m_env.Getdegreescale90Batch()->Begin(GL_TRIANGLE_FAN, 4, 1);
	m_env.Getdegreescale90Batch()->MultiTexCoord2f(texture, 0.0f, 0.0f);
	m_env.Getdegreescale90Batch()->Vertex3f(-panel_width/scale,-panel_height/2.0,0.0f);//(-10,0,0);//
	m_env.Getdegreescale90Batch()->MultiTexCoord2f(texture, 0.0f, 1.0f);
	m_env.Getdegreescale90Batch()->Vertex3f(-panel_width/scale,panel_height/2.0,0.0f);
	m_env.Getdegreescale90Batch()->MultiTexCoord2f(texture, 1.0f, 1.0f);
	m_env.Getdegreescale90Batch()->Vertex3f(panel_width/scale,panel_height/2.0,0.0f);
	m_env.Getdegreescale90Batch()->MultiTexCoord2f(texture, 1.0f, 0.0f);
	m_env.Getdegreescale90Batch()->Vertex3f(panel_width/scale,-panel_height/2.0,0.0f);
	m_env.Getdegreescale90Batch()->End();

	scale=2.0;
	m_env.Getdegreescale180Batch()->Begin(GL_TRIANGLE_FAN, 4, 1);
	m_env.Getdegreescale180Batch()->MultiTexCoord2f(texture, 0.0f, 0.0f);
	m_env.Getdegreescale180Batch()->Vertex3f(-panel_width/scale,-panel_height/2.0,0.0f);//(-10,0,0);//
	m_env.Getdegreescale180Batch()->MultiTexCoord2f(texture, 0.0f, 1.0f);
	m_env.Getdegreescale180Batch()->Vertex3f(-panel_width/scale,panel_height/2.0,0.0f);
	m_env.Getdegreescale180Batch()->MultiTexCoord2f(texture, 1.0f, 1.0f);
	m_env.Getdegreescale180Batch()->Vertex3f(panel_width/scale,panel_height/2.0,0.0f);
	m_env.Getdegreescale180Batch()->MultiTexCoord2f(texture, 1.0f, 0.0f);
	m_env.Getdegreescale180Batch()->Vertex3f(panel_width/scale,-panel_height/2.0,0.0f);
	m_env.Getdegreescale180Batch()->End();
}

void Render::RenderRulerView(GLEnv &m_env,GLint x, GLint y, GLint w, GLint h,int type)
{
	glViewport(x,y,w,h);
	glClear(GL_DEPTH_BUFFER_BIT);
	m_env.GetviewFrustum()->SetPerspective(90.0f, float(w) / float(h), 1.0f, 4000.0f);
	m_env.GetprojectionMatrix()->LoadMatrix(m_env.GetviewFrustum()->GetProjectionMatrix());

	m_env.GetmodelViewMatrix()->PushMatrix();
	m_env.GetmodelViewMatrix()->LoadIdentity();

	M3DMatrix44f mCamera;


		static bool once[3];
		for(int i=0;i<3;i++)
		{
			once[i]=true;
		}
		CompassCameraFrame.GetCameraMatrix(mCamera);
		m_env.GetmodelViewMatrix()->PushMatrix(mCamera);

	// move h since the shadow dimension is [-1,1], use h/2 if it is [0,1]
	m_env.GetmodelViewMatrix()->Translate(0.0f, 0.0f, -h);//-h
	m_env.GetmodelViewMatrix()->Scale(w*1.39/(13.0*2.0), h/6.0, 1.0f);

	DrawRulerVideo(m_env,once[type],type);
	if(once[type])
	{
		once[type]=false;

	}


	m_env.GetmodelViewMatrix()->PopMatrix();
	m_env.GetmodelViewMatrix()->PopMatrix();
	//DrawNeedleonCompass();
}

void Render::DrawRulerVideo(GLEnv &m_env,bool needSendData,int type)
{
#if 1
	int idx =0;// GetCurrentExtesionVideoId();
#if USE_ICON
	m_env.GetmodelViewMatrix()->PushMatrix();
//	m_env.GetmodelViewMatrix()->Rotate(180.0, 1.0f, 0.0f, 0.0f);
//	m_env.GetmodelViewMatrix()->Translate(0.0,0.0,-6.0);
	m_env.GetmodelViewMatrix()->Rotate(180.0f, 0.0f, 0.0f, 1.0f);
	m_env.GetmodelViewMatrix()->Rotate(180.0f,0.0f, 1.0f, 0.0f);
	switch(type)
	{
	case RULER_45:
		glActiveTexture(GL_IconRuler45TextureIDs[idx]);

		if(needSendData){
					m_env.Getp_PBOExtMgr()->sendData(m_env,iconRuler45Textures[idx], (PFN_PBOFILLBUFFER)captureRuler45Cam,ICON_45DEGREESCALE+MAGICAL_NUM);
		}
		else{
			glBindTexture(GL_TEXTURE_2D, iconRuler45Textures[idx]);
		}
		break;
	case RULER_90:
		glActiveTexture(GL_IconRuler90TextureIDs[idx]);

		if(needSendData){
			m_env.Getp_PBOExtMgr()->sendData(m_env,iconRuler90Textures[idx], (PFN_PBOFILLBUFFER)captureRuler90Cam,ICON_90DEGREESCALE+MAGICAL_NUM);
		}
		else{
			glBindTexture(GL_TEXTURE_2D, iconRuler90Textures[idx]);
		}
		break;
	case RULER_180:
		glActiveTexture(GL_IconRuler180TextureIDs[idx]);

		if(needSendData){
			m_env.Getp_PBOExtMgr()->sendData(m_env,iconRuler180Textures[idx], (PFN_PBOFILLBUFFER)captureRuler180Cam,ICON_180DEGREESCALE+MAGICAL_NUM);
		}
		else{
			glBindTexture(GL_TEXTURE_2D, iconRuler180Textures[idx]);
		}
		break;
	}
#if USE_CPU
	shaderManager.UseStockShader(GLT_SHADER_ORI,m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), idx+17+type);//ICON texture start from 16
#else
	shaderManager.UseStockShader(GLT_SHADER_TEXTURE_REPLACE,m_env.GettransformPipeline()->GetModelViewProjectionMatrix(), idx+17+type);//ICON texture start from 16
#endif
	switch(type)
	{
	case RULER_45:
		m_env.Getdegreescale45Batch()->Draw();
		break;
	case RULER_90:
		m_env.Getdegreescale90Batch()->Draw();
		break;
	case RULER_180:
		m_env.Getdegreescale180Batch()->Draw();
		break;
	}

	m_env.GetmodelViewMatrix()->PopMatrix();

#endif
#endif
}


void Render::ReadPanoScaleArrayData(char * filename)
{
	FILE * fp;
	int i=0;
	float read_data=0.0;
	fp=fopen(filename,"r");
	for(i=0;i<CAM_COUNT;i++)
	{
		channel_left_scale[i]=1.0;
		channel_right_scale[i]=1.0;
	}

	if(fp!=NULL)
	{
		for(i=0;i<CAM_COUNT;i++)
		{
			fscanf(fp,"%f\n",&channel_left_scale[i]);
			printf("%f\n",channel_left_scale[i]);
		}
		for(i=0;i<CAM_COUNT;i++)
		{
			fscanf(fp,"%f\n",&channel_right_scale[i]);
			printf("%f\n",channel_right_scale[i]);
		}
		for(i=0;i<CAM_COUNT;i++)
		{
			fscanf(fp,"%f\n",&move_hor[i]);
			printf("%f\n",move_hor[i]);
		}
		fclose(fp);
	}
	else
	{
		WritePanoScaleArrayData(filename,channel_left_scale,channel_right_scale,move_hor);
	}
}

void Render::InitPanoScaleArrayData()
{
	int i=0;
	for(i=0;i<CAM_COUNT;i++)
	{
		channel_left_scale[i]=1.0;
		channel_right_scale[i]=1.0;
	}
	ReadPanoScaleArrayData(PANO_SCALE_ARRAY_FILE);
}


void Render::WritePanoScaleArrayData(char * filename,float * arraydata_left,float * arraydata_right,float * arraydata_level)
{
	FILE * fp;
	int i=0;
	char data[20];
	fp=fopen(filename,"w");
	for(i=0;i<CAM_COUNT;i++)
	{
		sprintf(data,"%f\n",arraydata_left[i]);
		fwrite(data,strlen(data),1,fp);
	}
	for(i=0;i<CAM_COUNT;i++)
	{
		sprintf(data,"%f\n",arraydata_right[i]);
		fwrite(data,strlen(data),1,fp);
	}
	for(i=0;i<CAM_COUNT;i++)
	{
		sprintf(data,"%f\n",arraydata_level[i]);
		fwrite(data,strlen(data),1,fp);
	}
	fclose(fp);
}
//================end of embedded fixed billboard implementation========

void* getDefaultShaderMgr()
{
	return render.getShaderManager();
}

void * getDefaultTransformPipeline(GLEnv &m_env)
{
	//GLEnv &m_env =env1;
	return m_env.GettransformPipeline();
//	return render.getTransformPipeline();
}

void set10camsOverlapArea(int count,int & direction,bool &AppOverlap)
{
	int coutOfeachCam=48;//480/10
	int halfOfcam0=4;//24;//48/2, cus petal 0 is cut and put at each side of the panel;
	int overlapcount=2;
	int temp_x=count%480;
	int array[10]={0};
#if 1
	int delta=48;
	if(temp_x<2)
	{
		direction=9;
		AppOverlap=true;
	}
	else if(temp_x<delta)
			{
				direction=0;
			}
			else if(temp_x<delta+2)
			{
				direction=0;
				AppOverlap=true;
			}
			else if(temp_x<delta+48)
			{
				direction=1;
			}
			else if(temp_x<delta+48+2)
			{
				direction=1;
				AppOverlap=true;
			}
			else if(temp_x<delta+48*2)
				{
					direction=2;
				}
				else if(temp_x<delta+48*2+2)
				{
					direction=2;
					AppOverlap=true;
				}
				else if(temp_x<delta+48*3)
					{
						direction=3;
					}
					else if(temp_x<delta+48*3+2)
					{
						direction=3;
						AppOverlap=true;
					}
					else if(temp_x<delta+48*4)
						{
							direction=4;
						}
						else if(temp_x<delta+48*4+2)
						{
							direction=4;
							AppOverlap=true;
						}
						else if(temp_x<delta+48*5)
							{
								direction=5;
							}
							else if(temp_x<delta+48*5+2)
							{
								direction=5;
								AppOverlap=true;
							}

							else if(temp_x<delta+48*6)
								{
									direction=6;
								}
								else if(temp_x<delta+48*6+2)
								{
									direction=6;
									AppOverlap=true;
								}
								else if(temp_x<delta+48*7)
									{
										direction=7;
									}
									else if(temp_x<delta+48*7+2)
									{
										direction=7;
										AppOverlap=true;
									}
									else if(temp_x<delta+48*8)
										{
											direction=8;
										}
										else if(temp_x<delta+48*8+2)
										{
											direction=8;
											AppOverlap=true;
										}
										else if(temp_x<delta+48*9)
											{
												direction=9;
											}
#else
	int delta=24;
			if(temp_x<delta)
			{
				direction=0;
			}
			else if(temp_x<delta+2)
			{
				direction=0;
				AppOverlap=true;
			}
			else if(temp_x<delta+48)
			{
				direction=1;
			}
			else if(temp_x<delta+48+2)
			{
				direction=1;
				AppOverlap=true;
			}
			else if(temp_x<delta+48*2)
				{
					direction=2;
				}
				else if(temp_x<delta+48*2+2)
				{
					direction=2;
					AppOverlap=true;
				}
				else if(temp_x<delta+48*3)
					{
						direction=3;
					}
					else if(temp_x<delta+48*3+2)
					{
						direction=3;
						AppOverlap=true;
					}
					else if(temp_x<delta+48*4)
						{
							direction=4;
						}
						else if(temp_x<delta+48*4+2)
						{
							direction=4;
							AppOverlap=true;
						}
						else if(temp_x<delta+48*5)
							{
								direction=5;
							}
							else if(temp_x<delta+48*5+2)
							{
								direction=5;
								AppOverlap=true;
							}

							else if(temp_x<delta+48*6)
								{
									direction=6;
								}
								else if(temp_x<delta+48*6+2)
								{
									direction=6;
									AppOverlap=true;
								}
								else if(temp_x<delta+48*7)
									{
										direction=7;
									}
									else if(temp_x<delta+48*7+2)
									{
										direction=7;
										AppOverlap=true;
									}
									else if(temp_x<delta+48*8)
										{
											direction=8;
										}
										else if(temp_x<delta+48*8+2)
										{
											direction=8;
											AppOverlap=true;
										}
										else if(temp_x<delta+48*9)
											{
												direction=9;
											}
										else if(temp_x<delta+48*9+2)
										{
											direction=9;
											AppOverlap=true;
										}
#endif



}

void setOverlapArea(int count,int & direction,bool &AppOverlap)
{
	int set_corner_angle[CAM_COUNT*2];
	int y=0;
	int temp_x=0;
	//temp_x=count%512;
	temp_x=count%480;
	for(y=0;y<CAM_COUNT;y++)
	{
		set_corner_angle[2*y]=240*SET_POINT_SCALE/(CAM_COUNT*2)+240*y*SET_POINT_SCALE/(CAM_COUNT);
	//	set_corner_angle[2*y]=256*SET_POINT_SCALE/(CAM_COUNT*2)+256*y*SET_POINT_SCALE/(CAM_COUNT);
		if(set_corner_angle[2*y]%2!=0)
		{
			set_corner_angle[2*y]=set_corner_angle[2*y]-1;
		}
		set_corner_angle[2*y+1]=set_corner_angle[2*y]+1;
	}

	if(temp_x<set_corner_angle[0])
	{
		direction=0;
	}
	else if(temp_x>=set_corner_angle[0]&&temp_x<=set_corner_angle[CAM_COUNT*2-2])
	{
		for(y=0;y<(CAM_COUNT);y++)
		{
			if(temp_x>=set_corner_angle[2*y]&&temp_x<=set_corner_angle[2*y+1])
			{
				direction=y;
				AppOverlap=true;
			}
			else if(temp_x>(set_corner_angle[2*y+1])&&temp_x<set_corner_angle[2*y+2])
			{
				direction=y+1;
			}
		}
	}
	else if(temp_x>=(set_corner_angle[CAM_COUNT*2-2])&&temp_x<=(set_corner_angle[CAM_COUNT*2-1]))
	{
		direction=CAM_COUNT-1;
		AppOverlap=true;
	}
	else
	{
		direction=0;
	}

/*
	int delta_count=2;
	if(temp_x<20)
	{
		direction=0;
	}
	else if(temp_x<22)
	{
		direction=0;
		AppOverlap=true;
	}
	else if(temp_x<(20+42+delta_count))//42
	{
		direction=1;
	}
	else if(temp_x<(20+42+2+delta_count))//44
	{
		direction=1;
		AppOverlap=true;
	}
	else if(temp_x<(20+42*2+delta_count))//84
	{
		direction=2;
	}
	else if(temp_x<(20+42*2+2+delta_count))//86
	{
		direction=2;
		AppOverlap=true;
	}
	else if(temp_x<(20+42*3+2*1))//128
	{
		direction=3;
	}
	else if(temp_x<(20+42*3+2*1+2))//130
	{
		direction=3;
		AppOverlap=true;
	}
	else if(temp_x<(20+42*4+2*1+delta_count))//170
	{
		direction=4;
	}
	else if(temp_x<(20+42*4+2*1+2+delta_count))//172
	{
		direction=4;
		AppOverlap=true;
	}
	else if(temp_x<(20+42*5+2*1+delta_count))//212
	{
		direction=5;
	}
	else if(temp_x<(20+42*5+2*1+2+delta_count))//214
	{
		direction=5;
		AppOverlap=true;
	}
	else if(temp_x<(20+42*6+2*2))//256
	{
		direction=6;
	}
	else if(temp_x<(20+42*6+2*2+2))//258
	{
		direction=6;
		AppOverlap=true;
	}
	else if(temp_x<(20+42*7+2*2+delta_count))//298
	{
		direction=7;
	}
	else if(temp_x<(20+42*7+2*2+2+delta_count))//300
	{
		direction=7;
		AppOverlap=true;
	}
	else if(temp_x<(20+42*8+2*2+delta_count+2))//340
	{
		direction=8;
	}
	else if(temp_x<(20+42*8+2*2+2+delta_count+2))//342
	{
		direction=8;
		AppOverlap=true;
	}
	else if(temp_x<(20+42*9+2*3+delta_count))//384
	{
		direction=9;
	}
	else if(temp_x<(20+42*9+2*3+2+delta_count))//386
	{
		direction=9;
		AppOverlap=true;
	}
	else if(temp_x<(20+42*10+2*3+delta_count))//426
	{
		direction=10;
	}
	else if(temp_x<(20+42*10+2*3+2+delta_count))//428
	{
		direction=10;
		AppOverlap=true;
	}

	else if(temp_x<(20+42*11+2*3+delta_count+2))//468
		{
			direction=11;
		}
		else if(temp_x<(20+42*11+2*3+2+delta_count+2))//470
		{
			direction=11;
			AppOverlap=true;
		}
		else
			direction=0;*/
}

void SendBackXY(int *Pos)
{
#if USE_UART
IPC_msg   ipc_msg;
	ipc_msg.msg_type=IPC_MSG_TYPE_COORDINATES_FEEDBACK;
	ipc_msg.payload.ipc_coordinates.coordinates_orientation=Pos[0];
	ipc_msg.payload.ipc_coordinates.acoordinates_ver=Pos[1];
    WriteMessage(&ipc_msg);
#endif
    cout<<"X="<<Pos[0]<<"  Y="<<Pos[1]<<endl;
}

void Render::sendTrackSpeed(int w,int h)
{
#if USE_UART
#if TRACK_MODE
IPC_msg   ipc_msg;
static float lastrecvX=0,lastrecvY=0;
static struct timeval mt;
static bool Once=true;
float tempX=0,tempY=0;
float distance=0;
float deltatime=0;
struct timeval tv2;
memset(&tv2,0,sizeof(timeval));
gettimeofday(&tv2,NULL);
deltatime= (1000000.0*(tv2.tv_sec - mt.tv_sec) +(tv2.tv_usec - mt.tv_usec));
if(deltatime>1000000)
{
	tempX=track_pos[0]+track_pos[2];//接收值
	tempY=track_pos[1]+track_pos[3];
	distance=sqrt((tempX-lastrecvX)*(tempX-lastrecvX)+
			(tempY-lastrecvY)*(tempY-lastrecvY));
	lastrecvX=tempX;
	lastrecvY=tempY;
	mt=tv2;
	ipc_msg.msg_type=IPC_MSG_TYPE_AUTOTRACKING;
	TrackSpeed=1000000.0*distance/deltatime;
	//printf("track_pos---0:=%f\t 1=%f\t 2=%f\t 3=%f\n",track_pos[0],track_pos[1],track_pos[2],track_pos[3]);
	//cout<<"Speed 1秒"<<TrackSpeed<<endl<<endl;
	if(Once)
	{
		Once=false;
	}
	else
	{
		ipc_msg.msg_type=IPC_MSG_TYPE_AUTOTRACKING;
		ipc_msg.payload.ipc_tracking.palstance=TrackSpeed;
	    WriteMessage(&ipc_msg);
	}
}
//#endif
#endif
#endif
}

void Render::initLabelBatch()
{
    M3DVector3f vVerts[100];       
	int start_x=20;
	int start_y=300;
	int hor_dis=50;
	int ver_dis=30;
	int two_dis=500;
	int i=0,j=0;

    	GLfloat r = 30.0f;
    	GLfloat angle = 0.0f;   // Another looping variable
	int nVerts = 0;

	FILE * fp;
	fp=fopen("label_point_pos.txt","r");
	if(fp!=NULL)
	{
		fscanf(fp,"%d\n",&start_x);
		fscanf(fp,"%d\n",&start_y);
		fscanf(fp,"%d\n",&hor_dis);
		fscanf(fp,"%d\n",&ver_dis);
		fscanf(fp,"%d\n",&two_dis);
		fscanf(fp,"%f\n",&r);
		fclose(fp);
	}
	
	for(i=0;i<6;i++)
	{
		for(j=0;j<3;j++)
		{
			
			    nVerts = 0;
			    vVerts[nVerts][0] = start_x+j*hor_dis-1000;
			    vVerts[nVerts][1] = start_y-i*ver_dis+50;
			    vVerts[nVerts][2] = 0.0f;
				nVerts++;
				for(angle = 0; angle < 2.0f * 3.141592f; angle += 0.2f) {
				   
				   vVerts[nVerts][0] = start_x+j*hor_dis -1000+ float(cos(angle)) * r;
				   vVerts[nVerts][1] = start_y-i*ver_dis+50 + float(sin(angle)) * r;
				   vVerts[nVerts][2] = 0.0f;
					nVerts++;
				   }
			    
			   
			    vVerts[nVerts][0] = start_x+j*hor_dis -1000+ r;
			    vVerts[nVerts][1] = start_y-i*ver_dis+50;
			    vVerts[nVerts][2] = 0.0f;
				nVerts++;
				array_round_point[i*3+j].Begin(GL_TRIANGLE_FAN, nVerts);
			    array_round_point[i*3+j].CopyVertexData3f(vVerts);
			    array_round_point[i*3+j].End(); 
			
		}
	}


	for(i=0;i<6;i++)
	{
		for(j=0;j<3;j++)
		{
			
			    nVerts = 0;
			    vVerts[nVerts][0] = start_x+j*hor_dis+two_dis;
			    vVerts[nVerts][1] = start_y-i*ver_dis+ver_dis;
			    vVerts[nVerts][2] = 0.0f;
				nVerts++;
				for(angle = 0; angle < 2.0f * 3.141592f; angle += 0.2f) {
				   
				   vVerts[nVerts][0] = start_x+j*hor_dis+two_dis + float(cos(angle)) * r;
				   vVerts[nVerts][1] = start_y-i*ver_dis+ver_dis + float(sin(angle)) * r;
				   vVerts[nVerts][2] = 0.0f;
					nVerts++;
				   }
			    
			   
			    vVerts[nVerts][0] = start_x+j*hor_dis+two_dis + r;
			    vVerts[nVerts][1] = start_y-i*ver_dis+ver_dis;
			    vVerts[nVerts][2] = 0.0f;
				nVerts++;
				array_round_point[i*3+j+18].Begin(GL_TRIANGLE_FAN, nVerts);
			    array_round_point[i*3+j+18].CopyVertexData3f(vVerts);
			    array_round_point[i*3+j+18].End(); 
			
		}
	}	
}




void math_scale_pos(int direction,int count,int & scale_count,int & this_channel_max_count)
{
	//微调使用
	int y=0;
	//int set_corner_angle[CAM_COUNT*2]={24,25,76,77,128,129,180,181,232,233,284,285,336,337,386,387,436,437,486,487};
	//={20,21,64,65,106,107,148,149,192,193,234,235,276,277,320,321,364,365,406,408,448,449,490,491};
	int set_corner_angle[CAM_COUNT*2]={0};
	int temp_count=0;
//	temp_count=count%512;
	temp_count=count%480;

	for(y=0;y<CAM_COUNT;y++)
	{
		set_corner_angle[2*y]=240*SET_POINT_SCALE/(CAM_COUNT*2)+240*y*SET_POINT_SCALE/(CAM_COUNT);

	//	set_corner_angle[2*y]=256*SET_POINT_SCALE/(CAM_COUNT*2)+256*y*SET_POINT_SCALE/(CAM_COUNT);
		if(set_corner_angle[2*y]%2!=0)
		{
			set_corner_angle[2*y]=set_corner_angle[2*y]-1;
		}
		set_corner_angle[2*y+1]=set_corner_angle[2*y]+1;
	}

	if(direction>0)
	{
		scale_count=temp_count-set_corner_angle[2*(direction-1)];
		this_channel_max_count=set_corner_angle[2*direction]-set_corner_angle[2*(direction-1)];
	}
	else
	{
		if(temp_count<set_corner_angle[0])
		{
		//	scale_count=temp_count+512-set_corner_angle[2*(CAM_COUNT-1)];
			scale_count=temp_count+480-set_corner_angle[2*(CAM_COUNT-1)];
		}
		else
		{
			scale_count=temp_count-set_corner_angle[2*(CAM_COUNT-1)];
		}
		//this_channel_max_count=512-set_corner_angle[2*(CAM_COUNT-1)]+set_corner_angle[0];
		this_channel_max_count=480-set_corner_angle[2*(CAM_COUNT-1)]+set_corner_angle[0];

	}

}
