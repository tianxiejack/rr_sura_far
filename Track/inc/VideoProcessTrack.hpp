/*
 * VideoProcess.hpp
 *
 *  Created on:
 *      Author: sh
 */

#ifndef VIDEOPROCESS_HPP_
#define VIDEOPROCESS_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#include "osa.h"
#include "osa_thr.h"
#include "osa_sem.h"
#include "osa_mutex.h"

#include "UtcTrack.h"


//for zodiac
enum TrackVideoChid{
	sdi_Track=0,
	vga_Track,
	pal_Track,
	pal2_Track};
//for zodiac


#define TRK_LOST_TIME 		(5000)//ms


#define DS_DC_CNT		(4)
typedef struct _main_thr_obj_cxt{
	bool bTrack;
	bool bMtd;
	bool bBlobDetect;
	int chId;
	int iTrackStat;
	//Mat frame;
}MAIN_ProcThrObj_cxt;
typedef struct _main_thr_obj{
	MAIN_ProcThrObj_cxt cxt[2];
	OSA_ThrHndl		thrHandleProc;
	OSA_SemHndl	 procNotifySem;
	int pp;
	bool bFirst;
	volatile bool	exitProcThread;
	bool						initFlag;
	void 						*pParent;
}MAIN_ProcThrObj;

typedef void (*TRKSUCCESSCB)(UTC_RECT_float m_rcTrack);  //return track param
typedef void (*TRKFAILEDCB)();   

class CVideoProcess
{
	
public:
	MAIN_ProcThrObj	mainProcThrObj;
	Mat mainFrame[2];

	static OSA_MutexHndl SingleTon_m_mutex;
	static CVideoProcess* getInstance()
	{
		if(instance==NULL)
		{
			OSA_mutexLock(&SingleTon_m_mutex);
		
			if(instance==NULL)
			{
				instance=new CVideoProcess();
			}
			OSA_mutexUnlock(&SingleTon_m_mutex);
		}
		return instance;
	}
	~CVideoProcess();
	int creat();
	int destroy();
	int stop();
	int process_frame(int chId,Mat frame);
	void trackData_printf(int type, UTC_RECT_float rcResult);
	int enableTrackAcq(int x, int y,int width, int height,int chId);
	void disableTrack(int chId);
	int SwitchTrackChId(int chId);    //when switch sdi-vga | vga-sdi |sdi-pal |pal-sdi |   ... exec this
	bool isTracking();

	void registerCB(TRKSUCCESSCB trkSuccessFunc,TRKFAILEDCB trkFailedFunc)
	{
		 this->trkSuccessFunc=trkSuccessFunc;
		 this->trkFailedFunc=trkFailedFunc;
	}
protected:

	typedef enum{
		DS_DC_CLEAR = 0,
		DS_DC_ALG,
		DS_DC_OSD,
	}DS_DC_IDX;
	Mat m_dc[DS_DC_CNT];
	UTCTRACK_HANDLE m_track;
//	ALGMTD_HANDLE m_mtd[MAX_CHAN];
//	CMultitarget m_mtd_detect;
//	CMSTracker m_mtd_mstrk;
	static bool m_bTrack;
	static bool m_bMtd;
	static int m_iTrackStat;
	static int m_iTrackLostCnt;

	
	int process_track(int trackStatus, Mat frame_gray, Mat frame_dis, UTC_RECT_float &rcResult);
	int ReAcqTarget();

	//static CVideoProcess *pThis;
	static int callback_process(void *handle, int chId, Mat frame);
	void extractYUYV2Gray2(Mat src, Mat dst);
	 void extractUYVY2Gray2(Mat src, Mat dst);
	 void main_proc_func();

protected:
	
	OSA_MutexHndl m_mutex;
	
	int MAIN_threadCreate(void);
	int MAIN_threadDestroy(void);
	static void *mainProcTsk(void *context)
	{
		OSA_printf("%s:%d context=%p\n", __func__, __LINE__, context);
		MAIN_ProcThrObj  * pObj= (MAIN_ProcThrObj*) context;
		CVideoProcess *ctxHdl = (CVideoProcess *) pObj->pParent;

		ctxHdl->main_proc_func();
		return NULL;
	}

	static int64 tstart;
	

	TRKSUCCESSCB trkSuccessFunc;   // return to show
	TRKFAILEDCB trkFailedFunc;   //return to show

protected:
	UTC_RECT_float m_rcTrack, m_rcAcq;
	int 	m_trkChId;
	int	m_intervalFrame;
	//int	m_ImageAxisx;
	//int	m_ImageAxisy;
	int m_ImageAxisxSdi;
	int m_ImageAxisySdi;

	int m_ImageAxisxVga;
	int m_ImageAxisyVga;

	int m_ImageAxisxPal;
	int m_ImageAxisyPal;


	int 	m_mtdChId;


	int	adaptiveThred;

	int	m_curChId;
	int	m_curSubChId;
private:
	CVideoProcess();
	
	static CVideoProcess * instance;

};



#endif /* VIDEOPROCESS_HPP_ */
