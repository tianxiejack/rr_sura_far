#if TRACK_MODE
#include "VideoProcessTrack.hpp"
//#include "arm_neon.h"







//CVideoProcess * CVideoProcess::pThis = NULL;
//bool CVideoProcess::m_bMtd = false;
bool CVideoProcess::m_bTrack = false;
int CVideoProcess::m_iTrackStat = 0;
int CVideoProcess::m_iTrackLostCnt = 0;
CVideoProcess * CVideoProcess::instance=NULL;
OSA_MutexHndl  CVideoProcess::SingleTon_m_mutex;



//TARGETBOX mBox[MAX_TARGET_NUMBER];
//vector<Rect> Box(MAX_TARGET_NUMBER);
void CVideoProcess::extractYUYV2Gray2(Mat src, Mat dst)
{
	int ImgHeight, ImgWidth,ImgStride, stride16x8;

	ImgWidth = src.cols;
	ImgHeight = src.rows;
	ImgStride = ImgWidth*2;
	stride16x8 = ImgStride/16;

	OSA_assert((ImgStride&15)==0);
#pragma omp parallel for
	for(int y = 0; y < ImgHeight; y++)
	{
		#if !NO_ARM_NEON
		uint8x8_t  * __restrict__ pDst8x8_t;
		uint8_t * __restrict__ pSrc8_t;
		pSrc8_t = (uint8_t*)(src.data+ ImgStride*y);
		pDst8x8_t = (uint8x8_t*)(dst.data+ ImgWidth*y);
		for(int x=0; x<stride16x8; x++)
		{
			uint8x8x2_t d;
			d = vld2_u8((uint8_t*)(pSrc8_t+16*x));
			pDst8x8_t[x] = d.val[0];
		}
		#endif
	}
}
int CVideoProcess::SwitchTrackChId(int chId)
{
	if(chId!=m_curChId)
	{
		m_curChId=chId;
		return true;
	}
	return false;
}
int CVideoProcess::enableTrackAcq(int x, int y,int width, int height,int chId)
{
	SwitchTrackChId(chId);
	if(chId==m_curChId)
	{
		UTC_RECT_float rc;
		rc.x=x;
		rc.y=y;
		rc.width=width;
		rc.height=height;
		OSA_mutexLock(&m_mutex);
		m_rcAcq = rc;
		m_rcTrack = rc;
		if(m_bTrack)	// already in trk
		{
			m_bTrack = 1;
			m_intervalFrame = 1;
			m_iTrackLostCnt = 0;
		}
		else
		{
			m_intervalFrame = 1;
			m_iTrackStat = 0;
			mainProcThrObj.bFirst = true;
			m_bTrack = 1 ;
			m_iTrackLostCnt = 0;
		}
		OSA_mutexUnlock(&m_mutex);
	}
}
void CVideoProcess::disableTrack(int chId)
{
	OSA_mutexLock(&m_mutex);
	if(chId==m_curChId)
	{
		m_bTrack = 0;
		m_intervalFrame = 0;
		m_iTrackLostCnt = 0;
	}
	OSA_mutexUnlock(&m_mutex);
}
bool CVideoProcess::isTracking()
{
	if(m_bTrack)
		return true;
	else 
		return false;
}	
int CVideoProcess::MAIN_threadCreate(void)
{
	int iRet = OSA_SOK;
	iRet = OSA_semCreate(&mainProcThrObj.procNotifySem ,1,0) ;
	OSA_assert(iRet == OSA_SOK);


	mainProcThrObj.exitProcThread = false;

	mainProcThrObj.initFlag = true;

	mainProcThrObj.pParent = (void*)this;

	iRet = OSA_thrCreate(&mainProcThrObj.thrHandleProc, mainProcTsk, 0, 0, &mainProcThrObj);

	return iRet;
}
void  CVideoProcess::trackData_printf( int type,UTC_RECT_float rcResult)
{
	if(type==0)
		printf("i get acq data\n");
	else
		printf("i get trk data\n");
	printf("x=%.4f,y=%.4f,width=%.4f,height=%.4f\n",rcResult.x,rcResult.y,rcResult.width,rcResult.height);
}

	
void CVideoProcess::main_proc_func()
{
	while(mainProcThrObj.exitProcThread ==  false)
	{	
		OSA_semWait(&mainProcThrObj.procNotifySem, OSA_TIMEOUT_FOREVER);
		Mat frame = mainFrame[mainProcThrObj.pp^1];
		bool bTrack = mainProcThrObj.cxt[mainProcThrObj.pp^1].bTrack;
		int chId = mainProcThrObj.cxt[mainProcThrObj.pp^1].chId;
		//bool bMtd = mainProcThrObj.cxt[mainProcThrObj.pp^1].bMtd;
		int iTrackStat = mainProcThrObj.cxt[mainProcThrObj.pp^1].iTrackStat;
		int channel = frame.channels();
		Mat frame_gray;
		mainProcThrObj.pp ^=1;
		if(!m_bTrack){
			continue;
		}
		if(chId != m_curChId)
			continue;

		frame_gray = Mat(frame.rows, frame.cols, CV_8UC1);
		if(channel == 2)
		{
			if(chId==0||chId==1)//sdi
			{
				extractYUYV2Gray2(frame, frame_gray);
			}
			else   //vga and pal
			{
				extractUYVY2Gray2(frame, frame_gray);
				
			}
		}else{
			memcpy(frame_gray.data, frame.data, frame.cols * frame.rows*channel*sizeof(unsigned char));
		}	
		if(bTrack)
		{
			iTrackStat = ReAcqTarget();
			int iret = process_track(iTrackStat, frame_gray, m_dc[DS_DC_ALG], m_rcTrack);
			if(m_iTrackStat != iret)
			{
			//	OSA_printf("ALL-Trk: m_iTrackStat = %d iret=%d\n", m_iTrackStat, iret);
				m_iTrackStat = iret;   
			}
			if(m_iTrackStat == 1)
				m_iTrackLostCnt = 0;
			else
				m_iTrackLostCnt++;
			if(m_iTrackStat ==2)
			{		
				if(m_iTrackLostCnt < (TRK_LOST_TIME/20))
				{
					trkSuccessFunc(m_rcTrack);
				}
				else
				{
					disableTrack(m_curChId);
					trkFailedFunc();				
				}
			
			}
			else
				trkSuccessFunc(m_rcTrack);
		}
		/*
		if(bMtd)
		{
//			tstart = getTickCount();
			if(m_mtdChId != m_curChId)
			{
				m_mtdChId = m_curChId;
				m_mtd_mstrk.ClearAllMStg(mBox, true);
				framecount=0;
				OSA_printf("ALL-MTD: mtd channel change to %d\n", m_mtdChId);
			}
			
			if(m_mtd[m_curChId]->state == 0)
			{
				if(m_mtd_detect.initMTD(frame_gray,m_curChId)!= -1)
				{
					m_mtd[m_curChId]->state = 1;
					//OSA_printf(" %d:%s chId %d start mtd\n", OSA_getCurTimeInMsec(),__func__, chId);
				}
			}

			int starttime=OSA_getCurTimeInMsec();
			m_mtd_mstrk.Process(frame_gray, frame_gray.cols, frame_gray.rows, framecount, mBox,m_curChId);
			//OSA_printf("ALL-MTD: timedelay=%d  framecount=%d\n",OSA_getCurTimeInMsec()-starttime,framecount);

			for(int i=0;i<MAX_TARGET_NUMBER;i++)
			{
				m_mtd[m_curChId]->tg[i].cur_x=mBox[i].Box.x+mBox[i].Box.width/2;
				m_mtd[m_curChId]->tg[i].cur_y=mBox[i].Box.y+mBox[i].Box.height/2;
				m_mtd[m_curChId]->tg[i].valid=mBox[i].valid;
				m_mtd[m_curChId]->tg[i].subscript = i+1;	// default id
				//OSA_printf("ALL-MTD: time  valid=%d x=%f y=%f\n",m_mtd[chId]->tg[i].valid,m_mtd[chId]->tg[i].cur_x,m_mtd[chId]->tg[i].cur_y);
			}
//			OSA_printf("ALL-MTD: time = %f sec \n", ( (getTickCount() - tstart)/getTickFrequency()) );
		}
		*/
	/************************* while ********************************/
	}
	OSA_printf("%s: Main Proc Tsk Is Exit...\n",__func__);
}

int CVideoProcess::MAIN_threadDestroy(void)
{
	int iRet = OSA_SOK;

	mainProcThrObj.exitProcThread = true;
	OSA_semSignal(&mainProcThrObj.procNotifySem);

	iRet = OSA_thrDelete(&mainProcThrObj.thrHandleProc);

	mainProcThrObj.initFlag = false;
	OSA_semDelete(&mainProcThrObj.procNotifySem);

	return iRet;
}


CVideoProcess::CVideoProcess()
	:m_track(NULL),m_curChId(0),m_curSubChId(-1),adaptiveThred(180)
{
	memset(&mainProcThrObj, 0, sizeof(MAIN_ProcThrObj));
	m_ImageAxisxSdi=1920/2;
	m_ImageAxisySdi=1080/2;

	m_ImageAxisxVga=1024/2;
	m_ImageAxisyVga=768/2;

	m_ImageAxisxPal=720/2;
	m_ImageAxisyPal=576/2;
	

	
	m_intervalFrame = 0;
	//m_mtdChId=0;
}

CVideoProcess::~CVideoProcess()
{
	
}

int CVideoProcess::creat()
{

	m_track = CreateUtcTrk();
	//UtcSetPLT_BS(m_track, tPLT_WRK, BoreSight_Mid);
	MAIN_threadCreate();
	OSA_mutexCreate(&m_mutex);
	return 0;
}

int CVideoProcess::destroy()
{
	stop();
	OSA_mutexDelete(&m_mutex);
	MAIN_threadDestroy();


	return 0;
}


int CVideoProcess::stop()
{
	if(m_track != NULL)
		DestroyUtcTrk(m_track);
	m_track = NULL;
	return 0;
}



void CVideoProcess::extractUYVY2Gray2(Mat src, Mat dst)
{
	#if !NO_ARM_NEON
	int ImgHeight, ImgWidth,ImgStride;

	ImgWidth = src.cols;
	ImgHeight = src.rows;
	ImgStride = ImgWidth*2;
	
	uint8_t  *  pDst8_t;
	uint8_t *  pSrc8_t;

	pSrc8_t = (uint8_t*)(src.data);
	pDst8_t = (uint8_t*)(dst.data);

	for(int y = 0; y < ImgHeight*ImgWidth; y++)
	{
		pDst8_t[y] = pSrc8_t[y*2+1];
	}
	#endif 
}

int CVideoProcess::ReAcqTarget()
{
	int iRet = m_iTrackStat;
	/*
	if(m_trkChId != m_curChId){
		iRet = 0;
		m_rcTrack = m_rcAcq;
		m_trkChId = m_curChId;
		m_iTrackLostCnt = 0;
		printf("++++++++++++++++++++++++++++++++++++++++++++++++++1=\n");
	}
	*/
	if(m_intervalFrame > 0){
		m_intervalFrame--;
		if(m_intervalFrame == 0){
			iRet = 0;
			m_rcTrack = m_rcAcq;
			m_iTrackLostCnt = 0;
		}
	}
	return iRet;

}

int CVideoProcess::process_frame(int chId,Mat frame)
{
	int format = -1;
	if(frame.cols<=0 || frame.rows<=0)
		return 0;
	OSA_mutexLock(&m_mutex);
       if(chId == m_curChId /*&& (m_bTrack ||m_bMtd )*/)
       {
		mainFrame[mainProcThrObj.pp] = frame;
		mainProcThrObj.cxt[mainProcThrObj.pp].bTrack = m_bTrack;
		mainProcThrObj.cxt[mainProcThrObj.pp].iTrackStat = m_iTrackStat;
		mainProcThrObj.cxt[mainProcThrObj.pp].chId = chId;
		if(mainProcThrObj.bFirst){
			mainFrame[mainProcThrObj.pp^1] = frame;
			mainProcThrObj.cxt[mainProcThrObj.pp^1].bTrack = m_bTrack;
			mainProcThrObj.cxt[mainProcThrObj.pp^1].iTrackStat = m_iTrackStat;
			mainProcThrObj.cxt[mainProcThrObj.pp^1].chId = chId;
			mainProcThrObj.bFirst = false;
		}
		OSA_semSignal(&mainProcThrObj.procNotifySem);
		
       	}
	  OSA_mutexUnlock(&m_mutex);

	return 0;
}

int CVideoProcess::process_track(int trackStatus, Mat frame_gray, Mat frame_dis, UTC_RECT_float &rcResult)
{
	IMG_MAT image;

	image.data_u8 	= frame_gray.data;
	image.width 	= frame_gray.cols;
	image.height 	= frame_gray.rows;
	image.channels 	= 1;
	image.step[0] 	= image.width;
	image.dtype 	= 0;
	if(trackStatus != 0)
	{
		rcResult = UtcTrkProc(m_track, image, &trackStatus);
	}
	else
	{
		UTC_ACQ_param acq;
		if(m_curChId==pal_Track)
		{
		acq.axisX = m_ImageAxisxPal;// image.width/2;
		acq.axisY = m_ImageAxisyPal;//image.height/2;
		}else if(m_curChId==vga_Track)
		{
		acq.axisX = m_ImageAxisxVga;
		acq.axisY = m_ImageAxisyVga;
		}else if(m_curChId==sdi_Track)
		{
		acq.axisX = m_ImageAxisxSdi;
		acq.axisY = m_ImageAxisySdi;
		}
		acq.rcWin.x = (int)(rcResult.x);
		acq.rcWin.y = (int)(rcResult.y);
		acq.rcWin.width = (int)(rcResult.width);
		acq.rcWin.height = (int)(rcResult.height);
		if(acq.rcWin.width<0)
		{
			acq.rcWin.width=0;

		}
		else if(acq.rcWin.width>= image.width)
		{
			acq.rcWin.width=80;
		}
		if(acq.rcWin.height<0)
		{
			acq.rcWin.height=0;

		}
		else if(acq.rcWin.height>= image.height)
		{
			acq.rcWin.height=60;
		}
		if(acq.rcWin.x<0)
		{
			acq.rcWin.x=0;
		}
		else if(acq.rcWin.x>image.width-acq.rcWin.width)
		{

			acq.rcWin.x=image.width-acq.rcWin.width;
		}
		if(acq.rcWin.y<0)
		{
			acq.rcWin.y=0;
		}
		else if(acq.rcWin.y>image.height-acq.rcWin.height)
		{

			acq.rcWin.y=image.height-acq.rcWin.height;
		}
		rcResult = UtcTrkAcq(m_track, image, acq);
		trackStatus = 1;
	}
	return trackStatus;
}


#endif

