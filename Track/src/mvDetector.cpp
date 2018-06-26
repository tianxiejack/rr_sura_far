#if MVDETECTOR_MODE
#include <GL/glew.h>
#include <GL/glut.h>
#include "grpFont.h"
#include <cuda.h>
#include <cuda_gl_interop.h>
#include <cuda_runtime.h>
#include "mvDetector.hpp"
//#include "cuProcess.hpp"



mvDetector * mvDetector::pThis = NULL;
mvDetector * mvDetector::m_Instance=NULL;
mvDetector::mvDetector()
{
	pThis = this;
	m_bCreate = false;
	m_pMovDetector = NULL;
}

mvDetector::~mvDetector()
{
	pThis = NULL;
}
void mvDetector::mvDetectorDrawCB(MVDETECTORDRAW mvDetectorDraw)
{
	this->mvDetectorDraw=mvDetectorDraw;
}
int mvDetector::creat()
{
	OSA_mutexCreate(&m_mutex);
	m_bCreate = true;
	for(int i=0;i<DETECTOR_NUM;i++)
	{
		m_bBusy[i]=false;
	}
	if(m_pMovDetector == NULL)
		m_pMovDetector = MvDetector_Create();
	OSA_assert(m_pMovDetector != NULL);
	return 0;
}

int mvDetector::destroy()
{
	if(!m_bCreate)
		return 0;
	OSA_printf("%s ...", __func__);
	DeInitMvDetect();
	OSA_mutexDelete(&m_mutex);
	m_bCreate = false;
	OSA_printf("%s ... ok", __func__);
	return 0;
}


int mvDetector::init()
{
	initMvDetect();
	return 0;
}
bool mvDetector::isBusy(int chId)
{
	if(m_bBusy[chId])
		return true;
	else
		return false;
}

void mvDetector::setWarningRoi(std::vector<cv::Point> polyWarnRoi ,int chId)
{
	m_pMovDetector->setWarningRoi(polyWarnRoi,	chId);
	
	
}
void mvDetector::setWarnMode(WARN_MODE warn_mode,int chId)
{

	m_pMovDetector->setWarnMode(warn_mode, chId);
	m_currentMarmMode[chId]=warn_mode;

}
void   mvDetector::initMvDetect( )
{
	int	i;
	OSA_printf("%s:mvDetect start ", __func__);
	OSA_assert(m_pMovDetector != NULL);

	m_pMovDetector->init(NotifyFunc, (void*)this);
	std::vector<cv::Point> polyWarnRoi ;
	polyWarnRoi.resize(4);
	polyWarnRoi[0]	= cv::Point(0,0);
	polyWarnRoi[1]	= cv::Point(1920,0);
	polyWarnRoi[2]	= cv::Point(1920,640);
	polyWarnRoi[3]	= cv::Point(0,640);
	for(i=0; i<DETECTOR_NUM; i++){
		m_pMovDetector->setWarningRoi(polyWarnRoi,	i);
	//	m_pMovDetector->setDrawOSD(m_dc, i);
	//	m_pMovDetector->enableSelfDraw(true, i);
		m_pMovDetector->setWarnMode(WARN_INVAD_LOST_MODE, i);
	}
	
}

void	mvDetector::DeInitMvDetect()
{
	if(m_pMovDetector != NULL)
		m_pMovDetector->destroy();
}

static void extractUYVY2Gray(Mat src, Mat dst)
{
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
}

int mvDetector::process_frame(Mat frame,int chId)
{

	if(frame.cols<=0 || frame.rows<=0)
		return 0;
//	tstart = getTickCount();

	OSA_mutexLock(&m_mutex);
	int channel = frame.channels();
	Mat frame_gray;
	frame_gray = Mat(frame.rows, frame.cols, CV_8UC1);
	if(channel == 2){
		extractUYVY2Gray(frame, frame_gray);
	}else if(channel==1){
		memcpy(frame_gray.data, frame.data, frame.cols * frame.rows*channel*sizeof(unsigned char));
	}else
	{
	/*	static int a=0;
		a++;
		if(a==50)
			imwrite("./data/alarm_ori.bmp",frame);
		cvtColor(frame,frame_gray,CV_BGRA2GRAY);*/
//		imwrite("alarm_gray.bmp",frame_gray);
		//printf("\n save bmp\n");
	}
	if(m_pMovDetector != NULL)
		m_pMovDetector->setFrame(frame_gray,chId);
	OSA_mutexUnlock(&m_mutex);

//	OSA_printf("process_frame: chId = %d, time = %f sec \n",chId,  ( (getTickCount() - tstart)/getTickFrequency()) );
	return 0;
}

void mvDetector::NotifyFunc(void *context, int chId)
{
	mvDetector *pParent =(mvDetector*)context;
	switch(pParent ->m_currentMarmMode[chId])
	{
		case WARN_MOVEDETECT_MODE:
			pParent->m_pMovDetector->getMoveTarget(pParent->m_movTarget[chId],chId);
			pParent->mvDetectorDraw(pParent->m_movTarget[chId],chId);
			break;
		case WARN_BOUNDARY_MODE:
			pParent->m_pMovDetector->getBoundTarget(pParent->m_edgeTarget[chId],chId);
			pParent->mvDetectorDraw(pParent->m_edgeTarget[chId],chId);
			break;
		case WARN_INVADE_MODE:
			pParent->m_pMovDetector->getInvadeTarget(pParent->m_warnInvadeTarget[chId],chId);
			pParent->mvDetectorDraw(pParent->m_warnInvadeTarget[chId],chId);
			break;
		case WARN_LOST_MODE:
			pParent->m_pMovDetector->getLostTarget(pParent->m_warnLostTarget[chId],chId);
			pParent->mvDetectorDraw(pParent->m_warnLostTarget[chId],chId);
			break;
		case WARN_INVAD_LOST_MODE:
			pParent->m_pMovDetector->getInvadeTarget(pParent->m_warnInvadeTarget[chId],chId);
			pParent->mvDetectorDraw(pParent->m_warnInvadeTarget[chId],chId);
			pParent->m_pMovDetector->getLostTarget(pParent->m_warnLostTarget[chId],chId);
			pParent->mvDetectorDraw(pParent->m_warnLostTarget[chId],chId);	
			break;
		default:
			break;
	}
	pParent->m_bBusy[chId]=false;
}



#endif

