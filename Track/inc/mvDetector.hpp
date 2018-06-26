
#ifndef mvDetector_HPP_
#define mvDetector_HPP_

//#include "osa.h"
#include "osa_mutex.h"
#include "osa_tsk.h"
#include "osa_sem.h"
//#include "app_osdgrp.h"
//#include "app_status.h"
#include "mvdectInterface.hpp"
using namespace cv;


typedef void (*MVDETECTORDRAW)(std::vector<TRK_RECT_INFO> &resTarget,int chId);

class mvDetector
{

public:
	~mvDetector();
	int creat();
	int destroy();
	int init();
	bool isBusy(int chId);
	//warn_mode:warningArea's mode ,chId:warningArea's uniq ID 
	void setWarnMode(WARN_MODE warn_mode,int chId);
	//polyWarnRoi:warningArea's four points,chId:warningArea's uniq ID 
	void setWarningRoi(std::vector<cv::Point> polyWarnRoi,int chId);
	//frame:send frame to detector ,chId:warningArea's uniq ID 
	int process_frame(Mat frame,int chId);
	
	void mvDetectorDrawCB(MVDETECTORDRAW mvDetectorDraw);
	static  mvDetector* getInstance()
	{
		if(m_Instance==NULL)
		{
			m_Instance=new  mvDetector();
		}
		return m_Instance;
	}

protected:
	
	bool m_bCreate;

	

	static mvDetector *pThis;


protected:
	CMvDectInterface	*m_pMovDetector;
	void	initMvDetect();
	void	DeInitMvDetect();
	static void NotifyFunc(void *context, int chId);
	std::vector<TRK_RECT_INFO>		m_warnLostTarget[DETECTOR_NUM];
	std::vector<TRK_RECT_INFO>		m_warnInvadeTarget[DETECTOR_NUM];
	std::vector<TRK_RECT_INFO>		m_movTarget[DETECTOR_NUM];
	std::vector<TRK_RECT_INFO>		m_edgeTarget[DETECTOR_NUM];
	std::vector<TRK_RECT_INFO>		m_warnTarget[DETECTOR_NUM];


private:
	mvDetector();
	OSA_MutexHndl m_mutex;
	static mvDetector* m_Instance;
	MVDETECTORDRAW mvDetectorDraw;
	bool m_bBusy[DETECTOR_NUM];
	WARN_MODE m_currentMarmMode[4];
	//int           m_pp;

};



#endif /* VIDEOPROCESS_HPP_ */
