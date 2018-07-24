#include<opencv2/opencv.hpp>
#include "mvdetectInterface.h"
#include<string.h>
#include<stdio.h>
#include "infoHead.h"
#include "osa_sem.h"
using namespace cv;
extern bool IsMvDetect;
int  parm_accuracy=4;
int parm_inputArea=8/*6*/;
int parm_inputMaxArea=200;
int parm_threshold = 30;
int MV_CHOSE_IDX=0;
Mat m4(2160,640,CV_8UC3);
Mat m6(3240,640,CV_8UC3);

Mat m4_2cc(2160,640,CV_8UC2);
Mat m6_2cc(3240,640,CV_8UC2);
unsigned char * p_newestMvSrc[CAM_COUNT]={NULL,NULL,NULL,NULL,NULL,NULL,NULL
,NULL,NULL,NULL};
extern MvDetect mv_detect;
#if  MVDECT

float	  mvRect::x_angle()
{
	 return camIdx*SDI_WIDTH+outRect.targetRect.x;
}

float	  mvRect::y_angle()
{
	 return  outRect.targetRect.y;
}

void MvDetect::ReSetLineY()
{
	for(int idx=0;idx<CAM_COUNT;idx++)
	{
		lineY[idx]=lineY[idx]+linedelta[idx];
		if(lineY[idx]+half_RoiAreah*1080>=1080)
		{
			lineY[idx]=1080-half_RoiAreah*1080;
		}
		if(lineY[idx]-half_RoiAreah*1080<=0)
		{
			lineY[idx]=half_RoiAreah*1080;
		}
	}
}

float MvDetect::GetRoiStartY_OffsetCoefficient(int idx)
{
	int whiteSpace=-1;
	float OffsetCoefficient=-1;
	whiteSpace=lineY[idx]-half_RoiAreah*1080;
	OffsetCoefficient=((float)whiteSpace)/1080.0;
	return OffsetCoefficient;
}

MvDetect::MvDetect(CMvDectInterface *pmvIf):
		m_pMovDetector(pmvIf),half_RoiAreah(0.3)
{
	int ret=-1;
	for(int i=0;i<CAM_COUNT;i++)
		{
			tempRect_Srcptr[i].clear();
			grayFrame[i]=(unsigned char *)malloc(MAX_SCREEN_WIDTH*MAX_SCREEN_HEIGHT*1);
			pSemMV[i]=(OSA_SemHndl *)malloc(sizeof(OSA_SemHndl)) ;
			ret=OSA_semCreate(pSemMV[i],1,1);
			if(ret<0)
			{
				printf("pSemMV OSA_semCreate failed\n");
			}
			lineY[i]=540;
			linedelta[i]=0;
		}
}
MvDetect::~MvDetect()
{
	if(m_pMovDetector != NULL)
		m_pMovDetector->destroy();
	for(int i=0;i<CAM_COUNT;i++)
	{
		free(grayFrame[i]);
		free(pSemMV[i]);

	}
}

void MvDetect::NotifyFunc(void *context, int chId)
{
	int len=0;
	MvDetect *p = static_cast<MvDetect*>(context);
	if(IsMvDetect){
		OSA_semWait(p->GetpSemMV(chId),100000);
		p->m_pMovDetector->getMoveTarget(p->tempRect_Srcptr[chId],chId);
		//OSA_semWait(p->GetpSemMV(8),100000);
				//p->m_pMovDetector->getMoveTarget(p->tempRect_Srcptr[8],8);
/*		if(p->GetptempRect_Srcptr(chId)->size()>=10)
						len=10;
		else if(p->GetptempRect_Srcptr(chId)->size()<10)
		{
			len=p->GetptempRect_Srcptr(chId)->size();
		}
				*/
		OSA_semSignal(p->GetpSemMV(chId));
	//			OSA_semSignal(p->GetpSemMV(8));
	}
}
void MvDetect::ClearAllVector(bool IsOpen)
{
	if(IsOpen)
	{
		for(int i=0;i<CAM_COUNT;i++)
		{
			tempRect_Srcptr[i].clear();
			outRect[i].clear();
		}
		m_WholeRect[0].clear();
		m_WholeRect[1].clear();
	}
	else
		m_pMovDetector->mvPause();
}
void MvDetect::init(int w,int h)
{
	if(m_pMovDetector == NULL)
			m_pMovDetector = MvDetector_Create();
	m_pMovDetector->init(NotifyFunc, (void*)this);
}
void MvDetect::uyvy2gray(unsigned char* src,unsigned char* dst,int idx,int width,int height)
{
	float sf=GetRoiStartY_OffsetCoefficient(idx);
	float	src_sf=1920*1080*2*sf;
	src+=(int)src_sf;
//	src+=1920*432;//1920*1080*0.2*2;
	for(int i=0;i<width*height*half_RoiAreah*2;i++)
		{
	    *(dst++) =*(++src) ;
	    src+=1;
		}
}
void MvDetect::m_mvDetect(int idx,unsigned char* inframe,int w,int h)
{
		idx-=1;
		uyvy2gray(inframe,grayFrame[idx],idx);
		Mat gm(h*half_RoiAreah*2,w,CV_8UC1,grayFrame[idx]);
		if(m_pMovDetector != NULL)
			m_pMovDetector->setFrame(gm,gm.cols,gm.rows,idx,parm_accuracy,parm_inputArea,parm_inputMaxArea,parm_threshold);
}
#endif

void MvDetect::SetoutRect()
{
	int len=0;
#if 1
	for(int idx=0;idx<CAM_COUNT;idx++)
	{
		outRect[idx].clear();
		if(!tempRect_Srcptr[idx].empty())
		{
			OSA_semWait(this->GetpSemMV(idx),100000);
			mvRect tempOut;
				{
					len=tempRect_Srcptr[idx].size();
				}
				for(int j=0;j<len;j++)
				{
					if(tempRect_Srcptr[idx][j].targetRect.x>0)
					{
					//	tempOut.outRect=tempRect_Srcptr[idx][j];
						(tempOut.outRect.targetRect)=tempRect_Srcptr[idx][j].targetRect;
						tempOut.camIdx=idx;
						outRect[idx].push_back(tempOut);
	/*					printf("Cam:%d     x=%d   y=%d  w=%d  h=%d\n",idx,
								tempRect_Srcptr[idx][j].targetRect.x,
								tempRect_Srcptr[idx][j].targetRect.y,
								tempRect_Srcptr[idx][j].targetRect.width,
								tempRect_Srcptr[idx][j].targetRect.height);*/
					}
				}
				OSA_semSignal(this->GetpSemMV(idx));
		}
	}
#else

	outRect[0].clear();
	if(!tempRect_Srcptr[8].empty())
	{
		OSA_semWait(this->GetpSemMV(8),100000);
		mvRect tempOut;
					{
						len=tempRect_Srcptr[8].size();
					}
					for(int j=0;j<len;j++)
					{
						if(tempRect_Srcptr[8][j].targetRect.x>0)
						{
						//	tempOut.outRect=tempRect_Srcptr[idx][j];
							(tempOut.outRect.targetRect)=tempRect_Srcptr[8][j].targetRect;
							tempOut.camIdx=8;
							outRect[0].push_back(tempOut);
						}
					}
		OSA_semSignal(this->GetpSemMV(8));
		}
#endif
}

void MvDetect::DrawRectOnpic(unsigned char *src,int capidx,int cc)
{
	//DrawAllRectOri(capidx);

	std::vector<mvRect> tempRecv[CAM_COUNT];
	if(capidx==MAIN_FPGA_SIX)
	{
		if(cc==3)
		{
			m6.data=src;
		}
		else if(cc==2)
		{
			m6_2cc.data=src;
		}
		for(int i=0;i<6;i++)                        //0  1  2
		{															//3  4  5
			tempRecv[i].assign(outRect[i].begin(),outRect[i].end());
			if(tempRecv[i].size()!=0)//容器dix不为空
			{
				for(int rectIdx=0;rectIdx<tempRecv[i].size();rectIdx++)//从容器中一个一个取出
				{
					int startx=tempRecv[i][rectIdx].outRect.targetRect.x/3;
					int starty=tempRecv[i][rectIdx].outRect.targetRect.y/2+540*i;
					int w=tempRecv[i][rectIdx].outRect.targetRect.width/3;
					int h=tempRecv[i][rectIdx].outRect.targetRect.height/2;//取出容器中rect的值
			//		starty+=540*0.2;
					int endx=startx+w;
					int endy=starty+h;
					if(cc==3)
					{
						cv::rectangle(m6,cvPoint(startx,starty),cvPoint(endx,endy),cvScalar(0,0,0),2);
					}
					else if(cc==2)
					{
						cv::rectangle(m6_2cc,cvPoint(startx,starty),cvPoint(endx,endy),cvScalar(0,0,0),2);
					}
				}
			}
		}
	}
	if(capidx==MAIN_FPGA_FOUR)
		{
		if(cc==3)
			{
			m4.data=src;
			}
			else if(cc==2)
			{
				m4_2cc.data=src;
			}
			for(int i=6;i<10;i++)						//6   7
			{															//8	 9
				tempRecv[i].assign(outRect[i].begin(),outRect[i].end());
				if(tempRecv[i].size()!=0)//容器dix不为空
				{
					for(int rectIdx=0;rectIdx<tempRecv[i].size();rectIdx++)//从容器中一个一个取出
					{
						int startx=tempRecv[i][rectIdx].outRect.targetRect.x/3;
						int starty=tempRecv[i][rectIdx].outRect.targetRect.y/2+540*(i-6);
						int w=tempRecv[i][rectIdx].outRect.targetRect.width/3;
						int h=tempRecv[i][rectIdx].outRect.targetRect.height/2;//取出容器中rect的值
	//					starty+=540*0.2;
						int endx=startx+w;
						int endy=starty+h;
						if(cc==3)
						{
							cv::rectangle(m4,cvPoint(startx,starty),cvPoint(endx,endy),cvScalar(0,0,0),2);
						}
						else if(cc==2)
						{
							cv::rectangle(m4_2cc,cvPoint(startx,starty),cvPoint(endx,endy),cvScalar(0,0,0),2);
						}
					}
				}
			}
		}
}

