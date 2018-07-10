#include<opencv2/opencv.hpp>
#include"MvDetect.hpp"
#include<string.h>
#include<stdio.h>
using namespace cv;
Mat m4(2160,640,CV_8UC3);
Mat m6(3240,640,CV_8UC3);

Mat m4_2cc(2160,640,CV_8UC2);
Mat m6_2cc(3240,640,CV_8UC2);
unsigned char * p_newestMvSrc[CAM_COUNT]={NULL,NULL,NULL,NULL,NULL,NULL,NULL
,NULL,NULL,NULL};
extern MvDetect mv_detect;

#if  MVDECT
MvDetect::MvDetect()
{
	for(int i=0;i<CAM_COUNT;i++)
	{
		for(int j=0;j<6;j++)
		{
			tempRect_Srcptr[i].tempoutRect.rects[j].x=-1;
			tempRect_Srcptr[i].tempoutRect.rects[j].y=-1;
			tempRect_Srcptr[i].tempoutRect.rects[j].width=-1;
			tempRect_Srcptr[i].tempoutRect.rects[j].height=-1;
			tempRect_Srcptr[i].isDetectionDone=false;
		}
	}
for(int i=0;i<2;i++)
{
		enableMD[i]=false;
		MDopen[i]=false;
}
		for(int i=0;i<CAM_COUNT;i++)
		{
			grayFrame[i]=(unsigned char *)malloc(MAX_SCREEN_WIDTH*MAX_SCREEN_HEIGHT*1);
		}
}
MvDetect::~MvDetect()
{
			exitDetect();
			for(int i=0;i<CAM_COUNT;i++)
			{
				free(grayFrame[i]);
			}
}

void MvDetect::init(int w,int h)
{
		createDetect((unsigned char)CAM_COUNT,w,h);
}
void MvDetect::uyvy2gray(unsigned char* src,unsigned char* dst,int width,int height)
{
	for(int i=0;i<width*height;i++)
		{
	    *(dst++) =*(++src) ;
	    src+=1;
		}
}
void MvDetect::m_mvDetect(int idx,unsigned char* inframe,int w,int h)
{
		idx-=1;
		uyvy2gray(inframe,grayFrame[idx]);
		{
			tempRect_Srcptr[idx].isDetectionDone = false;

	/*		for(int i=0;i<6;i++)
			{
				tempRect_Srcptr[idx].tempoutRect.rects[i].width=20;
				tempRect_Srcptr[idx].tempoutRect.rects[i].height=80;
				tempRect_Srcptr[idx].tempoutRect.rects[i].x=33;
				tempRect_Srcptr[idx].tempoutRect.rects[i].y=432;
			}
			tempRect_Srcptr[idx].isDetectionDone = true;
		*/

			mvDetect((unsigned char) (idx+1), grayFrame[idx], w, h,
					tempRect_Srcptr[idx].tempoutRect.rects,
					&tempRect_Srcptr[idx].isDetectionDone);

		}


/*	uyvy2gray(inframe,grayFrame[idx-1]);
	{
		mvDetect((unsigned char) (idx), grayFrame[idx-1], w, h,tempoutRect[idx-1].rects);
	}*/
	//mvDetect((unsigned char) idx, grayFrame[idx], w, h,&outRect[idx]);
}
#endif

void MvDetect::saveConfig()
{
	char buf[4]={0};
	FILE * fp=fopen("./data/MvDetect.txt","w");
	if(fp==NULL)
	{
		printf("MvDetect open failed!\n");
	}
	if(GetMD(MAIN) &&GetMD(SUB))
	{
		sprintf(buf,"%d\n",1);
	}
	else
		sprintf(buf,"%d\n",0);
	fwrite(buf,sizeof(buf),1,fp);
	fclose(fp);
}

void myRect(unsigned char *dst,unsigned char *src,int x,int y,int w,int h)
{
#if 0
	unsigned char temp[540*640*4];
	Mat SRC(1080,1920,CV_8UC4,src);
	Mat (540,640,CV_8UC4,dst);
	Rect rect(x,y,w,h);
	Mat DST=SRC(rect);
	memcpy(dst,DST.data,540*640*4);
#endif
}
void MvDetect::selectFrame(unsigned char *dst,unsigned char *src,int targetId,int camIdx)
{
#if 0
	int startx=0;
	int starty=0;
	int w=640;
	int h=540;
	int midx=0;
	int midy=0;
	if(targetnum[camIdx]==0)
	{
		for(int i=0;i<CAM_COUNT;i++)
		{
			for(int j=0;j<4;j++)
				targetidx[i][j]=0;
		}
		memset(dst,0xffff,640*540*4);
	}
	else
	{
		std::vector<cv::Rect> tempRecv[CAM_COUNT];
		for(int i=0;i<CAM_COUNT;i++)
		{
			tempRecv[i].assign(outRect[i].begin(),outRect[i].end());
		}
		startx=tempRecv[camIdx][targetidx[camIdx][targetId]].x;
		starty=tempRecv[camIdx][targetidx[camIdx][targetId]].y;
		w=tempRecv[camIdx][targetidx[camIdx][targetId]].width;
		h=tempRecv[camIdx][targetidx[camIdx][targetId]].height;
		midx=startx+w/2;
		midy=starty+h/2;
		if(midx<320)
		{
			midx=320;
		}
		else if (midx >1600)
		{
			midx=1600;
		}
		if(midy<270)
			{
				midy=270;
			}
			else if (midy >810)
			{
				midy=810;
			}
		startx=midx-640/2;
		starty=midy-540/2;
		myRect(dst,src,startx,starty,640,540);
	}

#endif
}
void MvDetect::ReadConfig()
{
	char buf[4];
	int Yes=-1;
	FILE * fp=fopen("./data/MvDetect.txt","r");
	if(fp==NULL)
	{
		printf("MvDetect open failed!\n");
	}
	fread(buf,sizeof(buf),1,fp);
	fclose(fp);
	Yes=atoi(buf);
	if(Yes)
	{
		SetMD(true,MAIN);
		SetMD(true,SUB);
	}
	else
	{
		SetMD(false,MAIN);
		SetMD(false,SUB);
	}
}
bool MvDetect::CanUseMD(int mainorsub)
{
	if(enableMD[mainorsub]==true &&MDopen[mainorsub]==true)
		return true;
	else
		return false;
}

void MvDetect::SetoutRect()
{
	for(int idx=0;idx<CAM_COUNT;idx++)
	{
		if(tempRect_Srcptr[idx].isDetectionDone)//沒有檢測完成，保留上次結果
				outRect[idx].clear();
		mvRect tempOut;
			for(int j=0;j<6;j++)
			{
				if(tempRect_Srcptr[idx].isDetectionDone){
					if(tempRect_Srcptr[idx].tempoutRect.rects[j].x>0)
					{
					//	tempOut.srcptr=tempRect_Srcptr[idx].srcptr;
						tempOut.outRect=tempRect_Srcptr[idx].tempoutRect.rects[j];
						tempOut.camIdx=idx;
						outRect[idx].push_back(tempOut);
					}
				}
			}
	}
	/*
	for(int i=0;i<CAM_COUNT;i++)
	{
		outRect[i].clear();
		for(int j=0;j<6;j++)
		{
			if(tempoutRect[i].rects[j].x>0)
			{
				outRect[i].push_back(tempoutRect[i].rects[j]);
			}
		}
	}*/
}



#if 0
void MvDetect:: DrawAllRectOri(int fourOrsix)
{

	vector<mvRect>::iterator  it;
	vector<mvRect> *wholeV;
	vector<mvRect> *wholeVrcv;
	vector<mvRect> tempV;

	m_WholeRect.clear();
	for(int i=0;i<CAM_COUNT;i++)
		m_WholeRect.insert(m_WholeRect.end(),outRect[i].begin(),outRect[i].end());
	wholeVrcv=&m_WholeRect;
//将范围内的rect找出
	if(range[END]>range[START]){					//不跨360度
		for(int i=0;i<wholeVrcv->size();i++)
		{
			if((*wholeVrcv)[i].x_angle()>range[START]   && (*wholeVrcv)[i].x_angle()<=range[END] )
			{
				tempV.push_back((*wholeVrcv)[i]);
			}
		}
	}
	else{//跨360度
		for(int i=0;i<wholeVrcv->size();i++)
		{
			if((*wholeVrcv)[i].x_angle()>range[START]   && (*wholeVrcv)[i].x_angle()<=360.0)
			{
				tempV.push_back((*wholeVrcv)[i]);
			}
			for(int i=0;i<wholeVrcv->size();i++)
			{
				if((*wholeVrcv)[i].x_angle()>0   && (*wholeVrcv)[i].x_angle()<=range[END] )
				{
					tempV.push_back((*wholeVrcv)[i]);
				}
			}
		}
	}
	wholeV=&tempV;

		//按x从小到达排序
		sort(wholeV->begin(),wholeV->end(),CmpXsamller);
		/*****找出所以的框*****/
		for(int i=0;i<wholeV->size();i++)
		{
			if(wholeV->size()>m_sumTarget)//个数大于m_sumTarget
			{
				if(i>=0&&i<=m_sumTarget) //前m_sumTarget个绿色
				{
					(*wholeV)[i].color[0]=0;
					(*wholeV)[i].color[1]=255.0;
					(*wholeV)[i].color[2]=0;
					targetRect[i]=(*wholeV)[i];  //将前m_sumTarget个绿色的保留以显示图片
					lastRect[i]=targetRect[i];//黄色target默认targetRect
				}
				else//红色
				{
					(*wholeV)[i].color[0]=255.0;
					(*wholeV)[i].color[1]=0;
					(*wholeV)[i].color[2]=0;
				}
			}
			else if(wholeV->size()<=m_sumTarget)//个数小于等于m_sumTarget 全绿色
			{
				(*wholeV)[i].color[0]=0;
				(*wholeV)[i].color[1]=255.0;
				(*wholeV)[i].color[2]=0;
				targetRect[i]=(*wholeV)[i];//将前n个绿色的保留以显示图片
				lastRect[i]=targetRect[i];//黄色target默认targetRect
			}
		}
		//个数小于m_sumTarget 个，其余的 targetRect lastRect的angle 为-1
		 if(wholeV->size()<=m_sumTarget)
		 {
			int j=m_sumTarget-wholeV->size();
			for(int k=wholeV->size();k<j;k++)
			{
				lastRect[k]=targetRect[k];
			}
		 }

		if(ISanySingleRect())//如果有Singletarget在，则再画黄色
		{
			//m_sumTarget 个目标
		for(int targetidx=0;targetidx<m_sumTarget;targetidx++)
		{
	//如果targetidx圈出的rect选择下一个
	 if(IsChooseN(targetidx))
	 {
		 //在所有从小到大排序好的vector中插入lastrect[targetidx],找出lastrect的下一个作为黄色rect
		 //并将这个rect赋值给lastrect[targetidx]
			for(int i=0;i<wholeV->size();i++)
			{
				//如果找到第一个大于last的，则跳出循环
				if(lastRect[targetidx].x_angle<(*wholeV)[i].x_angle)
				{
					(*wholeV)[i].color[0]=0;
					(*wholeV)[i].color[1]=255;
					(*wholeV)[i].color[2]=255;
					lastRect[targetidx]=(*wholeV)[i];
					break;
				}
				//如果没找，没有break出循环，则把第最小的给到lastrect，相当于转了一圈回来
				lastRect[targetidx]=(*wholeV)[0];
			}
	 }//next
	 else if(IsChooseP(targetidx))
		 {
		 	 sort(wholeV->begin(),wholeV->end(),CmpXbigger);
			 //在所有从大到小排序好的vector中插入lastrect[targetidx],找出lastrect的上一个作为黄色rect
			 //并将这个rect赋值给lastrect[targetidx]
				for(int i=0;i<wholeV->size();i++)
				{
					//如果找到第一个小于last的，则跳出循环
					if(lastRect[targetidx].x_angle>(*wholeV)[i].x_angle)
					{
						(*wholeV)[i].color[0]=0;
						(*wholeV)[i].color[1]=255;
						(*wholeV)[i].color[2]=255;
						lastRect[targetidx]=(*wholeV)[i];
						break;
					}
					//如果没找，没有break出循环，则把第最大的给到lastrect，相当于转了一圈回来
					lastRect[targetidx]=(*wholeV)[0];
				}
		 }//pre
	 else if(IsChooseUp(targetidx))
		 {
		 	 sort(wholeV->begin(),wholeV->end(),CmpYbigger);
			 //在所有从大到小排序好的vector中插入lastrect[targetidx],找出lastrect的上一个作为黄色rect
			 //并将这个rect赋值给lastrect[targetidx]
				for(int i=0;i<wholeV->size();i++)
				{
					//如果找到第一个小于last的，则跳出循环
					if(lastRect[targetidx].y_angle>(*wholeV)[i].y_angle)
					{
						(*wholeV)[i].color[0]=0;
						(*wholeV)[i].color[1]=255;
						(*wholeV)[i].color[2]=255;
						lastRect[targetidx]=(*wholeV)[i];
						break;
					}
					//如果没找，没有break出循环，则把第最大的给到lastrect，相当于转了一圈回来
					lastRect[targetidx]=(*wholeV)[0];
				}
		 }//up
	 else if(IsChooseDown(targetidx))
		 {
		 	 sort(wholeV->begin(),wholeV->end(),CmpYsmaller);
			 //在所有从小到大排序好的vector中插入lastrect[targetidx],找出lastrect的下一个作为黄色rect
			 //并将这个rect赋值给lastrect[targetidx]
				for(int i=0;i<wholeV->size();i++)
				{
					//如果找到第一个大于last的，则跳出循环
					if(lastRect[targetidx].y_angle<(*wholeV)[i].y_angle)
					{
						(*wholeV)[i].color[0]=0;
						(*wholeV)[i].color[1]=255;
						(*wholeV)[i].color[2]=255;
						lastRect[targetidx]=(*wholeV)[i];
						break;
					}
					//如果没找，没有break出循环，则把第最小的给到lastrect，相当于转了一圈回来
					lastRect[targetidx]=(*wholeV)[0];
				}
		 }//down
			}//从0～targetNUM次

		}//有target在

		/*******把绿色 红色 黄色 全画了****/
				if(fourOrsix==MAIN_FPGA_FOUR)//只画cam 6~9
				{
					for(int i=0;i<wholeV->size();i++)
					{
						if((*wholeV)[i].camIdx>=6 &&(*wholeV)[i].camIdx<=9)
						{
							MRectangle(MAIN_FPGA_FOUR,&(*wholeV)[i]);
						}
					}
				}
				else if(fourOrsix==MAIN_FPGA_SIX)//只画cam 0~5
				{
					for(int i=0;i<wholeV->size();i++)
					{
						if((*wholeV)[i].camIdx>=0&&(*wholeV)[i].camIdx<=5)
						{
							MRectangle(MAIN_FPGA_SIX,&(*wholeV)[i]);
						}
					}
				}

}

#endif
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
					int startx=tempRecv[i][rectIdx].outRect.x/3;
					int starty=tempRecv[i][rectIdx].outRect.y/2+540*i;
					int w=tempRecv[i][rectIdx].outRect.width/3;
					int h=tempRecv[i][rectIdx].outRect.height/2;//取出容器中rect的值
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
						int startx=tempRecv[i][rectIdx].outRect.x/3;
						int starty=tempRecv[i][rectIdx].outRect.y/2+540*(i-6);
						int w=tempRecv[i][rectIdx].outRect.width/3;
						int h=tempRecv[i][rectIdx].outRect.height/2;//取出容器中rect的值
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

