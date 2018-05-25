#include<opencv2/opencv.hpp>
#include"MvDetect.hpp"
#include<string.h>
#include<stdio.h>
using namespace cv;
Mat m4(1080,1920,CV_8UC4);
Mat m6(1080,1920,CV_8UC4);
#if  MVDECT
MvDetect::MvDetect()
{
	for(int i=0;i<CAM_COUNT;i++)
	{
		for(int j=0;j<6;j++)
		{
			tempoutRect[i].rects[j].x=-1;
			tempoutRect[i].rects[j].y=-1;
			tempoutRect[i].rects[j].width=-1;
			tempoutRect[i].rects[j].height=-1;
		}
	}

for(int i=0;i<2;i++)
{
		enableMD[i]=true;
		MDopen[i]=true;
}
		for(int i=0;i<CAM_COUNT;i++)
		{
			targetnum[i]=0;
			grayFrame[i]=(unsigned char *)malloc(MAX_SCREEN_WIDTH*MAX_SCREEN_HEIGHT*1);
			for(int j=0;j<4;j++)
				targetidx[i][j]=0;
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
void MvDetect::yuyv2gray(unsigned char* src,unsigned char* dst,int width,int height)
{
	for(int i=0;i<width*height;i++)
		{
	    *(dst++) =*(src) ;
	    src+=2;
		}
}
void MvDetect::m_mvDetect(int idx,unsigned char* inframe,int w,int h)
{
	yuyv2gray(inframe,grayFrame[idx]);

	{
		mvDetect((unsigned char) idx, grayFrame[idx], w, h,tempoutRect[idx].rects);
	}
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
	unsigned char temp[540*640*4];
	Mat SRC(1080,1920,CV_8UC4,src);
	Mat (540,640,CV_8UC4,dst);
	Rect rect(x,y,w,h);
	Mat DST=SRC(rect);
	memcpy(dst,DST.data,540*640*4);
}
void MvDetect::selectFrame(unsigned char *dst,unsigned char *src,int targetId,int camIdx)
{
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
	for(int i=0;i<CAM_COUNT;i++)
	{
		outRect[i].clear();
		for(int j=0;j<6;j++)
		{
			if(tempoutRect[i].rects[j].x>0)
			{
printf("CAM:%d,rect[%d] x=%d y=%d,w=%d h=%d\n",i,j,
		tempoutRect[i].rects[j].x,tempoutRect[i].rects[j].y,tempoutRect[i].rects[j].width,tempoutRect[i].rects[j].height);
				outRect[i].push_back(tempoutRect[i].rects[j]);
			}
		}
	}

}
void MvDetect::DrawRectOnpic(unsigned char *src,int capidx)
{
	int cc=4;
	std::vector<cv::Rect> tempRecv[CAM_COUNT];
#if 1
	if(capidx==MAIN_FPGA_SIX)
	{
		m6.data=src;
		for(int i=0;i<6;i++)
		{
			tempRecv[i].assign(outRect[i].begin(),outRect[i].end());
		}
		for(int i=0;i<6;i++)
		{
			if(tempRecv[i].size()!=0)//容器dix不为空
			{
				for(int rectIdx=0;rectIdx<tempRecv[i].size();rectIdx++)//从容器中一个一个取出
				{
					int startx=tempRecv[i][rectIdx].x/3+(640*(i%3));
					int starty=tempRecv[i][rectIdx].y/2+(540*(i/3));
					int w=tempRecv[i][rectIdx].width/3;
					int h=tempRecv[i][rectIdx].height/2;//取出容器中rect的值
					int endx=startx+w;
					int endy=starty+h;
					cv::rectangle(m6,cvPoint(startx,starty),cvPoint(endx,endy),cvScalar(0,0,0),1);
				}
			}
		}
	}
	if(capidx==MAIN_FPGA_FOUR)
		{
		m4.data=src;
		tempRecv[0].assign(outRect[6].begin(),outRect[6].end());
		tempRecv[1].assign(outRect[7].begin(),outRect[7].end());
		tempRecv[3].assign(outRect[8].begin(),outRect[8].end());
		tempRecv[4].assign(outRect[9].begin(),outRect[9].end());
			for(int i=0;i<6;i++)
			{
				if(i==2||i==5)
				{
					continue;
				}
				if(tempRecv[i].size()!=0)//容器dix不为空
				{
					for(int rectIdx=0;rectIdx<tempRecv[i].size();rectIdx++)//从容器中一个一个取出
					{
						int startx=tempRecv[i][rectIdx].x/3+(640*(i%3));
						int starty=tempRecv[i][rectIdx].y/2+(540*(i/3));
						int w=tempRecv[i][rectIdx].width/3;
						int h=tempRecv[i][rectIdx].height/2;//取出容器中rect的值
						int endx=startx+w;
						int endy=starty+h;
						cv::rectangle(m4,cvPoint(startx,starty),cvPoint(endx,endy),cvScalar(0,0,0),1);
					}
				}
			}
		}
#else
	if(capidx==MAIN_FPGA_SIX)
	{
		for(int i=0;i<6;i++)
		{
			tempRecv[i].assign(outRect[i].begin(),outRect[i].end());
		}
		for(int i=0;i<6;i++)//6个图
		{
			if(tempRecv[i].size()!=0)//容器dix不为空
			{
				for(int rectIdx=0;rectIdx<tempRecv[i].size();rectIdx++)//从容器中一个一个取出
				{
					int x=tempRecv[i][rectIdx].x/3;
					int y=tempRecv[i][rectIdx].y/2;
					int w=tempRecv[i][rectIdx].width/3;
					int h=tempRecv[i][rectIdx].height/2;//取出容器中rect的值
					for(int j=y*1920*1080*cc+(640*(i%3)+x)*cc;
							j<y*1920*1080*cc+(640*(i%3)+x)*cc+w*cc;
							j++)
					{
						src[j]=0;
					}//横第一行
					for(int j=(y+h)*1920*1080*cc+(640*(i%3)+x)*cc;
							j<(y+h)*1920*1080*cc+(640*(i%3)+x)*cc+w*cc;
							j++)
					{
						src[j]=0;
					}//横第二行

					for(int k=0;k<h-2;k++)
					{
						for(int j=(y+1)*1920*1080*cc+(640*(i%3)+x)*cc;
								j<(y+1)*1920*1080*cc+(640*(i%3)+x)*cc+1*cc;
							j+=1920*1080*cc)
						{
							src[j]=0;
						}//竖第一行，除去两行横的第一个像素
					}
					for(int k=0;k<h-2;k++)
					{
						for(int j=(y+1)*1920*1080*cc+(640*(i%3)+x)*cc+w*cc;
								j<(y+1)*1920*1080*cc+(640*(i%3)+x)*cc+1*cc;
							j+=1920*1080*cc)
						{
							src[j]=0;
						}//竖第二行，除去两行横的第一个像素
					}
				}
			}
		}
	}
	else if (capidx==MAIN_FPGA_FOUR)
	{
			tempRecv[0].assign(outRect[6].begin(),outRect[6].end());
			tempRecv[1].assign(outRect[7].begin(),outRect[7].end());
			tempRecv[3].assign(outRect[8].begin(),outRect[8].end());
			tempRecv[4].assign(outRect[9].begin(),outRect[9].end());
		for(int i=0;i<6;i++)//6个图
				{
					if(tempRecv[i].size()!=0)//容器dix不为空
					{
						for(int rectIdx=0;rectIdx<tempRecv[i].size();rectIdx++)//从容器中一个一个取出
						{
							int x=tempRecv[i][rectIdx].x/3;
							int y=tempRecv[i][rectIdx].y/2;
							int w=tempRecv[i][rectIdx].width/3;
							int h=tempRecv[i][rectIdx].height/2;//取出容器中rect的值
							for(int j=y*1920*1080*cc+(640*(i%3)+x)*cc;
									j<y*1920*1080*cc+(640*(i%3)+x)*cc+w*cc;
									j++)
							{
								src[j]=0;
							}//横第一行
							for(int j=(y+h)*1920*1080*cc+(640*(i%3)+x)*cc;
									j<(y+h)*1920*1080*cc+(640*(i%3)+x)*cc+w*cc;
									j++)
							{
								src[j]=0;
							}//横第二行

							for(int k=0;k<h-2;k++)
							{
								for(int j=(y+1)*1920*1080*cc+(640*(i%3)+x)*cc;
										j<(y+1)*1920*1080*cc+(640*(i%3)+x)*cc+1*cc;
									j+=1920*1080*cc)
								{
									src[j]=0;
								}//竖第一行，除去两行横的第一个像素
							}
							for(int k=0;k<h-2;k++)
							{
								for(int j=(y+1)*1920*1080*cc+(640*(i%3)+x)*cc+w*cc;
										j<(y+1)*1920*1080*cc+(640*(i%3)+x)*cc+1*cc;
									j+=1920*1080*cc)
								{
									src[j]=0;
								}//竖第二行，除去两行横的第一个像素
							}
						}
					}
				}
	}
#endif
}

