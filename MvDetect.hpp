#ifndef MVDETECT_HPP_
#define MVDETECT_HPP_
#include"StlGlDefines.h"
#include<vector>
using namespace std;
//inNumber 代表创建 检测OBJ的实例个数  有效值从1到8
extern void createDetect(unsigned char inNumber,int inwidth,int inheight);
extern void exitDetect();
extern void mvDetect(unsigned char index,unsigned char* inframe,int width,int height,cv::Rect *boundRect);

//index 代表第几个 检测OBJ 执行，boundRect 输出 目标的矩形框参数
//extern void mvDetect(unsigned char index,unsigned char* inframe,int width,int height,vector<cv::Rect> *boundRect);




class MvDetect
{
public:
	MvDetect();
	~MvDetect();
	void yuyv2gray(unsigned char* src,unsigned char* dst,int width=MAX_SCREEN_WIDTH,int height=MAX_SCREEN_HEIGHT);
	void init(int w=MAX_SCREEN_WIDTH,int h=MAX_SCREEN_HEIGHT);
	void m_mvDetect(int idx,unsigned char* inframe,int w=MAX_SCREEN_WIDTH,int h=MAX_SCREEN_HEIGHT);
	void DrawRectOnpic(unsigned char *src,int capidx);
	void selectFrame(unsigned char *dst,unsigned char *src,int targetId,int camIdx);
	void saveConfig();
	void ReadConfig();
	bool GetMD(int mainorsub){return  enableMD[mainorsub];};
	void SetMD(bool tof,int mainorsub){enableMD[mainorsub]=tof;};
	void OpenMD(int mainorsub){MDopen[mainorsub]=true;};
	void CloseMD(int mainorsub){MDopen[mainorsub]=false;};
	bool CanUseMD(int mainorsub);
	bool MDisStart(){enableMD[MAIN]==true && enableMD[SUB]==true;};
	int getTargetNum(int cam_idx){ targetnum[cam_idx]=outRect[cam_idx].size();
		return targetnum[cam_idx];}
	int Choosetargetidx(int cam_idx,int tidx){
		if(targetidx[cam_idx][tidx]<getTargetNum(cam_idx))
			targetidx[cam_idx][tidx]++;
		else
			targetidx[cam_idx][tidx]=0;
		return targetidx[cam_idx][tidx];};
	void SetoutRect(int idx);
private:
	int targetidx[CAM_COUNT][4];
	int targetnum[CAM_COUNT];
	bool enableMD[2];
	bool MDopen[2];
	static const int MAX_RECT_COUNT=6;
	typedef struct{cv::Rect rects[MAX_RECT_COUNT];}RECT_Array;
	RECT_Array tempoutRect[CAM_COUNT];
	std::vector<cv::Rect> outRect[CAM_COUNT];
	unsigned char* grayFrame[CAM_COUNT];
};
#endif
