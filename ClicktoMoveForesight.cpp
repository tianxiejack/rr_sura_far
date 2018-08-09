/*
 * ClicktoMoveForesight.cpp
 *
 *  Created on: 2018年7月16日
 *      Author: fsmdn121
 */

#include"ClicktoMoveForesight.h"
#include "stdio.h"
#include "GLRender.h"
#include "ForeSight.h"
extern Render render;
extern ForeSightPos foresightPos[MS_COUNT];

#define nowW	1024
#define nowH	768

void clicktoMoveForesight(int x,int y,int mainOrsub)
{
int dd=2;
int touch_delta_angle=13;
	#if USE_BMPCAP
	float Xangle=(float)(x*360.0/nowW);
	float Yangle=(float)(y*360.0/nowH);
#else
	float Xangle=(float)(x/1920.0*1024.0*360.0/nowW);
	float Yangle=(float)(y/1080.0*768.0*360.0/nowH);
#endif
	if(Yangle>=288+touch_delta_angle-dd && Yangle<=347+touch_delta_angle)//Up
	{
		foresightPos[mainOrsub].SetPos(Xangle,Yangle,mainOrsub);
	}
	else if(Yangle>=216+touch_delta_angle-dd && Yangle<=275+touch_delta_angle)//Down
	{
		foresightPos[mainOrsub].SetPos(Xangle,Yangle,mainOrsub);
	}
}
