/*FBOdraw() will be expanded,
 * so I write it in a single file to support GLRender,
 * by this means I can add various of cases.
 *author: Fwy */
#include "GLRender.h"
#include"GLEnv.h"
extern GLEnv env1,env2;
float Rh=0.0;
float Lh=0.0;
void Render::FBOdraw()
{
	int t[10]={0};
	static timeval startT[20]={0};
	static int b=0;
	b++;
	#if 1
	GLEnv &env=env1;
	bool needSendData=true;
	switch(fboMode)
	{
	case  FBO_ALL_VIEW_MODE:
	//	gettimeofday(&startT[4],0);
		RenderRightPanoView(env,0,1080.0*580.0/1080.0+49,1920, 1080.0*216.0/1080.0,MAIN,0,0,0,0,true);
		RenderLeftPanoView(env,0,1080.0*831.0/1080.0+15,1920, 1080.0*216.0/1080.0,MAIN,false);

		if(displayMode!=TRIM_MODE)
		{
			RenderOnetimeView(env,Rh,0,1920*1920/1920, 1080.0*648.0/1080,MAIN);//1152
			p_ChineseCBillBoard->ChooseTga=TURRET_T;
			RenderChineseCharacterBillBoardAt(env,g_windowWidth*1700/1920.0, g_windowHeight*500.0/1080.0, g_windowWidth*0.6,g_windowHeight*0.6);
			p_ChineseCBillBoard->ChooseTga=PANORAMIC_MIRROR_T;
			RenderChineseCharacterBillBoardAt(env,g_windowWidth*1700/1920.0, g_windowHeight*300.0/1080.0, g_windowWidth*0.6,g_windowHeight*0.6);
			DrawGapLine(env);
			RenderRulerView(env,(-3.0*1920.0)/1920.0,1080.0*1030/1080.0,1920.0,1080.0*140.0/1080.0/2.0,RULER_90);
			RenderRulerView(env,(-3.0*1920.0)/1920.0,1080.0*779.0/1080.0+34,1920.0,1080.0*140.0/1080.0/2.0,RULER_180);
		}
//		p_ChineseCBillBoard->ChooseTga=ANGLE_T;
//		RenderChineseCharacterBillBoardAt(env,g_windowWidth*1250/1920.0, g_windowHeight*630.0/1080.0, g_windowWidth*0.85,g_windowHeight*0.85);

	//	RenderMilView(env,g_windowWidth*1700/1920.0, g_windowHeight*500.0/1080.0+Rh, g_windowWidth,g_windowHeight);
	#if 0
		 if(g_windowHeight==768)
		{
			RenderRightPanoView(env,0,g_windowHeight*538.0/768.0,g_windowWidth, g_windowHeight*116.0/768.0,MAIN,0,0,0,0,true);
			RenderLeftPanoView(env,0,g_windowHeight*655.0/768.0,g_windowWidth, g_windowHeight*115.0/768.0,MAIN,false);
		if(displayMode!=TRIM_MODE)
			RenderOnetimeView(env,0,0,g_windowWidth*944.0/1024.0, g_windowHeight*537.0/768.0,MAIN);
		}
		 else
			{
			RenderRightPanoView(env,0,g_windowHeight*648.0/1080.0,g_windowWidth, g_windowHeight*216.0/1080.0,MAIN,0,0,0,0,true);
			RenderLeftPanoView(env,0,g_windowHeight*864.0/1080.0,g_windowWidth, g_windowHeight*216.0/1080.0,MAIN,false);
			if(displayMode!=TRIM_MODE)
			RenderOnetimeView(env,0,0,g_windowWidth*1152/1920, g_windowHeight*648/1080,MAIN);
			}
#endif
		break;
	case	FBO_ALL_VIEW_559_MODE:
	{
#if 0
		 if(g_windowHeight==768)
		{
			RenderRightPanoView(env,0,g_windowHeight*(384.0+2)/768.0,g_windowWidth, g_windowHeight*154.0/768.0,MAIN,0,0,0,0,true);
			RenderLeftPanoView(env,0,g_windowHeight*546.0/768.0,g_windowWidth, g_windowHeight*154.0/768.0,MAIN,false);
			if(displayMode!=TRIM_MODE)
			{
			RenderOnetimeView(env,g_windowWidth*6.0/1024,0,g_windowWidth*348.0/1024.0, g_windowHeight*380.0/768.0,MAIN);
			RenderTwotimesView(env,g_windowWidth*(354.0+6)/1024.0,0,g_windowWidth*348.0/1024.0, g_windowHeight*380.0/768.0,MAIN);
			RenderRulerView(env,g_windowWidth*0/1024.0,g_windowHeight*660/768.0,g_windowWidth,g_windowHeight*150.0/768.0,RULER_90);
			RenderPositionView(env,g_windowWidth*728.0/1024.0,g_windowHeight*340.0/768.0,g_windowWidth,g_windowHeight);

			}//1X 2X  MIWEI
		}
		 else
			{
			RenderRightPanoView(env,0,g_windowHeight*648.0/1080.0,g_windowWidth, g_windowHeight*216.0/1080.0,MAIN,0,0,0,0,true);
			RenderLeftPanoView(env,0,g_windowHeight*864.0/1080.0,g_windowWidth, g_windowHeight*216.0/1080.0,MAIN,false);
			if(displayMode!=TRIM_MODE)
			{
				RenderOnetimeView(env,0,0,g_windowWidth*1152/1920, g_windowHeight*648/1080,MAIN);
			RenderTwotimesView(env,0,0,g_windowWidth*944.0/1024.0, g_windowHeight*537.0/768.0,MAIN);
			RenderRulerView(env,g_windowWidth*0/1920.0,g_windowHeight*540/1080.0,g_windowWidth,g_windowHeight*140.0/1080,RULER_90);
			RenderPositionView(env,g_windowWidth*728.0/1024.0,g_windowHeight*340.0/768.0,g_windowWidth,g_windowHeight);
			}
			//		RenderRulerView(env,g_windowWidth*0/1920.0,g_windowHeight*540/1080.0,g_windowWidth,g_windowHeight*140.0/1080,RULER_180);
			//1X 2X  MIWEI
			}
#endif
		 break;
	}
	}
#endif
}

