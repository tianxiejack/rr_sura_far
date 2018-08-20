/*FBOdraw() will be expanded,
 * so I write it in a single file to support GLRender,
 * by this means I can add various of cases.
 *author: Fwy */
#include "GLRender.h"
#include"GLEnv.h"
extern GLEnv env1,env2;
float Rh=0;
float Lh=0;
void Render::FBOdraw()
{
	#if 1
	GLEnv &env=env1;
	bool needSendData=true;
	switch(fboMode)
	{
	case  FBO_ALL_VIEW_MODE:
	//	gettimeofday(&startT[4],0);
		RenderRightPanoView(env,0,1080.0*664.0/1080.0,1920, 1080.0*216.0/1080.0,MAIN,0,0,0,0,true);
		RenderLeftPanoView(env,0,1080.0*881.0/1080.0,1920, 1080.0*216.0/1080.0,MAIN,false);

		if(displayMode!=TRIM_MODE)
		{
			RenderOnetimeView(env,0,0,1920*1920/1920, 1080.0*648.0/1080,MAIN);//1152
			p_ChineseCBillBoard->ChooseTga=TURRET_T;
			RenderChineseCharacterBillBoardAt(env,g_windowWidth*1700/1920.0, g_windowHeight*500.0/1080.0, g_windowWidth*0.6,g_windowHeight*0.6);
			p_ChineseCBillBoard->ChooseTga=PANORAMIC_MIRROR_T;
			RenderChineseCharacterBillBoardAt(env,g_windowWidth*1700/1920.0, g_windowHeight*300.0/1080.0, g_windowWidth*0.6,g_windowHeight*0.6);
		}
		RenderRulerView(env,(-3.0*1920.0)/1920.0,1080.0*847.0/1080.0,1920.0,1080.0*140.0/1080.0/2.0,RULER_90);
		RenderRulerView(env,(-3.0*1920.0)/1920.0,1080.0*630.0/1080.0,1920.0,1080.0*140.0/1080.0/2.0,RULER_180);
		NoSigInf();
		break;
	case	FBO_ALL_VIEW_559_MODE:
	{
		 break;
	}
	}
#endif
}

