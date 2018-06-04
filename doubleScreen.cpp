#include"RenderMain.h"
#include "GLRender.h"
#include "common.h"
#include "GLEnv.h"
#include"Thread_Priority.h"
#include "thread_idle.h"
#include"MvDetect.hpp"
extern thread_idle tIdle;
extern Render render;
extern MvDetect mv_detect;
Common comSecondSC;
extern float forward_data;
extern GLEnv env2,env1;
extern ForeSightPos foresightPos[MS_COUNT];
extern char chosenCam[2];
void InitBowlDS()
{

}


void Render::RenderSceneDS()
{
#if MVDECT
	if(mv_detect.CanUseMD(SUB) ||mv_detect.CanUseMD(MAIN))
	{
		tIdle.threadRun(MVDECT_CN);
	}
	else
	{
		tIdle.threadIdle(MVDECT_CN);
	}
#endif
	static bool setpriorityOnce=true;
	if(setpriorityOnce)
	{
		setCurrentThreadHighPriority(THREAD_L_S_RENDER);
		setpriorityOnce=false;
	}
	int t[10]={0};
	static timeval startT[20]={0};

	//GLEnv &env=env2;
	GLEnv &env=env1;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	switch(SecondDisplayMode)
	{
	case SECOND_ALL_VIEW_MODE:
#if 1
#if MVDECT
		if(mv_detect.CanUseMD(SUB))
		{
	//		mv_detect.SetoutRect();
		}
#endif
		tIdle.threadIdle(SUB_CN);
		env.Getp_FboPboFacade()->Render2Front(SUB,g_subwindowWidth,g_subwindowHeight);
		if(g_windowHeight==768)
		{
				RenderRightForeSightView(env,0,g_subwindowHeight*538.0/768.0,g_subwindowWidth, g_subwindowHeight*116.0/768.0,SUB);
				RenderLeftForeSightView(env,0,g_subwindowHeight*655.0/768.0,g_subwindowWidth, g_subwindowHeight*115.0/768.0,SUB);
				glScissor(0,0,944,538);
							//glScissor(g_subwindowWidth*448.0/1920.0,g_subwindowHeight*156.0/1080.0,g_subwindowWidth*1024,g_subwindowHeight*537);
				glEnable(GL_SCISSOR_TEST);
				glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
				glDisable(GL_SCISSOR_TEST);
				RenderOnetimeView(env,0,0,g_subwindowWidth*944.0/1024.0, g_subwindowHeight*537.0/768.0,SUB);

		}
		else
		{
			RenderRightForeSightView(env,0,g_subwindowHeight*643.0/1080.0,g_subwindowWidth, g_subwindowHeight*216.0/1080.0,SUB);
			RenderLeftForeSightView(env,0,g_subwindowHeight*864.0/1080.0,g_subwindowWidth, g_subwindowHeight*216.0/1080.0,SUB);
			glScissor(0,0,1152,648);
									//glScissor(g_subwindowWidth*448.0/1920.0,g_subwindowHeight*156.0/1080.0,g_subwindowWidth*1024,g_subwindowHeight*537);
			glEnable(GL_SCISSOR_TEST);
			glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
			glDisable(GL_SCISSOR_TEST);
			RenderOnetimeView(env,0,0,g_subwindowWidth*1152.0/1920.0, g_subwindowHeight*648.0/1080,SUB);

		}

#endif
				break;
	case SECOND_559_ALL_VIEW_MODE:

		tIdle.threadIdle(SUB_CN);
		env.Getp_FboPboFacade()->Render2Front(SUB,g_subwindowWidth,g_subwindowHeight);
		RenderRightForeSightView(env,0,g_subwindowHeight*538.0/768.0,g_subwindowWidth, g_subwindowHeight*116.0/768.0,SUB);
		RenderLeftForeSightView(env,0,g_subwindowHeight*655.0/768.0,g_subwindowWidth, g_subwindowHeight*115.0/768.0,SUB);

		glScissor(0,0,1920,380);
			//glScissor(g_subwindowWidth*448.0/1920.0,g_subwindowHeight*156.0/1080.0,g_subwindowWidth*1024,g_subwindowHeight*537);
		glEnable(GL_SCISSOR_TEST);
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
		glDisable(GL_SCISSOR_TEST);
		RenderOnetimeView(env,g_windowWidth*6.0/1024,0,g_windowWidth*348.0/1024.0, g_windowHeight*380.0/768.0,SUB);
		RenderTwotimesView(env,g_windowWidth*(354.0+6)/1024.0,0,g_windowWidth*348.0/1024.0, g_windowHeight*380.0/768.0,SUB);
		RenderPositionView(env,g_windowWidth*728.0/1024.0,g_windowHeight*340.0/768.0,g_windowWidth,g_windowHeight);

		break;
	case	SECOND_CHOSEN_VIEW_MODE:
		tIdle.threadRun(SUB_CN);
	//	RenderChosenView(env,g_subwindowWidth*448.0/1920.0,g_subwindowHeight*156.0/1080.0,g_subwindowWidth*1024.0/1920.0, g_subwindowHeight*768.0/1920.0,true);
		RenderChosenView(env,0,0,g_subwindowWidth, g_subwindowHeight,SUB,true);
		break;
	case 	SECOND_TELESCOPE_FRONT_MODE:
		tIdle.threadIdle(SUB_CN);
		p_ForeSightFacade2[SUB]->Reset(TELESCOPE_FRONT_MODE,SUB);
			    RenderRulerView(env,-g_subwindowWidth*3.0/1920.0,g_subwindowHeight*980.0/1080.0,g_subwindowWidth,g_subwindowHeight*140.0/1080.0,RULER_45);
				RenderPanoTelView(env,0,g_subwindowHeight*478.0/1080,g_subwindowWidth, g_subwindowHeight*592.0/1080.0,FRONT,SUB);
#if			MVDECT
				if(mv_detect.CanUseMD(SUB))
				{
			//		mv_detect.SetoutRect();
				//	TargectTelView(env,g_subwindowWidth*60/1920.0,g_subwindowHeight*39.0/1080.0,g_subwindowWidth*480.0/1920.0, g_subwindowHeight*400.0/1080.0,0,0,0,SUB);
			//		TargectTelView(env,g_subwindowWidth*560/1920.0,g_subwindowHeight*39.0/1080.0,g_subwindowWidth*480.0/1920.0, g_subwindowHeight*400.0/1080.0,1,1,1,SUB);
				}
#endif
				RenderPositionView(env,g_subwindowWidth*0,g_subwindowHeight*0,g_subwindowWidth, g_subwindowHeight);

	break;
	case	SECOND_TELESCOPE_RIGHT_MODE:
		tIdle.threadIdle(SUB_CN);
			p_ForeSightFacade2[SUB]->Reset(TELESCOPE_RIGHT_MODE,SUB);
			   RenderRulerView(env,-g_subwindowWidth*3.0/1920.0,g_subwindowHeight*980.0/1080.0,g_subwindowWidth,g_subwindowHeight*140.0/1080.0,RULER_45);
				RenderPanoTelView(env,0,g_subwindowHeight*478.0/1080,g_subwindowWidth, g_subwindowHeight*592.0/1080.0,RIGHT,SUB);
#if			MVDECT
				if(mv_detect.CanUseMD(SUB))
						{
				//			mv_detect.SetoutRect();
				//			TargectTelView(env,g_subwindowWidth*60/1920.0,g_subwindowHeight*39.0/1080.0,g_subwindowWidth*480.0/1920.0, g_subwindowHeight*400.0/1080.0,0,0);
				//			TargectTelView(env,g_subwindowWidth*560/1920.0,g_subwindowHeight*39.0/1080.0,g_subwindowWidth*480.0/1920.0, g_subwindowHeight*400.0/1080.0,1,1);
						}
#endif
				RenderPositionView(env,g_subwindowWidth*0,g_subwindowHeight*0,g_subwindowWidth, g_subwindowHeight);
	break;
	case	SECOND_TELESCOPE_BACK_MODE:
		tIdle.threadIdle(SUB_CN);
		p_ForeSightFacade2[SUB]->Reset(TELESCOPE_BACK_MODE,SUB);
		   RenderRulerView(env,-g_subwindowWidth*3.0/1920.0,g_subwindowHeight*980.0/1080.0,g_subwindowWidth,g_subwindowHeight*140.0/1080.0,RULER_45);
		   RenderPanoTelView(env,0,g_subwindowHeight*478.0/1080,g_subwindowWidth, g_subwindowHeight*592.0/1080.0,BACK,SUB);
#if			MVDECT
		   if(mv_detect.CanUseMD(SUB))
					{
			//			mv_detect.SetoutRect();
			//			TargectTelView(env,g_subwindowWidth*60/1920.0,g_subwindowHeight*39.0/1080.0,g_subwindowWidth*480.0/1920.0, g_subwindowHeight*400.0/1080.0,0,0);
			//			TargectTelView(env,g_subwindowWidth*560/1920.0,g_subwindowHeight*39.0/1080.0,g_subwindowWidth*480.0/1920.0, g_subwindowHeight*400.0/1080.0,1,1);
					}
#endif
			RenderPositionView(env,g_subwindowWidth*0,g_subwindowHeight*0,g_subwindowWidth, g_subwindowHeight);


	break;
	case	SECOND_TELESCOPE_LEFT_MODE:
		tIdle.threadIdle(SUB_CN);
		p_ForeSightFacade2[SUB]->Reset(TELESCOPE_LEFT_MODE,SUB);
		  RenderRulerView(env,-g_subwindowWidth*3.0/1920.0,g_subwindowHeight*980.0/1080.0,g_subwindowWidth,g_subwindowHeight*140.0/1080.0,RULER_45);
			RenderPanoTelView(env,0,g_subwindowHeight*478.0/1080,g_subwindowWidth, g_subwindowHeight*592.0/1080.0,LEFT,SUB);
#if			MVDECT
			if(mv_detect.CanUseMD(SUB))
					{
			//		mv_detect.SetoutRect();
			//			TargectTelView(env,g_subwindowWidth*60/1920.0,g_subwindowHeight*39.0/1080.0,g_subwindowWidth*480.0/1920.0, g_subwindowHeight*400.0/1080.0,0,0);
			//			TargectTelView(env,g_subwindowWidth*560/1920.0,g_subwindowHeight*39.0/1080.0,g_subwindowWidth*480.0/1920.0, g_subwindowHeight*400.0/1080.0,1,1);
					}
#endif
					RenderPositionView(env,g_subwindowWidth*0,g_subwindowHeight*0,g_subwindowWidth, g_subwindowHeight);
	break;
	default :
		break;
	}
		 if(SecondDisplayMode==	SECOND_559_ALL_VIEW_MODE)
		{
			p_ChineseCBillBoard->ChooseTga=ONEX_REALTIME_T;
			RenderChineseCharacterBillBoardAt(env,-g_windowWidth*1050.0/1920.0, g_windowHeight*120.0/1080.0, g_windowWidth*1344.0/1920.0,g_windowHeight*1536.0/1920.0);
			p_ChineseCBillBoard->ChooseTga=TWOX_REALTIME_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.0/1920.0,g_windowHeight*120.0/1080.0, g_windowWidth*1344.0/1920.0,g_windowHeight*1536.0/1920.0);

			p_ChineseCBillBoard->ChooseTga=ANGLE_T;
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*999.0/1920.0, g_windowHeight*174.0/1080.0, g_windowWidth*900.0/1920.0,g_windowHeight*980.0/1080.0);

			//	p_ChineseCBillBoard->ChooseTga=LOCATION_T;
			//		RenderChineseCharacterBillBoardAt(env,g_windowWidth*950.0/1920.0, g_windowHeight*50/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);

		}

		else if(SecondDisplayMode==SECOND_TELESCOPE_FRONT_MODE
				||SecondDisplayMode==SECOND_TELESCOPE_RIGHT_MODE
				||SecondDisplayMode==SECOND_TELESCOPE_BACK_MODE
				||SecondDisplayMode==SECOND_TELESCOPE_LEFT_MODE)
		{
			/*
		p_ChineseCBillBoard->ChooseTga=TWOX_REALTIME_T;
		RenderChineseCharacterBillBoardAt(env,-g_windowWidth*1050.0/1920.0, g_windowHeight*120.0/1080.0, g_windowWidth*1344.0/1920.0,g_windowHeight*1536.0/1920.0);

			p_ChineseCBillBoard->ChooseTga=FOURX_REALTIME_T;
			RenderChineseCharacterBillBoardAt(env,g_windowWidth*0.0/1920.0,g_windowHeight*120.0/1080.0, g_windowWidth*1344.0/1920.0,g_windowHeight*1536.0/1920.0);
	*/
		p_ChineseCBillBoard->ChooseTga=ANGLE_T;
		RenderChineseCharacterBillBoardAt(env,g_windowWidth*999.0/1920.0, g_windowHeight*174.0/1080.0, g_windowWidth*900.0/1920.0,g_windowHeight*980.0/1080.0);

			 if(SecondDisplayMode==SECOND_TELESCOPE_FRONT_MODE)
			{
				p_ChineseCBillBoard->ChooseTga=RADAR_FRONT_T;
			//	if(mv_detect.CanUseMD())
			//		RenderChineseCharacterBillBoardAt(g_windowWidth*750.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
			//	else
				RenderChineseCharacterBillBoardAt(env,g_windowWidth*200.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
			}
			else if(SecondDisplayMode==SECOND_TELESCOPE_RIGHT_MODE)
			{
					p_ChineseCBillBoard->ChooseTga=RADAR_RIGHT_T;
					//	if(mv_detect.CanUseMD())
							//		RenderChineseCharacterBillBoardAt(g_windowWidth*750.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
							//	else
								RenderChineseCharacterBillBoardAt(env,g_windowWidth*200.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
			}
			else if(displayMode==SECOND_TELESCOPE_BACK_MODE)
			{
				p_ChineseCBillBoard->ChooseTga=RADAR_BACK_T;
				//	if(mv_detect.CanUseMD())
						//		RenderChineseCharacterBillBoardAt(g_windowWidth*750.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
						//	else
							RenderChineseCharacterBillBoardAt(env,g_windowWidth*200.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
			}
			else if(SecondDisplayMode==SECOND_TELESCOPE_LEFT_MODE)
			{
			p_ChineseCBillBoard->ChooseTga=RADAR_LEFT_T;
			//	if(mv_detect.CanUseMD())
					//		RenderChineseCharacterBillBoardAt(g_windowWidth*750.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
					//	else
						RenderChineseCharacterBillBoardAt(env,g_windowWidth*200.0/1920.0, g_windowHeight*200/1920.0, g_windowWidth*1000.0/1920.0,g_windowWidth*798.0/1920.0);
			}
	}

}
void Render::SetupRCDS(int windowWidth, int windowHeight)
{
#if 0
//#define PANO_FLOAT_DATA_FILENAME "panofloatdata.yml"
	GLEnv &env=env2;
	GLubyte *pBytes;
#if USE_CPU
	GLint nWidth=DEFAULT_IMAGE_WIDTH, nHeight=DEFAULT_IMAGE_HEIGHT, nComponents=GL_RGB8;
	GLenum format= GL_BGR;
#else
	GLint nWidth=DEFAULT_IMAGE_WIDTH, nHeight=DEFAULT_IMAGE_HEIGHT, nComponents=GL_RGBA8;
	GLenum format= GL_BGRA;
#endif
	if(!shaderManager.InitializeStockShaders()){
		cout<<"failed to intialize shaders"<<endl;
		exit(1);
	}

	if(!env.Getp_PBOMgr()->Init()
				|| !env.Getp_PBOExtMgr()->Init()
				||!env.Getp_PBORcr()->Init()
				|| !env.Getp_PBOVGAMgr()->Init()
				|| !env.Getp_PBOSDIMgr()->Init()
				||!env.Getp_PBOChosenMgr()->Init()){
		cout<<"Failed to init PBO manager"<<endl;
			exit(1);
		}
		if(!env.Getp_FBOmgr()->Init())
		{
			printf("FBO init failed\n");
			exit(-1);
		}
#if 1
	glClearColor(0.0f, 0/255.0f, 0.0f, 1.0f);//25/255.0f, 25/255.0f, 112/255.0f, 0.0f);
	glLineWidth(1.5f);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);     //enable blending
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glClearDepth(1.0);				/* Enables Clearing Of The Depth Buffer*/
	glDepthFunc(GL_LESS);			        /* The Type Of Depth Test To Do*/
	glEnable(GL_DEPTH_TEST);		        /* Enables Depth Testing*/
	glShadeModel(GL_SMOOTH);			/* Enables Smooth Color Shading*/
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	SetView(windowWidth, windowHeight);
#endif
	{//setting up models and their textures
#if 0
		overLapRegion::GetoverLapRegion()->SetSingleHightLightState(false);
			ReadPanoFloatDataFromFile(PANO_FLOAT_DATA_FILENAME);
#endif
		env.GettransformPipeline()->SetMatrixStacks(*(env.GetmodelViewMatrix()), *(env.GetprojectionMatrix()));
		InitLineofRuler(env);

#if 1
		GenerateCenterView();
		GenerateCompassView();
		GenerateScanPanelView();
		GeneratePanoView();

		GenerateTriangleView();
		GeneratePanoTelView();
		GenerateTrack();

		GenerateLeftPanoView();
		GenerateRightPanoView();
#endif
		GenerateLeftSmallPanoView();
		GenerateRightSmallPanoView();
#if 1
		float x;
		x=(p_LineofRuler->Load())/360.0*(render.get_PanelLoader().Getextent_pos_x()-render.get_PanelLoader().Getextent_neg_x());
		RulerAngle=p_LineofRuler->Load();\
		GenerateOnetimeView();
		GenerateOnetimeView2();

		GenerateTwotimesView();
		GenerateTwotimesView2();

		GenerateTwotimesTelView();
		GenerateFourtimesTelView();
		GenerateCheckView();

		GenerateBirdView();
		GenerateFrontView();
		GenerateRearTopView();
		GenerateExtentView();

		GenerateSDIView();
		GenerateVGAView();
		GenerateChosenView();
#endif

		PanoLen=(PanelLoader.Getextent_pos_x()-PanelLoader.Getextent_neg_x());
		PanoHeight=(PanelLoader.Getextent_pos_z()-PanelLoader.Getextent_neg_z());
	//	foresightPos.SetPanoLen_Height(PanoLen,PanoHeight);
//		zodiac_msg.setPanoHeight_Length(PanoHeight,PanoLen);

	//	camonforesight.setPanoheight(PanoHeight);
//		panocamonforesight.setPanoheight(PanoHeight);
//		panocamonforesight.setPanolen(PanoLen);
//		telcamonforesight.setPanoheight(PanoHeight);
//		telcamonforesight.setPanolen(PanoLen);
	//camonforesight.setPanolen(PanoLen);

		InitALPHA_ZOOM_SCALE();
	//	InitBowl();
		InitScanAngle();

		InitPanoScaleArrayData();
		InitPanel(env);
		InitFollowCross();
		InitRuler(env);
		InitCalibrate();
	//	InitOitVehicle(env);
	//	    glmDelete(VehicleLoader);
	//	pVehicle->initFBOs(windowWidth, windowHeight);

		InitShadow(env);
		InitBillBoard(env);
//		InitFrontTracks();
	//	InitWheelTracks();
	//	InitCrossLines();
		InitWealTrack();
		InitDynamicTrack(env);
		InitCornerMarkerGroup(env);
		initAlphaMask();
//		InitDataofAlarmarea();

				FILE *fp;
				char read_data[20];
				fp=fopen("forward.yml","r");
				if(fp!=NULL)
				{
					fscanf(fp,"%f\n",&forward_data);
					fclose(fp);
					printf("forward:%f\n",forward_data);
				}
		InitForesightGroupTrack(env);
		DrawNeedleonCompass(env);
		DrawTriangle(env);

		pthread_t th_rec;
	   	int arg_rec = 10;
	 	env.Set_FboPboFacade(*(env.Getp_FBOmgr()),*(env.Getp_PBORcr()));
	   	mPresetCamGroup.LoadCameras();
		// Load up CAM_COUNT textures
		glGenTextures(PETAL_TEXTURE_COUNT, textures);

#if WHOLE_PIC
		for(int i = 0; i < 1; i++){
				glBindTexture(GL_TEXTURE_2D, textures[i]);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
				glTexImage2D(GL_TEXTURE_2D,0,nComponents,PANO_TEXTURE_WIDTH, PANO_TEXTURE_HEIGHT, 0,
											format, GL_UNSIGNED_BYTE, 0);
			}
#else
		for(int i = 0; i < CAM_COUNT; i++){
			glBindTexture(GL_TEXTURE_2D, textures[i]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
			if(i==1)
			{
			glTexImage2D(GL_TEXTURE_2D,0,nComponents,1280, 1080, 0,
										format, GL_UNSIGNED_BYTE, 0);
			}
			else
			glTexImage2D(GL_TEXTURE_2D,0,nComponents,PANO_TEXTURE_WIDTH, PANO_TEXTURE_HEIGHT, 0,
								format, GL_UNSIGNED_BYTE, 0);
		}
#endif


		// Alpha mask: 1/16 size of 1920x1080
		glBindTexture(GL_TEXTURE_2D, textures[ALPHA_TEXTURE_IDX]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,ALPHA_MASK_WIDTH, ALPHA_MASK_HEIGHT, 0,
				GL_RGBA, GL_UNSIGNED_BYTE, alphaMask);

		glBindTexture(GL_TEXTURE_2D, textures[ALPHA_TEXTURE_IDX0]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,ALPHA_MASK_WIDTH, ALPHA_MASK_HEIGHT, 0,
				GL_RGBA, GL_UNSIGNED_BYTE, alphaMask0);

		glBindTexture(GL_TEXTURE_2D, textures[ALPHA_TEXTURE_IDX1]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,ALPHA_MASK_WIDTH, ALPHA_MASK_HEIGHT, 0,
				GL_RGBA, GL_UNSIGNED_BYTE, alphaMask1);
	}
	//setting up extension textures etc.
	{
		glGenTextures(EXTENSION_TEXTURE_COUNT, extensionTextures);
		for(int i = 0; i < EXTENSION_TEXTURE_COUNT; i++){
			glBindTexture(GL_TEXTURE_2D, extensionTextures[i]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
			glTexImage2D(GL_TEXTURE_2D,0,nComponents,1920, 1080, 0,
					format, GL_UNSIGNED_BYTE, 0);
		}

		glGenTextures(VGA_TEXTURE_COUNT, VGATextures);
				for(int i = 0; i < VGA_TEXTURE_COUNT; i++){
					glBindTexture(GL_TEXTURE_2D, VGATextures[i]);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
					glTexImage2D(GL_TEXTURE_2D,0,nComponents,VGA_WIDTH, VGA_HEIGHT, 0,
							format, GL_UNSIGNED_BYTE, 0);
				}

		glGenTextures(SDI_TEXTURE_COUNT, SDITextures);
				for(int i = 0; i < SDI_TEXTURE_COUNT; i++){
					glBindTexture(GL_TEXTURE_2D, SDITextures[i]);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
					glTexImage2D(GL_TEXTURE_2D,0,nComponents,SDI_WIDTH, SDI_HEIGHT, 0,
							format, GL_UNSIGNED_BYTE, 0);
				}
				glGenTextures(CHOSEN_TEXTURE_COUNT, GL_ChosenTextures);
								for(int i = 0; i < CHOSEN_TEXTURE_COUNT; i++){
									glBindTexture(GL_TEXTURE_2D, GL_ChosenTextures[i]);
									glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
									glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
									glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
									glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
									glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
									glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
									glTexImage2D(GL_TEXTURE_2D,0,nComponents,SDI_WIDTH, SDI_HEIGHT, 0,
											format, GL_UNSIGNED_BYTE, 0);
								}

#if USE_COMPASS_ICON
		glGenTextures(1, iconTextures);
		for(int i = 0; i < 1; i++){
			glBindTexture(GL_TEXTURE_2D, iconTextures[i]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
			glTexImage2D(GL_TEXTURE_2D,0,nComponents,nWidth, nHeight, 0,
					format, GL_UNSIGNED_BYTE, 0);
		}
#endif

		glGenTextures(1, iconRuler45Textures);
		for(int i = 0; i < 1; i++){
			glBindTexture(GL_TEXTURE_2D, iconRuler45Textures[i]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
			glTexImage2D(GL_TEXTURE_2D,0,nComponents,PAL_WIDTH, PAL_HEIGHT, 0,
					format, GL_UNSIGNED_BYTE, 0);
		}

		glGenTextures(1, iconRuler90Textures);
		for(int i = 0; i < 1; i++){
			glBindTexture(GL_TEXTURE_2D, iconRuler90Textures[i]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
			glTexImage2D(GL_TEXTURE_2D,0,nComponents,PAL_WIDTH, PAL_HEIGHT, 0,
					format, GL_UNSIGNED_BYTE, 0);
		}

		glGenTextures(1, iconRuler180Textures);
		for(int i = 0; i < 1; i++){
			glBindTexture(GL_TEXTURE_2D, iconRuler180Textures[i]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
			glTexImage2D(GL_TEXTURE_2D,0,nComponents,PAL_WIDTH, PAL_HEIGHT, 0,
					format, GL_UNSIGNED_BYTE, 0);
		}

	}
	glMatrixMode(GL_MODELVIEW);
#endif
}
void Render::ProcessOitKeysDS(GLEnv &m_env,unsigned char key, int x, int y)
{
	switch(key)
		{
	case '(':
				foresightPos[SUB].SetSpeedX((render.get_PanelLoader().Getextent_pos_x()-render.get_PanelLoader().Getextent_neg_x())/1920.0*10.0);
					break;
			case ')':
				foresightPos[SUB].SetSpeedY((render.get_PanelLoader().Getextent_pos_z()-render.get_PanelLoader().Getextent_neg_z())/1920.0*20.0);
				break;
			case'N':
			{
				SECOND_DISPLAY nextMode=SECOND_DISPLAY(((int)SecondDisplayMode+1)%SECOND_TOTAL_MODE_COUNT);
				if(nextMode==SECOND_559_ALL_VIEW_MODE)
				{
					nextMode=SECOND_ALL_VIEW_MODE;
				}
				SecondDisplayMode = nextMode;
			}
				break;
			case	'n':
			{
#if USE_CAP_SPI
				chosenCam[SUB]=(chosenCam[SUB]+1)%CAM_COUNT+1;
				ChangeSubChosenCamidx(chosenCam[SUB]);
#endif
			}
			break;
			case '1':
					if(SecondDisplayMode==	SECOND_ALL_VIEW_MODE
							||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE)
						p_ForeSightFacade[SUB]->MoveLeft(-PanoLen*100.0,SUB);
				//	else if(SecondDisplayMode==SECOND_TELESCOPE_FRONT_MODE)

						break;
				case '2':
					if(SecondDisplayMode==	SECOND_ALL_VIEW_MODE
							||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE)
						p_ForeSightFacade[SUB]->MoveRight(PanoLen*100.0,SUB);
			//		else if(SecondDisplayMode==SECOND_TELESCOPE_RIGHT_MODE)
									break;
				case '3':
					if(SecondDisplayMode==	SECOND_ALL_VIEW_MODE
							||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE)
						p_ForeSightFacade[SUB]->MoveUp(PanoHeight/(OUTER_RECT_AND_PANO_TWO_TIMES_CAM_LIMIT),SUB);
				//	else if(SecondDisplayMode==SECOND_TELESCOPE_BACK_MODE)
									break;
				case '4':
					if(SecondDisplayMode==	SECOND_ALL_VIEW_MODE
							||SecondDisplayMode==SECOND_559_ALL_VIEW_MODE)
						p_ForeSightFacade[SUB]->MoveDown(-PanoHeight/(OUTER_RECT_AND_PANO_TWO_TIMES_CAM_LIMIT),SUB);
			//		else if(SecondDisplayMode==SECOND_TELESCOPE_LEFT_MODE)
									break;
				case 'O':
#if MVDECT
			mv_detect.OpenMD(SUB);
#endif
					break;
				case 'o':
#if MVDECT
			mv_detect.CloseMD(SUB);
#endif
					break;

			default:
		break;
		}
}
void Render::GetFPSDS()
{
	/* Number of samples for frame rate */
#define FR_SAMPLES 10

	static struct timeval last={0,0};
	struct timeval now;
	float delta;
	if (comSecondSC.plusAndGetFrameCount() >= FR_SAMPLES) {
		gettimeofday(&now, NULL);
		delta= (now.tv_sec - last.tv_sec +(now.tv_usec - last.tv_usec)/1000000.0);
		last = now;
		comSecondSC.setFrameRate(FR_SAMPLES / delta);
		comSecondSC.setFrameCount(0);
	}
}
void Render::DrawGLSceneDS()
{
	char arg1[128],arg2[128];
		RenderSceneDS();
		glutSwapBuffers();
		glFinish();
		GetFPSDS();  /* Get frame rate stats */

		/* Copy saved window name into temp string arg1 so that we can add stats */
		strcpy (arg1, comSecondSC.getWindowName());

		if (sprintf(arg2, "%.2f FPS", comSecondSC.getFrameRate()))
		{
			strcat (arg1, arg2);
		}

		/* cut down on the number of redraws on window title.  Only draw once per sample*/
		if (comSecondSC.isCountUpdate())
		{
			glutSetWindowTitle(arg1);
		}
}

void Render::ReSizeGLSceneDS(int Width, int Height)
{
	if (Height==0)	/* Prevent A Divide By Zero If The Window Is Too Small*/
		Height=1;
	ChangeSizeDS(Width, Height);
	comSecondSC.setUpdate(GL_YES);
}
void Render::keyPressedDS(GLEnv &m_env,unsigned char key, int x, int y)
{
	usleep(100);
	ProcessOitKeysDS(m_env,key, x, y);
}



void RenderMain::ReSizeGLSceneDS(int Width, int Height)
{
	render.ReSizeGLSceneDS(Width,Height);
}
void RenderMain::DrawGLSceneDS()
{
		render.DrawGLSceneDS();
		glutPostRedisplay();
}
void RenderMain::keyPressedDS(unsigned char key, int x, int y)
{
	GLEnv &env=env2;
	render.keyPressedDS(env,key,x,y);
}
	void 	RenderMain::doubleScreenInit(int argc, char **argv)
	{
		char arg1[256], arg2[256];
	//	glutInit (&argc, argv);
		glutCreateSubWindow(2,0,0,1920,1080);
		glutInitWindowPosition(1921,0);
		glutInitWindowSize(1920, 1080);

		glutInitDisplayMode ( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

		sprintf(arg1,"Second %s (%s, %s):",VERSION_STRING, __DATE__,__TIME__);
		strcat (arg1, argv[1]);
		strcat (arg1, "+");
		strcat (arg1, argv[2]);

			/* getting a warning here about passing arg1 of sprinf incompatable pointer type ?? */
			/* WTF ?!? */
			if (sprintf(arg2, " %i+%i Polygons using ", render.BowlGetpoly_count(), render.VehicleGetpoly_count()))
			{
				strcat (arg1, arg2);
			}
			if (sprintf(arg2, "%i+%i Kb", render.BowlGetMemSize()/1024, render.VehicleGetMemSize()/1024))
			{
				strcat (arg1, arg2);
			}

			/* save most of the name for use later */
			comSecondSC.setWindowName(arg1);

			if (sprintf(arg2, "%.2f FPS", comSecondSC.getFrameRate()))
			{
				strcat (arg1, arg2);
			}

			glutCreateWindow (arg1);


		glutSetCursor(GLUT_CURSOR_NONE);
	//	glewInit();

			glutDisplayFunc(DrawGLSceneDS); /* Register the function to do all our OpenGL drawing. */
			//glutIdleFunc(DrawIdleDS);
			glutReshapeFunc(ReSizeGLSceneDS); /* Register the function called when our window is resized. */
			glutKeyboardFunc(keyPressedDS); /* Register the function called when the keyboard is pressed. */
	//		glutSpecialFunc(specialkeyPressed); /* Register the special key function */
//			glutMouseFunc(mouseButtonPress); /* Register the function called when the mouse buttons are pressed */
//			glutMotionFunc(mouseMotionPress); /*Register the mouse motion function */
	}

