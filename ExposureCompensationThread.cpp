#include<stdio.h>
#include<unistd.h>
#include<stdlib.h>
#include <pthread.h>
#include"GLRender.h"
#include"overLapRegion.h"
#include"CaptureGroup.h"
#include"ExposureCompensationThread.h"
#include"GLEnv.h"
extern GLEnv env1,env2;
//extern overLapRegion  overlapregion;


#if USE_GAIN


void *exposure_thread(void *arg)
{
	GLEnv &env=env1;
	sleep(1);
	while(1)
	{
		if(!overLapRegion::GetoverLapRegion()->GetSingleHightLightState())
				{
					env.GetPanoCaptureGroup()->saveExposureCompensationCapImg();
					 sleep(2);
			//		 if(overLapRegion::GetoverLapRegion()->van_save_coincidence())
					 {  //此处我如果不能打开文件就返回，GetSingleHightLightState() 是否会继续使用亮度均衡的值？
			//			 overLapRegion::GetoverLapRegion()->brightness_blance();
					 }
				}
			 sleep(3);
	}
	return 0;
}



void start_exposure_thread(void)
{
	pthread_t tid;
	int ret;
	ret = pthread_create( &tid, NULL,exposure_thread, NULL );
}
#endif
