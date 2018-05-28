#include<stdio.h>
#include<unistd.h>
#include<stdlib.h>
#include <pthread.h>
#include"GLRender.h"
#include"overLapRegion.h"
#include"CaptureGroup.h"
#include"SelfCheckThread.h"
#include"CheckMyself.h"
extern SelfCheck selfcheck;

void *SelfCheck_thread(void *arg)
{
	sleep(5);
	while(1)
	{
		selfcheck.initState();
		selfcheck.CheckBrokenCam();
		//selfcheck.Check12CAM();
		//selfcheck.CheckExtra2CAM();
	//	selfcheck.Check2HD();
	//	selfcheck.CaptureCheckAll();
		selfcheck.SendBrokenCAM();
	 sleep(10);
	}
	return 0;
}

void start_SelfCheck_thread(void)
{
	pthread_t tid;
	int ret;
	ret = pthread_create( &tid, NULL,SelfCheck_thread, NULL );
}
