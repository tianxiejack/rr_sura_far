/*
 * overLapRegion.h

 *
 *  Created on: August 10, 2017
 *      Author: van
 *
 *  save current CAM_COUNT images
 *  cut and save/get overLapRegion images
 *  caculate gain of each images
 */
#if USE_GAIN
#include"StlGlDefines.h"
using namespace std;
using namespace cv;

#ifndef OVERLAPREGION_H_
#define OVERLAPREGION_H_

class overLapRegion
{
public:
	bool van_save_coincidence();
	void brightness_blance();
	void push_overLap_triangle(int direction,uint x){vectors[direction].push_back(x);};
	bool beExist();
	static  overLapRegion * GetoverLapRegion();
	void set_change_gain(bool);
	bool get_change_gain();
	void SetSingleHightLightState(bool state){EnableSingleHightLight=state;};
	bool GetSingleHightLightState(){return EnableSingleHightLight;};
	private:
	overLapRegion(){CHANGE_GAIN=false;};//false start open
	static  overLapRegion *overlapregion;
	std::vector<int> vectors[CAM_COUNT];
	std::vector<int> max_min[CAM_COUNT];
	Mat roi_image[CAM_COUNT][ROI_COUNT];
	bool CHANGE_GAIN;
	bool EnableSingleHightLight;
};

#endif
#endif
