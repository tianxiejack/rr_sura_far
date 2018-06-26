#ifndef _UTC_TRK_
#define _UTC_TRK_

#include "PCTracker.h"

#ifdef __cplusplus
extern "C"{
#endif

typedef enum{
	SEARCH_MODE_ALL	= 0x01,
	SEARCH_MODE_LEFT,
	SEARCH_MODE_RIGHT,
	SEARCH_MODE_NEAREST,
	SEARCH_MODE_MAX,
}SEARCH_MODE_TYPE;

typedef struct
{
	int  width;
	int  height;
}UTC_SIZE;

typedef struct
{
	int  x;
	int  y;
	int  width;
	int  height;
}UTC_Rect;

typedef struct
{
	float  x;
	float  y;
	float  width;
	float  height;
}UTC_RECT_float;

typedef struct _track_acq_param{
	int axisX;
	int axisY;
	UTC_Rect rcWin;
}UTC_ACQ_param;

typedef struct _utctrack_obj{
	bool     m_bInited;

	int axisX;
	int axisY;
	UTC_Rect rcWin;

}UTCTRACK_OBJ, *UTCTRACK_HANDLE;

typedef struct _utctrack_dynamic_param{
	float 	occlusion_thred;
	float	retry_acq_thred;

}UTC_DYN_PARAM;

typedef	struct	 _utctrack_search_param{
	int	min_bias_pix;
	int	max_bias_pix;
	float	blend_value;
}UTC_SEARCH_PARAM;

typedef enum{
	tPLT_TST = 0,
	tPLT_WRK = 1,
}tPLT;

typedef enum{
	BoreSight_Big	= 0,
	BoreSight_Mid	= 1,
	BoreSight_Sm	= 2,
}BS_Type;

typedef struct _trk_sech_t{
	int	res_distance;
	int	res_area;
}TRK_SECH_RESTRAINT;

UTCTRACK_HANDLE CreateUtcTrk();
void DestroyUtcTrk(UTCTRACK_HANDLE handle);
UTC_RECT_float UtcTrkAcq(UTCTRACK_OBJ* pUtcTrkObj, IMG_MAT frame, UTC_ACQ_param inputParam);
UTC_RECT_float UtcTrkProc(UTCTRACK_OBJ *pUtcTrkObj, IMG_MAT frame, int *pRtnStat);

void UtcSetDynParam(UTCTRACK_OBJ *pUtcTrkObj, UTC_DYN_PARAM dynamicParam);
void UtcSetUpFactor(UTCTRACK_OBJ *pUtcTrkObj, float up_factor);
void UtcSetSerchMode(UTCTRACK_OBJ *pUtcTrkObj, SEARCH_MODE_TYPE searchMode);

void UtcGetOptValue(UTCTRACK_OBJ *pUtcTrkObj, float *optValue);
void UtcSetIntervalFrame(UTCTRACK_OBJ *pUtcTrkObj, int gapFrame);

void UtcSetSearchParam(UTCTRACK_OBJ *pUtcTrkObj, UTC_SEARCH_PARAM searchParam);
void UtcSetPLT_BS(UTCTRACK_OBJ *pUtcTrkObj, tPLT pltWork, BS_Type bsType);
void UtcSetRestraint(UTCTRACK_OBJ *pUtcTrkObj,TRK_SECH_RESTRAINT resTraint);

#ifdef __cplusplus
}
#endif


#endif
