#if USE_GAIN
#include <stdio.h>
#include<stdlib.h>
#include <unistd.h>
#include<iostream>
#include <string.h>
#include<vector>
#include<opencv2/opencv.hpp>
#include"StlGlDefines.h"
#include"overLapRegion.h"
#include"GLRender.h"
using namespace std;
using namespace cv;

extern Render render;
GLShaderManager *my_shaderm;
overLapRegion *overLapRegion::overlapregion;

overLapRegion *  overLapRegion::GetoverLapRegion()
{
	static bool once=false;
	if(!once){
		once = true;
		overlapregion=new overLapRegion();
	}
	return  overlapregion;
}

void  overLapRegion::set_change_gain(bool TOF)
{
	this->CHANGE_GAIN=TOF;
}

bool  overLapRegion::get_change_gain()
{
	return this->CHANGE_GAIN;
}


bool overLapRegion::van_save_coincidence()
{
	Point3d Isum1 = Point3d(0.0,0.0,0.0), Isum2 = Point3d(0.0,0.0,0.0);
	double gamma=0.25;
	vector<Point3d>	van_alpha(CAM_COUNT);
	vector<Mat> van_images(CAM_COUNT);
	vector<Mat> van_images_warped_f(CAM_COUNT);
	char buf[1024];
	int i;
	for(i=0; i<CAM_COUNT; i++){
		sprintf(buf,"./data/gain/%02d.bmp",i);
		van_images[i] = imread(buf);
		if (van_images[i].empty()){
			cout<<"Can't open gain/image " << buf<<"open other one"<<endl;
			sprintf(buf,"./data/%02d.bmp",i);
			van_images[i] = imread(buf);
		}
		else{
			strcpy(buf,"'\0'");
			van_images[i].convertTo(van_images_warped_f[i], CV_8U);
		}
	}

	vector<int>::iterator it;
	Vec3b van_pix_1, van_pix_2;

	cv::Point2f vPoint1[3], vPoint2[3];
	int direction;
	char buf_1[1024];
	char buf_2[1024];
	van_alpha[0].x  =van_alpha[0].y = van_alpha[0].z = 1.0;
	float x_min[CAM_COUNT]={},y_min[CAM_COUNT]={},x_max[CAM_COUNT]={},y_max[CAM_COUNT]={};
	float x2_min[CAM_COUNT]={},y2_min[CAM_COUNT]={},x2_max[CAM_COUNT]={},y2_max[CAM_COUNT]={};
int a=0,b=0,c=0,d=0;
	for(direction=0;direction<CAM_COUNT;direction++)
	{
		float cmp_x_min=30000, cmp_y_min=30000,cmp_x_max=-1,cmp_y_max=-1;
		float cmp_x2_min=30000, cmp_y2_min=30000,cmp_x2_max=-1,cmp_y2_max=-1;
		for(it=vectors[direction].begin();it!= vectors[direction].end();it++)
		{
			for(int van_index=0; van_index<3; van_index++)
			{
				render.getPointsValue(direction, *it,vPoint1);     //van_get
				render.getPointsValue((direction+1)%(CAM_COUNT),*it, vPoint2);//van_get

				if(vPoint1[van_index].x>cmp_x_max)
					cmp_x_max=vPoint1[van_index].x;
				if(vPoint1[van_index].y>cmp_y_max)
					cmp_y_max=vPoint1[van_index].y;
				if(vPoint1[van_index].x<cmp_x_min)
					cmp_x_min=vPoint1[van_index].x;
				if(vPoint1[van_index].y<cmp_y_min)
					cmp_y_min=vPoint1[van_index].y;

				if(vPoint2[van_index].x>cmp_x2_max)
					cmp_x2_max=vPoint2[van_index].x;
				if(vPoint2[van_index].y>cmp_y2_max)
					cmp_y2_max=vPoint2[van_index].y;
				if(vPoint2[van_index].x<cmp_x2_min)
					cmp_x2_min=vPoint2[van_index].x;
				if(vPoint2[van_index].y<cmp_y2_min)
					cmp_y2_min=vPoint2[van_index].y;
			}
		}

		x_min[direction]= cmp_x_min;
		y_min[direction]= cmp_y_min;
		x_max[direction]= cmp_x_max;
		y_max[direction]= cmp_y_max;
		if(direction==CAM_COUNT-1)
		{
			x2_min[0]= cmp_x2_min;
			y2_min[0]= cmp_y2_min;
			x2_max[0]= cmp_x2_max;
			y2_max[0]= cmp_y2_max;
		}
		else{
		x2_min[direction+1]= cmp_x2_min;
		y2_min[direction+1]= cmp_y2_min;
		x2_max[direction+1]= cmp_x2_max;
		y2_max[direction+1]= cmp_y2_max;
		}
//	strcpy(buf_1,"'\0'");
//	strcpy(buf_2,"'\0'");
//	sprintf(buf_1,"./van_2/%d_left.bmp",direction);
	roi_image[direction][LEFT_ROI]=van_images[direction](Range(y_min[direction],y_max[direction]+1),Range(x_min[direction],	x_max[direction])+1);
//		imwrite(buf_1,roi_image[direction][LEFT_ROI]);
	if(direction+1==CAM_COUNT){
//		strcpy(buf_2,"./van_2/0_right.bmp");
		roi_image[0][RIGHT_ROI]=van_images[0](Range(y2_min[0],y2_max[0]+1),Range(x2_min[0],	x2_max[0])+1);
//		imwrite(buf_2,roi_image[0][RIGHT_ROI]);
	}
	else{
//	sprintf(buf_2,"./van_2/%d_right.bmp",direction+1);
	roi_image[direction+1][RIGHT_ROI]=van_images[direction+1](Range(y2_min[direction+1],y2_max[direction+1]+1),Range(x2_min[direction+1],	x2_max[direction+1])+1);
//	imwrite(buf_2,roi_image[direction+1][RIGHT_ROI]);
	}
	}
	return true;
}


bool overLapRegion::beExist()
{
	for(int direction=0;direction<CAM_COUNT;direction++)
	{
		if(vectors[direction].empty())
			return false;
	}
	return true;
}

void overLapRegion::brightness_blance()
{
	Point3d gain_c;
	Mat subimg1, subimg2;
	 std::vector<Point3d>	alpha;

	float gamma=0.25;
	alpha.resize(CAM_COUNT);
	alpha[0].x  = alpha[0].y = alpha[0].z = 1.0;
	for(int i=0;i<CAM_COUNT;i++)
	{
		 if(i+1==CAM_COUNT)
			 subimg2=roi_image[0][RIGHT_ROI];
		 else
		 subimg2=roi_image[i+1][RIGHT_ROI];
		 subimg1=roi_image[i][LEFT_ROI];
		 resize(subimg1,subimg1,Size(subimg2.cols,subimg2.rows),0,0,INTER_LINEAR);

		Point3d Isum1 = Point3d(0.0,0.0,0.0), Isum2 = Point3d(0.0,0.0,0.0);
		for(int y=0;y< subimg1.rows;y++){
			const Point3_<uchar>* r1 = subimg1.ptr<Point3_<uchar> >(y);
			const Point3_<uchar>* r2 = subimg2.ptr<Point3_<uchar> >(y);
			for(int x=0;x< subimg1.cols;x++){
				Isum1.x += std::pow(static_cast<double>(r1[x].x/255.0),gamma)*255.0;
				Isum1.y += std::pow(static_cast<double>(r1[x].y/255.0),gamma)*255.0;
				Isum1.z += std::pow(static_cast<double>(r1[x].z/255.0),gamma)*255.0;
		//		printf("x=%d  y=%d\n",x,y);

				Isum2.x += std::pow(static_cast<double>(r2[x].x/255.0),gamma)*255.0;
				Isum2.y += std::pow(static_cast<double>(r2[x].y/255.0),gamma)*255.0;
				Isum2.z += std::pow(static_cast<double>(r2[x].z/255.0),gamma)*255.0;
			}
		}
		if(i+1==CAM_COUNT)
		{
			alpha[0].x  = Isum1.x/Isum2.x;
			alpha[0].y  = Isum1.y/Isum2.y;
			alpha[0].z  = Isum1.z/Isum2.z;

			alpha[0].x  *= alpha[i].x;
			alpha[0].y  *= alpha[i].y;
			alpha[0].z  *= alpha[i].z;

		}
		else{
		alpha[i+1].x  = Isum1.x/Isum2.x;
		alpha[i+1].y  = Isum1.y/Isum2.y;
		alpha[i+1].z  = Isum1.z/Isum2.z;

		alpha[i+1].x  *= alpha[i].x;
		alpha[i+1].y  *= alpha[i].y;
		alpha[i+1].z  *= alpha[i].z;
		}
	}
	Point3d sum_alph = Point3d(0.0, 0.0, 0.0);
	Point3d sum_alph2 = Point3d(0.0, 0.0, 0.0);
		for (int img_idx= 0; img_idx < CAM_COUNT; ++img_idx)
		{
			sum_alph.x += alpha[img_idx].x;
			sum_alph.y += alpha[img_idx].y;
			sum_alph.z += alpha[img_idx].z;

			sum_alph2.x += alpha[img_idx].x*alpha[img_idx].x;
			sum_alph2.y += alpha[img_idx].y*alpha[img_idx].y;
			sum_alph2.z += alpha[img_idx].z*alpha[img_idx].z;
		}
		gain_c.x = sum_alph.x/sum_alph2.x;
		gain_c.y = sum_alph.y/sum_alph2.y;
		gain_c.z = sum_alph.z/sum_alph2.z;   //求出增益值

		float x,y,z;

		//  Point3d gain_2[CAM_COUNT];
	if(EnableSingleHightLight==false)
	{
		for(int index=0;index<CAM_COUNT;index++)
		{
			x=1.0/(std::pow(alpha[index].x*gain_c.x, 1/gamma));
			y=1.0/(std::pow(alpha[index].y*gain_c.y, 1/gamma));
			z=1.0/(std::pow(alpha[index].z*gain_c.z, 1/gamma));

			my_shaderm=(GLShaderManager *)getDefaultShaderMgr();
			my_shaderm->set_gain_(index,x,y,z);
		}
	}

}
#endif
