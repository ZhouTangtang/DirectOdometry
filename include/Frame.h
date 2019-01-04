#pragma once
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <vector>
using namespace Eigen;
using namespace cv;
using namespace std;

const int MAX_LEVEL = 6;
const int PADDING = 4;


class PointDatum;
class ImageDatum;
class FrameDatum;
class CalibDatum;
class FrameShell;

float getColor(const cv::Mat &img, float u, float v);
float getGrad(const cv::Mat &img, float u, float v);
void getProjectUV(const double u0, const double v0, const double idepth0, const Matrix3d &K0,
                  double &u1, double &v1, const Matrix3d &K1,
                  const Matrix4d &T01,const double lvl_factor=1);

double getRoughIdepth(const double u0,const double v0,const double u1,const double v1,
                    const Matrix3d &K0,const Matrix3d &K1,const Matrix4d &T01,const double lvl_factor=1);
					
bool Match_NCC(const FrameDatum& img0,const FrameDatum& img1,
vector<PointDatum> &pts0, vector<PointDatum> &pts1,
const Matrix3d &K0,const Matrix3d &K1,
const Matrix4d &T01,const int lvl=0);

class CalibDatum
{
public:
	Matrix3d K;
	double a;
	double b;

	Matrix3d R_0_1;
	Matrix3d t_1_0;
};


class PointDatum
{
public:
	FrameShell* fs=nullptr;
	double u;
	double v;
	double idepth;
	bool is_good;
};

class ImageDatum
{
public:
	int w;
	int h;
	Mat color;
	Mat du;
	Mat dv;
	Mat grad;
};

class FrameDatum
{
public:
	FrameDatum(const Mat& image_src);
	~FrameDatum() {};
public:
	vector<ImageDatum> imagePyramid;
	vector<PointDatum> points;
};

class FrameShell
{
public:
	FrameShell() {};
	~FrameShell() 
	{ 
		if(data) delete data;
		if(data_right) delete data_right;
		if(calib) delete calib;
		if(calib_right) delete calib_right;
	};

  public:
	static long long next_id;
	long long id;
	double time;


	Matrix3d R_w_f;
	Vector3d t_f_w;
	FrameDatum *data = nullptr;
	FrameDatum* data_right=nullptr;
	CalibDatum* calib=nullptr;
	CalibDatum* calib_right=nullptr;
};