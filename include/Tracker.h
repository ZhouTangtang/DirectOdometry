#pragma once
#include "Frame.h"
#include <map>
#include "parameter.h"

using namespace std;
using namespace cv;
using namespace Eigen;

class Tracker
{
public:
	Tracker() { Parameter::LoadParameter(); };
	~Tracker()
	{
		for (int i = 0; i < frame_buffer.size();i++)
		{
			delete frame_buffer[i];
		}
	};

public:
	bool AddNewImage(const cv::Mat &img_src);
	bool AddNewStereo(const cv::Mat &img_src,
					  const cv::Mat &img_src_right,
					  double time);

  private:
	bool InitializeFirstFrame();
	bool TrackNewFrame();
	bool MakeKeyFrame();
	bool Detect(const FrameDatum& frame);

private:
	vector<FrameShell*> frame_buffer;
	FrameShell *curr_fs = nullptr;
	FrameShell *prev_fs = nullptr;
	int count = 0;
	vector<PointDatum> active_points[MAX_LEVEL];
	vector<PointDatum> active_points_right[MAX_LEVEL];
	
};