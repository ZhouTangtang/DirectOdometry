#include "Tracker.h"
#include <math.h>
#include <Eigen/Dense>
#include "PixelSelector2.h"


const int PADDING = 4;
bool Tracker::AddNewImage(const cv::Mat & img_src)
{
	return false;
}

bool Tracker::AddNewStereo(const cv::Mat & img_src, const cv::Mat & img_src_right)
{
	cout<<"mmm"<<endl;
	if (curr_fs) delete curr_fs;
	cout<<"mmm"<<endl;


	curr_fs = new FrameShell();
	curr_fs->id = FrameShell::next_id++;
	curr_fs->calib = new CalibDatum();
	curr_fs->data = new FrameDatum(img_src);
	curr_fs->data_right = new FrameDatum(img_src_right);


	if (frame_buffer.size() == 0)
	{
		Initialize();
	}
	else
	{
		//TrackNewFrame();
	}
	return true;
}

bool Tracker::Initialize()
{
	if(!Detect(*curr_fs->data)) return false;
	cout<<active_points[0].size()<<endl;
	for (int lvl = MAX_LEVEL-1; lvl>=0; lvl++)
	{
		/*Matrix3d R_0_1 = curr_fs->calib->R_0_1;
		Vector3d t_1_0 = curr_fs->calib->t_1_0;*/

		lvl=0;

		Matrix4d T01=Matrix4d::Identity();
		T01.block<3, 3>(0, 0) = Parameter::R01;
		T01.block<3, 1>(0, 3) = Parameter::t01;
		Matrix4d T10 = T01.inverse();
		Matrix3d R10 = T10.block<3, 3>(0, 0);
		Vector3d t10 = T10.block<3, 1>(0, 3);


		while (true)
		{
			vector<Point2f> v_p0;
			vector<Point2f> v_p1;
			vector<KeyPoint> v_kp0;
			vector<KeyPoint> v_kp1;
			vector<double> v_depth;

			for (auto &pt : active_points[lvl])
			{
				ImageDatum &img = curr_fs->data->imagePyramid[lvl];
				ImageDatum &img_right = curr_fs->data_right->imagePyramid[lvl];

				if (pt.u >= PADDING && pt.v >= PADDING && pt.u < img.w - PADDING && pt.v < img.h - PADDING)
				{
					double lvl_factor = std::pow(2, lvl);

					Vector3d xy0 = Parameter::K0.inverse()*Vector3d(pt.u*lvl_factor, pt.v*lvl_factor, 1);
					double idepth0 = pt.idepth;
					double idepth1 = 1 / (R10.transpose().block<1, 3>(2, 0)*(xy0 / idepth0 - t10));
					double x1 = idepth1 * R10.transpose().block<1, 3>(0, 0)*(xy0 / idepth0 - t10);
					double y1 = idepth1 * R10.transpose().block<1, 3>(1, 0)*(xy0 / idepth0 - t10);
					Vector3d uv1 = Parameter::K1*Vector3d(x1, y1, 1);

					Vector3d uv1_lvl = Vector3d(uv1(0) / lvl_factor, uv1(1) / lvl_factor, 1);

					float color_src = getColor(img.color, pt.u, pt.v);
					float color_hit = getColor(img_right.color, uv1_lvl(0), uv1_lvl(1));

					v_p0.push_back(Point2f(pt.u, pt.v));
					v_p1.push_back(Point2f(uv1_lvl(0), uv1_lvl(1)));
					v_depth.push_back(1 / pt.idepth);
					

				}
			}
			cv::Mat img0_show, img1_show,img_depth;

			curr_fs->data->imagePyramid[lvl].color.copyTo(img0_show);
			curr_fs->data_right->imagePyramid[lvl].color.copyTo(img1_show);
			curr_fs->data->imagePyramid[lvl].color.copyTo(img_depth);

			for (int i = 0; i < v_p0.size(); i++)
			{
				circle(img_depth, v_p0[i], 0, cv::Scalar(0.6));
				circle(img0_show, v_p0[i], 0, cv::Scalar(0,1.0,0));
				circle(img1_show, v_p1[i], 0, cv::Scalar(0,1.0,0));
			}

			resize(img0_show, img0_show, img0_show.size() * 1,0,0,0);
			resize(img1_show, img1_show, img1_show.size() * 1,0,0,0);
			resize(img_depth, img_depth, img_depth.size() * 1, 0, 0, 0);

			imshow("img0", img0_show);
			imshow("img1", img1_show);
			imshow("depth", img_depth);
			waitKey(0);
			cin.get();
		}
	}


	return false;
}


bool Tracker::Detect(const FrameDatum & frame)
{
	PixelSelector selector(frame.imagePyramid[0].w,frame.imagePyramid[0].h);
	float *map_out=new float[frame.imagePyramid[0].w*frame.imagePyramid[0].h];
	memset(map_out,0,sizeof(float)*frame.imagePyramid[0].w*frame.imagePyramid[0].h);
	cout<<1<<endl;
	float densities[] = {0.03,0.05,0.15,0.5,1};
	int npts=selector.makeMaps(&frame,map_out,densities[0],1,false,2);
	cout<<"npts:"<<npts<<endl;
	for(int x=0;x<frame.imagePyramid[0].w;x++)
	for(int y=0;y<frame.imagePyramid[0].h;y++)
	{
		if(map_out[x+y*frame.imagePyramid[0].w]>0)
		{
		PointDatum pt;
		pt.fs=curr_fs;
		pt.idepth=1;
		pt.u=x;
		pt.v=y;
		active_points[0].push_back(pt);
		}
	}
	cout<<2<<endl;
	delete[] map_out;
	return true;
}

