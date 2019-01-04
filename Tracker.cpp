#include "Tracker.h"
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "tools.hpp"
#include "PixelSelector2.h"
#include "outer/GLFWViewer.h"

using namespace Eigen;

const bool DEBUG_THIS = false;

// 取像素本身及其周围的8个像素
const int PATTERN_NUM = 1;
const int PATTERN_U[9] = {0, -2, -1, 0, 1, 2, 1, 0, -1};
const int PATTERN_V[9] = {0, 0, 1, 2, 1, 0, -1, -2, -1};


vector<Vector3d> v_PointCloud;
vector<pair<Matrix3d,Vector3d>> v_Frames;
vector<Vector3d> v_Trajectory;

bool Tracker::AddNewImage(const cv::Mat & img_src)
{
	return false;
}

bool Tracker::AddNewStereo(const cv::Mat & img_src, const cv::Mat & img_src_right,double time)
{
	viewer.Show();
	//viewer.Hide();
	//if (curr_fs) delete curr_fs;
	if(frame_buffer.size()>7)
	{
		delete frame_buffer[0];
		frame_buffer.erase(frame_buffer.begin());
	}


	curr_fs = new FrameShell();
	curr_fs->id = FrameShell::next_id++;
	curr_fs->time = time;
	curr_fs->calib = new CalibDatum();
	curr_fs->data = new FrameDatum(img_src);
	curr_fs->data_right = new FrameDatum(img_src_right);
	

	cout << frame_buffer.size() << endl;
	//cin.get();
	if (frame_buffer.size() == 0)
	{
		InitializeFirstFrame();
	}
	else
	{
		TrackNewFrame();
		/*
		while(frame_buffer.size()>7)
		{
		delete frame_buffer[0];
		frame_buffer.erase(frame_buffer.begin());
		}
		*/
	}

	return true;
}

bool Tracker::InitializeFirstFrame()
{
	curr_fs->R_w_f = Matrix3d::Identity();
	curr_fs->t_f_w = Vector3d::Zero();
	return MakeKeyFrame();
}

bool Tracker::MakeKeyFrame()
{
	for (int i = 0; i <= MAX_LEVEL - 1;i++)
	{
		active_points[i].clear();
		active_points[i].resize(0);
		active_points_right[i].clear();
		active_points_right[i].resize(0);
	}
	//v_PointCloud.clear();
	//return false;
	if(!Detect(*curr_fs->data)) return false;
	cout<<"active_points: "<<active_points[0].size()<<endl;


	Matrix4d T01=Matrix4d::Identity();
		T01.block<3, 3>(0, 0) = Parameter::R01;
		T01.block<3, 1>(0, 3) = Parameter::t01;
		Matrix4d T10 = T01.inverse();
		Matrix3d R10 = T10.block<3, 3>(0, 0);
		Vector3d t10 = T10.block<3, 1>(0, 3);


	for (int lvl = 0; lvl<=MAX_LEVEL-1; lvl++)
	{

		//cv::Mat img0_show, img1_show, img_depth;

		//curr_fs->data->imagePyramid[lvl].color.copyTo(img0_show);
		//curr_fs->data_right->imagePyramid[lvl].color.copyTo(img1_show);
		//curr_fs->data->imagePyramid[lvl].color.copyTo(img_depth);
		cout << "LVL:" << lvl << endl;
		if(lvl==0)
		{

			Match_NCC(*(curr_fs->data), *(curr_fs->data_right), active_points[lvl],active_points_right[lvl], Parameter::K0, Parameter::K1, T01, lvl);
			cout << active_points[lvl].size() << " " << active_points_right[lvl].size() << endl;
			for (int i = 0; i < active_points[lvl].size();i++)
			{
				if(!active_points[lvl][i].is_good)
					continue;
				auto &pt0 = active_points[lvl][i];
				auto &pt1 = active_points_right[lvl][i];
				//circle(img_depth, Point2f(pt0.u,pt0.v), 0, cv::Scalar(0.6));
				//circle(img0_show, Point2f(pt0.u,pt0.v), 0, cv::Scalar(0,1.0,0));
				//circle(img1_show, Point2f(pt1.u,pt1.v), 0, cv::Scalar(0,1.0,0));
				v_PointCloud.push_back(curr_fs->R_w_f.transpose()*(1 / pt0.idepth 
				* Parameter::K0.inverse() * Vector3d(pt0.u, pt0.v, 1))+curr_fs->t_f_w);

			}
			//resize(img0_show, img0_show, img0_show.size() * 1,0,0,0);
			//resize(img1_show, img1_show, img1_show.size() * 1,0,0,0);
			//resize(img_depth, img_depth, img_depth.size() * 1, 0, 0, 0);
			viewer.SetPointCloud(v_PointCloud);
		}
		else
		{
			cout << "lvl:" << lvl << " ";
			int h = curr_fs->data->imagePyramid[lvl].h;
			int w = curr_fs->data->imagePyramid[lvl].w;
			MatrixXf map_lvl = MatrixXf::Zero(h, w);
			MatrixXi count_lvl = MatrixXi::Zero(h, w);
			for (int i = 0; i < active_points[lvl - 1].size();i++)
			{
				float u = active_points[lvl - 1][i].u / 2;
				float v = active_points[lvl - 1][i].v / 2;
				float idepth = active_points[lvl - 1][i].idepth;
				if(u<0||u>=w-1||v<0||v>=h-1)
					continue;
				count_lvl((int)(v + 0.5), (int)(u + 0.5))++;
				map_lvl((int)(v + 0.5), (int)(u + 0.5))+=idepth;
			}
			for (int u = 0; u < w;u++)
			{
				for (int v = 0; v < h;v++)
				{

					if(count_lvl(v,u)!=0)
					{
					PointDatum pt;
					pt.is_good = true;
					pt.fs = curr_fs;
					pt.u = u;
					pt.v = v;
					pt.idepth = map_lvl(v, u) / count_lvl(v, u);
					//cout << active_points[lvl].size() << endl;
					active_points[lvl].push_back(pt);


					}

				}
			}
			cout << "lvl end!" << endl;
		}

		cout << "lvl end2!" << endl;

	}
	cout << "active_points:";
	for (int i = 0; i <= MAX_LEVEL - 1;i++)
	{
		cout << active_points[i].size() << " ";
	}
	cout << endl;
	//cin.get();
	frame_buffer.push_back(curr_fs);

	return true;
}

bool Tracker::Detect(const FrameDatum & frame)
{
	PixelSelector selector(frame.imagePyramid[0].w,frame.imagePyramid[0].h);
	float *map_out=new float[frame.imagePyramid[0].w*frame.imagePyramid[0].h];
	memset(map_out,0,sizeof(float)*frame.imagePyramid[0].w*frame.imagePyramid[0].h);
	cout<<1<<endl;
	float densities[] = {0.01,0.05,0.15,0.5,1};
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

bool Tracker::TrackNewFrame()
{
	//cin.get();
	float fx0 = Parameter::K0(0, 0);
	float fy0 = Parameter::K0(1, 1);
	float cx0 = Parameter::K0(0, 2);
	float cy0 = Parameter::K0(1, 2);

	Matrix3d R = Matrix3d::Identity();
	Vector3d t = Vector3d::Zero();
		
	float a = 0;
	float b = 0;

	//  如果存在上一帧，并且上一帧不为关键帧
	if(prev_fs)
	{
			//cout << prev_fs->R_w_f << endl;

		FrameShell *ref_fs = frame_buffer.back();
		//curr_fs->R_w_f = R *frame_buffer.back()->R_w_f;

		R = Quaterniond(prev_fs->R_w_f * ref_fs->R_w_f.transpose()).toRotationMatrix();

		//curr_fs->t_f_w =frame_buffer.back()->t_f_w - curr_fs->R_w_f.transpose() * t;

		t =  prev_fs->R_w_f*(ref_fs->t_f_w-prev_fs->t_f_w);
	}
	//cout << R << endl
	//	 << t << endl;
	

	for (int lvl = MAX_LEVEL-1; lvl >= 0;lvl--)
	{
		double energy_old = 100000000;
		double energy_new = 0;
		int count = 0;
		int lvl_factor = pow(2, lvl);

		cv::Mat img_new_show;

		while (true /*未收敛*/)
		{
			// 构建关于active_points的误差方程
			// （当前来说是把上一关键帧的点投影到当前帧来构建）
			int res_num = 0;
			Matrix<double, 8, 8> JJ = Matrix<double, 8, 8>::Zero();
			Matrix<double, 8, 1> Jr = Matrix<double, 8, 1>::Zero();

			curr_fs->data->imagePyramid[lvl].color.copyTo(img_new_show);

			// 目前只有最后关键帧有用。
			FrameShell *ref_fs = frame_buffer.back();

			Matrix4d T = Matrix4d::Identity();
			T.block<3, 3>(0, 0) = R;
			T.block<3, 1>(0, 3) = t;

			for (int i = 0; i < active_points[lvl].size(); i++)
			{

				if (!active_points[lvl][i].is_good)
					continue;
				for (int j = 0; j < PATTERN_NUM; j++)
				{
					double u0 = active_points[lvl][i].u + PATTERN_U[j];
					double v0 = active_points[lvl][i].v + PATTERN_V[j];
					double idepth0 = active_points[lvl][i].idepth;

					if (u0 < (float)PADDING / lvl_factor || u0 > curr_fs->data->imagePyramid[lvl].w - (float)PADDING / lvl_factor - 1 ||
						v0 < (float)PADDING / lvl_factor || v0 > curr_fs->data->imagePyramid[lvl].h - (float)PADDING / lvl_factor - 1)
					{
						//active_points[lvl][i].is_good = false;
						continue;
					}

					

					//cout << "u0 v0:" << u0 << " " << v0 << endl;
					Vector3d pt2 = R * Parameter::K0.inverse() * Vector3d(u0 * lvl_factor, v0 * lvl_factor, 1) + t * idepth0;
					float x2 = pt2(0) / pt2(2);
					float y2 = pt2(1) / pt2(2);
					float idepth2 = idepth0 / pt2(2);
					float u2 = (fx0 * x2 + cx0) / lvl_factor;
					float v2 = (fy0 * y2 + cy0) / lvl_factor;
					circle(img_new_show, Point2f(u2, v2), 0, cv::Scalar(0, 0, 0));

					if (u2 < (float)PADDING / lvl_factor || u2 > curr_fs->data->imagePyramid[lvl].w - (float)PADDING / lvl_factor - 1 ||
						v2 < (float)PADDING / lvl_factor || v2 > curr_fs->data->imagePyramid[lvl].h - (float)PADDING / lvl_factor - 1)
					{
						//active_points[lvl][i].is_good = false;
						continue;
					}

					float color_src = getColor(ref_fs->data->imagePyramid[lvl].color, u0, v0);
					float color_hit = getColor(curr_fs->data->imagePyramid[lvl].color, u2, v2);
					float grad_u = getGrad(curr_fs->data->imagePyramid[lvl].du, u2, v2);
					float grad_v = getGrad(curr_fs->data->imagePyramid[lvl].dv, u2, v2);

					float res = color_hit - (float)(expf(a) * color_src + b);
					res_num++;
					//cout << "res:" << res << endl;

					float setting_huberTH = 9;
					float hw = fabs(res) < setting_huberTH ? 1 : setting_huberTH / fabs(res);

					energy_new += (hw * res) * (hw * res);

					// 参数包括：平移量3个，旋转3个，光度2个，（深度1个）

					Matrix<double, 1, 8> J = Matrix<double, 1, 8>::Zero();
					float dxInterp = grad_u * fx0;
					float dyInterp = grad_v * fy0;
					J(0, 0) = idepth2 * dxInterp;
					J(0, 1) = idepth2 * dyInterp;
					J(0, 2) = -idepth2 * (x2 * dxInterp + y2 * dyInterp);
					J(0, 3) = -x2 * y2 * dxInterp - (1 + y2 * y2) * dyInterp;
					J(0, 4) = (1 + x2 * x2) * dxInterp + x2 * y2 * dyInterp;
					J(0, 5) = -y2 * dxInterp + x2 * dyInterp;
					J(0, 6) = -expf(a) * (color_src);
					J(0, 7) = -1;

					J = J * hw;
					JJ += J.transpose() * J;
					Jr += J.transpose() * hw * res;

					//float dxdd = t(0) - t(1)
				}
			}

			if (DEBUG_THIS)
				cout << "lvl:" << lvl << " count:" << count << " "
					 << "res_num:" << res_num << " "
					 << "energy:" << energy_new << endl;

			// 增加先验
			// 提高a,b的稳定程度
			Matrix<double, 1, 8> J_prior = Matrix<double, 1, 8>::Zero();
			// r=a
			// r=b
			J_prior(0, 6) = 10;
			J_prior(0, 7) = 1;
			//double weight_prior = 1000000000000;
			JJ += J_prior.transpose() * J_prior;
			Jr(6) += J_prior(0, 6) * J_prior(0, 6) * a;
			Jr(7) += J_prior(0, 7) * J_prior(0, 7) * b;

			Matrix<double, 8, 1> x = JJ.ldlt().solve(Jr);
			if (DEBUG_THIS)
				cout << x.transpose() << endl;
			if (energy_new > energy_old || x.norm() < 0.00001 || count > 30 /*&&energy_new<150000*/)
				break;

			if (x.norm() > 1 && DEBUG_THIS) //WRONG SITUATION!
			{
				cout << "JJ:" << endl;
				cout << JJ << endl;
				cout << "Jr:" << endl;
				cout << Jr << endl;
				cout << "x:" << endl;
				cout << x << endl;
				cout << energy_new << endl;

				cin.get();
			}

			x = -x;
			Quaterniond dq = tools::smallAngleQuaternion(x.segment<3>(3));
			t = t + x.head<3>();
			R = R * dq.toRotationMatrix();
			a = a + x(6);
			b = b + x(7);
			count++;
			energy_old = energy_new;
			energy_new = 0;
	}
	// FOR DEBUG
	if(curr_fs->time>1403715349962142976.0/1e9-8&&lvl==0)
	{
		imshow("show", img_new_show);
		waitKey(200);
	}
	}
	

	curr_fs->R_w_f = R *frame_buffer.back()->R_w_f;
	curr_fs->t_f_w =frame_buffer.back()->t_f_w - curr_fs->R_w_f.transpose() * t;
	

	Matrix4d T_f_w = Matrix4d::Identity();
	T_f_w.block<3, 3>(0, 0) = curr_fs->R_w_f.transpose();
	T_f_w.block<3, 1>(0, 3) = curr_fs->t_f_w;
	Matrix4d T_w_f = T_f_w.inverse();
	v_Frames.clear();
	v_Frames.push_back(make_pair(T_f_w.block<3,3>(0,0), T_f_w.block<3,1>(0,3)));
	v_Trajectory.push_back(T_f_w.block<3, 1>(0, 3));
	viewer.SetFrames(v_Frames);
	viewer.SetTrajectory(v_Trajectory);

	// 插入关键帧
	count++;
	if(count>20||t.norm()>0.3||Quaterniond(R).w()<cos(5.0/180*3.1415926))
	{
		cout << ">>>>count:" << count << endl;
		MakeKeyFrame();
		count = 0;
		prev_fs = nullptr;
	}
	else
	{
		delete prev_fs;
		prev_fs = curr_fs;
	}

	
	// cin.get();
	// cout << "t:"<<t.transpose() << endl;
	// 然后判断是否要插入新的关键帧
	// 如果要插入新的关键帧的话就要重新提取点

	return true;
}