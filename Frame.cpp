#include "Frame.h"
#include <Eigen/Dense>
long long FrameShell::next_id = 0;

FrameDatum::FrameDatum(const Mat & image_src)
{
	vector<Mat> pyramid_src;
	buildPyramid(image_src, pyramid_src, MAX_LEVEL);
	for (int i = 0; i < MAX_LEVEL; i++)
	{
		ImageDatum image;
		image.color = pyramid_src[i];
		image.w = image.color.cols;
		image.h = image.color.rows;
		image.du = cv::Mat(image.h, image.w, CV_32F);
		image.dv = cv::Mat(image.h, image.w, CV_32F);
		image.grad = cv::Mat(image.h, image.w, CV_32F);

		for(int x=0;x<image.w;x++)
			for (int y = 0; y < image.h; y++)
			{
				if (x == 0 || y == 0 || x == image.w - 1 || y == image.h - 1)
				{
					image.du.at<float>(y, x) = 0;
					image.dv.at<float>(y, x) = 0;
					image.grad.at<float>(y, x) = 0;
				}
				else
				{
					//cout << x << " " << y << endl;
					/*image.du.at<float>(y, x) = 0.5*((float)image.color.at<uchar>(y, x + 1) - (float)image.color.at<uchar>(y, x - 1)+256)/512;
					image.dv.at<float>(y, x) = 0.5*((float)image.color.at<uchar>(y + 1, x) - (float)image.color.at<uchar>(y - 1, x)+256)/512;*/
					image.du.at<float>(y, x) = 0.5*((float)image.color.at<uchar>(y, x + 1) - (float)image.color.at<uchar>(y, x - 1));
					image.dv.at<float>(y, x) = 0.5*((float)image.color.at<uchar>(y + 1, x) - (float)image.color.at<uchar>(y - 1, x));
					image.grad.at<float>(y, x) = image.du.at<float>(y, x)*image.du.at<float>(y, x)
												+ image.dv.at<float>(y, x)*image.dv.at<float>(y, x);
					//cout << image.du.at<float>(y, x) << endl;
				}
			}
		imagePyramid.push_back(image);
	}
}

float getColor(const cv::Mat & img, float u, float v)
{
	float color1 = (float)img.at<uchar>((int)v, (int)u);
	float color2 = (float)img.at<uchar>((int)v , (int)u+1);
	float color3 = (float)img.at<uchar>((int)v+1, (int)u);
	float color4 = (float)img.at<uchar>((int)v + 1, (int)u + 1);
	float du = u - (int)u;
	float dv = v - (int)v;
	float color = (color1 * (1 - du) + color2 * du)*(1 - dv)
		+ (color3*(1 - du) + color4 * du)*dv;
	return color;
}
float getGrad(const cv::Mat & img, float u, float v)
{
    float color1 = (float)img.at<float>((int)v, (int)u);
	float color2 = (float)img.at<float>((int)v , (int)u+1);
	float color3 = (float)img.at<float>((int)v+1, (int)u);
	float color4 = (float)img.at<float>((int)v + 1, (int)u + 1);
	float du = u - (int)u;
	float dv = v - (int)v;
	float color = (color1 * (1 - du) + color2 * du)*(1 - dv)
		+ (color3*(1 - du) + color4 * du)*dv;
	return color;   
}

void getProjectUV(const double u0, const double v0, const double idepth0, const Matrix3d &K0,
                  double &u1, double &v1, const Matrix3d &K1,
                  const Matrix4d &T01,const double lvl_factor)
{
    Matrix4d T10 = T01.inverse();
    Matrix3d R01 = T01.block<3, 3>(0, 0);
    Vector3d t10 = T10.block<3, 1>(0, 3);

    Vector3d xy0 = K0.inverse() * Vector3d(u0*lvl_factor, v0*lvl_factor, 1);
    double idepth1 = 1 / (R01.block<1, 3>(2, 0) * (xy0 / idepth0 - t10));
    double x1 = idepth1 * R01.block<1, 3>(0, 0) * (xy0 / idepth0 - t10);
    double y1 = idepth1 * R01.block<1, 3>(1, 0) * (xy0 / idepth0 - t10);

    Vector3d uv1 = K1 * Vector3d(x1, y1, 1);

    u1 = uv1(0)/lvl_factor;
    v1 = uv1(1)/lvl_factor;
    return;
}


// WRONG!
double getRoughIdepth(const double u0, const double v0, const double u1, const double v1,
                    const Matrix3d &K0,const Matrix3d &K1,const Matrix4d &T01,const double lvl_factor)
{
    Matrix4d T10 = T01.inverse();
    Matrix3d R01 = T01.block<3, 3>(0, 0);
    Vector3d t01 = T01.block<3, 1>(0, 3);
    Matrix3d R10 = T10.block<3, 3>(0, 0);
    Vector3d t10 = T10.block<3, 1>(0, 3);

    double x0=(u0*lvl_factor-K0(0,2))/K0(0,0);
    double y0=(v0*lvl_factor-K0(1,2))/K0(1,1);
    double x1=(u1*lvl_factor-K1(0,2))/K1(0,0);
    double y1=(v1*lvl_factor-K1(1,2))/K1(1,1);
    //cout<<x0 <<" "<<y0<<" "<<x1<<" "<<y1<<" "<<endl;
   // cout<<T01<<endl;
    //double y1=();
    Vector3d r1=Vector3d(x1,y1,1).cross(R01*Vector3d(x0,y0,1));
    Vector3d r2=Vector3d(x1,y1,1).cross(t01);
    //cout<<"trangulate: "<< r1.transpose()<<endl<<r2.transpose()<<endl;
    return -r1(1)/r2(1);
}

bool Match_NCC(const FrameDatum& frame0,const FrameDatum& frame1,
vector<PointDatum> &pts0, vector<PointDatum> &pts1,
const Matrix3d &K0,const Matrix3d &K1,
const Matrix4d &T01,const int lvl)
{
	// int PADDING = 4;
    // 原本以为这边不用金字塔
    // 但是似乎不用不行
	int patchSize[2] = {3, 5};
	double idepth0_min = 0.01;
	double idepth0_max = 1;

	pts1=pts0;
	double u1_min, u1_max, v1_min, v1_max;

	const cv::Mat &img0_lvl = frame0.imagePyramid[lvl].color;
	const cv::Mat &img1_lvl = frame1.imagePyramid[lvl].color;
	double lvl_factor = pow(2, lvl);

	for (int i = 0; i < pts0.size();i++)
	{
		auto &pt0 = pts0[i];
		auto &pt1 = pts1[i];
		float u = pt0.u;
		float v = pt0.v;
		pt0.is_good = false;
		getProjectUV(u, v, idepth0_min, K0,
					 u1_min, v1_min, K1, T01, lvl_factor);
		getProjectUV(u, v, idepth0_max, K0,
					 u1_max, v1_max, K1, T01, lvl_factor);

		if(u<patchSize[1]+PADDING||u>img0_lvl.cols-patchSize[1]-PADDING-1
		||v<patchSize[0]+PADDING||v>img0_lvl.rows-patchSize[0]-PADDING-1)
						continue;

		// start searching...
        double NCC_max=0;
        double NCC_second_max = 0;
        double NCC_min=0;
        int u1_match=-1;
        int v1=(int)(v1_min+v1_max)/2.0;

        for(int u1=(int) u1_max;u1<(int)u1_min;u1++)
        {
            if(u1<patchSize[1]+PADDING||u1>img0_lvl.cols-patchSize[1]-PADDING-1||v1<patchSize[0]+PADDING||v1>img0_lvl.rows-patchSize[0]-PADDING-1)
						continue;

            // calculate NCC
            double NCC=0;
            double sum1=0,sum2=0,sum3=0;
            double sum_c0=0;
            double sum_c1=0;
            double ave_c0;
            double ave_c1;
            int patch_num=patchSize[0]*patchSize[1];

            //int count=0;
            int patch_du = patchSize[1] / 2;
            int patch_dv = patchSize[0] / 2;
            for(int du=-patch_du;du<=patch_du;du++)
            for(int dv=-patch_dv;dv<=patch_dv;dv++)
            {
                //count++;
                double c0=(float)img0_lvl.at<uchar>(v+dv,u+du);
                double c1=(float)img1_lvl.at<uchar>(v1+dv,u1+du);
                //cout<<c0<<" "<<c1<<endl;
                sum_c0+=c0;
                sum_c1+=c1;
            }
            ave_c0=sum_c0/patch_num;
            ave_c1=sum_c1/patch_num;

            for(int du=-patch_du;du<=patch_du;du++)
            for(int dv=-patch_dv;dv<=patch_dv;dv++)
            {
                //count++;
                double c0=(float)img0_lvl.at<uchar>(v+dv,u+du)-ave_c0;
                double c1=(float)img1_lvl.at<uchar>(v1+dv,u1+du)-ave_c1;
                sum1+=c0*c0;
                sum2+=c1*c1;
                sum3+=c0*c1;
            }
            //cout<<(sum1)*(sum2)<<" ";
            NCC=(sum3)/sqrt((sum1)*(sum2));
            //cout<<NCC<<" ";
            //cin.get();
            if(NCC>NCC_max) 
            {
                NCC_second_max = NCC_max;
                NCC_max = NCC;
                u1_match = u1;
            }
            if(NCC<NCC_min)
             NCC_min=NCC;
        }
       // cout<<endl;
        //cout<<u1_match<<endl;

        // 临时！！！
        // 两个条件
        // 如果NCC太小
        // 如果附近NCC差别太小
        if(NCC_max<0.9|| (NCC_max-NCC_min)<0.1 || (NCC_max-NCC_second_max)<0.001) continue; 
        //cv::line(img1_show,Point2f(u1_min,v1),Point2f(u1_max,v1),Scalar(255));
        cout << "NCC:" << NCC_max << " " << NCC_second_max << endl;

        //cv::circle(img0_show,Point2f(u,v),0,Scalar(0));
        //cv::circle(img1_show,Point2f(u1_match,v1),0,Scalar(0));
        double idepth0=getRoughIdepth(u,v,u1_match,v1,K0,K1,T01,lvl_factor);
        

		pt0.idepth = idepth0;
		pt0.is_good = true;

		pt1.u = u1_match;
		pt1.v = v1;
		pt1.is_good = true;

        if(idepth0<0.1||idepth0>1)
        {
            pt0.is_good = false;
            pt1.is_good = false;
        }
		//double x0=(u*lvl_factor-K0(0,2))/K0(0,0);
        //double y0=(v*lvl_factor-K0(1,2))/K0(1,1);
        //points_buf.push_back(Rw0*Vector3d(x0,y0,1)/idepth0);

	    //depth_lvl.at<float>(v,u)=1/(idepth0/idepth0_min)*40-0.2;
		}
		return true;
}