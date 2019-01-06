#include <string>
#include "Frame.h"
#include "Tracker.h"
using namespace std;

string img_stamp_path="/home/zhouyuxuan/data/mav0/cam0/data_2.csv";
string img0_path="/home/zhouyuxuan/data/mav0/cam0/data";
string img1_path="/home/zhouyuxuan/data/mav0/cam1/data";

vector<pair<double, string>> getImageStamps();
vector<string> split(const string &s, const string &seperator);

int main()
{
	//cv::Mat img0_src = imread("/home/zhouyuxuan/data/mav0/cam0/data/1403715273262142976.png");
	//cout << img0_src.channels() << endl;
	double fx0 = 458.654;
	double fy0 = 457.296;
	double cx0 = 367.215;
	double cy0 = 248.375;

	double k10 = -0.28340811;
	double k20 = 0.07395907;
	double k30 = 0.00019359;
	double k40 = 1.76187114e-05;

	double fx1 = 457.586;
	double fy1 = 456.134;
	double cx1 = 379.999;
	double cy1 = 255.238;

	double k11 = -0.28368365;
	double k21 = 0.07451284;
	double k31 = -0.00010473;
	double k41 = -3.55590700e-05;
	
	cv::Mat K0;	K0 = (cv::Mat_<double>(3, 3) << fx0, 0, cx0,
		0, fy0, cy0,
		0, 0, 1);
	cv::Mat distortion_coeffs0;
	distortion_coeffs0 = (cv::Mat_<double>(4, 1) << k10, k20, k30, k40);

	cv::Mat K1;
	K1 = (cv::Mat_<double>(3, 3) << fx1, 0, cx1,
		0, fy1, cy1,
		0, 0, 1);
	cv::Mat distortion_coeffs1;
	distortion_coeffs1 = (cv::Mat_<double>(4, 1) << k11, k21, k31, k41);

	vector<pair<double, string>> img_stamps = getImageStamps();
	Tracker tracker;

	for (int i = 0; i < img_stamps.size();i++)
	{
		cout<<img_stamps[i].first<<endl;
		cout<<img_stamps[i].second<<endl;
		cv::Mat img0_src = imread(string(img0_path + string("/") + string(img_stamps[i].second)));
		cvtColor(img0_src, img0_src, CV_RGB2GRAY);
		cv::Mat img0_undist;

		cv::undistort(img0_src, img0_undist, K0, distortion_coeffs0);

		cout << img1_path + "/" + img_stamps[i].second << endl;
		cv::Mat img1_src = imread(img1_path + "/" + img_stamps[i].second);
		cvtColor(img1_src, img1_src, CV_RGB2GRAY);
		cv::Mat img1_undist;

		cv::undistort(img1_src, img1_undist, K1, distortion_coeffs1);
		tracker.AddNewStereo(img0_undist, img1_undist,img_stamps[i].first);
	}

	return 0;


}

vector<pair<double, string>> getImageStamps()
{
	ifstream ifs(img_stamp_path);
	string line;
	vector<pair<double,string>> imagestamps;
	while (ifs.good())
	{
		getline(ifs, line);
		if (line[0] == '%' || line[0] == '#') continue;
		if((int)line.back()==13)
			line=line.substr(0,line.size()-1);
		vector<string> buffer = split(line, ",");
		
		if (buffer.size() < 2) continue;
		imagestamps.push_back(make_pair(stod(buffer[0])/1e9, buffer[1]));
		// cout << buffer[0] << " " << buffer[1] << endl;
	}
	ifs.close();
	cout << "image_stamps: " << imagestamps.size() << endl;
	return imagestamps;
}


vector<string> split(const string &s, const string &seperator) {
	vector<string> result;
	typedef string::size_type string_size;
	string_size i = 0;

	while (i != s.size()) {
		int flag = 0;
		while (i != s.size() && flag == 0) {
			flag = 1;
			for (string_size x = 0; x < seperator.size(); ++x)
				if (s[i] == seperator[x]) {
					++i;
					flag = 0;
					break;
				}
		}

		flag = 0;
		string_size j = i;
		while (j != s.size() && flag == 0) {
			for (string_size x = 0; x < seperator.size(); ++x)
				if (s[j] == seperator[x]) {
					flag = 1;
					break;
				}
			if (flag == 0)
				++j;
		}
		if (i != j) {
			result.push_back(s.substr(i, j - i));
			i = j;
		}
	}
	return result;
}
