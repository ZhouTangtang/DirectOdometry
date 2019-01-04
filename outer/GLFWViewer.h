#ifndef GLFWVIEWER_H
#define GLFWVIEWER_H

#include <eigen3/Eigen/Core>
#include <GLFW/glfw3.h>
#include <vector>
#include <utility>
#include <thread>
#include <chrono>
#include <mutex>
#include <math.h>
#include <map>
//#include <Windows.h>

using namespace std;
using namespace Eigen;
typedef vector<pair<Matrix3d, Vector3d>> Frames;
typedef vector<Vector3d> PointCloud;
typedef vector<Vector3d> Trajectory;
class GLFWViewer;

//Global Viewer
extern GLFWViewer viewer;

class GLFWViewer
{		
public:
	GLFWViewer();
	~GLFWViewer();
public:
	void Show();
	void ClearView();
	void SetFrames(const Frames &t);
	void SetPointCloud(const PointCloud &pc);
	void SetFrames(const vector<Frames> &vt);
	void SetPointCloud(const vector<PointCloud> &vpc);
	void SetTrajectory(const Trajectory &t);
	void SetTrajectory(const vector<Trajectory> &vt);
	void Hide();
private:
	void Run();
	
private:
	static vector<GLFWViewer*> vptr;
	GLFWwindow* window;
	thread *t;
	mutex m_mutex;
	vector<Trajectory> mv_trajectory;
	vector<PointCloud> mv_pointCloud;
	vector<Frames> mv_frames;
	

};


#endif;

