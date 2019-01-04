#pragma once
#include <Eigen/Core>
using namespace Eigen;
namespace Parameter
{
	extern Matrix3d K0;
	extern Matrix3d K1;
	extern Matrix3d R01;
	extern Vector3d t01;
	void LoadParameter();
}