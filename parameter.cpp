#include "parameter.h"
Matrix3d Parameter::K0;
Matrix3d Parameter::K1;
Matrix3d Parameter::R01;
Vector3d Parameter::t01;
void Parameter::LoadParameter()
{
	double fx0 = 458.654;
	double fy0 = 457.296;
	double cx0 = 367.215;
	double cy0 = 248.375;

	double fx1 = 457.586;
	double fy1 = 456.134;
	double cx1 = 379.999;
	double cy1 = 255.238;
	K0 = Matrix3d();
	K0 << fx0, 0, cx0,
		0, fy0, cy0,
		0, 0, 1;


	K1 = Matrix3d();
	K1 << fx1, 0, cx1,
		0, fy1, cy1,
		0, 0, 1;
	Matrix4d T01;
	T01 << 0.999997256477881, 0.002312067192424, 0.000376008102415, -0.110073808127187,
		-0.002317135723281, 0.999898048506644, 0.014089835846648, 0.000399121547014,
		-0.000343393120525, -0.014090668452714, 0.999900662637729, -0.000853702503357,
		0, 0, 0, 1;

	R01 = T01.block<3, 3>(0, 0);
	t01 = T01.block<3, 1>(0, 3);


}

