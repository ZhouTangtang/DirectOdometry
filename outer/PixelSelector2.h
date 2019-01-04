/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once
 
#include <Eigen/Core>
#include <Frame.h>

enum PixelSelectorStatus {PIXSEL_VOID=0, PIXSEL_1, PIXSEL_2, PIXSEL_3};


//struct FrameHessian;
class FrameDatum;
class PixelSelector
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	int makeMaps(
			const FrameDatum* const fh,
			float* map_out, float density, int recursionsLeft=1, bool plot=false, float thFactor=1);

	PixelSelector(int w, int h);
	~PixelSelector();
	int currentPotential;


	bool allowFast;
	void makeHists(const FrameDatum* const fh);
private:

	Eigen::Vector3i select(const FrameDatum* const fh,
			float* map_out, int pot, float thFactor=1);


	unsigned char* randomPattern;


	int* gradHist;
	float* ths;
	float* thsSmoothed;
	int thsStep;
	const FrameDatum* gradHistFrame;
};

