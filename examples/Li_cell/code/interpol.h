// interpol.h:
// Copyright (C) 2018 Simone Orcioni
/*
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#ifndef INTERPOL_H
#define INTERPOL_H

#include <tuple>
#include <vector>
#define ARMA_DONT_USE_WRAPPER
#include <armadillo>
using namespace arma;
using namespace std;


//  Declaration of class interpol for 2D interpolation of data coming from file.
// The file structure should be a csv file of points without order, like:
// X1,Y1,Z1
// X2,Y2,Z2
// ......

class interpol
{
public:
	struct Record2D
	{
		double in1;
		double in2;
		double out;
	};
	tuple<vector<double>*,vector<double>*, Mat<double>* >  load2DRecords(const std::string &cvs_fileName);
	double interpol2D(double input1, double input2, vector<double> *vin1, vector<double> *vin2, Mat<double> *vout);
	void order2DRecords(std::vector<Record2D>* records);

	virtual ~interpol() {}
};

#endif //INTERPOL_H
