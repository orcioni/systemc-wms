// cell.h:
// Copyright (C) 2016 Simone Orcioni
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

#ifndef CELL_H
#define CELL_H

#include <tuple>
#include <vector>
#include "analog_system"
#include "nature/electrical"
#include "nature/thermal"
#define ARMA_DONT_USE_WRAPPER
#include <armadillo>
#include "interpol.h"
using namespace arma;
using namespace std;


//  Declaration of Litium-ion battery Li_cell with variable component values
//  shunt diode on R2-C2 and internal 2D interpolation

struct Li_cell : wave_module <2, electrical, thermal>, analog_module, interpol
{
	typedef wave_module<> base_class;
	SC_HAS_PROCESS(Li_cell);
	Li_cell(sc_core::sc_module_name name, const vector<string> &param_names, double soc_i, double Qn);
	Li_cell(sc_core::sc_module_name name, const vector<string> &param_names, double soc_i, double Qn, std::vector<double> rv);
	ab_port <electrical> &e_port;
	ab_port <thermal> &th_port;
	sc_core::sc_in_clk clock_port;
public:
	void ics(double VC1, double VC2);
	double soc, T, Vocv;
private:
	
	void calculus ();
	void set_params();
	void field (double *var) const;
	double Qn_, Qn, soc_i, Ri, R1, R2, C1, C2, Rth, iR;
	
	Mat<double> *Ri_out, *R1_out, *C1_out, *R2_out, *C2_out, *OCV_out;
	vector<double> *Ri_in1, *Ri_in2, *R1_in1, *R1_in2, *C1_in1, *C1_in2, *R2_in1, *R2_in2, *C2_in1, *C2_in2, *OCV_in1, *OCV_in2;
	
	vector<double> rv;
	const double RON = 1.0e-4;
};

typedef Li_cell Li_cell_itrp;

//  Declaration of Litium-ion battery Li_cell
// with fixed parameters

struct Li_cell_x : wave_module <2, electrical, thermal>, analog_module
{
	SC_HAS_PROCESS(Li_cell_x);
	Li_cell_x(sc_core::sc_module_name name, double Ri_, double R1_, double C1_, double R2_, double C2_, double Vocv_, double Rth_);
	ab_port <electrical> &e_port;
	ab_port <thermal> &th_port;
public:
	void ics(double VC1, double VC2);
private:
	void calculus ();
	void field (double *var) const;
	const double Ri, R1, R2, C1, C2, Rth, Vocv;
};

//  Declaration of Litium-ion battery Li_cell with variable component values

struct Li_cell_var : wave_module <2, electrical, thermal>, analog_module
{
	typedef wave_module<> base_class;
	SC_HAS_PROCESS(Li_cell_var);
	Li_cell_var(sc_core::sc_module_name name, double soc_i, double Qn);
	Li_cell_var(sc_core::sc_module_name name, double soc_i, double Qn, std::vector<double> rv);
	ab_port <electrical> &e_port;
	ab_port <thermal> &th_port;
	sc_core::sc_in <double> Rth_port;
	sc_core::sc_in <double> Ri_port;
	sc_core::sc_in <double> R1_port;
	sc_core::sc_in <double> C1_port;
	sc_core::sc_in <double> R2_port;
	sc_core::sc_in <double> C2_port;
	sc_core::sc_in <double> Vocv_port;
	sc_core::sc_out <double> soc;
public:
	void ics(double VC1, double VC2);
private:
	void calculus ();
	void set_params();
	void field (double *var) const;
	double Qn, soc_i, Ri, R1, R2, C1, C2, Rth, Vocv, iR;
	std::vector<double> rv;
};

//  Declaration of Litium-ion battery Li_cell with variable component values
//  and shunt diode on R2-C2

struct Li_cell_d : wave_module <2, electrical, thermal>, analog_module
{
	typedef wave_module<> base_class;
	SC_HAS_PROCESS(Li_cell_d);
	Li_cell_d(sc_core::sc_module_name name, double soc_i, double Qn);
	Li_cell_d(sc_core::sc_module_name name, double soc_i, double Qn, std::vector<double> rv);
	ab_port <electrical> &e_port;
	ab_port <thermal> &th_port;
	sc_core::sc_in <double> Rth_port;
	sc_core::sc_in <double> Ri_port;
	sc_core::sc_in <double> R1_port;
	sc_core::sc_in <double> C1_port;
	sc_core::sc_in <double> R2_port;
	sc_core::sc_in <double> C2_port;
	sc_core::sc_in <double> Vocv_port;
	sc_core::sc_out <double> soc;
public:
	void ics(double VC1, double VC2);
private:
	void calculus ();
	void set_params();
	void field (double *var) const;
	double Qn, soc_i, Ri, R1, R2, C1, C2, Rth, Vocv, iR;
	std::vector<double> rv;
	const double RON = 1.0e-4;
};

#endif //CELL_H
