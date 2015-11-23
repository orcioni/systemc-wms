// electromechanical.cpp:
// Copyright (C) 2004-2013 Giorgio Biagetti and Simone Orcioni
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

#include "../../include/nature/threephase"
#include "../../include/nature/rotational"
#include "../../include/units/constants"
#include "../../include/devices/electromechanical.h"

//  implementation of class motor

induction_motor::induction_motor (sc_core::sc_module_name name,double rs_in, double rr_in, double Lleaks_in, double Lleakr_in, double Lmag_in, double P_in, double J_in, double B_in) : analog_module(5, 50e-6, 50e-6),
supply(port<threephase>(1)), load(port<rotational>(2)), rs(rs_in), rr(rr_in),
Lleaks(Lleaks_in),	Lleakr(Lleakr_in), Lmag(Lmag_in),
P(P_in),       // # of poles
J(J_in), // kgm^2
B(B_in)  // Nms/rad

{
	SC_THREAD(calculus); sensitive << activation;
	//set normalization resistances for typed ports:
	supply <<= 1.0;
	load   <<= 1.0;
	// initial values of output variables (unused):
	Te =  0;
	accel = 0;
}

void induction_motor::ics(double speed)
{
	double ICS[5] = {0.0, 0.0, 0.0, 0.0, speed};
	this->ic(ICS);
}

void induction_motor::field (double *var) const
{
	static const std::complex <double> j(0, 1);
	static const double re = supply->get_normalization();
	static const double rm = load->get_normalization();
	static const double rmroot = sqrt(rm);
	static const double	ls = Lleaks + 1.5*Lmag;
	static const double	lr = Lleakr + 1.5*Lmag;
	static const double lm = 1.5*Lmag;
	static const double lx = ls * lr - lm * lm;
	
	const std::complex <double> is(state[0], state[1]);
	const std::complex <double> ir(state[2], state[3]);
	const std::complex <double> vs = 2.0 * supply->read() - is;
	const double omega = state[4];
	const double torque = 2.0 * load->read() - omega;
	std::complex <double> f1 = re * vs - rs * is;
	std::complex <double> f2 = (lm * is + lr * ir) * omega * P / 2.0 * j * rmroot - rr * ir;
	std::complex <double> v1 = lr / lx * f1 - lm / lx * f2;
	std::complex <double> v2 = ls / lx * f2 - lm / lx * f1;
	
	Te =  P / (2 / rmroot) * lm / re * (is * std::conj(ir)).imag();
	accel =  Te / rm / J - (B * omega - torque / rm) / J;
	
	var[0] = v1.real();
	var[1] = v1.imag();
	var[2] = v2.real();
	var[3] = v2.imag();
	var[4] = accel;
}

void induction_motor::calculus ()
{
	while (step()) {
		supply->write(supply->read() - std::complex<double>(state[0], state[1]));
		load->write(state[4] - load->read());
	}
}



