// devices.cpp:
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

// implementation of class LsCp_ladder
/*
LsCp_ladder::LsCp_ladder(sc_core::sc_module_name name, double s_ind, double p_cap):analog_module(2,sqrt(p_cap*s_ind)/1000,sqrt(p_cap*s_ind)/10),C(p_cap),L(s_ind)
{
	SC_THREAD(calculus);
}


void LsCp_ladder::field (double *var) const
{
	const double sqrt_R1 = port[0]->get_normalization_sqrt();
	const double R1 = port[0]->get_normalization();
	const double sqrt_R2 = port[1]->get_normalization_sqrt();
	const double R2 = port[1]->get_normalization();

	double a1 = port[0]->read();double a2 = port[1]->read();
	var[0] = state[1]/L+(2*a2)/sqrt_R2-state[0]/(R2*C);
	var[1] =2*a1*sqrt_R1-(R1/L)*state[1]-(state[0])/C;
}


void LsCp_ladder::calculus ()
{
	const double sqrt_R1 = port[0]->get_normalization_sqrt();
	const double sqrt_R2 = port[1]->get_normalization_sqrt();

	state[0]=0;
	state[1]=0;
	while (step()) {
		double a1 = port[0]->read(); double a2 = port[1]->read();
		double b1=a1-(sqrt_R1/L)*state[1];
		double b2 =-a2+state[0]/(sqrt_R2*C);
		port[0]->write(b1);
		port[1]->write(b2);
	}	
}

*/


