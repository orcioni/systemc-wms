// electromechanical devices:
// Copyright (C) 2013 Giorgio Biagetti and Simone Orcioni
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

#ifndef ELECTROMECHANICAL_H
#define ELECTROMECHANICAL_H

#include "../analog_system"
#include "../nature/threephase"
#include "../nature/rotational"

//
// Library of electromechanical devices


// Induction motor model
// motor (sc_core::sc_module_name name, double rs, double rr, double Lleaks, double Lleakr, double Lmag, double P, double J, double B)



struct induction_motor : wave_module <2, threephase, rotational>, analog_module
{
	SC_HAS_PROCESS(induction_motor);
	induction_motor (sc_core::sc_module_name name, double rs=14.6, double rr=12.76, double Lleaks=0.02220211456132, double Lleakr=0.05180493397641, double Lmag=0.29629345238941, double P=4, double J=0.001, double B=0.000124);
// Default parameters are from Bodine Electric Company model 295
	ab_port <threephase> &supply;
	ab_port <rotational> &load;
private:
	void field (double *var) const;
	void calculus ();
private: // motor parameters:
	const double rs, rr, Lleaks, Lleakr, P, J, B, Lmag;
public: // internal variables used for tracing purposes only and initial conditions
	mutable double accel, Te;
	void ics(double speed);

};

#endif //ELECTROMECHANICAL_H
