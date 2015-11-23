// transistor.h:
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

#ifndef TRANSISTOR_H
#define TRANSISTOR_H

#ifndef WAVE_SYSTEM_H
//TODO: why including the following multiple times does not work?
#include "wave_system"
#include "analog_system"
#include "nature/electrical"
#endif

/*	Simple level-1 MOS model:
 *	Gate control is assumed to be digital.
 *	If gate is on, then:
 *	 iD = K/2 * (VGS - VTn)²;               0 < VGS - VTn < vDS
 *	 iD =  K  * (VGS - VTn - vDS/2) * vDS;  0 < vDS < VGS - VTn
 *	Reverse conduction (vDS < 0) is modeled as a short!
 *	Definitions:
 *	 Vsat = VGS - VTn;   [saturation voltage]
 *	 Isat = K/2 * Vsat²; [saturation current]
 *	Given Ron, it results Vsat = 2 * Ron * Isat.
 */

struct mosfet
{
	mosfet (double Vsat, double Isat, int n);
	double current (double a, bool gate) const;
	const double R, alpha, beta, gamma;
};

struct mosfet_1p : mosfet, wave_module <1, electrical>
{       
	SC_HAS_PROCESS(mosfet_1p);
	mosfet_1p (sc_core::sc_module_name name, double Vsat, double Isat);
	sc_core::sc_in <bool> control;
private:
	void calculus ();
};

struct mosfet_2p : mosfet, wave_module <2, electrical>
{       
	SC_HAS_PROCESS(mosfet_2p);
	mosfet_2p (sc_core::sc_module_name name, double Vsat, double Isat);
	sc_core::sc_in <bool> control;
private:
	void calculus ();
};


/*	Simple exponential diode model:
 *	 i = Is * (exp(v / nVT) - 1)
 *	Breakdown is not modeled!
 *	Definitions:
 *	 Is = If / (exp(Vf / nVT) - 1)
 *	(i.e., the i(v) curve passes through the Vf,If point).
 */

struct diode
{
	diode (double Is, double nVT);
	double current (double a) const;
	const double R, alpha, beta, gamma;
};

struct diode_1p : diode, wave_module <1, electrical>
{       
	SC_HAS_PROCESS(diode_1p);
	diode_1p (sc_core::sc_module_name name, double Vf, double If, double nVT = 0.025);
private:
	void calculus ();
};

#endif
