// transistor.cpp:
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

#include "sys/sources"
#include "devices/electrical_oneport.h"

#include "tab_trace"

#include <cmath>
 
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

/*	Implementation of the MOS model:
 *	(single analog port (n = 1), hence i = iD, v = vDS)
 *	Let
 *	 w = v / Vsat,
 *	 y = i / Isat,
 *	the model becomes:
 *	 y = 0;              (when v < 0 < Vsat, i.e., w < 0 < 1)
 *	 y = 2 * w - w * w;  (when 0 < v < Vsat, i.e., 0 < w < 1)
 *	 y = 1;              (when 0 < Vsat < v, i.e., 0 < 1 < w)
 *	Using the a,b parameters:
 *	For a current source,
 *	 v = 2 * a * sqrt(R) - R * i;
 *	 b = a - i * sqrt(R);
 *	hence:
 *	 w = a * alpha - y * beta;
 *	 b = a - y * gamma;
 *	with the constants alpha, beta, gamma, defined as below.
 *	In triode we must solve the quadratic system of equations:
 *	 w = a * alpha - y * beta;  y = 2 * w - w * w;
 *
 *	(double analog port (n = 2), hence i1 = -i2 = iD, v = v1 - v2)
 *	almost all as before, but now
 *	 v = 2 * (a1 - a2) * sqrt(R) - 2 * R * i;
 *	 b1 = a1 - i * sqrt(R);
 *	 b2 = a2 + i * sqrt(R);
 *	so that the same code can be used, with
 *	 a <- a1 - a2
 *	 beta <- 2 * beta
 */

struct mosfet
{
	mosfet (double Vsat, double Isat, int n);
	double current (double a, bool gate) const;
	const double R, alpha, beta, gamma;
};

mosfet::mosfet (double Vsat, double Isat, int n) :
	// normalization resistance R is chosen so as to make beta=1 for better efficiency
	R(Vsat / Isat),
	alpha(2 * sqrt(R) / Vsat),
	beta(R * Isat / Vsat * n),
	gamma(sqrt(R) * Isat)
{
}

double mosfet::current (double a, bool gate) const
{
	double x = a * alpha; // x = w @ (i = 0);
	double y = 0;
	if (gate && x - beta >= 1) {
		// transistor in saturation:
		y = 1;
	} else if (gate && x >= 0) {
		// transistor in triode zone:
		double c2 = beta * beta;
		double c1 = 0.5 + beta * (1 - x);
		double c0 = x * x - 2 * x;
		y = (sqrt(c1 * c1 - c0 * c2) - c1) / c2;
	} else if (x < 0) {
		// reversed polarity:
		y = x / beta;
	}
	return y * gamma;
}

/*	Simple exponential diode model:
 *	 i = Is * (exp(v / nVT) - 1)
 *	Letting V0 be the voltage across the series of the diode and a resistor R, from [1]:
 *	 i = Is * (nVT / (R * Is) * W0(R * Is / nVT * exp((V0 + R * Is) / nVT)) - 1)
 *	where W0 is the branch 0 of the Lambert W-function.
 *	With
 *	 alpha = 2 * sqrt(R) / nVT
 *	 beta = R * Is / nVT
 *	 gamma = sqrt(R) * Is;
 *	we have
 *	 V0 = a * alpha * nVT;
 *	 i = Is * (W0(beta * exp(a * alpha + beta)) / beta - 1);
 *	 b = a - (i / Is) * gamma;
 *	References:
 *	[1] T.C. Banwell and A. Jayakumar, "Exact analytical solution for current flow through diode with series resistance", ELECTRONICS LETTERS, vol. 36, no. 4, pp. 291-292, 17th February 2000. DOI: 10.1049/el:20000301
 *	[2] Rob Johnson, http://www.whim.org/nebula/math/lambertw.html
 */

struct diode
{
	diode (double Is, double nVT);
	double current (double a) const;
	const double R, alpha, beta, gamma;
};

diode::diode (double Is, double nVT) :
	R(1),
	alpha(2 * sqrt(R) / nVT),
	beta(R * Is / nVT),
	gamma(sqrt(R) * Is)
{
}

double diode::current (double a) const
{
	if (a * alpha > 400) {
		// open-circuit voltage grater than 200*nVT (about 5 V): exponential model will saturate!
		return a + a - 400 / alpha;
	}
	double x = beta * exp(a * alpha + beta);
	// compute Lambert-W function W0(x),
	// five Newton-Raphson iterations usually suffice:
	double w = x > 2 ? log(x) - log(log(x)) : 0.5;
	for (int n = 0; n < 5; ++n)
		w = (x * exp(-w) + w * w) / (w + 1);
	return gamma * (w / beta - 1);
}



// Single-port module implementation:

struct mosfet_1p : mosfet, wave_module <1, electrical>
{       
	SC_HAS_PROCESS(mosfet_1p);
	mosfet_1p (sc_core::sc_module_name name, double Vsat, double Isat);
	sc_core::sc_in <bool> control;
private:
	void calculus ();
};

mosfet_1p::mosfet_1p (sc_core::sc_module_name name, double Vsat, double Isat) : 
	mosfet(Vsat, Isat, 1)
{
	SC_METHOD(calculus);
	sensitive << activation << control;
	port <<= R;
}

void mosfet_1p::calculus ()
{
	bool gate = control->read();
	double a = port->read();
	double i = current(a, gate);
	port->write(a - i);
}


struct diode_1p : diode, wave_module <1, electrical>
{       
	SC_HAS_PROCESS(diode_1p);
	diode_1p (sc_core::sc_module_name name, double Vf, double If, double nVT = 0.025);
private:
	void calculus ();
};

diode_1p::diode_1p (sc_core::sc_module_name name, double Vf, double If, double nVT) : 
	diode(If / (exp(Vf / nVT) - 1), nVT)
{
	SC_METHOD(calculus);
	sensitive << activation;
	port <<= R;
}

void diode_1p::calculus ()
{
	double a = port->read();
	double i = current(a);
	port->write(a - i);
}


// Two-port module implementation:

struct mosfet_2p : mosfet, wave_module <2, electrical>
{       
	SC_HAS_PROCESS(mosfet_2p);
	mosfet_2p (sc_core::sc_module_name name, double Vsat, double Isat);
	sc_core::sc_in <bool> control;
private:
	void calculus ();
};

mosfet_2p::mosfet_2p (sc_core::sc_module_name name, double Vsat, double Isat) : 
	mosfet(Vsat, Isat, 2)
{
	SC_METHOD(calculus);
	sensitive << activation << control;
	port[0] <<= R;
	port[1] <<= R;
}

void mosfet_2p::calculus ()
{
	bool gate = control->read();
	double a1 = port[0]->read();
	double a2 = port[1]->read();
	double i = current(a1 - a2, gate);
	port[0]->write(a1 - i);
	port[1]->write(a2 + i);
}

// main program:
int sc_main (int argc, char *argv[])
{
	sc_core::sc_set_time_resolution(1.0, sc_core::SC_NS);
	sc_core::sc_signal <double> angle;
	ab_signal <electrical, parallel> mains;
	ab_signal <electrical, series> output;

	sc_core::sc_clock pwm("PWM", sc_core::sc_time(0.1, sc_core::SC_MS));

	sc_core::sc_trace_file *f = create_tab_trace_file("TRACES", 5e-6);
	mains.trace(f, "MAINS");
	output.trace(f, "OUTPUT");

	generator <double> signal_source("SOURCE1", sine(10, 100, 0, 1024));
	signal_source(angle);
	source <electrical> supply("GENERATOR1", cfg::across);
	supply.input(angle);
	supply.port(mains);

	mosfet_2p m("M1", 0.75, 2.5);
	m(mains, output, pwm);
	m.port[0] <<= 2 ohm;
	m.port[1] <<= 2 ohm;

	R_load r("R1", 2);
	r.port <<= 2 ohm;
	r(output);
	
	diode_1p d("D1", 0.8, 2);
	d.port <<= 2 ohm;
	d(-output);

	sc_core::sc_start(sc_core::sc_time(10e-3, sc_core::SC_SEC));

	close_tab_trace_file(f);
	return 0;
}
