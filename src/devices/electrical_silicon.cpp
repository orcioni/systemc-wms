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

#include "../../include/devices/electrical_silicon.h"
#include <cmath>


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
		// ideal diode
//		y = x / beta;
		
		// OFF
		y = 0;
	}
	return y * gamma;
}


/*	Implementation of the diode model:
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
		// open-circuit voltage grater than 200*nVT (about 5 V):
		// exponential model will saturate!
		// Just pretend to be a fixed voltage source:
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


// SystemC module impementation:

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
