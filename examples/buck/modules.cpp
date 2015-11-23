// modules.h:
// Copyright (C) 2005-2016 Giorgio Biagetti and Simone Orcioni
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
 51 Franklin Street, Fifth Floor, Boston, MA 02110-1601 USA.
 */



#define SC_INCLUDE_FX
#include <systemc.h>

#include "modules.h"

adder::adder (sc_core::sc_module_name instname)
{
	SC_METHOD(doing);
	sensitive << in_a << in_b;
}

void adder::doing ()
{
	out.write(in_a.read()+in_b.read());
}


amp::amp (sc_core::sc_module_name instname, sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> coeff) : coeff(coeff)
{
	SC_METHOD(doing);
	sensitive << in;
}

void amp::doing ()
{
	out.write(in.read()*coeff);
	
}


comp::comp (sc_core::sc_module_name instname)
{
	SC_METHOD(doing);
	sensitive << input;
}
void comp::doing ()
{
	if (input.read() <= 1) {
		output.write(false);
		output_z.write(true);
		output_fx.write(0.0);
		
	}
	else {
		output.write(true);
		output_z.write(false);
		output_fx.write(2.0);
	}
}


delayff::delayff (sc_core::sc_module_name instname)
{
	SC_METHOD(doing);
	sensitive << clk.pos();
}

void delayff::doing ()
{
	out.write(in.read());
}

diff::diff (sc_core::sc_module_name name)
{
	SC_METHOD(doing);
	sensitive << a << b;
}

void diff::doing()
{
	out.write(a.read()-b.read());
}


inverter::inverter (sc_core::sc_module_name name)
{
	SC_METHOD(doing);
	sensitive << in;
}

void inverter::doing ()
{
	out->write(!(in->read()));
}

quantizer::quantizer (sc_core::sc_module_name name)
{
	SC_METHOD(doing);
	sensitive << clk.pos()/*input*/;
}

void quantizer::doing()
{
	// the desired output voltage
	const double range=5e-3;
	//	const double range=0.0945;
	if ((input.read()) < (-1*range)) {
		out.write((sc_fixed_fast<wl,iwl,SC_RND, SC_SAT>)-1);
	}
	else {	if ((input.read()) > (range)) {
		out.write((sc_fixed_fast<wl,iwl,SC_RND, SC_SAT>)+1);
	}
	else {
		out.write((sc_fixed_fast<wl,iwl,SC_RND, SC_SAT>)0);
	}
	}
}



subtractor::subtractor (sc_core::sc_module_name instname)
{
	SC_METHOD(doing);
	sensitive << in_plus << in_minus;
}

void subtractor::doing ()
{
	out.write(in_plus.read()-in_minus.read());
}
