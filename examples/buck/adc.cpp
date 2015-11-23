// adc.cpp:
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

#define SC_INCLUDE_FX		// 	Need this if using fixed-point
#include <systemc.h>

#include "nature/electrical"
#include "sys/probes"

/*   ADC module contains a voltage probe to read the voltage at the port, 
     which returns a double. We use the C++ powerful level of abstraction 
     to encapsule inside the quantizer module the subtraction of the voltage
     reference, put here at 5 V. A pure systemC signal is binding the probe
     output to a pure systemC quantizer.
*/


// let's build up a 3 level 1.5bit quantizer

//a hierarchical ADC module
struct wADC: sc_core::sc_module
{
	ab_port <electrical> wave_input;
	sc_core::sc_in <double> vref;
	sc_core::sc_in <bool> clk_in;
	sc_core::sc_out <sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > out;

	sc_core::sc_signal <double> prbout,diffout;
  	iprobe <electrical> prb;	
	quantizer qtz;
	diff adiff;

	wADC (sc_core::sc_module_name name) :
	  prb("PRB", cfg::across),adiff("ADIFF"),
	  qtz("QTZ")

	{
		prb(wave_input, prbout);
		adiff(vref,prbout,diffout);
		qtz(diffout,out,clk_in);
	}
};
