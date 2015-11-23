// MyHB.cpp:
// Copyright (C) 2004 Giorgio Biagetti and Simone Orcioni
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

#include <systemc.h>
#include "sys/sources"
//#include "sys/nonlinear"
#include "devices/electrical_oneport.h"
#include "devices/electrical_twoport.h"
#include "nature/electrical"
#include "units/electrical"
#include "units/constants"

#include "tab_trace"


struct controller : sc_core::sc_module
{
	SC_HAS_PROCESS(controller);
	controller (sc_core::sc_module_name name, double frequency);
	sc_core::sc_out <bool> output1, output2;
private:
	void calculus ();
	double frequency;
};

controller::controller (sc_core::sc_module_name name, double frequency) : frequency(frequency)
{
	SC_THREAD(calculus);
}

void controller::calculus ()
{
	double T = 1.0 / frequency;
	double Ton = 0.48 * T;
	double Tdelay = 0.5 * T - Ton;
	for(bool x = true; true; x = !x) {
		output1->write(x);
		output2->write(!x);
		wait(Ton, sc_core::SC_SEC);
		output1->write(false);
		output2->write(false);
		wait(Tdelay, sc_core::SC_SEC);
	}
}


struct HalfBridge : sc_core::sc_module
{
	ab_port <electrical> power, load;
	controller ctrl;
	onoff_switchd_2s switch1, switch2;
	RCs_load snubber;
	sc_core::sc_signal <bool> pulse1, pulse2;
	ab_signal <electrical, series> shunt;
	ab_signal <electrical, parallel> chopped;
	ab_connector <electrical> connector;

	HalfBridge (sc_core::sc_module_name name) :
		ctrl("CTRL1", 20 kHz), switch1("SWITCH1"), switch2("SWITCH2"), snubber("SNUBBER", 10 ohm, 10 nF), connector("OUTPUT")
	{
		ctrl(pulse1, pulse2);
		switch1(chopped, power, pulse1);
		switch2(shunt, chopped, pulse2);
		snubber(chopped);
		snubber.port <<= 10.0 ohm;
		connector(load, chopped);
	}

};

// main program:

int sc_main (int argc, char *argv[])
{
	sc_core::sc_signal <electrical::wave_type> in;
	ab_signal <electrical, parallel> mains, rectified, chopped;

	sc_core::sc_trace_file *f = create_tab_trace_file("TRACES", 10e-9);
//	f->delta_cycles(true);
	chopped.trace(f, "CHOPPED");

	generator <electrical::wave_type> signal_source("SOURCE1", sine(sqrt(2) * 230, 50 Hz, pi / 2));
	signal_source(in);

	source <electrical> wave_source("SOURCE2", cfg::across);
	wave_source(mains, in);

	diode_bridge bridge("BRIDGE");
	bridge(mains, rectified);

	RCs_load line_filter("FILTER", 1 ohm, 5 uF);
	line_filter.set_steplimits(1e-9, 1e-6);

	line_filter(rectified);

	HalfBridge hb("HB");
	hb(rectified, chopped);

	RLCs_load load("LOAD", 3 ohm, 80 uH, 740 nF);
	load.port <<= 5 ohm;
	load(chopped);

	sc_core::sc_start(150e-6, sc_core::SC_SEC);
	close_tab_trace_file(f);
	return 0;
}
