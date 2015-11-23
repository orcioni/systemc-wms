// motor.cpp:
// Copyright (C) 2005-2006 Giorgio Biagetti and Simone Orcioni
// Copyright (C) 2009-2013 Giorgio Biagetti, Simone Orcioni and Marco Giammarini
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
#include <complex>
#include <iostream>
#include <string>

#include "sys/sources"
#include "devices/electrical_oneport.h"
#include "devices/electrical_twoport.h"
#include "devices/electromechanical.h"
#include "devices/threephase.h"

#include "tab_trace"

/*
#include "wave_system"
#include "analog_system"
#include "sources"

#include "nature/electrical"
#include "nature/threephase"
#include "nature/rotational"
#include "units/electrical"
#include "units/constants"

#include "tab_trace"

#include "nonlinear"

#include "sys/power"
#include "sys/ideal"
*/
 
// main program:
int sc_main (int argc, char *argv[])
{
	// Command-line parameters:
	double sim_time;
	double sim_voltage;
	double sim_freq;
	double sim_time_step;
	double sim_value_step;

	if (argc == 4) {
		sscanf(argv[1], "%lf", &sim_time);
		sscanf(argv[2], "%lf", &sim_voltage);
		sscanf(argv[3], "%lf", &sim_freq);
		sim_time_step = 0.0;
		sim_value_step = 0.0;
	} else if (argc == 6) {
		sscanf(argv[1], "%lf", &sim_time);
		sscanf(argv[2], "%lf", &sim_voltage);
		sscanf(argv[3], "%lf", &sim_freq);
		sscanf(argv[4], "%lf", &sim_time_step);
		sscanf(argv[5], "%lf", &sim_value_step);
	} else {
		std::cout << "Usage: " << argv[0] << " <sim_time> <voltage> <freq> [<load_time> <load_value>]\n\n";
		std::cout << "sim_time   -> duration of simulation [s];\n";
		std::cout << "voltage    -> value of 3-phase RMS voltage [V];\n";
		std::cout << "freq       -> value of 3-phase frequency [Hz];\n";
		std::cout << "load_time  -> instant of load application [s]\n";
		std::cout << "load_value -> value of load [Nm].\n";
		std::cout << std::endl;
		return 1;
	}

	// SystemC and SystemC-WMS signals instantiation:
	sc_core::sc_set_time_resolution(1.0, sc_core::SC_NS);

	sc_core::sc_signal <std::complex <double> > angle;
	sc_core::sc_signal <double> brake;
	ab_signal <threephase, parallel> mains;
	ab_signal <rotational, parallel> shaft;

	// Output files:
	sc_core::sc_trace_file *f = create_tab_trace_file("TRACES");
	mains.trace(f, "MAINS");
	shaft.trace(f, "SHAFT");

	// Modules instantiation and connection:
	generator <std::complex <double> > signal_source("SOURCE1", sine_threephase(sqrt(2) * sim_voltage, sim_freq, 0));
	signal_source(angle);
	source <threephase> supply("GENERATOR", cfg::across);
	supply.input(angle);
	supply.port(mains);

	generator <double> brake_source("SOURCE2", step(sim_value_step, sim_time_step));
	brake_source(brake);

	source <rotational> load("BRAKE", cfg::through);
	load.input(brake);
	load.port(shaft);

	induction_motor m("MOTOR");
	m(mains, shaft);
	sc_core::sc_trace(f, m.Te, "electric_torque");
//	sc_core::sc_trace(f, m.accel, "acceleration");

	sc_core::sc_start(sc_core::sc_time(sim_time, sc_core::SC_SEC));
	
	close_tab_trace_file(f);
	return 0;
}
