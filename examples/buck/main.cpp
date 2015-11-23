// main.cpp:
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
#include "devices/electrical_oneport.h"
#include "devices/electrical_twoport.h"
#include "sys/sources"
#include "units/electrical"
#include "units/constants"

#include "tab_trace"

//sc_core::sc_trace_file *f3 = sc_create_vcd_trace_file("DIGITAL");

#include "transistor.h"

#include "modules.h"
#include "sigmadelta.cpp"
#include "adc.cpp"
#include "compensator.cpp"


//main

int sc_main (int argc, char *argv[])
{
 sc_report_handler::set_actions("/IEEE_Std_1666/deprecated", SC_DO_NOTHING);

	
	double sim_time;
	double sim_voltage_in;
	double sim_vdd;
	double sim_delay_imp;
	double sim_duration_imp;
	double sim_value_imp;
	
	if (argc == 4) {
		sscanf(argv[1], "%lf", &sim_time);
		sscanf(argv[2], "%lf", &sim_voltage_in);
		sscanf(argv[3], "%lf", &sim_vdd);
		sim_delay_imp = 0.0;
		sim_duration_imp = 0.0;
		sim_value_imp = 0.0;
	} else if (argc == 7) {
		sscanf(argv[1], "%lf", &sim_time);
		sscanf(argv[2], "%lf", &sim_voltage_in);
		sscanf(argv[3], "%lf", &sim_delay_imp);
		sscanf(argv[4], "%lf", &sim_duration_imp);
		sscanf(argv[5], "%lf", &sim_value_imp);
	} else {
		std::cout << "Usage: " << argv[0] << " <sim_time> <buck voltage in> <buck voltage out> [<impulse_delay> <impulse_duration> <impulse_amplitude>]\n\n";
		std::cout << std::endl;
		return 1;
	}
	
	sc_core::sc_signal <double> in,vref;
	ab_signal <electrical, parallel> mains(1 ohm, 1e-8, 0), line1(1 ohm, 1e-8, 0), out(1 ohm, 1e-8, 0);
	sc_core::sc_signal <sc_fixed_fast<16,4,SC_RND, SC_SAT> > s1, s2;
	sc_core::sc_signal <bool> contr, contrz;

	const sc_time t_PERIOD1 (1.0, SC_US);
	const sc_time t_PERIOD2 (1.0/8.0, SC_US);

	sc_core::sc_clock clk1("clk1", t_PERIOD1);
	sc_core::sc_clock clk2("clk2", t_PERIOD2);

	sc_core::sc_trace_file *f1 = create_tab_trace_file("LOAD");
	sc_core::sc_trace_file *f2 = create_tab_trace_file("LINE1");

	out.trace(f1, "OUT");
	line1.trace(f2, "LINE1");

	

	generator <electrical::wave_type> dc_supply("DC", step(-0.5,2e-3) + dc (sim_voltage_in));
	dc_supply(in);

	source <electrical> wave_supply("SUPPLY", cfg::across);
	wave_supply(mains, in);


	mosfet_2p switch1("SWITCH", 0.75, 2.5); // NTR4170N @ VGS=1.8V
	switch1(mains, line1, contr);
	
    mosfet_1p switch2("SWITCH2", 0.75, 2.5);
	switch2(-line1, contrz);


	LsCp_ladder filter("FILTER", 10 uH, 10 uF);
	filter.port[0] <<= 2.0;
	filter.port[1] <<=2.0;
	filter.set_steplimits(10e-9,100e-9);
 	filter(line1,out);
    
	R_load load("LOAD", 2 ohm);
	load(out);

	generator <electrical::wave_type> vrefgen("VREF", step(-0.4,1.25e-3) + dc (sim_vdd));
	vrefgen(vref);

	wADC adc1("ADC1");
	adc1.wave_input(out);
	adc1.vref(vref);
    adc1.clk_in(clk1);
	adc1.out(s1);

	
	compensator comp("COMP");
	comp(s1, clk1, s2);
		
	sigmadelta3rd sd("SD");
	sd(s2, clk2, contr, contrz);

	sc_core::sc_start(sim_time, sc_core::SC_SEC);
	close_tab_trace_file(f1);
	close_tab_trace_file(f2);
//		sc_close_vcd_trace_file(f3);

	return 0;
}