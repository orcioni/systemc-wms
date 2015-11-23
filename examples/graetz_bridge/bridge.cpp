// bridge.cpp:
// Copyright (C) 2006 Giorgio Biagetti and Simone Orcioni
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


#include "wave_system"

#include "sys/sources"
#include "devices/electrical_oneport.h"

#include "nature/electrical"
#include "units/electrical"
#include "units/constants"

#include "tab_trace"


struct graetz : sc_core::sc_module
{
	graetz (sc_core::sc_module_name name);
	ab_port <electrical> supply;
	ab_port <electrical> load;
private:
	ab_signal <electrical, bridge> channel;
	ab_connector <electrical> ac_connector, dc_connector;
	idiode diode1, diode2, diode3, diode4;
};

graetz::graetz (sc_core::sc_module_name name) : ac_connector("AC"), dc_connector("DC"), diode1("D1"), diode2("D2"), diode3("D3"), diode4("D4")
{
	supply <<= 1 ohm;
	load   <<= 500 ohm;
	diode1(+channel->nw);
	diode2(-channel->ws);
	diode3(-channel->ne);
	diode4(+channel->es);
	ac_connector(channel->ns, supply);
	dc_connector(channel->we, load);
}


// main program:
int sc_main (int argc, char *argv[])
{
	sc_core::sc_signal <double> angle;
	ab_signal <electrical, parallel> mains(10 ohm), rectified(500 ohm);

	sc_core::sc_trace_file *f = create_tab_trace_file("TRACES");
	mains.trace(f, "MAINS");
	rectified.trace(f, "OUTPUT");

	generator <double> signal_source("SOURCE1", sine(sqrt(2) * 230, 50 Hz, 0));
	signal_source(angle);

	source <electrical> wave_source("GENERATOR", cfg::across);
	wave_source.input(angle);
	wave_source.port(mains);

	graetz rectifier("RECTIFIER");
	rectifier(mains, rectified);

//	rectifier.channel.trace(f, "BRIDGE");

	R_load load("LOAD", 1000 ohm);
	load(rectified);

	sc_core::sc_start(sc_core::sc_time(0.2, sc_core::SC_SEC));

	close_tab_trace_file(f);
	return 0;
}
