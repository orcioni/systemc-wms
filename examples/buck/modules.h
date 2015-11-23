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

#ifndef MODULES_H
const int wl = 16;
const int iwl = 4;

struct adder : sc_core::sc_module
{
	SC_HAS_PROCESS(adder);
	adder (sc_core::sc_module_name instname);
	sc_core::sc_in <sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > in_a, in_b;
	sc_core::sc_out <sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > out;
	
private:
	void doing();
};


struct amp : sc_core::sc_module
{
	SC_HAS_PROCESS(amp);
	amp (sc_core::sc_module_name instname, sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> coeff);
	sc_core::sc_in <sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > in;
	sc_core::sc_out <sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > out;
	sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> coeff;
	
private:
	void doing();
};

struct comp : sc_core::sc_module
{
	SC_HAS_PROCESS(comp);
	comp (sc_core::sc_module_name instname);
	
	sc_in <sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > input;
	sc_out <bool> output;
	sc_out <bool> output_z;
	sc_out <sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > output_fx;
	
private:
	void doing();
};


struct delayff : sc_core::sc_module
{
	SC_HAS_PROCESS(delayff);
	delayff (sc_core::sc_module_name instname);
	
	sc_in <sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > in;
	sc_out <sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > out;
	sc_in <bool> clk;
	
private:
	void doing();
};



struct diff : sc_core::sc_module
{
	SC_HAS_PROCESS(diff);
	diff (sc_core::sc_module_name name);
	sc_in <double> a;
	sc_in <double> b;
	sc_out <double> out;
	
private:
	void doing();
};

struct inverter : sc_core::sc_module
{
	SC_HAS_PROCESS(inverter);
	inverter (sc_core::sc_module_name name);
	sc_core::sc_in <bool> in;
	sc_core::sc_out <bool> out;
private:
	void doing ();
};

struct quantizer : sc_core::sc_module
{
	SC_HAS_PROCESS(quantizer);
	quantizer (sc_core::sc_module_name name);
	sc_in <double> input;
	sc_out<sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > out;
	sc_in <bool> clk;
	//double in;//
	
private:
	void doing();
};

struct subtractor : sc_core::sc_module
{
	SC_HAS_PROCESS(subtractor);
	subtractor (sc_core::sc_module_name instname);
	sc_core::sc_in <sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > in_plus, in_minus;
	sc_core::sc_out <sc_fixed_fast<wl,iwl,SC_RND, SC_SAT> > out;
	
private:
	void doing();
};
#endif

#define MODULES_H


